#!/usr/bin/env python3
"""ROS2 adapter from any valid Qwen pixel result to low-speed Twist."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from control.qwen_visual_servo import (
    CommandRateLimiter,
    QwenVisualServo,
    RateLimitConfig,
    ServoConfig,
    ServoInput,
)


def _nested(config: dict[str, Any], *keys: str, default: Any = None) -> Any:
    value: Any = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _angle_delta(angle: np.ndarray, center: float) -> np.ndarray:
    return np.arctan2(np.sin(angle - center), np.cos(angle - center))


class QwenVisualServoNode(Node):
    def __init__(self, config_path: str, force_enable_motion: bool):
        super().__init__("qwen_visual_servo_node")
        self.config_path = str(config_path)
        self.config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
        control = self.config.get("control", {})
        freshness = self.config.get("freshness", {})
        safety = self.config.get("safety", {})
        topics = self.config.get("topics", {})
        rate_limit = self.config.get("rate_limit", {})

        servo_cfg = ServoConfig(
            max_vx=float(control.get("max_vx", 0.04)),
            max_wz=float(control.get("max_wz", 0.05)),
            kp_wz=float(control.get("kp_wz", 0.10)),
            angular_sign=float(control.get("angular_sign", -1.0)),
            center_deadband=float(control.get("center_deadband", 0.06)),
            turn_only_threshold=float(control.get("turn_only_threshold", 0.40)),
            cmd_wz_deadband=float(control.get("cmd_wz_deadband", 0.006)),
            min_confidence=float(control.get("min_confidence", 0.0)),
            point_results_before_forward=int(
                control.get(
                    "point_results_before_forward",
                    control.get("visible_results_before_forward", 1),
                )
            ),
            blocked_states=tuple(
                str(v).strip().upper()
                for v in (
                    control.get(
                        "blocked_states",
                        ["WAIT_IMAGE", "PAUSED", "SUCCESS", "ERROR"],
                    )
                    or []
                )
                if str(v).strip()
            ),
            full_speed_source_age_sec=float(
                freshness.get("full_speed_source_age_sec", 0.95)
            ),
            stop_source_age_sec=float(
                freshness.get("stop_source_age_sec", 2.0)
            ),
            max_receive_gap_sec=float(
                freshness.get("max_receive_gap_sec", 1.6)
            ),
            require_lidar=bool(safety.get("require_lidar", True)),
            scan_timeout_sec=float(safety.get("scan_timeout_sec", 0.5)),
            emergency_stop_distance=float(
                safety.get("emergency_stop_distance", 0.28)
            ),
            stop_distance=float(safety.get("stop_distance", 0.42)),
            slow_distance=float(safety.get("slow_distance", 0.65)),
            allow_turn_inside_stop_distance=bool(
                safety.get("allow_turn_inside_stop_distance", True)
            ),
        )
        self.servo = QwenVisualServo(servo_cfg)
        self.limiter = CommandRateLimiter(
            RateLimitConfig(
                max_linear_accel=float(rate_limit.get("max_linear_accel", 0.08)),
                max_linear_decel=float(rate_limit.get("max_linear_decel", 0.16)),
                max_angular_accel=float(rate_limit.get("max_angular_accel", 0.12)),
                max_angular_decel=float(rate_limit.get("max_angular_decel", 0.25)),
            )
        )

        self.motion_enabled = bool(control.get("motion_enabled", False))
        if force_enable_motion:
            self.motion_enabled = True
        self.rate_hz = float(control.get("rate_hz", 20.0))
        self.front_center_rad = math.radians(
            float(safety.get("camera_lidar_yaw_offset_deg", -170.0))
        )
        self.front_half_width_rad = 0.5 * math.radians(
            float(safety.get("front_sector_deg", 30.0))
        )
        self.front_percentile = float(safety.get("front_percentile", 10.0))

        self.state = "WAIT_IMAGE"
        self.result = ""
        self.point_x: Optional[float] = None
        self.point_y: Optional[float] = None
        self.point_role = "none"
        self.image_width = int(control.get("image_width_fallback", 960))
        self.confidence = 0.0
        self.latency_ms = 0.0
        self.result_received_sec = 0.0
        self.last_request_id = -1
        self.point_streak = 0
        self.front_distance: Optional[float] = None
        self.scan_received_sec: Optional[float] = None
        self.last_tick_sec = time.monotonic()
        self.last_reason = "starting"

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )

        self.create_subscription(
            String,
            str(topics.get("state", "/qwen_vln/state")),
            self._on_state,
            reliable,
        )
        self.create_subscription(
            String,
            str(topics.get("result_json", "/qwen_vln/result_json")),
            self._on_result,
            reliable,
        )
        self.create_subscription(
            LaserScan,
            str(topics.get("scan", "/scan_filtered")),
            self._on_scan,
            sensor_qos,
        )
        fallback_scan = str(topics.get("scan_fallback", "")).strip()
        if fallback_scan and fallback_scan != str(topics.get("scan", "/scan_filtered")):
            self.create_subscription(
                LaserScan, fallback_scan, self._on_scan, sensor_qos
            )
        self.create_subscription(
            String,
            str(topics.get("servo_command", "/qwen_vln/servo/command")),
            self._on_servo_command,
            reliable,
        )

        self.cmd_pub = self.create_publisher(
            Twist, str(topics.get("cmd_output", "/cmd_vel_autonomy")), 10
        )
        self.raw_cmd_pub = self.create_publisher(
            Twist, str(topics.get("cmd_raw", "/qwen_vln/servo/cmd_raw")), 10
        )
        self.limited_cmd_pub = self.create_publisher(
            Twist,
            str(topics.get("cmd_limited", "/qwen_vln/servo/cmd_limited")),
            10,
        )
        self.status_pub = self.create_publisher(
            String, str(topics.get("status", "/qwen_vln/servo/status")), 10
        )
        self.front_pub = self.create_publisher(
            Float32,
            str(topics.get("front_distance", "/qwen_vln/servo/front_distance")),
            10,
        )
        self.enabled_pub = self.create_publisher(
            Bool, str(topics.get("enabled", "/qwen_vln/servo/enabled")), 10
        )

        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)
        self.get_logger().info(
            "Qwen visual servo ready: "
            f"motion_enabled={self.motion_enabled} "
            f"cmd={topics.get('cmd_output', '/cmd_vel_autonomy')} "
            f"scan={topics.get('scan', '/scan_filtered')} "
            f"max_vx={servo_cfg.max_vx:.3f} max_wz={servo_cfg.max_wz:.3f} "
            f"point_policy=any_valid_point blocked_states={list(servo_cfg.blocked_states)}"
        )
        if not self.motion_enabled:
            self.get_logger().warning(
                "DRY RUN: only zero is sent to the chassis topic. "
                "Restart with --enable-motion after checking Foxglove status."
            )

    def _on_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.state = str(payload.get("state", self.state))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"invalid state JSON: {exc}")

    def _on_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            request_id = int(payload.get("request_id", -1))
            if request_id <= self.last_request_id:
                return
            self.last_request_id = request_id
            self.result = str(payload.get("result", ""))
            point = payload.get("point")
            if isinstance(point, dict) and "x" in point:
                parsed_x = float(point["x"])
                if not math.isfinite(parsed_x):
                    raise ValueError("point.x must be finite")
                self.point_x = parsed_x
                parsed_y = None if point.get("y") is None else float(point.get("y"))
                if parsed_y is not None and not math.isfinite(parsed_y):
                    raise ValueError("point.y must be finite")
                self.point_y = parsed_y
            else:
                self.point_x = None
                self.point_y = None
            self.point_role = str(payload.get("point_role", "none"))
            self.image_width = max(
                2, int(payload.get("image_width", self.image_width))
            )
            self.confidence = float(payload.get("confidence", 0.0))
            self.latency_ms = max(0.0, float(payload.get("latency_ms", 0.0)))
            self.result_received_sec = time.monotonic()
            if self.point_x is not None:
                self.point_streak += 1
            else:
                self.point_streak = 0
        except Exception as exc:  # noqa: BLE001
            self.point_streak = 0
            self.point_x = None
            self.point_y = None
            self.point_role = "none"
            self.get_logger().warning(f"invalid result JSON: {exc}")

    def _on_scan(self, msg: LaserScan) -> None:
        try:
            ranges = np.asarray(msg.ranges, dtype=np.float64)
            if ranges.size == 0:
                return
            angles = float(msg.angle_min) + np.arange(ranges.size) * float(
                msg.angle_increment
            )
            sector = (
                np.abs(_angle_delta(angles, self.front_center_rad))
                <= self.front_half_width_rad
            )
            valid = (
                sector
                & np.isfinite(ranges)
                & (ranges >= max(0.0, float(msg.range_min)))
                & (ranges <= float(msg.range_max))
                & (ranges > 0.02)
            )
            values = ranges[valid]
            if values.size == 0:
                self.front_distance = None
            else:
                self.front_distance = float(
                    np.percentile(values, self.front_percentile)
                )
            self.scan_received_sec = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"scan parse failed: {exc}")

    def _on_servo_command(self, msg: String) -> None:
        command = (msg.data or "").strip().lower()
        if command in {"enable", "start", "run"}:
            self.motion_enabled = True
            self.get_logger().warning("motion ENABLED by servo command")
        elif command in {"disable", "stop", "pause"}:
            self.motion_enabled = False
            self._publish_zero()
            self.get_logger().warning("motion DISABLED by servo command")
        elif command in {"reset"}:
            self.point_streak = 0
            self.limiter.reset()
            self._publish_zero()
        else:
            self.get_logger().warning(
                "servo command must be enable/disable/reset"
            )

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self.last_tick_sec
        self.last_tick_sec = now
        decision = self.servo.compute(
            ServoInput(
                now_sec=now,
                state=self.state,
                result=self.result,
                point_role=self.point_role,
                point_x=self.point_x,
                image_width=self.image_width,
                confidence=self.confidence,
                latency_ms=self.latency_ms,
                result_received_sec=self.result_received_sec,
                point_streak=self.point_streak,
                front_distance=self.front_distance,
                scan_received_sec=self.scan_received_sec,
            )
        )

        raw = self._twist(decision.vx, decision.wz)
        self.raw_cmd_pub.publish(raw)

        safety_stop = decision.hard_stop or not self.motion_enabled
        limited_vx, limited_wz = self.limiter.step(
            decision.vx, decision.wz, dt, hard_stop=safety_stop
        )
        if not self.motion_enabled:
            reason = "motion_disabled_dry_run"
        else:
            reason = decision.reason

        limited = self._twist(limited_vx, limited_wz)
        self.limited_cmd_pub.publish(limited)
        # Always publish at 20 Hz so the downstream watchdog receives either a
        # fresh command or an explicit zero.
        self.cmd_pub.publish(limited)
        self.enabled_pub.publish(Bool(data=bool(self.motion_enabled)))
        if self.front_distance is not None:
            self.front_pub.publish(Float32(data=float(self.front_distance)))

        status = {
            "motion_enabled": self.motion_enabled,
            "state": self.state,
            "result": self.result,
            "request_id": self.last_request_id,
            "point_policy": "any_valid_point",
            "point_streak": self.point_streak,
            "visible_streak": self.point_streak,
            "point_role": self.point_role,
            "pixel_x": self.point_x,
            "pixel_y": self.point_y,
            "image_width": self.image_width,
            "confidence": round(self.confidence, 4),
            "latency_ms": round(self.latency_ms, 1),
            "source_age_sec": (
                None
                if not math.isfinite(decision.source_age_sec)
                else round(decision.source_age_sec, 3)
            ),
            "horizontal_error": round(decision.horizontal_error, 4),
            "freshness_scale": round(decision.freshness_scale, 4),
            "heading_scale": round(decision.heading_scale, 4),
            "obstacle_scale": round(decision.obstacle_scale, 4),
            "front_distance": (
                None
                if self.front_distance is None
                else round(self.front_distance, 3)
            ),
            "raw_cmd": {
                "vx": round(decision.vx, 4),
                "wz": round(decision.wz, 4),
            },
            "limited_cmd": {
                "vx": round(limited_vx, 4),
                "wz": round(limited_wz, 4),
            },
            "reason": reason,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if reason != self.last_reason:
            self.get_logger().info(
                f"reason={reason} state={self.state} "
                f"e={decision.horizontal_error:+.3f} "
                f"front={self.front_distance} "
                f"cmd=({limited_vx:.3f},{limited_wz:.3f})"
            )
            self.last_reason = reason

    @staticmethod
    def _twist(vx: float, wz: float) -> Twist:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        return msg

    def _publish_zero(self) -> None:
        self.limiter.reset()
        zero = self._twist(0.0, 0.0)
        self.cmd_pub.publish(zero)
        self.limited_cmd_pub.publish(zero)

    def stop(self) -> None:
        for _ in range(4):
            self._publish_zero()
            time.sleep(0.03)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "configs/qwen3_vln_servo.yaml"),
    )
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="Actually publish non-zero commands. Default is dry-run zero only.",
    )
    args = parser.parse_args()

    rclpy.init()
    node: Optional[QwenVisualServoNode] = None
    try:
        node = QwenVisualServoNode(args.config, args.enable_motion)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
