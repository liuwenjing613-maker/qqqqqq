#!/usr/bin/env python3
"""ROS2 adapter for POINT servo and one-shot TURN view adjustment."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Optional

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
    EmergencyReverseConfig,
    EmergencyReverseController,
    QwenVisualServo,
    RateLimitConfig,
    ServoConfig,
    ServoInput,
    TurnPendingConfig,
    TurnPendingGate,
    ViewAdjustConfig,
    ViewAdjustController,
    ViewAdjustPhase,
)


def _angle_delta(angle: np.ndarray, center: float) -> np.ndarray:
    return np.arctan2(np.sin(angle - center), np.cos(angle - center))


class QwenVisualServoNode(Node):
    def __init__(self, config_path: str, force_enable_motion: bool):
        super().__init__("qwen_visual_servo_node")
        self.config_path = str(config_path)
        self.config = yaml.safe_load(
            Path(config_path).read_text(encoding="utf-8")
        ) or {}
        control = self.config.get("control", {})
        freshness = self.config.get("freshness", {})
        safety = self.config.get("safety", {})
        topics = self.config.get("topics", {})
        rate_limit = self.config.get("rate_limit", {})
        view_cfg = self.config.get("view_adjust", {})

        servo_cfg = ServoConfig(
            max_vx=float(control.get("max_vx", 0.07)),
            max_wz=float(control.get("max_wz", 0.05)),
            kp_wz=float(control.get("kp_wz", 0.05)),
            angular_sign=float(control.get("angular_sign", -1.0)),
            center_deadband=float(control.get("center_deadband", 0.06)),
            turn_only_threshold=float(
                control.get("turn_only_threshold", 0.40)
            ),
            cmd_wz_deadband=float(control.get("cmd_wz_deadband", 0.006)),
            # c is intentionally not used by V3; default 0 makes the gate inert.
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
        self.emergency_reverse = EmergencyReverseController(
            EmergencyReverseConfig(
                enabled=bool(safety.get("emergency_reverse_enabled", True)),
                # Trigger on the normal stop band, not only the tighter emergency
                # band, so the robot does not freeze between stop and emergency.
                trigger_distance=servo_cfg.stop_distance,
                clearance=float(
                    safety.get("emergency_reverse_clearance", 0.18)
                ),
                reverse_vx=float(safety.get("emergency_reverse_vx", -0.055)),
            )
        )
        self.view_adjust = ViewAdjustController(
            ViewAdjustConfig(
                turn_left_wz=float(view_cfg.get("turn_left_wz", 0.06)),
                turn_right_wz=float(view_cfg.get("turn_right_wz", -0.06)),
                pre_turn_stop_sec=float(
                    view_cfg.get("pre_turn_stop_sec", 0.15)
                ),
                turn_pulse_sec=float(view_cfg.get("turn_pulse_sec", 1.20)),
                settle_sec=float(view_cfg.get("settle_sec", 0.30)),
            )
        )
        self.turn_gate = TurnPendingGate(
            TurnPendingConfig(
                entry_distance=float(
                    view_cfg.get("turn_entry_distance", 0.45)
                ),
                entry_frames=int(view_cfg.get("turn_entry_frames", 3)),
                pending_vx=float(view_cfg.get("turn_pending_vx", 0.04)),
            )
        )
        self.pause_qwen_during_turn = bool(
            view_cfg.get("pause_qwen_during_turn", True)
        )
        self.resume_command = str(
            view_cfg.get("resume_command", "search")
        ).strip().lower()
        self.qwen_pause_owned = False

        goal_cfg = self.config.get("goal", {})
        self.goal_enabled = bool(goal_cfg.get("enabled", True))
        self.success_distance = float(goal_cfg.get("success_distance", 0.50))
        if self.success_distance <= 0.0:
            raise ValueError("goal.success_distance must be positive")
        self.mission_success = False

        self.limiter = CommandRateLimiter(
            RateLimitConfig(
                max_linear_accel=float(
                    rate_limit.get("max_linear_accel", 0.08)
                ),
                max_linear_decel=float(
                    rate_limit.get("max_linear_decel", 0.16)
                ),
                max_angular_accel=float(
                    rate_limit.get("max_angular_accel", 0.12)
                ),
                max_angular_decel=float(
                    rate_limit.get("max_angular_decel", 0.25)
                ),
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
        self.action = "STOP"
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
        if fallback_scan and fallback_scan != str(
            topics.get("scan", "/scan_filtered")
        ):
            self.create_subscription(
                LaserScan,
                fallback_scan,
                self._on_scan,
                sensor_qos,
            )
        self.create_subscription(
            String,
            str(topics.get("servo_command", "/qwen_vln/servo/command")),
            self._on_servo_command,
            reliable,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            str(topics.get("cmd_output", "/cmd_vel_autonomy")),
            10,
        )
        self.raw_cmd_pub = self.create_publisher(
            Twist,
            str(topics.get("cmd_raw", "/qwen_vln/servo/cmd_raw")),
            10,
        )
        self.limited_cmd_pub = self.create_publisher(
            Twist,
            str(topics.get("cmd_limited", "/qwen_vln/servo/cmd_limited")),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(topics.get("status", "/qwen_vln/servo/status")),
            10,
        )
        self.front_pub = self.create_publisher(
            Float32,
            str(
                topics.get(
                    "front_distance", "/qwen_vln/servo/front_distance"
                )
            ),
            10,
        )
        self.enabled_pub = self.create_publisher(
            Bool,
            str(topics.get("enabled", "/qwen_vln/servo/enabled")),
            10,
        )
        # Controller owns pause only during a TURN pulse. After settling it sends
        # search, forcing the next request to use a stable post-turn frame.
        self.qwen_command_pub = self.create_publisher(
            String,
            str(topics.get("qwen_command", "/qwen_vln/command")),
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(1.0, self.rate_hz), self._tick
        )
        self.get_logger().info(
            "Qwen action servo ready: "
            f"motion_enabled={self.motion_enabled} "
            f"cmd={topics.get('cmd_output', '/cmd_vel_autonomy')} "
            f"point_max=({servo_cfg.max_vx:.3f},{servo_cfg.max_wz:.3f}) "
            f"emergency_reverse=({self.emergency_reverse.cfg.trigger_distance:.2f}"
            f"->{self.emergency_reverse.cfg.release_distance:.2f}m,"
            f" vx={self.emergency_reverse.cfg.reverse_vx:.3f}) "
            f"turn_wz=({self.view_adjust.cfg.turn_left_wz:.3f},"
            f"{self.view_adjust.cfg.turn_right_wz:.3f}) "
            f"turn_entry=({self.turn_gate.cfg.entry_distance:.2f}m x"
            f"{self.turn_gate.cfg.entry_frames}) "
            f"pending_vx={self.turn_gate.cfg.pending_vx:.3f} "
            f"pre_stop={self.view_adjust.cfg.pre_turn_stop_sec:.2f}s "
            f"pulse={self.view_adjust.cfg.turn_pulse_sec:.2f}s "
            f"settle={self.view_adjust.cfg.settle_sec:.2f}s "
            f"goal_success={self.success_distance:.2f}m"
        )
        if not self.motion_enabled:
            self.get_logger().warning(
                "DRY RUN: raw desired commands are visible, chassis output stays zero"
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
            self.action = str(payload.get("action", "POINT")).strip().upper()
            point = payload.get("point")
            if isinstance(point, dict):
                self.point_x = float(point["x"])
                self.point_y = float(point["y"])
            else:
                self.point_x = None
                self.point_y = None
            self.point_role = str(payload.get("point_role", "none"))
            self.image_width = max(
                2,
                int(payload.get("image_width", self.image_width)),
            )
            # c is displayed only. It does not choose or suppress an action.
            self.confidence = float(payload.get("confidence", 0.0))
            self.latency_ms = max(
                0.0,
                float(payload.get("latency_ms", 0.0)),
            )
            self.result_received_sec = time.monotonic()

            if self.action == "POINT" and self.point_x is not None:
                self.point_streak += 1
                self.turn_gate.cancel()
                self.view_adjust.cancel()
                self._release_qwen_pause(send_resume=False)
            else:
                self.point_streak = 0

            if self.action in {"TURN_LEFT", "TURN_RIGHT"}:
                started = self.turn_gate.start(self.action, request_id)
                if started:
                    self.get_logger().warning(
                        f"pending {self.action} request_id={request_id} "
                        f"front_distance={self.front_distance}"
                    )
            elif self.action == "STOP":
                self.turn_gate.cancel()
                self.view_adjust.cancel()
                self.limiter.reset()
                self._release_qwen_pause(send_resume=False)
        except Exception as exc:  # noqa: BLE001
            self.point_streak = 0
            self.point_x = None
            self.point_y = None
            self.action = "STOP"
            self.turn_gate.cancel()
            self.view_adjust.cancel()
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
            self.front_distance = (
                None
                if values.size == 0
                else float(np.percentile(values, self.front_percentile))
            )
            self.scan_received_sec = time.monotonic()
            self.turn_gate.update_scan(self.front_distance)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"scan parse failed: {exc}")

    def _on_servo_command(self, msg: String) -> None:
        command = (msg.data or "").strip().lower()
        if command in {"enable", "start", "run"}:
            self.motion_enabled = True
            self.mission_success = False
            self.get_logger().warning("motion ENABLED by servo command")
        elif command in {"disable", "stop", "pause"}:
            self.motion_enabled = False
            self._cancel_turn_and_resume_qwen()
            self._publish_zero()
            self.get_logger().warning("motion DISABLED by servo command")
        elif command == "reset":
            self.point_streak = 0
            self.mission_success = False
            self._cancel_turn_and_resume_qwen()
            self._publish_zero()
        else:
            self.get_logger().warning(
                "servo command must be enable/disable/reset"
            )

    def _target_visible_in_fov(self) -> bool:
        result = str(self.result or "").strip().upper()
        role = str(self.point_role or "none").strip().lower()
        action = str(self.action or "").strip().upper()
        return (
            action == "POINT"
            and self.point_x is not None
            and (result == "TARGET_VISIBLE" or role == "target")
        )

    def _declare_mission_success(self, front_distance: float) -> None:
        if self.mission_success:
            return
        self.mission_success = True
        self.motion_enabled = False
        self.emergency_reverse.reset()
        self.turn_gate.cancel()
        self.view_adjust.cancel()
        self.qwen_pause_owned = False
        self.limiter.reset()
        self._send_qwen_command("success")
        self._publish_zero()
        self.get_logger().warning(
            "MISSION SUCCESS: target visible and "
            f"front_distance={front_distance:.3f}m "
            f"<= success_distance={self.success_distance:.3f}m; "
            "stopped and exited navigation"
        )

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self.last_tick_sec
        self.last_tick_sec = now

        horizontal_error = 0.0
        source_age = float("inf")
        freshness_scale = 0.0
        heading_scale = 0.0
        obstacle_scale = 0.0
        view_phase = self.view_adjust.phase.value
        bypass_rate_limit = False

        scan_fresh = (
            self.front_distance is not None
            and self.scan_received_sec is not None
            and now - self.scan_received_sec <= self.servo.cfg.scan_timeout_sec
        )

        # Simple arrive-and-exit: visible target + close enough lidar reading.
        if (
            self.goal_enabled
            and not self.mission_success
            and self._target_visible_in_fov()
            and scan_fresh
            and self.front_distance is not None
            and self.front_distance <= self.success_distance
        ):
            self._declare_mission_success(self.front_distance)

        if self.mission_success:
            desired_vx = desired_wz = 0.0
            hard_stop = True
            reason = "mission_success"
            view_phase = "MISSION_SUCCESS"
            self.emergency_reverse.reset()
            self.turn_gate.cancel()
            self.view_adjust.cancel()
        else:
            was_emergency_reversing = self.emergency_reverse.active
            emergency_reversing = (
                self.emergency_reverse.update(self.front_distance)
                if scan_fresh
                else self.emergency_reverse.active
            )
            if was_emergency_reversing and not emergency_reversing:
                self.limiter.reset()
                self._send_qwen_command(self.resume_command)
                self.get_logger().warning(
                    "emergency reverse cleared; requested fresh observation"
                )

            if emergency_reversing:
                self.turn_gate.cancel()
                self.view_adjust.cancel()
                self._release_qwen_pause(send_resume=False)
                desired_wz = 0.0
                view_phase = "EMERGENCY_REVERSE"
                if scan_fresh:
                    desired_vx = self.emergency_reverse.cfg.reverse_vx
                    hard_stop = False
                    reason = "emergency_reverse"
                else:
                    desired_vx = 0.0
                    hard_stop = True
                    reason = "emergency_reverse_wait_lidar"
            elif self.view_adjust.active:
                view = self.view_adjust.update(now)
                desired_vx, desired_wz = view.vx, view.wz
                hard_stop = view.hard_stop
                reason = view.reason
                view_phase = view.phase.value
                bypass_rate_limit = view.phase == ViewAdjustPhase.TURNING
                if view.request_fresh_observation:
                    self._release_qwen_pause(send_resume=True)
            elif self.turn_gate.active:
                view_phase = (
                    f"TURN_PENDING_{self.turn_gate.action.removeprefix('TURN_')}"
                )
                if self.turn_gate.ready:
                    turn_action = self.turn_gate.action
                    turn_request_id = self.turn_gate.request_id
                    self.turn_gate.cancel()
                    started = self.view_adjust.start(
                        turn_action,
                        turn_request_id,
                        now,
                    )
                    if started:
                        self.limiter.reset()
                        if self.pause_qwen_during_turn:
                            self._send_qwen_command("pause")
                            self.qwen_pause_owned = True
                        self.get_logger().warning(
                            f"start {turn_action} after lidar gate "
                            f"request_id={turn_request_id}"
                        )
                        view = self.view_adjust.update(now)
                        desired_vx, desired_wz = view.vx, view.wz
                        hard_stop = view.hard_stop
                        reason = view.reason
                        view_phase = view.phase.value
                        bypass_rate_limit = (
                            view.phase == ViewAdjustPhase.TURNING
                        )
                    else:
                        desired_vx = desired_wz = 0.0
                        hard_stop = True
                        reason = "turn_request_already_used"
                        view_phase = self.view_adjust.phase.value
                elif (
                    scan_fresh
                    and self.front_distance is not None
                    and self.front_distance
                    <= self.servo.cfg.emergency_stop_distance
                ):
                    desired_vx = desired_wz = 0.0
                    hard_stop = True
                    reason = "turn_pending_emergency_obstacle"
                else:
                    desired_vx = self.turn_gate.desired_vx(
                        self.limiter.vx,
                        self.servo.cfg.max_vx,
                    )
                    desired_wz = 0.0
                    hard_stop = False
                    reason = (
                        f"turn_pending_{self.turn_gate.action.lower()}"
                        if scan_fresh
                        else "turn_pending_no_lidar"
                    )
            elif self.action == "STOP":
                desired_vx = desired_wz = 0.0
                hard_stop = True
                reason = "action_stop"
            else:
                decision = self.servo.compute(
                    ServoInput(
                        now_sec=now,
                        state=self.state,
                        result=self.result,
                        action=self.action,
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
                desired_vx, desired_wz = decision.vx, decision.wz
                hard_stop = decision.hard_stop
                reason = decision.reason
                horizontal_error = decision.horizontal_error
                source_age = decision.source_age_sec
                freshness_scale = decision.freshness_scale
                heading_scale = decision.heading_scale
                obstacle_scale = decision.obstacle_scale

        self.raw_cmd_pub.publish(self._twist(desired_vx, desired_wz))
        output_hard_stop = hard_stop or not self.motion_enabled
        if not self.motion_enabled:
            self.limiter.reset()
            limited_vx = limited_wz = 0.0
        elif bypass_rate_limit:
            self.limiter.vx = desired_vx
            self.limiter.wz = desired_wz
            limited_vx, limited_wz = desired_vx, desired_wz
        else:
            limited_vx, limited_wz = self.limiter.step(
                desired_vx,
                desired_wz,
                dt,
                hard_stop=output_hard_stop,
            )
        effective_reason = (
            "motion_disabled_dry_run" if not self.motion_enabled else reason
        )
        limited = self._twist(limited_vx, limited_wz)
        self.limited_cmd_pub.publish(limited)
        self.cmd_pub.publish(limited)
        self.enabled_pub.publish(Bool(data=bool(self.motion_enabled)))
        if self.front_distance is not None:
            self.front_pub.publish(
                Float32(data=float(self.front_distance))
            )

        status = {
            "motion_enabled": self.motion_enabled,
            "state": self.state,
            "result": self.result,
            "action": self.action,
            "confidence_logged_only": round(self.confidence, 1),
            "request_id": self.last_request_id,
            "point_streak": self.point_streak,
            "point_role": self.point_role,
            "pixel_x": self.point_x,
            "pixel_y": self.point_y,
            "image_width": self.image_width,
            "latency_ms": round(self.latency_ms, 1),
            "source_age_sec": (
                None
                if not math.isfinite(source_age)
                else round(source_age, 3)
            ),
            "horizontal_error": round(horizontal_error, 4),
            "freshness_scale": round(freshness_scale, 4),
            "heading_scale": round(heading_scale, 4),
            "obstacle_scale": round(obstacle_scale, 4),
            "front_distance": (
                None
                if self.front_distance is None
                else round(self.front_distance, 3)
            ),
            "view_adjust_phase": view_phase,
            "mission_success": self.mission_success,
            "success_distance": round(self.success_distance, 3),
            "emergency_reverse_active": self.emergency_reverse.active,
            "emergency_release_distance": round(
                self.emergency_reverse.cfg.release_distance,
                3,
            ),
            "turn_pending_action": (
                self.turn_gate.action if self.turn_gate.active else None
            ),
            "turn_near_count": self.turn_gate.near_count,
            "turn_entry_frames": self.turn_gate.cfg.entry_frames,
            "qwen_pause_owned": self.qwen_pause_owned,
            "raw_cmd": {
                "vx": round(desired_vx, 4),
                "wz": round(desired_wz, 4),
            },
            "limited_cmd": {
                "vx": round(limited_vx, 4),
                "wz": round(limited_wz, 4),
            },
            "reason": effective_reason,
        }
        self.status_pub.publish(
            String(data=json.dumps(status, ensure_ascii=False))
        )
        if effective_reason != self.last_reason:
            self.get_logger().info(
                f"reason={effective_reason} action={self.action} "
                f"phase={view_phase} cmd=({limited_vx:.3f},{limited_wz:.3f})"
            )
            self.last_reason = effective_reason

    def _send_qwen_command(self, command: str) -> None:
        self.qwen_command_pub.publish(String(data=str(command)))

    def _release_qwen_pause(self, send_resume: bool) -> None:
        if not self.qwen_pause_owned:
            return
        self.qwen_pause_owned = False
        if send_resume:
            self._send_qwen_command(self.resume_command)
            self.get_logger().info(
                f"turn settled; requested fresh Qwen mode={self.resume_command}"
            )

    def _cancel_turn_and_resume_qwen(self) -> None:
        self.emergency_reverse.reset()
        self.turn_gate.cancel()
        self.view_adjust.cancel()
        if self.qwen_pause_owned:
            self._release_qwen_pause(send_resume=True)
        self.limiter.reset()

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
        self._cancel_turn_and_resume_qwen()
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
        help="Actually publish non-zero commands. Default is dry-run.",
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
