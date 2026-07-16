#!/usr/bin/env python3
"""Small deterministic EGO/MAP/HOLD velocity multiplexer.

It is started only when third_view_intervention.enabled=true.  The existing joy
priority mux remains downstream and therefore retains manual override priority.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Optional

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from intervention.mux_logic import choose_source


class CmdVelInterventionMux(Node):
    def __init__(self, config_path: str):
        super().__init__("cmd_vel_intervention_mux")
        root_cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
        raw = root_cfg.get("third_view_intervention", {}) or {}
        topics = raw.get("topics", {}) or {}
        mux = raw.get("cmd_mux", {}) or {}

        self.ego_topic = str(topics.get("ego_cmd", "/cmd_vel_ego"))
        self.map_topic = str(topics.get("map_cmd", "/cmd_vel_map"))
        self.output_topic = str(topics.get("mux_output", "/cmd_vel_autonomy"))
        self.mode_topic = str(
            topics.get("control_mode", "/third_view/intervention/control_mode")
        )
        self.servo_status_topic = str(
            topics.get("servo_status", "/qwen_vln/servo/status")
        )

        self.rate_hz = float(mux.get("rate_hz", 20.0))
        self.ego_timeout_sec = float(mux.get("ego_timeout_sec", 0.45))
        self.map_timeout_sec = float(mux.get("map_timeout_sec", 0.45))
        self.mode_timeout_sec = float(mux.get("mode_timeout_sec", 2.0))
        self.default_mode = str(mux.get("default_mode", "EGO")).strip().upper()
        self.safety_override_enabled = bool(
            mux.get("safety_override_enabled", True)
        )
        self.safety_status_timeout_sec = float(
            mux.get("safety_status_timeout_sec", 0.80)
        )
        self.require_fresh_safety_status_in_map = bool(
            mux.get("require_fresh_safety_status_in_map", True)
        )
        if self.default_mode not in {"EGO", "MAP", "HOLD"}:
            raise ValueError("cmd_mux.default_mode must be EGO/MAP/HOLD")

        self.mode = self.default_mode
        self.mode_received_sec = time.monotonic()
        self.ego_msg = Twist()
        self.map_msg = Twist()
        self.ego_received_sec: Optional[float] = None
        self.map_received_sec: Optional[float] = None
        self.last_effective = ""
        self.emergency_reverse_active = False
        self.servo_status_received_sec: Optional[float] = None

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Twist, self.ego_topic, self._on_ego, reliable)
        self.create_subscription(Twist, self.map_topic, self._on_map, reliable)
        self.create_subscription(String, self.mode_topic, self._on_mode, mode_qos)
        self.create_subscription(
            String, self.servo_status_topic, self._on_servo_status, reliable
        )
        self.output_pub = self.create_publisher(Twist, self.output_topic, reliable)
        self.status_pub = self.create_publisher(
            String, "/third_view/intervention/cmd_mux_status", reliable
        )
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._tick)
        self.get_logger().info(
            f"cmd mux ready: {self.ego_topic} + {self.map_topic} -> "
            f"{self.output_topic}; default={self.default_mode}"
        )

    def _on_ego(self, msg: Twist) -> None:
        self.ego_msg = msg
        self.ego_received_sec = time.monotonic()

    def _on_map(self, msg: Twist) -> None:
        self.map_msg = msg
        self.map_received_sec = time.monotonic()

    def _on_mode(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            mode = str(payload.get("mode", "")).strip().upper()
        except Exception:
            mode = str(msg.data).strip().upper()
        if mode not in {"EGO", "MAP", "HOLD"}:
            self.get_logger().warning(f"ignore invalid control mode: {mode!r}")
            return
        if mode != self.mode:
            self.get_logger().warning(f"control mode {self.mode} -> {mode}")
        self.mode = mode
        self.mode_received_sec = time.monotonic()


    def _on_servo_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.emergency_reverse_active = bool(
                payload.get("emergency_reverse_active", False)
            )
            self.servo_status_received_sec = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"invalid servo safety status: {exc}")

    def _tick(self) -> None:
        now = time.monotonic()
        ego_age = (
            None
            if self.ego_received_sec is None
            else now - self.ego_received_sec
        )
        map_age = (
            None
            if self.map_received_sec is None
            else now - self.map_received_sec
        )
        safety_age = (
            None
            if self.servo_status_received_sec is None
            else now - self.servo_status_received_sec
        )
        selection = choose_source(
            requested_mode=self.mode,
            mode_age_sec=now - self.mode_received_sec,
            ego_age_sec=ego_age,
            map_age_sec=map_age,
            safety_age_sec=safety_age,
            emergency_reverse_active=self.emergency_reverse_active,
            mode_timeout_sec=self.mode_timeout_sec,
            ego_timeout_sec=self.ego_timeout_sec,
            map_timeout_sec=self.map_timeout_sec,
            safety_status_timeout_sec=self.safety_status_timeout_sec,
            safety_override_enabled=self.safety_override_enabled,
            require_fresh_safety_status_in_map=(
                self.require_fresh_safety_status_in_map
            ),
        )

        output = Twist()
        if selection.source == "EGO":
            output = self.ego_msg
        elif selection.source == "MAP":
            output = self.map_msg

        self.output_pub.publish(output)
        key = f"{self.mode}/{selection.effective_mode}/{selection.reason}"
        if key != self.last_effective:
            self.get_logger().info(
                f"mode={self.mode} effective={selection.effective_mode} "
                f"reason={selection.reason}"
            )
            self.last_effective = key
        status = {
            "requested_mode": self.mode,
            "effective_mode": selection.effective_mode,
            "reason": selection.reason,
            "ego_age_sec": None if ego_age is None else round(ego_age, 3),
            "map_age_sec": None if map_age is None else round(map_age, 3),
            "safety_status_age_sec": (
                None if safety_age is None else round(safety_age, 3)
            ),
            "emergency_reverse_active": self.emergency_reverse_active,
            "output": {
                "vx": round(float(output.linear.x), 4),
                "wz": round(float(output.angular.z), 4),
            },
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def stop(self) -> None:
        zero = Twist()
        for _ in range(4):
            self.output_pub.publish(zero)
            time.sleep(0.03)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    rclpy.init()
    node: Optional[CmdVelInterventionMux] = None
    try:
        node = CmdVelInterventionMux(args.config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
