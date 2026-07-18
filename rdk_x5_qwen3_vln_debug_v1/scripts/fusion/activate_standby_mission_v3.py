#!/usr/bin/env python3
"""Safely inject one mission into an already-warm Qwen/servo stack."""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=2,
    )


def mode_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class MissionGate(Node):
    def __init__(self, task: str, enable_motion: bool, timeout_sec: float) -> None:
        super().__init__("standby_mission_gate_v3")
        self.task = task
        self.enable_motion = enable_motion
        self.timeout_sec = timeout_sec

        self.qwen_state: Optional[dict[str, Any]] = None
        self.servo_status: Optional[dict[str, Any]] = None
        self.last_image_monotonic: Optional[float] = None
        self.last_scan_monotonic: Optional[float] = None
        self.last_odom_monotonic: Optional[float] = None

        reliable = reliable_qos()
        self.instruction_pub = self.create_publisher(
            String, "/qwen_vln/instruction", reliable
        )
        self.qwen_command_pub = self.create_publisher(
            String, "/qwen_vln/command", reliable
        )
        self.servo_command_pub = self.create_publisher(
            String, "/qwen_vln/servo/command", reliable
        )
        self.mode_pub = self.create_publisher(
            String,
            "/third_view/intervention/control_mode",
            mode_qos(),
        )
        self.zero_pub = self.create_publisher(
            Twist, "/cmd_vel_autonomy", reliable
        )

        self.create_subscription(
            String, "/qwen_vln/state", self._on_qwen_state, reliable
        )
        self.create_subscription(
            String, "/qwen_vln/servo/status", self._on_servo_status, reliable
        )
        # /image_raw bridge is RELIABLE in this project.
        self.create_subscription(
            Image, "/image_raw", self._on_image, reliable_qos(depth=2)
        )
        self.create_subscription(
            LaserScan, "/scan_filtered", self._on_scan, sensor_qos()
        )
        self.create_subscription(
            Odometry, "/odom", self._on_odom, reliable_qos(depth=2)
        )

    @staticmethod
    def _parse_json(data: str) -> Optional[dict[str, Any]]:
        try:
            value = json.loads(data)
        except Exception:
            return None
        return value if isinstance(value, dict) else None

    def _on_qwen_state(self, msg: String) -> None:
        parsed = self._parse_json(msg.data)
        if parsed is not None:
            self.qwen_state = parsed

    def _on_servo_status(self, msg: String) -> None:
        parsed = self._parse_json(msg.data)
        if parsed is not None:
            self.servo_status = parsed

    def _on_image(self, _msg: Image) -> None:
        self.last_image_monotonic = time.monotonic()

    def _on_scan(self, _msg: LaserScan) -> None:
        self.last_scan_monotonic = time.monotonic()

    def _on_odom(self, _msg: Odometry) -> None:
        self.last_odom_monotonic = time.monotonic()

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, deadline - time.monotonic()))

    def wait_until(self, predicate, label: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                self.get_logger().info(f"READY {label}")
                return True
        self.get_logger().error(f"timeout waiting {label}")
        return False

    def publish_mode(self, mode: str, reason: str, repeats: int = 4) -> None:
        payload = json.dumps(
            {"mode": mode, "reason": reason}, ensure_ascii=False
        )
        for _ in range(repeats):
            self.mode_pub.publish(String(data=payload))
            self.zero_pub.publish(Twist())
            self.spin_for(0.08)

    def publish_servo_command(self, command: str, repeats: int = 3) -> None:
        for _ in range(repeats):
            self.servo_command_pub.publish(String(data=command))
            self.spin_for(0.08)

    def safe_stop(self, reason: str) -> None:
        self.publish_servo_command("disable", repeats=5)
        self.publish_mode("HOLD", reason, repeats=6)
        for _ in range(5):
            self.zero_pub.publish(Twist())
            self.spin_for(0.05)

    def run(self) -> int:
        # Obtain at least one baseline state before changing the task.
        if not self.wait_until(
            lambda: self.qwen_state is not None,
            "Qwen state baseline",
            min(8.0, self.timeout_sec),
        ):
            self.safe_stop("missing_qwen_state")
            return 2
        if not self.wait_until(
            lambda: self.servo_status is not None,
            "servo status baseline",
            min(8.0, self.timeout_sec),
        ):
            self.safe_stop("missing_servo_status")
            return 3

        baseline_generation = int(self.qwen_state.get("generation", -1))

        # Primary lock: servo disabled. HOLD is an additional lock, but the
        # supervisor may later heartbeat EGO during its startup.
        self.publish_mode("HOLD", "mission_prepare")
        self.publish_servo_command("disable")
        self.publish_servo_command("reset")
        self.zero_pub.publish(Twist())

        instruction_sent_at = time.monotonic()
        for _ in range(6):
            self.instruction_pub.publish(String(data=self.task))
            self.spin_for(0.10)

        def task_acked() -> bool:
            if not self.qwen_state:
                return False
            instruction = str(self.qwen_state.get("instruction", "")).strip()
            try:
                generation = int(self.qwen_state.get("generation", -1))
            except Exception:
                return False
            return instruction == self.task and generation > baseline_generation

        if not self.wait_until(task_acked, "new Qwen instruction ACK", self.timeout_sec):
            self.safe_stop("task_ack_timeout")
            return 4

        def fresh_inputs() -> bool:
            timestamps = (
                self.last_image_monotonic,
                self.last_scan_monotonic,
                self.last_odom_monotonic,
            )
            return all(
                stamp is not None and stamp >= instruction_sent_at
                for stamp in timestamps
            )

        if not self.wait_until(fresh_inputs, "fresh image/scan/odom", self.timeout_sec):
            self.safe_stop("fresh_input_timeout")
            return 5

        # auto_enter_search is deliberately false in standby. Start inference
        # only after the new task and fresh sensors are acknowledged.
        for _ in range(5):
            self.qwen_command_pub.publish(String(data="search"))
            self.spin_for(0.10)

        if self.enable_motion:
            self.publish_mode("EGO", "mission_activated")
            self.publish_servo_command("enable", repeats=5)
            if not self.wait_until(
                lambda: bool(
                    self.servo_status
                    and self.servo_status.get("motion_enabled") is True
                ),
                "servo motion enabled",
                5.0,
            ):
                self.safe_stop("servo_enable_timeout")
                return 6
            self.get_logger().warning(
                f"MISSION ACTIVE with real motion: {self.task}"
            )
        else:
            self.publish_servo_command("disable", repeats=3)
            self.publish_mode("HOLD", "dry_run_active", repeats=5)
            self.get_logger().info(
                f"MISSION ACTIVE in dry-run/HOLD: {self.task}"
            )
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", required=True)
    parser.add_argument("--motion", action="store_true")
    parser.add_argument("--timeout", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    task = " ".join(args.task.replace("\r", " ").replace("\n", " ").split())
    if not task:
        print("empty task", file=sys.stderr)
        return 2

    rclpy.init()
    node: Optional[MissionGate] = None
    try:
        node = MissionGate(task, bool(args.motion), max(5.0, args.timeout))
        return node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.safe_stop("mission_gate_interrupted")
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
