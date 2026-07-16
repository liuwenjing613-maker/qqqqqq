#!/usr/bin/env python3
"""Mock teammate backend for safe topic/handshake tests.

It never publishes non-zero velocity unless --motion-test is explicitly passed.
Use with wheels raised for that mode.  Humans apparently require reminders not
to test unfinished navigation on the floor, so here is one in executable form.
"""
from __future__ import annotations

import argparse
import json
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def jm(payload: Dict[str, Any]) -> String:
    return String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))


class MockBackend(Node):
    def __init__(self, scenario: str, motion_test: bool):
        super().__init__("mock_online_map_plan_backend")
        self.scenario = scenario
        self.motion_test = motion_test
        self.active_request_id: Optional[str] = None
        self.started_at = 0.0
        self.last_stage = ""
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.status_pub = self.create_publisher(String, "/map_qwen_plan/status", reliable)
        self.candidate_pub = self.create_publisher(
            String, "/map_qwen_plan/candidate_summary", reliable
        )
        self.cmd_pub = self.create_publisher(Twist, "/map_qwen_plan/cmd_vel", reliable)
        self.create_subscription(
            String, "/map_qwen_plan/request", self._on_request, reliable
        )
        self.create_subscription(
            String, "/map_qwen_plan/cancel", self._on_cancel, reliable
        )
        self.create_subscription(
            String, "/map_qwen_plan/candidate_probe", self._on_probe, reliable
        )
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().warning(
            f"MOCK backend scenario={scenario} motion_test={motion_test}"
        )

    def _candidate_summary(self) -> Dict[str, Any]:
        if self.scenario == "direct_success":
            candidates = [
                {
                    "candidate_id": "F1",
                    "relative_heading_deg": 52.0,
                    "final_score": 0.82,
                    "reachable": True,
                    "status": "UNSEEN",
                    "goal_pose": {"x": 1.2, "y": 0.4, "yaw": 0.7},
                }
            ]
        else:
            candidates = [
                {
                    "candidate_id": "F_LEFT",
                    "relative_heading_deg": 66.0,
                    "final_score": 0.72,
                    "reachable": True,
                    "status": "UNSEEN",
                    "goal_pose": {"x": 1.0, "y": 0.8, "yaw": 1.1},
                },
                {
                    "candidate_id": "F_RIGHT",
                    "relative_heading_deg": -61.0,
                    "final_score": 0.69,
                    "reachable": True,
                    "status": "UNSEEN",
                    "goal_pose": {"x": 1.1, "y": -0.7, "yaw": -1.0},
                },
            ]
        return {
            "stamp": time.time(),
            "map_seq": int(time.monotonic() * 10),
            "distance_to_decision_m": 0.75,
            "candidate_points": candidates,
        }

    def _on_probe(self, msg: String) -> None:
        del msg
        self.candidate_pub.publish(jm(self._candidate_summary()))

    def _on_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        request_id = str(payload.get("request_id", "")).strip()
        if not request_id:
            return
        self.active_request_id = request_id
        self.started_at = time.monotonic()
        self.last_stage = ""
        self.status_pub.publish(
            jm({"request_id": request_id, "state": "ACCEPTED", "mock": True})
        )

    def _on_cancel(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {}
        request_id = str(payload.get("request_id", ""))
        if self.active_request_id and request_id == self.active_request_id:
            self.status_pub.publish(
                jm({"request_id": request_id, "state": "CANCELLED", "mock": True})
            )
            self.active_request_id = None
        self.cmd_pub.publish(Twist())

    def _publish_stage(self, stage: str) -> None:
        if not self.active_request_id or stage == self.last_stage:
            return
        self.last_stage = stage
        payload: Dict[str, Any] = {
            "request_id": self.active_request_id,
            "state": stage,
            "mock": True,
        }
        if stage in {"COMPLETED", "ARRIVED_ALIGNED"}:
            payload["selected_candidate_id"] = "F_LEFT"
            payload["final_orientation_done"] = True
        self.status_pub.publish(jm(payload))

    def _tick(self) -> None:
        cmd = Twist()
        if not self.active_request_id:
            self.cmd_pub.publish(cmd)
            return
        age = time.monotonic() - self.started_at
        if age < 0.5:
            self._publish_stage("EXTRACTING")
        elif age < 1.0:
            self._publish_stage("QWEN_SELECTING")
        elif age < 3.0:
            self._publish_stage("NAVIGATING")
            if self.motion_test:
                cmd.linear.x = 0.02
                cmd.angular.z = 0.01
        else:
            if self.scenario == "fail":
                self._publish_stage("PLAN_FAILED")
            elif self.scenario == "target":
                self._publish_stage("TARGET_VISIBLE")
            else:
                self._publish_stage("ARRIVED_ALIGNED")
            self.active_request_id = None
        self.cmd_pub.publish(cmd)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        choices=("multi_success", "direct_success", "fail", "target"),
        default="multi_success",
    )
    parser.add_argument("--motion-test", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = MockBackend(args.scenario, args.motion_test)
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
