#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第三视角介入模块 ROS2 状态级 Mock 测试器。

适配此前 V1 约定的话题：
  输入到介入管理器：
    /qwen_vln/servo/status            std_msgs/String(JSON)
    /odom                             nav_msgs/Odometry
    /third_view/candidate_summary     std_msgs/String(JSON)
    /third_view/navigation_status     std_msgs/String(JSON，可选自动 ACK)

  观察介入管理器输出：
    /third_view/intervention/decision
    /third_view/intervention/status
    /third_view/intervention/request
    /third_view/intervention/control_mode

本脚本不会发布 /cmd_vel，也不会启动底盘。

重要：`/qwen_vln/servo/status` 中的 `request_id` 按当前介入模块协议使用整数。
请只启动 third_view_intervention_node.py，不要启动底盘桥和速度 mux。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


POSITIVE_SCENARIOS = {
    "branch": ("MAP_QWEN", "BRANCH_AMBIGUOUS"),
    "junction": ("MAP_DIRECT", "RETURNED_JUNCTION"),
    "no_progress": ("MAP_QWEN", "NO_PROGRESS"),
    "oscillation": ("MAP_QWEN", "ACTION_OSCILLATION"),
    "emergency": ("MAP_QWEN", "REPEATED_EMERGENCY_REVERSE"),
}

NEGATIVE_SCENARIOS = frozenset(
    {
        "healthy",
        "guard_spawn",
        "guard_target",
        "guard_turn",
        "guard_emergency",
    }
)

ALL_SCENARIOS = frozenset(POSITIVE_SCENARIOS) | NEGATIVE_SCENARIOS

_overlap = set(POSITIVE_SCENARIOS) & NEGATIVE_SCENARIOS
if _overlap:
    raise ValueError(f"POSITIVE/NEGATIVE scenario overlap: {sorted(_overlap)}")

SCENARIO_DURATIONS = {
    "healthy": 14.0,
    "branch": 16.0,
    "junction": 16.0,
    "no_progress": 30.0,
    "oscillation": 20.0,
    "emergency": 18.0,
    "guard_spawn": 14.0,
    "guard_target": 14.0,
    "guard_turn": 14.0,
    "guard_emergency": 14.0,
}

_missing_durations = ALL_SCENARIOS - set(SCENARIO_DURATIONS)
_extra_durations = set(SCENARIO_DURATIONS) - ALL_SCENARIOS
if _missing_durations or _extra_durations:
    raise ValueError(
        "SCENARIO_DURATIONS mismatch: "
        f"missing={sorted(_missing_durations)} extra={sorted(_extra_durations)}"
    )


def compact_json(data: Dict[str, Any]) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


def parse_json(text: str) -> Dict[str, Any]:
    try:
        obj = json.loads(text)
        return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}


@dataclass
class NavEvent:
    due_monotonic: float
    status: str
    request_id: str


class InterventionMockNode(Node):
    def __init__(
        self,
        scenario: str,
        warmup_sec: float,
        duration_sec: float,
        auto_ack: bool,
    ) -> None:
        super().__init__("third_view_intervention_mock_test")

        self.scenario = scenario
        self.warmup_sec = warmup_sec
        self.duration_sec = duration_sec
        self.auto_ack = auto_ack
        self.start_mono = time.monotonic()

        self.status_pub = self.create_publisher(
            String, "/qwen_vln/servo/status", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.candidate_pub = self.create_publisher(
            String, "/third_view/candidate_summary", 10
        )
        self.nav_status_pub = self.create_publisher(
            String, "/third_view/navigation_status", 10
        )

        self.create_subscription(
            String,
            "/third_view/intervention/decision",
            self._on_decision,
            10,
        )
        self.create_subscription(
            String,
            "/third_view/intervention/status",
            self._on_manager_status,
            10,
        )
        self.create_subscription(
            String,
            "/third_view/intervention/request",
            self._on_request,
            10,
        )
        self.create_subscription(
            String,
            "/third_view/intervention/control_mode",
            self._on_control_mode,
            10,
        )

        self.timer = self.create_timer(0.1, self._tick)
        self.last_candidate_pub_mono = 0.0
        self.last_seen_status_key: Optional[str] = None
        self.last_control_mode: Optional[str] = None
        self.latest_request_id: Optional[str] = None
        self.nav_events: List[NavEvent] = []

        self.received_decisions: List[Dict[str, Any]] = []
        self.received_requests: List[Dict[str, Any]] = []
        self.control_modes: List[str] = []

        self.get_logger().info(
            f"scenario={scenario} warmup={warmup_sec:.1f}s "
            f"duration={duration_sec:.1f}s auto_ack={auto_ack}"
        )
        self.get_logger().info(
            "本脚本不发布速度。等待介入管理器启动保护期结束后再注入条件。"
        )

    def elapsed(self) -> float:
        return time.monotonic() - self.start_mono

    def active_elapsed(self) -> float:
        return max(0.0, self.elapsed() - self.warmup_sec)

    def _base_status(self) -> Dict[str, Any]:
        now = time.time()
        return {
            "stamp": now,
            "timestamp": now,
            "state": "SEARCHING",
            "fsm_state": "SEARCHING",
            "result": "TARGET_INFERRED",
            "point_role": "search",
            "target_visible": False,
            "mission_success": False,
            "action": "POINT",
            "request_id": 1000,
            "point_streak": 5,
            "horizontal_error": 0.0,
            "front_distance": 1.50,
            "view_adjust_phase": "IDLE",
            "spawn_scan_phase": "IDLE",
            "emergency_reverse_active": False,
            "turn_pending_action": None,
            "motion_enabled": True,
            "raw_cmd": {
                "vx": 0.0,
                "wz": 0.0,
                "linear_x": 0.0,
                "angular_z": 0.0,
            },
            "limited_cmd": {
                "vx": 0.0,
                "wz": 0.0,
                "linear_x": 0.0,
                "angular_z": 0.0,
            },
            "reason": f"mock_{self.scenario}",
        }

    def _status_for_scenario(self) -> Dict[str, Any]:
        status = self._base_status()
        t = self.active_elapsed()

        if self.scenario == "no_progress" and self.elapsed() >= self.warmup_sec:
            # 有持续前进命令，但 /odom 始终不移动。
            vx = 0.040
            status["raw_cmd"].update({"vx": vx, "linear_x": vx})
            status["limited_cmd"].update({"vx": vx, "linear_x": vx})
            status["request_id"] = 2000 + int(t // 1.0)

        elif self.scenario == "oscillation" and self.elapsed() >= self.warmup_sec:
            # 0.60 s 产生一个新的 Qwen request_id，共 8 个，跨度超过 4 s。
            idx = min(7, int(t / 0.60))
            action = "TURN_LEFT" if idx % 2 == 0 else "TURN_RIGHT"
            error = -0.45 if idx % 2 == 0 else 0.45
            status["request_id"] = 3000 + idx
            status["action"] = action
            status["horizontal_error"] = error
            # 注意：只模拟“最新模型动作”，并不声明当前正在执行 TURN。
            status["view_adjust_phase"] = "IDLE"
            status["turn_pending_action"] = None

        elif self.scenario == "emergency" and self.elapsed() >= self.warmup_sec:
            # false -> true -> false -> true -> false，制造两个上升沿。
            if 1.0 <= t < 1.9 or 3.0 <= t < 3.9:
                status["emergency_reverse_active"] = True
                status["reason"] = "mock_emergency_reverse"
                status["raw_cmd"].update({"vx": -0.03, "linear_x": -0.03})
                status["limited_cmd"].update({"vx": -0.03, "linear_x": -0.03})
            status["request_id"] = 4000 + int(t * 10)

        elif self.scenario == "guard_spawn":
            status["state"] = "SPAWN_SCAN"
            status["fsm_state"] = "SPAWN_SCAN"
            status["spawn_scan_phase"] = "ROTATING"
            status["request_id"] = 5000 + int(self.elapsed() * 10)

        elif self.scenario == "guard_target":
            status["state"] = "TARGET_LOCKED"
            status["fsm_state"] = "TARGET_LOCKED"
            status["result"] = "TARGET_VISIBLE"
            status["point_role"] = "target"
            status["target_visible"] = True
            status["request_id"] = 6000 + int(self.elapsed() * 10)

        elif self.scenario == "guard_turn":
            status["action"] = "TURN_LEFT"
            status["view_adjust_phase"] = "ROTATING"
            status["turn_pending_action"] = "TURN_LEFT"
            status["request_id"] = 7000 + int(self.elapsed() * 10)

        elif self.scenario == "guard_emergency":
            status["emergency_reverse_active"] = True
            status["raw_cmd"].update({"vx": -0.03, "linear_x": -0.03})
            status["limited_cmd"].update({"vx": -0.03, "linear_x": -0.03})
            status["request_id"] = 8000 + int(self.elapsed() * 10)

        return status

    def _candidate_summary(self) -> Dict[str, Any]:
        now = time.time()
        two_candidates = [
            {
                "id": "F_LEFT",
                "heading_deg": -65.0,
                "score": 0.72,
                "reachable": True,
                "visited": False,
                "status": "UNSEEN",
                "path_length": 1.9,
            },
            {
                "id": "F_RIGHT",
                "heading_deg": 60.0,
                "score": 0.68,
                "reachable": True,
                "visited": False,
                "status": "UNSEEN",
                "path_length": 2.1,
            },
        ]

        base: Dict[str, Any] = {
            "stamp": now,
            "timestamp": now,
            "map_version": f"mock-map-{self.scenario}",
            "decision_distance_m": 0.80,
            "returned_to_junction": False,
            "junction_id": None,
            "unseen_candidate_count": 2,
            "candidate_count": 2,
            "candidates": two_candidates,
        }

        if self.scenario == "junction":
            base.update(
                {
                    "returned_to_junction": True,
                    "junction_id": "J_MOCK_01",
                    "unseen_candidate_count": 1,
                    "candidate_count": 1,
                    "candidates": [
                        {
                            "id": "F_ONLY",
                            "heading_deg": 75.0,
                            "score": 0.81,
                            "reachable": True,
                            "visited": False,
                            "status": "UNSEEN",
                            "path_length": 1.7,
                        }
                    ],
                }
            )
        elif self.scenario == "healthy":
            base.update(
                {
                    "unseen_candidate_count": 1,
                    "candidate_count": 1,
                    "candidates": [
                        {
                            "id": "F_SINGLE_HEALTHY",
                            "heading_deg": 5.0,
                            "score": 0.90,
                            "reachable": True,
                            "visited": False,
                            "status": "UNSEEN",
                            "path_length": 2.0,
                        }
                    ],
                }
            )

        return base

    def _publish_string(self, publisher: Any, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = compact_json(payload)
        publisher.publish(msg)

    def _publish_odom(self) -> None:
        msg = Odometry()
        now_msg = self.get_clock().now().to_msg()
        msg.header.stamp = now_msg
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # 所有逻辑测试都保持位姿不动。这样 no_progress 可精确复现，
        # 其余场景不会因实际位移影响统计窗口。
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def _publish_nav_events(self) -> None:
        now = time.monotonic()
        due = [event for event in self.nav_events if event.due_monotonic <= now]
        self.nav_events = [
            event for event in self.nav_events if event.due_monotonic > now
        ]
        for event in due:
            payload = {
                "request_id": event.request_id,
                "status": event.status,
                "stamp": time.time(),
            }
            self._publish_string(self.nav_status_pub, payload)
            self.get_logger().info(
                f"[MOCK NAV] request_id={event.request_id} status={event.status}"
            )

    def _tick(self) -> None:
        self._publish_string(self.status_pub, self._status_for_scenario())
        self._publish_odom()
        self._publish_nav_events()

        now_mono = time.monotonic()
        if (
            self.elapsed() >= self.warmup_sec
            and now_mono - self.last_candidate_pub_mono >= 0.90
        ):
            self._publish_string(self.candidate_pub, self._candidate_summary())
            self.last_candidate_pub_mono = now_mono

    def _on_decision(self, msg: String) -> None:
        data = parse_json(msg.data)
        self.received_decisions.append(data)
        self.get_logger().info(
            "[DECISION] " + (compact_json(data) if data else msg.data)
        )

    def _on_request(self, msg: String) -> None:
        data = parse_json(msg.data)
        self.received_requests.append(data)
        self.latest_request_id = str(data.get("request_id", "") or "")
        self.get_logger().info(
            "[REQUEST] " + (compact_json(data) if data else msg.data)
        )

        if self.auto_ack and self.latest_request_id:
            now = time.monotonic()
            self.nav_events.extend(
                [
                    NavEvent(now + 0.40, "ACCEPTED", self.latest_request_id),
                    NavEvent(now + 0.90, "NAVIGATING", self.latest_request_id),
                    NavEvent(now + 3.50, "COMPLETED", self.latest_request_id),
                ]
            )

    def _on_control_mode(self, msg: String) -> None:
        text = msg.data.strip()
        data = parse_json(text)
        mode = str(
            data.get("mode")
            or data.get("control_mode")
            or data.get("state")
            or text
        )
        if mode != self.last_control_mode:
            self.last_control_mode = mode
            self.control_modes.append(mode)
            self.get_logger().info(f"[CONTROL_MODE] {mode}")

    def _on_manager_status(self, msg: String) -> None:
        data = parse_json(msg.data)
        if data:
            key_obj = {
                "phase": data.get("phase"),
                "state": data.get("state"),
                "control_mode": data.get("control_mode"),
                "decision": data.get("decision"),
                "reason_code": data.get("reason_code"),
                "active_request_id": data.get("active_request_id"),
            }
            key = compact_json(key_obj)
            if key != self.last_seen_status_key:
                self.last_seen_status_key = key
                self.get_logger().info(f"[MANAGER_STATUS] {key}")
        elif msg.data != self.last_seen_status_key:
            self.last_seen_status_key = msg.data
            self.get_logger().info(f"[MANAGER_STATUS] {msg.data}")

    def evaluate_result(self) -> Tuple[bool, str]:
        if self.scenario in NEGATIVE_SCENARIOS:
            if self.received_requests:
                return (
                    False,
                    f"本场景应被屏蔽，但收到了 {len(self.received_requests)} 个 request",
                )
            return True, "未产生第三视角 request，硬屏蔽/健康基线符合预期"

        expected_decision, expected_reason_part = POSITIVE_SCENARIOS[self.scenario]
        if not self.received_requests:
            return False, "未收到 /third_view/intervention/request"

        matched = False
        for req in self.received_requests:
            decision = str(req.get("decision", ""))
            reason = str(req.get("reason_code", ""))
            if (
                decision == expected_decision
                and expected_reason_part in reason
            ):
                matched = True
                break

        if not matched:
            return (
                False,
                f"收到 request，但没有匹配 decision={expected_decision}, "
                f"reason 包含 {expected_reason_part}；实际={self.received_requests}",
            )

        if self.auto_ack:
            modes = " -> ".join(self.control_modes)
            if "MAP" not in modes:
                return False, f"已自动 ACK，但未观察到 MAP 控制状态；modes={modes}"
            return True, f"条件触发正确，且完成模拟握手；modes={modes}"

        return True, "条件触发正确并发布了第三视角 request"


def default_duration(scenario: str) -> float:
    return SCENARIO_DURATIONS[scenario]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="仅用 Mock 话题验证第三视角介入状态，不启动底盘。"
    )
    parser.add_argument(
        "--scenario",
        required=True,
        choices=sorted(ALL_SCENARIOS),
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=6.5,
        help="等待介入管理器启动保护期，默认 6.5 秒。",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="总测试时长；未指定时根据场景自动选择。",
    )
    parser.add_argument(
        "--auto-ack",
        action="store_true",
        help="收到 request 后模拟 ACCEPTED/NAVIGATING/COMPLETED，验证 MAP 状态握手。",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    duration = args.duration or default_duration(args.scenario)

    rclpy.init()
    node = InterventionMockNode(
        scenario=args.scenario,
        warmup_sec=args.warmup,
        duration_sec=duration,
        auto_ack=args.auto_ack,
    )

    try:
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        node.get_logger().warning("测试被手动中断")
    finally:
        passed, detail = node.evaluate_result()
        prefix = "PASS" if passed else "FAIL"
        print(f"\n[{prefix}] scenario={args.scenario}: {detail}\n")
        node.destroy_node()
        rclpy.shutdown()

    return 0 if passed else 2


if __name__ == "__main__":
    sys.exit(main())
