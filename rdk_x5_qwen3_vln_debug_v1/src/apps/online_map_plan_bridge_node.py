#!/usr/bin/env python3
"""Bridge the tested intervention manager to an online map/Qwen/A* backend.

The old run_joy_map_qwen_plan_session.sh is an excellent offline debug session,
but it waits for manual OK, saves a map, and may tear down live SLAM before a
cold Nav2 start.  This node keeps the useful contract (candidate extraction,
Qwen selection, full goal pose, navigation result) while replacing the session
filesystem boundary with request-id-scoped ROS topics.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fusion.online_map_protocol import (
    BridgeConfig,
    BridgeSession,
    ProtocolError,
    build_backend_request,
    normalize_backend_status,
    normalize_candidate_summary,
)
from intervention.flow_log import FlowLogger, default_flow_log_path


def _load_config(path: str) -> Dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    raw = data.get("online_map_plan_fusion", data) or {}
    if not isinstance(raw, dict):
        raise ValueError("online_map_plan_fusion config must be a mapping")
    return raw


def _json_message(payload: Dict[str, Any]) -> String:
    return String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))


class OnlineMapPlanBridge(Node):
    def __init__(self, config_path: str, task: str):
        super().__init__("online_map_plan_bridge")
        raw = _load_config(config_path)
        if not bool(raw.get("enabled", False)):
            raise RuntimeError("online_map_plan_fusion.enabled is false")

        topics = raw.get("topics", {}) or {}
        safety = raw.get("safety", {}) or {}
        timing = raw.get("timing", {}) or {}
        probe = raw.get("candidate_probe", {}) or {}
        integration = raw.get("integration", {}) or {}

        self.task = str(task or integration.get("default_task", "")).strip()
        self.map_topic = str(topics.get("map", "/map"))
        self.odom_topic = str(topics.get("odom", "/odom"))
        self.scan_topic = str(topics.get("scan", "/scan_filtered"))

        self.intervention_request_topic = str(
            topics.get("intervention_request", "/third_view/intervention/request")
        )
        self.intervention_cancel_topic = str(
            topics.get("intervention_cancel", "/third_view/intervention/cancel")
        )
        self.intervention_status_topic = str(
            topics.get("intervention_status", "/third_view/intervention/status")
        )
        self.intervention_navigation_status_topic = str(
            topics.get("intervention_navigation_status", "/third_view/navigation_status")
        )
        self.intervention_candidate_topic = str(
            topics.get("intervention_candidate_summary", "/third_view/candidate_summary")
        )
        self.map_cmd_topic = str(topics.get("map_cmd_output", "/cmd_vel_map"))

        self.backend_request_topic = str(
            topics.get("backend_request", "/map_qwen_plan/request")
        )
        self.backend_cancel_topic = str(
            topics.get("backend_cancel", "/map_qwen_plan/cancel")
        )
        self.backend_status_topic = str(
            topics.get("backend_status", "/map_qwen_plan/status")
        )
        self.backend_candidate_topic = str(
            topics.get("backend_candidate_summary", "/map_qwen_plan/candidate_summary")
        )
        self.backend_cmd_topic = str(
            topics.get("backend_cmd", "/map_qwen_plan/cmd_vel")
        )
        self.backend_probe_topic = str(
            topics.get("backend_candidate_probe", "/map_qwen_plan/candidate_probe")
        )
        self.bridge_status_topic = str(
            topics.get("bridge_status", "/map_qwen_plan/bridge_status")
        )

        self.use_memory = bool(integration.get("use_memory", False))
        self.require_final_orientation = bool(
            integration.get("require_final_orientation", False)
        )
        self.auto_ack_to_intervention = bool(
            integration.get("auto_ack_to_intervention", True)
        )

        cfg = BridgeConfig(
            request_timeout_sec=float(timing.get("backend_request_timeout_sec", 8.0)),
            backend_heartbeat_timeout_sec=float(
                timing.get("backend_heartbeat_timeout_sec", 8.0)
            ),
            backend_cmd_timeout_sec=float(timing.get("backend_cmd_timeout_sec", 0.45)),
            cancel_hold_sec=float(timing.get("cancel_hold_sec", 0.40)),
            output_rate_hz=float(timing.get("output_rate_hz", 20.0)),
            max_linear_x=float(safety.get("max_linear_x", 0.06)),
            max_angular_z=float(safety.get("max_angular_z", 0.06)),
            require_final_orientation=self.require_final_orientation,
        )
        self.session = BridgeSession(cfg)
        self.last_forwarded_request: Optional[Dict[str, Any]] = None
        self.last_backend_payload: Optional[Dict[str, Any]] = None
        self._last_flow_backend_status: Optional[str] = None
        self.latest_candidate_summary: Optional[Dict[str, Any]] = None
        self.intervention_phase = "EGO"
        self.probe_enabled = bool(probe.get("enabled", True))
        self.probe_interval_sec = float(probe.get("interval_sec", 1.0))
        self.probe_seq = 0
        self.last_probe_at = float("-inf")
        self.last_status_reason = "startup"
        self.last_effective_cmd_reason = "startup"

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        # Best-effort subscriber connects to either a reliable or best-effort
        # backend publisher.  The output to the existing mux stays reliable.
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.backend_request_pub = self.create_publisher(
            String, self.backend_request_topic, reliable
        )
        self.backend_cancel_pub = self.create_publisher(
            String, self.backend_cancel_topic, reliable
        )
        self.backend_probe_pub = self.create_publisher(
            String, self.backend_probe_topic, reliable
        )
        self.intervention_nav_pub = self.create_publisher(
            String, self.intervention_navigation_status_topic, reliable
        )
        self.intervention_candidate_pub = self.create_publisher(
            String, self.intervention_candidate_topic, reliable
        )
        self.map_cmd_pub = self.create_publisher(Twist, self.map_cmd_topic, reliable)
        self.bridge_status_pub = self.create_publisher(
            String, self.bridge_status_topic, reliable
        )

        self.create_subscription(
            String,
            self.intervention_request_topic,
            self._on_intervention_request,
            reliable,
        )
        self.create_subscription(
            String,
            self.intervention_cancel_topic,
            self._on_intervention_cancel,
            reliable,
        )
        self.create_subscription(
            String,
            self.intervention_status_topic,
            self._on_intervention_status,
            reliable,
        )
        self.create_subscription(
            String, self.backend_status_topic, self._on_backend_status, reliable
        )
        self.create_subscription(
            String,
            self.backend_candidate_topic,
            self._on_backend_candidate_summary,
            reliable,
        )
        self.create_subscription(
            Twist, self.backend_cmd_topic, self._on_backend_cmd, best_effort
        )

        self.timer = self.create_timer(
            1.0 / max(1.0, cfg.output_rate_hz), self._tick
        )
        self.status_timer = self.create_timer(0.5, self._publish_bridge_status)
        self.flow = FlowLogger(
            default_flow_log_path(PROJECT_ROOT),
            also_stdout=False,
            source="bridge",
        )
        self.flow.event(
            "READY",
            f"地图桥就绪 | request→backend | auto_ack={self.auto_ack_to_intervention} | "
            f"task={self.task!r}",
        )
        self.get_logger().info(
            "online map-plan bridge ready: "
            f"{self.intervention_request_topic} -> {self.backend_request_topic}; "
            f"backend_cmd={self.backend_cmd_topic} -> {self.map_cmd_topic}; "
            f"memory={self.use_memory}"
        )

    def _parse_json(self, msg: String, label: str) -> Optional[Dict[str, Any]]:
        try:
            value = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"invalid {label} JSON: {exc}")
            return None
        if not isinstance(value, dict):
            self.get_logger().warning(f"invalid {label}: JSON root must be object")
            return None
        return value

    def _on_intervention_request(self, msg: String) -> None:
        payload = self._parse_json(msg, "intervention request")
        if payload is None:
            return
        now = time.monotonic()
        request_id = str(payload.get("request_id", "")).strip()
        if not request_id:
            self.get_logger().warning("ignore intervention request without request_id")
            return
        if self.session.state != "IDLE":
            # Duplicate delivery of the same request is harmless; a different
            # request while active is rejected rather than stealing the chassis.
            if request_id == self.session.active_request_id:
                return
            self._publish_navigation_status(
                request_id,
                "REJECTED",
                reason=f"bridge_busy:{self.session.active_request_id}",
            )
            self.flow.event(
                "HANDSHAKE",
                f"拒绝请求 {request_id} | bridge忙于 {self.session.active_request_id}",
            )
            return
        try:
            backend_request = build_backend_request(
                payload,
                instruction=self.task,
                map_topic=self.map_topic,
                odom_topic=self.odom_topic,
                scan_topic=self.scan_topic,
                use_memory=self.use_memory,
            )
            candidate_ids = [str(x) for x in (payload.get("candidate_ids") or [])]
            if self.latest_candidate_summary is not None:
                snapshot = [
                    c for c in self.latest_candidate_summary.get("candidates", [])
                    if not candidate_ids or str(c.get("id")) in candidate_ids
                ]
                backend_request["candidate_snapshot"] = snapshot
                backend_request["map_version"] = self.latest_candidate_summary.get("map_version")
            self.session.start(request_id, now)
        except ProtocolError as exc:
            self._publish_navigation_status(request_id, "REJECTED", reason=str(exc))
            self.flow.event("HANDSHAKE", f"拒绝请求 {request_id} | protocol: {exc}")
            return

        self.last_forwarded_request = backend_request
        self.backend_request_pub.publish(_json_message(backend_request))
        self.last_status_reason = "request_forwarded"
        if self.auto_ack_to_intervention:
            # The manager can safely switch to MAP now.  Until the backend is
            # authorized, this bridge publishes fresh zero Twist heartbeats.
            self._publish_navigation_status(
                request_id,
                "ACCEPTED",
                reason="bridge_accepted_waiting_backend",
            )
        self.flow.event(
            "HANDSHAKE",
            f"转发后端 {request_id} | op={backend_request['operation']} | "
            f"candidates={backend_request['candidate_ids']} | "
            f"auto_ack={self.auto_ack_to_intervention}",
        )
        self.get_logger().warning(
            f"forwarded {request_id}: {backend_request['operation']} "
            f"candidates={backend_request['candidate_ids']}"
        )

    def _on_intervention_cancel(self, msg: String) -> None:
        payload = self._parse_json(msg, "intervention cancel") or {}
        request_id = str(payload.get("request_id", "")).strip()
        if not self.session.active_request_id:
            return
        if request_id and request_id != self.session.active_request_id:
            return
        reason = str(payload.get("reason", "intervention_cancel"))
        cancel_payload = {
            "protocol_version": 1,
            "request_id": self.session.active_request_id,
            "reason": reason,
        }
        self.backend_cancel_pub.publish(_json_message(cancel_payload))
        self.session.cancel(time.monotonic(), reason)
        self.last_status_reason = f"cancel:{reason}"
        self.flow.event(
            "HANDSHAKE",
            f"取消后端 {cancel_payload['request_id']} | reason={reason}",
        )
        self.get_logger().warning(
            f"cancel backend request {cancel_payload['request_id']}: {reason}"
        )

    def _on_intervention_status(self, msg: String) -> None:
        payload = self._parse_json(msg, "intervention status")
        if payload is None:
            return
        self.intervention_phase = str(payload.get("phase", "UNKNOWN")).upper()

    def _on_backend_status(self, msg: String) -> None:
        payload = self._parse_json(msg, "backend status")
        if payload is None:
            return
        self.last_backend_payload = payload
        normalized = normalize_backend_status(
            payload,
            active_request_id=self.session.active_request_id,
            require_final_orientation=self.require_final_orientation,
        )
        if normalized is None:
            return
        now = time.monotonic()
        result = self.session.on_backend_status(normalized, now)
        self.intervention_nav_pub.publish(_json_message(normalized))
        self.last_status_reason = f"backend:{normalized['status'].lower()}"
        status_key = f"{normalized.get('request_id')}:{normalized['status']}"
        if status_key != self._last_flow_backend_status:
            self._last_flow_backend_status = status_key
            self.flow.event(
                "NAV",
                f"后端状态 → 介入侧 status={normalized['status']} "
                f"request={normalized.get('request_id')} "
                f"reason={normalized.get('reason', '')}",
            )
        if result in {"COMPLETED", "FAILED"}:
            self.session.reset(f"backend_{result.lower()}")
            self.flow.event(
                "NAV",
                f"会话结束 result={result} request={normalized.get('request_id')}",
            )
            self._last_flow_backend_status = None
        # TARGET_VISIBLE/TARGET_LOCKED deliberately keep the request id in a
        # zero-output FINISHING state.  The intervention manager will publish a
        # matching cancel, which must still be forwarded to stop the backend's
        # planner and asynchronous Qwen callback.

    def _on_backend_candidate_summary(self, msg: String) -> None:
        payload = self._parse_json(msg, "backend candidate summary")
        if payload is None:
            return
        try:
            normalized = normalize_candidate_summary(payload)
        except ProtocolError as exc:
            self.get_logger().warning(f"reject candidate summary: {exc}")
            return
        self.latest_candidate_summary = normalized
        self.intervention_candidate_pub.publish(_json_message(normalized))

    def _on_backend_cmd(self, msg: Twist) -> None:
        accepted = self.session.on_backend_cmd(
            msg.linear.x, msg.angular.z, time.monotonic()
        )
        if not accepted:
            return

    def _publish_navigation_status(
        self,
        request_id: str,
        status: str,
        *,
        reason: Optional[str] = None,
    ) -> None:
        payload: Dict[str, Any] = {
            "request_id": str(request_id),
            "status": str(status).upper(),
            "source": "online_map_plan_bridge",
        }
        if reason:
            payload["reason"] = reason
        self.intervention_nav_pub.publish(_json_message(payload))

    def _publish_probe(self, now: float) -> None:
        if not self.probe_enabled:
            return
        if self.intervention_phase != "EGO" or self.session.state != "IDLE":
            return
        if now - self.last_probe_at < self.probe_interval_sec:
            return
        self.probe_seq += 1
        payload = {
            "protocol_version": 1,
            "probe_id": f"probe-{self.probe_seq:08d}",
            "operation": "EXTRACT_CANDIDATES_ONLY",
            "instruction": self.task,
            "live_inputs": {
                "map_topic": self.map_topic,
                "odom_topic": self.odom_topic,
                "scan_topic": self.scan_topic,
            },
            "options": {
                "call_qwen": False,
                "execute_navigation": False,
                "use_memory": self.use_memory,
            },
        }
        self.backend_probe_pub.publish(_json_message(payload))
        self.last_probe_at = now

    def _tick(self) -> None:
        now = time.monotonic()
        event = self.session.tick(now)
        if event == "FAILED":
            request_id = self.session.active_request_id
            reason = self.session.last_reason
            if request_id:
                self._publish_navigation_status(request_id, "FAILED", reason=reason)
            self.session.reset(reason)
        elif event == "CANCEL_COMPLETE":
            self.last_status_reason = "cancel_complete"

        linear_x, angular_z, reason = self.session.output_cmd(now)
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.map_cmd_pub.publish(cmd)
        self.last_effective_cmd_reason = reason
        self._publish_probe(now)

    def _publish_bridge_status(self) -> None:
        payload = {
            "enabled": True,
            "state": self.session.state,
            "active_request_id": self.session.active_request_id,
            "backend_authorized": self.session.backend_authorized,
            "intervention_phase": self.intervention_phase,
            "last_reason": self.last_status_reason,
            "cmd_reason": self.last_effective_cmd_reason,
            "task": self.task,
            "use_memory": self.use_memory,
            "require_final_orientation": self.require_final_orientation,
        }
        self.bridge_status_pub.publish(_json_message(payload))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--task", default="")
    args = parser.parse_args()

    rclpy.init()
    node: Optional[OnlineMapPlanBridge] = None
    try:
        node = OnlineMapPlanBridge(args.config, args.task)
        rclpy.spin(node)
    finally:
        if node is not None:
            zero = Twist()
            for _ in range(3):
                node.map_cmd_pub.publish(zero)
                time.sleep(0.03)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
