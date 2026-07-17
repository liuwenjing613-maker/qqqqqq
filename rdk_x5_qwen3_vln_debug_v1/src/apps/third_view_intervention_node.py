#!/usr/bin/env python3
"""ROS2 adapter for the first-version third-view intervention policy.

The node never computes a map goal itself.  It decides *when* the current
first-person VLM+lidar controller should hand control to the teammate's
candidate/Qwen/A* stack, publishes a structured request, and performs a safe
command-source handshake through ``cmd_vel_intervention_mux.py``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from intervention.core import (
    Candidate,
    CandidateSummary,
    Decision,
    DecisionKind,
    InterventionConfig,
    InterventionCore,
    PoseSample,
    ServoSample,
)
from intervention.flow_log import FlowLogger, default_flow_log_path, fmt_check


ACK_STATES = {"ACCEPTED", "PLANNING", "NAVIGATING", "RUNNING", "ACTIVE"}
DONE_STATES = {"COMPLETED", "REACHED", "SUCCEEDED", "DONE"}
FAIL_STATES = {"FAILED", "REJECTED", "CANCELLED", "ABORTED", "TIMEOUT"}
TARGET_STATES = {"TARGET_VISIBLE", "TARGET_LOCKED"}

# Quiet KEEP_EGO reasons: only log when entering/leaving, not every tick.
_QUIET_KEEP = {"EGO_HEALTHY", "STARTUP_GRACE", "COOLDOWN", "DISABLED"}


def _format_checklist(report: Dict[str, Any]) -> str:
    checks = report.get("checks") or []
    parts = [
        fmt_check(str(c.get("name")), bool(c.get("ok")), str(c.get("detail", "")))
        for c in checks
    ]
    met = report.get("met")
    total = report.get("total")
    head = ""
    if met is not None and total is not None:
        head = f"{met}/{total}项 "
    ready = " ★达标可触发" if report.get("ready") else ""
    return head + " ".join(parts) + ready


class ThirdViewInterventionNode(Node):
    def __init__(self, config_path: str, flow_log_path: Optional[str] = None):
        super().__init__("third_view_intervention_node")
        self.config_path = str(config_path)
        root_cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
        raw = root_cfg.get("third_view_intervention", {}) or {}
        self.cfg = InterventionConfig.from_mapping(raw)
        if not self.cfg.enabled:
            raise RuntimeError(
                "third_view_intervention.enabled is false; wrapper should not start this node"
            )

        topics = raw.get("topics", {}) or {}
        handshake = raw.get("handshake", {}) or {}

        self.servo_status_topic = str(
            topics.get("servo_status", "/qwen_vln/servo/status")
        )
        self.odom_topic = str(topics.get("odom", "/odom"))
        self.candidate_topic = str(
            topics.get("candidate_summary", "/third_view/candidate_summary")
        )
        self.navigation_status_topic = str(
            topics.get("navigation_status", "/third_view/navigation_status")
        )
        self.request_topic = str(
            topics.get("request", "/third_view/intervention/request")
        )
        self.decision_topic = str(
            topics.get("decision", "/third_view/intervention/decision")
        )
        self.status_topic = str(
            topics.get("status", "/third_view/intervention/status")
        )
        self.control_mode_topic = str(
            topics.get("control_mode", "/third_view/intervention/control_mode")
        )
        self.cancel_topic = str(
            topics.get("cancel", "/third_view/intervention/cancel")
        )
        self.qwen_command_topic = str(
            topics.get("qwen_command", "/qwen_vln/command")
        )
        self.map_cmd_topic = str(topics.get("map_cmd", "/cmd_vel_map"))

        self.stop_hold_sec = float(handshake.get("stop_hold_sec", 0.30))
        self.request_ack_timeout_sec = float(
            handshake.get("request_ack_timeout_sec", 6.0)
        )
        self.navigation_timeout_sec = float(
            handshake.get("navigation_timeout_sec", 120.0)
        )
        self.resume_min_hold_sec = float(
            handshake.get("resume_min_hold_sec", 0.30)
        )
        self.resume_fresh_result_timeout_sec = float(
            handshake.get("resume_fresh_result_timeout_sec", 3.0)
        )
        self.resume_qwen_command = str(
            handshake.get("resume_qwen_command", "search")
        ).strip() or "search"
        self.target_takeover_enabled = bool(
            handshake.get("target_takeover_enabled", True)
        )

        log_path = Path(
            flow_log_path
            or default_flow_log_path(PROJECT_ROOT)
        )
        self.flow = FlowLogger(log_path, also_stdout=True, source="intervention")
        self._last_flow_decision_key: Optional[Tuple[str, str]] = None
        self._last_branch_persist_logged = -1
        self._last_junction_persist_logged = -1
        self._last_recovery_stage_logged = "IDLE"
        self._last_guard_logged: Optional[str] = None

        now = time.monotonic()
        self.core = InterventionCore(self.cfg, start_stamp=now)
        self.phase = "EGO"
        self.phase_started = now
        self.pending_decision: Optional[Decision] = None
        self.trigger_request_id = -1
        self.request_seq = 0
        self.active_request_id: Optional[str] = None
        self.last_navigation_status: Optional[str] = None
        self.last_navigation_status_sec: Optional[float] = None
        self.last_published_decision_key: Optional[Tuple[str, str]] = None
        self.last_servo_payload: Dict[str, Any] = {}
        self.candidate_generation_counter = 0
        self.current_control_mode = "EGO"
        self.current_control_reason = "startup"
        self.resume_reference_request_id = -1
        self.resume_fresh_ready = False

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            String, self.servo_status_topic, self._on_servo_status, reliable
        )
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, sensor_qos
        )
        self.create_subscription(
            String, self.candidate_topic, self._on_candidate_summary, reliable
        )
        self.create_subscription(
            String,
            self.navigation_status_topic,
            self._on_navigation_status,
            reliable,
        )

        self.request_pub = self.create_publisher(String, self.request_topic, reliable)
        self.decision_pub = self.create_publisher(
            String, self.decision_topic, reliable
        )
        self.status_pub = self.create_publisher(String, self.status_topic, reliable)
        self.control_mode_pub = self.create_publisher(
            String, self.control_mode_topic, latched
        )
        self.cancel_pub = self.create_publisher(String, self.cancel_topic, reliable)
        self.qwen_command_pub = self.create_publisher(
            String, self.qwen_command_topic, reliable
        )

        self._publish_control_mode("EGO", "startup")
        self.timer = self.create_timer(
            1.0 / max(1.0, self.cfg.evaluation_hz), self._tick
        )
        self.flow.section(
            "监控启动 | phase=EGO | "
            f"branch连续≥{self.cfg.branch_persistence_updates} "
            f"半径≤{self.cfg.branch_decision_radius_m:.2f}m "
            f"夹角≥{self.cfg.branch_min_heading_separation_deg:.0f}° "
            f"分差≤{self.cfg.branch_max_top_score_gap:.2f} "
            f"比值≤{self.cfg.branch_max_top_score_ratio:.2f} | "
            f"grace={self.cfg.startup_grace_sec:.0f}s cooldown={self.cfg.cooldown_sec:.0f}s | "
            f"log={log_path}"
        )
        self.get_logger().info(
            "third-view intervention ready: "
            f"servo={self.servo_status_topic} odom={self.odom_topic} "
            f"candidates={self.candidate_topic} request={self.request_topic} "
            f"map_cmd={self.map_cmd_topic} flow_log={log_path}"
        )

    def _on_servo_status(self, msg: String) -> None:
        now = time.monotonic()
        try:
            payload = json.loads(msg.data)
            self.last_servo_payload = payload
            limited = payload.get("limited_cmd", {}) or {}
            sample = ServoSample(
                stamp=now,
                state=str(payload.get("state", "WAIT_IMAGE")),
                result=str(payload.get("result", "")),
                action=str(payload.get("action", "STOP")).strip().upper(),
                request_id=int(payload.get("request_id", -1)),
                point_role=str(payload.get("point_role", "none")),
                horizontal_error=float(payload.get("horizontal_error", 0.0) or 0.0),
                limited_vx=float(limited.get("vx", 0.0) or 0.0),
                limited_wz=float(limited.get("wz", 0.0) or 0.0),
                spawn_scan_phase=str(payload.get("spawn_scan_phase", "IDLE")),
                view_adjust_phase=str(payload.get("view_adjust_phase", "IDLE")),
                emergency_reverse_active=bool(
                    payload.get("emergency_reverse_active", False)
                ),
                turn_pending_action=payload.get("turn_pending_action"),
                mission_success=bool(payload.get("mission_success", False)),
                motion_enabled=bool(payload.get("motion_enabled", False)),
            )
            self.core.update_servo(sample)

            if (
                self.target_takeover_enabled
                and self.phase in {"STOPPING", "WAIT_ACK", "MAP_NAV"}
                and sample.target_visible
                and sample.request_id > self.trigger_request_id
            ):
                self._cancel_map_for_target(now, sample)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"invalid servo status JSON: {exc}")

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        p = msg.pose.pose.position
        self.core.update_pose(PoseSample(now, float(p.x), float(p.y), yaw))

    def _on_candidate_summary(self, msg: String) -> None:
        now = time.monotonic()
        try:
            payload = json.loads(msg.data)
            raw_candidates = payload.get("candidates", []) or []
            candidates = []
            for index, raw in enumerate(raw_candidates):
                if not isinstance(raw, dict):
                    continue
                candidate_id = str(raw.get("id", raw.get("candidate_id", index)))
                heading = raw.get("heading_deg", raw.get("direction_deg", 0.0))
                score = raw.get("score", raw.get("geometric_score"))
                candidates.append(
                    Candidate(
                        candidate_id=candidate_id,
                        heading_deg=float(heading or 0.0),
                        score=None if score is None else float(score),
                        reachable=bool(raw.get("reachable", True)),
                        visited=bool(raw.get("visited", False)),
                        status=str(raw.get("status", "UNSEEN")),
                        path_length=(
                            None
                            if raw.get("path_length") is None
                            else float(raw.get("path_length"))
                        ),
                    )
                )
            self.candidate_generation_counter += 1
            generation = self.candidate_generation_counter
            summary = CandidateSummary(
                stamp=now,
                generation=generation,
                candidates=tuple(candidates),
                map_version=(
                    None
                    if payload.get("map_version") is None
                    else str(payload.get("map_version"))
                ),
                decision_distance_m=(
                    None
                    if payload.get("decision_distance_m") is None
                    else float(payload.get("decision_distance_m"))
                ),
                returned_to_junction=bool(
                    payload.get("returned_to_junction", False)
                ),
                unseen_candidate_count=(
                    None
                    if payload.get("unseen_candidate_count") is None
                    else int(payload.get("unseen_candidate_count"))
                ),
                junction_id=(
                    None
                    if payload.get("junction_id") is None
                    else str(payload.get("junction_id"))
                ),
            )
            self.core.update_candidates(summary)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"invalid candidate summary JSON: {exc}")

    def _on_navigation_status(self, msg: String) -> None:
        now = time.monotonic()
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"status": str(msg.data)}
        status = str(payload.get("status", "")).strip().upper()
        request_id = payload.get("request_id")
        if not status:
            return
        # Navigation status is meaningful only for the currently active
        # handshake. Missing or stale request ids must never stop a healthy EGO
        # run or acknowledge a request that has not actually been published.
        if self.active_request_id is None:
            return
        if request_id is None:
            self.get_logger().warning(
                f"ignore navigation status without request_id: {status}"
            )
            return
        if str(request_id) != self.active_request_id:
            return
        self.last_navigation_status = status
        self.last_navigation_status_sec = now

        if status in TARGET_STATES:
            self._cancel_map_for_external_target(now, payload)
            return
        if status in ACK_STATES:
            if self.phase == "WAIT_ACK":
                self.flow.event(
                    "HANDSHAKE",
                    f"收到导航ACK status={status} request={self.active_request_id} → 切换MAP",
                )
                self._enter_phase("MAP_NAV", now, detail=f"ack={status}")
                self._publish_control_mode("MAP", f"navigation_{status.lower()}")
            return
        if status in DONE_STATES:
            self.flow.event(
                "NAV",
                f"导航结束 status={status} request={self.active_request_id}",
            )
            self._begin_resume(now, f"navigation_{status.lower()}")
            return
        if status in FAIL_STATES:
            self.flow.event(
                "NAV",
                f"导航失败/取消 status={status} request={self.active_request_id} "
                f"reason={payload.get('reason', '')}",
            )
            self._begin_resume(now, f"navigation_{status.lower()}")

    def _tick(self) -> None:
        now = time.monotonic()

        if self.phase == "EGO":
            decision = self.core.evaluate(now, external_busy=False)
            self._publish_decision(decision)
            self._flow_log_ego_decision(now, decision)
            if decision.kind in {DecisionKind.MAP_DIRECT, DecisionKind.MAP_QWEN}:
                self._start_transfer(now, decision)
        elif self.phase == "STOPPING":
            if now - self.phase_started >= self.stop_hold_sec:
                self._publish_map_request(now)
                self._enter_phase(
                    "WAIT_ACK",
                    now,
                    detail=(
                        f"停车{self.stop_hold_sec:.2f}s结束, "
                        f"等待ACK≤{self.request_ack_timeout_sec:.0f}s"
                    ),
                )
        elif self.phase == "WAIT_ACK":
            if now - self.phase_started >= self.request_ack_timeout_sec:
                self.flow.event(
                    "HANDSHAKE",
                    f"ACK超时 {self.request_ack_timeout_sec:.0f}s → 恢复EGO",
                )
                self.get_logger().warning("third-view request ACK timeout; resume ego")
                self._begin_resume(now, "request_ack_timeout")
        elif self.phase == "MAP_NAV":
            if now - self.phase_started >= self.navigation_timeout_sec:
                self.flow.event(
                    "NAV",
                    f"导航超时 {self.navigation_timeout_sec:.0f}s → cancel并恢复EGO",
                )
                self._publish_cancel("navigation_timeout")
                self._begin_resume(now, "navigation_timeout")
        elif self.phase == "RESUMING":
            latest = self.core.latest_servo
            if (
                latest is not None
                and latest.request_id > self.resume_reference_request_id
            ):
                self.resume_fresh_ready = True
            phase_age = now - self.phase_started
            fresh_or_timeout = (
                self.resume_fresh_ready
                or phase_age >= self.resume_fresh_result_timeout_sec
            )
            if phase_age >= self.resume_min_hold_sec and fresh_or_timeout:
                reason = (
                    "resume_fresh_result"
                    if self.resume_fresh_ready
                    else "resume_fresh_result_timeout"
                )
                self._publish_control_mode("EGO", reason)
                self._enter_phase(
                    "EGO",
                    now,
                    detail=(
                        f"恢复第一人称 ({reason}) | "
                        f"冷却{self.cfg.cooldown_sec:.0f}s"
                    ),
                )
                self.flow.section("本轮介入结束 → 回到EGO监控")
                self.core.mark_intervention_finished(now)
                self.pending_decision = None
                self.active_request_id = None
                self.last_navigation_status = None
                self.last_navigation_status_sec = None
                self.resume_reference_request_id = -1
                self.resume_fresh_ready = False
                self._last_branch_persist_logged = -1
                self._last_junction_persist_logged = -1

        self._publish_status(now)
        # Heartbeat keeps the downstream mux from treating a healthy manager as stale.
        self._emit_control_mode()

    def _flow_log_ego_decision(self, now: float, decision: Decision) -> None:
        """Log meaningful EGO-phase progress without 10 Hz spam."""
        key = (decision.kind.value, decision.reason)

        if decision.kind == DecisionKind.KEEP_EGO:
            if decision.reason != self._last_guard_logged:
                if decision.reason not in _QUIET_KEEP:
                    self.flow.event(
                        "GUARD",
                        f"暂不介入 reason={decision.reason} "
                        f"state={decision.evidence.get('state')} "
                        f"action={decision.evidence.get('action')}",
                    )
                elif (
                    self._last_guard_logged is not None
                    and self._last_guard_logged not in _QUIET_KEEP
                    and decision.reason == "EGO_HEALTHY"
                ):
                    self.flow.event("GUARD", f"解除阻塞 → {decision.reason}")
                elif (
                    self._last_guard_logged == "STARTUP_GRACE"
                    and decision.reason == "EGO_HEALTHY"
                ):
                    self.flow.event("GUARD", "启动宽限期结束 → 开始条件评估")
                self._last_guard_logged = decision.reason
        else:
            self._last_guard_logged = decision.reason

        if self.phase == "EGO" and decision.kind == DecisionKind.KEEP_EGO:
            branch = self.core.branch_condition_report(now)
            if branch is not None:
                persist = int(branch.get("persistence", 0))
                if persist != self._last_branch_persist_logged and (
                    persist > 0 or branch.get("raw_ambiguous")
                ):
                    self._last_branch_persist_logged = persist
                    self.flow.event(
                        "CHECK",
                        f"BRANCH 条件推进 持续{persist}/{branch.get('persistence_need')} | "
                        f"{_format_checklist(branch)} | "
                        f"ids={branch.get('candidate_ids')}",
                    )
            junction = self.core.junction_condition_report(now)
            if junction is not None:
                jpersist = int(junction.get("persistence", 0))
                if jpersist != self._last_junction_persist_logged and jpersist > 0:
                    self._last_junction_persist_logged = jpersist
                    self.flow.event(
                        "CHECK",
                        f"JUNCTION 条件推进 持续{jpersist}/{junction.get('persistence_need')} | "
                        f"{_format_checklist(junction)}",
                    )

        recovery = self.core.progress_recovery_report(now)
        stage = str(recovery.get("stage", "IDLE"))
        if stage != self._last_recovery_stage_logged:
            if stage != "IDLE" or self._last_recovery_stage_logged != "IDLE":
                age = recovery.get("stage_age_sec")
                age_s = f" age={age}s" if age is not None else ""
                self.flow.event(
                    "CHECK",
                    f"PROGRESS 恢复阶段 {self._last_recovery_stage_logged} → {stage}{age_s}",
                )
            self._last_recovery_stage_logged = stage

        if decision.kind == DecisionKind.RECOVERY and key != self._last_flow_decision_key:
            self._last_flow_decision_key = key
            self.flow.event(
                "TRIGGER",
                f"本地恢复(不切MAP) kind=RECOVERY reason={decision.reason} | "
                f"{self._evidence_brief(decision)}",
            )

    @staticmethod
    def _evidence_brief(decision: Decision) -> str:
        ev = decision.evidence or {}
        bits = []
        for key in (
            "eligible_candidate_count",
            "decision_distance_m",
            "max_heading_separation_deg",
            "top_score_gap",
            "top_score_ratio",
            "cmd_active_ratio",
            "integrated_cmd_distance_m",
            "actual_displacement_m",
            "turn_direction_flips",
            "horizontal_sign_flips",
            "emergency_reverse_count",
        ):
            if key in ev and ev[key] is not None:
                bits.append(f"{key}={ev[key]}")
        if decision.candidate_ids:
            bits.append(f"ids={list(decision.candidate_ids)}")
        return " ".join(bits) if bits else "—"

    def _start_transfer(self, now: float, decision: Decision) -> None:
        self.pending_decision = decision
        self.trigger_request_id = (
            -1 if self.core.latest_servo is None else self.core.latest_servo.request_id
        )
        self.request_seq += 1
        self.active_request_id = f"intervention-{self.request_seq:06d}"
        self._last_flow_decision_key = (decision.kind.value, decision.reason)

        if decision.reason == "BRANCH_AMBIGUOUS":
            report = self.core.branch_condition_report(now)
            if report is not None:
                self.flow.event("CHECK", f"BRANCH 触发前终检 | {_format_checklist(report)}")
        elif "JUNCTION" in decision.reason:
            report = self.core.junction_condition_report(now)
            if report is not None:
                self.flow.event(
                    "CHECK", f"JUNCTION 触发前终检 | {_format_checklist(report)}"
                )

        self.flow.section(
            f"★触发介入 | {decision.kind.value} | reason={decision.reason} | "
            f"request={self.active_request_id} | {self._evidence_brief(decision)}"
        )
        self._enter_phase(
            "STOPPING",
            now,
            detail=f"控制→HOLD 停车{self.stop_hold_sec:.2f}s",
        )
        self._publish_control_mode("HOLD", decision.reason)
        self.get_logger().warning(
            f"intervention trigger: {decision.kind.value} {decision.reason} "
            f"candidates={list(decision.candidate_ids)}"
        )

    def _publish_map_request(self, now: float) -> None:
        if self.pending_decision is None or self.active_request_id is None:
            self._begin_resume(now, "missing_pending_decision")
            return
        pose = self.core.latest_pose
        payload = {
            "stamp_monotonic": round(now, 6),
            "request_id": self.active_request_id,
            "decision": self.pending_decision.kind.value,
            "reason_code": self.pending_decision.reason,
            "candidate_ids": list(self.pending_decision.candidate_ids),
            "evidence": self.pending_decision.evidence,
            "robot_pose": None
            if pose is None
            else {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
            "expected_navigation_status_topic": self.navigation_status_topic,
            "map_cmd_topic": self.map_cmd_topic,
        }
        self.request_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.flow.event(
            "REQUEST",
            f"已发MAP请求 {self.active_request_id} | "
            f"{self.pending_decision.kind.value}/{self.pending_decision.reason} | "
            f"candidates={list(self.pending_decision.candidate_ids)}",
        )
        self.get_logger().warning(
            f"published third-view request {self.active_request_id}"
        )

    def _begin_resume(
        self,
        now: float,
        reason: str,
        fresh_result_already_available: bool = False,
    ) -> None:
        if self.phase == "RESUMING":
            return
        latest = self.core.latest_servo
        self.resume_reference_request_id = (
            -1 if latest is None else latest.request_id
        )
        self.resume_fresh_ready = bool(fresh_result_already_available)
        self._publish_control_mode("HOLD", reason)
        if not fresh_result_already_available:
            self.qwen_command_pub.publish(String(data=self.resume_qwen_command))
            cmd_note = f"发送Qwen命令'{self.resume_qwen_command}'"
        else:
            cmd_note = "已有新鲜第一人称结果"
        self._enter_phase(
            "RESUMING",
            now,
            detail=f"原因={reason} | {cmd_note} | 控制=HOLD",
        )

    def _cancel_map_for_target(self, now: float, sample: ServoSample) -> None:
        self._publish_cancel("fresh_first_person_target_visible")
        self.flow.event(
            "TRIGGER",
            f"第一人称看见目标 request_id={sample.request_id} → 取消MAP回EGO",
        )
        self.get_logger().warning(
            f"fresh target request_id={sample.request_id}; cancel map and return ego"
        )
        self._begin_resume(
            now, "target_visible", fresh_result_already_available=True
        )

    def _cancel_map_for_external_target(
        self, now: float, payload: Dict[str, Any]
    ) -> None:
        self._publish_cancel("third_view_target_visible")
        self.flow.event(
            "TRIGGER",
            f"地图侧目标事件 status={payload.get('status')} → 取消MAP回EGO",
        )
        self.get_logger().warning(
            f"third-view target event: {payload.get('status')}; return ego"
        )
        self._begin_resume(now, "third_view_target_visible")

    def _publish_cancel(self, reason: str) -> None:
        payload = {
            "request_id": self.active_request_id,
            "reason": reason,
        }
        self.cancel_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.flow.event(
            "REQUEST",
            f"取消MAP request={self.active_request_id} reason={reason}",
        )

    def _publish_decision(self, decision: Decision) -> None:
        key = (decision.kind.value, decision.reason)
        if key == self.last_published_decision_key:
            return
        payload = {
            "decision": decision.kind.value,
            "reason_code": decision.reason,
            "candidate_ids": list(decision.candidate_ids),
            "evidence": decision.evidence,
        }
        self.decision_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.last_published_decision_key = key

    def _publish_status(self, now: float) -> None:
        payload = {
            "enabled": self.cfg.enabled,
            "phase": self.phase,
            "phase_age_sec": round(now - self.phase_started, 3),
            "active_request_id": self.active_request_id,
            "trigger_request_id": self.trigger_request_id,
            "navigation_status": self.last_navigation_status,
            "pending_decision": None
            if self.pending_decision is None
            else self.pending_decision.kind.value,
            "pending_reason": None
            if self.pending_decision is None
            else self.pending_decision.reason,
            "cooldown_remaining_sec": round(
                max(0.0, self.core.cooldown_until - now), 3
            ),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_control_mode(self, mode: str, reason: str) -> None:
        prev = self.current_control_mode
        self.current_control_mode = mode.upper()
        self.current_control_reason = reason
        if prev != self.current_control_mode:
            self.flow.event(
                "CTRL",
                f"控制源 {prev} → {self.current_control_mode} | reason={reason}",
            )
        self._emit_control_mode()

    def _emit_control_mode(self) -> None:
        payload = {
            "mode": self.current_control_mode,
            "reason": self.current_control_reason,
        }
        self.control_mode_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _enter_phase(self, phase: str, now: float, detail: str = "") -> None:
        old = self.phase
        self.phase = phase
        self.phase_started = now
        if old != phase:
            msg = f"{old} → {phase}"
            if detail:
                msg = f"{msg} | {detail}"
            self.flow.event("PHASE", msg)

    def stop(self) -> None:
        self.flow.event("====", "节点关闭 → HOLD")
        self._publish_control_mode("HOLD", "node_shutdown")
        time.sleep(0.05)
        try:
            self.flow.close()
        except Exception:  # noqa: BLE001
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "configs/qwen3_vln_servo.yaml"),
    )
    parser.add_argument(
        "--flow-log",
        default="",
        help="Dedicated third-view flow log path (default: logs/third_view_flow.log)",
    )
    args = parser.parse_args()

    rclpy.init()
    node: Optional[ThirdViewInterventionNode] = None
    try:
        node = ThirdViewInterventionNode(
            args.config,
            flow_log_path=args.flow_log or None,
        )
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
