#!/usr/bin/env python3
"""Simplified, evidence-based EGO <-> MAP handoff supervisor.

The supervisor owns only the transfer decision and handshake. It does not alter
first-person navigation, candidate generation, Qwen selection, Nav2, or the
existing downstream joystick-priority mux.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from intervention.simple_handoff_core_v2 import (  # noqa: E402
    CoverageMetrics,
    CoverageThresholds,
    GridSpec,
    ProgressMetrics,
    ProgressThresholds,
    TimedCommand,
    TimedPose,
    evaluate_coverage,
    evaluate_progress,
    grid_to_world,
    polyline_length,
    trim_path_by_distance,
    wrap_angle,
)

import rclpy  # noqa: E402
from geometry_msgs.msg import Point, PoseStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String  # noqa: E402
from tf2_ros import Buffer, TransformException, TransformListener  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402


def _dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list_upper(value: Any) -> List[str]:
    return [str(x).strip().upper() for x in (value or []) if str(x).strip()]


def _qos_reliable(depth: int = 20) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _yaw_from_quaternion(q: Any) -> float:
    siny = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy = 1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2)
    return math.atan2(siny, cosy)


def _json_msg(payload: Dict[str, Any]) -> String:
    return String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))


class SimpleHandoffSupervisor(Node):
    PHASE_EGO = "EGO"
    PHASE_HOLD_TRIGGER = "HOLD_TRIGGER"
    PHASE_WAIT_ACK = "WAIT_ACK"
    PHASE_MAP = "MAP"
    PHASE_RETURN_HOLD = "RETURN_HOLD"

    def __init__(self, config_path: str, task: str = "") -> None:
        super().__init__("simple_handoff_supervisor_v2")
        self.config_path = str(config_path)
        root = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
        cfg = _dict(root.get("simple_handoff_v2", root))
        if not bool(cfg.get("enabled", True)):
            raise RuntimeError("simple_handoff_v2.enabled is false")
        self.cfg = cfg
        self.task = str(task).strip()

        topics = _dict(cfg.get("topics"))
        frames = _dict(cfg.get("frames"))
        eligibility = _dict(cfg.get("eligibility"))
        stuck = _dict(cfg.get("stuck"))
        fallback = _dict(stuck.get("fallback_without_recovery"))
        revisit = _dict(cfg.get("revisit"))
        handoff = _dict(cfg.get("handoff"))
        runtime = _dict(cfg.get("runtime"))
        visualization = _dict(cfg.get("visualization"))

        self.map_frame = str(frames.get("map", "map"))
        self.base_frame = str(frames.get("base", "base_link"))

        self.servo_topic = str(topics.get("servo_status", "/qwen_vln/servo/status"))
        self.odom_topic = str(topics.get("odom", "/odom"))
        self.map_topic = str(topics.get("map", "/map"))
        self.qwen_command_topic = str(topics.get("qwen_command", "/qwen_vln/command"))
        self.mode_topic = str(
            topics.get("control_mode", "/third_view/intervention/control_mode")
        )
        self.request_topic = str(
            topics.get("intervention_request", "/third_view/intervention/request")
        )
        self.cancel_topic = str(
            topics.get("intervention_cancel", "/third_view/intervention/cancel")
        )
        self.intervention_status_topic = str(
            topics.get("intervention_status", "/third_view/intervention/status")
        )
        self.navigation_status_topic = str(
            topics.get("navigation_status", "/third_view/navigation_status")
        )
        self.status_topic = str(
            topics.get("status", "/third_view/simple_handoff/status")
        )
        self.event_topic = str(topics.get("event", "/third_view/simple_handoff/event"))
        self.force_topic = str(topics.get("force", "/third_view/simple_handoff/force"))
        self.markers_topic = str(
            topics.get("markers", "/third_view/simple_handoff/markers")
        )
        self.recent_path_topic = str(
            topics.get("recent_path", "/third_view/simple_handoff/recent_path")
        )
        self.history_path_topic = str(
            topics.get("history_path", "/third_view/simple_handoff/history_path")
        )

        self.allowed_states = set(_list_upper(eligibility.get("allowed_states")))
        self.blocked_states = set(_list_upper(eligibility.get("blocked_states")))
        self.active_spawn_phases = set(
            _list_upper(eligibility.get("active_spawn_phases"))
        )
        self.active_view_phases = set(
            _list_upper(eligibility.get("active_view_phases"))
        )
        self.require_motion_enabled = bool(
            eligibility.get("require_motion_enabled", True)
        )
        self.block_target_visible = bool(
            eligibility.get("block_target_visible", True)
        )
        self.startup_grace_s = float(eligibility.get("startup_grace_s", 6.0))
        self.servo_max_age_s = float(eligibility.get("servo_max_age_s", 0.8))
        self.odom_max_age_s = float(eligibility.get("odom_max_age_s", 0.8))
        self.map_max_age_s = float(eligibility.get("map_max_age_s", 3.0))
        self.tf_max_age_s = float(eligibility.get("tf_max_age_s", 1.0))

        self.stuck_enabled = bool(stuck.get("enabled", True))
        self.progress_cfg = ProgressThresholds(
            window_s=float(stuck.get("window_s", 8.0)),
            max_sample_gap_s=float(stuck.get("max_sample_gap_s", 0.35)),
            pose_sample_period_s=float(stuck.get("pose_sample_period_s", 0.25)),
            min_linear_cmd_mps=float(stuck.get("min_linear_cmd_mps", 0.020)),
            min_linear_active_s=float(stuck.get("min_linear_active_s", 5.0)),
            min_commanded_linear_m=float(
                stuck.get("min_commanded_linear_m", 0.18)
            ),
            max_actual_path_m=float(stuck.get("max_actual_path_m", 0.08)),
            min_angular_cmd_radps=float(
                stuck.get("min_angular_cmd_radps", 0.020)
            ),
            min_angular_active_s=float(stuck.get("min_angular_active_s", 5.0)),
            min_commanded_yaw_rad=float(
                stuck.get("min_commanded_yaw_rad", 0.25)
            ),
            max_actual_yaw_rad=float(stuck.get("max_actual_yaw_rad", 0.10)),
        )
        self.require_local_recovery = bool(
            stuck.get("require_local_recovery", True)
        )
        self.recovery_lookback_s = float(stuck.get("recovery_lookback_s", 20.0))
        self.fallback_enabled = bool(fallback.get("enabled", True))
        self.fallback_required_windows = max(
            1, int(fallback.get("required_windows", 2))
        )
        self.fallback_min_gap_s = float(fallback.get("min_gap_s", 4.0))
        self.stuck_cooldown_after_trigger_s = float(
            stuck.get("cooldown_after_trigger_s", 12.0)
        )
        self.stuck_cooldown_after_map_s = float(
            stuck.get("cooldown_after_map_s", 2.0)
        )

        self.revisit_enabled = bool(revisit.get("enabled", True))
        self.coverage_cfg = CoverageThresholds(
            corridor_radius_m=float(revisit.get("corridor_radius_m", 0.50)),
            recent_path_length_m=float(
                revisit.get("recent_path_length_m", 1.20)
            ),
            history_gap_length_m=float(
                revisit.get("history_gap_length_m", 1.00)
            ),
            min_history_path_length_m=float(
                revisit.get("min_history_path_length_m", 0.80)
            ),
            free_cell_max_value=int(revisit.get("free_cell_max_value", 20)),
            max_new_area_ratio=float(revisit.get("max_new_area_ratio", 0.20)),
            min_valid_recent_cells=int(
                revisit.get("min_valid_recent_cells", 80)
            ),
        )
        self.path_sample_step_m = float(revisit.get("path_sample_step_m", 0.05))
        self.max_pose_jump_m = float(revisit.get("max_pose_jump_m", 0.60))
        self.max_total_path_m = float(revisit.get("max_total_path_m", 80.0))
        self.revisit_required_hits = max(
            1, int(revisit.get("required_consecutive_checks", 2))
        )
        self.revisit_check_travel_m = float(
            revisit.get("min_travel_between_checks_m", 0.35)
        )
        self.revisit_cooldown_after_map_s = float(
            revisit.get("cooldown_after_map_s", 20.0)
        )
        self.revisit_rearm_travel_m = float(
            revisit.get("rearm_travel_after_map_m", 0.60)
        )

        self.stop_hold_s = float(handoff.get("stop_hold_s", 0.30))
        self.request_ack_timeout_s = float(
            handoff.get("request_ack_timeout_s", 8.0)
        )
        self.navigation_timeout_s = float(
            handoff.get("navigation_timeout_s", 150.0)
        )
        self.resume_hold_s = float(handoff.get("resume_hold_s", 0.35))
        self.resume_qwen_command = str(
            handoff.get("resume_qwen_command", "search")
        )
        self.observe_during_map = bool(handoff.get("observe_during_map", True))
        self.decision = str(handoff.get("decision", "MAP_QWEN")).upper()
        self.cancel_on_fresh_target_visible = bool(
            handoff.get("cancel_on_fresh_target_visible", True)
        )

        self.evaluation_hz = float(runtime.get("evaluation_hz", 10.0))
        self.status_hz = float(runtime.get("status_hz", 2.0))
        self.metrics_log_interval_s = float(
            runtime.get("metrics_log_interval_s", 2.0)
        )

        self.visualization_enabled = bool(visualization.get("enabled", True))
        self.visualization_hz = float(visualization.get("publish_hz", 2.0))
        self.text_height_m = float(visualization.get("text_height_m", 0.18))
        self.line_width_m = float(visualization.get("line_width_m", 0.035))
        self.cell_size_m = float(visualization.get("cell_size_m", 0.045))
        self.max_cell_markers = int(visualization.get("max_cell_markers", 500))
        self.recent_path_z_m = float(visualization.get("recent_path_z_m", 0.045))
        self.history_path_z_m = float(
            visualization.get("history_path_z_m", 0.025)
        )

        self.started_at = time.monotonic()
        history_keep_s = max(60.0, self.progress_cfg.window_s + 15.0)
        self.commands: Deque[TimedCommand] = deque()
        self.odom_poses: Deque[TimedPose] = deque()
        self.history_keep_s = history_keep_s
        self.latest_servo: Dict[str, Any] = {}
        self.latest_servo_at: Optional[float] = None
        self.latest_odom_at: Optional[float] = None
        self.latest_map_at: Optional[float] = None
        self.latest_tf_at: Optional[float] = None
        self.latest_map_pose: Optional[Tuple[float, float, float]] = None
        self.map_msg: Optional[OccupancyGrid] = None

        self.map_path: List[Tuple[float, float]] = []
        self.path_distance_accum = 0.0
        self.last_novelty_eval_distance = float("-inf")
        self.revisit_rearm_distance = 0.0
        self.revisit_epoch_distance = 0.0
        self.revisit_cooldown_until = self.started_at + self.startup_grace_s
        self.revisit_hits = 0

        self.last_progress_metrics: Optional[ProgressMetrics] = None
        self.last_coverage_metrics: Optional[CoverageMetrics] = None
        self.last_eligibility_reason = "startup"
        self.last_trigger_reason = "none"
        self.last_event_message = "startup"
        self.last_trigger_at = float("-inf")
        self.stuck_cooldown_until = self.started_at + self.startup_grace_s
        self.fallback_no_progress_hits = 0
        self.last_fallback_hit_at = float("-inf")

        self.recovery_active = False
        self.recovery_started_at: Optional[float] = None
        self.last_recovery_completed_at: Optional[float] = None

        self.phase = self.PHASE_EGO
        self.phase_started_at = self.started_at
        self.phase_deadline = float("inf")
        self.active_request_id: Optional[str] = None
        self.active_trigger_reason: Optional[str] = None
        self.active_evidence: Dict[str, Any] = {}
        self.request_sequence = 0
        self.map_enter_at: Optional[float] = None
        self.last_metrics_log_at = float("-inf")

        reliable = _qos_reliable()
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.request_pub = self.create_publisher(String, self.request_topic, reliable)
        self.cancel_pub = self.create_publisher(String, self.cancel_topic, reliable)
        self.intervention_status_pub = self.create_publisher(
            String, self.intervention_status_topic, reliable
        )
        self.mode_pub = self.create_publisher(String, self.mode_topic, mode_qos)
        self.qwen_pub = self.create_publisher(
            String, self.qwen_command_topic, reliable
        )
        self.status_pub = self.create_publisher(String, self.status_topic, reliable)
        self.event_pub = self.create_publisher(String, self.event_topic, reliable)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.markers_topic, reliable
        )
        self.recent_path_pub = self.create_publisher(
            NavPath, self.recent_path_topic, reliable
        )
        self.history_path_pub = self.create_publisher(
            NavPath, self.history_path_topic, reliable
        )

        self.create_subscription(String, self.servo_topic, self._on_servo, reliable)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, reliable)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, reliable)
        self.create_subscription(
            String, self.navigation_status_topic, self._on_navigation_status, reliable
        )
        self.create_subscription(String, self.force_topic, self._on_force, reliable)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.eval_timer = self.create_timer(
            1.0 / max(1.0, self.evaluation_hz), self._tick
        )
        self.status_timer = self.create_timer(
            1.0 / max(0.2, self.status_hz), self._publish_status
        )
        self.visual_timer = self.create_timer(
            1.0 / max(0.2, self.visualization_hz), self._publish_visualization
        )

        self._publish_mode("EGO", "startup")
        self._event(
            "READY",
            "简化中转站就绪：A=恢复后无进展，B=低新增覆盖；参数全部来自配置文件",
            config=self.config_path,
        )
        self.get_logger().info(
            f"simple handoff ready | servo={self.servo_topic} map={self.map_topic} "
            f"request={self.request_topic} task={self.task!r}"
        )

    def _prune_windows(self, now: float) -> None:
        cutoff = now - self.history_keep_s
        while self.commands and self.commands[0].t < cutoff:
            self.commands.popleft()
        while self.odom_poses and self.odom_poses[0].t < cutoff:
            self.odom_poses.popleft()

    def _parse_json(self, text: str, label: str) -> Optional[Dict[str, Any]]:
        try:
            payload = json.loads(text)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"invalid {label} JSON: {exc}")
            return None
        if not isinstance(payload, dict):
            self.get_logger().warning(f"invalid {label}: root must be object")
            return None
        return payload

    def _on_servo(self, msg: String) -> None:
        payload = self._parse_json(msg.data, "servo status")
        if payload is None:
            return
        now = time.monotonic()
        self.latest_servo = payload
        self.latest_servo_at = now
        cmd = _dict(payload.get("limited_cmd")) or _dict(payload.get("raw_cmd"))
        try:
            vx = float(cmd.get("vx", 0.0))
            wz = float(cmd.get("wz", 0.0))
        except (TypeError, ValueError):
            vx, wz = 0.0, 0.0
        self.commands.append(TimedCommand(now, vx, wz))

        recovery_now = bool(payload.get("emergency_reverse_active", False))
        if recovery_now and not self.recovery_active:
            self.recovery_started_at = now
            self._event("RECOVERY", "第一视角局部恢复开始")
        elif not recovery_now and self.recovery_active:
            self.last_recovery_completed_at = now
            self._event("RECOVERY", "第一视角局部恢复完成")
        self.recovery_active = recovery_now
        self._prune_windows(now)

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_poses.append(
            TimedPose(now, float(p.x), float(p.y), _yaw_from_quaternion(q))
        )
        self.latest_odom_at = now
        self._prune_windows(now)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.latest_map_at = time.monotonic()

    def _on_force(self, msg: String) -> None:
        command = str(msg.data).strip().lower()
        if command in {"trigger", "map", "handoff"}:
            self._trigger("FORCED_MANUAL", {"source": "force_topic"}, force=True)
        elif command == "stuck":
            self._trigger("FORCED_STUCK", {"source": "force_topic"}, force=True)
        elif command in {"revisit", "loop", "repeat"}:
            self._trigger("FORCED_REVISIT", {"source": "force_topic"}, force=True)
        elif command in {"reset", "ego", "cancel"}:
            if self.phase != self.PHASE_EGO:
                self._begin_return("FORCED_RESET", cancel_backend=True)
            else:
                self._reset_detection_windows("forced_reset")
                self._event("RESET", "已清空触发统计，保持第一视角")
        else:
            self._event("WARN", f"未知 force 命令：{command!r}")

    def _on_navigation_status(self, msg: String) -> None:
        payload = self._parse_json(msg.data, "navigation status")
        if payload is None:
            return
        request_id = str(payload.get("request_id", "")).strip()
        if not self.active_request_id or request_id != self.active_request_id:
            return
        status = str(payload.get("status", payload.get("state", ""))).upper()
        reason = str(payload.get("reason", payload.get("backend_status", "")))
        if status in {"ACCEPTED", "NAVIGATING", "RUNNING", "ACTIVE"}:
            if self.phase == self.PHASE_WAIT_ACK:
                now = time.monotonic()
                self.phase = self.PHASE_MAP
                self.phase_started_at = now
                self.phase_deadline = now + self.navigation_timeout_s
                self.map_enter_at = now
                self._publish_mode("MAP", f"backend_{status.lower()}")
                if self.observe_during_map:
                    self._publish_qwen(self.resume_qwen_command)
                self._event(
                    "MAP_ENTER",
                    f"第三视角接管，后端状态={status}",
                    request_id=request_id,
                    backend_reason=reason,
                )
            return
        if status in {"COMPLETED", "SUCCEEDED", "DONE", "REACHED"}:
            self._begin_return(
                f"MAP_{status}", cancel_backend=False, backend_reason=reason
            )
        elif status in {
            "FAILED",
            "REJECTED",
            "NO_CANDIDATES",
            "UNREACHABLE",
            "ABORTED",
            "TIMEOUT",
            "CANCELLED",
            "CANCELED",
        }:
            self._begin_return(
                f"MAP_{status}", cancel_backend=True, backend_reason=reason
            )
        elif status in {"TARGET_VISIBLE", "TARGET_LOCKED"}:
            self._begin_return(status, cancel_backend=True, backend_reason=reason)

    def _target_visible(self) -> bool:
        payload = self.latest_servo
        result = str(payload.get("result", "")).strip().upper()
        point_role = str(payload.get("point_role", "")).strip().upper()
        action = str(payload.get("action", "")).strip().upper()
        if result in {"TARGET_VISIBLE", "TARGET_LOCKED"}:
            return True
        if point_role in {"TARGET", "TARGET_VISIBLE"} and action == "POINT":
            return True
        return False

    def _eligibility(self, now: float) -> Tuple[bool, str]:
        if self.phase != self.PHASE_EGO:
            return False, f"phase_{self.phase.lower()}"
        if now - self.started_at < self.startup_grace_s:
            return False, "startup_grace"
        if self.latest_servo_at is None or now - self.latest_servo_at > self.servo_max_age_s:
            return False, "servo_stale"
        if self.latest_odom_at is None or now - self.latest_odom_at > self.odom_max_age_s:
            return False, "odom_stale"
        if self.latest_map_at is None or now - self.latest_map_at > self.map_max_age_s:
            return False, "map_stale"
        if self.latest_tf_at is None or now - self.latest_tf_at > self.tf_max_age_s:
            return False, "tf_stale"
        if self.require_motion_enabled and not bool(
            self.latest_servo.get("motion_enabled", False)
        ):
            return False, "motion_disabled"
        if bool(self.latest_servo.get("mission_success", False)):
            return False, "mission_success"
        state = str(self.latest_servo.get("state", "")).strip().upper()
        if self.allowed_states and state not in self.allowed_states:
            return False, f"state_not_allowed:{state or 'empty'}"
        if state in self.blocked_states:
            return False, f"state_blocked:{state}"
        spawn_phase = str(
            self.latest_servo.get("spawn_scan_phase", "")
        ).strip().upper()
        if spawn_phase in self.active_spawn_phases:
            return False, f"spawn_scan:{spawn_phase}"
        view_phase = str(
            self.latest_servo.get("view_adjust_phase", "")
        ).strip().upper()
        if view_phase in self.active_view_phases:
            return False, f"view_adjust:{view_phase}"
        if self.recovery_active:
            return False, "local_recovery_active"
        if self.block_target_visible and self._target_visible():
            return False, "target_visible"
        return True, "eligible"

    def _lookup_map_pose(self, now: float) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.03),
            )
        except TransformException:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        pose = (float(t.x), float(t.y), _yaw_from_quaternion(q))
        self.latest_map_pose = pose
        self.latest_tf_at = now
        # Record all physical travel, including MAP-owned travel, because the
        # camera still observes it. Trigger evaluation itself remains frozen
        # outside EGO.
        self._append_map_path(pose[0], pose[1])

    def _append_map_path(self, x: float, y: float) -> None:
        point = (float(x), float(y))
        if not self.map_path:
            self.map_path.append(point)
            return
        step = math.hypot(point[0] - self.map_path[-1][0], point[1] - self.map_path[-1][1])
        if step < self.path_sample_step_m:
            return
        if step > self.max_pose_jump_m:
            # A map/TF discontinuity would otherwise paint a fake corridor across
            # the room and instantly poison the novelty ratio. Reset spatial
            # evidence rather than pretending teleportation was exploration.
            self.map_path = [point]
            self.revisit_hits = 0
            self.last_novelty_eval_distance = self.path_distance_accum
            self.revisit_epoch_distance = self.path_distance_accum
            self._event("MAP_PATH_RESET", f"检测到 {step:.2f}m 位姿跳变，重置覆盖轨迹")
            return
        self.path_distance_accum += step
        self.map_path.append(point)
        if polyline_length(self.map_path) > self.max_total_path_m:
            self.map_path = trim_path_by_distance(self.map_path, self.max_total_path_m)

    def _progress_evidence(self, metrics: ProgressMetrics) -> Dict[str, Any]:
        return {
            "window_covered_s": round(metrics.window_covered_s, 3),
            "linear_active_s": round(metrics.linear_active_s, 3),
            "commanded_linear_m": round(metrics.commanded_linear_m, 3),
            "actual_path_m": round(metrics.actual_path_m, 3),
            "angular_active_s": round(metrics.angular_active_s, 3),
            "commanded_yaw_rad": round(metrics.commanded_yaw_rad, 3),
            "actual_yaw_rad": round(metrics.actual_yaw_rad, 3),
            "translation_failed": metrics.translation_failed,
            "rotation_failed": metrics.rotation_failed,
        }

    def _evaluate_stuck(self, now: float) -> Optional[Tuple[str, Dict[str, Any]]]:
        if not self.stuck_enabled or now < self.stuck_cooldown_until:
            return None
        metrics = evaluate_progress(
            list(self.commands), list(self.odom_poses), now, self.progress_cfg
        )
        self.last_progress_metrics = metrics
        if not metrics.failed:
            self.fallback_no_progress_hits = 0
            return None
        evidence = self._progress_evidence(metrics)
        recovery_recent = bool(
            self.last_recovery_completed_at is not None
            and now - self.last_recovery_completed_at <= self.recovery_lookback_s
        )
        evidence["recovery_recent"] = recovery_recent
        evidence["fallback_hits"] = self.fallback_no_progress_hits
        if not self.require_local_recovery or recovery_recent:
            return "STUCK_AFTER_LOCAL_RECOVERY", evidence
        if not self.fallback_enabled:
            return None
        if now - self.last_fallback_hit_at >= self.fallback_min_gap_s:
            self.fallback_no_progress_hits += 1
            self.last_fallback_hit_at = now
            self._event(
                "STUCK_WINDOW",
                f"无进展窗口 {self.fallback_no_progress_hits}/{self.fallback_required_windows}",
                **evidence,
            )
        if self.fallback_no_progress_hits >= self.fallback_required_windows:
            evidence["fallback_hits"] = self.fallback_no_progress_hits
            return "STUCK_TWO_WINDOWS_FALLBACK", evidence
        return None

    def _grid_from_map(self) -> Tuple[np.ndarray, GridSpec]:
        if self.map_msg is None:
            raise ValueError("map unavailable")
        info = self.map_msg.info
        occupancy = np.asarray(self.map_msg.data, dtype=np.int16).reshape(
            int(info.height), int(info.width)
        )
        spec = GridSpec(
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            origin_yaw=_yaw_from_quaternion(info.origin.orientation),
        )
        return occupancy, spec

    def _evaluate_revisit(self, now: float) -> Optional[Tuple[str, Dict[str, Any]]]:
        if not self.revisit_enabled:
            return None
        if now < self.revisit_cooldown_until:
            return None
        if self.path_distance_accum < self.revisit_rearm_distance:
            return None
        # "Reset recent window" after MAP means the latest comparison corridor
        # must be accumulated after the return, while older MAP/EGO travel remains
        # valid history.
        if (
            self.path_distance_accum - self.revisit_epoch_distance
            < self.coverage_cfg.recent_path_length_m
        ):
            return None
        if (
            self.path_distance_accum - self.last_novelty_eval_distance
            < self.revisit_check_travel_m
        ):
            return None
        self.last_novelty_eval_distance = self.path_distance_accum
        try:
            occupancy, spec = self._grid_from_map()
            metrics = evaluate_coverage(
                self.map_path,
                occupancy,
                spec,
                self.coverage_cfg,
                include_masks=self.visualization_enabled,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"coverage evaluation failed: {exc}")
            self.revisit_hits = 0
            return None
        self.last_coverage_metrics = metrics
        if not metrics.available:
            self.revisit_hits = 0
            return None
        if metrics.low_novelty:
            self.revisit_hits += 1
            self._event(
                "NOVELTY_CHECK",
                f"新增覆盖 {metrics.new_area_ratio:.1%}，低收益命中 "
                f"{self.revisit_hits}/{self.revisit_required_hits}",
                new_area_ratio=round(float(metrics.new_area_ratio), 4),
                recent_valid_cells=metrics.recent_valid_cells,
                new_cells=metrics.new_cells,
                overlap_cells=metrics.overlap_cells,
            )
        else:
            self.revisit_hits = 0
        if self.revisit_hits < self.revisit_required_hits:
            return None
        return (
            "LOW_NOVELTY_REVISIT",
            {
                "new_area_ratio": round(float(metrics.new_area_ratio), 4),
                "threshold": self.coverage_cfg.max_new_area_ratio,
                "recent_path_m": round(metrics.recent_path_m, 3),
                "history_path_m": round(metrics.history_path_m, 3),
                "recent_valid_cells": metrics.recent_valid_cells,
                "new_cells": metrics.new_cells,
                "overlap_cells": metrics.overlap_cells,
                "consecutive_hits": self.revisit_hits,
            },
        )

    def _tick(self) -> None:
        now = time.monotonic()
        self._lookup_map_pose(now)
        self._prune_windows(now)

        if self.phase == self.PHASE_HOLD_TRIGGER and now >= self.phase_deadline:
            self._send_intervention_request(now)
        elif self.phase == self.PHASE_WAIT_ACK and now >= self.phase_deadline:
            self._begin_return("ACK_TIMEOUT", cancel_backend=True)
        elif self.phase == self.PHASE_MAP:
            if (
                self.cancel_on_fresh_target_visible
                and self.latest_servo_at is not None
                and self.map_enter_at is not None
                and self.latest_servo_at >= self.map_enter_at
                and now - self.latest_servo_at <= self.servo_max_age_s
                and self._target_visible()
            ):
                self._begin_return("FRESH_TARGET_VISIBLE", cancel_backend=True)
            elif now >= self.phase_deadline:
                self._begin_return("MAP_TIMEOUT", cancel_backend=True)
        elif self.phase == self.PHASE_RETURN_HOLD and now >= self.phase_deadline:
            self._finish_return(now)

        eligible, reason = self._eligibility(now)
        self.last_eligibility_reason = reason
        if eligible:
            stuck_result = self._evaluate_stuck(now)
            if stuck_result is not None:
                self._trigger(stuck_result[0], stuck_result[1])
            elif self.phase == self.PHASE_EGO:
                revisit_result = self._evaluate_revisit(now)
                if revisit_result is not None:
                    self._trigger(revisit_result[0], revisit_result[1])

        if now - self.last_metrics_log_at >= self.metrics_log_interval_s:
            self.last_metrics_log_at = now
            self._log_metrics()

    def _trigger(
        self, reason: str, evidence: Dict[str, Any], *, force: bool = False
    ) -> None:
        now = time.monotonic()
        if self.phase != self.PHASE_EGO:
            self._event("TRIGGER_IGNORED", f"当前 phase={self.phase}，忽略 {reason}")
            return
        if not force:
            eligible, why = self._eligibility(now)
            if not eligible:
                self._event("TRIGGER_IGNORED", f"资格不满足：{why}", reason=reason)
                return
            if now - self.last_trigger_at < self.stuck_cooldown_after_trigger_s:
                return
        self.request_sequence += 1
        self.active_request_id = (
            f"simple-{int(time.time())}-{self.request_sequence:04d}"
        )
        self.active_trigger_reason = str(reason)
        self.active_evidence = dict(evidence)
        self.last_trigger_reason = str(reason)
        self.last_trigger_at = now
        self.phase = self.PHASE_HOLD_TRIGGER
        self.phase_started_at = now
        self.phase_deadline = now + self.stop_hold_s
        self._publish_qwen("pause")
        self._publish_mode("HOLD", f"trigger:{reason}")
        self._event(
            "TRIGGER",
            f"第一视角 → HOLD，原因={reason}",
            request_id=self.active_request_id,
            evidence=evidence,
        )

    def _send_intervention_request(self, now: float) -> None:
        if not self.active_request_id:
            self._begin_return("INTERNAL_NO_REQUEST_ID", cancel_backend=False)
            return
        robot_pose = None
        if self.latest_map_pose is not None:
            robot_pose = {
                "frame_id": self.map_frame,
                "x": round(self.latest_map_pose[0], 4),
                "y": round(self.latest_map_pose[1], 4),
                "yaw": round(self.latest_map_pose[2], 4),
            }
        payload = {
            "protocol_version": 1,
            "request_id": self.active_request_id,
            "decision": self.decision,
            "reason_code": self.active_trigger_reason or "UNKNOWN",
            "candidate_ids": [],
            "robot_pose": robot_pose,
            "evidence": self.active_evidence,
            "task": self.task,
            "timestamp": time.time(),
        }
        self.request_pub.publish(_json_msg(payload))
        self.phase = self.PHASE_WAIT_ACK
        self.phase_started_at = now
        self.phase_deadline = now + self.request_ack_timeout_s
        self._event(
            "REQUEST",
            "已请求第三视角实时提取候选、Qwen 选点并导航",
            request_id=self.active_request_id,
            reason=self.active_trigger_reason,
        )

    def _begin_return(
        self,
        reason: str,
        *,
        cancel_backend: bool,
        backend_reason: str = "",
    ) -> None:
        if self.phase in {self.PHASE_EGO, self.PHASE_RETURN_HOLD}:
            return
        now = time.monotonic()
        request_id = self.active_request_id
        self._publish_mode("HOLD", f"return:{reason}")
        if cancel_backend and request_id:
            self.cancel_pub.publish(
                _json_msg(
                    {
                        "protocol_version": 1,
                        "request_id": request_id,
                        "reason": reason,
                    }
                )
            )
        self._publish_qwen(self.resume_qwen_command)
        self.phase = self.PHASE_RETURN_HOLD
        self.phase_started_at = now
        self.phase_deadline = now + self.resume_hold_s
        self._event(
            "MAP_EXIT",
            f"第三视角结束 → HOLD，原因={reason}",
            request_id=request_id,
            backend_reason=backend_reason,
        )

    def _finish_return(self, now: float) -> None:
        completed_request = self.active_request_id
        self.phase = self.PHASE_EGO
        self.phase_started_at = now
        self.phase_deadline = float("inf")
        self.active_request_id = None
        self.active_trigger_reason = None
        self.active_evidence = {}
        self.map_enter_at = None
        self._reset_detection_windows("map_return")
        self.stuck_cooldown_until = now + self.stuck_cooldown_after_map_s
        self.revisit_cooldown_until = now + self.revisit_cooldown_after_map_s
        self.revisit_epoch_distance = self.path_distance_accum
        self.revisit_rearm_distance = (
            self.path_distance_accum + self.revisit_rearm_travel_m
        )
        self._publish_mode("EGO", "map_return_complete")
        self._publish_qwen(self.resume_qwen_command)
        self._event(
            "EGO_RESUME",
            "控制权已返回第一视角；保留历史覆盖，重置近期证据",
            request_id=completed_request,
            revisit_rearm_distance=round(self.revisit_rearm_distance, 3),
        )

    def _reset_detection_windows(self, reason: str) -> None:
        self.commands.clear()
        self.odom_poses.clear()
        self.last_progress_metrics = None
        self.revisit_hits = 0
        self.fallback_no_progress_hits = 0
        self.last_fallback_hit_at = float("-inf")
        self.last_novelty_eval_distance = self.path_distance_accum
        self.last_recovery_completed_at = None
        self.get_logger().info(f"detection windows reset: {reason}")

    def _publish_mode(self, mode: str, reason: str) -> None:
        self.mode_pub.publish(
            _json_msg(
                {
                    "mode": str(mode).upper(),
                    "reason": str(reason),
                    "request_id": self.active_request_id,
                    "timestamp": time.time(),
                }
            )
        )

    def _publish_qwen(self, command: str) -> None:
        self.qwen_pub.publish(String(data=str(command)))

    def _event(self, kind: str, message: str, **fields: Any) -> None:
        self.last_event_message = str(message)
        payload: Dict[str, Any] = {
            "event": str(kind).upper(),
            "message": str(message),
            "phase": self.phase,
            "request_id": self.active_request_id,
            "timestamp": time.time(),
        }
        payload.update(fields)
        self.event_pub.publish(_json_msg(payload))
        self.get_logger().warning(f"[{payload['event']}] {message}")

    def _progress_status(self) -> Optional[Dict[str, Any]]:
        return (
            None
            if self.last_progress_metrics is None
            else self._progress_evidence(self.last_progress_metrics)
        )

    def _coverage_status(self) -> Optional[Dict[str, Any]]:
        m = self.last_coverage_metrics
        if m is None:
            return None
        return {
            "available": m.available,
            "reason": m.reason,
            "total_path_m": round(m.total_path_m, 3),
            "recent_path_m": round(m.recent_path_m, 3),
            "history_path_m": round(m.history_path_m, 3),
            "recent_valid_cells": m.recent_valid_cells,
            "new_cells": m.new_cells,
            "overlap_cells": m.overlap_cells,
            "new_area_ratio": (
                None if m.new_area_ratio is None else round(m.new_area_ratio, 4)
            ),
            "low_novelty": m.low_novelty,
            "consecutive_hits": self.revisit_hits,
            "required_hits": self.revisit_required_hits,
        }

    def _publish_status(self) -> None:
        now = time.monotonic()
        requested_mode = (
            "EGO" if self.phase == self.PHASE_EGO
            else "MAP" if self.phase == self.PHASE_MAP
            else "HOLD"
        )
        # cmd_vel_intervention_mux fails closed when mode messages become stale.
        # Re-publish at status_hz so a healthy long EGO/MAP phase never expires.
        self._publish_mode(requested_mode, "heartbeat")
        payload = {
            "enabled": True,
            "phase": self.phase,
            "request_id": self.active_request_id,
            "task": self.task,
            "eligible": self.last_eligibility_reason == "eligible",
            "eligibility_reason": self.last_eligibility_reason,
            "target_visible": self._target_visible(),
            "recovery_active": self.recovery_active,
            "last_trigger": self.last_trigger_reason,
            "last_event": self.last_event_message,
            "path_distance_accum_m": round(self.path_distance_accum, 3),
            "progress": self._progress_status(),
            "coverage": self._coverage_status(),
            "ages_s": {
                "servo": None
                if self.latest_servo_at is None
                else round(now - self.latest_servo_at, 3),
                "odom": None
                if self.latest_odom_at is None
                else round(now - self.latest_odom_at, 3),
                "map": None
                if self.latest_map_at is None
                else round(now - self.latest_map_at, 3),
                "tf": None
                if self.latest_tf_at is None
                else round(now - self.latest_tf_at, 3),
            },
            "timestamp": time.time(),
        }
        msg = _json_msg(payload)
        self.status_pub.publish(msg)
        self.intervention_status_pub.publish(msg)

    def _log_metrics(self) -> None:
        p = self.last_progress_metrics
        c = self.last_coverage_metrics
        p_text = "A=waiting"
        if p is not None:
            p_text = (
                f"A cmd={p.commanded_linear_m:.2f}m/"
                f"{p.commanded_yaw_rad:.2f}rad actual={p.actual_path_m:.2f}m/"
                f"{p.actual_yaw_rad:.2f}rad fail={int(p.failed)}"
            )
        c_text = "B=waiting"
        if c is not None:
            ratio = "-" if c.new_area_ratio is None else f"{c.new_area_ratio:.1%}"
            c_text = (
                f"B new={ratio} hits={self.revisit_hits}/{self.revisit_required_hits} "
                f"cells={c.recent_valid_cells} reason={c.reason}"
            )
        self.get_logger().info(
            f"[METRICS] phase={self.phase} eligible={self.last_eligibility_reason} "
            f"{p_text} | {c_text}"
        )

    def _path_message(self, points: Sequence[Tuple[float, float]]) -> NavPath:
        msg = NavPath()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _publish_visualization(self) -> None:
        if not self.visualization_enabled:
            return
        now_msg = self.get_clock().now().to_msg()
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        metrics = self.last_coverage_metrics
        history_points: Sequence[Tuple[float, float]] = ()
        recent_points: Sequence[Tuple[float, float]] = ()
        if metrics is not None:
            history_points = metrics.history_points
            recent_points = metrics.recent_points

        def line_marker(
            marker_id: int,
            ns: str,
            points: Sequence[Tuple[float, float]],
            z: float,
            rgba: Tuple[float, float, float, float],
        ) -> Marker:
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now_msg
            m.ns = ns
            m.id = marker_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = self.line_width_m
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y in points]
            return m

        array.markers.append(
            line_marker(10, "history_path", history_points, self.history_path_z_m, (0.25, 0.75, 0.95, 0.65))
        )
        array.markers.append(
            line_marker(11, "recent_path", recent_points, self.recent_path_z_m, (0.95, 0.85, 0.20, 0.95))
        )
        self.history_path_pub.publish(self._path_message(history_points))
        self.recent_path_pub.publish(self._path_message(recent_points))

        if self.latest_map_pose is not None:
            x, y, _ = self.latest_map_pose
            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now_msg
            text.ns = "handoff_text"
            text.id = 20
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.42
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_height_m
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            novelty = "NEW --"
            if metrics is not None and metrics.new_area_ratio is not None:
                novelty = f"NEW {metrics.new_area_ratio:.0%} ({self.revisit_hits}/{self.revisit_required_hits})"
            stuck = "A --"
            if self.last_progress_metrics is not None:
                stuck = (
                    f"A cmd {self.last_progress_metrics.commanded_linear_m:.2f}m "
                    f"act {self.last_progress_metrics.actual_path_m:.2f}m"
                )
            text.text = f"{self.phase} | {stuck} | {novelty}"
            array.markers.append(text)

        if metrics is not None and self.map_msg is not None:
            try:
                _, spec = self._grid_from_map()
                marker_id = 100
                for ns, mask, rgba in (
                    ("new_cells", metrics.new_mask, (0.10, 0.95, 0.25, 0.65)),
                    ("overlap_cells", metrics.overlap_mask, (1.00, 0.45, 0.05, 0.70)),
                ):
                    if mask is None:
                        continue
                    indices = np.argwhere(mask)
                    if len(indices) > self.max_cell_markers:
                        stride = int(math.ceil(len(indices) / self.max_cell_markers))
                        indices = indices[::stride]
                    cube_list = Marker()
                    cube_list.header.frame_id = self.map_frame
                    cube_list.header.stamp = now_msg
                    cube_list.ns = ns
                    cube_list.id = marker_id
                    marker_id += 1
                    cube_list.type = Marker.CUBE_LIST
                    cube_list.action = Marker.ADD
                    cube_list.pose.orientation.w = 1.0
                    cube_list.scale.x = max(self.cell_size_m, spec.resolution)
                    cube_list.scale.y = max(self.cell_size_m, spec.resolution)
                    cube_list.scale.z = 0.015
                    (
                        cube_list.color.r,
                        cube_list.color.g,
                        cube_list.color.b,
                        cube_list.color.a,
                    ) = rgba
                    cube_list.points = [
                        Point(
                            x=grid_to_world(float(col), float(row), spec)[0],
                            y=grid_to_world(float(col), float(row), spec)[1],
                            z=0.015,
                        )
                        for row, col in indices
                    ]
                    array.markers.append(cube_list)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"visualization mask skipped: {exc}")

        self.marker_pub.publish(array)

    def stop(self) -> None:
        self._publish_mode("HOLD", "supervisor_shutdown")
        if self.active_request_id:
            self.cancel_pub.publish(
                _json_msg(
                    {
                        "protocol_version": 1,
                        "request_id": self.active_request_id,
                        "reason": "supervisor_shutdown",
                    }
                )
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--task", default="")
    args = parser.parse_args()
    rclpy.init()
    node: Optional[SimpleHandoffSupervisor] = None
    try:
        node = SimpleHandoffSupervisor(args.config, args.task)
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
