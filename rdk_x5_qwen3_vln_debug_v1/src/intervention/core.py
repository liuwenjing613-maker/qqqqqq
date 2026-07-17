"""Pure decision logic for third-view intervention.

This module deliberately contains no ROS imports.  The policy can therefore be
unit-tested and replayed with recorded JSON/odom samples before it is allowed to
switch a real robot's control source.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple


class DecisionKind(str, Enum):
    KEEP_EGO = "KEEP_EGO"
    RECOVERY = "RECOVERY"
    MAP_DIRECT = "MAP_DIRECT"
    MAP_QWEN = "MAP_QWEN"


@dataclass(frozen=True)
class Decision:
    kind: DecisionKind
    reason: str
    evidence: Dict[str, Any] = field(default_factory=dict)
    candidate_ids: Tuple[str, ...] = ()


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ServoSample:
    stamp: float
    state: str = "WAIT_IMAGE"
    result: str = ""
    action: str = "STOP"
    request_id: int = -1
    point_role: str = "none"
    horizontal_error: float = 0.0
    limited_vx: float = 0.0
    limited_wz: float = 0.0
    spawn_scan_phase: str = "IDLE"
    view_adjust_phase: str = "IDLE"
    emergency_reverse_active: bool = False
    turn_pending_action: Optional[str] = None
    mission_success: bool = False
    motion_enabled: bool = False

    @property
    def target_visible(self) -> bool:
        return (
            self.result.strip().upper() == "TARGET_VISIBLE"
            or self.point_role.strip().lower() == "target"
        )


@dataclass(frozen=True)
class Candidate:
    candidate_id: str
    heading_deg: float
    score: Optional[float] = None
    reachable: bool = True
    visited: bool = False
    status: str = "UNSEEN"
    path_length: Optional[float] = None

    @property
    def eligible(self) -> bool:
        blocked = {"BLOCKED", "FAILED", "EXHAUSTED", "BLACKLISTED"}
        return self.reachable and not self.visited and self.status.upper() not in blocked


@dataclass(frozen=True)
class CandidateSummary:
    stamp: float
    generation: int
    candidates: Tuple[Candidate, ...] = ()
    map_version: Optional[str] = None
    decision_distance_m: Optional[float] = None
    returned_to_junction: bool = False
    unseen_candidate_count: Optional[int] = None
    junction_id: Optional[str] = None

    @property
    def eligible_candidates(self) -> Tuple[Candidate, ...]:
        return tuple(c for c in self.candidates if c.eligible)


@dataclass(frozen=True)
class InterventionConfig:
    enabled: bool = False
    evaluation_hz: float = 10.0
    startup_grace_sec: float = 5.0
    cooldown_sec: float = 12.0
    servo_max_age_sec: float = 0.6
    odom_max_age_sec: float = 0.6
    candidates_max_age_sec: float = 2.5
    require_motion_enabled: bool = True

    blocked_states: Tuple[str, ...] = (
        "WAIT_IMAGE",
        "SPAWN_SCAN",
        "PAUSED",
        "SUCCESS",
        "ERROR",
    )
    active_spawn_phases: Tuple[str, ...] = (
        "TURNING",
        "SETTLING",
        "DWELLING",
        "WAIT_RESULT",
        "WAITING_RESULT",
        "RETURNING",
    )
    active_view_phases: Tuple[str, ...] = (
        "PRE_STOP",
        "TURNING",
        "SETTLING",
    )

    branch_min_candidates: int = 2
    branch_require_decision_distance: bool = True
    branch_min_heading_separation_deg: float = 50.0
    branch_decision_radius_m: float = 1.30
    branch_max_top_score_gap: float = 0.12
    branch_max_top_score_ratio: float = 1.25
    branch_persistence_updates: int = 2

    returned_junction_persistence_updates: int = 2

    progress_window_sec: float = 6.0
    progress_min_cmd_vx: float = 0.025
    progress_min_cmd_active_ratio: float = 0.55
    progress_min_integrated_cmd_distance_m: float = 0.14
    progress_max_actual_displacement_m: float = 0.08
    progress_recovery_grace_sec: float = 4.0
    progress_recovery_success_distance_m: float = 0.18

    oscillation_result_window: int = 8
    oscillation_min_turn_direction_flips: int = 3
    oscillation_min_horizontal_sign_flips: int = 4
    oscillation_horizontal_deadband: float = 0.15
    oscillation_max_displacement_m: float = 0.22
    oscillation_min_span_sec: float = 4.0

    emergency_reverse_window_sec: float = 15.0
    emergency_reverse_trigger_count: int = 2

    allow_unknown_candidates_for_failure: bool = True

    history_sec: float = 40.0

    @classmethod
    def from_mapping(cls, raw: Dict[str, Any]) -> "InterventionConfig":
        raw = raw or {}
        freshness = raw.get("freshness", {}) or {}
        guard = raw.get("guard", {}) or {}
        branch = raw.get("branch", {}) or {}
        progress = raw.get("progress", {}) or {}
        oscillation = raw.get("oscillation", {}) or {}
        reverse = raw.get("emergency_reverse", {}) or {}

        def _tuple_str(value: Any, default: Sequence[str]) -> Tuple[str, ...]:
            seq = value if isinstance(value, (list, tuple)) else default
            return tuple(str(v).strip().upper() for v in seq if str(v).strip())

        return cls(
            enabled=bool(raw.get("enabled", False)),
            evaluation_hz=float(raw.get("evaluation_hz", 10.0)),
            startup_grace_sec=float(guard.get("startup_grace_sec", 5.0)),
            cooldown_sec=float(guard.get("cooldown_sec", 12.0)),
            servo_max_age_sec=float(freshness.get("servo_max_age_sec", 0.6)),
            odom_max_age_sec=float(freshness.get("odom_max_age_sec", 0.6)),
            candidates_max_age_sec=float(
                freshness.get("candidates_max_age_sec", 2.5)
            ),
            require_motion_enabled=bool(
                guard.get("require_motion_enabled", True)
            ),
            blocked_states=_tuple_str(
                guard.get("blocked_states"), cls.blocked_states
            ),
            active_spawn_phases=_tuple_str(
                guard.get("active_spawn_phases"), cls.active_spawn_phases
            ),
            active_view_phases=_tuple_str(
                guard.get("active_view_phases"), cls.active_view_phases
            ),
            branch_min_candidates=int(branch.get("min_candidates", 2)),
            branch_require_decision_distance=bool(
                branch.get("require_decision_distance", True)
            ),
            branch_min_heading_separation_deg=float(
                branch.get("min_heading_separation_deg", 50.0)
            ),
            branch_decision_radius_m=float(
                branch.get("decision_radius_m", 1.30)
            ),
            branch_max_top_score_gap=float(
                branch.get("max_top_score_gap", 0.12)
            ),
            branch_max_top_score_ratio=float(
                branch.get("max_top_score_ratio", 1.25)
            ),
            branch_persistence_updates=int(
                branch.get("persistence_updates", 2)
            ),
            returned_junction_persistence_updates=int(
                branch.get("returned_junction_persistence_updates", 2)
            ),
            progress_window_sec=float(progress.get("window_sec", 6.0)),
            progress_min_cmd_vx=float(progress.get("min_cmd_vx", 0.025)),
            progress_min_cmd_active_ratio=float(
                progress.get("min_cmd_active_ratio", 0.55)
            ),
            progress_min_integrated_cmd_distance_m=float(
                progress.get("min_integrated_cmd_distance_m", 0.14)
            ),
            progress_max_actual_displacement_m=float(
                progress.get("max_actual_displacement_m", 0.08)
            ),
            progress_recovery_grace_sec=float(
                progress.get("recovery_grace_sec", 4.0)
            ),
            progress_recovery_success_distance_m=float(
                progress.get("recovery_success_distance_m", 0.18)
            ),
            oscillation_result_window=int(
                oscillation.get("result_window", 8)
            ),
            oscillation_min_turn_direction_flips=int(
                oscillation.get("min_turn_direction_flips", 3)
            ),
            oscillation_min_horizontal_sign_flips=int(
                oscillation.get("min_horizontal_sign_flips", 4)
            ),
            oscillation_horizontal_deadband=float(
                oscillation.get("horizontal_deadband", 0.15)
            ),
            oscillation_max_displacement_m=float(
                oscillation.get("max_displacement_m", 0.22)
            ),
            oscillation_min_span_sec=float(
                oscillation.get("min_span_sec", 4.0)
            ),
            emergency_reverse_window_sec=float(
                reverse.get("window_sec", 15.0)
            ),
            emergency_reverse_trigger_count=int(
                reverse.get("trigger_count", 2)
            ),
            allow_unknown_candidates_for_failure=bool(
                raw.get("allow_unknown_candidates_for_failure", True)
            ),
            history_sec=float(raw.get("history_sec", 40.0)),
        )


class InterventionCore:
    """Stateful, deterministic intervention policy."""

    def __init__(self, cfg: InterventionConfig, start_stamp: float = 0.0):
        self.cfg = cfg
        self.start_stamp = float(start_stamp)
        self.cooldown_until = float("-inf")

        self.latest_servo: Optional[ServoSample] = None
        self.latest_pose: Optional[PoseSample] = None
        self.latest_candidates: Optional[CandidateSummary] = None

        self.servo_history: Deque[ServoSample] = deque()
        self.pose_history: Deque[PoseSample] = deque()
        self.result_history: Deque[ServoSample] = deque()
        self.reverse_edges: Deque[float] = deque()

        self._last_result_request_id = -1
        self._last_reverse_active = False
        self._last_candidate_generation: Optional[int] = None
        self._branch_confirmations = 0
        self._junction_confirmations = 0

        self._recovery_stage = "IDLE"
        self._recovery_stage_started = float("-inf")
        self._recovery_pose_start: Optional[PoseSample] = None
        self._recovery_notice_emitted = False

    def mark_intervention_finished(self, now: float) -> None:
        self.cooldown_until = max(self.cooldown_until, now + self.cfg.cooldown_sec)
        self._reset_recovery_stage()
        self._branch_confirmations = 0
        self._junction_confirmations = 0
        # Do not let pre-intervention oscillation/reverse evidence fire again
        # after cooldown. Keep only the latest continuous samples as a new base.
        self.result_history.clear()
        self.reverse_edges.clear()
        self.servo_history.clear()
        self.pose_history.clear()
        if self.latest_servo is not None:
            self.servo_history.append(self.latest_servo)
            self._last_reverse_active = self.latest_servo.emergency_reverse_active
        if self.latest_pose is not None:
            self.pose_history.append(self.latest_pose)

    def update_pose(self, sample: PoseSample) -> None:
        self.latest_pose = sample
        self.pose_history.append(sample)
        self._trim_histories(sample.stamp)

    def update_servo(self, sample: ServoSample) -> None:
        self.latest_servo = sample
        self.servo_history.append(sample)

        if sample.request_id > self._last_result_request_id:
            self._last_result_request_id = sample.request_id
            self.result_history.append(sample)

        if sample.emergency_reverse_active and not self._last_reverse_active:
            self.reverse_edges.append(sample.stamp)
        self._last_reverse_active = sample.emergency_reverse_active
        self._trim_histories(sample.stamp)

    def update_candidates(self, summary: CandidateSummary) -> None:
        self.latest_candidates = summary
        if summary.generation != self._last_candidate_generation:
            self._last_candidate_generation = summary.generation
            self._branch_confirmations = (
                self._branch_confirmations + 1
                if self._branch_ambiguous_raw(summary)
                else 0
            )
            self._junction_confirmations = (
                self._junction_confirmations + 1
                if summary.returned_to_junction
                else 0
            )

    def evaluate(self, now: float, external_busy: bool = False) -> Decision:
        guard = self._guard_reason(now, external_busy=external_busy)
        if guard is not None:
            return Decision(DecisionKind.KEEP_EGO, guard, self._basic_evidence(now))

        assert self.latest_servo is not None

        # A target in the current first-person result always wins.
        if self.latest_servo.target_visible:
            self._reset_recovery_stage()
            return Decision(
                DecisionKind.KEEP_EGO,
                "TARGET_VISIBLE",
                self._basic_evidence(now),
            )

        junction = self._returned_junction_decision(now)
        if junction is not None:
            return junction

        branch = self._branch_decision(now)
        if branch is not None:
            return branch

        progress = self._progress_decision(now)
        if progress is not None:
            return progress

        oscillation = self._oscillation_metrics(now)
        if oscillation["triggered"]:
            return self._candidate_context_decision(
                "ACTION_OSCILLATION",
                now,
                extra=oscillation,
            )

        reverse = self._reverse_metrics(now)
        if reverse["triggered"]:
            return self._candidate_context_decision(
                "REPEATED_EMERGENCY_REVERSE",
                now,
                extra=reverse,
            )

        return Decision(
            DecisionKind.KEEP_EGO,
            "EGO_HEALTHY",
            self._basic_evidence(now),
        )

    def branch_condition_report(self, now: float) -> Optional[Dict[str, Any]]:
        """Structured checklist for BRANCH_AMBIGUOUS (for flow logging)."""
        summary = self._fresh_candidate_summary(now)
        need = self.cfg.branch_persistence_updates
        if summary is None:
            return {
                "track": "BRANCH",
                "fresh": False,
                "persistence": self._branch_confirmations,
                "persistence_need": need,
                "ready": False,
                "checks": [
                    {
                        "name": "候选摘要新鲜",
                        "ok": False,
                        "detail": f"缺失或超过{self.cfg.candidates_max_age_sec:.1f}s",
                    }
                ],
            }

        eligible = summary.eligible_candidates
        count_ok = len(eligible) >= self.cfg.branch_min_candidates
        dist = summary.decision_distance_m
        if self.cfg.branch_require_decision_distance and dist is None:
            dist_present_ok = False
            dist_ok = False
            dist_detail = "无decision_distance"
        else:
            dist_present_ok = True
            if dist is None:
                dist_ok = True
                dist_detail = "不要求距离"
            else:
                dist_ok = dist <= self.cfg.branch_decision_radius_m
                dist_detail = f"{dist:.2f}≤{self.cfg.branch_decision_radius_m:.2f}m"

        max_sep = self._max_heading_separation_deg([c.heading_deg for c in eligible])
        sep_ok = max_sep >= self.cfg.branch_min_heading_separation_deg
        scored = sorted(
            (c.score for c in eligible if c.score is not None), reverse=True
        )
        gap: Optional[float] = None
        ratio: Optional[float] = None
        if len(scored) < 2:
            score_ok = count_ok
            score_detail = "分值不足2个→视为分歧"
        else:
            gap = scored[0] - scored[1]
            gap_ok = gap <= self.cfg.branch_max_top_score_gap
            if scored[1] <= 1e-9:
                ratio_ok = gap_ok
            else:
                ratio = scored[0] / scored[1]
                ratio_ok = ratio <= self.cfg.branch_max_top_score_ratio
            score_ok = gap_ok or ratio_ok
            score_detail = (
                f"gap={gap:.3f}≤{self.cfg.branch_max_top_score_gap:.2f}"
                f" 或 ratio="
                f"{'n/a' if ratio is None else f'{ratio:.2f}'}"
                f"≤{self.cfg.branch_max_top_score_ratio:.2f}"
            )

        raw_ok = self._branch_ambiguous_raw(summary)
        persist_ok = self._branch_confirmations >= need
        checks = [
            {
                "name": "候选摘要新鲜",
                "ok": True,
                "detail": f"age={now - summary.stamp:.2f}s",
            },
            {
                "name": f"合格候选≥{self.cfg.branch_min_candidates}",
                "ok": count_ok,
                "detail": f"{len(eligible)}/{self.cfg.branch_min_candidates}",
            },
            {
                "name": "决策距离可用",
                "ok": dist_present_ok,
                "detail": dist_detail if not dist_present_ok else "ok",
            },
            {
                "name": f"进入决策半径",
                "ok": dist_ok,
                "detail": dist_detail,
            },
            {
                "name": f"航向夹角≥{self.cfg.branch_min_heading_separation_deg:.0f}°",
                "ok": sep_ok,
                "detail": f"{max_sep:.1f}°",
            },
            {
                "name": "分数接近(分歧)",
                "ok": score_ok,
                "detail": score_detail,
            },
            {
                "name": f"连续确认≥{need}",
                "ok": persist_ok,
                "detail": f"{self._branch_confirmations}/{need}",
            },
        ]
        met = sum(1 for c in checks if c["ok"])
        return {
            "track": "BRANCH",
            "fresh": True,
            "raw_ambiguous": raw_ok,
            "persistence": self._branch_confirmations,
            "persistence_need": need,
            "ready": raw_ok and persist_ok,
            "met": met,
            "total": len(checks),
            "checks": checks,
            "candidate_ids": [c.candidate_id for c in eligible],
            "gap": gap,
            "ratio": ratio,
            "decision_distance_m": dist,
            "max_heading_separation_deg": max_sep,
        }

    def junction_condition_report(self, now: float) -> Optional[Dict[str, Any]]:
        """Structured checklist for returned-junction triggers."""
        summary = self._fresh_candidate_summary(now)
        need = self.cfg.returned_junction_persistence_updates
        if summary is None:
            return {
                "track": "JUNCTION",
                "fresh": False,
                "persistence": self._junction_confirmations,
                "persistence_need": need,
                "ready": False,
                "checks": [
                    {
                        "name": "候选摘要新鲜",
                        "ok": False,
                        "detail": "缺失或过期",
                    }
                ],
            }
        flag_ok = bool(summary.returned_to_junction)
        persist_ok = self._junction_confirmations >= need
        unseen = (
            summary.unseen_candidate_count
            if summary.unseen_candidate_count is not None
            else len(summary.eligible_candidates)
        )
        checks = [
            {
                "name": "返回路口标记",
                "ok": flag_ok,
                "detail": str(bool(summary.returned_to_junction)),
            },
            {
                "name": f"连续确认≥{need}",
                "ok": persist_ok,
                "detail": f"{self._junction_confirmations}/{need}",
            },
            {
                "name": "未访问候选",
                "ok": True,
                "detail": f"unseen={unseen} eligible={len(summary.eligible_candidates)}",
            },
        ]
        met = sum(1 for c in checks if c["ok"])
        return {
            "track": "JUNCTION",
            "fresh": True,
            "persistence": self._junction_confirmations,
            "persistence_need": need,
            "ready": flag_ok and persist_ok,
            "met": met,
            "total": len(checks),
            "checks": checks,
            "unseen": unseen,
            "eligible": len(summary.eligible_candidates),
            "candidate_ids": [c.candidate_id for c in summary.eligible_candidates],
        }

    def progress_recovery_report(self, now: float) -> Dict[str, Any]:
        return {
            "track": "PROGRESS",
            "stage": self._recovery_stage,
            "stage_age_sec": (
                None
                if self._recovery_stage == "IDLE"
                else round(now - self._recovery_stage_started, 2)
            ),
        }

    def _guard_reason(self, now: float, external_busy: bool) -> Optional[str]:
        if not self.cfg.enabled:
            return "DISABLED"
        if now - self.start_stamp < self.cfg.startup_grace_sec:
            return "STARTUP_GRACE"
        if external_busy:
            return "EXTERNAL_BUSY"
        if now < self.cooldown_until:
            return "COOLDOWN"
        if self.latest_servo is None:
            return "NO_SERVO_STATUS"
        if now - self.latest_servo.stamp > self.cfg.servo_max_age_sec:
            return "SERVO_STATUS_STALE"
        if self.latest_pose is None:
            return "NO_ODOM"
        if now - self.latest_pose.stamp > self.cfg.odom_max_age_sec:
            return "ODOM_STALE"

        if self.cfg.require_motion_enabled and not self.latest_servo.motion_enabled:
            return "MOTION_DISABLED"

        state = self.latest_servo.state.strip().upper()
        if state in self.cfg.blocked_states:
            return f"STATE_{state}"
        if self.latest_servo.mission_success:
            return "MISSION_SUCCESS"
        if self.latest_servo.emergency_reverse_active:
            return "EMERGENCY_REVERSE_ACTIVE"
        if self.latest_servo.turn_pending_action:
            return "TURN_PENDING"

        spawn_phase = self.latest_servo.spawn_scan_phase.strip().upper()
        if spawn_phase in self.cfg.active_spawn_phases:
            return f"SPAWN_{spawn_phase}"
        view_phase = self.latest_servo.view_adjust_phase.strip().upper()
        if view_phase in self.cfg.active_view_phases:
            return f"VIEW_{view_phase}"
        return None

    def _returned_junction_decision(self, now: float) -> Optional[Decision]:
        summary = self._fresh_candidate_summary(now)
        if summary is None:
            return None
        if (
            self._junction_confirmations
            < self.cfg.returned_junction_persistence_updates
        ):
            return None
        unseen = (
            summary.unseen_candidate_count
            if summary.unseen_candidate_count is not None
            else len(summary.eligible_candidates)
        )
        if unseen <= 0:
            return Decision(
                DecisionKind.RECOVERY,
                "RETURNED_JUNCTION_EMPTY",
                self._candidate_evidence(now, summary),
            )
        eligible = summary.eligible_candidates
        ids = tuple(c.candidate_id for c in eligible)
        if len(eligible) == 1:
            return Decision(
                DecisionKind.MAP_DIRECT,
                "RETURNED_JUNCTION_SINGLE",
                self._candidate_evidence(now, summary),
                ids[:1],
            )
        return Decision(
            DecisionKind.MAP_QWEN,
            "RETURNED_JUNCTION_MULTI",
            self._candidate_evidence(now, summary),
            ids,
        )

    def _branch_decision(self, now: float) -> Optional[Decision]:
        summary = self._fresh_candidate_summary(now)
        if summary is None:
            return None
        if self._branch_confirmations < self.cfg.branch_persistence_updates:
            return None
        if not self._branch_ambiguous_raw(summary):
            return None
        eligible = summary.eligible_candidates
        return Decision(
            DecisionKind.MAP_QWEN,
            "BRANCH_AMBIGUOUS",
            self._candidate_evidence(now, summary),
            tuple(c.candidate_id for c in eligible),
        )

    def _branch_ambiguous_raw(self, summary: CandidateSummary) -> bool:
        eligible = summary.eligible_candidates
        if len(eligible) < self.cfg.branch_min_candidates:
            return False
        if (
            self.cfg.branch_require_decision_distance
            and summary.decision_distance_m is None
        ):
            return False
        if (
            summary.decision_distance_m is not None
            and summary.decision_distance_m > self.cfg.branch_decision_radius_m
        ):
            return False

        max_sep = self._max_heading_separation_deg(
            [c.heading_deg for c in eligible]
        )
        if max_sep < self.cfg.branch_min_heading_separation_deg:
            return False

        scored = sorted(
            (c.score for c in eligible if c.score is not None), reverse=True
        )
        if len(scored) < 2:
            return True
        top1, top2 = scored[0], scored[1]
        gap_ok = (top1 - top2) <= self.cfg.branch_max_top_score_gap
        if top2 <= 1e-9:
            ratio_ok = gap_ok
        else:
            ratio_ok = (top1 / top2) <= self.cfg.branch_max_top_score_ratio
        return gap_ok or ratio_ok

    def _progress_decision(self, now: float) -> Optional[Decision]:
        if self._recovery_stage == "IDLE":
            metrics = self._progress_metrics(now, self.cfg.progress_window_sec)
            if metrics["triggered"]:
                self._recovery_stage = "GRACE"
                self._recovery_stage_started = now
                self._recovery_pose_start = self.latest_pose
                self._recovery_notice_emitted = True
                return Decision(
                    DecisionKind.RECOVERY,
                    "NO_PROGRESS_LOCAL_RECOVERY",
                    {**self._basic_evidence(now), **metrics},
                )
            return None

        assert self._recovery_pose_start is not None
        recovered_distance = self._distance_between(
            self._recovery_pose_start, self.latest_pose
        )
        if recovered_distance >= self.cfg.progress_recovery_success_distance_m:
            self._reset_recovery_stage()
            return None

        if self._recovery_stage == "GRACE":
            if now - self._recovery_stage_started < self.cfg.progress_recovery_grace_sec:
                return None
            self._recovery_stage = "VERIFY"
            self._recovery_stage_started = now
            return None

        # VERIFY: require an entirely new progress window after the grace period.
        if now - self._recovery_stage_started < self.cfg.progress_window_sec:
            return None
        metrics = self._progress_metrics_since(self._recovery_stage_started, now)
        if metrics["triggered"]:
            decision = self._candidate_context_decision(
                "NO_PROGRESS_AFTER_RECOVERY",
                now,
                extra=metrics,
            )
            self._reset_recovery_stage()
            return decision
        self._reset_recovery_stage()
        return None

    def _progress_metrics(self, now: float, window_sec: float) -> Dict[str, Any]:
        return self._progress_metrics_since(now - window_sec, now)

    def _progress_metrics_since(self, start: float, end: float) -> Dict[str, Any]:
        samples = [s for s in self.servo_history if start <= s.stamp <= end]
        poses = [p for p in self.pose_history if start <= p.stamp <= end]
        actual_span = 0.0
        active_time = 0.0
        integrated_cmd = 0.0

        for left, right in zip(samples, samples[1:]):
            dt = max(0.0, min(right.stamp, end) - max(left.stamp, start))
            if dt <= 0.0:
                continue
            actual_span += dt
            vx = max(0.0, left.limited_vx)
            integrated_cmd += vx * dt
            if vx >= self.cfg.progress_min_cmd_vx:
                active_time += dt

        active_ratio = active_time / actual_span if actual_span > 1e-6 else 0.0
        displacement = (
            self._distance_between(poses[0], poses[-1]) if len(poses) >= 2 else 0.0
        )
        triggered = (
            actual_span >= 0.8 * self.cfg.progress_window_sec
            and active_ratio >= self.cfg.progress_min_cmd_active_ratio
            and integrated_cmd
            >= self.cfg.progress_min_integrated_cmd_distance_m
            and displacement <= self.cfg.progress_max_actual_displacement_m
        )
        return {
            "triggered": triggered,
            "progress_span_sec": round(actual_span, 3),
            "cmd_active_ratio": round(active_ratio, 3),
            "integrated_cmd_distance_m": round(integrated_cmd, 3),
            "actual_displacement_m": round(displacement, 3),
        }

    def _oscillation_metrics(self, now: float) -> Dict[str, Any]:
        items = list(self.result_history)[-self.cfg.oscillation_result_window :]
        if len(items) < self.cfg.oscillation_result_window:
            return {
                "triggered": False,
                "turn_direction_flips": 0,
                "horizontal_sign_flips": 0,
            }
        span = items[-1].stamp - items[0].stamp
        if span < self.cfg.oscillation_min_span_sec:
            return {
                "triggered": False,
                "turn_direction_flips": 0,
                "horizontal_sign_flips": 0,
                "oscillation_span_sec": round(span, 3),
            }

        turn_dirs = [
            a.action
            for a in items
            if a.action in {"TURN_LEFT", "TURN_RIGHT"}
        ]
        turn_flips = self._count_changes(turn_dirs)

        signs: List[int] = []
        deadband = self.cfg.oscillation_horizontal_deadband
        for item in items:
            if item.horizontal_error > deadband:
                signs.append(1)
            elif item.horizontal_error < -deadband:
                signs.append(-1)
        sign_flips = self._count_changes(signs)

        displacement = self._pose_displacement(items[0].stamp, items[-1].stamp)
        triggered = (
            displacement <= self.cfg.oscillation_max_displacement_m
            and (
                turn_flips >= self.cfg.oscillation_min_turn_direction_flips
                or sign_flips >= self.cfg.oscillation_min_horizontal_sign_flips
            )
        )
        return {
            "triggered": triggered,
            "turn_direction_flips": turn_flips,
            "horizontal_sign_flips": sign_flips,
            "oscillation_displacement_m": round(displacement, 3),
            "oscillation_span_sec": round(span, 3),
            "result_window": len(items),
        }

    def _reverse_metrics(self, now: float) -> Dict[str, Any]:
        cutoff = now - self.cfg.emergency_reverse_window_sec
        while self.reverse_edges and self.reverse_edges[0] < cutoff:
            self.reverse_edges.popleft()
        count = len(self.reverse_edges)
        return {
            "triggered": count >= self.cfg.emergency_reverse_trigger_count,
            "emergency_reverse_count": count,
            "emergency_reverse_window_sec": self.cfg.emergency_reverse_window_sec,
        }

    def _candidate_context_decision(
        self, reason: str, now: float, extra: Optional[Dict[str, Any]] = None
    ) -> Decision:
        summary = self._fresh_candidate_summary(now)
        evidence = self._basic_evidence(now)
        if extra:
            evidence.update(extra)

        if summary is None:
            evidence["candidate_summary"] = "missing_or_stale"
            if self.cfg.allow_unknown_candidates_for_failure:
                return Decision(DecisionKind.MAP_QWEN, reason, evidence)
            return Decision(DecisionKind.RECOVERY, f"{reason}_NO_CANDIDATES", evidence)

        evidence.update(self._candidate_evidence(now, summary))
        eligible = summary.eligible_candidates
        ids = tuple(c.candidate_id for c in eligible)
        if len(eligible) == 0:
            return Decision(
                DecisionKind.RECOVERY,
                f"{reason}_NO_VALID_CANDIDATE",
                evidence,
            )
        if len(eligible) == 1:
            return Decision(DecisionKind.MAP_DIRECT, reason, evidence, ids)
        return Decision(DecisionKind.MAP_QWEN, reason, evidence, ids)

    def _fresh_candidate_summary(self, now: float) -> Optional[CandidateSummary]:
        summary = self.latest_candidates
        if summary is None:
            return None
        if now - summary.stamp > self.cfg.candidates_max_age_sec:
            return None
        return summary

    def _candidate_evidence(
        self, now: float, summary: CandidateSummary
    ) -> Dict[str, Any]:
        eligible = summary.eligible_candidates
        scores = sorted(
            (c.score for c in eligible if c.score is not None), reverse=True
        )
        gap = scores[0] - scores[1] if len(scores) >= 2 else None
        ratio = (
            scores[0] / scores[1]
            if len(scores) >= 2 and abs(scores[1]) > 1e-9
            else None
        )
        return {
            **self._basic_evidence(now),
            "candidate_age_sec": round(now - summary.stamp, 3),
            "candidate_count": len(summary.candidates),
            "eligible_candidate_count": len(eligible),
            "candidate_ids": [c.candidate_id for c in eligible],
            "max_heading_separation_deg": round(
                self._max_heading_separation_deg(
                    [c.heading_deg for c in eligible]
                ),
                2,
            ),
            "top_score_gap": None if gap is None else round(gap, 4),
            "top_score_ratio": None if ratio is None else round(ratio, 4),
            "decision_distance_m": summary.decision_distance_m,
            "map_version": summary.map_version,
            "junction_id": summary.junction_id,
            "returned_to_junction": summary.returned_to_junction,
        }

    def _basic_evidence(self, now: float) -> Dict[str, Any]:
        servo = self.latest_servo
        pose = self.latest_pose
        return {
            "stamp": round(now, 3),
            "state": None if servo is None else servo.state,
            "result": None if servo is None else servo.result,
            "action": None if servo is None else servo.action,
            "request_id": None if servo is None else servo.request_id,
            "pose": None
            if pose is None
            else [round(pose.x, 3), round(pose.y, 3), round(pose.yaw, 4)],
            "cooldown_remaining_sec": round(max(0.0, self.cooldown_until - now), 3),
        }

    def _pose_displacement(self, start: float, end: float) -> float:
        poses = [p for p in self.pose_history if start <= p.stamp <= end]
        if len(poses) < 2:
            return 0.0
        return self._distance_between(poses[0], poses[-1])

    def _reset_recovery_stage(self) -> None:
        self._recovery_stage = "IDLE"
        self._recovery_stage_started = float("-inf")
        self._recovery_pose_start = None
        self._recovery_notice_emitted = False

    def _trim_histories(self, now: float) -> None:
        cutoff = now - self.cfg.history_sec
        for queue in (self.servo_history, self.pose_history, self.result_history):
            while queue and queue[0].stamp < cutoff:
                queue.popleft()
        reverse_cutoff = now - self.cfg.emergency_reverse_window_sec
        while self.reverse_edges and self.reverse_edges[0] < reverse_cutoff:
            self.reverse_edges.popleft()

    @staticmethod
    def _distance_between(
        left: Optional[PoseSample], right: Optional[PoseSample]
    ) -> float:
        if left is None or right is None:
            return 0.0
        return math.hypot(right.x - left.x, right.y - left.y)

    @staticmethod
    def _count_changes(values: Iterable[Any]) -> int:
        seq = list(values)
        return sum(1 for a, b in zip(seq, seq[1:]) if a != b)

    @staticmethod
    def _max_heading_separation_deg(headings: Sequence[float]) -> float:
        if len(headings) < 2:
            return 0.0
        best = 0.0
        for i, left in enumerate(headings):
            for right in headings[i + 1 :]:
                delta = abs((left - right + 180.0) % 360.0 - 180.0)
                best = max(best, delta)
        return best
