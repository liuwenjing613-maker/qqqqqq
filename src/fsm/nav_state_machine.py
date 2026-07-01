#!/usr/bin/env python3
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class NavState(str, Enum):
    BOOT = "BOOT"
    WAIT_SENSORS = "WAIT_SENSORS"
    SEARCH = "SEARCH"
    CANDIDATE_LOCK = "CANDIDATE_LOCK"
    TRACK = "TRACK"
    LOST_RECOVERY = "LOST_RECOVERY"
    BLOCKED = "BLOCKED"
    ARRIVE_VERIFY = "ARRIVE_VERIFY"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


@dataclass
class NavObservation:
    now: float
    image_fresh: bool
    scan_fresh: bool
    require_lidar: bool

    target_visible: bool
    target_stale: bool
    target_score: float
    target_score_ok: bool
    target_u: Optional[float]
    target_v: Optional[float]
    target_centered: bool
    target_center_error_px: Optional[float] = None
    target_area_ratio: Optional[float] = None
    target_height_ratio: Optional[float] = None

    front_distance: Optional[float] = None
    emergency: bool = False
    blocked: bool = False

    qwen_verified: Optional[bool] = None


@dataclass
class NavFSMConfig:
    stable_frames_required: int = 3
    lost_frames_limit: int = 5
    arrive_required_frames: int = 4
    verify_required_frames: int = 4
    centered_required_frames: int = 3
    max_search_sec: float = 30.0
    max_task_sec: float = 180.0
    min_state_frames: int = 2
    qwen_verify_required: bool = False
    qwen_verify_timeout_sec: float = 12.0
    qwen_verify_fail_policy: str = "search"
    recovery_max_sec: float = 4.0
    min_safe_distance: float = 0.35
    stop_distance: float = 0.75
    verify_distance_max: float = 0.85
    emergency_stop_distance: float = 0.25
    arrive_area_ratio: float = 0.16
    arrive_height_ratio: float = 0.32
    success_min_score: float = 0.20
    success_center_px: float = 140.0
    success_target_recent_sec: float = 1.0
    lidar_success_distance: float = 0.95
    stop_verify_sec: float = 0.8
    emergency_target_recent_sec: float = 1.5
    lost_target_servo_sec: float = 0.6
    center_only_arrive_enabled: bool = False


@dataclass
class NavFSMResult:
    state: NavState
    previous_state: NavState
    changed: bool
    reason: str
    stable_frames: int
    lost_frames: int
    arrive_frames: int
    centered_frames: int
    state_elapsed_sec: float
    task_elapsed_sec: float


class NavStateMachine:
    def __init__(self, cfg: Optional[NavFSMConfig] = None):
        self.cfg = cfg or NavFSMConfig()
        self.state = NavState.BOOT
        self.task_start_time: Optional[float] = None
        self.state_enter_time: Optional[float] = None
        self.stable_frames = 0
        self.lost_frames = 0
        self.arrive_frames = 0
        self.centered_frames = 0
        self.block_clear_frames = 0
        self.arrive_verify_frames = 0
        self.last_target_seen_time: Optional[float] = None
        self.last_target_score = 0.0
        self.last_center_error_px: Optional[float] = None
        self.last_area_ratio: Optional[float] = None
        self.last_height_ratio: Optional[float] = None
        self._last_result_reason = "init"

    def reset(self, now: Optional[float] = None) -> None:
        self.state = NavState.BOOT
        self.task_start_time = now
        self.state_enter_time = now
        self.stable_frames = 0
        self.lost_frames = 0
        self.arrive_frames = 0
        self.centered_frames = 0
        self.block_clear_frames = 0
        self.arrive_verify_frames = 0
        self.last_target_seen_time = None
        self.last_target_score = 0.0
        self.last_center_error_px = None
        self.last_area_ratio = None
        self.last_height_ratio = None
        self._last_result_reason = "reset"

    def update(self, obs: NavObservation) -> NavFSMResult:
        if self.task_start_time is None:
            self.task_start_time = obs.now
        if self.state_enter_time is None:
            self.state_enter_time = obs.now

        previous = self.state
        reason = "hold"

        target_ok = self._target_ok(obs)
        sensors_ok = obs.image_fresh and (not obs.require_lidar or obs.scan_fresh)
        task_elapsed = max(0.0, obs.now - self.task_start_time)
        self._update_target_memory(obs)

        if self.state in (NavState.SUCCESS, NavState.FAILED):
            reason = "terminal"
        elif self.cfg.max_task_sec > 0 and task_elapsed > self.cfg.max_task_sec:
            self._enter(NavState.FAILED, obs.now)
            reason = "max_task_sec"
        elif obs.require_lidar and not obs.scan_fresh:
            self._enter(NavState.WAIT_SENSORS, obs.now)
            reason = "scan_stale"
        elif not obs.image_fresh:
            self._enter(NavState.WAIT_SENSORS, obs.now)
            reason = "image_stale"
        elif obs.emergency and self.state != NavState.ARRIVE_VERIFY:
            self._enter(NavState.BLOCKED, obs.now)
            reason = "emergency"
        elif self.state == NavState.BOOT:
            self._enter(NavState.WAIT_SENSORS, obs.now)
            reason = "boot"
        elif self.state == NavState.WAIT_SENSORS:
            if sensors_ok:
                if target_ok:
                    self._enter(NavState.CANDIDATE_LOCK, obs.now)
                    self.stable_frames = 1
                    reason = "sensor_ready_target"
                else:
                    self._enter(NavState.SEARCH, obs.now)
                    reason = "sensor_ready_search"
            else:
                reason = "waiting_sensors"
        elif self.state == NavState.SEARCH:
            if obs.blocked:
                self._enter(NavState.BLOCKED, obs.now)
                reason = "blocked"
            elif target_ok:
                self._enter(NavState.CANDIDATE_LOCK, obs.now)
                self.stable_frames = 1
                reason = "target_candidate"
            elif self._state_elapsed(obs.now) > self.cfg.max_search_sec:
                self._enter(NavState.FAILED, obs.now)
                reason = "max_search_sec"
            else:
                reason = "searching"
        elif self.state == NavState.CANDIDATE_LOCK:
            if obs.blocked:
                self._enter(NavState.BLOCKED, obs.now)
                reason = "blocked"
            elif not target_ok:
                self._enter(NavState.LOST_RECOVERY, obs.now)
                reason = "candidate_lost"
            else:
                self.stable_frames += 1
                if self.stable_frames >= max(1, self.cfg.stable_frames_required):
                    self._enter(NavState.TRACK, obs.now)
                    self.lost_frames = 0
                    reason = "target_stable"
                else:
                    reason = "candidate_lock"
        elif self.state == NavState.TRACK:
            if obs.emergency:
                self._enter(NavState.BLOCKED, obs.now)
                reason = "emergency"
            elif obs.blocked:
                self._enter(NavState.BLOCKED, obs.now)
                reason = "blocked"
            elif self.arrive_ok_recent_target(obs):
                self.lost_frames = 0
                self.arrive_frames = 0
                self._enter(NavState.ARRIVE_VERIFY, obs.now)
                self.arrive_verify_frames = 0
                reason = "arrive_ok_recent_target"
            elif target_ok:
                self.lost_frames = 0
                self.arrive_frames = 0
                reason = "tracking"
            elif self.target_recent(obs.now, self.cfg.lost_target_servo_sec):
                self.lost_frames = 0
                self.arrive_frames = 0
                reason = "target_lost_recent_servo"
            else:
                self.lost_frames += 1
                self.arrive_frames = 0
                if self.lost_frames >= max(1, self.cfg.lost_frames_limit):
                    self._enter(NavState.LOST_RECOVERY, obs.now)
                    reason = "target_lost"
                else:
                    reason = "target_lost_grace"
        elif self.state == NavState.LOST_RECOVERY:
            if target_ok:
                self._enter(NavState.CANDIDATE_LOCK, obs.now)
                self.stable_frames = 1
                reason = "target_reacquired"
            elif self._state_elapsed(obs.now) > self.cfg.recovery_max_sec:
                self._enter(NavState.SEARCH, obs.now)
                reason = "recovery_timeout"
            else:
                reason = "lost_recovery"
        elif self.state == NavState.BLOCKED:
            if obs.emergency:
                self.block_clear_frames = 0
                reason = "emergency"
            elif not obs.blocked:
                self.block_clear_frames += 1
                if self.block_clear_frames >= 3:
                    if target_ok:
                        self._enter(NavState.CANDIDATE_LOCK, obs.now)
                        self.stable_frames = 1
                        reason = "block_clear_target"
                    else:
                        self._enter(NavState.SEARCH, obs.now)
                        reason = "block_clear_search"
                else:
                    reason = "block_clearing"
            else:
                self.block_clear_frames = 0
                reason = "blocked"
        elif self.state == NavState.ARRIVE_VERIFY:
            if obs.emergency:
                if self.target_recent(obs.now, self.cfg.emergency_target_recent_sec):
                    self._enter(NavState.SUCCESS, obs.now)
                    reason = "emergency_recent_target_success"
                else:
                    self._enter(NavState.BLOCKED, obs.now)
                    reason = "emergency_no_recent_target"
            elif self.cfg.qwen_verify_required:
                if obs.qwen_verified is True:
                    self._enter(NavState.SUCCESS, obs.now)
                    reason = "qwen_verified"
                elif obs.qwen_verified is False:
                    self._enter(NavState.SEARCH, obs.now)
                    reason = "qwen_rejected"
                elif self._state_elapsed(obs.now) > self.cfg.qwen_verify_timeout_sec:
                    if self.cfg.qwen_verify_fail_policy == "success":
                        self._enter(NavState.SUCCESS, obs.now)
                        reason = "qwen_timeout_success"
                    else:
                        self._enter(NavState.SEARCH, obs.now)
                        reason = "qwen_timeout_search"
                else:
                    reason = "waiting_qwen_verify"
            elif self._state_elapsed(obs.now) >= self.cfg.stop_verify_sec:
                self._enter(NavState.SUCCESS, obs.now)
                reason = "stop_verify_timeout_success"
            else:
                reason = "stop_verify_wait"

        if obs.target_centered and target_ok:
            self.centered_frames += 1
        else:
            self.centered_frames = 0

        self._last_result_reason = reason
        return NavFSMResult(
            state=self.state,
            previous_state=previous,
            changed=self.state != previous,
            reason=reason,
            stable_frames=self.stable_frames,
            lost_frames=self.lost_frames,
            arrive_frames=self.arrive_frames,
            centered_frames=self.centered_frames,
            state_elapsed_sec=self._state_elapsed(obs.now),
            task_elapsed_sec=max(0.0, obs.now - (self.task_start_time or obs.now)),
        )

    def arrive_ok(self, obs: NavObservation) -> bool:
        return self.arrive_ok_recent_target(obs)

    def arrive_ok_recent_target(self, obs: NavObservation) -> bool:
        target_recent = self.target_recent(obs.now, self.cfg.success_target_recent_sec)
        score_ok = self.last_target_score >= self.cfg.success_min_score
        center_recent_ok = (
            self.last_center_error_px is not None
            and abs(self.last_center_error_px) <= self.cfg.success_center_px
        )
        lidar_success = (
            obs.front_distance is not None
            and obs.front_distance <= self.cfg.lidar_success_distance
        )
        bbox_close = (
            self.last_area_ratio is not None
            and self.last_area_ratio >= self.cfg.arrive_area_ratio
        ) or (
            self.last_height_ratio is not None
            and self.last_height_ratio >= self.cfg.arrive_height_ratio
        )
        return bool(target_recent and score_ok and center_recent_ok and (lidar_success or bbox_close))

    def _verify_distance_ok(self, obs: NavObservation) -> bool:
        if obs.front_distance is None:
            return not obs.require_lidar
        return (
            obs.front_distance > self.cfg.min_safe_distance
            and obs.front_distance <= self.cfg.verify_distance_max
        )

    @staticmethod
    def _target_ok(obs: NavObservation) -> bool:
        return bool(obs.target_visible and not obs.target_stale and obs.target_score_ok)

    def target_recent(self, now: float, within_sec: float) -> bool:
        return self.target_seen_age(now) <= within_sec

    def target_seen_age(self, now: float) -> float:
        if self.last_target_seen_time is None:
            return float("inf")
        return max(0.0, now - self.last_target_seen_time)

    def _update_target_memory(self, obs: NavObservation) -> None:
        if not obs.target_visible or obs.target_stale:
            return
        self.last_target_seen_time = obs.now
        self.last_target_score = float(obs.target_score)
        self.last_center_error_px = obs.target_center_error_px
        self.last_area_ratio = obs.target_area_ratio
        self.last_height_ratio = obs.target_height_ratio

    def _enter(self, state: NavState, now: float) -> None:
        if state == self.state:
            return
        self.state = state
        self.state_enter_time = now
        if state != NavState.CANDIDATE_LOCK:
            self.stable_frames = 0
        if state != NavState.TRACK:
            self.lost_frames = 0
        if state != NavState.BLOCKED:
            self.block_clear_frames = 0

    def _state_elapsed(self, now: float) -> float:
        return max(0.0, now - (self.state_enter_time or now))
