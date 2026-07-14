#!/usr/bin/env python3
"""Pure POINT visual servo and one-shot view-adjust logic.

No ROS imports are used here, so parser/control behavior can be tested off-board.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class ServoConfig:
    max_vx: float = 0.07
    max_wz: float = 0.05
    kp_wz: float = 0.05
    angular_sign: float = -1.0
    center_deadband: float = 0.06
    turn_only_threshold: float = 0.40
    cmd_wz_deadband: float = 0.006
    min_confidence: float = 0.0
    point_results_before_forward: int = 1
    blocked_states: Tuple[str, ...] = (
        "WAIT_IMAGE",
        "PAUSED",
        "SUCCESS",
        "ERROR",
    )
    full_speed_source_age_sec: float = 0.95
    stop_source_age_sec: float = 2.00
    max_receive_gap_sec: float = 1.60
    require_lidar: bool = True
    scan_timeout_sec: float = 0.50
    emergency_stop_distance: float = 0.28
    stop_distance: float = 0.42
    slow_distance: float = 0.65
    allow_turn_inside_stop_distance: bool = True

    def validate(self) -> None:
        if self.max_vx < 0.0 or self.max_wz < 0.0:
            raise ValueError("max_vx/max_wz must be non-negative")
        if not 0.0 <= self.center_deadband < self.turn_only_threshold <= 1.0:
            raise ValueError(
                "require 0 <= center_deadband < turn_only_threshold <= 1"
            )
        if not 0.0 <= self.min_confidence <= 100.0:
            raise ValueError("min_confidence must be within [0,100]")
        if self.point_results_before_forward < 1:
            raise ValueError("point_results_before_forward must be >= 1")
        if not 0.0 <= self.full_speed_source_age_sec < self.stop_source_age_sec:
            raise ValueError(
                "require full_speed_source_age_sec < stop_source_age_sec"
            )
        if not (
            0.0 < self.emergency_stop_distance
            <= self.stop_distance
            < self.slow_distance
        ):
            raise ValueError(
                "require emergency_stop_distance <= stop_distance < slow_distance"
            )


@dataclass(frozen=True)
class ServoInput:
    now_sec: float
    state: str
    result: str
    action: str
    point_role: str
    point_x: Optional[float]
    image_width: int
    confidence: float
    latency_ms: float
    result_received_sec: float
    point_streak: int
    front_distance: Optional[float]
    scan_received_sec: Optional[float]


@dataclass(frozen=True)
class ServoDecision:
    vx: float
    wz: float
    hard_stop: bool
    reason: str
    horizontal_error: float = 0.0
    source_age_sec: float = float("inf")
    freshness_scale: float = 0.0
    heading_scale: float = 0.0
    obstacle_scale: float = 0.0


class QwenVisualServo:
    """Existing any-point servo, now explicitly gated by a=POINT."""

    _TRACK_RESULTS = frozenset({"TARGET_VISIBLE"})

    def __init__(self, config: ServoConfig):
        config.validate()
        self.cfg = config
        self._blocked = {
            str(s).strip().upper() for s in config.blocked_states
        }

    def compute(self, data: ServoInput) -> ServoDecision:
        cfg = self.cfg
        state = str(data.state or "").strip().upper()
        result = str(data.result or "").strip().upper()
        action = str(data.action or "POINT").strip().upper()
        role = str(data.point_role or "none").strip().lower()

        if action != "POINT":
            return self._stop(f"action_{action.lower()}")
        if state in self._blocked:
            return self._stop(f"fsm_{state.lower()}")
        if data.point_x is None or data.image_width <= 1:
            return self._stop("no_valid_pixel")
        if data.confidence < cfg.min_confidence:
            return self._stop("low_confidence")

        receive_gap = max(0.0, data.now_sec - data.result_received_sec)
        source_age = receive_gap + max(0.0, data.latency_ms) / 1000.0
        if receive_gap > cfg.max_receive_gap_sec:
            return self._stop("result_receive_timeout", source_age)
        if source_age >= cfg.stop_source_age_sec:
            return self._stop("source_frame_stale", source_age)
        freshness = self._freshness_scale(source_age)

        is_track = result in self._TRACK_RESULTS or role == "target"
        if cfg.require_lidar and is_track:
            if data.scan_received_sec is None or data.front_distance is None:
                return self._stop("waiting_for_lidar", source_age)
            if data.now_sec - data.scan_received_sec > cfg.scan_timeout_sec:
                return self._stop("lidar_stale", source_age)
            if data.front_distance <= cfg.emergency_stop_distance:
                return self._stop("emergency_obstacle", source_age)
        elif (
            data.front_distance is not None
            and data.scan_received_sec is not None
            and data.now_sec - data.scan_received_sec <= cfg.scan_timeout_sec
            and data.front_distance <= cfg.emergency_stop_distance
        ):
            return self._stop("emergency_obstacle", source_age)

        half_width = 0.5 * float(data.image_width - 1)
        error = (float(data.point_x) - half_width) / max(1.0, half_width)
        error = clamp(error, -1.0, 1.0)
        abs_error = abs(error)

        if abs_error <= cfg.center_deadband:
            wz = 0.0
        else:
            wz = cfg.angular_sign * cfg.kp_wz * error * freshness
            wz = clamp(wz, -cfg.max_wz, cfg.max_wz)
            if abs(wz) < cfg.cmd_wz_deadband:
                wz = 0.0

        heading_scale = self._heading_scale(abs_error)
        if (not is_track) and data.front_distance is None:
            obstacle_scale = 1.0
        else:
            obstacle_scale = self._obstacle_scale(data.front_distance)

        forward_confirmed = (
            data.point_streak >= cfg.point_results_before_forward
        )
        searchish = (not is_track) or role in {"search", "verify"}
        if not forward_confirmed:
            vx = 0.0
            reason = "rotate_only_wait_point_confirm"
        elif heading_scale <= 0.0:
            vx = 0.0
            reason = (
                "search_rotate_only_large_error"
                if searchish
                else "rotate_only_large_error"
            )
        elif obstacle_scale <= 0.0:
            vx = 0.0
            reason = "stop_distance_reached"
            if not cfg.allow_turn_inside_stop_distance:
                wz = 0.0
        else:
            vx = cfg.max_vx * heading_scale * freshness * obstacle_scale
            reason = (
                "search_visual_servo"
                if searchish
                else "continuous_visual_servo"
            )

        return ServoDecision(
            vx=vx,
            wz=wz,
            hard_stop=False,
            reason=reason,
            horizontal_error=error,
            source_age_sec=source_age,
            freshness_scale=freshness,
            heading_scale=heading_scale,
            obstacle_scale=obstacle_scale,
        )

    def _freshness_scale(self, source_age: float) -> float:
        cfg = self.cfg
        if source_age <= cfg.full_speed_source_age_sec:
            return 1.0
        span = cfg.stop_source_age_sec - cfg.full_speed_source_age_sec
        return clamp(
            (cfg.stop_source_age_sec - source_age) / span,
            0.0,
            1.0,
        )

    def _heading_scale(self, abs_error: float) -> float:
        cfg = self.cfg
        if abs_error <= cfg.center_deadband:
            return 1.0
        if abs_error >= cfg.turn_only_threshold:
            return 0.0
        span = cfg.turn_only_threshold - cfg.center_deadband
        linear = (cfg.turn_only_threshold - abs_error) / span
        return clamp(linear * linear, 0.0, 1.0)

    def _obstacle_scale(self, front_distance: Optional[float]) -> float:
        cfg = self.cfg
        if front_distance is None:
            return 0.0 if cfg.require_lidar else 1.0
        if front_distance <= cfg.stop_distance:
            return 0.0
        if front_distance >= cfg.slow_distance:
            return 1.0
        return clamp(
            (front_distance - cfg.stop_distance)
            / (cfg.slow_distance - cfg.stop_distance),
            0.0,
            1.0,
        )

    @staticmethod
    def _stop(
        reason: str,
        source_age: float = float("inf"),
    ) -> ServoDecision:
        return ServoDecision(
            vx=0.0,
            wz=0.0,
            hard_stop=True,
            reason=reason,
            source_age_sec=source_age,
        )


class ViewAdjustPhase(str, Enum):
    IDLE = "IDLE"
    PRE_TURN_STOP = "PRE_TURN_STOP"
    TURNING = "TURNING"
    SETTLING = "SETTLING"
    WAITING_FRESH_RESULT = "WAITING_FRESH_RESULT"


@dataclass(frozen=True)
class ViewAdjustConfig:
    turn_left_wz: float = 0.06
    turn_right_wz: float = -0.06
    pre_turn_stop_sec: float = 0.15
    turn_pulse_sec: float = 1.20
    settle_sec: float = 0.30

    def validate(self) -> None:
        if self.turn_left_wz == 0.0 or self.turn_right_wz == 0.0:
            raise ValueError("turn wz values must be non-zero")
        if self.turn_left_wz * self.turn_right_wz >= 0.0:
            raise ValueError("left/right turn wz must have opposite signs")
        if (
            self.pre_turn_stop_sec < 0.0
            or self.turn_pulse_sec <= 0.0
            or self.settle_sec < 0.0
        ):
            raise ValueError("invalid pre_turn_stop_sec/turn_pulse_sec/settle_sec")


@dataclass(frozen=True)
class ViewAdjustDecision:
    vx: float
    wz: float
    hard_stop: bool
    phase: ViewAdjustPhase
    reason: str
    request_fresh_observation: bool = False


class ViewAdjustController:
    """One pulse per unique TURN result, then stop and request a fresh image.

    This deliberately uses time, not a claimed fixed angle. Without a verified
    yaw source, pretending an accurate angle would merely convert uncertainty
    into a more sophisticated-looking uncertainty.
    """

    def __init__(self, config: ViewAdjustConfig):
        config.validate()
        self.cfg = config
        self.phase = ViewAdjustPhase.IDLE
        self.action = "STOP"
        self.request_id = -1
        self.started_sec = 0.0
        self._fresh_event_sent = False

    @property
    def active(self) -> bool:
        return self.phase in {
            ViewAdjustPhase.PRE_TURN_STOP,
            ViewAdjustPhase.TURNING,
            ViewAdjustPhase.SETTLING,
            ViewAdjustPhase.WAITING_FRESH_RESULT,
        }

    def start(self, action: str, request_id: int, now_sec: float) -> bool:
        action = str(action).strip().upper()
        if action not in {"TURN_LEFT", "TURN_RIGHT"}:
            raise ValueError(f"unsupported view action: {action}")
        if request_id == self.request_id:
            return False
        self.action = action
        self.request_id = int(request_id)
        self.started_sec = float(now_sec)
        self.phase = ViewAdjustPhase.PRE_TURN_STOP
        self._fresh_event_sent = False
        return True

    def cancel(self) -> None:
        self.phase = ViewAdjustPhase.IDLE
        self.action = "STOP"
        self._fresh_event_sent = False

    def update(self, now_sec: float) -> ViewAdjustDecision:
        if self.phase == ViewAdjustPhase.IDLE:
            return ViewAdjustDecision(
                0.0,
                0.0,
                True,
                self.phase,
                "view_adjust_idle",
            )

        elapsed = max(0.0, float(now_sec) - self.started_sec)
        if elapsed < self.cfg.pre_turn_stop_sec:
            self.phase = ViewAdjustPhase.PRE_TURN_STOP
            return ViewAdjustDecision(
                0.0,
                0.0,
                True,
                self.phase,
                "view_adjust_pre_turn_stop",
            )

        turn_elapsed = elapsed - self.cfg.pre_turn_stop_sec
        if turn_elapsed < self.cfg.turn_pulse_sec:
            self.phase = ViewAdjustPhase.TURNING
            wz = (
                self.cfg.turn_left_wz
                if self.action == "TURN_LEFT"
                else self.cfg.turn_right_wz
            )
            return ViewAdjustDecision(
                0.0,
                wz,
                False,
                self.phase,
                f"view_adjust_{self.action.lower()}",
            )

        if turn_elapsed < self.cfg.turn_pulse_sec + self.cfg.settle_sec:
            self.phase = ViewAdjustPhase.SETTLING
            return ViewAdjustDecision(
                0.0,
                0.0,
                True,
                self.phase,
                "view_adjust_settling",
            )

        self.phase = ViewAdjustPhase.WAITING_FRESH_RESULT
        request = not self._fresh_event_sent
        self._fresh_event_sent = True
        return ViewAdjustDecision(
            0.0,
            0.0,
            True,
            self.phase,
            "view_adjust_waiting_fresh_result",
            request_fresh_observation=request,
        )


@dataclass(frozen=True)
class TurnPendingConfig:
    entry_distance: float = 0.45
    entry_frames: int = 3
    pending_vx: float = 0.04

    def validate(self) -> None:
        if self.entry_distance <= 0.0:
            raise ValueError("entry_distance must be positive")
        if self.entry_frames < 1:
            raise ValueError("entry_frames must be >= 1")
        if self.pending_vx < 0.0:
            raise ValueError("pending_vx must be non-negative")


class TurnPendingGate:
    def __init__(self, config: TurnPendingConfig):
        config.validate()
        self.cfg = config
        self.action = ""
        self.request_id = -1
        self.near_count = 0

    @property
    def active(self) -> bool:
        return self.action in {"TURN_LEFT", "TURN_RIGHT"}

    @property
    def ready(self) -> bool:
        return self.active and self.near_count >= self.cfg.entry_frames

    def start(self, action: str, request_id: int) -> bool:
        action = str(action).strip().upper()
        if action not in {"TURN_LEFT", "TURN_RIGHT"}:
            raise ValueError(f"unsupported pending turn action: {action}")
        if self.active and self.request_id == int(request_id):
            return False
        self.action = action
        self.request_id = int(request_id)
        self.near_count = 0
        return True

    def cancel(self) -> None:
        self.action = ""
        self.request_id = -1
        self.near_count = 0

    def update_scan(self, front_distance: Optional[float]) -> None:
        if not self.active:
            return
        if front_distance is not None and front_distance <= self.cfg.entry_distance:
            self.near_count += 1
        else:
            self.near_count = 0

    def desired_vx(self, current_vx: float, max_vx: float) -> float:
        cap = min(max(0.0, self.cfg.pending_vx), max(0.0, max_vx))
        if cap <= 0.0:
            return 0.0
        if current_vx > 0.0:
            return min(current_vx, cap)
        return cap


@dataclass(frozen=True)
class RateLimitConfig:
    max_linear_accel: float = 0.08
    max_linear_decel: float = 0.16
    max_angular_accel: float = 0.12
    max_angular_decel: float = 0.25


class CommandRateLimiter:
    def __init__(self, config: RateLimitConfig):
        self.cfg = config
        self.vx = 0.0
        self.wz = 0.0

    def reset(self) -> None:
        self.vx = 0.0
        self.wz = 0.0

    def step(
        self,
        target_vx: float,
        target_wz: float,
        dt: float,
        hard_stop: bool = False,
    ) -> tuple[float, float]:
        if hard_stop:
            self.reset()
            return 0.0, 0.0
        dt = max(1e-3, min(0.5, float(dt)))
        self.vx = self._limit_axis(
            self.vx,
            target_vx,
            self.cfg.max_linear_accel,
            self.cfg.max_linear_decel,
            dt,
        )
        self.wz = self._limit_axis(
            self.wz,
            target_wz,
            self.cfg.max_angular_accel,
            self.cfg.max_angular_decel,
            dt,
        )
        return self.vx, self.wz

    @staticmethod
    def _limit_axis(
        current: float,
        target: float,
        accel: float,
        decel: float,
        dt: float,
    ) -> float:
        delta = target - current
        speeding_up = abs(target) > abs(current) and current * target >= 0.0
        limit = max(0.0, accel if speeding_up else decel) * dt
        if abs(delta) <= limit:
            return target
        return current + (limit if delta > 0.0 else -limit)
