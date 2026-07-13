#!/usr/bin/env python3
"""Pure visual-servo control logic for Qwen pixel outputs.

This module deliberately contains no ROS imports so it can be unit-tested on a
laptop.  The ROS node adapts /qwen_vln/result_json, /qwen_vln/state and LaserScan
messages into :class:`ServoInput`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class ServoConfig:
    max_vx: float = 0.04
    max_wz: float = 0.05
    kp_wz: float = 0.10
    angular_sign: float = -1.0
    center_deadband: float = 0.06
    turn_only_threshold: float = 0.40
    cmd_wz_deadband: float = 0.006
    min_confidence: float = 0.55
    visible_results_before_forward: int = 2
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
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be within [0, 1]")
        if self.visible_results_before_forward < 1:
            raise ValueError("visible_results_before_forward must be >= 1")
        if not 0.0 <= self.full_speed_source_age_sec < self.stop_source_age_sec:
            raise ValueError(
                "require 0 <= full_speed_source_age_sec < stop_source_age_sec"
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
    point_x: Optional[float]
    image_width: int
    confidence: float
    latency_ms: float
    result_received_sec: float
    visible_streak: int
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
    """Sample-and-hold visual servo with freshness and lidar safety gates.

    The latest Qwen pixel is held between model replies.  Command magnitude
    decreases as the originating image becomes old, and reaches exactly zero at
    ``stop_source_age_sec``.

    Motion is allowed for:
    - TARGET_LOCKED + TARGET_VISIBLE (track the target)
    - SEARCHING / TARGET_INFERRED + TARGET_INFERRED / VERIFY_FAILED
      (drive toward the search waypoint). Search mode accepts a null front
      distance and does not require lidar range_max returns.
    """

    _SEARCH_STATES = frozenset({"SEARCHING", "TARGET_INFERRED"})
    _SEARCH_RESULTS = frozenset({"TARGET_INFERRED", "VERIFY_FAILED"})

    def __init__(self, config: ServoConfig):
        config.validate()
        self.cfg = config

    def compute(self, data: ServoInput) -> ServoDecision:
        cfg = self.cfg

        target_visible = (
            data.state == "TARGET_LOCKED" and data.result == "TARGET_VISIBLE"
        )
        target_search = (
            data.state in self._SEARCH_STATES
            and data.result in self._SEARCH_RESULTS
        )
        if not target_visible and not target_search:
            if data.state not in {"TARGET_LOCKED", *self._SEARCH_STATES}:
                return self._stop(f"fsm_{data.state.lower()}")
            return self._stop(f"result_{data.result.lower() or 'none'}")
        if data.point_x is None or data.image_width <= 1:
            return self._stop("invalid_pixel")
        if data.confidence < cfg.min_confidence:
            return self._stop("low_confidence")

        receive_gap = max(0.0, data.now_sec - data.result_received_sec)
        source_age = receive_gap + max(0.0, data.latency_ms) / 1000.0
        if receive_gap > cfg.max_receive_gap_sec:
            return self._stop("result_receive_timeout", source_age)
        if source_age >= cfg.stop_source_age_sec:
            return self._stop("source_frame_stale", source_age)

        freshness = self._freshness_scale(source_age)

        if target_visible and cfg.require_lidar:
            # Track mode keeps the full lidar gate, including rejecting null
            # front_distance (often caused by no returns within range_max).
            if data.scan_received_sec is None or data.front_distance is None:
                return self._stop("waiting_for_lidar", source_age)
            if data.now_sec - data.scan_received_sec > cfg.scan_timeout_sec:
                return self._stop("lidar_stale", source_age)
            if data.front_distance <= cfg.emergency_stop_distance:
                return self._stop("emergency_obstacle", source_age)
        elif (
            target_search
            and data.front_distance is not None
            and data.scan_received_sec is not None
            and data.now_sec - data.scan_received_sec <= cfg.scan_timeout_sec
            and data.front_distance <= cfg.emergency_stop_distance
        ):
            # Search still hard-stops on a known near obstacle, but null /
            # missing lidar range_max returns are accepted and do not block.
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
            # The existing PWM bridge has a non-zero deadband offset.  Sending
            # microscopic wz values can therefore still cause a visible turn.
            if abs(wz) < cfg.cmd_wz_deadband:
                wz = 0.0

        heading_scale = self._heading_scale(abs_error)
        if target_search and data.front_distance is None:
            obstacle_scale = 1.0
        else:
            obstacle_scale = self._obstacle_scale(data.front_distance)

        # Search waypoints are not TARGET_VISIBLE, so visible_streak stays 0;
        # do not force rotate-only while exploring.
        forward_confirmed = target_search or (
            data.visible_streak >= cfg.visible_results_before_forward
        )
        if not forward_confirmed:
            vx = 0.0
            reason = "rotate_only_wait_second_visible"
        elif heading_scale <= 0.0:
            vx = 0.0
            reason = (
                "search_rotate_only_large_error"
                if target_search
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
                "search_visual_servo" if target_search else "continuous_visual_servo"
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
        return clamp((cfg.stop_source_age_sec - source_age) / span, 0.0, 1.0)

    def _heading_scale(self, abs_error: float) -> float:
        cfg = self.cfg
        if abs_error <= cfg.center_deadband:
            return 1.0
        if abs_error >= cfg.turn_only_threshold:
            return 0.0
        span = cfg.turn_only_threshold - cfg.center_deadband
        # Smoothly suppress forward speed as the target moves away from center.
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
    def _stop(reason: str, source_age: float = float("inf")) -> ServoDecision:
        return ServoDecision(
            vx=0.0,
            wz=0.0,
            hard_stop=True,
            reason=reason,
            source_age_sec=source_age,
        )


@dataclass(frozen=True)
class RateLimitConfig:
    max_linear_accel: float = 0.08
    max_linear_decel: float = 0.16
    max_angular_accel: float = 0.12
    max_angular_decel: float = 0.25


class CommandRateLimiter:
    """Symmetric ROS-command slew limiter with immediate safety stop."""

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
