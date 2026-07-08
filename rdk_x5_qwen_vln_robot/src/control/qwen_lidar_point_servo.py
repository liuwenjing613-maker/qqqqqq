#!/usr/bin/env python3
"""Point visual servo: same steering law as rdk_x5_vln_robot PointServo + LiDAR safety."""
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Union

from geometry_msgs.msg import Twist


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# Distinguish "caller did not pass arrive_distance" from "explicitly disable arrive".
_ARRIVE_UNSET = object()


def turn_dir_from_ex(ex: float) -> float:
    """Match src/nav/search_strategy.py used by YOLO shared_nav."""
    if abs(ex) < 1e-6:
        return 1.0
    return -1.0 if ex > 0 else 1.0


def turn_angle_deg_from_ex(
    ex: float,
    center_deadband: float = 0.10,
    gain: float = 120.0,
    power: float = 0.85,
    max_turn_deg: float = 30.0,
) -> float:
    """Scheme B: θ = clamp(0, gain × e^power, max_turn_deg), e = max(0, |ex| - deadband)."""
    e = max(0.0, abs(ex) - float(center_deadband))
    if e <= 0.0:
        return 0.0
    theta = float(gain) * (e ** float(power))
    return clamp(theta, 0.0, float(max_turn_deg))


@dataclass
class QwenLidarServoResult:
    state: str
    cmd: Twist
    ex: float
    depth_used: Optional[float]
    front_distance: Optional[float]
    target_distance: Optional[float]
    reason: str
    wz: float = 0.0
    turn_angle_deg: float = 0.0
    remaining_yaw_deg: float = 0.0


def creep_wz_from_ex(ex: float, creep_wz: float) -> float:
    """Same turn sign as PointServo (-kp*ex), fixed magnitude."""
    if abs(ex) < 1e-6 or creep_wz <= 0.0:
        return 0.0
    return math.copysign(creep_wz, -ex)


class QwenLidarPointServo:
    """Steering law identical to PointServo; LiDAR only scales vx / emergency stop."""

    def __init__(
        self,
        image_width: int = 1280,
        require_lidar: bool = True,
        kp_turn: float = 0.1,
        max_wz: float = 0.05,
        center_deadband: float = 0.06,
        straight_hysteresis: float = 0.02,
        cmd_wz_deadband: float = 0.006,
        turn_only_threshold: float = 0.4,
        max_vx: float = 0.06,
        steer_vx: float = 0.05,
        turn_only_vx: float = 0.0,
        creep_mode: bool = False,
        creep_vx: float = 0.012,
        creep_wz: float = 0.012,
        creep_turn_vx: float = 0.0,
        allow_track_without_depth: bool = True,
        forward_vx_no_depth: float = 0.04,
        emergency_stop_distance: float = 0.28,
        hard_stop_distance: float = 0.42,
        slow_distance: float = 0.65,
        arrive_distance: float = 0.6,
        angle_servo_enabled: bool = False,
        angle_gain: float = 120.0,
        angle_power: float = 0.85,
        angle_max_turn_deg: float = 30.0,
        angle_turn_wz: float = 0.04,
        angle_complete_tol_deg: float = 0.5,
        angle_max_pending_deg: float = 45.0,
        angle_skip_while_turning: bool = True,
        angle_wait_turn_complete: bool = True,
        angle_absolute_cap_deg: float = 1.0,
    ):
        self.image_width = int(image_width)
        self.require_lidar = bool(require_lidar)
        self.kp_turn = float(kp_turn)
        self.max_wz = float(max_wz)
        self.center_deadband = float(center_deadband)
        self.straight_hysteresis = max(0.0, float(straight_hysteresis))
        self.cmd_wz_deadband = float(cmd_wz_deadband)
        self.turn_only_threshold = float(turn_only_threshold)
        self.max_vx = float(max_vx)
        self.steer_vx = float(steer_vx)
        self.turn_only_vx = float(turn_only_vx)
        self.creep_mode = bool(creep_mode)
        self.creep_vx = float(creep_vx)
        self.creep_wz = float(creep_wz)
        self.creep_turn_vx = float(creep_turn_vx)
        self.allow_track_without_depth = bool(allow_track_without_depth)
        self.forward_vx_no_depth = float(forward_vx_no_depth)
        self.emergency_stop_distance = float(emergency_stop_distance)
        self.hard_stop_distance = float(hard_stop_distance)
        self.slow_distance = float(slow_distance)
        self.arrive_distance = float(arrive_distance)
        self.angle_servo_enabled = bool(angle_servo_enabled)
        self.angle_gain = float(angle_gain)
        self.angle_power = float(angle_power)
        cap = max(0.0, float(angle_absolute_cap_deg))
        self.angle_absolute_cap_deg = cap
        self.angle_max_turn_deg = min(float(angle_max_turn_deg), cap) if cap > 0.0 else float(angle_max_turn_deg)
        self.angle_turn_wz = clamp(float(angle_turn_wz), 0.0, float(max_wz))
        self.angle_complete_tol_rad = math.radians(float(angle_complete_tol_deg))
        pending_cap = min(float(angle_max_pending_deg), self.angle_max_turn_deg)
        self.angle_max_pending_rad = math.radians(pending_cap)
        self.angle_skip_while_turning = bool(angle_skip_while_turning)
        self.angle_wait_turn_complete = bool(angle_wait_turn_complete)
        self._in_straight_band = True
        self.remaining_yaw_rad = 0.0
        self.last_turn_angle_deg = 0.0

    def update_image_width(self, image_width: int) -> None:
        self.image_width = int(image_width)

    def straight_band_px(self) -> float:
        """Half-width of straight-only zone in pixels (|u-cx| below this => no turn)."""
        return self.center_deadband * float(self.image_width)

    def remaining_yaw_deg(self) -> float:
        return math.degrees(self.remaining_yaw_rad)

    def clear_remaining(self) -> None:
        self.remaining_yaw_rad = 0.0
        self.last_turn_angle_deg = 0.0

    def is_turn_busy(self) -> bool:
        """True while an open-loop angle budget is still being executed."""
        if not self.angle_servo_enabled:
            return False
        return abs(self.remaining_yaw_rad) > self.angle_complete_tol_rad

    def _update_straight_band_state(self, abs_ex: float) -> bool:
        """Hysteresis: avoid micro-steer when Qwen u jitters near band edge."""
        enter_steer = self.center_deadband + self.straight_hysteresis
        enter_straight = max(0.0, self.center_deadband - self.straight_hysteresis)
        if self._in_straight_band:
            if abs_ex > enter_steer:
                self._in_straight_band = False
        elif abs_ex < enter_straight:
            self._in_straight_band = True
        return self._in_straight_band

    def _turn_budget_deg(self, ex: float) -> float:
        return turn_angle_deg_from_ex(
            ex,
            center_deadband=self.center_deadband,
            gain=self.angle_gain,
            power=self.angle_power,
            max_turn_deg=self.angle_max_turn_deg,
        )

    def _resolve_steer_ex(self, ex: float, ex_raw: Optional[float] = None) -> float:
        steer_ex = float(ex)
        if ex_raw is not None:
            ex_raw = float(ex_raw)
            if steer_ex * ex_raw < 0.0 or abs(ex_raw - steer_ex) > 0.08:
                steer_ex = ex_raw
        return steer_ex

    def set_turn_from_ex(self, ex: float, ex_raw: Optional[float] = None) -> float:
        """Set one-shot turn budget for this Qwen frame (never stack angles)."""
        if self.angle_wait_turn_complete and self.is_turn_busy():
            return self.last_turn_angle_deg

        steer_ex = self._resolve_steer_ex(ex, ex_raw)

        abs_ex = abs(steer_ex)
        if self._update_straight_band_state(abs_ex):
            if not self.is_turn_busy():
                self.clear_remaining()
            return 0.0

        theta_deg = self._turn_budget_deg(steer_ex)
        if theta_deg <= 0.0:
            if not self.is_turn_busy():
                self.clear_remaining()
            return 0.0

        delta_rad = math.copysign(math.radians(theta_deg), -steer_ex)
        if (
            self.angle_skip_while_turning
            and not self.angle_wait_turn_complete
            and abs(self.remaining_yaw_rad) > self.angle_complete_tol_rad
        ):
            if self.remaining_yaw_rad * delta_rad > 0.0:
                return self.last_turn_angle_deg

        self.remaining_yaw_rad = clamp(
            delta_rad,
            -self.angle_max_pending_rad,
            self.angle_max_pending_rad,
        )
        self.last_turn_angle_deg = theta_deg
        return theta_deg

    def accumulate_turn_from_ex(self, ex: float, ex_raw: Optional[float] = None) -> float:
        """Backward-compat alias: each frame replaces the turn budget (no multi-frame sum)."""
        return self.set_turn_from_ex(ex, ex_raw=ex_raw)

    def peek_turn_wz(self) -> float:
        if not self.angle_servo_enabled:
            return 0.0
        if abs(self.remaining_yaw_rad) <= self.angle_complete_tol_rad:
            return 0.0
        return math.copysign(self.angle_turn_wz, self.remaining_yaw_rad)

    def step_remaining(self, dt: float, ex: Optional[float] = None) -> float:
        """Advance open-loop yaw budget at fixed wz; returns wz for this control tick."""
        if not self.angle_servo_enabled or dt <= 0.0:
            return 0.0
        if (
            ex is not None
            and not self.angle_wait_turn_complete
        ):
            enter_straight = max(0.0, self.center_deadband - self.straight_hysteresis)
            if abs(float(ex)) < enter_straight:
                self.clear_remaining()
                return 0.0
        if abs(self.remaining_yaw_rad) <= self.angle_complete_tol_rad:
            self.remaining_yaw_rad = 0.0
            return 0.0

        wz = math.copysign(self.angle_turn_wz, self.remaining_yaw_rad)
        delta = wz * dt
        next_remaining = self.remaining_yaw_rad - delta
        if self.remaining_yaw_rad > 0.0 and next_remaining <= 0.0:
            self.remaining_yaw_rad = 0.0
        elif self.remaining_yaw_rad < 0.0 and next_remaining >= 0.0:
            self.remaining_yaw_rad = 0.0
        else:
            self.remaining_yaw_rad = next_remaining
        return wz

    def stop_result(
        self,
        state: str,
        reason: str,
        front_distance=None,
        target_distance=None,
        ex: float = 0.0,
        depth_used=None,
    ) -> QwenLidarServoResult:
        self.clear_remaining()
        return QwenLidarServoResult(
            state=state,
            cmd=Twist(),
            ex=float(ex),
            depth_used=depth_used,
            front_distance=front_distance,
            target_distance=target_distance,
            reason=reason,
            wz=0.0,
            turn_angle_deg=0.0,
            remaining_yaw_deg=0.0,
        )

    def _apply_front_speed_scale(self, vx: float, front: Optional[float]) -> float:
        if front is None or vx <= 0.0:
            return vx
        front = float(front)
        if front >= self.slow_distance:
            return vx
        span = max(self.slow_distance - self.hard_stop_distance, 1e-6)
        return min(vx, self.max_vx * (front - self.hard_stop_distance) / span)

    def compute_point_servo(
        self,
        target: Dict[str, Any],
    ) -> tuple[str, str, float, float, float]:
        """Same branching as PointServo.compute_cmd (point_servo.py)."""
        u = target.get("u", target.get("cx"))
        ex = (float(u) - self.image_width / 2.0) / max(float(self.image_width), 1.0)
        abs_ex = abs(ex)
        in_straight = self._update_straight_band_state(abs_ex)

        if in_straight:
            vx = self.creep_vx if self.creep_mode else self.max_vx
            return "FORWARD", "target_in_straight_band", vx, 0.0, ex

        if self.angle_servo_enabled:
            if abs_ex < self.turn_only_threshold:
                vx = self.creep_vx if self.creep_mode else self.steer_vx
                return "FORWARD_STEER", "angle_servo_steer", vx, 0.0, ex
            vx = self.creep_turn_vx if self.creep_mode else self.turn_only_vx
            return "TURN_ONLY", "angle_servo_turn", vx, 0.0, ex

        if self.creep_mode:
            wz = creep_wz_from_ex(ex, self.creep_wz)
            if abs_ex < self.turn_only_threshold:
                return "FORWARD_STEER", "creep_steer", self.creep_vx, wz, ex
            return "TURN_ONLY", "creep_turn", self.creep_turn_vx, wz, ex

        wz = clamp(-self.kp_turn * ex, -self.max_wz, self.max_wz)
        if abs(wz) < self.cmd_wz_deadband:
            wz = 0.0

        if abs_ex < self.turn_only_threshold:
            return "FORWARD_STEER", "target_steer", self.steer_vx, wz, ex
        return "TURN_ONLY", "target_turn_only", self.turn_only_vx, wz, ex

    def _scale_inferred_vx(self, vx: float, inferred_confidence: Optional[float], inferred_vx_scale: float) -> float:
        if vx <= 0.0:
            return vx
        conf = 0.35 if inferred_confidence is None else clamp(float(inferred_confidence), 0.0, 1.0)
        conf_scale = clamp(conf / 0.5, 0.35, 1.0)
        return vx * float(inferred_vx_scale) * conf_scale

    def compute_cmd(
        self,
        target: Dict[str, Any],
        front_distance: Optional[float],
        target_distance: Optional[float],
        arrive_distance: Union[float, None, object] = _ARRIVE_UNSET,
        point_kind: str = "locked",
        inferred_confidence: Optional[float] = None,
        inferred_vx_scale: float = 1.0,
    ) -> QwenLidarServoResult:
        if not target or not target.get("visible", False):
            return self.stop_result("LOST_STOP", "no_qwen_point", front_distance, target_distance)

        u = target.get("u", target.get("cx"))
        if u is None:
            return self.stop_result("LOST_STOP", "missing_u", front_distance, target_distance)

        is_inferred = str(point_kind).lower() == "inferred" or target.get("point_kind") == "inferred"
        state, reason, vx, wz, ex = self.compute_point_servo(target)
        if is_inferred:
            state = f"INFERRED_{state}"
            reason = f"inferred_{reason}"
        turn_angle_deg = 0.0
        if self.angle_servo_enabled:
            ex_raw = None
            raw_u = target.get("raw_u", target.get("u"))
            if raw_u is not None:
                ex_raw = (float(raw_u) - self.image_width / 2.0) / max(float(self.image_width), 1.0)
            steer_ex = self._resolve_steer_ex(ex, ex_raw=ex_raw)
            turn_angle_deg = self.set_turn_from_ex(ex, ex_raw=ex_raw)
            wz = self.peek_turn_wz()
            # Large offset: turn in place. Smaller offset: keep FORWARD_STEER (walk while turning).
            if (turn_angle_deg > 0.0 or self.is_turn_busy()) and abs(steer_ex) >= self.turn_only_threshold:
                vx = 0.0
                state = "TURN_ONLY"
                reason = "angle_servo_turn_large"
            elif turn_angle_deg > 0.0 or self.is_turn_busy():
                reason = "angle_servo_steer_forward"

        if is_inferred:
            safety_dist = front_distance
            arrive_dist = None
        else:
            safety_dist = target_distance if target_distance is not None else front_distance
            if arrive_distance is _ARRIVE_UNSET:
                arrive_dist = safety_dist
            else:
                # Explicit None from nav (lidar_arrive_enable=false) must skip ARRIVED.
                arrive_dist = arrive_distance

        if front_distance is not None and float(front_distance) <= self.emergency_stop_distance:
            return self.stop_result(
                "EMERGENCY_STOP", "front_emergency", front_distance, target_distance, ex=ex
            )
        if not is_inferred and arrive_dist is not None and float(arrive_dist) <= self.arrive_distance:
            return self.stop_result(
                "ARRIVED",
                "target_lidar_arrive",
                front_distance,
                arrive_dist,
                ex=ex,
                depth_used=arrive_dist,
            )
        if safety_dist is not None and float(safety_dist) <= self.hard_stop_distance:
            return self.stop_result(
                "OBSTACLE_STOP", "front_too_close", front_distance, target_distance, ex=ex
            )
        if self.require_lidar and safety_dist is None and not self.allow_track_without_depth:
            return self.stop_result(
                "DEPTH_STOP", "depth_unknown_stop", front_distance, target_distance, ex=ex
            )

        if self.require_lidar and safety_dist is None and vx > 0.0:
            vx = min(vx, self.forward_vx_no_depth)
            if state.endswith("FORWARD"):
                reason = "inferred_no_depth" if is_inferred else "target_in_straight_band_no_depth"

        if is_inferred:
            vx = self._scale_inferred_vx(vx, inferred_confidence, inferred_vx_scale)

        vx = self._apply_front_speed_scale(vx, safety_dist)
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        return QwenLidarServoResult(
            state,
            cmd,
            ex,
            safety_dist,
            front_distance,
            target_distance,
            reason,
            wz=float(wz),
            turn_angle_deg=float(turn_angle_deg),
            remaining_yaw_deg=self.remaining_yaw_deg(),
        )
