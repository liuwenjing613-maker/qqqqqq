#!/usr/bin/env python3
"""Point visual servo: same steering law as rdk_x5_vln_robot PointServo + LiDAR safety."""
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional

from geometry_msgs.msg import Twist


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def turn_dir_from_ex(ex: float) -> float:
    """Match src/nav/search_strategy.py used by YOLO shared_nav."""
    if abs(ex) < 1e-6:
        return 1.0
    return -1.0 if ex > 0 else 1.0


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
        self._in_straight_band = True

    def update_image_width(self, image_width: int) -> None:
        self.image_width = int(image_width)

    def straight_band_px(self) -> float:
        """Half-width of straight-only zone in pixels (|u-cx| below this => no turn)."""
        return self.center_deadband * float(self.image_width)

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

    def stop_result(
        self,
        state: str,
        reason: str,
        front_distance=None,
        target_distance=None,
        ex: float = 0.0,
        depth_used=None,
    ) -> QwenLidarServoResult:
        return QwenLidarServoResult(
            state=state,
            cmd=Twist(),
            ex=float(ex),
            depth_used=depth_used,
            front_distance=front_distance,
            target_distance=target_distance,
            reason=reason,
            wz=0.0,
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

    def compute_cmd(
        self,
        target: Dict[str, Any],
        front_distance: Optional[float],
        target_distance: Optional[float],
    ) -> QwenLidarServoResult:
        if not target or not target.get("visible", False):
            return self.stop_result("LOST_STOP", "no_qwen_point", front_distance, target_distance)

        u = target.get("u", target.get("cx"))
        if u is None:
            return self.stop_result("LOST_STOP", "missing_u", front_distance, target_distance)

        state, reason, vx, wz, ex = self.compute_point_servo(target)
        safety_dist = target_distance if target_distance is not None else front_distance

        if front_distance is not None and float(front_distance) <= self.emergency_stop_distance:
            return self.stop_result(
                "EMERGENCY_STOP", "front_emergency", front_distance, target_distance, ex=ex
            )
        if target_distance is not None and float(target_distance) <= self.arrive_distance:
            return self.stop_result(
                "ARRIVED",
                "target_lidar_arrive",
                front_distance,
                target_distance,
                ex=ex,
                depth_used=target_distance,
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
            if state == "FORWARD":
                reason = "target_in_straight_band_no_depth"

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
        )
