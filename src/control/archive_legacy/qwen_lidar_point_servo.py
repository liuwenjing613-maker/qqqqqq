#!/usr/bin/env python3
"""
Independent point visual servo for Qwen-only navigation.
"""
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from geometry_msgs.msg import Twist


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class QwenLidarServoResult:
    state: str
    cmd: Twist
    ex: float
    depth_used: Optional[float]
    front_distance: Optional[float]
    target_distance: Optional[float]
    reason: str


class QwenLidarPointServo:
    def __init__(
        self,
        image_width: int = 1280,
        require_lidar: bool = True,
        kp_turn: float = 0.11,
        max_wz: float = 0.050,
        wz_deadzone: float = 0.07,
        cmd_wz_deadzone: float = 0.012,
        turn_threshold: float = 0.24,
        forward_turn_scale: float = 0.45,
        max_vx: float = 0.030,
        mid_vx: float = 0.022,
        slow_vx: float = 0.014,
        min_vx: float = 0.010,
        emergency_stop_distance: float = 0.22,
        hard_stop_distance: float = 0.32,
        slow_distance: float = 0.55,
        normal_distance: float = 0.90,
    ):
        self.image_width = int(image_width)
        self.require_lidar = bool(require_lidar)
        self.kp_turn = float(kp_turn)
        self.max_wz = float(max_wz)
        self.wz_deadzone = float(wz_deadzone)
        self.cmd_wz_deadzone = float(cmd_wz_deadzone)
        self.turn_threshold = float(turn_threshold)
        self.forward_turn_scale = float(forward_turn_scale)
        self.max_vx = float(max_vx)
        self.mid_vx = float(mid_vx)
        self.slow_vx = float(slow_vx)
        self.min_vx = float(min_vx)
        self.emergency_stop_distance = float(emergency_stop_distance)
        self.hard_stop_distance = float(hard_stop_distance)
        self.slow_distance = float(slow_distance)
        self.normal_distance = float(normal_distance)

    def update_image_width(self, image_width: int) -> None:
        self.image_width = int(image_width)

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
        )

    def scan_result(self, cmd: Twist, front_distance, target_distance, reason: str) -> QwenLidarServoResult:
        return QwenLidarServoResult(
            state="SEARCH_SCAN",
            cmd=cmd,
            ex=0.0,
            depth_used=front_distance,
            front_distance=front_distance,
            target_distance=target_distance,
            reason=reason,
        )

    def _turn_wz(self, ex: float, *, turn_in_place: bool) -> float:
        scale = 1.0 if turn_in_place else self.forward_turn_scale
        wz = -self.kp_turn * ex * scale
        wz = clamp(wz, -self.max_wz, self.max_wz)
        if abs(wz) < self.cmd_wz_deadzone:
            wz = 0.0
        return float(wz)

    def _vx_from_depth(self, depth: Optional[float]) -> Tuple[float, str]:
        if depth is None:
            if self.require_lidar:
                return 0.0, "depth_unknown_stop"
            return self.slow_vx, "depth_unknown_slow"
        depth = float(depth)
        if depth <= self.emergency_stop_distance:
            return 0.0, "emergency_stop"
        if depth <= self.hard_stop_distance:
            return 0.0, "hard_stop"
        if depth <= self.slow_distance:
            return self.slow_vx, "slow"
        if depth <= self.normal_distance:
            return self.mid_vx, "mid"
        return self.max_vx, "normal"

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

        ex = (float(u) - self.image_width / 2.0) / max(1.0, float(self.image_width))

        if front_distance is not None and float(front_distance) <= self.hard_stop_distance:
            return self.stop_result(
                "OBSTACLE_STOP",
                "front_too_close",
                front_distance,
                target_distance,
                ex=ex,
                depth_used=front_distance,
            )

        if abs(ex) >= self.turn_threshold:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self._turn_wz(ex, turn_in_place=True)
            return QwenLidarServoResult(
                "TURN_ONLY",
                cmd,
                ex,
                front_distance,
                front_distance,
                target_distance,
                "large_lateral_error",
            )

        depth_used = target_distance if target_distance is not None else front_distance
        vx, speed_reason = self._vx_from_depth(depth_used)
        cmd = Twist()
        cmd.linear.x = float(vx)

        if vx <= 0.0:
            state = "DEPTH_STOP"
            cmd.angular.z = 0.0
        elif abs(ex) < self.wz_deadzone:
            state = "FORWARD"
            cmd.angular.z = 0.0
        else:
            state = "FORWARD_STEER"
            cmd.angular.z = self._turn_wz(ex, turn_in_place=False)

        return QwenLidarServoResult(state, cmd, ex, depth_used, front_distance, target_distance, speed_reason)
