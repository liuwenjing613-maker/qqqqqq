#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple

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
    """
    Qwen-only 独立点视觉伺服：
    - Qwen 输出 u/v
    - u 用于转向
    - 雷达 front/target distance 用于调 vx 和安全停
    """

    def __init__(
        self,
        image_width: int = 1280,
        kp_turn: float = 0.11,
        max_wz: float = 0.060,
        wz_deadzone: float = 0.07,
        cmd_wz_deadzone: float = 0.012,
        turn_threshold: float = 0.24,
        forward_turn_scale: float = 0.45,
        max_vx: float = 0.040,
        mid_vx: float = 0.030,
        slow_vx: float = 0.018,
        min_vx: float = 0.012,
        emergency_stop_distance: float = 0.22,
        hard_stop_distance: float = 0.32,
        slow_distance: float = 0.55,
        normal_distance: float = 0.90,
    ):
        self.image_width = int(image_width)

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

    def _make_stop(self, state: str, ex: float, depth_used, front, target, reason: str) -> QwenLidarServoResult:
        return QwenLidarServoResult(
            state=state,
            cmd=Twist(),
            ex=ex,
            depth_used=depth_used,
            front_distance=front,
            target_distance=target,
            reason=reason,
        )

    def _turn_wz(self, ex: float, turn_in_place: bool) -> float:
        scale = 1.0 if turn_in_place else self.forward_turn_scale
        wz = -self.kp_turn * ex * scale
        wz = clamp(wz, -self.max_wz, self.max_wz)
        if abs(wz) < self.cmd_wz_deadzone:
            wz = 0.0
        return wz

    def _vx_from_depth(self, depth: Optional[float]) -> Tuple[float, str]:
        if depth is None:
            return self.slow_vx, "depth_unknown_slow"

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
            return self._make_stop("LOST_STOP", 0.0, None, front_distance, target_distance, "no_qwen_point")

        u = target.get("u", target.get("cx"))
        if u is None:
            return self._make_stop("LOST_STOP", 0.0, None, front_distance, target_distance, "missing_u")

        ex = (float(u) - self.image_width / 2.0) / max(1.0, self.image_width)

        # 安全优先：正前方过近，任何 Qwen 点都不能让车继续冲
        if front_distance is not None and front_distance <= self.hard_stop_distance:
            return self._make_stop("OBSTACLE_STOP", ex, front_distance, front_distance, target_distance, "front_too_close")

        # 转向误差大：原地转，不前进
        if abs(ex) >= self.turn_threshold:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self._turn_wz(ex, turn_in_place=True)
            return QwenLidarServoResult(
                state="TURN_ONLY",
                cmd=cmd,
                ex=ex,
                depth_used=front_distance,
                front_distance=front_distance,
                target_distance=target_distance,
                reason="large_lateral_error",
            )

        # 居中或小偏差：允许前进，速度由雷达深度决定
        depth_used = target_distance if target_distance is not None else front_distance
        vx, speed_reason = self._vx_from_depth(depth_used)

        cmd = Twist()
        cmd.linear.x = float(vx)

        if abs(ex) < self.wz_deadzone:
            cmd.angular.z = 0.0
            state = "FORWARD"
        else:
            cmd.angular.z = self._turn_wz(ex, turn_in_place=False)
            state = "FORWARD_STEER"

        if vx <= 0.0:
            state = "DEPTH_STOP"

        return QwenLidarServoResult(
            state=state,
            cmd=cmd,
            ex=ex,
            depth_used=depth_used,
            front_distance=front_distance,
            target_distance=target_distance,
            reason=speed_reason,
        )