#!/usr/bin/env python3
from geometry_msgs.msg import Twist


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class MVPVisualServo:
    """
    标准化视觉伺服：
      ex = (cx - 画面中心) / 画面宽度
      ex < 0 → 目标在左，wz > 0 左转
      ex > 0 → 目标在右，wz < 0 右转
      |ex| < wz_deadzone → 不转，只慢速前进
      wz_deadzone <= |ex| < turn_threshold → 边前进边小角速度修正
      |ex| >= turn_threshold → 原地慢转
    """

    def __init__(
        self,
        image_width=1280,
        kp_turn=0.08,
        max_vx=0.01,
        max_wz=0.16,
        center_threshold=0.28,
        arrive_area_ratio=0.12,
        turn_threshold=None,
        forward_threshold=None,
        wz_deadzone=0.0,
        forward_turn_scale=1.0,
        slowdown_area_ratio=None,
        min_cruise_wz=0.16,
        cmd_wz_deadzone=0.01,
    ):
        self.image_width = image_width
        self.kp_turn = kp_turn
        self.max_vx = max_vx
        self.max_wz = max_wz
        self.turn_threshold = float(
            turn_threshold if turn_threshold is not None else center_threshold
        )
        self.wz_deadzone = float(wz_deadzone)
        self.forward_turn_scale = float(forward_turn_scale)
        self.cmd_wz_deadzone = float(cmd_wz_deadzone)
        self.arrive_area_ratio = arrive_area_ratio
        # 保留入参兼容旧配置；启动死区由底盘桥 kick 解决，伺服只给真实巡航速度。
        self.min_cruise_wz = float(min_cruise_wz)

    def _scale_turn_wz(self, ex, turn_in_place=False):
        """比例转向；不在伺服层抬速度，避免小偏差也猛转。"""
        scale = 1.0 if turn_in_place else self.forward_turn_scale
        raw = -self.kp_turn * ex * scale
        if abs(raw) < 1e-4:
            return 0.0
        return raw

    def compute_cmd(self, target):
        msg = Twist()

        if not target or not target.get("visible", False):
            return "LOST_STOP", msg

        cx = target["cx"]
        area_ratio = target["area_ratio"]
        ex = (cx - self.image_width / 2.0) / self.image_width

        if area_ratio >= self.arrive_area_ratio:
            state = "ARRIVED_STOP"
            vx = 0.0
            wz = 0.0
        elif abs(ex) < self.wz_deadzone:
            state = "FORWARD"
            vx = self.max_vx
            wz = 0.0
        elif abs(ex) >= self.turn_threshold:
            state = "TURN_ONLY"
            vx = 0.0
            wz = self._scale_turn_wz(ex, turn_in_place=True)
        else:
            state = "FORWARD"
            vx = self.max_vx
            wz = self._scale_turn_wz(ex, turn_in_place=False)

        wz = clamp(wz, -self.max_wz, self.max_wz)
        if abs(wz) < self.cmd_wz_deadzone:
            wz = 0.0

        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        return state, msg
