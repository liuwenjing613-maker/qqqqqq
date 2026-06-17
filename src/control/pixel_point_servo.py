#!/usr/bin/env python3
from geometry_msgs.msg import Twist


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class PixelPointServo:
    """
    纯点视觉伺服：只根据目标点 u（横向像素）控制转向/前进。
    不依赖 bbox 或 area_ratio；到达判定由上层 VLM verify 负责。
    """

    def __init__(
        self,
        image_width=1280,
        kp_turn=0.09,
        max_vx=0.035,
        max_wz=0.045,
        wz_deadzone=0.08,
        turn_threshold=0.28,
        forward_turn_scale=0.45,
        cmd_wz_deadzone=0.012,
    ):
        self.image_width = int(image_width)
        self.kp_turn = float(kp_turn)
        self.max_vx = float(max_vx)
        self.max_wz = float(max_wz)
        self.wz_deadzone = float(wz_deadzone)
        self.turn_threshold = float(turn_threshold)
        self.forward_turn_scale = float(forward_turn_scale)
        self.cmd_wz_deadzone = float(cmd_wz_deadzone)

    def _scale_turn_wz(self, ex, turn_in_place=False):
        scale = 1.0 if turn_in_place else self.forward_turn_scale
        raw = -self.kp_turn * ex * scale
        if abs(raw) < 1e-4:
            return 0.0
        return raw

    def compute_cmd(self, target):
        msg = Twist()

        if not target or not target.get("visible", False):
            return "LOST_STOP", msg

        u = target.get("u", target.get("cx"))
        if u is None:
            return "LOST_STOP", msg

        ex = (float(u) - self.image_width / 2.0) / self.image_width

        if abs(ex) < self.wz_deadzone:
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
