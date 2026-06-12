#!/usr/bin/env python3
from geometry_msgs.msg import Twist


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class MVPVisualServo:
    def __init__(
        self,
        image_width=1280,
        kp_turn=0.08,
        max_vx=0.01,
        max_wz=0.16,
        center_threshold=0.28,
        arrive_area_ratio=0.12,
    ):
        self.image_width = image_width
        self.kp_turn = kp_turn
        self.max_vx = max_vx
        self.max_wz = max_wz
        self.center_threshold = center_threshold
        self.arrive_area_ratio = arrive_area_ratio

    def compute_cmd(self, target):
        """
        输入统一 target dict。
        输出：
        state, Twist
        """
        msg = Twist()

        if not target or not target.get("visible", False):
            return "LOST_STOP", msg

        cx = target["cx"]
        area_ratio = target["area_ratio"]

        ex = (cx - self.image_width / 2.0) / self.image_width

        if area_ratio >= self.arrive_area_ratio:
            vx = 0.0
            wz = 0.0
            state = "ARRIVED_STOP"
        elif abs(ex) > self.center_threshold:
            vx = 0.0
            wz = -self.kp_turn * ex
            state = "TURN_ONLY"
        else:
            vx = self.max_vx
            wz = -self.kp_turn * ex
            state = "FORWARD"

        wz = clamp(wz, -self.max_wz, self.max_wz)

        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)

        return state, msg
