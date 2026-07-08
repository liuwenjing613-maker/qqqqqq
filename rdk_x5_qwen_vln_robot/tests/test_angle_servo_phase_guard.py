#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from geometry_msgs.msg import Twist

from src.apps.run_qwen_api_lidar_nav import RunQwenApiLidarNav
from src.control.qwen_lidar_point_servo import QwenLidarPointServo


def _nav_node_stub():
    node = object.__new__(RunQwenApiLidarNav)
    node.control_hz = 20.0
    node._last_control_time = None
    node.desired_reason = "EXPLORE_FORWARD"
    node.last_target_ex = 0.12
    node.servo = QwenLidarPointServo(
        image_width=1280,
        require_lidar=False,
        angle_servo_enabled=True,
        angle_turn_wz=0.04,
        angle_max_turn_deg=1.0,
        angle_absolute_cap_deg=1.0,
        angle_wait_turn_complete=True,
    )
    # Simulate leftover 1° budget from target tracking.
    node.servo.set_turn_from_ex(0.12)
    assert node.servo.is_turn_busy()
    return node


def test_angle_servo_skipped_during_explore_forward():
    node = _nav_node_stub()
    cmd = Twist()
    cmd.linear.x = 0.06
    cmd.angular.z = 0.0
    node._apply_angle_servo_wz(cmd)
    assert cmd.angular.z == 0.0


def test_angle_servo_applies_during_target_steer():
    node = _nav_node_stub()
    node.desired_reason = "FORWARD_STEER"
    cmd = Twist()
    cmd.linear.x = 0.04
    cmd.angular.z = 0.0
    node._apply_angle_servo_wz(cmd)
    assert cmd.angular.z != 0.0


if __name__ == "__main__":
    test_angle_servo_skipped_during_explore_forward()
    test_angle_servo_applies_during_target_steer()
    print("ok")
