#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.control.point_servo import PointServo, PointServoConfig


def test_lost_target_stops():
    servo = PointServo(PointServoConfig(image_width=640))
    res = servo.compute_cmd({"visible": False})
    assert res.state == "LOST_STOP"
    assert res.cmd.vx == 0.0
    assert res.cmd.wz == 0.0


def test_centered_target_moves_forward():
    servo = PointServo(PointServoConfig(image_width=640, center_deadband=0.06))
    res = servo.compute_cmd({"visible": True, "u": 320})
    assert res.state == "FORWARD"
    assert res.cmd.vx > 0.0
    assert res.cmd.wz == 0.0


def test_left_target_turns_positive_wz():
    servo = PointServo(PointServoConfig(image_width=640, kp_turn=0.12, max_wz=0.06))
    res = servo.compute_cmd({"visible": True, "u": 120})
    assert res.state == "TURN_ONLY"
    assert res.cmd.vx == 0.0
    assert res.cmd.wz > 0.0


def test_right_target_turns_negative_wz():
    servo = PointServo(PointServoConfig(image_width=640, kp_turn=0.12, max_wz=0.06))
    res = servo.compute_cmd({"visible": True, "u": 520})
    assert res.state == "TURN_ONLY"
    assert res.cmd.vx == 0.0
    assert res.cmd.wz < 0.0


def test_moderate_error_forward_steer():
    servo = PointServo(PointServoConfig(image_width=640, center_deadband=0.03, turn_only_threshold=0.20))
    res = servo.compute_cmd({"visible": True, "u": 390})
    assert res.state == "FORWARD_STEER"
    assert res.cmd.vx > 0.0
    assert res.cmd.wz < 0.0


if __name__ == "__main__":
    test_lost_target_stops()
    test_centered_target_moves_forward()
    test_left_target_turns_positive_wz()
    test_right_target_turns_negative_wz()
    test_moderate_error_forward_steer()
    print("PASS test_point_servo")
