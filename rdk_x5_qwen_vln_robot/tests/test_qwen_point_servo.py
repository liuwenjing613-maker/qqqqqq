#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.control.qwen_lidar_point_servo import QwenLidarPointServo, turn_dir_from_ex


def test_left_target_positive_wz():
    servo = QwenLidarPointServo(image_width=1280, kp_turn=0.1, max_wz=0.05, require_lidar=False)
    res = servo.compute_cmd({"visible": True, "u": 400.0}, None, None)
    assert res.ex < 0
    assert res.wz > 0.0
    assert res.cmd.angular.z > 0.0


def test_right_target_negative_wz():
    servo = QwenLidarPointServo(image_width=1280, kp_turn=0.1, max_wz=0.05, require_lidar=False)
    res = servo.compute_cmd({"visible": True, "u": 900.0}, None, None)
    assert res.ex > 0
    assert res.wz < 0.0
    assert res.cmd.angular.z < 0.0


def test_search_turn_dir_matches_servo():
    assert turn_dir_from_ex(-0.2) > 0
    assert turn_dir_from_ex(0.2) < 0


def test_creep_mode_fixed_speed():
    servo = QwenLidarPointServo(
        image_width=1280,
        require_lidar=False,
        creep_mode=True,
        creep_vx=0.012,
        creep_wz=0.012,
        center_deadband=0.06,
        turn_only_threshold=0.4,
    )
    res = servo.compute_cmd({"visible": True, "u": 100.0}, None, None)
    assert res.state == "TURN_ONLY"
    assert res.cmd.linear.x == 0.0
    assert abs(res.cmd.angular.z - 0.012) < 1e-6

    res2 = servo.compute_cmd({"visible": True, "u": 1100.0}, None, None)
    assert res2.cmd.angular.z == -0.012

    res3 = servo.compute_cmd({"visible": True, "u": 640.0}, None, None)
    assert res3.state == "FORWARD"
    assert res3.cmd.linear.x == 0.012
    assert res3.cmd.angular.z == 0.0


def test_straight_band_no_turn():
    servo = QwenLidarPointServo(
        image_width=1280,
        require_lidar=False,
        creep_mode=True,
        center_deadband=0.10,
        straight_hysteresis=0.02,
        creep_vx=0.06,
        creep_wz=0.05,
    )
    # u=700 -> ex≈0.047, inside 0.10 band => straight, wz=0
    res = servo.compute_cmd({"visible": True, "u": 700.0}, None, None)
    assert res.state == "FORWARD"
    assert res.cmd.angular.z == 0.0
    assert res.cmd.linear.x == 0.06

    # u=850 -> ex≈0.16, outside band => steer
    res2 = servo.compute_cmd({"visible": True, "u": 850.0}, None, None)
    assert res2.state == "FORWARD_STEER"
    assert res2.cmd.angular.z != 0.0


def test_straight_band_hysteresis():
    servo = QwenLidarPointServo(
        image_width=1280,
        require_lidar=False,
        center_deadband=0.10,
        straight_hysteresis=0.02,
    )
    # Start straight at center
    servo.compute_cmd({"visible": True, "u": 640.0}, None, None)
    # ex=0.09 still inside enter_steer threshold 0.12 while in straight mode
    res = servo.compute_cmd({"visible": True, "u": 755.0}, None, None)
    assert res.state == "FORWARD"
    assert res.cmd.angular.z == 0.0
    # ex=0.13 exceeds 0.12 => start steering
    res2 = servo.compute_cmd({"visible": True, "u": 806.0}, None, None)
    assert res2.state == "FORWARD_STEER"


def test_arrive_at_target_distance():
    servo = QwenLidarPointServo(
        image_width=1280,
        require_lidar=False,
        creep_mode=True,
        arrive_distance=0.6,
        hard_stop_distance=0.42,
    )
    res = servo.compute_cmd({"visible": True, "u": 640.0}, 2.0, 0.55)
    assert res.state == "ARRIVED"
    assert res.cmd.linear.x == 0.0
    assert res.cmd.angular.z == 0.0
    assert res.reason == "target_lidar_arrive"

    res2 = servo.compute_cmd({"visible": True, "u": 640.0}, 2.0, 0.75)
    assert res2.state != "ARRIVED"
    assert res2.cmd.linear.x > 0.0


if __name__ == "__main__":
    test_left_target_positive_wz()
    test_right_target_negative_wz()
    test_search_turn_dir_matches_servo()
    test_creep_mode_fixed_speed()
    test_straight_band_no_turn()
    test_straight_band_hysteresis()
    test_arrive_at_target_distance()
    print("PASS test_qwen_point_servo")
