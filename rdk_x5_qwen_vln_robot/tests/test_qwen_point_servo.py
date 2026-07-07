#!/usr/bin/env python3
import math
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.control.qwen_lidar_point_servo import (
    QwenLidarPointServo,
    turn_angle_deg_from_ex,
    turn_dir_from_ex,
)


def _angle_servo(**kwargs):
    defaults = dict(
        image_width=1280,
        require_lidar=False,
        creep_mode=True,
        creep_vx=0.04,
        center_deadband=0.10,
        straight_hysteresis=0.02,
        turn_only_threshold=0.4,
        angle_servo_enabled=True,
        angle_gain=120.0,
        angle_power=0.85,
        angle_max_turn_deg=30.0,
        angle_turn_wz=0.04,
        angle_complete_tol_deg=0.5,
        angle_max_pending_deg=45.0,
        angle_absolute_cap_deg=1.0,
        angle_wait_turn_complete=True,
    )
    defaults.update(kwargs)
    return QwenLidarPointServo(**defaults)


def test_scheme_b_turn_angle_mapping():
    assert turn_angle_deg_from_ex(0.10) == 0.0
    theta_20 = turn_angle_deg_from_ex(0.20)
    assert 15.0 <= theta_20 <= 17.5
    theta_30 = turn_angle_deg_from_ex(0.30)
    assert theta_30 == 30.0
    assert turn_angle_deg_from_ex(0.50) == 30.0


def test_left_target_positive_wz():
    servo = _angle_servo()
    res = servo.compute_cmd({"visible": True, "u": 400.0}, None, None)
    assert res.ex < 0
    assert res.turn_angle_deg > 0.0
    assert res.remaining_yaw_deg > 0.0
    wz = servo.step_remaining(0.05)
    assert wz > 0.0


def test_right_target_negative_wz():
    servo = _angle_servo()
    res = servo.compute_cmd({"visible": True, "u": 900.0}, None, None)
    assert res.ex > 0
    assert res.turn_angle_deg > 0.0
    assert res.remaining_yaw_deg < 0.0
    wz = servo.step_remaining(0.05)
    assert wz < 0.0


def test_search_turn_dir_matches_servo():
    assert turn_dir_from_ex(-0.2) > 0
    assert turn_dir_from_ex(0.2) < 0


def test_angle_servo_turn_only_large_offset():
    servo = _angle_servo()
    res = servo.compute_cmd({"visible": True, "u": 100.0}, None, None)
    assert res.state == "TURN_ONLY"
    assert res.cmd.linear.x == 0.0
    assert res.turn_angle_deg == 1.0
    assert res.remaining_yaw_deg > 0.0


def test_straight_band_no_turn():
    servo = _angle_servo(creep_vx=0.06)
    res = servo.compute_cmd({"visible": True, "u": 700.0}, None, None)
    assert res.state == "FORWARD"
    assert res.turn_angle_deg == 0.0
    assert res.remaining_yaw_deg == 0.0
    assert res.cmd.angular.z == 0.0
    assert res.cmd.linear.x == 0.06

    res2 = servo.compute_cmd({"visible": True, "u": 850.0}, None, None)
    assert res2.state == "FORWARD_STEER"
    assert res2.turn_angle_deg > 0.0
    assert res2.cmd.linear.x > 0.0


def test_straight_band_hysteresis():
    servo = _angle_servo()
    servo.compute_cmd({"visible": True, "u": 640.0}, None, None)
    res = servo.compute_cmd({"visible": True, "u": 755.0}, None, None)
    assert res.state == "FORWARD"
    assert res.turn_angle_deg == 0.0
    res2 = servo.compute_cmd({"visible": True, "u": 806.0}, None, None)
    assert res2.state == "FORWARD_STEER"
    assert res2.turn_angle_deg > 0.0
    assert res2.cmd.linear.x > 0.0


def test_steer_forward_while_turning():
    servo = _angle_servo(creep_vx=0.06)
    res = servo.compute_cmd({"visible": True, "u": 480.0}, None, None)
    assert res.state == "FORWARD_STEER"
    assert res.turn_angle_deg > 0.0
    assert res.cmd.linear.x == 0.06
    assert res.remaining_yaw_deg > 0.0


def test_replace_not_accumulate():
    servo = _angle_servo(angle_wait_turn_complete=True)
    r1 = servo.compute_cmd({"visible": True, "u": 400.0}, None, None)
    deg1 = abs(r1.remaining_yaw_deg)
    assert deg1 > 0.0
    servo.step_remaining(0.05)
    assert abs(servo.remaining_yaw_deg()) < deg1
    r2 = servo.compute_cmd({"visible": True, "u": 300.0}, None, None)
    assert abs(r2.remaining_yaw_deg) == abs(servo.remaining_yaw_deg())
    assert r2.turn_angle_deg == r1.turn_angle_deg


def test_skip_new_angle_while_turning():
    servo = _angle_servo(angle_wait_turn_complete=True)
    r1 = servo.compute_cmd({"visible": True, "u": 400.0, "raw_u": 400.0}, None, None)
    assert abs(r1.remaining_yaw_deg) > 0.0
    r2 = servo.compute_cmd({"visible": True, "u": 380.0, "raw_u": 380.0}, None, None)
    assert abs(r2.remaining_yaw_deg) == abs(r1.remaining_yaw_deg)


def test_absolute_cap_one_degree():
    servo = _angle_servo(angle_max_turn_deg=30.0, angle_absolute_cap_deg=1.0)
    res = servo.compute_cmd({"visible": True, "u": 100.0}, None, None)
    assert res.turn_angle_deg <= 1.0 + 1e-6
    assert abs(res.remaining_yaw_deg) <= 1.0 + 1e-6


def test_finish_turn_before_accepting_new_budget():
    servo = _angle_servo(angle_wait_turn_complete=True)
    r1 = servo.compute_cmd({"visible": True, "u": 900.0}, None, None)
    assert abs(r1.remaining_yaw_deg) > 0.0
    while servo.is_turn_busy():
        servo.step_remaining(0.05)
    r2 = servo.compute_cmd({"visible": True, "u": 850.0}, None, None)
    assert r2.turn_angle_deg > 0.0
    assert abs(r2.remaining_yaw_deg) > 0.0


def test_raw_ex_overrides_lagging_filter():
    servo = _angle_servo(angle_wait_turn_complete=False, angle_max_turn_deg=1.0)
    # filtered left (ex<0) but raw far right (ex>0) -> should turn left (wz<0), not right
    res = servo.compute_cmd(
        {"visible": True, "u": 470.0, "raw_u": 1000.0, "cx": 470.0},
        None,
        None,
    )
    assert res.ex < 0
    assert res.remaining_yaw_deg < 0.0


def test_step_remaining_does_not_abort_while_wait_complete():
    servo = _angle_servo(angle_wait_turn_complete=True)
    servo.compute_cmd({"visible": True, "u": 400.0}, None, None)
    assert abs(servo.remaining_yaw_rad) > 0.0
    wz = servo.step_remaining(0.05, ex=0.02)
    assert wz != 0.0
    assert abs(servo.remaining_yaw_rad) > servo.angle_complete_tol_rad


def test_remaining_yaw_steps_down():
    servo = _angle_servo()
    servo.compute_cmd({"visible": True, "u": 900.0}, None, None)
    start = abs(servo.remaining_yaw_rad)
    assert start > 0.0
    total = 0.0
    for _ in range(200):
        wz = servo.step_remaining(0.05)
        total += abs(wz * 0.05)
        if abs(servo.remaining_yaw_rad) <= servo.angle_complete_tol_rad:
            break
    assert total > 0.0
    assert abs(servo.remaining_yaw_rad) <= servo.angle_complete_tol_rad + 1e-6


def test_arrive_at_target_distance():
    servo = _angle_servo(arrive_distance=0.6, hard_stop_distance=0.42)
    res = servo.compute_cmd({"visible": True, "u": 640.0}, 2.0, 0.55)
    assert res.state == "ARRIVED"
    assert res.cmd.linear.x == 0.0
    assert res.remaining_yaw_deg == 0.0
    assert res.reason == "target_lidar_arrive"

    res2 = servo.compute_cmd({"visible": True, "u": 640.0}, 2.0, 0.75)
    assert res2.state != "ARRIVED"
    assert res2.cmd.linear.x > 0.0


def test_creep_mode_fixed_speed_without_angle_servo():
    servo = QwenLidarPointServo(
        image_width=1280,
        require_lidar=False,
        creep_mode=True,
        creep_vx=0.012,
        creep_wz=0.012,
        center_deadband=0.06,
        turn_only_threshold=0.4,
        angle_servo_enabled=False,
    )
    res = servo.compute_cmd({"visible": True, "u": 100.0}, None, None)
    assert res.state == "TURN_ONLY"
    assert res.cmd.linear.x == 0.0
    assert abs(res.cmd.angular.z - 0.012) < 1e-6


def test_inferred_does_not_arrive_at_target_distance():
    servo = _angle_servo(arrive_distance=0.6, hard_stop_distance=0.42)
    res = servo.compute_cmd(
        {"visible": True, "u": 640.0, "point_kind": "inferred"},
        2.0,
        0.55,
        arrive_distance=0.55,
        point_kind="inferred",
        inferred_confidence=0.4,
        inferred_vx_scale=0.85,
    )
    assert res.state.startswith("INFERRED_")
    assert res.state != "ARRIVED"
    assert res.cmd.linear.x > 0.0


if __name__ == "__main__":
    test_scheme_b_turn_angle_mapping()
    test_left_target_positive_wz()
    test_right_target_negative_wz()
    test_search_turn_dir_matches_servo()
    test_angle_servo_turn_only_large_offset()
    test_straight_band_no_turn()
    test_straight_band_hysteresis()
    test_steer_forward_while_turning()
    test_replace_not_accumulate()
    test_skip_new_angle_while_turning()
    test_absolute_cap_one_degree()
    test_finish_turn_before_accepting_new_budget()
    test_raw_ex_overrides_lagging_filter()
    test_step_remaining_does_not_abort_while_wait_complete()
    test_remaining_yaw_steps_down()
    test_arrive_at_target_distance()
    test_inferred_does_not_arrive_at_target_distance()
    test_creep_mode_fixed_speed_without_angle_servo()
    print("PASS test_qwen_point_servo")
