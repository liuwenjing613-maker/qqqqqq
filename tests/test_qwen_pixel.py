#!/usr/bin/env python3
import os
import sys

import pytest

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.pixel_point_servo import PixelPointServo
from src.vlm.qwen_ollama_client import parse_nav_result


def test_parse_nav_result_normalizes_enum_template():
    raw = {
        "status": "target_locked/searching/success/unsafe",
        "target_visible": True,
        "u": 430,
        "v": 512,
        "confidence": 0.98,
        "stop": False,
    }
    out = parse_nav_result(raw, orig_w=1280, orig_h=1707, model_w=256, model_h=341, sx=5.0, sy=5.0)
    assert out["status"] == "target_locked"
    assert out["u"] == 430


def test_parse_nav_result_orig_coords():
    raw = {
        "status": "target_locked",
        "target_visible": True,
        "u": 640,
        "v": 360,
        "confidence": 0.8,
        "stop": False,
    }
    out = parse_nav_result(raw, orig_w=1280, orig_h=720, model_w=256, model_h=144, sx=5.0, sy=5.0)
    assert out["u"] == 640
    assert out["v"] == 360
    assert out["cx"] == 640
    assert out["target_visible"] is True
    assert out["_point_valid"] is True


def test_parse_nav_result_scales_model_coords():
    raw = {
        "status": "target_locked",
        "target_visible": True,
        "u": 100,
        "v": 80,
        "confidence": 0.7,
        "stop": False,
    }
    out = parse_nav_result(raw, orig_w=1280, orig_h=720, model_w=256, model_h=144, sx=5.0, sy=5.0)
    assert out["u"] == 500
    assert out["v"] == 400
    assert out["_coords_scaled_from_model"] is True


def test_parse_nav_result_null_uv():
    raw = {
        "status": "searching",
        "target_visible": False,
        "u": None,
        "v": None,
        "confidence": 0.1,
        "stop": False,
    }
    out = parse_nav_result(raw, orig_w=1280, orig_h=720, model_w=256, model_h=144, sx=5.0, sy=5.0)
    assert out["u"] is None
    assert out["target_visible"] is False
    assert out["_point_valid"] is False


def test_pixel_point_servo_forward_when_centered():
    servo = PixelPointServo(image_width=1280, wz_deadzone=0.08, turn_threshold=0.28)
    state, cmd = servo.compute_cmd({"visible": True, "u": 640})
    assert state == "FORWARD"
    assert cmd.linear.x > 0
    assert cmd.angular.z == 0.0


def test_pixel_point_servo_turn_when_far():
    servo = PixelPointServo(image_width=1280, wz_deadzone=0.08, turn_threshold=0.28)
    state, cmd = servo.compute_cmd({"visible": True, "u": 200})
    assert state == "TURN_ONLY"
    assert cmd.linear.x == 0.0
    assert cmd.angular.z != 0.0


def test_pixel_point_servo_lost_without_u():
    servo = PixelPointServo(image_width=1280)
    state, cmd = servo.compute_cmd({"visible": False})
    assert state == "LOST_STOP"
    assert cmd.linear.x == 0.0
