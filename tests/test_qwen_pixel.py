#!/usr/bin/env python3
import os
import sys

import pytest

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.pixel_point_servo import PixelPointServo
from src.vlm.qwen_ollama_client import parse_nav_result


def test_normalize_keep_alive():
    from src.vlm.qwen_ollama_client import _normalize_keep_alive

    assert _normalize_keep_alive("-1") == -1
    assert _normalize_keep_alive("30m") == "30m"
    assert _normalize_keep_alive(-1) == -1


def test_attach_ollama_timing():
    from src.vlm.qwen_ollama_client import QwenOllamaClient

    result: dict = {}
    QwenOllamaClient._attach_ollama_timing(
        result,
        {
            "total_duration": 461_408_902_645,
            "load_duration": 78_536_426_951,
            "prompt_eval_duration": 350_000_000_000,
            "eval_duration": 30_000_000_000,
            "eval_count": 42,
        },
    )
    assert result["_ollama_total_ms"] == pytest.approx(461408.902645, rel=1e-6)
    assert result["_ollama_load_ms"] == pytest.approx(78536.426951, rel=1e-6)
    assert result["_ollama_eval_count"] == 42


def test_parse_nav_result_norm1000_uv_only():
    raw = {"u": 500, "v": 500}
    out = parse_nav_result(
        raw,
        orig_w=1280,
        orig_h=1707,
        model_w=192,
        model_h=256,
        sx=1280 / 192,
        sy=1707 / 256,
        coord_mode="norm1000",
    )
    assert out["usable"] is True
    assert out["u"] == pytest.approx(639.5, rel=0.01)
    assert out["v"] == pytest.approx(853.0, rel=0.01)
    assert "status" not in out
    assert "confidence" not in out


def test_parse_nav_result_null_uv():
    raw = {"u": None, "v": None}
    out = parse_nav_result(
        raw,
        orig_w=1280,
        orig_h=720,
        model_w=192,
        model_h=256,
        sx=1280 / 192,
        sy=720 / 256,
        coord_mode="norm1000",
    )
    assert out["u"] is None
    assert out["usable"] is False
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
