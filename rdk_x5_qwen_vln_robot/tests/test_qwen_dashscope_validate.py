#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_dashscope_client import QwenDashScopeClient, build_prompt, parse_instruction_sequence


def test_parse_instruction_sequence():
    assert parse_instruction_sequence("find bottle") == ["bottle"]
    assert parse_instruction_sequence("first find bottle, then find cup") == ["bottle", "cup"]


def test_build_prompt_target_contains_target_mode():
    prompt = build_prompt("bottle", mode="target", first_request=True, image_width=640, image_height=480)
    assert "TARGET|PATH|NONE" in prompt
    assert "TARGET_TASK_FIRST" in prompt
    assert "Do NOT return PATH mode" in prompt
    assert "GATE_CENTER_STRICT" in prompt
    assert "visual gates" in prompt.lower()


def test_build_prompt_path_contains_path_mode():
    prompt = build_prompt("bottle", mode="path", first_request=True)
    assert "PATH_TASK_FIRST" in prompt
    assert "Do NOT return TARGET mode" in prompt
    assert "wall_like_no_floor" in prompt
    assert "GATE_STATE_SAFE" in prompt
    assert "GATE_FLOOR" in prompt
    assert "Do NOT output motion commands" in prompt


def test_validate_target_usable():
    client = QwenDashScopeClient.__new__(QwenDashScopeClient)
    client.min_confidence = 0.60
    image_info = {"orig_w": 1280, "orig_h": 720, "sent_w": 640, "sent_h": 360}
    out = client._validate_and_map(
        {
            "mode": "TARGET",
            "target_visible": True,
            "u": 0.5,
            "v": 0.4,
            "waypoint_visible": False,
            "waypoint_u": None,
            "waypoint_v": None,
            "confidence": 0.9,
            "reason": "target visible",
        },
        image_info,
        task_mode="target",
    )
    assert out["mode"] == "TARGET"
    assert out["usable"] is True
    assert out["u"] is not None
    assert out["waypoint_u"] is None


def test_validate_path_usable():
    client = QwenDashScopeClient.__new__(QwenDashScopeClient)
    client.min_confidence = 0.60
    image_info = {"orig_w": 1280, "orig_h": 720, "sent_w": 640, "sent_h": 360}
    out = client._validate_and_map(
        {
            "mode": "PATH",
            "target_visible": False,
            "u": None,
            "v": None,
            "waypoint_visible": True,
            "waypoint_u": 0.5,
            "waypoint_v": 0.7,
            "confidence": 0.75,
            "reason": "safe corridor",
        },
        image_info,
        task_mode="path",
    )
    assert out["mode"] == "PATH"
    assert out["usable"] is True
    assert out["u"] is None
    assert out["waypoint_u"] is not None


def test_validate_none_not_usable():
    client = QwenDashScopeClient.__new__(QwenDashScopeClient)
    client.min_confidence = 0.60
    image_info = {"orig_w": 1280, "orig_h": 720, "sent_w": 640, "sent_h": 360}
    out = client._validate_and_map(
        {
            "mode": "NONE",
            "target_visible": False,
            "u": None,
            "v": None,
            "waypoint_visible": False,
            "waypoint_u": None,
            "waypoint_v": None,
            "confidence": 0.0,
            "reason": "no target and no path",
        },
        image_info,
        task_mode="target",
    )
    assert out["mode"] == "NONE"
    assert out["usable"] is False


def test_validate_legacy_locked_still_works():
    client = QwenDashScopeClient.__new__(QwenDashScopeClient)
    client.min_confidence = 0.60
    image_info = {"orig_w": 1280, "orig_h": 720, "sent_w": 640, "sent_h": 360}
    out = client._validate_and_map(
        {"status": "locked", "u": 0.5, "v": 0.4, "confidence": 0.9, "reason": ""},
        image_info,
        task_mode="target",
    )
    assert out["mode"] == "TARGET"
    assert out["usable"] is True


if __name__ == "__main__":
    test_parse_instruction_sequence()
    test_build_prompt_target_contains_target_mode()
    test_build_prompt_path_contains_path_mode()
    test_validate_target_usable()
    test_validate_path_usable()
    test_validate_none_not_usable()
    test_validate_legacy_locked_still_works()
    print("PASS test_qwen_dashscope_validate")
