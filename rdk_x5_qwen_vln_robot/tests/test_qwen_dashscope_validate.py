#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_dashscope_client import QwenDashScopeClient, build_prompt, parse_instruction_sequence


def test_parse_instruction_sequence():
    assert parse_instruction_sequence("find bottle") == ["bottle"]
    assert parse_instruction_sequence("first find bottle, then find cup") == ["bottle", "cup"]


def test_build_prompt_track_contains_locked_and_inferred():
    prompt = build_prompt("bottle", mode="track", first_request=True, image_width=640, image_height=480)
    assert "locked|inferred" in prompt
    assert "TRACK_FIRST" in prompt
    assert "inferred" in prompt.lower()
    assert "navigable path waypoint" in prompt


def test_validate_locked_usable():
    client = QwenDashScopeClient.__new__(QwenDashScopeClient)
    client.min_confidence = 0.60
    image_info = {"orig_w": 1280, "orig_h": 720, "sent_w": 640, "sent_h": 360}
    out = client._validate_and_map(
        {"status": "locked", "u": 0.5, "v": 0.4, "confidence": 0.9, "reason": ""},
        image_info,
    )
    assert out["usable"] is True
    assert out["direction_valid"] is True
    assert out["status"] == "locked"
    assert out["u"] is not None
    assert abs(out["_raw_u"] - 0.5) < 1e-6


def test_validate_inferred_direction_valid_not_usable():
    client = QwenDashScopeClient.__new__(QwenDashScopeClient)
    client.min_confidence = 0.60
    image_info = {"orig_w": 1280, "orig_h": 720, "sent_w": 640, "sent_h": 360}
    out = client._validate_and_map(
        {"status": "inferred", "u": 0.5, "v": 0.7, "confidence": 0.35, "reason": "clear path"},
        image_info,
    )
    assert out["usable"] is False
    assert out["direction_valid"] is True
    assert out["status"] == "inferred"
    assert out["u"] is not None
    assert out["_coord_reason"] == "inferred_waypoint"


def test_validate_pixel_coords_converted():
    client = QwenDashScopeClient.__new__(QwenDashScopeClient)
    client.min_confidence = 0.60
    image_info = {"orig_w": 1280, "orig_h": 720, "sent_w": 640, "sent_h": 360}
    out = client._validate_and_map(
        {"status": "inferred", "u": 320.0, "v": 180.0, "confidence": 0.4, "reason": ""},
        image_info,
    )
    assert out["direction_valid"] is True
    assert abs(out["_raw_u"] - 0.5) < 1e-3
    assert abs(out["_raw_v"] - 0.5) < 1e-3


if __name__ == "__main__":
    test_parse_instruction_sequence()
    test_build_prompt_track_contains_locked_and_inferred()
    test_validate_locked_usable()
    test_validate_inferred_direction_valid_not_usable()
    test_validate_pixel_coords_converted()
    print("PASS test_qwen_dashscope_validate")
