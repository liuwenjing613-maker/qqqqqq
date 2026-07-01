#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.nav.nav_video_overlay import NavOverlayContext, build_overlay_lines, front_min_from_nav_state, parse_bbox_xyxy


def test_build_overlay_lines_core_fields():
    ctx = NavOverlayContext(cmd_vx=0.04, cmd_wz=-0.02)
    ctx.update_nav_state({"fsm_mode": "TRACK", "fsm_reason": "tracking", "front_distance": 0.62})
    ctx.target_bbox = {
        "visible": True,
        "class_name": "bottle",
        "score": 0.81,
        "area_ratio": 0.12,
        "bbox_xyxy": [250, 120, 390, 420],
        "u": 320.0,
        "v": 270.0,
    }
    lines = build_overlay_lines(ctx)
    text = "\n".join(lines)
    assert "[1 TARGET]" in text
    assert "[2 STATE]" in text and "TRACK" in text
    assert "[3 UV]" in text and "320.0" in text
    assert "[4 VEL]" in text and "0.040" in text
    assert "front=0.62m" in text


def test_state_transition_shown():
    ctx = NavOverlayContext()
    ctx.update_nav_state({"fsm_mode": "SEARCH"}, now=1.0)
    ctx.update_nav_state({"fsm_mode": "TRACK"}, now=1.5)
    lines = build_overlay_lines(ctx, now=2.0)
    assert any("switch=SEARCH -> TRACK" in line for line in lines)


def test_front_min_from_nav_state_prefers_nav_distance():
    assert front_min_from_nav_state({"front_distance": 0.52, "front_min_distance": 1.2}) == 0.52
    assert front_min_from_nav_state({"target_distance": 0.48, "front_min_distance": 1.2}) == 0.48
    assert front_min_from_nav_state({"front_min_distance": 0.62}) == 0.62
    assert front_min_from_nav_state({"safety": {"target_distance": 0.48}}) == 0.48


def test_parse_bbox_xywh():
    assert parse_bbox_xyxy([250, 120, 140, 300]) == (250, 120, 390, 420)


if __name__ == "__main__":
    test_build_overlay_lines_core_fields()
    test_state_transition_shown()
    test_front_min_from_nav_state_prefers_nav_distance()
    test_parse_bbox_xywh()
    print("PASS test_nav_video_overlay")
