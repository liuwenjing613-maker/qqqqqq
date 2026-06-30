#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.config.nav_success import load_success_config


def test_load_success_from_success_section():
    cfg = {
        "success": {
            "require_lidar": True,
            "center_px": 80,
            "min_distance": 0.5,
            "max_distance": 0.7,
            "arrive_frames": 3,
            "verify_frames": 5,
            "qwen_verify_required": True,
        }
    }
    s = load_success_config(cfg)
    assert s["center_px"] == 80.0
    assert s["min_distance"] == 0.5
    assert s["arrive_frames"] == 3
    assert s["verify_frames"] == 5
    assert s["qwen_verify_required"] is True


def test_load_success_legacy_arrive_fallback():
    cfg = {
        "fsm": {"require_lidar": False, "arrive_required_frames": 6},
        "arrive": {
            "center_px": 60,
            "arrive_min_distance": 0.55,
            "arrive_max_distance": 0.8,
            "arrive_area_ratio": 0.2,
        },
    }
    s = load_success_config(cfg)
    assert s["require_lidar"] is False
    assert s["center_px"] == 60.0
    assert s["min_distance"] == 0.55
    assert s["arrive_frames"] == 6
    assert s["verify_frames"] == 6
    assert s["min_area_ratio"] == 0.2


if __name__ == "__main__":
    test_load_success_from_success_section()
    test_load_success_legacy_arrive_fallback()
    print("PASS test_nav_success")
