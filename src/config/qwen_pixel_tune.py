#!/usr/bin/env python3
"""Load Qwen pixel task tuning from configs/qwen_pixel_tune.yaml."""

import argparse
import os
from typing import Any, Dict

import yaml

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
DEFAULT_TUNE_PATH = os.path.join(PROJECT_ROOT, "configs/qwen_pixel_tune.yaml")


def _as_float(value: Any, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid float for {name}: {value!r}") from exc


def _as_int(value: Any, name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid int for {name}: {value!r}") from exc


def load_qwen_pixel_tune(path: str = DEFAULT_TUNE_PATH) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    return {
        "model": str(raw.get("model", "qwen2.5vl:3b")),
        "image_topic": str(raw.get("image_topic", "/image_raw")),
        "cmd_topic": str(raw.get("cmd_topic", "/cmd_vel")),
        "qwen_json_topic": str(raw.get("qwen_json_topic", "/qwen_nav_json")),
        "image_width": _as_int(raw.get("image_width", 1280), "image_width"),
        "image_height": _as_int(raw.get("image_height", 720), "image_height"),
        "qwen_resize_width": _as_int(raw.get("qwen_resize_width", 256), "qwen_resize_width"),
        "qwen_interval_sec": _as_float(raw.get("qwen_interval_sec", 240.0), "qwen_interval_sec"),
        "qwen_timeout_sec": _as_float(raw.get("qwen_timeout_sec", 900.0), "qwen_timeout_sec"),
        "target_lock_conf": _as_float(raw.get("target_lock_conf", 0.45), "target_lock_conf"),
        "max_vx": _as_float(raw.get("max_vx", 0.035), "max_vx"),
        "max_wz": _as_float(raw.get("max_wz", 0.045), "max_wz"),
        "kp_turn": _as_float(raw.get("kp_turn", 0.09), "kp_turn"),
        "wz_deadzone": _as_float(raw.get("wz_deadzone", 0.08), "wz_deadzone"),
        "turn_threshold": _as_float(raw.get("turn_threshold", 0.28), "turn_threshold"),
        "forward_turn_scale": _as_float(raw.get("forward_turn_scale", 0.45), "forward_turn_scale"),
        "cmd_wz_deadzone": _as_float(raw.get("cmd_wz_deadzone", 0.012), "cmd_wz_deadzone"),
        "verify_v_min": _as_float(raw.get("verify_v_min", 520.0), "verify_v_min"),
        "verify_u_center_px": _as_float(raw.get("verify_u_center_px", 80.0), "verify_u_center_px"),
        "verify_min_forward_bursts": _as_int(
            raw.get("verify_min_forward_bursts", 3), "verify_min_forward_bursts"
        ),
        "servo_burst_sec": _as_float(raw.get("servo_burst_sec", 0.25), "servo_burst_sec"),
        "observe_stop_sec": _as_float(raw.get("observe_stop_sec", 0.45), "observe_stop_sec"),
        "scan_wz": _as_float(raw.get("scan_wz", 0.040), "scan_wz"),
        "scan_burst_sec": _as_float(raw.get("scan_burst_sec", 0.18), "scan_burst_sec"),
        "scan_observe_sec": _as_float(raw.get("scan_observe_sec", 0.50), "scan_observe_sec"),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load configs/qwen_pixel_tune.yaml")
    parser.add_argument("--config", default=DEFAULT_TUNE_PATH)
    args = parser.parse_args()
    tune = load_qwen_pixel_tune(args.config)
    for k, v in tune.items():
        print(f"{k}={v}")
