#!/usr/bin/env python3
"""Load unified MVP tuning from configs/mvp_tune.yaml."""

from __future__ import annotations

import argparse
import json
import os
from typing import Any, Dict

import yaml

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
DEFAULT_TUNE_PATH = os.path.join(PROJECT_ROOT, "configs/mvp_tune.yaml")


def _as_float(value, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid float for {name}: {value!r}") from exc


def _as_int(value, name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid int for {name}: {value!r}") from exc


def load_mvp_tune(path: str | None = None) -> Dict[str, Any]:
    tune_path = os.path.expanduser(path or DEFAULT_TUNE_PATH)
    with open(tune_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    yolo_min_score = _as_float(raw.get("yolo_min_score", 0.006), "yolo_min_score")
    max_vx = _as_float(raw.get("max_vx", 0.04), "max_vx")
    turn_threshold = _as_float(raw.get("turn_threshold", raw.get("center_threshold", 0.30)), "turn_threshold")
    forward_threshold = _as_float(
        raw.get("forward_threshold", max(0.12, turn_threshold * 0.65)),
        "forward_threshold",
    )
    min_drive_ratio = _as_float(raw.get("min_drive_vx_ratio", 0.9), "min_drive_vx_ratio")
    min_drive_floor = _as_float(raw.get("min_drive_vx_floor", 0.03), "min_drive_vx_floor")
    enable_kick_start = bool(raw.get("enable_kick_start", False))
    arrive_area_ratio = _as_float(raw.get("arrive_area_ratio", 0.14), "arrive_area_ratio")
    slowdown_area_ratio = _as_float(
        raw.get("slowdown_area_ratio", max(0.04, arrive_area_ratio * 0.55)),
        "slowdown_area_ratio",
    )

    tune = {
        "config_path": tune_path,
        "yolo_min_score": yolo_min_score,
        "score_threshold": yolo_min_score,
        "min_score": yolo_min_score,
        "max_vx": max_vx,
        "max_wz": _as_float(raw.get("max_wz", 0.08), "max_wz"),
        "kp_turn": _as_float(raw.get("kp_turn", 0.06), "kp_turn"),
        "center_threshold": turn_threshold,
        "turn_threshold": turn_threshold,
        "forward_threshold": forward_threshold,
        "wz_deadzone": _as_float(raw.get("wz_deadzone", 0.05), "wz_deadzone"),
        "cmd_wz_deadzone": _as_float(
            raw.get("cmd_wz_deadzone", 0.01),
            "cmd_wz_deadzone",
        ),
        "forward_turn_scale": _as_float(raw.get("forward_turn_scale", 0.35), "forward_turn_scale"),
        "arrive_area_ratio": arrive_area_ratio,
        "slowdown_area_ratio": slowdown_area_ratio,
        "stable_frames_required": _as_int(raw.get("stable_frames_required", 2), "stable_frames_required"),
        "lost_frames_limit": _as_int(raw.get("lost_frames_limit", 8), "lost_frames_limit"),
        "max_area_ratio": _as_float(raw.get("max_area_ratio", 0.15), "max_area_ratio"),
        "det_stale_sec": _as_float(raw.get("det_stale_sec", 1.0), "det_stale_sec"),
        "min_red_ratio": _as_float(raw.get("min_red_ratio", 0.06), "min_red_ratio"),
        "recovery_scan_wz": _as_float(raw.get("recovery_scan_wz", 0.006), "recovery_scan_wz"),
        "chassis_port": str(raw.get("chassis_port", "/dev/ttyUSB1")),
        "chassis_max_vx": _as_float(raw.get("chassis_max_vx", 0.08), "chassis_max_vx"),
        "chassis_max_wz": _as_float(raw.get("chassis_max_wz", 0.12), "chassis_max_wz"),
        "enable_kick_start": enable_kick_start,
        "kick_vx": _as_float(raw.get("kick_vx", 0.05), "kick_vx"),
        "kick_wz": _as_float(raw.get("kick_wz", 0.24), "kick_wz"),
        "kick_duration": _as_float(raw.get("kick_duration", 0.12), "kick_duration"),
        "kick_cooldown": _as_float(raw.get("kick_cooldown", 2.0), "kick_cooldown"),
        "cmd_smooth_alpha": _as_float(raw.get("cmd_smooth_alpha", 0.45), "cmd_smooth_alpha"),
        "max_vx_delta": _as_float(raw.get("max_vx_delta", 0.012), "max_vx_delta"),
        "max_wz_delta": _as_float(raw.get("max_wz_delta", 0.015), "max_wz_delta"),
        "control_rate_hz": _as_float(raw.get("control_rate_hz", 20.0), "control_rate_hz"),
        "min_drive_vx": max(min_drive_floor, max_vx * min_drive_ratio),
        "min_cruise_wz": _as_float(
            raw.get("min_cruise_wz", raw.get("max_wz", 0.16)),
            "min_cruise_wz",
        ),
    }
    return tune


def shell_export(tune: Dict[str, Any]) -> str:
    mapping = {
        "SCORE_THRESHOLD": tune["score_threshold"],
        "MIN_SCORE": tune["min_score"],
        "MAX_VX": tune["max_vx"],
        "MAX_WZ": tune["max_wz"],
        "KP_TURN": tune["kp_turn"],
        "CENTER_THRESHOLD": tune["center_threshold"],
        "ARRIVE_AREA_RATIO": tune["arrive_area_ratio"],
        "SLOWDOWN_AREA_RATIO": tune["slowdown_area_ratio"],
        "STABLE_FRAMES_REQUIRED": tune["stable_frames_required"],
        "LOST_FRAMES_LIMIT": tune["lost_frames_limit"],
        "MAX_AREA_RATIO": tune["max_area_ratio"],
        "DET_STALE_SEC": tune["det_stale_sec"],
        "MIN_RED_RATIO": tune["min_red_ratio"],
        "CHASSIS_PORT": tune["chassis_port"],
        "CHASSIS_MAX_VX": tune["chassis_max_vx"],
        "CHASSIS_MAX_WZ": tune["chassis_max_wz"],
        "KICK_VX": tune["kick_vx"],
        "KICK_WZ": tune["kick_wz"],
        "KICK_DURATION": tune["kick_duration"],
        "MIN_DRIVE_VX": tune["min_drive_vx"],
        "MIN_CRUISE_WZ": tune["min_cruise_wz"],
        "KICK_COOLDOWN": tune["kick_cooldown"],
        "ENABLE_KICK_START": "1" if tune["enable_kick_start"] else "0",
        "CMD_SMOOTH_ALPHA": tune["cmd_smooth_alpha"],
        "MAX_VX_DELTA": tune["max_vx_delta"],
        "MAX_WZ_DELTA": tune["max_wz_delta"],
        "CONTROL_RATE_HZ": tune["control_rate_hz"],
        "RECOVERY_SCAN_WZ": tune["recovery_scan_wz"],
        "TURN_THRESHOLD": tune["turn_threshold"],
        "FORWARD_THRESHOLD": tune["forward_threshold"],
        "WZ_DEADZONE": tune["wz_deadzone"],
        "CMD_WZ_DEADZONE": tune["cmd_wz_deadzone"],
        "FORWARD_TURN_SCALE": tune["forward_turn_scale"],
        "MVP_TUNE_FILE": tune["config_path"],
    }
    lines = []
    for key, value in mapping.items():
        if isinstance(value, str):
            lines.append(f'export {key}="{value}"')
        else:
            lines.append(f"export {key}={value}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Load configs/mvp_tune.yaml")
    parser.add_argument("--config", default=DEFAULT_TUNE_PATH)
    parser.add_argument("--shell-export", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    tune = load_mvp_tune(args.config)
    if args.shell_export:
        print(shell_export(tune))
    elif args.json:
        print(json.dumps(tune, indent=2, ensure_ascii=False))
    else:
        print(json.dumps(tune, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
