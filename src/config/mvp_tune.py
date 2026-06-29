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


def _as_bool(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


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
        "sync_max_delta_sec": _as_float(raw.get("sync_max_delta_sec", 0.12), "sync_max_delta_sec"),
        "min_red_ratio": _as_float(raw.get("min_red_ratio", 0.06), "min_red_ratio"),
        "lost_hold_frames": _as_int(raw.get("lost_hold_frames", 3), "lost_hold_frames"),
        "lost_observe_frames": _as_int(raw.get("lost_observe_frames", 7), "lost_observe_frames"),
        "recovery_scan_max_frames": _as_int(
            raw.get("recovery_scan_max_frames", 25),
            "recovery_scan_max_frames",
        ),
        "recovery_replan_sec": _as_float(raw.get("recovery_replan_sec", 2.0), "recovery_replan_sec"),
        "lost_hold_wz_scale": _as_float(raw.get("lost_hold_wz_scale", 0.4), "lost_hold_wz_scale"),
        "lost_hold_max_wz": _as_float(raw.get("lost_hold_max_wz", 0.035), "lost_hold_max_wz"),
        "recovery_scan_wz": _as_float(raw.get("recovery_scan_wz", 0.006), "recovery_scan_wz"),
        "recovery_pulse_frames": _as_int(raw.get("recovery_pulse_frames", 2), "recovery_pulse_frames"),
        "recovery_observe_frames": _as_int(raw.get("recovery_observe_frames", 6), "recovery_observe_frames"),
        "chassis_port": str(raw.get("chassis_port", "/dev/ttyUSB1")),
        "chassis_max_vx": _as_float(raw.get("chassis_max_vx", 0.06), "chassis_max_vx"),
        "chassis_max_wz": _as_float(raw.get("chassis_max_wz", 0.06), "chassis_max_wz"),
        "chassis_pwm_wheel_layout": str(raw.get("chassis_pwm_wheel_layout", "fl-rl-fr-rr")),
        "chassis_motor_signs": str(raw.get("chassis_motor_signs", "1,1,1,1")),
        "chassis_vx_pwm_deadband": _as_float(raw.get("chassis_vx_pwm_deadband", 6.0), "chassis_vx_pwm_deadband"),
        "chassis_wz_pwm_deadband": _as_float(raw.get("chassis_wz_pwm_deadband", 8.0), "chassis_wz_pwm_deadband"),
        "chassis_pwm_max": _as_float(raw.get("chassis_pwm_max", 30.0), "chassis_pwm_max"),
        "chassis_vx_pwm_gain": _as_float(raw.get("chassis_vx_pwm_gain", 180.0), "chassis_vx_pwm_gain"),
        "chassis_wz_pwm_gain": _as_float(raw.get("chassis_wz_pwm_gain", 120.0), "chassis_wz_pwm_gain"),
        "chassis_pwm_smooth_alpha": _as_float(raw.get("chassis_pwm_smooth_alpha", 0.35), "chassis_pwm_smooth_alpha"),
        "chassis_max_pwm_delta": _as_float(raw.get("chassis_max_pwm_delta", 3.0), "chassis_max_pwm_delta"),
        "chassis_watchdog_timeout": _as_float(raw.get("chassis_watchdog_timeout", 0.5), "chassis_watchdog_timeout"),
        "enable_kick_start": enable_kick_start,
        "kick_vx": _as_float(raw.get("kick_vx", 0.05), "kick_vx"),
        "kick_wz": _as_float(raw.get("kick_wz", 0.24), "kick_wz"),
        "kick_duration": _as_float(raw.get("kick_duration", 0.12), "kick_duration"),
        "kick_cooldown": _as_float(raw.get("kick_cooldown", 2.0), "kick_cooldown"),
        "cmd_smooth_alpha": _as_float(raw.get("cmd_smooth_alpha", 0.45), "cmd_smooth_alpha"),
        "max_vx_delta": _as_float(raw.get("max_vx_delta", 0.012), "max_vx_delta"),
        "max_wz_delta": _as_float(raw.get("max_wz_delta", 0.015), "max_wz_delta"),
        "control_rate_hz": _as_float(raw.get("control_rate_hz", 20.0), "control_rate_hz"),
        "sanitize_on_start": _as_bool(raw.get("sanitize_on_start", True)),
        "sanitize_pid_kp": _as_float(raw.get("sanitize_pid_kp", 1.2), "sanitize_pid_kp"),
        "sanitize_pid_ki": _as_float(raw.get("sanitize_pid_ki", 0.05), "sanitize_pid_ki"),
        "sanitize_pid_kd": _as_float(raw.get("sanitize_pid_kd", 0.02), "sanitize_pid_kd"),
        "sanitize_speed_limit_vx": _as_float(raw.get("sanitize_speed_limit_vx", 0.0), "sanitize_speed_limit_vx"),
        "sanitize_speed_limit_wz": _as_float(raw.get("sanitize_speed_limit_wz", 0.0), "sanitize_speed_limit_wz"),
        "sanitize_imu_adjust": _as_bool(raw.get("sanitize_imu_adjust", False)),
        "sanitize_yaw_pid_zero": _as_bool(raw.get("sanitize_yaw_pid_zero", True)),
        "sanitize_write_flash": _as_bool(raw.get("sanitize_write_flash", False)),
        "sanitize_settle_sec": _as_float(raw.get("sanitize_settle_sec", 0.8), "sanitize_settle_sec"),
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
        "SYNC_MAX_DELTA_SEC": tune["sync_max_delta_sec"],
        "MIN_RED_RATIO": tune["min_red_ratio"],
        "CHASSIS_PORT": tune["chassis_port"],
        "CHASSIS_MAX_VX": tune["chassis_max_vx"],
        "CHASSIS_MAX_WZ": tune["chassis_max_wz"],
        "CHASSIS_PWM_WHEEL_LAYOUT": tune["chassis_pwm_wheel_layout"],
        "CHASSIS_MOTOR_SIGNS": tune["chassis_motor_signs"],
        "CHASSIS_VX_PWM_DEADBAND": tune["chassis_vx_pwm_deadband"],
        "CHASSIS_WZ_PWM_DEADBAND": tune["chassis_wz_pwm_deadband"],
        "CHASSIS_PWM_MAX": tune["chassis_pwm_max"],
        "CHASSIS_VX_PWM_GAIN": tune["chassis_vx_pwm_gain"],
        "CHASSIS_WZ_PWM_GAIN": tune["chassis_wz_pwm_gain"],
        "CHASSIS_PWM_SMOOTH_ALPHA": tune["chassis_pwm_smooth_alpha"],
        "CHASSIS_MAX_PWM_DELTA": tune["chassis_max_pwm_delta"],
        "CHASSIS_WATCHDOG_TIMEOUT": tune["chassis_watchdog_timeout"],
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
        "LOST_HOLD_FRAMES": tune["lost_hold_frames"],
        "LOST_OBSERVE_FRAMES": tune["lost_observe_frames"],
        "RECOVERY_SCAN_MAX_FRAMES": tune["recovery_scan_max_frames"],
        "RECOVERY_REPLAN_SEC": tune["recovery_replan_sec"],
        "LOST_HOLD_WZ_SCALE": tune["lost_hold_wz_scale"],
        "LOST_HOLD_MAX_WZ": tune["lost_hold_max_wz"],
        "RECOVERY_SCAN_WZ": tune["recovery_scan_wz"],
        "RECOVERY_PULSE_FRAMES": tune["recovery_pulse_frames"],
        "RECOVERY_OBSERVE_FRAMES": tune["recovery_observe_frames"],
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
