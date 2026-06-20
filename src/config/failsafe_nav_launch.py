#!/usr/bin/env python3
"""Load P0 failsafe nav stack launch settings from configs/yolo_lidar_failsafe_nav.yaml."""

from __future__ import annotations

import argparse
import json
import os
from typing import Any, Dict, List

import yaml

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
DEFAULT_CONFIG = os.path.join(PROJECT_ROOT, "configs", "yolo_lidar_failsafe_nav.yaml")


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


def _section(raw: Dict[str, Any], key: str) -> Dict[str, Any]:
    block = raw.get(key, {})
    return block if isinstance(block, dict) else {}


def _join_words(words: Any) -> str:
    if isinstance(words, str):
        return words.strip()
    if isinstance(words, list):
        return ",".join(str(x).strip() for x in words if str(x).strip())
    return ""


def load_launch_config(path: str | None = None) -> Dict[str, Any]:
    cfg_path = os.path.expanduser(path or DEFAULT_CONFIG)
    with open(cfg_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    camera = _section(raw, "camera")
    yolo_world = _section(raw, "yolo_world")
    yolo_bridge = _section(raw, "yolo_bridge")
    chassis = _section(raw, "chassis")

    target_words = raw.get("target_words", [])
    target_words_csv = _join_words(target_words)
    bridge_classes = str(
        yolo_bridge.get("target_classes", target_words_csv or "bottle,cup")
    ).strip()

    score_threshold = _as_float(
        yolo_world.get("score_threshold", yolo_bridge.get("min_score", raw.get("target_min_score", 0.002))),
        "yolo_world.score_threshold",
    )

    return {
        "config_path": cfg_path,
        "instruction": str(raw.get("instruction", "bottle")),
        "target_words_csv": target_words_csv,
        "target_classes": bridge_classes,
        "score_threshold": score_threshold,
        "camera_device": str(camera.get("device", "/dev/video0")),
        "camera_compressed_topic": str(camera.get("compressed_topic", "/image")),
        "image_raw_topic": str(raw.get("image_topic", "/image_raw")),
        "image_raw_max_fps": _as_float(camera.get("image_raw_max_fps", 2.0), "camera.image_raw_max_fps"),
        "yolo_det_topic": str(yolo_world.get("det_topic", yolo_bridge.get("det_topic", "/hobot_yolo_world"))),
        "yolo_image_topic": str(yolo_world.get("image_topic", raw.get("image_topic", "/image_raw"))),
        "yolo_iou_threshold": _as_float(yolo_world.get("iou_threshold", 0.45), "yolo_world.iou_threshold"),
        "yolo_feed_type": _as_int(yolo_world.get("feed_type", 1), "yolo_world.feed_type"),
        "bridge_out_topic": str(yolo_bridge.get("out_topic", raw.get("target_bbox_topic", "/target_bbox_json"))),
        "bridge_min_score": _as_float(yolo_bridge.get("min_score", score_threshold), "yolo_bridge.min_score"),
        "bridge_max_area_ratio": _as_float(
            yolo_bridge.get("max_area_ratio", raw.get("bbox_max_area_ratio", 0.24)),
            "yolo_bridge.max_area_ratio",
        ),
        "bridge_require_red_verify": _as_bool(yolo_bridge.get("require_red_verify", False)),
        "bridge_publish_rate_hz": _as_float(yolo_bridge.get("publish_rate_hz", 10.0), "yolo_bridge.publish_rate_hz"),
        "bridge_sync_max_delta_sec": _as_float(yolo_bridge.get("sync_max_delta_sec", 0.5), "yolo_bridge.sync_max_delta_sec"),
        "bridge_min_red_ratio": _as_float(yolo_bridge.get("min_red_ratio", 0.06), "yolo_bridge.min_red_ratio"),
        "bridge_voter_window": _as_int(yolo_bridge.get("voter_window_size", 10), "yolo_bridge.voter_window_size"),
        "bridge_voter_min_votes": _as_int(yolo_bridge.get("voter_min_votes", 3), "yolo_bridge.voter_min_votes"),
        "bridge_voter_lost_hold": _as_int(yolo_bridge.get("voter_lost_hold_frames", 3), "yolo_bridge.voter_lost_hold_frames"),
        "chassis_port": str(chassis.get("port", "/dev/ttyUSB1")),
        "chassis_max_vx": _as_float(chassis.get("max_vx", 0.09), "chassis.max_vx"),
        "chassis_max_wz": _as_float(chassis.get("max_wz", 0.35), "chassis.max_wz"),
        "chassis_watchdog_timeout": _as_float(chassis.get("watchdog_timeout", 0.8), "chassis.watchdog_timeout"),
        "chassis_enable_kick_start": _as_bool(chassis.get("enable_kick_start", True)),
        "chassis_kick_vx": _as_float(chassis.get("kick_vx", 0.08), "chassis.kick_vx"),
        "chassis_kick_wz": _as_float(chassis.get("kick_wz", 0.24), "chassis.kick_wz"),
        "chassis_kick_duration": _as_float(chassis.get("kick_duration", 0.25), "chassis.kick_duration"),
        "chassis_kick_cooldown": _as_float(chassis.get("kick_cooldown", 0.6), "chassis.kick_cooldown"),
        "chassis_cmd_wz_deadzone": _as_float(chassis.get("cmd_wz_deadzone", 0.012), "chassis.cmd_wz_deadzone"),
        "chassis_cmd_smooth_alpha": _as_float(chassis.get("cmd_smooth_alpha", 0.0), "chassis.cmd_smooth_alpha"),
        "chassis_max_vx_delta": _as_float(chassis.get("max_vx_delta", 0.09), "chassis.max_vx_delta"),
        "chassis_max_wz_delta": _as_float(chassis.get("max_wz_delta", 0.35), "chassis.max_wz_delta"),
        "chassis_control_rate_hz": _as_float(chassis.get("control_rate_hz", 20.0), "chassis.control_rate_hz"),
        "chassis_reset_on_zero": _as_bool(chassis.get("reset_on_zero", False)),
        "chassis_zero_reset_hold_sec": _as_float(chassis.get("zero_reset_hold_sec", 0.4), "chassis.zero_reset_hold_sec"),
        "chassis_debug": _as_bool(chassis.get("debug", True)),
    }


def shell_export(cfg: Dict[str, Any]) -> str:
    mapping = {
        "FAILSAFE_CONFIG": cfg["config_path"],
        "INSTRUCTION": cfg["instruction"],
        "TARGET_WORDS": cfg["target_words_csv"],
        "TARGET_CLASSES": cfg["target_classes"],
        "SCORE_THRESHOLD": cfg["score_threshold"],
        "CAMERA_DEV": cfg["camera_device"],
        "CAMERA_COMPRESSED_TOPIC": cfg["camera_compressed_topic"],
        "IMAGE_RAW_TOPIC": cfg["image_raw_topic"],
        "IMAGE_RAW_MAX_FPS": cfg["image_raw_max_fps"],
        "DET_TOPIC": cfg["yolo_det_topic"],
        "YOLO_IMAGE_TOPIC": cfg["yolo_image_topic"],
        "YOLO_IOU_THRESHOLD": cfg["yolo_iou_threshold"],
        "YOLO_FEED_TYPE": cfg["yolo_feed_type"],
        "BRIDGE_OUT_TOPIC": cfg["bridge_out_topic"],
        "YOLO_BRIDGE_MIN_SCORE": cfg["bridge_min_score"],
        "YOLO_BRIDGE_MAX_AREA_RATIO": cfg["bridge_max_area_ratio"],
        "CHASSIS_PORT": cfg["chassis_port"],
        "CHASSIS_MAX_VX": cfg["chassis_max_vx"],
        "CHASSIS_MAX_WZ": cfg["chassis_max_wz"],
        "CHASSIS_WATCHDOG_TIMEOUT": cfg["chassis_watchdog_timeout"],
        "CHASSIS_ENABLE_KICK_START": "1" if cfg["chassis_enable_kick_start"] else "0",
        "CHASSIS_KICK_VX": cfg["chassis_kick_vx"],
        "CHASSIS_KICK_WZ": cfg["chassis_kick_wz"],
        "CHASSIS_KICK_DURATION": cfg["chassis_kick_duration"],
        "CHASSIS_KICK_COOLDOWN": cfg["chassis_kick_cooldown"],
        "CHASSIS_CMD_WZ_DEADZONE": cfg["chassis_cmd_wz_deadzone"],
        "CHASSIS_CMD_SMOOTH_ALPHA": cfg["chassis_cmd_smooth_alpha"],
        "CHASSIS_MAX_VX_DELTA": cfg["chassis_max_vx_delta"],
        "CHASSIS_MAX_WZ_DELTA": cfg["chassis_max_wz_delta"],
        "CHASSIS_CONTROL_RATE_HZ": cfg["chassis_control_rate_hz"],
        "CHASSIS_RESET_ON_ZERO": "1" if cfg["chassis_reset_on_zero"] else "0",
        "CHASSIS_ZERO_RESET_HOLD_SEC": cfg["chassis_zero_reset_hold_sec"],
        "CHASSIS_DEBUG": "1" if cfg["chassis_debug"] else "0",
    }
    lines = []
    for key, value in mapping.items():
        if isinstance(value, str):
            lines.append(f'export {key}="{value}"')
        else:
            lines.append(f"export {key}={value}")
    return "\n".join(lines)


def bridge_argv(cfg: Dict[str, Any]) -> List[str]:
    args = [
        "--config",
        cfg["config_path"],
    ]
    return args


def main():
    parser = argparse.ArgumentParser(description="Load configs/yolo_lidar_failsafe_nav.yaml for stack launch")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--shell-export", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    cfg = load_launch_config(args.config)
    if args.shell_export:
        print(shell_export(cfg))
    else:
        print(json.dumps(cfg, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
