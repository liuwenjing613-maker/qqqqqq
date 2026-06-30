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
    yolov5s_bpu = _section(raw, "yolov5s_bpu")
    yolo_bridge = _section(raw, "yolo_bridge")
    target_cfg = _section(raw, "target")
    chassis = _section(raw, "chassis")

    target_words = raw.get("target_words", target_cfg.get("words", []))
    target_words_csv = _join_words(target_words)
    bridge_classes = str(
        yolov5s_bpu.get(
            "target_classes",
            yolo_bridge.get("target_classes", _join_words(target_cfg.get("classes", [])) or target_words_csv or "bottle"),
        )
    ).strip()

    use_yolov5s_bpu = _as_bool(yolov5s_bpu.get("enabled", False))
    score_threshold = _as_float(
        yolov5s_bpu.get(
            "score_threshold",
            yolo_world.get("score_threshold", yolo_bridge.get("min_score", target_cfg.get("min_score", 0.002))),
        ),
        "detector.score_threshold",
    )

    return {
        "config_path": cfg_path,
        "instruction": str(raw.get("instruction", "bottle")),
        "target_words_csv": target_words_csv,
        "target_classes": bridge_classes,
        "score_threshold": score_threshold,
        "detector_backend": "yolov5s_bpu" if use_yolov5s_bpu else "yolo_world",
        "yolov5s_model": str(
            yolov5s_bpu.get(
                "model",
                "/root/rdk_model_zoo/samples/vision/yolov5/model/yolov5s_tag_v7.0_detect_640x640_bayese_nv12.bin",
            )
        ),
        "yolov5s_runtime_dir": str(
            yolov5s_bpu.get("runtime_dir", "/root/rdk_model_zoo/samples/vision/yolov5/runtime/python")
        ),
        "yolov5s_zoo_root": str(yolov5s_bpu.get("zoo_root", "/root/rdk_model_zoo")),
        "yolov5s_input_type": str(yolov5s_bpu.get("input_type", "raw")),
        "yolov5s_image_topic": str(
            yolov5s_bpu.get("image_topic", raw.get("image_topic", "/image_raw"))
        ),
        "yolov5s_out_topic": str(
            yolov5s_bpu.get("out_topic", raw.get("target_bbox_topic", "/target_bbox_json"))
        ),
        "yolov5s_target_words_topic": str(
            yolov5s_bpu.get("target_words_topic", raw.get("target_words_topic", "/target_words"))
        ),
        "yolov5s_nms_threshold": _as_float(yolov5s_bpu.get("nms_threshold", 0.45), "yolov5s_bpu.nms_threshold"),
        "yolov5s_max_hz": _as_float(yolov5s_bpu.get("max_hz", 10.0), "yolov5s_bpu.max_hz"),
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
        "chassis_bridge": str(chassis.get("bridge", "pwm")),
        "chassis_max_vx": _as_float(chassis.get("max_vx", 0.06), "chassis.max_vx"),
        "chassis_max_wz": _as_float(chassis.get("max_wz", 0.06), "chassis.max_wz"),
        "chassis_watchdog_timeout": _as_float(chassis.get("watchdog_timeout", 0.5), "chassis.watchdog_timeout"),
        "chassis_control_rate_hz": _as_float(chassis.get("control_rate_hz", 20.0), "chassis.control_rate_hz"),
        "chassis_pwm_wheel_layout": str(chassis.get("wheel_layout", "fl-rl-fr-rr")),
        "chassis_motor_signs": str(chassis.get("motor_signs", "1,1,1,1")),
        "chassis_vx_pwm_deadband": _as_float(chassis.get("vx_pwm_deadband", 6.0), "chassis.vx_pwm_deadband"),
        "chassis_wz_pwm_deadband": _as_float(chassis.get("wz_pwm_deadband", 8.0), "chassis.wz_pwm_deadband"),
        "chassis_pwm_max": _as_float(chassis.get("pwm_max", 30.0), "chassis.pwm_max"),
        "chassis_vx_pwm_gain": _as_float(chassis.get("vx_pwm_gain", 180.0), "chassis.vx_pwm_gain"),
        "chassis_wz_pwm_gain": _as_float(chassis.get("wz_pwm_gain", 120.0), "chassis.wz_pwm_gain"),
        "chassis_pwm_smooth_alpha": _as_float(chassis.get("pwm_smooth_alpha", 0.35), "chassis.pwm_smooth_alpha"),
        "chassis_max_pwm_delta": _as_float(chassis.get("max_pwm_delta", 3.0), "chassis.max_pwm_delta"),
        "chassis_debug": _as_bool(chassis.get("debug", True)),
    }


def shell_export(cfg: Dict[str, Any]) -> str:
    mapping = {
        "FAILSAFE_CONFIG": cfg["config_path"],
        "INSTRUCTION": cfg["instruction"],
        "TARGET_WORDS": cfg["target_words_csv"],
        "TARGET_CLASSES": cfg["target_classes"],
        "SCORE_THRESHOLD": cfg["score_threshold"],
        "DETECTOR_BACKEND": cfg["detector_backend"],
        "YOLOV5S_MODEL": cfg["yolov5s_model"],
        "YOLOV5S_RUNTIME_DIR": cfg["yolov5s_runtime_dir"],
        "YOLOV5S_ZOO_ROOT": cfg["yolov5s_zoo_root"],
        "YOLOV5S_INPUT_TYPE": cfg["yolov5s_input_type"],
        "YOLOV5S_IMAGE_TOPIC": cfg["yolov5s_image_topic"],
        "YOLOV5S_OUT_TOPIC": cfg["yolov5s_out_topic"],
        "YOLOV5S_TARGET_WORDS_TOPIC": cfg["yolov5s_target_words_topic"],
        "YOLOV5S_NMS_THRESHOLD": cfg["yolov5s_nms_threshold"],
        "YOLOV5S_MAX_HZ": cfg["yolov5s_max_hz"],
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
        "CHASSIS_CONTROL_RATE_HZ": cfg["chassis_control_rate_hz"],
        "CHASSIS_PWM_WHEEL_LAYOUT": cfg["chassis_pwm_wheel_layout"],
        "CHASSIS_MOTOR_SIGNS": cfg["chassis_motor_signs"],
        "CHASSIS_VX_PWM_DEADBAND": cfg["chassis_vx_pwm_deadband"],
        "CHASSIS_WZ_PWM_DEADBAND": cfg["chassis_wz_pwm_deadband"],
        "CHASSIS_PWM_MAX": cfg["chassis_pwm_max"],
        "CHASSIS_VX_PWM_GAIN": cfg["chassis_vx_pwm_gain"],
        "CHASSIS_WZ_PWM_GAIN": cfg["chassis_wz_pwm_gain"],
        "CHASSIS_PWM_SMOOTH_ALPHA": cfg["chassis_pwm_smooth_alpha"],
        "CHASSIS_MAX_PWM_DELTA": cfg["chassis_max_pwm_delta"],
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
