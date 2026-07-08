#!/usr/bin/env python3
"""Load semantic_mapping.yaml configuration."""

from __future__ import annotations

import os
from typing import Any, Dict, List

import yaml

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
DEFAULT_CONFIG = os.path.join(PROJECT_ROOT, "configs", "semantic_mapping.yaml")


def _section(raw: Dict[str, Any], key: str) -> Dict[str, Any]:
    block = raw.get(key, {})
    return block if isinstance(block, dict) else {}


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or str(raw).strip() == "":
        return default
    return _as_float(raw, default)


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _as_list(value: Any) -> List[str]:
    if isinstance(value, list):
        return [str(x).strip() for x in value if str(x).strip()]
    if isinstance(value, str):
        return [x.strip() for x in value.split(",") if x.strip()]
    return []


def _resolve_semantic_classes(classes: Dict[str, Any]) -> Dict[str, List[str]]:
    """Expand whitelist_mode: all_coco to full COCO 80-class lists."""
    from src.perception.coco_classes import (
        COCO_CLASS_NAMES,
        COCO_DYNAMIC_CLASSES,
        COCO_LARGE_OBJECTS,
        COCO_SMALL_OBJECTS,
        expand_class_list,
        is_all_coco_mode,
    )

    mode = str(classes.get("whitelist_mode", "")).lower().strip()
    use_all = is_all_coco_mode(mode)

    whitelist_raw = _as_list(classes.get("whitelist"))
    if use_all or (len(whitelist_raw) == 1 and is_all_coco_mode(whitelist_raw[0])):
        whitelist = list(COCO_CLASS_NAMES)
    else:
        whitelist = whitelist_raw

    dynamic_raw = _as_list(classes.get("dynamic"))
    if use_all and not dynamic_raw:
        dynamic = sorted(COCO_DYNAMIC_CLASSES)
    else:
        dynamic = dynamic_raw

    small_mode = str(classes.get("small_objects_mode", "")).lower().strip()
    small_raw = _as_list(classes.get("small_objects"))
    if use_all and (not small_raw or is_all_coco_mode(small_mode)):
        small_objects = sorted(COCO_SMALL_OBJECTS)
    else:
        small_objects = expand_class_list(small_raw, auto_set=COCO_SMALL_OBJECTS)

    large_mode = str(classes.get("large_objects_mode", "")).lower().strip()
    large_raw = _as_list(classes.get("large_objects"))
    if use_all and (not large_raw or is_all_coco_mode(large_mode)):
        large_objects = sorted(COCO_LARGE_OBJECTS)
    else:
        large_objects = expand_class_list(large_raw, auto_set=COCO_LARGE_OBJECTS)

    return {
        "whitelist": whitelist,
        "dynamic": dynamic,
        "small_objects": small_objects,
        "large_objects": large_objects,
        "whitelist_mode": mode or ("all_coco" if use_all else ""),
    }


def load_semantic_config(path: str | None = None) -> Dict[str, Any]:
    cfg_path = os.path.expanduser(path or DEFAULT_CONFIG)
    with open(cfg_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    sm = _section(raw, "semantic_mapping")
    frames = _section(sm, "frames")
    topics = _section(sm, "topics")
    camera = _section(sm, "camera")
    projection = _section(sm, "projection")
    classes = _section(sm, "classes")
    detection = _section(sm, "detection_filter")
    low_conf = _section(sm, "low_confidence_policy")
    temporal = _section(sm, "temporal_vote")
    landmark = _section(sm, "landmark_fusion")
    viewpoints = _section(sm, "viewpoints")
    loop_quality = _section(sm, "loop_quality")
    storage = _section(sm, "storage")
    visualization = _section(sm, "visualization")

    return {
        "frames": {
            "fixed_frame": str(frames.get("fixed_frame", "map")),
            "fallback_frame": str(frames.get("fallback_frame", "odom")),
            "odom_frame": str(frames.get("odom_frame", "odom")),
            "base_frame": str(frames.get("base_frame", "base_link")),
            "laser_frame": str(frames.get("laser_frame", "laser")),
            "camera_frame": str(frames.get("camera_frame", "base_link")),
        },
        "topics": {k: str(v) for k, v in topics.items()},
        "camera": {
            "width": int(camera.get("width", 640)),
            "height": int(camera.get("height", 480)),
            "hfov_deg": _env_float(
                "SEMANTIC_CAMERA_HFOV_DEG",
                _as_float(camera.get("hfov_deg", 70.0), 70.0),
            ),
            "yaw_offset_deg": _as_float(camera.get("yaw_offset_deg", 0.0), 0.0),
        },
        "projection": {
            "min_range_m": _as_float(projection.get("min_range_m", 0.18), 0.18),
            "max_range_m": _as_float(projection.get("max_range_m", 4.0), 4.0),
            "target_window_deg": _as_float(projection.get("target_window_deg", 8.0), 8.0),
            "camera_to_laser_yaw_deg": _env_float(
                "SEMANTIC_CAMERA_TO_LASER_YAW_DEG",
                _as_float(projection.get("camera_to_laser_yaw_deg", 0.0), 0.0),
            ),
            "range_window_deg": _env_float(
                "SEMANTIC_RANGE_WINDOW_DEG",
                _as_float(projection.get("range_window_deg", 12.0), 12.0),
            ),
            "use_lidar_median": _as_bool(projection.get("use_lidar_median", True), True),
            "no_range_policy": str(projection.get("no_range_policy", "viewpoint_only")),
            "default_position_sigma_m": _as_float(
                projection.get("default_position_sigma_m", 1.20), 1.20
            ),
        },
        "classes": _resolve_semantic_classes(classes),
        "detection_filter": {
            "min_score_default": _as_float(detection.get("min_score_default", 0.25), 0.25),
            "min_score_small_object": _as_float(
                detection.get("min_score_small_object", 0.20), 0.20
            ),
            "min_score_observation_only": _as_float(
                detection.get("min_score_observation_only", 0.15), 0.15
            ),
            "min_area_ratio": _as_float(detection.get("min_area_ratio", 0.0006), 0.0006),
            "max_area_ratio": _as_float(detection.get("max_area_ratio", 0.45), 0.45),
            "edge_margin_px": _as_float(detection.get("edge_margin_px", 6.0), 6.0),
            "max_box_aspect_ratio": _as_float(
                detection.get("max_box_aspect_ratio", 6.0), 6.0
            ),
            "reject_stale": _as_bool(detection.get("reject_stale", True), True),
        },
        "low_confidence_policy": {
            "min_score_can_observe": _as_float(
                low_conf.get("min_score_can_observe", 0.15), 0.15
            ),
            "min_score_can_landmark": _as_float(
                low_conf.get("min_score_can_landmark", 0.20), 0.20
            ),
            "require_lidar_range": _as_bool(low_conf.get("require_lidar_range", True), True),
            "require_vote_count": int(low_conf.get("require_vote_count", 3)),
            "require_not_edge_box": _as_bool(
                low_conf.get("require_not_edge_box", True), True
            ),
        },
        "temporal_vote": {
            "window_size": int(temporal.get("window_size", 8)),
            "min_votes_candidate": int(temporal.get("min_votes_candidate", 2)),
            "min_votes_confirmed": int(temporal.get("min_votes_confirmed", 3)),
            "max_time_span_sec": _as_float(temporal.get("max_time_span_sec", 2.0), 2.0),
            "iou_threshold": _as_float(temporal.get("iou_threshold", 0.10), 0.10),
            "center_dist_threshold": _as_float(
                temporal.get("center_dist_threshold", 0.16), 0.16
            ),
            "hold_frames": int(temporal.get("hold_frames", 4)),
        },
        "landmark_fusion": {
            "merge_radius_small_m": _as_float(
                landmark.get("merge_radius_small_m", 0.45), 0.45
            ),
            "merge_radius_large_m": _as_float(
                landmark.get("merge_radius_large_m", 0.80), 0.80
            ),
            "confirm_seen_count": int(landmark.get("confirm_seen_count", 3)),
            "confirm_min_duration_sec": _as_float(
                landmark.get("confirm_min_duration_sec", 1.0), 1.0
            ),
            "max_position_sigma_m": _as_float(
                landmark.get("max_position_sigma_m", 0.80), 0.80
            ),
            "confidence_decay_sec": _as_float(
                landmark.get("confidence_decay_sec", 60.0), 60.0
            ),
            "min_viewpoint_baseline_m": _as_float(
                landmark.get("min_viewpoint_baseline_m", 0.15), 0.15
            ),
        },
        "viewpoints": {
            "enable": _as_bool(viewpoints.get("enable", True), True),
            "min_translation_m": _as_float(viewpoints.get("min_translation_m", 0.25), 0.25),
            "min_rotation_deg": _as_float(viewpoints.get("min_rotation_deg", 20.0), 20.0),
            "save_on_detection": _as_bool(viewpoints.get("save_on_detection", True), True),
            "view_range_m": _as_float(viewpoints.get("view_range_m", 3.0), 3.0),
            "view_fov_deg": _as_float(viewpoints.get("view_fov_deg", 70.0), 70.0),
            "save_keyframes": _as_bool(viewpoints.get("save_keyframes", True), True),
        },
        "loop_quality": {
            "enable": _as_bool(loop_quality.get("enable", True), True),
            "max_return_xy_error_m": _as_float(
                loop_quality.get("max_return_xy_error_m", 0.20), 0.20
            ),
            "max_return_yaw_error_deg": _as_float(
                loop_quality.get("max_return_yaw_error_deg", 12.0), 12.0
            ),
            "require_stationary_sec_before_save": _as_float(
                loop_quality.get("require_stationary_sec_before_save", 3.0), 3.0
            ),
        },
        "storage": {
            "root_dir": str(storage.get("root_dir", os.path.join(PROJECT_ROOT, "logs", "semantic_mapping"))),
            "session_name": str(storage.get("session_name", "auto")),
            "write_jsonl": _as_bool(storage.get("write_jsonl", True), True),
            "autosave_sec": _as_float(storage.get("autosave_sec", 2.0), 2.0),
            "save_keyframe_jpeg_quality": int(storage.get("save_keyframe_jpeg_quality", 70)),
        },
        "visualization": {
            "publish_markers": _as_bool(visualization.get("publish_markers", True), True),
            "marker_lifetime_sec": _as_float(
                visualization.get("marker_lifetime_sec", 0.0), 0.0
            ),
            "landmark_text": _as_bool(visualization.get("landmark_text", True), True),
            "observed_cone_markers": _as_bool(
                visualization.get("observed_cone_markers", True), True
            ),
            "show_depth_labels": _as_bool(
                visualization.get("show_depth_labels", True), True
            ),
        },
    }
