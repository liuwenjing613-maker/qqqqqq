#!/usr/bin/env python3
"""Load multi-frame target voter settings from nav YAML configs."""

from __future__ import annotations

from typing import Any, Dict


def _section(raw: Dict[str, Any], key: str) -> Dict[str, Any]:
    block = raw.get(key, {})
    return block if isinstance(block, dict) else {}


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def load_voter_config(cfg: Dict[str, Any]) -> Dict[str, Any]:
    voter = _section(cfg, "voter")
    yolo_bridge = _section(cfg, "yolo_bridge")

    return {
        "enabled": _as_bool(voter.get("enabled", False), False),
        "window_size": max(1, _as_int(voter.get("window_size", yolo_bridge.get("voter_window_size", 6)), 6)),
        "min_votes": max(1, _as_int(voter.get("min_votes", yolo_bridge.get("voter_min_votes", 2)), 2)),
        "lost_hold_frames": max(
            0,
            _as_int(voter.get("lost_hold_frames", yolo_bridge.get("voter_lost_hold_frames", 4)), 4),
        ),
        "iou_threshold": _as_float(voter.get("iou_threshold", 0.05), 0.05),
        "center_dist_threshold": _as_float(voter.get("center_dist_threshold", 0.18), 0.18),
        "smooth_alpha": _as_float(voter.get("smooth_alpha", 0.20), 0.20),
        "accept_held_target": _as_bool(voter.get("accept_held_target", True), True),
    }
