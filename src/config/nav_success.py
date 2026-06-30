#!/usr/bin/env python3
"""Load unified SUCCESS / arrive criteria from nav YAML configs."""

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


def load_success_config(cfg: Dict[str, Any]) -> Dict[str, Any]:
    """Merge success criteria from `success`, with legacy fallbacks in `arrive` / `fsm`."""
    success = _section(cfg, "success")
    arrive = _section(cfg, "arrive")
    fsm = _section(cfg, "fsm")
    qwen = _section(cfg, "qwen")
    safety = _section(cfg, "safety")

    arrive_frames = _as_int(
        success.get(
            "arrive_frames",
            success.get(
                "arrive_required_frames",
                arrive.get("arrive_required_frames", fsm.get("arrive_required_frames", 4)),
            ),
        ),
        4,
    )
    verify_frames = _as_int(
        success.get(
            "verify_frames",
            success.get("verify_required_frames", arrive_frames),
        ),
        arrive_frames,
    )

    legacy_min = _as_float(
        success.get("min_distance", arrive.get("arrive_min_distance", 0.58)),
        0.58,
    )
    legacy_max = _as_float(
        success.get("max_distance", arrive.get("arrive_max_distance", 0.75)),
        0.75,
    )
    hard_stop = _as_float(safety.get("hard_stop_distance", 0.35), 0.35)

    return {
        "require_lidar": _as_bool(
            success.get("require_lidar", fsm.get("require_lidar", safety.get("require_lidar", True))),
            True,
        ),
        "center_px": _as_float(success.get("center_px", arrive.get("center_px", 70)), 70.0),
        "min_safe_distance": _as_float(
            success.get("min_safe_distance", success.get("min_distance", max(legacy_min, hard_stop))),
            max(legacy_min, hard_stop),
        ),
        "stop_distance": _as_float(
            success.get("stop_distance", success.get("max_distance", legacy_max)),
            legacy_max,
        ),
        "verify_distance_max": _as_float(
            success.get("verify_distance_max", success.get("stop_distance", legacy_max) + 0.10),
            legacy_max + 0.10,
        ),
        "min_area_ratio": _as_float(
            success.get("min_area_ratio", arrive.get("arrive_area_ratio", 0.16)),
            0.16,
        ),
        "center_only_enabled": _as_bool(
            success.get("center_only_enabled", arrive.get("center_only_arrive_enabled", False)),
            False,
        ),
        "arrive_frames": max(1, arrive_frames),
        "verify_frames": max(1, verify_frames),
        "qwen_verify_required": _as_bool(
            success.get("qwen_verify_required", fsm.get("qwen_verify_required", False)),
            False,
        ),
        "qwen_verify_timeout_sec": _as_float(
            success.get("qwen_verify_timeout_sec", fsm.get("qwen_verify_timeout_sec", qwen.get("timeout_sec", 12.0))),
            12.0,
        ),
        "qwen_verify_fail_policy": str(
            success.get("qwen_verify_fail_policy", fsm.get("qwen_verify_fail_policy", "search"))
        ),
        "emergency_stop_distance": _as_float(
            safety.get("emergency_stop_distance", 0.25),
            0.25,
        ),
    }
