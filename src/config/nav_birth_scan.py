#!/usr/bin/env python3
"""Load birth-scan (startup wait + 360° survey) settings from nav YAML."""

from __future__ import annotations

import math
from typing import Any, Dict


def _section(raw: Dict[str, Any], key: str) -> Dict[str, Any]:
    block = raw.get(key, {})
    return block if isinstance(block, dict) else {}


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def birth_scan_duration_sec(scan_deg: float, scan_wz: float) -> float:
    wz = max(abs(float(scan_wz)), 1e-6)
    return math.radians(abs(float(scan_deg))) / wz


def load_birth_scan_config(cfg: Dict[str, Any]) -> Dict[str, Any]:
    birth = _section(cfg, "birth_scan")
    chassis = _section(cfg, "chassis")
    scan_wz = _as_float(birth.get("scan_wz", 0.03), 0.03)
    scan_deg = _as_float(birth.get("scan_deg", 360.0), 360.0)
    chassis_max_wz = max(_as_float(chassis.get("max_wz", 0.06), 0.06), 1e-6)
    effective_wz = min(abs(scan_wz), chassis_max_wz) if scan_wz else 0.03
    return {
        "enabled": _as_bool(birth.get("enabled", False), False),
        "wait_sec": max(0.0, _as_float(birth.get("wait_sec", 5.0), 5.0)),
        "scan_wz": scan_wz,
        "effective_scan_wz": effective_wz,
        "scan_deg": max(0.0, scan_deg),
        "turn_dir": 1.0 if _as_float(birth.get("turn_dir", 1.0), 1.0) >= 0.0 else -1.0,
        "scan_duration_sec": birth_scan_duration_sec(scan_deg, effective_wz),
    }
