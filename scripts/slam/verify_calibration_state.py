#!/usr/bin/env python3
"""Verify /chassis_bridge_state JSON contains expected SLAM calibration fields."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Dict

EXPECTED: Dict[str, Any] = {
    "motor_trims": [1.15, 1.15, 1.0, 1.0],
    "odom_vx_scale": 0.68,
    "odom_vy_scale": 1.0,
    "odom_wz_scale": -0.58,
    "odom_use_vy": False,
    "odom_vxy_deadzone": 0.003,
    "odom_wz_deadzone": 0.015,
}


def parse_ros_string_msg(raw: str) -> dict:
    line = ""
    for part in raw.splitlines():
        part = part.strip()
        if part.startswith("data:"):
            line = part.split("data:", 1)[1].strip().strip("'\"")
            break
    if not line:
        raise ValueError("no data: field in /chassis_bridge_state message")
    return json.loads(line)


def main() -> int:
    log_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    raw = sys.stdin.read()
    if not raw.strip():
        print("ERROR: empty /chassis_bridge_state message", file=sys.stderr)
        return 1

    try:
        state = parse_ros_string_msg(raw)
    except (ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: failed to parse chassis_bridge_state: {exc}", file=sys.stderr)
        return 1

    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    errors = []
    for key, expected in EXPECTED.items():
        actual = state.get(key, "<missing>")
        if actual != expected:
            errors.append(f"{key}: expected {expected!r}, got {actual!r}")

    if errors:
        print("ERROR: calibration parameters NOT loaded correctly:", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        return 1

    print("OK: all calibration fields verified")
    for key in EXPECTED:
        print(f"  {key} = {state[key]!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
