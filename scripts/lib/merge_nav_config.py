#!/usr/bin/env python3
"""Deep-merge nav yaml configs (base + overlay)."""

from __future__ import annotations

import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict

import yaml


def deep_merge(base: Any, overlay: Any) -> Any:
    if not isinstance(base, dict):
        return deepcopy(overlay)
    out = deepcopy(base)
    for key, value in (overlay or {}).items():
        if key in out and isinstance(out[key], dict) and isinstance(value, dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = deepcopy(value)
    return out


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: merge_nav_config.py <base.yaml> <overlay.yaml> <out.yaml>",
            file=sys.stderr,
        )
        return 2
    base_path, overlay_path, out_path = map(Path, sys.argv[1:4])
    with base_path.open("r", encoding="utf-8") as f:
        base = yaml.safe_load(f) or {}
    with overlay_path.open("r", encoding="utf-8") as f:
        overlay = yaml.safe_load(f) or {}
    merged: Dict[str, Any] = deep_merge(base, overlay)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(merged, f, allow_unicode=True, sort_keys=False)
    print(str(out_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
