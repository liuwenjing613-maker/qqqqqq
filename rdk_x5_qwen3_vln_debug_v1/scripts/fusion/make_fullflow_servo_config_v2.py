#!/usr/bin/env python3
"""Create a temporary V1 servo config with conservative full-flow overrides."""
from __future__ import annotations

import argparse
from pathlib import Path
import yaml


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    cfg = yaml.safe_load(Path(args.input).read_text(encoding="utf-8")) or {}
    third = cfg.setdefault("third_view_intervention", {})
    third["enabled"] = True
    branch = third.setdefault("branch", {})
    branch.update(
        {
            "min_candidates": 2,
            "min_heading_separation_deg": 50.0,
            "decision_radius_m": 1.80,
            "max_top_score_gap": 0.14,
            "max_top_score_ratio": 1.30,
            "persistence_updates": 2,
        }
    )
    handshake = third.setdefault("handshake", {})
    handshake["navigation_timeout_sec"] = 180.0
    handshake["request_ack_timeout_sec"] = 8.0
    Path(args.output).write_text(
        yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
