#!/usr/bin/env python3
"""Compatibility copy used by start_live_servo_with_intervention.sh.

The current branch wrapper calls this path, while the checked-in helper lives in
scripts/intervention.  Keeping this tiny implementation here removes that path
mismatch and only rewrites the runtime cmd_output topic.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    src = Path(args.input)
    dst = Path(args.output)
    cfg = yaml.safe_load(src.read_text(encoding="utf-8")) or {}
    third = cfg.get("third_view_intervention", {}) or {}
    topics = third.get("topics", {}) or {}
    ego_cmd = str(topics.get("ego_cmd", "/cmd_vel_ego"))
    cfg.setdefault("topics", {})["cmd_output"] = ego_cmd
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(
        yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(dst)


if __name__ == "__main__":
    main()
