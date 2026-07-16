#!/usr/bin/env python3
"""Create a runtime servo config that routes ego velocity into the new mux."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    src = Path(args.input)
    dst = Path(args.output)
    data = yaml.safe_load(src.read_text(encoding="utf-8")) or {}
    intervention = data.get("third_view_intervention", {}) or {}
    if not bool(intervention.get("enabled", False)):
        raise SystemExit(
            "third_view_intervention.enabled is false; runtime remap is not needed"
        )

    topics = intervention.get("topics", {}) or {}
    ego_topic = str(topics.get("ego_cmd", "/cmd_vel_ego")).strip() or "/cmd_vel_ego"
    data.setdefault("topics", {})["cmd_output"] = ego_topic

    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    print(f"[intervention] runtime servo cmd_output -> {ego_topic}: {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
