#!/usr/bin/env python3
"""Create a servo-rate Qwen config without editing the user's tuned config."""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--observe", type=float, default=1.0)
    parser.add_argument("--track", type=float, default=0.90)
    parser.add_argument("--search", type=float, default=1.20)
    args = parser.parse_args()

    source = Path(args.input)
    target = Path(args.output)
    config = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    state = config.setdefault("state_machine", {})
    state["observe_interval_sec"] = float(args.observe)
    state["track_interval_sec"] = float(args.track)
    state["search_interval_sec"] = float(args.search)
    # The node has a single in-flight Future, so a rare >1 s request cannot
    # create an API backlog even though track_interval_sec is 0.9 s.
    visual = config.setdefault("visualization", {})
    visual["publish_hz"] = max(8.0, float(visual.get("publish_hz", 8.0)))
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        yaml.safe_dump(config, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    print(f"generated {target} from {source}")
    print(
        "only changed intervals: "
        f"observe={args.observe}s track={args.track}s search={args.search}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
