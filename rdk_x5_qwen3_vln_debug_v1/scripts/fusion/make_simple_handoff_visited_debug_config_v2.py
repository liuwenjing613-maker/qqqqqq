#!/usr/bin/env python3
"""Runtime config for persistent visited-corridor visualization in simple_handoff V2."""
from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict

import yaml


def load_mapping(path: str) -> Dict[str, Any]:
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise SystemExit(f"YAML root must be a mapping: {path}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base",
        required=True,
        help="Base qwen_region_explore_debug.yaml from the repository root",
    )
    parser.add_argument(
        "--simple-config",
        required=True,
        help="simple_handoff_v2.yaml used to align corridor_radius_m",
    )
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--trajectory-file",
        required=True,
        help="Session trajectory JSON path (map-frame vertices)",
    )
    parser.add_argument("--log-root", required=True)
    args = parser.parse_args()

    cfg = deepcopy(load_mapping(args.base))
    simple_root = load_mapping(args.simple_config)
    simple = simple_root.get("simple_handoff_v2", simple_root)
    if not isinstance(simple, dict):
        raise SystemExit("simple_handoff_v2 root must be a mapping")

    revisit = simple.get("revisit", {}) or {}
    corridor_radius_m = float(revisit.get("corridor_radius_m", 0.50))

    trajectory = cfg.setdefault("trajectory", {})
    if not isinstance(trajectory, dict):
        raise SystemExit("trajectory section must be a mapping")
    trajectory["enabled"] = True
    trajectory["visited_corridor_radius_m"] = corridor_radius_m
    trajectory["runtime_file"] = str(args.trajectory_file)
    trajectory["persist_across_node_restart"] = False
    trajectory["publish_map_with_visited"] = True
    # Gray visited_area_grid is easy to confuse with local_costmap in Foxglove.
    trajectory["publish_visited_area_grid"] = False
    trajectory["publish_visited_map_cells"] = False
    trajectory["publish_path"] = False
    trajectory["publish_markers"] = False
    trajectory["draw_on_annotated_map"] = True
    trajectory["sample_period_s"] = 0.50
    trajectory["min_vertex_distance_m"] = 0.05

    node = cfg.setdefault("node", {})
    if isinstance(node, dict):
        node["analysis_period_s"] = 2.0

    logging_cfg = cfg.setdefault("logging", {})
    if not isinstance(logging_cfg, dict):
        raise SystemExit("logging section must be a mapping")
    logging_cfg["root_dir"] = str(args.log_root)
    logging_cfg["write_annotated_image"] = True
    logging_cfg["periodic_snapshot_s"] = 10.0

    safety = cfg.setdefault("safety", {})
    if isinstance(safety, dict):
        safety["observation_only"] = True
        safety["motion_enabled"] = False
        safety["qwen_enabled"] = False
        safety["nav2_enabled"] = False

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
