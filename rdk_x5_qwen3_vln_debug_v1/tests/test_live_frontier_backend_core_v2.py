#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from fusion.live_frontier_backend_core_v2 import (  # noqa: E402
    FrontierConfig,
    GridMeta,
    RobotPose2D,
    candidate_summary_payload,
    extract_frontier_candidates,
    parse_qwen_candidate_id,
)


def build_t_junction_map():
    h = w = 120
    # Obstacles surround the known corridors. Unknown pockets exist only after
    # the three exits, so each exit is a separate frontier component.
    grid = np.full((h, w), 100, dtype=np.int16)
    grid[50:70, 50:70] = 0
    grid[55:65, 20:50] = 0
    grid[55:65, 70:100] = 0
    grid[70:100, 55:65] = 0
    grid[55:65, 10:20] = -1
    grid[55:65, 100:110] = -1
    grid[100:110, 55:65] = -1
    return grid


def test_extract_and_summary():
    grid = build_t_junction_map()
    meta = GridMeta(120, 120, 0.05, -3.0, -3.0)
    # Robot below the T, facing north.
    robot = RobotPose2D(0.0, 0.0, math.pi / 2)
    cfg = FrontierConfig(
        obstacle_inflation_m=0.12,
        min_frontier_cells=4,
        min_goal_distance_m=0.30,
        max_goal_distance_m=3.0,
        min_heading_separation_deg=25.0,
        max_candidates=8,
    )
    candidates, diag = extract_frontier_candidates(grid, meta, robot, cfg)
    assert len(candidates) >= 2, (candidates, diag)
    headings = sorted(c.heading_deg for c in candidates)
    assert headings[-1] - headings[0] >= 45.0, headings
    summary = candidate_summary_payload(candidates, map_version="test", probe_id="p1")
    assert summary["decision_distance_m"] is not None
    assert len(summary["candidates"]) == len(candidates)
    ids = [c.candidate_id for c in candidates]
    selected, payload = parse_qwen_candidate_id(
        json.dumps({"candidate_id": ids[0], "confidence": 0.8}), ids
    )
    assert selected == ids[0]
    try:
        parse_qwen_candidate_id('{"candidate_id":"invented"}', ids)
    except ValueError:
        pass
    else:
        raise AssertionError("invented candidate should be rejected")


def test_single_connected_frontier_splits_by_heading():
    grid = np.full((140, 140), -1, dtype=np.int16)
    # One large known free rectangle. Its frontier is a single connected ring,
    # but the extractor must still propose directionally distinct choices.
    grid[35:105, 35:105] = 0
    meta = GridMeta(140, 140, 0.05, -3.5, -3.5)
    robot = RobotPose2D(0.0, 0.0, math.pi / 2)
    cfg = FrontierConfig(
        obstacle_inflation_m=0.10,
        min_frontier_cells=5,
        min_goal_distance_m=0.5,
        max_goal_distance_m=3.0,
        max_abs_relative_heading_deg=150.0,
        min_heading_separation_deg=30.0,
        max_candidates=8,
    )
    candidates, diag = extract_frontier_candidates(grid, meta, robot, cfg)
    assert len(candidates) >= 3, (candidates, diag)
    headings = sorted(c.heading_deg for c in candidates)
    assert headings[-1] - headings[0] >= 80.0, headings


if __name__ == "__main__":
    test_extract_and_summary()
    test_single_connected_frontier_splits_by_heading()
    print("test_live_frontier_backend_core_v2: PASS")
