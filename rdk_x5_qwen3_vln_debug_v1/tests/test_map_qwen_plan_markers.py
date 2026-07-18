#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from fusion.live_frontier_backend_core_v2 import FrontierCandidate
from fusion.map_qwen_plan_markers import (
    build_candidate_markers,
    build_clear_markers,
    build_selected_goal_markers,
)
from builtin_interfaces.msg import Time


class MapQwenPlanMarkersTest(unittest.TestCase):
    def test_candidate_and_goal_marker_counts(self) -> None:
        candidates = [
            FrontierCandidate(
                candidate_id="F_a",
                x=1.0,
                y=2.0,
                yaw=0.1,
                grid_x=10,
                grid_y=20,
                heading_deg=10.0,
                distance_m=1.2,
                information_gain=5,
                clearance_m=0.4,
                score=0.7,
                cluster_cells=8,
            ),
            FrontierCandidate(
                candidate_id="F_b",
                x=2.0,
                y=3.0,
                yaw=0.2,
                grid_x=11,
                grid_y=21,
                heading_deg=-5.0,
                distance_m=1.5,
                information_gain=6,
                clearance_m=0.5,
                score=0.8,
                cluster_cells=9,
            ),
        ]
        stamp = Time(sec=1, nanosec=0)
        cand_arr = build_candidate_markers(
            candidates, frame_id="map", stamp=stamp, selected_id="F_b"
        )
        self.assertEqual(len(cand_arr.markers), 1 + 2 * 2)
        goal_arr = build_selected_goal_markers(candidates[1], frame_id="map", stamp=stamp)
        self.assertEqual(len(goal_arr.markers), 4)
        cleared = build_clear_markers("map", stamp)
        self.assertEqual(len(cleared.markers), 2)


if __name__ == "__main__":
    unittest.main()
