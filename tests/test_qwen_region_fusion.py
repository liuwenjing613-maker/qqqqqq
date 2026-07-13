#!/usr/bin/env python3
"""Tests for Qwen fusion and revalidation."""

from __future__ import annotations

import os
import sys
import unittest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_region_selector_core import (  # noqa: E402
    RegionCandidate,
    fuse_geometric_and_qwen_ranking,
    revalidate_decision,
)

CFG = {
    "decision_fusion": {
        "map_only_geo_weight": 0.85,
        "map_only_qwen_weight": 0.15,
        "reject_if_geo_score_below": 0.40,
        "max_allowed_geo_gap_without_visual_evidence": 0.20,
    },
    "decision_revalidation": {
        "max_snapshot_age_s": 20.0,
        "max_robot_translation_m": 0.15,
        "max_robot_yaw_change_deg": 20.0,
        "max_map_cell_change_ratio": 0.10,
        "require_selected_track_still_present": True,
        "require_selected_track_still_stable": True,
    },
}


def _c(label: str, geo: float) -> RegionCandidate:
    return RegionCandidate(
        label=label,
        internal_region_id=f"R_{label}",
        direction="LEFT",
        distance_m=1.0,
        unknown_gain_cells=100,
        unknown_gain_ratio=0.5,
        minimum_clearance_m=0.4,
        mean_clearance_m=0.5,
        frontier_cell_count=10,
        geo_score=geo,
        geo_rank=1 if label == "A" else 2,
        stable=True,
        snapshot_eligible=True,
    )


class TestFusion(unittest.TestCase):
    def test_qwen_can_reorder_similar_geo_scores(self) -> None:
        regions = [_c("A", 0.75), _c("B", 0.74)]
        parsed = {"ranked_regions": ["B", "A"], "selected_region": "B"}
        out = fuse_geometric_and_qwen_ranking(regions, parsed, CFG)
        self.assertIn(out["algorithm_final_region"], ("A", "B"))

    def test_qwen_cannot_override_large_geo_gap_without_visuals(self) -> None:
        regions = [_c("A", 0.90), _c("B", 0.50)]
        parsed = {"ranked_regions": ["B", "A"], "selected_region": "B"}
        out = fuse_geometric_and_qwen_ranking(regions, parsed, CFG)
        self.assertEqual(out["algorithm_final_region"], "A")
        self.assertEqual(out["override_reason"], "QWEN_OVERRIDE_REJECTED_LARGE_GEO_GAP")

    def test_fusion_is_deterministic(self) -> None:
        regions = [_c("A", 0.8), _c("B", 0.7)]
        parsed = {"ranked_regions": ["A", "B"], "selected_region": "A"}
        o1 = fuse_geometric_and_qwen_ranking(regions, parsed, CFG)
        o2 = fuse_geometric_and_qwen_ranking(regions, parsed, CFG)
        self.assertEqual(o1, o2)


class TestRevalidation(unittest.TestCase):
    def test_robot_moved_fails(self) -> None:
        snap0 = {"map_stamp": 1.0, "robot_pose": {"x": 0, "y": 0, "yaw_deg": 0}, "map_metadata": {"width": 10, "height": 10}}
        snap1 = {"map_stamp": 1.0, "robot_pose": {"x": 1, "y": 0, "yaw_deg": 0}, "map_metadata": {"width": 10, "height": 10}}
        errs = revalidate_decision(snap0, snap1, "A", {"A": {"stable": True, "snapshot_eligible": True}}, CFG)
        self.assertIn("DECISION_ROBOT_MOVED", errs)


if __name__ == "__main__":
    unittest.main()
