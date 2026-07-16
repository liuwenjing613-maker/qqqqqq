#!/usr/bin/env python3
"""Tests for global region proposal map validation."""

from __future__ import annotations

import copy
import json
import os
import sys
import unittest
from pathlib import Path

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.frontier_region_debug_core import (  # noqa: E402
    MapMetadata,
    RobotPose2D,
    analyze_frontier_regions,
    build_map_render_metadata,
    deep_copy_grid_data,
)
from src.planning.global_region_proposal_validator import (  # noqa: E402
    build_region_from_global_proposal,
    build_selected_region_geometry_from_global,
    normalized_viewport_to_grid,
    proposal_bbox_to_grid_bounds,
    validate_global_region_proposal,
)
from src.vlm.qwen_global_region_selector_core import (  # noqa: E402
    GlobalRegionProposal,
    validate_global_region_proposal_response,
)


FIXTURES = Path(__file__).parent / "fixtures"


def _meta(w: int = 30, h: int = 30) -> MapMetadata:
    return MapMetadata(width=w, height=h, resolution=0.05, origin_x=0.0, origin_y=0.0, frame_id="map", stamp_sec=0.0)


def _door_map() -> np.ndarray:
    h, w = 30, 30
    data = np.full((h, w), -1, dtype=np.int16)
    data[5:25, 5:25] = 0
    data[5:25, 24] = -1
    return data


def _cfg() -> dict:
    return {
        "map_values": {"unknown_value": -1, "free_max": 20, "occupied_min": 65},
        "map_health": {"require_robot_inside_map": True, "reject_robot_cell_occupied": True},
        "frontier": {"min_clearance_m": 0.0},
        "region": {
            "min_frontier_cells": 3,
            "min_distance_m": 0.10,
            "max_distance_m": 10.0,
            "min_unknown_gain_cells": 1,
            "unknown_gain_radius_m": 1.0,
            "max_regions": 12,
        },
        "region_merge": {"enabled": False},
        "global_region_proposal": {
            "require_frontier_match": True,
            "max_frontier_snap_distance_m": 0.75,
            "minimum_frontier_overlap_ratio": 0.01,
            "maximum_occupied_ratio": 0.25,
            "require_region_stable": False,
        },
    }


def _metadata() -> dict:
    return build_map_render_metadata(_meta(), snapshot_id="RS_TEST")


def _proposal(u: float, v: float, du: float = 0.08, dv: float = 0.08) -> GlobalRegionProposal:
    return GlobalRegionProposal(
        proposal_id="GP_1",
        rank=1,
        region_type="FRONTIER_EXPLORATION_REGION",
        map_image_center_u=u,
        map_image_center_v=v,
        bbox_u_min=max(0.0, u - du),
        bbox_v_min=max(0.0, v - dv),
        bbox_u_max=min(1.0, u + du),
        bbox_v_max=min(1.0, v + dv),
    )


class TestMapMapping(unittest.TestCase):
    def test_normalized_viewport_to_grid(self) -> None:
        meta = _meta()
        md = _metadata()
        row, col = normalized_viewport_to_grid(0.5, 0.5, md, meta)
        self.assertTrue(0 <= row < meta.height)
        self.assertTrue(0 <= col < meta.width)

    def test_map_padding_not_used_as_map(self) -> None:
        md = build_map_render_metadata(_meta(), snapshot_id="RS", panel_size_px=900, content_padding_px=40)
        md["map_panel"]["content_x_min_px"] = 100
        md["map_panel"]["content_y_min_px"] = 100
        row_edge, col_edge = normalized_viewport_to_grid(0.0, 0.0, md, _meta())
        row_center, col_center = normalized_viewport_to_grid(0.5, 0.5, md, _meta())
        self.assertNotEqual((row_edge, col_edge), (row_center, col_center))

    def test_vertical_flip_conversion(self) -> None:
        meta = _meta(10, 10)
        md = build_map_render_metadata(meta, snapshot_id="RS")
        _, col_top = normalized_viewport_to_grid(0.5, 0.0, md, meta)
        _, col_bot = normalized_viewport_to_grid(0.5, 1.0, md, meta)
        self.assertEqual(col_top, col_bot)

    def test_map_origin_rotation(self) -> None:
        meta = MapMetadata(
            width=20, height=20, resolution=0.05, origin_x=1.0, origin_y=2.0, frame_id="map", stamp_sec=0.0
        )
        md = build_map_render_metadata(meta, snapshot_id="RS")
        row, col = normalized_viewport_to_grid(0.5, 0.5, md, meta)
        self.assertTrue(0 <= row < 20)

    def test_original_map_not_mutated(self) -> None:
        data = _door_map()
        before = deep_copy_grid_data(data)
        robot = RobotPose2D(0.5, 0.5, 0.0)
        result = analyze_frontier_regions(data, _meta(), robot, _cfg(), cycle_id=1)
        proposal = _proposal(0.75, 0.5)
        validate_global_region_proposal(
            proposal, data=data, meta=_meta(), metadata=_metadata(), result=result, cfg=_cfg()
        )
        self.assertTrue(np.array_equal(before, data))


class TestProposalValidation(unittest.TestCase):
    def _analyze(self):
        data = _door_map()
        robot = RobotPose2D(0.5, 0.5, 0.0)
        return data, analyze_frontier_regions(data, _meta(), robot, _cfg(), cycle_id=1)

    def test_valid_proposal_matches_real_frontier(self) -> None:
        data, result = self._analyze()
        outcome = validate_global_region_proposal(
            _proposal(0.78, 0.45),
            data=data,
            meta=_meta(),
            metadata=_metadata(),
            result=result,
            cfg=_cfg(),
        )
        self.assertTrue(outcome.validation_passed, outcome.rejection_reasons)

    def test_wall_proposal_rejected(self) -> None:
        data = np.full((30, 30), 100, dtype=np.int16)
        data[10:20, 10:20] = 0
        robot = RobotPose2D(0.5, 0.5, 0.0)
        result = analyze_frontier_regions(data, _meta(), robot, _cfg(), cycle_id=2)
        outcome = validate_global_region_proposal(
            _proposal(0.15, 0.15),
            data=data,
            meta=_meta(),
            metadata=_metadata(),
            result=result,
            cfg=_cfg(),
        )
        self.assertFalse(outcome.validation_passed)

    def test_no_frontier_proposal_rejected(self) -> None:
        data = np.zeros((30, 30), dtype=np.int16)
        data[0, :] = 100
        data[-1, :] = 100
        data[:, 0] = 100
        data[:, -1] = 100
        robot = RobotPose2D(0.5, 0.5, 0.0)
        result = analyze_frontier_regions(data, _meta(), robot, _cfg(), cycle_id=3)
        self.assertEqual(0, result.stats.raw_frontier_cells)
        outcome = validate_global_region_proposal(
            _proposal(0.50, 0.50),
            data=data,
            meta=_meta(),
            metadata=_metadata(),
            result=result,
            cfg=_cfg(),
        )
        self.assertFalse(outcome.validation_passed)

    def test_proposal_can_snap_to_nearby_frontier(self) -> None:
        data, result = self._analyze()
        outcome = validate_global_region_proposal(
            _proposal(0.70, 0.50),
            data=data,
            meta=_meta(),
            metadata=_metadata(),
            result=result,
            cfg=_cfg(),
        )
        self.assertLess(outcome.nearest_frontier_distance_m, 0.75)

    def test_blacklisted_region_rejected(self) -> None:
        data, result = self._analyze()
        if not result.regions:
            self.skipTest("no regions")
        region = result.regions[0]
        region.blacklisted = True
        outcome = validate_global_region_proposal(
            _proposal(0.78, 0.45),
            data=data,
            meta=_meta(),
            metadata=_metadata(),
            result=result,
            cfg=_cfg(),
        )
        self.assertFalse(outcome.validation_passed)


class TestPhase3AIntegration(unittest.TestCase):
    def test_valid_global_region_builds_selected_region_geometry(self) -> None:
        data = _door_map()
        robot = RobotPose2D(0.5, 0.5, 0.0)
        result = analyze_frontier_regions(data, _meta(), robot, _cfg(), cycle_id=4)
        proposal = _proposal(0.78, 0.45)
        outcome = validate_global_region_proposal(
            proposal, data=data, meta=_meta(), metadata=_metadata(), result=result, cfg=_cfg()
        )
        self.assertTrue(outcome.validation_passed)
        entry = build_region_from_global_proposal(proposal, outcome, _meta())
        geom = build_selected_region_geometry_from_global(entry, "RS_TEST")
        self.assertIn("frontier_cells_grid", geom)
        self.assertIn("region_geometry_fingerprint", geom)

    def test_path_checked_remains_false(self) -> None:
        data = _door_map()
        robot = RobotPose2D(0.5, 0.5, 0.0)
        result = analyze_frontier_regions(data, _meta(), robot, _cfg(), cycle_id=5)
        proposal = _proposal(0.78, 0.45)
        outcome = validate_global_region_proposal(
            proposal, data=data, meta=_meta(), metadata=_metadata(), result=result, cfg=_cfg()
        )
        entry = build_region_from_global_proposal(proposal, outcome, _meta())
        self.assertFalse(entry["path_checked"])
        self.assertIsNone(entry["reachable"])


class TestFallbackPipeline(unittest.TestCase):
    def test_second_proposal_used_when_first_invalid(self) -> None:
        raw = json.loads((FIXTURES / "global_region_proposal_wall.json").read_text())
        raw["ranked_region_proposals"].append(
            json.loads((FIXTURES / "global_region_proposal_valid.json").read_text())["ranked_region_proposals"][0]
        )
        raw["ranked_region_proposals"][1]["proposal_id"] = "GP_2"
        raw["ranked_region_proposals"][1]["rank"] = 2
        self.assertEqual(2, len(raw["ranked_region_proposals"]))

    def test_no_silent_fallback(self) -> None:
        from src.vlm.qwen_region_selector_core import unified_decision_dict

        d = unified_decision_dict(
            {
                "configured_strategy": "GLOBAL_REGION_PROPOSAL",
                "effective_strategy": "CANDIDATE_RANKING",
                "fallback_used": True,
                "fallback_reason": "GLOBAL_PROPOSAL_ALL_REJECTED",
                "global_proposal_failures": [{"proposal_id": "GP_1"}],
                "validation": type("V", (), {"decision_valid": False, "errors": []})(),
                "decision": type("D", (), {"selected_region": "A"})(),
                "fusion": {},
            },
            snapshot_id="RS_1",
        )
        self.assertTrue(d["fallback_used"])
        self.assertEqual("GLOBAL_PROPOSAL_ALL_REJECTED", d["fallback_reason"])


if __name__ == "__main__":
    unittest.main()
