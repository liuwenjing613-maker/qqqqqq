#!/usr/bin/env python3
"""Tests for global region proposal core and strategy dispatcher."""

from __future__ import annotations

import json
import os
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_global_region_selector_core import (  # noqa: E402
    STRATEGY_CANDIDATE_RANKING,
    STRATEGY_GLOBAL_REGION_PROPOSAL,
    build_global_prompt_context_from_snapshot,
    build_global_region_proposal_prompt,
    validate_global_region_proposal_response,
    validate_region_selection_config,
)
from src.vlm.qwen_region_selector_core import (  # noqa: E402
    build_prompt_for_strategy,
    load_region_snapshot,
    select_exploration_region,
)


FIXTURES = Path(__file__).parent / "fixtures"
SNAPSHOT_PATH = (
    Path(PROJECT_ROOT)
    / "logs/qwen_region_explore/20260713_174148/snapshots/RS_20260713T094328_0092/region_snapshot.json"
)


def _cfg(strategy: str = STRATEGY_GLOBAL_REGION_PROPOSAL) -> dict:
    return {
        "region_selection": {
            "strategy": strategy,
            "fallback_strategy": STRATEGY_CANDIDATE_RANKING,
            "strict_strategy_validation": True,
        },
        "global_region_proposal": {
            "enabled": True,
            "max_ranked_proposals": 3,
            "coordinate_space": "NORMALIZED_MAP_VIEWPORT",
            "require_frontier_match": True,
            "max_frontier_snap_distance_m": 0.75,
            "minimum_bbox_width_normalized": 0.04,
            "minimum_bbox_height_normalized": 0.04,
            "maximum_bbox_width_normalized": 0.45,
            "maximum_bbox_height_normalized": 0.45,
            "minimum_frontier_overlap_ratio": 0.10,
            "maximum_occupied_ratio": 0.25,
            "allow_map_only_fallback": True,
            "try_next_proposal_on_validation_failure": True,
            "fallback_to_candidate_ranking": True,
        },
        "decision_fusion": {
            "map_only_geo_weight": 0.85,
            "map_only_qwen_weight": 0.15,
            "reject_if_geo_score_below": 0.40,
        },
        "map_values": {"unknown_value": -1, "free_max": 20, "occupied_min": 65},
    }


class TestStrategyConfig(unittest.TestCase):
    def test_candidate_ranking_strategy_preserved(self) -> None:
        cfg = _cfg(STRATEGY_CANDIDATE_RANKING)
        self.assertEqual([], validate_region_selection_config(cfg))

    def test_global_region_strategy_selected(self) -> None:
        cfg = _cfg(STRATEGY_GLOBAL_REGION_PROPOSAL)
        self.assertEqual([], validate_region_selection_config(cfg))

    def test_invalid_strategy_rejected(self) -> None:
        cfg = _cfg("INVALID_MODE")
        self.assertTrue(any("strategy invalid" in e for e in validate_region_selection_config(cfg)))

    def test_fallback_strategy_validation(self) -> None:
        cfg = _cfg()
        cfg["region_selection"]["fallback_strategy"] = "BAD"
        self.assertTrue(any("fallback_strategy" in e for e in validate_region_selection_config(cfg)))


class TestGlobalPrompt(unittest.TestCase):
    def test_global_prompt_contains_complete_map_rules(self) -> None:
        ctx = build_global_prompt_context_from_snapshot(
            {"snapshot_id": "RS_1", "robot_pose": {}, "map_metadata": {}},
            target_instruction="find kitchen",
        )
        prompt = build_global_region_proposal_prompt(ctx, _cfg())
        self.assertIn("FREE represents known traversable space", prompt)
        self.assertIn("FRONTIER BOUNDARY", prompt)

    def test_global_prompt_contains_trajectory_rules(self) -> None:
        ctx = build_global_prompt_context_from_snapshot(
            {"snapshot_id": "RS_1", "robot_pose": {}, "map_metadata": {}},
            target_instruction="explore",
        )
        prompt = build_global_region_proposal_prompt(ctx, _cfg())
        self.assertIn("TRAVELED PATH", prompt)
        self.assertIn("VISITED CORRIDOR", prompt)

    def test_global_prompt_contains_360_view_rules(self) -> None:
        ctx = build_global_prompt_context_from_snapshot(
            {"snapshot_id": "RS_1", "robot_pose": {}, "map_metadata": {}},
            target_instruction="explore",
            view_ids=["VIEW_000", "VIEW_090"],
        )
        prompt = build_global_region_proposal_prompt(ctx, _cfg())
        self.assertIn("VIEW_000", prompt)
        self.assertIn("VIEW_090", prompt)

    def test_global_prompt_forbids_navigation_coordinates(self) -> None:
        prompt = build_global_region_proposal_prompt(
            build_global_prompt_context_from_snapshot(
                {"snapshot_id": "RS_1", "robot_pose": {}, "map_metadata": {}},
                target_instruction="x",
            ),
            _cfg(),
        )
        self.assertIn("Do not output map x/y coordinates", prompt)
        self.assertIn("Do not claim path_checked", prompt)

    def test_global_prompt_requires_frontier_region(self) -> None:
        prompt = build_global_region_proposal_prompt(
            build_global_prompt_context_from_snapshot(
                {"snapshot_id": "RS_1", "robot_pose": {}, "map_metadata": {}},
                target_instruction="x",
            ),
            _cfg(),
        )
        self.assertIn("FRONTIER_EXPLORATION_REGION", prompt)

    def test_global_prompt_handles_center_obstacle_side_paths(self) -> None:
        prompt = build_global_region_proposal_prompt(
            build_global_prompt_context_from_snapshot(
                {"snapshot_id": "RS_1", "robot_pose": {}, "map_metadata": {}},
                target_instruction="x",
            ),
            _cfg(),
        )
        self.assertIn("When an obstacle or wall occupies the central direction", prompt)

    def test_global_prompt_allows_empty_proposals(self) -> None:
        prompt = build_global_region_proposal_prompt(
            build_global_prompt_context_from_snapshot(
                {"snapshot_id": "RS_1", "robot_pose": {}, "map_metadata": {}},
                target_instruction="x",
            ),
            _cfg(),
        )
        self.assertIn("empty ranked_region_proposals array", prompt)


class TestGlobalJsonValidation(unittest.TestCase):
    def _load(self, name: str) -> str:
        return (FIXTURES / name).read_text(encoding="utf-8")

    def test_valid_global_response(self) -> None:
        result = validate_global_region_proposal_response(
            self._load("global_region_proposal_valid.json"),
            expected_snapshot_id="RS_TEST_GLOBAL_0001",
            expected_visual_context_id="VC_TEST_0001",
            valid_view_ids={"VIEW_090", "VIEW_000"},
            cfg=_cfg(),
        )
        self.assertTrue(result.valid)
        self.assertIsNotNone(result.response)
        self.assertEqual(1, len(result.response.ranked_region_proposals))

    def test_center_out_of_range_rejected(self) -> None:
        raw = json.loads(self._load("global_region_proposal_valid.json"))
        raw["ranked_region_proposals"][0]["map_image_center"]["u"] = 1.5
        result = validate_global_region_proposal_response(
            json.dumps(raw),
            expected_snapshot_id="RS_TEST_GLOBAL_0001",
            expected_visual_context_id="VC_TEST_0001",
            valid_view_ids={"VIEW_090"},
            cfg=_cfg(),
        )
        self.assertFalse(result.valid)

    def test_bbox_out_of_range_rejected(self) -> None:
        raw = json.loads(self._load("global_region_proposal_valid.json"))
        raw["ranked_region_proposals"][0]["map_image_bbox"]["u_max"] = 1.2
        result = validate_global_region_proposal_response(
            json.dumps(raw),
            expected_snapshot_id="RS_TEST_GLOBAL_0001",
            expected_visual_context_id="VC_TEST_0001",
            valid_view_ids={"VIEW_090"},
            cfg=_cfg(),
        )
        self.assertFalse(result.valid)

    def test_center_outside_bbox_rejected(self) -> None:
        raw = json.loads(self._load("global_region_proposal_valid.json"))
        raw["ranked_region_proposals"][0]["map_image_center"]["u"] = 0.90
        result = validate_global_region_proposal_response(
            json.dumps(raw),
            expected_snapshot_id="RS_TEST_GLOBAL_0001",
            expected_visual_context_id="VC_TEST_0001",
            valid_view_ids={"VIEW_090"},
            cfg=_cfg(),
        )
        self.assertFalse(result.valid)

    def test_duplicate_rank_rejected(self) -> None:
        raw = json.loads(self._load("global_region_proposal_valid.json"))
        raw["ranked_region_proposals"].append(dict(raw["ranked_region_proposals"][0]))
        raw["ranked_region_proposals"][1]["proposal_id"] = "GP_2"
        result = validate_global_region_proposal_response(
            json.dumps(raw),
            expected_snapshot_id="RS_TEST_GLOBAL_0001",
            expected_visual_context_id="VC_TEST_0001",
            valid_view_ids={"VIEW_090"},
            cfg=_cfg(),
        )
        self.assertFalse(result.valid)

    def test_invalid_view_id_rejected(self) -> None:
        result = validate_global_region_proposal_response(
            self._load("global_region_proposal_valid.json"),
            expected_snapshot_id="RS_TEST_GLOBAL_0001",
            expected_visual_context_id="VC_TEST_0001",
            valid_view_ids=set(),
            cfg=_cfg(),
        )
        self.assertFalse(result.valid)

    def test_forbidden_map_coordinate_rejected(self) -> None:
        raw = json.loads(self._load("global_region_proposal_valid.json"))
        raw["map_x"] = 1.0
        result = validate_global_region_proposal_response(
            json.dumps(raw),
            expected_snapshot_id="RS_TEST_GLOBAL_0001",
            expected_visual_context_id="VC_TEST_0001",
            valid_view_ids={"VIEW_090"},
            cfg=_cfg(),
        )
        self.assertFalse(result.valid)

    def test_empty_proposal_response_allowed(self) -> None:
        raw = json.loads(self._load("global_region_proposal_valid.json"))
        raw["ranked_region_proposals"] = []
        result = validate_global_region_proposal_response(
            json.dumps(raw),
            expected_snapshot_id="RS_TEST_GLOBAL_0001",
            expected_visual_context_id="VC_TEST_0001",
            valid_view_ids=set(),
            cfg=_cfg(),
        )
        self.assertTrue(result.valid)


class TestDispatcher(unittest.TestCase):
    def test_dispatcher_uses_candidate_path(self) -> None:
        if not SNAPSHOT_PATH.is_file():
            self.skipTest("region snapshot fixture missing")
        inp, raw = load_region_snapshot(SNAPSHOT_PATH, "explore")
        if not inp.regions:
            self.skipTest("snapshot has no regions")
        candidate_response = json.dumps(
            {
                "snapshot_id": inp.snapshot_id,
                "selected_region": inp.regions[0].label,
                "fallback_regions": [r.label for r in inp.regions[1:]],
                "confidence": 0.8,
                "reason_code": "UNEXPLORED_REGION_PRIORITY",
                "evidence": [],
                "ranked_regions": [r.label for r in inp.regions],
            }
        )
        result = select_exploration_region(
            STRATEGY_CANDIDATE_RANKING,
            inp=inp,
            raw_snapshot=raw,
            raw_response=candidate_response,
            cfg=_cfg(STRATEGY_CANDIDATE_RANKING),
        )
        self.assertEqual(STRATEGY_CANDIDATE_RANKING, result["effective_strategy"])
        self.assertFalse(result["fallback_used"])

    def test_build_prompt_for_strategy_candidate(self) -> None:
        if not SNAPSHOT_PATH.is_file():
            self.skipTest("region snapshot fixture missing")
        inp, _ = load_region_snapshot(SNAPSHOT_PATH, "explore")
        prompt = build_prompt_for_strategy(STRATEGY_CANDIDATE_RANKING, inp, _cfg())
        self.assertIn("re-rank", prompt.lower())


if __name__ == "__main__":
    unittest.main()
