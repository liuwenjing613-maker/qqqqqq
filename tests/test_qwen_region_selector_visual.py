#!/usr/bin/env python3
"""Tests for Qwen visual region selection — created but not executed."""

from __future__ import annotations

import json
import os
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_region_selector_core import (  # noqa: E402
    FORBIDDEN_DECISION_FIELDS,
    RegionCandidate,
    RegionSelectionInput,
    SELECTION_MODE_MAP_ONLY,
    SELECTION_MODE_MAP_PLUS_VISUAL,
    attach_visual_context_to_input,
    audit_qwen_visual_evidence,
    build_region_selection_prompt,
    fuse_geometric_and_qwen_ranking,
    resolve_selection_mode,
    validate_qwen_decision,
)

FIXTURES = Path(__file__).parent / "fixtures"

CFG = {
    "decision_fusion": {
        "map_only_geo_weight": 0.85,
        "map_only_qwen_weight": 0.15,
        "visual_geo_weight": 0.65,
        "visual_qwen_weight": 0.35,
        "reject_if_geo_score_below": 0.40,
        "max_allowed_geo_gap_without_visual_evidence": 0.20,
    }
}


def _candidate(label: str, geo: float, *, stable: bool = True) -> RegionCandidate:
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
        geo_rank=1,
        stable=stable,
        snapshot_eligible=True,
    )


def _base_input() -> RegionSelectionInput:
    return RegionSelectionInput(
        snapshot_id="RS_TEST001",
        cycle_id=1,
        map_stamp=0.0,
        capture_time="t",
        target_instruction="explore",
        robot_pose={"x": 0, "y": 0, "yaw_deg": 0},
        regions=[_candidate("A", 0.8), _candidate("B", 0.75)],
        annotated_map_file=str(FIXTURES / "annotated_map_placeholder.png"),
    )


class TestSelectionMode(unittest.TestCase):
    def test_map_only_mode_remains_supported(self) -> None:
        inp = _base_input()
        self.assertEqual(resolve_selection_mode(inp), SELECTION_MODE_MAP_ONLY)
        prompt = build_region_selection_prompt(inp)
        self.assertIn("MAP_ONLY", prompt)

    def test_visual_context_id_required_in_visual_mode(self) -> None:
        manifest = json.loads((FIXTURES / "visual_context_manifest.json").read_text(encoding="utf-8"))
        mapping = json.loads((FIXTURES / "region_view_mapping.json").read_text(encoding="utf-8"))
        inp = attach_visual_context_to_input(
            _base_input(),
            manifest,
            mapping,
            contact_sheet_file="/tmp/contact.jpg",
        )
        self.assertEqual(resolve_selection_mode(inp), SELECTION_MODE_MAP_PLUS_VISUAL)


class TestVisualValidation(unittest.TestCase):
    def setUp(self) -> None:
        self.labels = {"A", "B"}
        self.manifest = json.loads((FIXTURES / "visual_context_manifest.json").read_text(encoding="utf-8"))
        self.mapping = json.loads((FIXTURES / "region_view_mapping.json").read_text(encoding="utf-8"))

    def test_visual_context_id_mismatch_rejected(self) -> None:
        raw = json.dumps(
            {
                "snapshot_id": "RS_TEST001",
                "visual_context_id": "VC_WRONG",
                "ranked_regions": ["A", "B"],
                "selected_region": "A",
                "fallback_regions": ["B"],
                "confidence": 0.8,
                "reason_code": "X",
                "evidence": [],
            }
        )
        result = validate_qwen_decision(
            raw,
            "RS_TEST001",
            self.labels,
            expected_visual_context_id="VC_TEST001",
            visual_context_manifest=self.manifest,
            region_view_mapping=self.mapping,
            visual_mode=True,
        )
        self.assertIn("QWEN_VISUAL_CONTEXT_ID_MISMATCH", result.errors)

    def test_valid_mapped_view_evidence_passes(self) -> None:
        raw = json.dumps(
            {
                "snapshot_id": "RS_TEST001",
                "visual_context_id": "VC_TEST001",
                "ranked_regions": ["B", "A"],
                "selected_region": "B",
                "fallback_regions": ["A"],
                "confidence": 0.84,
                "reason_code": "GEOMETRY_AND_VISUAL_CONTEXT_AGREE",
                "evidence": [
                    {
                        "region": "B",
                        "map_factors": ["geo_score较高"],
                        "visual_factors": [
                            {"view_ids": ["VIEW_000"], "observation": "较开放延伸空间"}
                        ],
                        "risks": ["path_checked=false"],
                    }
                ],
            }
        )
        result = validate_qwen_decision(
            raw,
            "RS_TEST001",
            self.labels,
            expected_visual_context_id="VC_TEST001",
            visual_context_manifest=self.manifest,
            region_view_mapping=self.mapping,
            visual_mode=True,
        )
        self.assertNotIn("QWEN_EVIDENCE_VIEW_NOT_MAPPED", result.errors)

    def test_invalid_view_id_rejected(self) -> None:
        errs = audit_qwen_visual_evidence(
            [{"region": "A", "visual_factors": [{"view_ids": ["VIEW_999"], "observation": "x"}]}],
            self.labels,
            expected_visual_context_id="VC_TEST001",
            region_view_mapping=self.mapping,
            visual_context_manifest=self.manifest,
            visual_mode=True,
        )
        self.assertIn("QWEN_EVIDENCE_VIEW_MISSING", errs)

    def test_missing_view_reference_rejected(self) -> None:
        errs = audit_qwen_visual_evidence(
            [{"region": "A", "visual_factors": [{"view_ids": [], "observation": "x"}]}],
            self.labels,
            expected_visual_context_id="VC_TEST001",
            region_view_mapping=self.mapping,
            visual_context_manifest=self.manifest,
            visual_mode=True,
        )
        self.assertIn("QWEN_EVIDENCE_VIEW_MISSING", errs)

    def test_view_from_wrong_region_rejected(self) -> None:
        errs = audit_qwen_visual_evidence(
            [{"region": "B", "visual_factors": [{"view_ids": ["VIEW_135"], "observation": "x"}]}],
            self.labels,
            expected_visual_context_id="VC_TEST001",
            region_view_mapping=self.mapping,
            visual_context_manifest=self.manifest,
            visual_mode=True,
        )
        self.assertIn("QWEN_EVIDENCE_VIEW_NOT_MAPPED", errs)

    def test_visual_output_forbidden_coordinates_rejected(self) -> None:
        raw = json.dumps(
            {
                "snapshot_id": "RS_TEST001",
                "visual_context_id": "VC_TEST001",
                "ranked_regions": ["A", "B"],
                "selected_region": "A",
                "fallback_regions": ["B"],
                "confidence": 0.8,
                "reason_code": "X",
                "evidence": [],
                "x": 1.0,
            }
        )
        result = validate_qwen_decision(
            raw,
            "RS_TEST001",
            self.labels,
            expected_visual_context_id="VC_TEST001",
            visual_mode=True,
        )
        self.assertIn("QWEN_FORBIDDEN_FIELD_PRESENT", result.errors)


class TestFusionAndPrompt(unittest.TestCase):
    def test_low_geo_region_cannot_be_restored_by_visual_rank(self) -> None:
        regions = [_candidate("A", 0.9), _candidate("B", 0.35, stable=True)]
        parsed = {"ranked_regions": ["B", "A"], "selected_region": "B"}
        out = fuse_geometric_and_qwen_ranking(regions, parsed, CFG, has_visual_evidence=True)
        self.assertEqual(out["algorithm_final_region"], "A")

    def test_visual_fusion_weights(self) -> None:
        regions = [_candidate("A", 0.7), _candidate("B", 0.69)]
        parsed = {"ranked_regions": ["B", "A"], "selected_region": "B"}
        out = fuse_geometric_and_qwen_ranking(regions, parsed, CFG, has_visual_evidence=True)
        self.assertAlmostEqual(out["geo_weight"], 0.65)
        self.assertAlmostEqual(out["qwen_weight"], 0.35)

    def test_visual_prompt_contains_mapping_rules(self) -> None:
        manifest = json.loads((FIXTURES / "visual_context_manifest.json").read_text(encoding="utf-8"))
        mapping = json.loads((FIXTURES / "region_view_mapping.json").read_text(encoding="utf-8"))
        inp = attach_visual_context_to_input(
            _base_input(),
            manifest,
            mapping,
            contact_sheet_file="/tmp/contact.jpg",
        )
        prompt = build_region_selection_prompt(inp)
        self.assertIn("region_view_mapping", prompt)
        self.assertIn("MAP_PLUS_VISUAL", prompt)

    def test_no_motion_fields_in_visual_schema(self) -> None:
        self.assertIn("cmd_vel", FORBIDDEN_DECISION_FIELDS)


if __name__ == "__main__":
    unittest.main()
