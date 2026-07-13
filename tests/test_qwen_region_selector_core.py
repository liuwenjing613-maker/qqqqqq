#!/usr/bin/env python3
"""Unit tests for qwen_region_selector_core."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.vlm.qwen_region_selector_core import (  # noqa: E402
    FORBIDDEN_DECISION_FIELDS,
    RegionCandidate,
    RegionSelectionDecision,
    RegionSelectionInput,
    build_region_selection_prompt,
    decision_from_validation,
    decision_to_dict,
    extract_json_object,
    load_region_snapshot,
    validate_qwen_decision,
    validate_region_snapshot,
)

SNAPSHOT_PATH = (
    PROJECT_ROOT
    / "logs/qwen_region_explore/20260713_174148/snapshots/RS_20260713T094328_0092/region_snapshot.json"
)
FIXTURES = Path(__file__).parent / "fixtures"


class TestSnapshotLoading(unittest.TestCase):
    def test_load_valid_snapshot(self) -> None:
        self.assertTrue(SNAPSHOT_PATH.is_file(), f"missing {SNAPSHOT_PATH}")
        inp, raw = load_region_snapshot(SNAPSHOT_PATH, "寻找绿色瓶子")
        self.assertEqual(inp.snapshot_id, "RS_20260713T094328_0092")
        self.assertEqual(len(inp.regions), 2)
        self.assertEqual(inp.regions[0].label, "A")
        self.assertFalse(validate_region_snapshot(inp))
        self.assertIn("snapshot_id", raw)

    def test_reject_missing_snapshot_file(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            load_region_snapshot("/tmp/no_such_snapshot.json")
        self.assertIn("SNAPSHOT_FILE_NOT_FOUND", str(ctx.exception))

    def test_reject_duplicate_region_labels(self) -> None:
        inp = RegionSelectionInput(
            snapshot_id="RS_test",
            cycle_id=1,
            map_stamp=0.0,
            capture_time="t",
            target_instruction="x",
            robot_pose={},
            regions=[
                RegionCandidate("A", "R1", "LEFT", 1.0, 10, 0.1, 0.4, 0.5, 5),
                RegionCandidate("A", "R2", "BACK", 1.0, 10, 0.1, 0.4, 0.5, 5),
            ],
            annotated_map_file=str(SNAPSHOT_PATH.parent / "annotated_map.png"),
        )
        errors = validate_region_snapshot(inp)
        self.assertIn("SNAPSHOT_DUPLICATE_LABEL", errors)


class TestPrompt(unittest.TestCase):
    def test_prompt_contains_only_available_labels(self) -> None:
        inp, _ = load_region_snapshot(SNAPSHOT_PATH, "寻找绿色瓶子")
        prompt = build_region_selection_prompt(inp)
        self.assertIn("Valid region labels ONLY: A, B", prompt)
        self.assertNotIn("Label C", prompt)
        self.assertIn("MUST NOT output map coordinates", prompt)


class TestDecisionValidation(unittest.TestCase):
    def setUp(self) -> None:
        self.snapshot_id = "RS_20260713T094328_0092"
        self.labels = {"A", "B"}

    def test_valid_decision_passes(self) -> None:
        raw = (FIXTURES / "qwen_region_valid.json").read_text(encoding="utf-8")
        result = validate_qwen_decision(raw, self.snapshot_id, self.labels)
        self.assertTrue(result.decision_valid)
        self.assertEqual(result.errors, [])

    def test_invalid_selected_region_fails(self) -> None:
        raw = (FIXTURES / "qwen_region_invalid_label.json").read_text(encoding="utf-8")
        result = validate_qwen_decision(raw, self.snapshot_id, self.labels)
        self.assertFalse(result.decision_valid)
        self.assertIn("QWEN_SELECTED_REGION_INVALID", result.errors)

    def test_snapshot_id_mismatch_fails(self) -> None:
        raw = (FIXTURES / "qwen_region_snapshot_mismatch.json").read_text(encoding="utf-8")
        result = validate_qwen_decision(raw, self.snapshot_id, self.labels)
        self.assertFalse(result.decision_valid)
        self.assertIn("QWEN_SNAPSHOT_ID_MISMATCH", result.errors)

    def test_duplicate_fallback_fails(self) -> None:
        raw = json.dumps(
            {
                "snapshot_id": self.snapshot_id,
                "selected_region": "A",
                "fallback_regions": ["B", "B"],
                "confidence": 0.5,
                "reason_code": "X",
                "evidence": [],
            }
        )
        result = validate_qwen_decision(raw, self.snapshot_id, self.labels)
        self.assertFalse(result.decision_valid)
        self.assertIn("QWEN_FALLBACK_DUPLICATE", result.errors)

    def test_selected_in_fallback_fails(self) -> None:
        raw = json.dumps(
            {
                "snapshot_id": self.snapshot_id,
                "selected_region": "A",
                "fallback_regions": ["A"],
                "confidence": 0.5,
                "reason_code": "X",
                "evidence": [],
            }
        )
        result = validate_qwen_decision(raw, self.snapshot_id, self.labels)
        self.assertFalse(result.decision_valid)

    def test_invalid_confidence_fails(self) -> None:
        raw = json.dumps(
            {
                "snapshot_id": self.snapshot_id,
                "selected_region": "A",
                "fallback_regions": [],
                "confidence": 1.5,
                "reason_code": "X",
                "evidence": [],
            }
        )
        result = validate_qwen_decision(raw, self.snapshot_id, self.labels)
        self.assertFalse(result.decision_valid)
        self.assertIn("QWEN_CONFIDENCE_INVALID", result.errors)

    def test_forbidden_coordinates_fail(self) -> None:
        raw = (FIXTURES / "qwen_region_forbidden_coordinates.json").read_text(encoding="utf-8")
        result = validate_qwen_decision(raw, self.snapshot_id, self.labels)
        self.assertFalse(result.decision_valid)
        self.assertIn("QWEN_FORBIDDEN_FIELD_PRESENT", result.errors)

    def test_non_json_response_fails(self) -> None:
        raw = (FIXTURES / "qwen_region_non_json.txt").read_text(encoding="utf-8")
        result = validate_qwen_decision(raw, self.snapshot_id, self.labels)
        self.assertFalse(result.decision_valid)
        self.assertTrue(
            "QWEN_RESPONSE_NOT_JSON" in result.errors or "QWEN_RESPONSE_EMPTY" in result.errors
        )

    def test_empty_response_fails(self) -> None:
        result = validate_qwen_decision("", self.snapshot_id, self.labels)
        self.assertFalse(result.decision_valid)
        self.assertIn("QWEN_RESPONSE_EMPTY", result.errors)


class TestSerialization(unittest.TestCase):
    def test_decision_serialization(self) -> None:
        d = RegionSelectionDecision(
            snapshot_id="RS_x",
            selected_region="B",
            fallback_regions=["A"],
            confidence=0.8,
            reason_code="UNEXPLORED_REGION_PRIORITY",
            evidence=["e1"],
        )
        out = decision_to_dict(d, decision_valid=True)
        text = json.dumps(out)
        parsed = json.loads(text)
        self.assertEqual(parsed["selected_region"], "B")
        self.assertTrue(parsed["decision_valid"])

    def test_no_motion_fields_in_schema(self) -> None:
        for field in FORBIDDEN_DECISION_FIELDS:
            self.assertNotIn(field, RegionSelectionDecision.__dataclass_fields__)


class TestExtractJson(unittest.TestCase):
    def test_markdown_json(self) -> None:
        text = '```json\n{"snapshot_id": "x", "selected_region": "A"}\n```'
        obj = extract_json_object(text)
        self.assertEqual(obj["snapshot_id"], "x")


if __name__ == "__main__":
    unittest.main()
