#!/usr/bin/env python3
"""Unit tests for exploration decision bundle (Phase 3A.5). Not executed in code-only phase."""

from __future__ import annotations

import ast
import ast
import copy
import json
import os
import sys
import unittest
from datetime import datetime, timezone

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.exploration_contracts import (  # noqa: E402
    EXPLORATION_CONTRACT_VERSION,
    build_map_fingerprint_from_snapshot,
    build_region_geometry_fingerprint_from_entry,
)
from src.planning.exploration_decision_bundle import (  # noqa: E402
    ExplorationDecisionBundle,
    build_exploration_decision_bundle,
    build_safe_viewpoint_request_envelope,
    generate_bundle_id,
    validate_exploration_decision_bundle,
)

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


def _contract_cfg() -> dict:
    return {
        "contracts": {"exploration_contract_version": EXPLORATION_CONTRACT_VERSION},
        "fingerprints": {
            "float_precision_digits": 6,
            "sort_region_cells": True,
            "include_map_stamp_in_map_fingerprint": True,
        },
        "bundle_validation": {
            "max_snapshot_age_s": 20.0,
            "require_map_fingerprint": True,
            "require_region_geometry_fingerprint": True,
            "require_trajectory_revision_match": True,
            "require_decision_revalidation": True,
            "require_visual_context_for_visual_mode": True,
            "require_region_stable": True,
            "require_region_snapshot_eligible": True,
            "reject_blacklisted_region": True,
        },
        "safety": {
            "allow_motion": False,
            "allow_cmd_vel": False,
            "allow_nav2": False,
            "allow_goal_publish": False,
        },
    }


def _load_json(name: str) -> dict:
    path = os.path.join(FIXTURES, name)
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _merge_fixture(base: dict, overlay: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in overlay.items():
        if key in ("inherits", "description", "map_data_mutation"):
            continue
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge_fixture(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    if overlay.get("map_data_mutation"):
        mut = overlay["map_data_mutation"]
        data = list(out["map_data"])
        data[int(mut["index"])] = int(mut["value"])
        out["map_data"] = data
    return out


def _load_fixture(name: str) -> dict:
    data = _load_json(name)
    if data.get("inherits"):
        parent = _load_fixture(str(data["inherits"]))
        overlay = {
            k: v
            for k, v in data.items()
            if k not in ("inherits", "description")
        }
        return _merge_fixture(parent, overlay)
    return data


def _hydrate_fixture(raw: dict, cfg: dict) -> dict:
    data = copy.deepcopy(raw)
    snap = data["region_snapshot"]
    geo = data["region_geometry"]
    fps = build_map_fingerprint_from_snapshot(snap, data["map_data"], cfg=cfg)
    snap.update(fps)
    geo.update(
        {
            "map_fingerprint": fps["map_fingerprint"],
            "contract_version": EXPLORATION_CONTRACT_VERSION,
            "region_geometry_schema_version": "1.0",
        }
    )
    for label, entry in geo.get("regions", {}).items():
        entry["region_geometry_fingerprint"] = build_region_geometry_fingerprint_from_entry(
            snap["snapshot_id"], label, entry, cfg=cfg
        )
    qd = data.get("qwen_decision", {})
    qd.update(
        {
            "contract_version": EXPLORATION_CONTRACT_VERSION,
            "qwen_decision_schema_version": "1.0",
            "map_fingerprint": fps["map_fingerprint"],
            "algorithm_final_region": data["fusion"]["algorithm_final_region"],
            "qwen_recommended_region": data["fusion"]["qwen_recommended_region"],
            "decision_source": data["fusion"]["decision_source"],
        }
    )
    current_time = datetime.fromisoformat(data["current_time"].replace("Z", "+00:00"))
    bundle = build_exploration_decision_bundle(
        region_snapshot=snap,
        region_geometry=geo,
        qwen_decision=qd,
        fusion=data["fusion"],
        selection_mode=data.get("selection_mode", "MAP_ONLY"),
        decision_revalidation_passed=data.get("decision_revalidation_passed", True),
        visual_manifest=data.get("visual_manifest"),
        cfg=cfg,
        current_time=current_time,
    )
    if data.get("bundle_override"):
        for k, v in data["bundle_override"].items():
            setattr(bundle, k, v)
    data["bundle"] = bundle
    data["current_time_dt"] = current_time
    return data


class TestBundleValidation(unittest.TestCase):
    def _validate(self, fixture_name: str) -> list:
        cfg = _contract_cfg()
        raw = _load_fixture(fixture_name)
        hydrated = _hydrate_fixture(raw, cfg)
        result = validate_exploration_decision_bundle(
            hydrated["bundle"],
            region_snapshot=hydrated["region_snapshot"],
            region_geometry=hydrated["region_geometry"],
            qwen_decision=hydrated["qwen_decision"],
            fusion=hydrated["fusion"],
            visual_manifest=hydrated.get("visual_manifest"),
            region_view_mapping=hydrated.get("region_view_mapping"),
            cfg=cfg,
            current_time=hydrated["current_time_dt"],
            map_data=hydrated["map_data"],
        )
        return result.error_codes

    def test_valid_bundle_passes(self) -> None:
        errors = self._validate("exploration_bundle_valid.json")
        self.assertEqual(errors, [])

    def test_snapshot_id_mismatch_fails(self) -> None:
        errors = self._validate("exploration_bundle_snapshot_mismatch.json")
        self.assertIn("BUNDLE_SNAPSHOT_ID_MISMATCH", errors)

    def test_region_label_mismatch_fails(self) -> None:
        errors = self._validate("exploration_bundle_region_mismatch.json")
        self.assertIn("BUNDLE_REGION_LABEL_MISMATCH", errors)

    def test_internal_region_id_mismatch_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["bundle"].internal_region_id = "WRONG_ID"
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_INTERNAL_REGION_ID_MISMATCH", result.error_codes)

    def test_track_id_mismatch_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["bundle"].track_id = "T_WRONG"
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_TRACK_ID_MISMATCH", result.error_codes)

    def test_map_fingerprint_mismatch_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["map_data"] = list(raw["map_data"])
        raw["map_data"][11] = 100
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_MAP_DATA_MISMATCH", result.error_codes)

    def test_region_geometry_fingerprint_mismatch_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["bundle"].region_geometry_fingerprint = "deadbeef"
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_REGION_GEOMETRY_FINGERPRINT_MISMATCH", result.error_codes)

    def test_trajectory_revision_mismatch_fails(self) -> None:
        errors = self._validate("exploration_bundle_trajectory_mismatch.json")
        self.assertIn("BUNDLE_TRAJECTORY_REVISION_MISMATCH", errors)

    def test_visual_context_id_mismatch_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _load_fixture("exploration_bundle_visual_mismatch.json")
        hydrated = _hydrate_fixture(raw, cfg)
        hydrated["bundle"].selection_mode = "MAP_PLUS_VISUAL"
        hydrated["bundle"].visual_context_id = "VC_EXPECTED"
        result = validate_exploration_decision_bundle(
            hydrated["bundle"],
            region_snapshot=hydrated["region_snapshot"],
            region_geometry=hydrated["region_geometry"],
            qwen_decision=hydrated["qwen_decision"],
            fusion=hydrated["fusion"],
            visual_manifest=hydrated["visual_manifest"],
            region_view_mapping=hydrated["region_view_mapping"],
            cfg=cfg,
            current_time=hydrated["current_time_dt"],
            map_data=hydrated["map_data"],
        )
        self.assertIn("BUNDLE_VISUAL_CONTEXT_ID_MISMATCH", result.error_codes)

    def test_incomplete_visual_context_fails_in_visual_mode(self) -> None:
        errors = self._validate("exploration_bundle_visual_mismatch.json")
        self.assertTrue(
            "BUNDLE_VISUAL_CONTEXT_INCOMPLETE" in errors
            or "BUNDLE_VISUAL_CONTEXT_REQUIRED" in errors
            or "BUNDLE_REGION_VIEW_MAPPING_MISSING" in errors
        )

    def test_map_only_allows_missing_visual_context(self) -> None:
        errors = self._validate("exploration_bundle_valid.json")
        self.assertNotIn("BUNDLE_VISUAL_CONTEXT_REQUIRED", errors)

    def test_unstable_region_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["bundle"].region_stable = False
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_REGION_UNSTABLE", result.error_codes)

    def test_ineligible_region_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["bundle"].region_snapshot_eligible = False
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_REGION_NOT_ELIGIBLE", result.error_codes)

    def test_blacklisted_region_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["bundle"].region_blacklisted = True
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_REGION_BLACKLISTED", result.error_codes)

    def test_failed_decision_revalidation_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["bundle"].decision_revalidation_passed = False
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_DECISION_REVALIDATION_FAILED", result.error_codes)

    def test_stale_snapshot_fails(self) -> None:
        errors = self._validate("exploration_bundle_stale.json")
        self.assertIn("BUNDLE_SNAPSHOT_STALE", errors)

    def test_premature_reachability_claim_fails(self) -> None:
        cfg = _contract_cfg()
        raw = _hydrate_fixture(_load_json("exploration_bundle_valid.json"), cfg)
        raw["bundle"].reachable = True
        result = validate_exploration_decision_bundle(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=cfg,
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIn("BUNDLE_PREMATURE_REACHABILITY_CLAIM", result.error_codes)


class TestSafeViewpointRequest(unittest.TestCase):
    def _hydrated(self) -> dict:
        return _hydrate_fixture(_load_json("exploration_bundle_valid.json"), _contract_cfg())

    def test_valid_bundle_builds_safe_viewpoint_request(self) -> None:
        raw = self._hydrated()
        envelope, validation = build_safe_viewpoint_request_envelope(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=_contract_cfg(),
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertTrue(validation.valid)
        self.assertTrue(envelope.request_ready)

    def test_invalid_bundle_does_not_build_request(self) -> None:
        raw = _hydrate_fixture(
            _load_fixture("exploration_bundle_stale.json"),
            _contract_cfg(),
        )
        envelope, validation = build_safe_viewpoint_request_envelope(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=_contract_cfg(),
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertFalse(validation.valid)
        self.assertFalse(envelope.request_ready)

    def test_request_uses_algorithm_final_region_not_qwen_recommendation(self) -> None:
        raw = self._hydrated()
        envelope, _ = build_safe_viewpoint_request_envelope(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=_contract_cfg(),
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertEqual(envelope.region_label, "B")
        self.assertEqual(raw["fusion"]["qwen_recommended_region"], "A")

    def test_request_preserves_map_fingerprint(self) -> None:
        raw = self._hydrated()
        envelope, _ = build_safe_viewpoint_request_envelope(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=_contract_cfg(),
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertEqual(envelope.map_fingerprint, raw["region_snapshot"]["map_fingerprint"])

    def test_request_preserves_region_geometry_fingerprint(self) -> None:
        raw = self._hydrated()
        envelope, _ = build_safe_viewpoint_request_envelope(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=_contract_cfg(),
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertEqual(
            envelope.region_geometry_fingerprint,
            raw["region_geometry"]["regions"]["B"]["region_geometry_fingerprint"],
        )

    def test_path_checked_remains_false(self) -> None:
        raw = self._hydrated()
        envelope, _ = build_safe_viewpoint_request_envelope(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=_contract_cfg(),
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertFalse(envelope.path_checked)

    def test_reachable_remains_none(self) -> None:
        raw = self._hydrated()
        envelope, _ = build_safe_viewpoint_request_envelope(
            raw["bundle"],
            region_snapshot=raw["region_snapshot"],
            region_geometry=raw["region_geometry"],
            qwen_decision=raw["qwen_decision"],
            fusion=raw["fusion"],
            cfg=_contract_cfg(),
            current_time=raw["current_time_dt"],
            map_data=raw["map_data"],
        )
        self.assertIsNone(envelope.reachable)


class TestBundleId(unittest.TestCase):
    def test_bundle_id_format(self) -> None:
        created = datetime(2026, 7, 13, 19, 0, 0, tzinfo=timezone.utc)
        bid = generate_bundle_id("RS_20260713T190000_0092", "B", created)
        self.assertEqual(bid, "EDB_20260713T190000_0092_B")


class TestModuleSafety(unittest.TestCase):
    def test_no_ros_dependencies(self) -> None:
        path = os.path.join(PROJECT_ROOT, "src", "planning", "exploration_decision_bundle.py")
        with open(path, encoding="utf-8") as fh:
            source = fh.read()
        self.assertNotIn("import rclpy", source)

    def test_no_nav2_interfaces(self) -> None:
        for name in ("exploration_decision_bundle.py", "exploration_contracts.py"):
            path = os.path.join(PROJECT_ROOT, "src", "planning", name)
            with open(path, encoding="utf-8") as fh:
                tree = ast.parse(fh.read())
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and node.module and "nav2" in node.module:
                    self.fail(f"nav2 import found in {name}")

    def test_no_motion_interfaces(self) -> None:
        path = os.path.join(PROJECT_ROOT, "src", "planning", "exploration_decision_bundle.py")
        with open(path, encoding="utf-8") as fh:
            source = fh.read().lower()
        self.assertNotIn("cmd_vel", source)


if __name__ == "__main__":
    unittest.main()
