#!/usr/bin/env python3
"""Tests for region_visual_context_core — created but not executed."""

from __future__ import annotations

import inspect
import json
import math
import os
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.region_visual_context_core import (  # noqa: E402
    DirectionalFrame,
    DirectionalFrameCandidate,
    VIEW_IDS,
    accumulate_rotation_deg,
    associate_regions_with_views,
    manifest_to_dict,
    normalize_angle_deg,
    relative_angle_from_initial,
    select_directional_frames,
    shortest_angular_distance_deg,
    unwrap_yaw_sequence,
    validate_visual_context_config,
)

FIXTURES = Path(__file__).parent / "fixtures"

CFG = {
    "visual_context": {
        "target_relative_angles_deg": [0, 45, 90, 135, 180, 225, 270, 315],
        "max_angle_error_deg": 12.0,
        "max_tf_age_s": 0.30,
        "complete_rotation_threshold_deg": 350.0,
        "allow_same_frame_for_multiple_views": False,
    },
    "region_view_mapping": {
        "max_primary_angle_difference_deg": 35.0,
        "include_adjacent_views": True,
        "max_secondary_angle_difference_deg": 70.0,
    },
    "safety": {"allow_motion": False, "allow_cmd_vel": False, "allow_nav2": False},
}


def _candidate(rel_yaw: float, ref: str, *, tf_age: float = 0.05) -> DirectionalFrameCandidate:
    return DirectionalFrameCandidate(
        image_stamp_sec=1.0,
        tf_stamp_sec=1.0,
        tf_age_s=tf_age,
        absolute_yaw_deg=rel_yaw,
        relative_yaw_deg=rel_yaw,
        image_reference=ref,
        width=640,
        height=480,
        encoding="bgr8",
        valid=True,
    )


class TestAngles(unittest.TestCase):
    def test_normalize_angle(self) -> None:
        self.assertAlmostEqual(normalize_angle_deg(-10), 350.0)
        self.assertAlmostEqual(normalize_angle_deg(370), 10.0)

    def test_shortest_angular_distance(self) -> None:
        self.assertAlmostEqual(shortest_angular_distance_deg(179, -179), 2.0, places=1)

    def test_yaw_wraparound_clockwise(self) -> None:
        yaws = [0, 90, 180, -179, -90, 0]
        self.assertGreater(accumulate_rotation_deg(yaws), 350.0)

    def test_yaw_wraparound_counterclockwise(self) -> None:
        yaws = [0, -90, -180, 179, 90, 0]
        self.assertGreater(accumulate_rotation_deg(yaws), 350.0)

    def test_full_rotation_accumulation(self) -> None:
        seq = unwrap_yaw_sequence([0, 90, 180, -179, -90, 0])
        self.assertGreater(len(seq), 1)
        self.assertGreater(accumulate_rotation_deg([0, 90, 180, -179, -90, 0]), 350.0)

    def test_incomplete_rotation(self) -> None:
        self.assertLess(accumulate_rotation_deg([0, 45, 90]), 350.0)


class TestFrameSelection(unittest.TestCase):
    def test_select_nearest_directional_frame(self) -> None:
        cands = [_candidate(1.0, "f0"), _candidate(46.0, "f1"), _candidate(91.0, "f2")]
        frames, complete, _ = select_directional_frames(cands, CFG, initial_yaw_deg=0.0)
        self.assertEqual(len(frames), 8)
        self.assertTrue(frames[0].valid)
        self.assertAlmostEqual(frames[0].angle_error_deg, 1.0, places=1)

    def test_reject_stale_tf_frame(self) -> None:
        cands = [_candidate(0.0, "f0", tf_age=1.0)]
        frames, _, _ = select_directional_frames(cands, CFG)
        self.assertFalse(frames[0].valid)

    def test_reject_large_angle_error(self) -> None:
        cands = [_candidate(30.0, "f0")]
        frames, _, _ = select_directional_frames(cands, CFG)
        self.assertFalse(frames[0].valid)

    def test_prevent_duplicate_primary_frame(self) -> None:
        cands = [_candidate(1.0, "same"), _candidate(2.0, "same")]
        frames, _, _ = select_directional_frames(cands, CFG)
        valid_refs = [f.image_file for f in frames if f.valid]
        self.assertEqual(len(set(valid_refs)), len(valid_refs))

    def test_missing_view_marks_capture_incomplete(self) -> None:
        cands = [_candidate(0.0, "only0")]
        _, complete, errors = select_directional_frames(cands, CFG)
        self.assertFalse(complete)
        self.assertTrue(any("MISSING_VIEW" in e for e in errors))


class TestRegionMapping(unittest.TestCase):
    def test_region_maps_to_nearest_view(self) -> None:
        snapshot = {
            "snapshot_id": "RS1",
            "robot_pose": {"x": 0, "y": 0, "yaw_deg": 0},
            "accepted_regions": [
                {
                    "label": "A",
                    "internal_region_id": "R1",
                    "track_id": "T1",
                    "centroid": {"x": -1.0, "y": 1.0},
                }
            ],
        }
        frames = [
            DirectionalFrame(
                view_id=vid,
                target_relative_angle_deg=float(vid.split("_")[1]),
                captured_relative_angle_deg=float(vid.split("_")[1]),
                absolute_yaw_deg=float(vid.split("_")[1]),
                angle_error_deg=1.0,
                image_stamp_sec=1.0,
                tf_stamp_sec=1.0,
                tf_age_s=0.01,
                image_file=f"{vid}.jpg",
                width=640,
                height=480,
                valid=True,
            )
            for vid in VIEW_IDS
        ]
        assocs, errors = associate_regions_with_views(snapshot, frames, 0.0, CFG)
        self.assertEqual(len(assocs), 1)
        self.assertTrue(assocs[0].mapping_valid)
        self.assertEqual(assocs[0].primary_view_id, "VIEW_135")

    def test_region_mapping_rejects_large_angle(self) -> None:
        tight = dict(CFG)
        tight["region_view_mapping"] = {
            "max_primary_angle_difference_deg": 5.0,
            "include_adjacent_views": False,
            "max_secondary_angle_difference_deg": 10.0,
        }
        snapshot = {
            "snapshot_id": "RS1",
            "robot_pose": {"x": 0, "y": 0},
            "accepted_regions": [{"label": "A", "internal_region_id": "R1", "centroid": {"x": 0, "y": 5}}],
        }
        frames = [
            DirectionalFrame(
                view_id="VIEW_000",
                target_relative_angle_deg=0,
                captured_relative_angle_deg=0,
                absolute_yaw_deg=0,
                angle_error_deg=0,
                image_stamp_sec=1,
                tf_stamp_sec=1,
                tf_age_s=0.01,
                image_file="a.jpg",
                width=640,
                height=480,
                valid=True,
            )
        ]
        assocs, errors = associate_regions_with_views(snapshot, frames, 0.0, tight)
        self.assertFalse(assocs[0].mapping_valid)
        self.assertTrue(errors)

    def test_secondary_views_are_adjacent(self) -> None:
        mapping = json.loads((FIXTURES / "region_view_mapping.json").read_text(encoding="utf-8"))
        for item in mapping["associations"]:
            primary = item["primary_view_id"]
            idx = VIEW_IDS.index(primary)
            neighbors = {VIEW_IDS[(idx - 1) % 8], VIEW_IDS[(idx + 1) % 8]}
            for sid in item["secondary_view_ids"]:
                self.assertIn(sid, neighbors)


class TestSerialization(unittest.TestCase):
    def test_visual_manifest_serializable(self) -> None:
        raw = json.loads((FIXTURES / "visual_context_manifest.json").read_text(encoding="utf-8"))
        self.assertIn("visual_context_id", raw)
        self.assertEqual(len(raw["frames"]), 8)

    def test_no_motion_interfaces(self) -> None:
        core_src = inspect.getsource(__import__("src.vlm.region_visual_context_core", fromlist=["x"]))
        node_path = PROJECT_ROOT + "/src/vlm/region_visual_context_node.py"
        with open(node_path, encoding="utf-8") as fh:
            node_src = fh.read()
        self.assertNotIn("import rclpy", core_src)
        self.assertNotIn("/cmd_vel", node_src)
        self.assertNotIn("NavigateToPose", node_src)

    def test_config_validation_rejects_motion(self) -> None:
        bad = {"safety": {"allow_motion": True}, "visual_context": {"target_relative_angles_deg": [0, 45]}}
        errs = validate_visual_context_config(bad)
        self.assertTrue(any("allow_motion" in e for e in errs))


if __name__ == "__main__":
    unittest.main()
