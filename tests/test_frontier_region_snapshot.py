#!/usr/bin/env python3
"""Unit tests for region snapshot helpers."""

from __future__ import annotations

import json
import os
import sys
import unittest
from datetime import datetime, timezone

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.frontier_region_debug_core import (  # noqa: E402
    FrontierAnalysisResult,
    FrontierExtractionStats,
    FrontierRegion,
    MapHealthResult,
    MapMetadata,
    RobotPose2D,
    build_region_snapshot_payload,
    generate_snapshot_id,
)


def _meta() -> MapMetadata:
    return MapMetadata(
        width=20,
        height=20,
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        stamp_sec=123456789.123,
    )


def _region(
    region_id: str,
    bearing: float,
    distance: float,
    cx: float,
    cy: float,
) -> FrontierRegion:
    return FrontierRegion(
        region_id=region_id,
        accepted=True,
        direction_label="LEFT",
        centroid_x=cx,
        centroid_y=cy,
        nearest_x=cx,
        nearest_y=cy,
        distance_to_robot_m=distance,
        bearing_global_deg=bearing,
        bearing_relative_deg=bearing,
        frontier_cell_count=10,
        frontier_length_m=0.5,
        unknown_gain_cells=100,
        unknown_gain_ratio=0.5,
        minimum_clearance_m=0.4,
        mean_clearance_m=0.5,
        diagnostic_priority=1.0,
        path_checked=False,
        reachable=True,
        source_cluster_ids=["C001"],
        merged=False,
        merge_reasons=[],
        snapshot_eligible=True,
    )


def _result(regions: list[FrontierRegion]) -> FrontierAnalysisResult:
    stats = FrontierExtractionStats(status="OK", accepted_region_count=len(regions))
    health = MapHealthResult(ok=True, status="OK")
    return FrontierAnalysisResult(
        cycle_id=124,
        map_health=health,
        stats=stats,
        regions=regions,
        rejected_regions=[],
    )


class TestRegionSnapshot(unittest.TestCase):
    def test_snapshot_labels_are_deterministic(self) -> None:
        regions = [
            _region("R0124_02", 30.0, 1.5, 0.2, 1.0),
            _region("R0124_01", 10.0, 1.0, -0.2, 1.1),
        ]
        robot = RobotPose2D(0.0, 0.0, 0.0)
        payload = build_region_snapshot_payload(
            _result(regions),
            _meta(),
            robot,
            "RS_test",
            "2026-07-13T09:00:00Z",
        )
        labels = [r["label"] for r in payload["accepted_regions"]]
        ids = [r["internal_region_id"] for r in payload["accepted_regions"]]
        self.assertEqual(labels, ["A", "B"])
        self.assertEqual(ids[0], "R0124_01")
        self.assertEqual(ids[1], "R0124_02")

    def test_snapshot_contains_snapshot_id(self) -> None:
        fixed = datetime(2026, 7, 13, 17, 5, 0, tzinfo=timezone.utc)
        snap_id = generate_snapshot_id(124, when=fixed)
        self.assertEqual(snap_id, "RS_20260713T170500_0124")
        payload = build_region_snapshot_payload(
            _result([_region("R0124_01", 0.0, 1.0, 0.0, 1.0)]),
            _meta(),
            RobotPose2D(0.0, 0.0, 0.0),
            snap_id,
            "2026-07-13T09:00:00Z",
        )
        self.assertEqual(payload["snapshot_id"], snap_id)

    def test_snapshot_preserves_internal_region_ids(self) -> None:
        region = _region("R0124_07", 5.0, 0.9, 0.1, 0.9)
        region.source_cluster_ids = ["C012", "C016"]
        region.merged = True
        payload = build_region_snapshot_payload(
            _result([region]),
            _meta(),
            RobotPose2D(0.0, 0.0, 0.0),
            "RS_test",
            "2026-07-13T09:00:00Z",
        )
        entry = payload["accepted_regions"][0]
        self.assertEqual(entry["internal_region_id"], "R0124_07")
        self.assertEqual(entry["source_cluster_ids"], ["C012", "C016"])
        self.assertTrue(entry["merged"])

    def test_snapshot_rejects_empty_region_set(self) -> None:
        empty = _result([])
        self.assertEqual(len(empty.regions), 0)

    def test_snapshot_json_is_serializable(self) -> None:
        payload = build_region_snapshot_payload(
            _result([_region("R0124_01", 0.0, 1.0, 0.0, 1.0)]),
            _meta(),
            RobotPose2D(0.03, 0.02, 1.59),
            "RS_20260713T170500_0124",
            "2026-07-13T09:00:00Z",
            annotated_map_file="/tmp/annotated_map.png",
        )
        text = json.dumps(payload, ensure_ascii=False)
        parsed = json.loads(text)
        self.assertEqual(parsed["robot_pose"]["x"], 0.03)
        self.assertEqual(len(parsed["accepted_regions"]), 1)
        self.assertEqual(parsed["accepted_regions"][0]["label"], "A")


if __name__ == "__main__":
    unittest.main()
