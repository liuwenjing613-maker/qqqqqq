#!/usr/bin/env python3
"""Unit tests for frontier_region_debug_core."""

from __future__ import annotations

import copy
import math
import os
import sys
import unittest

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.frontier_region_debug_core import (  # noqa: E402
    MapMetadata,
    RobotPose2D,
    analyze_frontier_regions,
    build_region_snapshot_payload,
    classify_map_cells,
    connected_components_8,
    extract_raw_frontier_mask,
    filter_frontier_by_clearance,
    generate_snapshot_id,
    grid_to_world,
    merge_adjacent_clusters,
    normalize_angle_rad,
    relative_bearing_to_direction,
    validate_map,
    world_to_grid,
)


def _meta(w: int, h: int, res: float = 0.05) -> MapMetadata:
    return MapMetadata(
        width=w,
        height=h,
        resolution=res,
        origin_x=0.0,
        origin_y=0.0,
        frame_id="map",
        stamp_sec=0.0,
    )


def _cfg(**overrides) -> dict:
    base = {
        "map_values": {"unknown_value": -1, "free_max": 20, "occupied_min": 65},
        "map_health": {
            "require_robot_inside_map": True,
            "reject_robot_cell_occupied": True,
            "reject_robot_cell_unknown": False,
        },
        "frontier": {"min_clearance_m": 0.35},
        "region": {
            "min_frontier_cells": 8,
            "min_distance_m": 0.50,
            "max_distance_m": 4.00,
            "min_unknown_gain_cells": 10,
            "unknown_gain_radius_m": 1.00,
            "max_regions": 12,
        },
        "region_merge": {
            "enabled": True,
            "max_frontier_gap_m": 0.20,
            "max_centroid_distance_m": 0.45,
            "max_bearing_difference_deg": 25.0,
            "require_same_direction_label": False,
        },
    }
    for k, v in overrides.items():
        base[k] = v
    return base


class TestMapDataSizeMismatch(unittest.TestCase):
    def test_size_mismatch(self) -> None:
        data = np.zeros((4, 4), dtype=np.int16)
        meta = _meta(5, 4)
        health = validate_map(data, meta, RobotPose2D(0.1, 0.1, 0.0), _cfg())
        self.assertFalse(health.ok)
        codes = [e["code"] for e in health.errors]
        self.assertIn("MAP_DATA_SIZE_MISMATCH", codes)


class TestClosedRoomNoFrontier(unittest.TestCase):
    def test_no_frontier(self) -> None:
        h, w = 20, 20
        data = np.full((h, w), -1, dtype=np.int16)
        data[1:-1, 1:-1] = 0
        data[0, :] = 100
        data[-1, :] = 100
        data[:, 0] = 100
        data[:, -1] = 100
        robot = RobotPose2D(0.5, 0.5, 0.0)
        result = analyze_frontier_regions(data, _meta(w, h), robot, _cfg(), cycle_id=1)
        self.assertEqual(result.stats.raw_frontier_cells, 0)
        self.assertEqual(result.stats.accepted_region_count, 0)
        self.assertIn(result.stats.status, ("NO_FRONTIER", "OK"))


class TestSingleUnknownOpening(unittest.TestCase):
    def test_door_frontier(self) -> None:
        h, w = 30, 30
        data = np.full((h, w), -1, dtype=np.int16)
        data[5:25, 5:25] = 0
        data[5:25, 24] = -1  # unknown to the east (right)
        robot = RobotPose2D(0.5, 0.5, 0.0)
        cfg = _cfg(
            region={
                "min_frontier_cells": 3,
                "min_distance_m": 0.10,
                "max_distance_m": 10.0,
                "min_unknown_gain_cells": 1,
                "unknown_gain_radius_m": 1.0,
                "max_regions": 12,
            },
            frontier={"min_clearance_m": 0.0},
        )
        result = analyze_frontier_regions(data, _meta(w, h), robot, cfg, cycle_id=7)
        self.assertGreater(result.stats.raw_frontier_cells, 0)
        all_regions = result.regions + result.rejected_regions
        self.assertGreater(len(all_regions), 0)
        main = max(all_regions, key=lambda r: r.frontier_cell_count)
        self.assertGreater(main.centroid_x, robot.x)
        self.assertIn(
            main.direction_label,
            ("FRONT", "FRONT_RIGHT", "RIGHT", "FRONT_LEFT", "LEFT"),
        )


class TestSmallUnknownNoiseRejected(unittest.TestCase):
    def test_too_small(self) -> None:
        h, w = 25, 25
        data = np.full((h, w), -1, dtype=np.int16)
        data[8:17, 8:17] = 100
        data[9:16, 9:16] = 0
        for row in range(11, 16):
            data[row, 16] = 0
        robot = RobotPose2D(0.55, 0.55, 0.0)
        cfg = _cfg(
            frontier={"min_clearance_m": 0.0},
            region={
                "min_frontier_cells": 8,
                "min_distance_m": 0.10,
                "max_distance_m": 10.0,
                "min_unknown_gain_cells": 1,
                "unknown_gain_radius_m": 1.0,
                "max_regions": 12,
            },
        )
        result = analyze_frontier_regions(data, _meta(w, h), robot, cfg, cycle_id=3)
        self.assertEqual(result.stats.raw_frontier_cells, 5)
        codes = {x.code for r in result.rejected_regions for x in r.rejection_reasons}
        self.assertIn("REGION_TOO_SMALL", codes)


class TestLowClearanceRejected(unittest.TestCase):
    def test_low_clearance(self) -> None:
        h, w = 25, 25
        data = np.full((h, w), -1, dtype=np.int16)
        data[5:20, 5:20] = 0
        data[5:20, 19] = -1
        data[5:20, 4] = 100  # wall adjacent to frontier
        robot = RobotPose2D(0.4, 0.4, 0.0)
        cfg = _cfg(
            frontier={"min_clearance_m": 0.50},
            region={
                "min_frontier_cells": 3,
                "min_distance_m": 0.1,
                "max_distance_m": 10.0,
                "min_unknown_gain_cells": 1,
                "unknown_gain_radius_m": 1.0,
                "max_regions": 12,
            },
        )
        result = analyze_frontier_regions(data, _meta(w, h), robot, cfg, cycle_id=5)
        if result.rejected_regions:
            codes = {x.code for r in result.rejected_regions for x in r.rejection_reasons}
            self.assertTrue(
                "REGION_LOW_CLEARANCE" in codes or result.stats.removed_low_clearance > 0
            )


class TestEightDirections(unittest.TestCase):
    def test_direction_labels(self) -> None:
        expected = {
            0.0: "FRONT",
            math.radians(45): "FRONT_LEFT",
            math.radians(90): "LEFT",
            math.radians(135): "BACK_LEFT",
            math.radians(180): "BACK",
            math.radians(-135): "BACK_RIGHT",
            math.radians(-90): "RIGHT",
            math.radians(-45): "FRONT_RIGHT",
        }
        for angle, label in expected.items():
            self.assertEqual(relative_bearing_to_direction(angle), label)
            self.assertEqual(
                relative_bearing_to_direction(normalize_angle_rad(angle)), label
            )


class TestCoordinateRoundTrip(unittest.TestCase):
    def test_grid_world_grid(self) -> None:
        meta = _meta(40, 40, 0.05)
        for row, col in ((10, 12), (0, 0), (39, 39)):
            wx, wy = grid_to_world(row, col, meta)
            row2, col2 = world_to_grid(wx, wy, meta)
            self.assertEqual(row2, row)
            self.assertEqual(col2, col)


class TestInputImmutability(unittest.TestCase):
    def test_array_unchanged(self) -> None:
        h, w = 15, 15
        data = np.zeros((h, w), dtype=np.int16)
        data[7, 7] = -1
        data[6:9, 6:9] = 0
        data[5, 5:10] = -1
        before = data.copy()
        robot = RobotPose2D(0.4, 0.4, 0.0)
        cfg = _cfg(frontier={"min_clearance_m": 0.0})
        analyze_frontier_regions(data, _meta(w, h), robot, cfg, cycle_id=9)
        np.testing.assert_array_equal(data, before)


class TestDeterministicOutput(unittest.TestCase):
    def test_two_runs_identical(self) -> None:
        h, w = 22, 22
        data = np.full((h, w), -1, dtype=np.int16)
        data[4:18, 4:18] = 0
        data[4:18, 17] = -1
        robot = RobotPose2D(0.5, 0.5, 0.0)
        cfg = _cfg(
            frontier={"min_clearance_m": 0.0},
            region={
                "min_frontier_cells": 4,
                "min_distance_m": 0.1,
                "max_distance_m": 8.0,
                "min_unknown_gain_cells": 1,
                "unknown_gain_radius_m": 1.0,
                "max_regions": 12,
            },
        )
        r1 = analyze_frontier_regions(data, _meta(w, h), robot, cfg, cycle_id=11)
        r2 = analyze_frontier_regions(data, _meta(w, h), robot, cfg, cycle_id=11)
        self.assertEqual(len(r1.regions), len(r2.regions))
        self.assertEqual([x.region_id for x in r1.regions], [x.region_id for x in r2.regions])
        self.assertEqual(
            [x.frontier_cell_count for x in r1.regions],
            [x.frontier_cell_count for x in r2.regions],
        )


class TestRobotOutsideMap(unittest.TestCase):
    def test_robot_outside(self) -> None:
        data = np.zeros((10, 10), dtype=np.int16)
        meta = _meta(10, 10)
        robot = RobotPose2D(5.0, 5.0, 0.0)
        health = validate_map(data, meta, robot, _cfg())
        self.assertFalse(health.ok)
        codes = [e["code"] for e in health.errors]
        self.assertIn("ROBOT_OUTSIDE_MAP", codes)
        result = analyze_frontier_regions(data, meta, robot, _cfg(), cycle_id=2)
        self.assertFalse(result.map_health.ok)


class TestConnectedComponents(unittest.TestCase):
    def test_two_clusters(self) -> None:
        mask = np.zeros((10, 10), dtype=bool)
        mask[2, 2] = True
        mask[2, 3] = True
        mask[7, 7] = True
        comps = connected_components_8(mask)
        self.assertEqual(len(comps), 2)


def _split_frontier_room_map() -> tuple[np.ndarray, MapMetadata, RobotPose2D]:
    """Two 5-cell frontier segments separated by one occupied cell (10 cells if merged)."""
    h, w = 30, 30
    data = np.full((h, w), 65, dtype=np.int16)
    data[18, 15] = 0
    data[9, 10:15] = -1
    data[10, 10:15] = 0
    data[9, 16:21] = -1
    data[10, 16:21] = 0
    robot = RobotPose2D(0.75, 0.9, 0.0)
    return data, _meta(w, h, 0.05), robot


def _merge_friendly_cfg(**overrides) -> dict:
    cfg = _cfg(
        frontier={"min_clearance_m": 0.0},
        region={
            "min_frontier_cells": 8,
            "min_distance_m": 0.1,
            "max_distance_m": 8.0,
            "min_unknown_gain_cells": 1,
            "unknown_gain_radius_m": 1.0,
            "max_regions": 12,
        },
        region_merge={
            "enabled": True,
            "max_frontier_gap_m": 0.20,
            "max_centroid_distance_m": 0.45,
            "max_bearing_difference_deg": 90.0,
            "require_same_direction_label": False,
        },
    )
    for k, v in overrides.items():
        cfg[k] = v
    return cfg


class TestRegionMerge(unittest.TestCase):
    def test_nearby_split_frontier_clusters_merge(self) -> None:
        data, meta, robot = _split_frontier_room_map()
        result = analyze_frontier_regions(
            data, meta, robot, _merge_friendly_cfg(), cycle_id=12
        )
        self.assertGreaterEqual(result.stats.raw_cluster_count, 2)
        self.assertGreater(result.stats.merge_pairs_accepted, 0)
        self.assertGreater(result.stats.merged_group_count, 0)
        self.assertLess(result.stats.cluster_count_after_merge, result.stats.raw_cluster_count)
        merged = [r for r in result.regions + result.rejected_regions if r.merged]
        self.assertTrue(merged)
        self.assertGreaterEqual(merged[0].frontier_cell_count, 8)

    def test_far_clusters_do_not_merge(self) -> None:
        h, w = 40, 40
        data = np.full((h, w), -1, dtype=np.int16)
        data[15:25, 15:25] = 0
        data[14, 15:20] = -1
        data[14, 25:30] = -1
        robot = RobotPose2D(1.0, 1.0, 0.0)
        cfg = _cfg(frontier={"min_clearance_m": 0.0})
        result = analyze_frontier_regions(data, _meta(w, h, 0.05), robot, cfg, cycle_id=13)
        self.assertEqual(result.stats.merge_pairs_accepted, 0)
        self.assertEqual(result.stats.merged_group_count, 0)

    def test_large_bearing_difference_does_not_merge(self) -> None:
        h, w = 40, 40
        data = np.full((h, w), -1, dtype=np.int16)
        data[10:30, 10:30] = 0
        data[9, 15:18] = -1
        data[29, 15:18] = -1
        robot = RobotPose2D(1.0, 1.0, 0.0)
        cfg = _cfg(
            frontier={"min_clearance_m": 0.0},
            region_merge={
                "enabled": True,
                "max_frontier_gap_m": 2.0,
                "max_centroid_distance_m": 2.0,
                "max_bearing_difference_deg": 25.0,
                "require_same_direction_label": False,
            },
        )
        result = analyze_frontier_regions(data, _meta(w, h, 0.05), robot, cfg, cycle_id=14)
        self.assertEqual(result.stats.merge_pairs_accepted, 0)

    def test_merged_metrics_are_recomputed(self) -> None:
        data, meta, robot = _split_frontier_room_map()
        cfg = _merge_friendly_cfg()
        raw_mask, _ = extract_raw_frontier_mask(data, cfg)
        clusters = connected_components_8(raw_mask)
        merged, ids, _, _, _ = merge_adjacent_clusters(clusters, meta, robot, cfg, cycle_id=15)
        self.assertEqual(len(merged), 1)
        total_cells = sum(len(c) for c in clusters)
        self.assertEqual(len(merged[0]), total_cells)
        self.assertIn("+", ids[0])

    def test_merge_is_deterministic(self) -> None:
        data, meta, robot = _split_frontier_room_map()
        cfg = _merge_friendly_cfg()
        r1 = analyze_frontier_regions(data, meta, robot, cfg, cycle_id=16)
        r2 = analyze_frontier_regions(data, meta, robot, cfg, cycle_id=16)
        self.assertEqual(r1.stats.merge_pairs_accepted, r2.stats.merge_pairs_accepted)
        self.assertEqual(
            [(m.source_clusters, m.result_frontier_cells) for m in r1.merge_log],
            [(m.source_clusters, m.result_frontier_cells) for m in r2.merge_log],
        )

    def test_merge_happens_before_too_small_rejection(self) -> None:
        data, meta, robot = _split_frontier_room_map()
        cfg = _merge_friendly_cfg(region_merge={"enabled": False})
        disabled = analyze_frontier_regions(data, meta, robot, cfg, cycle_id=17)
        too_small_codes = [
            r.code
            for reg in disabled.rejected_regions
            for r in reg.rejection_reasons
        ]
        self.assertIn("REGION_TOO_SMALL", too_small_codes)

        cfg["region_merge"]["enabled"] = True
        cfg["region_merge"]["max_bearing_difference_deg"] = 90.0
        enabled = analyze_frontier_regions(data, meta, robot, cfg, cycle_id=17)
        accepted_merged = [r for r in enabled.regions if r.merged]
        self.assertTrue(accepted_merged)
        self.assertGreaterEqual(accepted_merged[0].frontier_cell_count, 8)


if __name__ == "__main__":
    unittest.main()
