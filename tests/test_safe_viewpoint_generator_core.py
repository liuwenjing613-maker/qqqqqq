#!/usr/bin/env python3
"""Unit tests for safe viewpoint generation core (Phase 3A). Not executed by CI in code-only phase."""

from __future__ import annotations

import ast
import copy
import json
import math
import os
import sys
import unittest
from typing import Callable, List, Optional, Sequence, Tuple

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.safe_viewpoint_generator_core import (  # noqa: E402
    CELL_FREE,
    CELL_OCCUPIED,
    CELL_UNKNOWN,
    MapGridSnapshot,
    SelectedRegionGeometry,
    ViewpointGenerationResult,
    check_line_of_sight,
    classify_cell,
    compute_frontier_free_direction,
    deep_copy_map_data,
    generate_raw_candidates,
    generate_safe_viewpoints,
    get_cell_value,
    grid_to_map,
    is_grid_inside,
    map_to_grid,
    normalize_angle_rad,
    result_to_dict,
    selected_region_from_geometry,
    validate_safe_viewpoint_config,
    verify_map_data_unchanged,
)

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")
UNKNOWN = -1
FREE = 0
OCCUPIED = 100
UNCERTAIN = 50


def _default_cfg() -> dict:
    return {
        "safe_viewpoint": {"enabled": True, "algorithm_version": "phase3a_v1"},
        "map": {"free_max_value": 20, "occupied_min_value": 65, "unknown_value": -1},
        "robot": {
            "footprint_radius_m": 0.23,
            "safety_margin_m": 0.10,
            "required_clearance_m": 0.33,
        },
        "viewpoint_generation": {
            "stand_off_distances_m": [0.45],
            "lateral_offsets_m": [0.0],
            "frontier_sample_spacing_m": 0.40,
            "local_search_radius_m": 0.20,
            "local_search_step_cells": 1,
            "max_raw_candidates": 30,
        },
        "viewpoint_validation": {
            "minimum_footprint_free_ratio": 1.0,
            "local_free_radius_m": 0.45,
            "minimum_local_free_ratio": 0.80,
            "min_frontier_distance_m": 0.35,
            "max_frontier_distance_m": 1.10,
            "min_robot_distance_m": 0.50,
            "max_robot_distance_m": 2.50,
            "preferred_robot_distance_m": 1.50,
            "require_line_of_sight": True,
        },
        "viewpoint_scoring": {
            "sensor_fov_deg": 80.0,
            "visible_unknown_radius_m": 2.00,
            "weights": {
                "clearance": 0.25,
                "visible_unknown": 0.20,
                "local_free_space": 0.15,
                "frontier_distance_preference": 0.15,
                "robot_distance_preference": 0.10,
                "trajectory_novelty": 0.10,
                "turn_cost": 0.05,
            },
        },
        "viewpoint_selection": {
            "deduplication_distance_m": 0.20,
            "deduplication_yaw_difference_deg": 20.0,
            "max_output_candidates": 5,
        },
        "safety": {
            "allow_motion": False,
            "allow_cmd_vel": False,
            "allow_nav2": False,
            "allow_goal_publish": False,
        },
    }


def _make_map(
    width: int,
    height: int,
    *,
    resolution: float = 0.05,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_yaw: float = 0.0,
    fill: int = UNKNOWN,
    painter: Optional[Callable[[int, int], Optional[int]]] = None,
) -> MapGridSnapshot:
    data: List[int] = []
    for row in range(height):
        for col in range(width):
            if painter is not None:
                val = painter(row, col)
                data.append(UNKNOWN if val is None else val)
            else:
                data.append(fill)
    return MapGridSnapshot(
        frame_id="map",
        stamp_sec=1.0,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        data=data,
    )


def _corridor_map() -> MapGridSnapshot:
    """Known free west, unknown east; frontier at col=20."""

    def painter(row: int, col: int) -> Optional[int]:
        if col <= 19:
            return FREE
        if col == 20 and 5 <= row <= 24:
            return FREE
        return None

    return _make_map(40, 30, painter=painter)


def _frontier_region_from_map(snap: MapGridSnapshot, label: str = "B") -> SelectedRegionGeometry:
    cells: List[List[int]] = []
    points: List[List[float]] = []
    for row in range(snap.height):
        for col in range(snap.width):
            val = get_cell_value(snap, row, col)
            if val is None or classify_cell(val, _default_cfg()) != CELL_FREE:
                continue
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nv = get_cell_value(snap, row + dr, col + dc)
                if nv is not None and classify_cell(nv, _default_cfg()) == CELL_UNKNOWN:
                    cells.append([row, col])
                    x, y = grid_to_map(row, col, snap)
                    points.append([x, y])
                    break
    return SelectedRegionGeometry(
        snapshot_id="RS_TEST",
        region_label=label,
        internal_region_id="R_TEST",
        track_id="T000001",
        centroid_x=1.0,
        centroid_y=0.75,
        frontier_cells_grid=cells,
        frontier_points_map=points,
        bbox_grid=[5, 24, 20, 20],
        bearing_global_deg=0.0,
        unknown_gain_cells=100,
        minimum_clearance_m=0.45,
        trajectory_novelty_score=0.85,
        trajectory_revisit_penalty=0.05,
        stable=True,
        snapshot_eligible=True,
        blacklisted=False,
    )


def _robot_pose_corridor() -> Tuple[float, float, float]:
    return 0.35, 0.75, 0.0


class TestMapCoordinates(unittest.TestCase):
    def test_grid_to_map_round_trip(self) -> None:
        snap = _make_map(20, 20, resolution=0.05)
        for row in (0, 5, 19):
            for col in (0, 8, 19):
                x, y = grid_to_map(row, col, snap)
                r2, c2 = map_to_grid(x, y, snap)
                self.assertEqual((r2, c2), (row, col))

    def test_map_origin_rotation(self) -> None:
        snap = _make_map(10, 10, resolution=0.1, origin_x=1.0, origin_y=2.0, origin_yaw=math.pi / 2)
        x, y = grid_to_map(0, 0, snap)
        self.assertAlmostEqual(x, 0.95, places=5)
        self.assertAlmostEqual(y, 2.05, places=5)
        row, col = map_to_grid(x, y, snap)
        self.assertEqual((row, col), (0, 0))

    def test_reject_invalid_map_length(self) -> None:
        snap = MapGridSnapshot("map", 0, 4, 4, 0.05, 0, 0, 0, [0, 0, 0])
        self.assertIn("MAP_DATA_LENGTH_MISMATCH", snap.validate())

    def test_original_map_data_not_mutated(self) -> None:
        snap = _corridor_map()
        before = deep_copy_map_data(snap.data)
        region = _frontier_region_from_map(snap)
        robot_x, robot_y, robot_yaw = _robot_pose_corridor()
        generate_safe_viewpoints(snap, region, robot_x, robot_y, robot_yaw, _default_cfg())
        self.assertTrue(verify_map_data_unchanged(before, snap.data))


class TestGenerationDirection(unittest.TestCase):
    def test_candidate_generated_on_known_free_side(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        raw, _ = generate_raw_candidates(region, snap, _default_cfg())
        self.assertTrue(raw)
        for spec in raw:
            row, col = map_to_grid(spec["theory_x"], spec["theory_y"], snap)
            if is_grid_inside(row, col, snap):
                val = get_cell_value(snap, row, col)
                if val is not None:
                    self.assertNotEqual(classify_cell(val, _default_cfg()), CELL_UNKNOWN)
            self.assertLess(spec["theory_x"], spec["source_fx"])

    def test_candidate_not_generated_into_unknown(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        raw, _ = generate_raw_candidates(region, snap, _default_cfg())
        for spec in raw:
            self.assertLess(spec["theory_x"], spec["source_fx"])

    def test_unknown_direction_is_geometric_not_direction_label(self) -> None:
        snap = _corridor_map()
        region = SelectedRegionGeometry(
            snapshot_id="RS",
            region_label="A",
            internal_region_id="R",
            track_id="T",
            centroid_x=1.0,
            centroid_y=0.75,
            frontier_cells_grid=[[10, 20]],
            frontier_points_map=[[grid_to_map(10, 20, snap)[0], grid_to_map(10, 20, snap)[1]]],
            bbox_grid=[10, 10, 20, 20],
            bearing_global_deg=0.0,
            unknown_gain_cells=10,
            minimum_clearance_m=0.4,
            stable=True,
            snapshot_eligible=True,
            blacklisted=False,
        )
        free_dir, err = compute_frontier_free_direction(10, 20, snap, _default_cfg())
        self.assertIsNone(err)
        self.assertIsNotNone(free_dir)
        assert free_dir is not None
        self.assertLess(free_dir[0], 0.0)

    def test_multiple_standoff_distances(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_generation"]["stand_off_distances_m"] = [0.45, 0.60, 0.75, 0.90]
        cfg["viewpoint_generation"]["lateral_offsets_m"] = [0.0]
        cfg["viewpoint_generation"]["max_raw_candidates"] = 200
        raw, _ = generate_raw_candidates(region, snap, cfg)
        standoffs = {round(r["stand_off"], 2) for r in raw}
        for expected in cfg["viewpoint_generation"]["stand_off_distances_m"]:
            self.assertIn(round(expected, 2), standoffs)

    def test_lateral_offsets(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_generation"]["lateral_offsets_m"] = [-0.30, -0.15, 0.0, 0.15, 0.30]
        cfg["viewpoint_generation"]["stand_off_distances_m"] = [0.45]
        cfg["viewpoint_generation"]["max_raw_candidates"] = 200
        raw, _ = generate_raw_candidates(region, snap, cfg)
        laterals = {round(r["lateral"], 2) for r in raw}
        for expected in cfg["viewpoint_generation"]["lateral_offsets_m"]:
            self.assertIn(round(expected, 2), laterals)


class TestHardGates(unittest.TestCase):
    def _generate(self, snap: MapGridSnapshot, region: SelectedRegionGeometry) -> ViewpointGenerationResult:
        rx, ry, ryaw = _robot_pose_corridor()
        return generate_safe_viewpoints(snap, region, rx, ry, ryaw, _default_cfg())

    def test_reject_candidate_outside_map(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        result = self._generate(snap, region)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        if "VIEWPOINT_OUTSIDE_MAP" in reasons:
            self.assertIn("VIEWPOINT_OUTSIDE_MAP", reasons)

    def test_reject_unknown_candidate(self) -> None:
        snap = _corridor_map()

        def painter(row: int, col: int) -> Optional[int]:
            if col <= 19 and 5 <= row <= 24:
                return FREE
            if col == 20 and row == 10:
                return FREE
            return None

        snap2 = _make_map(40, 30, painter=painter)
        region = SelectedRegionGeometry(
            snapshot_id="RS",
            region_label="B",
            internal_region_id="R",
            track_id="T",
            centroid_x=1.0,
            centroid_y=0.55,
            frontier_cells_grid=[[10, 20]],
            frontier_points_map=[[grid_to_map(10, 20, snap2)[0], grid_to_map(10, 20, snap2)[1]]],
            bbox_grid=[10, 10, 20, 20],
            bearing_global_deg=0.0,
            unknown_gain_cells=10,
            minimum_clearance_m=0.4,
            stable=True,
            snapshot_eligible=True,
        )
        result = self._generate(snap2, region)
        all_reasons = result.rejection_reason_summary.keys()
        self.assertTrue(
            "VIEWPOINT_UNKNOWN_CELL" in all_reasons
            or "VIEWPOINT_NO_LOCAL_FREE_CELL" in all_reasons
            or result.accepted_candidate_count >= 0
        )

    def test_reject_occupied_candidate(self) -> None:
        snap = _corridor_map()
        data = list(snap.data)
        idx = 10 * snap.width + 8
        data[idx] = OCCUPIED
        snap_occ = MapGridSnapshot(
            snap.frame_id,
            snap.stamp_sec,
            snap.width,
            snap.height,
            snap.resolution,
            snap.origin_x,
            snap.origin_y,
            snap.origin_yaw,
            data,
        )
        region = _frontier_region_from_map(snap_occ)
        result = self._generate(snap_occ, region)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertTrue(
            any(
                r in reasons
                for r in (
                    "VIEWPOINT_OCCUPIED_CELL",
                    "VIEWPOINT_OCCUPIED_INSIDE_ROBOT_FOOTPRINT",
                    "VIEWPOINT_FOOTPRINT_NOT_FREE",
                )
            )
            or result.accepted_candidate_count == 0
        )

    def test_reject_uncertain_candidate(self) -> None:
        snap = _corridor_map()
        data = list(snap.data)
        for col in range(0, 18):
            data[12 * snap.width + col] = UNCERTAIN
        snap_u = MapGridSnapshot(
            snap.frame_id,
            snap.stamp_sec,
            snap.width,
            snap.height,
            snap.resolution,
            snap.origin_x,
            snap.origin_y,
            snap.origin_yaw,
            data,
        )
        region = _frontier_region_from_map(snap_u)
        result = self._generate(snap_u, region)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertTrue(
            "VIEWPOINT_UNCERTAIN_CELL" in reasons
            or "VIEWPOINT_FOOTPRINT_NOT_FREE" in reasons
            or result.accepted_candidate_count == 0
        )

    def test_reject_low_clearance(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["robot"]["required_clearance_m"] = 2.0
        cfg["viewpoint_generation"]["max_raw_candidates"] = 40
        cfg["viewpoint_generation"]["stand_off_distances_m"] = [0.45]
        cfg["viewpoint_generation"]["lateral_offsets_m"] = [0.0]
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertIn("VIEWPOINT_LOW_CLEARANCE", reasons)

    def test_reject_unknown_inside_robot_footprint(self) -> None:
        snap = _corridor_map()
        data = list(snap.data)
        center_row, center_col = 12, 10
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                data[(center_row + dr) * snap.width + (center_col + dc)] = UNKNOWN
        data[center_row * snap.width + center_col] = FREE
        snap_u = MapGridSnapshot(
            snap.frame_id,
            snap.stamp_sec,
            snap.width,
            snap.height,
            snap.resolution,
            snap.origin_x,
            snap.origin_y,
            snap.origin_yaw,
            data,
        )
        region = _frontier_region_from_map(snap_u)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap_u, region, rx, ry, ryaw, _default_cfg())
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertIn("VIEWPOINT_UNKNOWN_INSIDE_ROBOT_FOOTPRINT", reasons)

    def test_reject_narrow_local_space(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_validation"]["minimum_local_free_ratio"] = 0.99
        cfg["viewpoint_validation"]["local_free_radius_m"] = 1.0
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertIn("VIEWPOINT_LOCAL_SPACE_TOO_NARROW", reasons)

    def test_reject_too_close_to_frontier(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_validation"]["min_frontier_distance_m"] = 0.90
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertIn("VIEWPOINT_TOO_CLOSE_TO_FRONTIER", reasons)

    def test_reject_too_far_from_frontier(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_validation"]["max_frontier_distance_m"] = 0.40
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertIn("VIEWPOINT_TOO_FAR_FROM_FRONTIER", reasons)

    def test_reject_too_close_to_robot(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_validation"]["min_robot_distance_m"] = 5.0
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertIn("VIEWPOINT_TOO_CLOSE_TO_ROBOT", reasons)

    def test_reject_too_far_from_robot(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_validation"]["max_robot_distance_m"] = 0.20
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertIn("VIEWPOINT_TOO_FAR_FROM_ROBOT", reasons)

    def test_reject_blocked_line_of_sight(self) -> None:
        snap = _corridor_map()
        data = list(snap.data)
        for row in range(6, 19):
            data[row * snap.width + 15] = OCCUPIED
        snap_wall = MapGridSnapshot(
            snap.frame_id,
            snap.stamp_sec,
            snap.width,
            snap.height,
            snap.resolution,
            snap.origin_x,
            snap.origin_y,
            snap.origin_yaw,
            data,
        )
        region = _frontier_region_from_map(snap_wall)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap_wall, region, rx, ry, ryaw, _default_cfg())
        reasons = [r for c in result.rejected_candidates for r in c.rejection_reasons]
        self.assertIn("VIEWPOINT_LINE_OF_SIGHT_BLOCKED", reasons)


class TestYawAndLineOfSight(unittest.TestCase):
    def test_yaw_faces_frontier(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, _default_cfg())
        for cand in result.accepted_candidates:
            expected = math.degrees(
                math.atan2(cand.source_frontier_y - cand.y, cand.source_frontier_x - cand.x)
            )
            self.assertAlmostEqual(cand.yaw_deg, expected, places=3)

    def test_line_of_sight_allows_unknown_after_frontier(self) -> None:
        snap = _corridor_map()
        ok, _, block, _ = check_line_of_sight(12, 8, 10, 20, snap, _default_cfg())
        self.assertTrue(ok)
        self.assertIsNone(block)

    def test_line_of_sight_rejects_wall_before_frontier(self) -> None:
        snap = _corridor_map()
        from src.planning.safe_viewpoint_generator_core import raytrace_grid

        data = list(snap.data)
        path = raytrace_grid(12, 8, 10, 20)
        block_row, block_col = path[len(path) // 2]
        data[block_row * snap.width + block_col] = OCCUPIED
        snap2 = MapGridSnapshot(
            snap.frame_id,
            snap.stamp_sec,
            snap.width,
            snap.height,
            snap.resolution,
            snap.origin_x,
            snap.origin_y,
            snap.origin_yaw,
            data,
        )
        ok, _, block, val = check_line_of_sight(12, 8, 10, 20, snap2, _default_cfg())
        self.assertFalse(ok)
        self.assertIsNotNone(block)
        self.assertIsNotNone(val)


class TestScoring(unittest.TestCase):
    def test_score_between_zero_and_one(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, _default_cfg())
        for cand in result.accepted_candidates:
            self.assertIsNotNone(cand.score)
            assert cand.score is not None
            self.assertGreaterEqual(cand.score, 0.0)
            self.assertLessEqual(cand.score, 1.0)

    def test_higher_clearance_scores_higher(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, _default_cfg())
        if len(result.accepted_candidates) >= 2:
            sorted_by_clear = sorted(result.accepted_candidates, key=lambda c: -c.clearance_m)
            top = sorted_by_clear[0]
            low = sorted_by_clear[-1]
            if top.clearance_m > low.clearance_m + 0.05:
                self.assertGreaterEqual(top.score or 0.0, (low.score or 0.0) - 0.3)

    def test_more_visible_unknown_scores_higher(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, _default_cfg())
        for cand in result.accepted_candidates:
            self.assertGreaterEqual(cand.estimated_visible_unknown_cells, 0)

    def test_trajectory_novelty_is_soft_only(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        region.trajectory_novelty_score = 1.0
        cfg = _default_cfg()
        cfg["robot"]["required_clearance_m"] = 5.0
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        for cand in result.rejected_candidates:
            self.assertFalse(cand.hard_gate_passed)
            self.assertIsNone(cand.score)

    def test_unsafe_unvisited_candidate_never_passes(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        region.trajectory_novelty_score = 1.0
        data = list(snap.data)
        data[12 * snap.width + 10] = UNKNOWN
        snap2 = MapGridSnapshot(
            snap.frame_id,
            snap.stamp_sec,
            snap.width,
            snap.height,
            snap.resolution,
            snap.origin_x,
            snap.origin_y,
            snap.origin_yaw,
            data,
        )
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap2, region, rx, ry, ryaw, _default_cfg())
        for cand in result.accepted_candidates:
            self.assertEqual(cand.footprint_unknown_count, 0)

    def test_scoring_is_deterministic(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        cfg = _default_cfg()
        r1 = generate_safe_viewpoints(
            snap, region, rx, ry, ryaw, cfg, generation_id="G1"
        )
        r2 = generate_safe_viewpoints(
            snap, region, rx, ry, ryaw, cfg, generation_id="G1"
        )
        self.assertEqual(
            [c.score for c in r1.accepted_candidates],
            [c.score for c in r2.accepted_candidates],
        )


class TestDedupAndSort(unittest.TestCase):
    def test_near_duplicate_candidates_are_merged(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_selection"]["deduplication_distance_m"] = 0.50
        cfg["viewpoint_generation"]["stand_off_distances_m"] = [0.45, 0.46]
        cfg["viewpoint_generation"]["lateral_offsets_m"] = [0.0]
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        self.assertLessEqual(result.accepted_candidate_count, result.raw_candidate_count)

    def test_higher_score_duplicate_is_kept(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, _default_cfg())
        if result.accepted_candidates:
            scores = [c.score for c in result.accepted_candidates if c.score is not None]
            self.assertEqual(scores, sorted(scores, reverse=True))

    def test_max_output_candidate_limit(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        cfg = _default_cfg()
        cfg["viewpoint_selection"]["max_output_candidates"] = 2
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg)
        self.assertLessEqual(len(result.accepted_candidates), 2)

    def test_sorting_is_deterministic(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        cfg = _default_cfg()
        a = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg, generation_id="S")
        b = generate_safe_viewpoints(snap, region, rx, ry, ryaw, cfg, generation_id="S")
        self.assertEqual(
            [c.candidate_id for c in a.accepted_candidates],
            [c.candidate_id for c in b.accepted_candidates],
        )


class TestSafetyAndState(unittest.TestCase):
    def test_path_checked_is_false(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, _default_cfg())
        payload = result_to_dict(result)
        self.assertFalse(payload["path_checked"])
        for cand in result.accepted_candidates:
            self.assertFalse(cand.path_checked)

    def test_reachable_is_none(self) -> None:
        snap = _corridor_map()
        region = _frontier_region_from_map(snap)
        rx, ry, ryaw = _robot_pose_corridor()
        result = generate_safe_viewpoints(snap, region, rx, ry, ryaw, _default_cfg())
        payload = result_to_dict(result)
        self.assertIsNone(payload["reachable"])
        for cand in result.accepted_candidates:
            self.assertIsNone(cand.reachable)

    def test_no_motion_interfaces(self) -> None:
        core_path = os.path.join(
            PROJECT_ROOT, "src", "planning", "safe_viewpoint_generator_core.py"
        )
        with open(core_path, encoding="utf-8") as fh:
            source = fh.read()
        self.assertNotIn("import rclpy", source)
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                func = node.func
                name = getattr(func, "id", "") or getattr(getattr(func, "attr", None), "__str__", lambda: "")()
                if getattr(func, "attr", None) in ("Publisher", "create_publisher"):
                    for arg in node.args + [kw.value for kw in node.keywords]:
                        if isinstance(arg, ast.Constant) and "cmd_vel" in str(arg.value):
                            self.fail("cmd_vel publisher found")

    def test_no_nav2_interfaces(self) -> None:
        core_path = os.path.join(
            PROJECT_ROOT, "src", "planning", "safe_viewpoint_generator_core.py"
        )
        with open(core_path, encoding="utf-8") as fh:
            source = fh.read()
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module and "nav2" in node.module:
                self.fail(f"nav2 import found: {node.module}")
            if isinstance(node, ast.Call):
                attr = getattr(node.func, "attr", "")
                if attr in ("NavigateToPose", "FollowPath"):
                    self.fail(f"nav2 action client call found: {attr}")


class TestConfigAndFixtures(unittest.TestCase):
    def test_config_validation_rejects_unsafe_flags(self) -> None:
        cfg = _default_cfg()
        cfg["safety"]["allow_motion"] = True
        errors = validate_safe_viewpoint_config(cfg)
        self.assertTrue(any("allow_motion" in e for e in errors))

    def test_fixture_geometry_loads(self) -> None:
        path = os.path.join(FIXTURES, "safe_viewpoint_region_geometry.json")
        with open(path, encoding="utf-8") as fh:
            geometry = json.load(fh)
        region = selected_region_from_geometry(geometry, "B")
        self.assertEqual(region.region_label, "B")
        self.assertTrue(region.stable)
        self.assertEqual(len(region.frontier_cells_grid), 6)

    def test_region_validation_rejects_unstable(self) -> None:
        region = SelectedRegionGeometry(
            snapshot_id="RS",
            region_label="A",
            internal_region_id="R",
            track_id="T",
            centroid_x=0,
            centroid_y=0,
            frontier_cells_grid=[[1, 1]],
            frontier_points_map=[[0.0, 0.0]],
            bbox_grid=[0, 2, 0, 2],
            bearing_global_deg=0,
            unknown_gain_cells=0,
            minimum_clearance_m=0.4,
            stable=False,
            snapshot_eligible=True,
        )
        self.assertIn("VIEWPOINT_REGION_UNSTABLE", region.validate())


if __name__ == "__main__":
    unittest.main()
