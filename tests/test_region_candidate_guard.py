#!/usr/bin/env python3
"""Tests for region_candidate_guard."""

from __future__ import annotations

import math
import os
import sys
import unittest
from types import SimpleNamespace

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.region_candidate_guard import (  # noqa: E402
    GuardedRegionMetrics,
    ObservationWindow,
    RegionTrack,
    accumulate_yaw_delta,
    apply_near_region_guard,
    apply_stability_gate,
    apply_trajectory_metrics,
    compute_geometric_score,
    guard_regions_for_cycle,
    match_regions_to_tracks,
    rank_geometric_candidates,
)


def _region(rid: str, cx: float, cy: float, bearing: float, cells: int = 10, dist: float = 1.0):
    return SimpleNamespace(
        region_id=rid,
        accepted=True,
        centroid_x=cx,
        centroid_y=cy,
        bearing_relative_deg=bearing,
        frontier_cell_count=cells,
        row_min=10,
        row_max=12,
        col_min=10,
        col_max=12,
        unknown_gain_cells=100,
        unknown_gain_ratio=0.5,
        minimum_clearance_m=0.4,
        mean_clearance_m=0.5,
        distance_to_robot_m=dist,
        source_cluster_ids=[f"C_{rid}"],
    )


CFG = {
    "region_tracking": {
        "enabled": True,
        "max_centroid_match_distance_m": 0.35,
        "max_bearing_difference_deg": 25.0,
        "max_cell_count_change_ratio": 0.60,
        "min_bbox_iou": 0.05,
        "max_missing_cycles": 2,
    },
    "region_stability": {
        "enabled": True,
        "min_consecutive_cycles": 3,
        "min_age_s": 2.0,
        "max_centroid_drift_m": 0.20,
        "max_bearing_drift_deg": 20.0,
        "max_cell_count_change_ratio": 0.40,
    },
    "near_region_guard": {
        "hard_reject_distance_m": 0.60,
        "soft_penalty_end_distance_m": 1.00,
        "soft_penalty_max": 0.45,
        "allow_near_region_if_persistent_cycles": 5,
        "allow_near_region_if_unknown_gain_ratio_over_best": 0.85,
    },
    "geometric_scoring": {
        "enabled": True,
        "weights": {
            "unknown_gain": 0.22,
            "clearance": 0.18,
            "frontier_size": 0.12,
            "distance_preference": 0.13,
            "persistence": 0.12,
            "region_completeness": 0.08,
            "trajectory_novelty": 0.15,
        },
        "penalties": {
            "near_robot": 0.15,
            "recent_observation": 0.20,
            "selection_count": 0.15,
            "visit_count": 0.15,
            "navigation_failure": 0.20,
            "trajectory_revisit": 0.15,
        },
        "preferred_distance_m": 1.50,
        "preferred_distance_tolerance_m": 1.00,
        "minimum_eligible_score": 0.40,
        "top_k_for_qwen": 5,
    },
    "trajectory": {
        "enabled": True,
        "hard_reject_heavily_revisited": False,
        "recent_path_window_s": 60.0,
        "local_region_analysis_radius_m": 0.80,
        "strong_revisit_distance_m": 0.45,
        "weak_revisit_distance_m": 1.20,
    },
    "region_history": {"enabled": False},
}


class TestTracking(unittest.TestCase):
    def test_same_region_keeps_track_id(self) -> None:
        tracks: dict = {}
        r1 = [_region("R1", 1.0, 1.0, 10.0)]
        tracks, m1, _ = match_regions_to_tracks(r1, tracks, 1, 0.0, CFG)
        tid = m1[0]
        r2 = [_region("R2", 1.02, 1.01, 11.0)]
        tracks, m2, _ = match_regions_to_tracks(r2, tracks, 2, 1.0, CFG)
        self.assertEqual(m2[0], tid)

    def test_far_region_gets_new_track_id(self) -> None:
        tracks: dict = {}
        tracks, m1, _ = match_regions_to_tracks([_region("A", 0, 0, 0)], tracks, 1, 0.0, CFG)
        tracks, m2, _ = match_regions_to_tracks([_region("B", 5, 5, 0)], tracks, 2, 1.0, CFG)
        self.assertNotEqual(m1[0], m2[0])

    def test_one_to_one_matching(self) -> None:
        tracks: dict = {}
        t1 = RegionTrack(
            track_id="T000001",
            first_seen_cycle=1,
            last_seen_cycle=1,
            last_centroid=(1.0, 1.0),
            last_bearing_deg=0.0,
            last_bbox=(10, 12, 10, 12),
        )
        t2 = RegionTrack(
            track_id="T000002",
            first_seen_cycle=1,
            last_seen_cycle=1,
            last_centroid=(3.0, 1.0),
            last_bearing_deg=0.0,
            last_bbox=(20, 22, 10, 12),
        )
        tracks = {"T000001": t1, "T000002": t2}
        regions = [_region("a", 1.01, 1.0, 1.0), _region("b", 3.01, 1.0, 1.0)]
        _, mapping, _ = match_regions_to_tracks(regions, tracks, 2, 1.0, CFG)
        self.assertEqual(len(set(mapping.values())), 2)

    def test_tracking_is_deterministic(self) -> None:
        r = [_region("X", 2.0, 2.0, 5.0)]
        t1, m1, _ = match_regions_to_tracks(r, {}, 1, 0.0, CFG)
        t2, m2, _ = match_regions_to_tracks(r, {}, 1, 0.0, CFG)
        self.assertEqual(m1, m2)


class TestStabilityAndNear(unittest.TestCase):
    def test_hard_reject_very_near_region(self) -> None:
        track = RegionTrack("T1", 1, 1, first_seen_time_s=0.0, consecutive_seen_cycles=5)
        region = _region("n", 0, 0, 0, dist=0.4)
        m = apply_stability_gate(region, track, 10.0, CFG)
        m.snapshot_eligible = True
        m = apply_near_region_guard(region, m, 100, CFG)
        self.assertIn("REGION_TOO_CLOSE_HARD", m.stability_rejection_reasons)

    def test_persistent_near_region_can_survive(self) -> None:
        track = RegionTrack("T1", 1, 1, first_seen_time_s=0.0, consecutive_seen_cycles=6)
        region = _region("n", 0, 0, 0, dist=0.8)
        m = apply_stability_gate(region, track, 10.0, CFG)
        m.snapshot_eligible = True
        m = apply_near_region_guard(region, m, 100, CFG)
        self.assertNotIn("REGION_NEAR_ROBOT_TRANSIENT", m.stability_rejection_reasons)


class TestGeometricScore(unittest.TestCase):
    def test_geo_score_before_and_after_trajectory(self) -> None:
        track = RegionTrack("T1", 1, 1, first_seen_time_s=0.0, consecutive_seen_cycles=5)
        region = _region("a", 1, 1, 0, dist=1.5)
        m = apply_stability_gate(region, track, 10.0, CFG)
        m.trajectory_novelty_score = 0.2
        m.trajectory_revisit_penalty = 0.8
        m = compute_geometric_score(
            region, m, track, CFG, max_unknown_gain=100, max_frontier_cells=10, max_clearance=0.5
        )
        self.assertLess(m.geo_score, m.geo_score_before_trajectory)

    def test_trajectory_near_region_soft_penalty_only(self) -> None:
        from src.planning.robot_trajectory_store import TrajectorySession, add_pose_sample

        session = TrajectorySession("TRJ", "t", "map")
        add_pose_sample(session, CFG, stamp_sec=0.0, x=1.0, y=0.0, yaw_rad=0.0, valid=True)
        add_pose_sample(session, CFG, stamp_sec=1.0, x=2.0, y=0.0, yaw_rad=0.0, valid=True)
        track = RegionTrack("T1", 1, 1, first_seen_time_s=0.0, consecutive_seen_cycles=5)
        region = _region("near", 1.0, 0.1, 0, dist=1.5)
        m = apply_stability_gate(region, track, 10.0, CFG)
        m.snapshot_eligible = True
        m = apply_trajectory_metrics(region, m, session, [], CFG, 5.0)
        self.assertTrue(m.snapshot_eligible)
        self.assertLess(m.trajectory_novelty_score, 1.0)

    def test_score_is_between_zero_and_one(self) -> None:
        track = RegionTrack("T1", 1, 1, first_seen_time_s=0.0, consecutive_seen_cycles=5)
        region = _region("a", 1, 1, 0, dist=1.5)
        m = apply_stability_gate(region, track, 10.0, CFG)
        m = compute_geometric_score(region, m, track, CFG, max_unknown_gain=100, max_frontier_cells=10, max_clearance=0.5)
        self.assertGreaterEqual(m.geo_score, 0.0)
        self.assertLessEqual(m.geo_score, 1.0)

    def test_scoring_is_deterministic(self) -> None:
        track = RegionTrack("T1", 1, 1, first_seen_time_s=0.0, consecutive_seen_cycles=5)
        region = _region("a", 1, 1, 0, dist=1.5)
        m1 = compute_geometric_score(region, apply_stability_gate(region, track, 10.0, CFG), track, CFG, max_unknown_gain=100, max_frontier_cells=10, max_clearance=0.5)
        m2 = compute_geometric_score(region, apply_stability_gate(region, track, 10.0, CFG), track, CFG, max_unknown_gain=100, max_frontier_cells=10, max_clearance=0.5)
        self.assertEqual(m1.geo_score, m2.geo_score)


class TestObservationWindow(unittest.TestCase):
    def test_yaw_wraparound_accumulation(self) -> None:
        self.assertAlmostEqual(abs(math.degrees(accumulate_yaw_delta(math.radians(179), math.radians(-179)))), 2.0, places=0)

    def test_incomplete_scan_blocks_snapshot(self) -> None:
        obs = ObservationWindow(state="OBSERVING", accumulated_rotation_deg=90.0)
        self.assertEqual(obs.snapshot_block_reason(1, True), "SNAPSHOT_FULL_SCAN_NOT_COMPLETE")


if __name__ == "__main__":
    unittest.main()
