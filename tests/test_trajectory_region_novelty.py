#!/usr/bin/env python3
"""Tests for trajectory region novelty integration — created but not executed."""

from __future__ import annotations

import inspect
import os
import sys
import unittest
from types import SimpleNamespace

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
    deep_copy_grid_data,
    render_annotated_map,
)
from src.planning.region_candidate_guard import (  # noqa: E402
    GuardedRegionMetrics,
    RegionTrack,
    apply_trajectory_metrics,
    compute_geometric_score,
)
from src.planning.robot_trajectory_store import (  # noqa: E402
    TrajectorySession,
    TrajectoryVertex,
    add_pose_sample,
    compute_trajectory_region_metrics,
)
from src.vlm.qwen_region_selector_core import (  # noqa: E402
    build_region_selection_prompt,
)


CFG = {
    "trajectory": {
        "enabled": True,
        "sample_period_s": 1.0,
        "max_tf_age_s": 0.30,
        "min_vertex_distance_m": 0.05,
        "min_vertex_yaw_change_deg": 10.0,
        "max_vertex_interval_s": 5.0,
        "recent_path_window_s": 60.0,
        "local_region_analysis_radius_m": 0.80,
        "visited_corridor_radius_m": 0.35,
        "strong_revisit_distance_m": 0.45,
        "weak_revisit_distance_m": 1.20,
        "hard_reject_heavily_revisited": False,
    },
    "region_tracking": {"enabled": False},
    "region_stability": {"enabled": False},
    "near_region_guard": {
        "hard_reject_distance_m": 0.60,
        "soft_penalty_end_distance_m": 1.00,
        "soft_penalty_max": 0.45,
    },
    "region_history": {"enabled": False},
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
        "minimum_eligible_score": 0.40,
    },
}


def _line_session() -> TrajectorySession:
    session = TrajectorySession("TRJ_LINE", "t", "map")
    add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
    add_pose_sample(session, CFG, stamp_sec=1.0, x=2.0, y=0.0, yaw_rad=0.0, valid=True)
    return session


def _region(cx: float, cy: float, clearance: float = 0.4) -> SimpleNamespace:
    return SimpleNamespace(
        region_id="R1",
        accepted=True,
        centroid_x=cx,
        centroid_y=cy,
        bearing_relative_deg=0.0,
        frontier_cell_count=10,
        row_min=0,
        row_max=1,
        col_min=0,
        col_max=1,
        unknown_gain_cells=100,
        unknown_gain_ratio=0.5,
        minimum_clearance_m=clearance,
        mean_clearance_m=clearance,
        distance_to_robot_m=1.0,
        source_cluster_ids=["C1"],
    )


class TestTrajectoryNovelty(unittest.TestCase):
    def test_empty_path_gives_high_novelty(self) -> None:
        m = compute_trajectory_region_metrics(1.0, 1.0, [], [], CFG, 10.0)
        self.assertEqual(m.trajectory_novelty_score, 1.0)
        self.assertEqual(m.trajectory_revisit_penalty, 0.0)

    def test_region_far_from_path_has_high_novelty(self) -> None:
        session = _line_session()
        m = compute_trajectory_region_metrics(5.0, 5.0, session.vertices, [], CFG, 10.0)
        self.assertGreater(m.trajectory_novelty_score, 0.7)

    def test_region_near_path_has_lower_novelty(self) -> None:
        session = _line_session()
        m = compute_trajectory_region_metrics(1.0, 0.1, session.vertices, [], CFG, 10.0)
        self.assertLess(m.trajectory_novelty_score, 0.5)
        self.assertGreater(m.trajectory_revisit_penalty, 0.3)

    def test_dense_recent_path_has_revisit_penalty(self) -> None:
        session = TrajectorySession("TRJ_DENSE", "t", "map")
        for i in range(6):
            add_pose_sample(
                session,
                CFG,
                stamp_sec=float(i),
                x=float(i) * 0.2,
                y=0.0,
                yaw_rad=0.0,
                valid=True,
            )
        m = compute_trajectory_region_metrics(0.5, 0.05, session.vertices, [], CFG, 5.0)
        self.assertGreater(m.trajectory_revisit_penalty, 0.4)

    def test_stationary_raw_samples_do_not_inflate_density(self) -> None:
        session = TrajectorySession("TRJ_STATIC", "t", "map")
        for t in range(10):
            add_pose_sample(
                session, CFG, stamp_sec=float(t), x=0.0, y=0.0, yaw_rad=0.0, valid=True
            )
        self.assertEqual(len(session.vertices), 1)
        m = compute_trajectory_region_metrics(0.2, 0.2, session.vertices, [], CFG, 10.0)
        self.assertLess(m.nearby_trajectory_vertex_count, 3)

    def test_unvisited_region_scores_higher_when_other_factors_equal(self) -> None:
        session = _line_session()
        track = RegionTrack("T1", 1, 1)
        near = apply_trajectory_metrics(
            _region(1.0, 0.1),
            GuardedRegionMetrics(snapshot_eligible=True),
            session,
            [],
            CFG,
            5.0,
        )
        far = apply_trajectory_metrics(
            _region(5.0, 5.0),
            GuardedRegionMetrics(snapshot_eligible=True),
            session,
            [],
            CFG,
            5.0,
        )
        near = compute_geometric_score(
            _region(1.0, 0.1), near, track, CFG,
            max_unknown_gain=100, max_frontier_cells=10, max_clearance=0.4,
        )
        far = compute_geometric_score(
            _region(5.0, 5.0), far, track, CFG,
            max_unknown_gain=100, max_frontier_cells=10, max_clearance=0.4,
        )
        self.assertGreater(far.geo_score, near.geo_score)

    def test_safer_visited_region_can_beat_unsafe_unvisited_region(self) -> None:
        session = _line_session()
        track = RegionTrack("T1", 1, 1)
        visited_safe = apply_trajectory_metrics(
            _region(1.0, 0.1, clearance=0.8),
            GuardedRegionMetrics(snapshot_eligible=True),
            session,
            [],
            CFG,
            5.0,
        )
        unvisited_unsafe = apply_trajectory_metrics(
            _region(5.0, 5.0, clearance=0.05),
            GuardedRegionMetrics(snapshot_eligible=True),
            session,
            [],
            CFG,
            5.0,
        )
        visited_safe = compute_geometric_score(
            _region(1.0, 0.1, clearance=0.8),
            visited_safe,
            track,
            CFG,
            max_unknown_gain=100,
            max_frontier_cells=10,
            max_clearance=0.8,
        )
        unvisited_unsafe = compute_geometric_score(
            _region(5.0, 5.0, clearance=0.05),
            unvisited_unsafe,
            track,
            CFG,
            max_unknown_gain=100,
            max_frontier_cells=10,
            max_clearance=0.8,
        )
        self.assertGreater(visited_safe.geo_score, unvisited_unsafe.geo_score)

    def test_trajectory_scoring_is_deterministic(self) -> None:
        session = _line_session()
        m1 = compute_trajectory_region_metrics(1.0, 0.1, session.vertices, [], CFG, 5.0)
        m2 = compute_trajectory_region_metrics(1.0, 0.1, session.vertices, [], CFG, 5.0)
        self.assertEqual(m1.trajectory_novelty_score, m2.trajectory_novelty_score)


class TestIntegration(unittest.TestCase):
    def test_snapshot_contains_trajectory_fields(self) -> None:
        region = FrontierRegion(
            region_id="R1",
            accepted=True,
            snapshot_eligible=True,
            stable=True,
            geo_score=0.7,
            geo_rank=1,
            trajectory_novelty_score=0.9,
            geo_score_before_trajectory=0.75,
            geo_score_after_trajectory=0.7,
        )
        result = FrontierAnalysisResult(
            cycle_id=1,
            map_health=MapHealthResult(ok=True, status="OK"),
            stats=FrontierExtractionStats(status="OK"),
            regions=[region],
        )
        meta = MapMetadata(
            width=10,
            height=10,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            frame_id="map",
            stamp_sec=0.0,
        )
        robot = RobotPose2D(0.0, 0.0, 0.0)
        payload = build_region_snapshot_payload(
            result,
            meta,
            robot,
            "RS_TEST",
            "2026-07-13T00:00:00+00:00",
            trajectory_meta={
                "trajectory_session_id": "TRJ_TEST",
                "trajectory_revision": 3,
                "trajectory_length_m": 2.0,
                "trajectory_raw_sample_count": 10,
                "trajectory_vertex_count": 2,
            },
        )
        self.assertEqual(payload["trajectory_session_id"], "TRJ_TEST")
        self.assertIn("nearest_trajectory_distance_m", payload["accepted_regions"][0])

    def test_qwen_prompt_contains_trajectory_rules(self) -> None:
        from src.vlm.qwen_region_selector_core import RegionCandidate, RegionSelectionInput

        inp = RegionSelectionInput(
            snapshot_id="RS_TEST",
            cycle_id=1,
            map_stamp=0.0,
            capture_time="t",
            target_instruction="explore",
            robot_pose={"x": 0, "y": 0, "yaw_deg": 0},
            regions=[
                RegionCandidate(
                    "A", "R1", "LEFT", 1.0, 10, 0.5, 0.4, 0.5, 5,
                    trajectory_novelty_score=0.9,
                )
            ],
            annotated_map_file="/tmp/annotated_map.png",
        )
        prompt = build_region_selection_prompt(inp)
        self.assertIn("TRAVELED PATH", prompt)
        self.assertIn("trajectory_novelty_score", prompt)

    def test_original_map_is_not_mutated(self) -> None:
        import numpy as np

        data = np.zeros((20, 20), dtype=np.int16)
        data[5:10, 5:10] = 100
        original = deep_copy_grid_data(data)
        meta = MapMetadata(
            width=20,
            height=20,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            frame_id="map",
            stamp_sec=0.0,
        )
        overlay = {
            "vertices": [{"x": 0.5, "y": 0.5}, {"x": 1.0, "y": 0.5}],
            "visited_area_data": [100] * (20 * 20),
            "trajectory_session_id": "TRJ_X",
            "trajectory_revision": 1,
        }
        render_annotated_map(
            data,
            meta,
            RobotPose2D(0.2, 0.2, 0.0),
            FrontierAnalysisResult(
                cycle_id=1,
                map_health=MapHealthResult(ok=True, status="OK"),
                stats=FrontierExtractionStats(status="OK"),
            ),
            1,
            CFG,
            overlay,
        )
        self.assertTrue(np.array_equal(data, original))

    def test_no_motion_interfaces_are_defined(self) -> None:
        node_path = PROJECT_ROOT + "/src/planning/frontier_region_debug_node.py"
        with open(node_path, encoding="utf-8") as fh:
            src = fh.read()
        self.assertNotIn("create_publisher(String, '/cmd_vel'", src)
        self.assertNotIn("NavigateToPose", src)
        self.assertNotIn("FollowPath", src)
        store_src = inspect.getsource(
            __import__("src.planning.robot_trajectory_store", fromlist=["x"])
        )
        self.assertNotIn("import rclpy", store_src)


if __name__ == "__main__":
    unittest.main()
