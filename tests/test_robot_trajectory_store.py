#!/usr/bin/env python3
"""Tests for robot_trajectory_store — created but not executed per task policy."""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.robot_trajectory_store import (  # noqa: E402
    RobotTrajectoryStore,
    TrajectorySession,
    TrajectoryVertex,
    add_pose_sample,
    load_safe,
    point_to_polyline_distance,
    point_to_segment_distance,
    polyline_length,
    save_atomic,
    session_from_dict,
    session_to_dict,
    should_create_vertex,
)


CFG = {
    "trajectory": {
        "enabled": True,
        "sample_period_s": 1.0,
        "tf_lookup_timeout_s": 0.30,
        "tf_warmup_s": 2.0,
        "tf_warn_age_s": 0.50,
        "max_tf_age_s": 1.50,
        "max_clock_future_skew_s": 0.10,
        "require_consecutive_valid_tf": 2,
        "min_vertex_distance_m": 0.05,
        "min_vertex_yaw_change_deg": 10.0,
        "max_vertex_interval_s": 5.0,
        "min_time_vertex_distance_m": 0.01,
        "min_time_vertex_yaw_change_deg": 2.0,
        "max_raw_samples": 20000,
        "max_vertices": 10000,
        "visited_corridor_radius_m": 0.35,
        "recent_path_window_s": 60.0,
        "local_region_analysis_radius_m": 0.80,
        "strong_revisit_distance_m": 0.45,
        "weak_revisit_distance_m": 1.20,
        "persist_across_node_restart": False,
    }
}


def _session() -> TrajectorySession:
    return TrajectorySession(
        trajectory_session_id="TRJ_TEST",
        start_time="2026-07-13T00:00:00+00:00",
        map_frame="map",
    )


class TestVertexCreation(unittest.TestCase):
    def test_first_valid_pose_creates_first_vertex(self) -> None:
        session = _session()
        _, vertex = add_pose_sample(
            session, CFG, stamp_sec=1.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True
        )
        self.assertIsNotNone(vertex)
        self.assertEqual(vertex.creation_reason, "FIRST_VERTEX")
        self.assertEqual(len(session.vertices), 1)

    def test_one_second_samples_are_recorded(self) -> None:
        session = _session()
        for t in range(3):
            add_pose_sample(
                session,
                CFG,
                stamp_sec=float(t),
                x=0.0,
                y=0.0,
                yaw_rad=0.0,
                valid=True,
            )
        self.assertEqual(len(session.raw_samples), 3)

    def test_stationary_robot_does_not_create_duplicate_vertices(self) -> None:
        session = _session()
        for t in range(5):
            add_pose_sample(
                session,
                CFG,
                stamp_sec=float(t),
                x=0.0,
                y=0.0,
                yaw_rad=0.0,
                valid=True,
            )
        self.assertEqual(len(session.raw_samples), 5)
        self.assertEqual(len(session.vertices), 1)

    def test_distance_threshold_creates_vertex(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
        _, v2 = add_pose_sample(
            session, CFG, stamp_sec=1.0, x=0.10, y=0.0, yaw_rad=0.0, valid=True
        )
        self.assertIsNotNone(v2)
        self.assertEqual(v2.creation_reason, "DISTANCE_THRESHOLD")
        self.assertEqual(len(session.vertices), 2)

    def test_yaw_threshold_creates_vertex(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
        _, v2 = add_pose_sample(
            session,
            CFG,
            stamp_sec=1.0,
            x=0.0,
            y=0.0,
            yaw_rad=math.radians(15.0),
            valid=True,
        )
        self.assertIsNotNone(v2)
        self.assertEqual(v2.creation_reason, "YAW_THRESHOLD")

    def test_time_threshold_creates_vertex(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
        _, v2 = add_pose_sample(
            session, CFG, stamp_sec=6.0, x=0.02, y=0.0, yaw_rad=0.0, valid=True
        )
        self.assertIsNotNone(v2)
        self.assertEqual(v2.creation_reason, "TIME_THRESHOLD")

    def test_stationary_robot_does_not_create_time_threshold_vertices(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
        for t in range(1, 61):
            _, vertex = add_pose_sample(
                session,
                CFG,
                stamp_sec=float(t),
                x=0.0,
                y=0.0,
                yaw_rad=0.0,
                valid=True,
            )
            self.assertIsNone(vertex)
        self.assertEqual(len(session.vertices), 1)
        self.assertEqual(session.trajectory_length_m, 0.0)

    def test_tiny_tf_noise_does_not_create_vertex(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
        for t in range(1, 11):
            _, vertex = add_pose_sample(
                session,
                CFG,
                stamp_sec=float(t),
                x=0.0003 * t,
                y=0.00015 * t,
                yaw_rad=math.radians(0.05 * t),
                valid=True,
            )
            self.assertIsNone(vertex)
        self.assertEqual(len(session.vertices), 1)

    def test_time_threshold_requires_small_real_motion(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
        _, v2 = add_pose_sample(
            session, CFG, stamp_sec=6.0, x=0.015, y=0.0, yaw_rad=0.0, valid=True
        )
        self.assertIsNotNone(v2)
        self.assertEqual(v2.creation_reason, "TIME_THRESHOLD")

    def test_time_threshold_requires_small_real_yaw_change(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
        _, v2 = add_pose_sample(
            session,
            CFG,
            stamp_sec=6.0,
            x=0.0,
            y=0.0,
            yaw_rad=math.radians(3.0),
            valid=True,
        )
        self.assertIsNotNone(v2)
        self.assertEqual(v2.creation_reason, "TIME_THRESHOLD")

    def test_raw_samples_still_accumulate_while_stationary(self) -> None:
        session = _session()
        for t in range(60):
            add_pose_sample(
                session, CFG, stamp_sec=float(t), x=0.0, y=0.0, yaw_rad=0.0, valid=True
            )
        self.assertEqual(len(session.raw_samples), 60)
        self.assertEqual(len(session.vertices), 1)

    def test_invalid_tf_sample_does_not_create_vertex(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=0.0, x=0.0, y=0.0, yaw_rad=0.0, valid=True)
        _, v2 = add_pose_sample(
            session,
            CFG,
            stamp_sec=1.0,
            x=0.0,
            y=0.0,
            yaw_rad=0.0,
            valid=False,
            rejection_reason="TRAJECTORY_TF_MISSING",
        )
        self.assertIsNone(v2)
        self.assertEqual(len(session.vertices), 1)


class TestGeometry(unittest.TestCase):
    def test_point_to_segment_distance(self) -> None:
        d = point_to_segment_distance(1.0, 0.1, 0.0, 0.0, 2.0, 0.0)
        self.assertAlmostEqual(d, 0.1, places=3)

    def test_point_to_polyline_distance(self) -> None:
        polyline = [(0.0, 0.0), (2.0, 0.0)]
        d, idx = point_to_polyline_distance(1.0, 0.1, polyline)
        self.assertAlmostEqual(d, 0.1, places=3)
        self.assertEqual(idx, 0)

    def test_polyline_length(self) -> None:
        verts = [
            TrajectoryVertex(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "FIRST_VERTEX"),
            TrajectoryVertex(2, 1.0, 2.0, 0.0, 0.0, 2.0, 0.0, 1.0, "DISTANCE_THRESHOLD"),
        ]
        self.assertAlmostEqual(polyline_length(verts), 2.0, places=3)


class TestPersistence(unittest.TestCase):
    def test_empty_trajectory_is_safe(self) -> None:
        session, diag = load_safe(Path("/tmp/no_such_trajectory_session.json"))
        self.assertIn(diag["load_status"], ("MISSING", "CORRUPT", "OK"))
        self.assertEqual(len(session.vertices), 0)

    def test_atomic_serialization_structure(self) -> None:
        session = _session()
        add_pose_sample(session, CFG, stamp_sec=1.0, x=1.0, y=2.0, yaw_rad=0.5, valid=True)
        payload = session_to_dict(session)
        self.assertIn("vertices", payload)
        self.assertIn("raw_samples", payload)
        restored = session_from_dict(payload)
        self.assertEqual(len(restored.vertices), 1)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trajectory_session.json"
            save_atomic(session, path)
            self.assertTrue(path.is_file())
            raw = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(raw["trajectory_session_id"], "TRJ_TEST")

    def test_reset_clears_only_trajectory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trajectory_session.json"
            store = RobotTrajectoryStore(CFG, path)
            store.ingest_tf_pose(
                stamp_sec=1.0,
                x=1.0,
                y=1.0,
                yaw_rad=0.0,
                tf_stamp_sec=1.0,
                tf_age_s=0.0,
            )
            self.assertGreater(len(store.session.vertices), 0)
            store.reset()
            self.assertEqual(len(store.session.vertices), 0)
            self.assertEqual(len(store.session.raw_samples), 0)


if __name__ == "__main__":
    unittest.main()
