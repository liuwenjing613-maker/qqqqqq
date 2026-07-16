#!/usr/bin/env python3
"""Unit tests for trajectory TF validation logic."""

from __future__ import annotations

import os
import sys
import unittest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.robot_trajectory_store import RobotTrajectoryStore  # noqa: E402
from src.planning.trajectory_tf_validation import (  # noqa: E402
    TF_AGE_WARNING,
    TF_OK,
    TRAJECTORY_TF_CLOCK_SKEW,
    TRAJECTORY_TF_LOOKUP_FAILED,
    TRAJECTORY_TF_STALE,
    TRAJECTORY_TF_WARMING_UP,
    TrajectoryTfTracker,
    compute_tf_age_s,
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
        "persist_across_node_restart": False,
    }
}


class TestTrajectoryTfValidation(unittest.TestCase):
    def setUp(self) -> None:
        self.tracker = TrajectoryTfTracker(CFG)

    def test_latest_available_tf_query_is_used(self) -> None:
        """Age uses transform stamp from lookup result, not wall clock subtraction."""
        now_ns = 10_000_000_000
        stamp_sec = 9
        stamp_nanosec = 700_000_000
        age = compute_tf_age_s(now_ns, stamp_sec, stamp_nanosec)
        self.assertAlmostEqual(age, 0.3, places=6)

    def test_ros_clock_is_used_for_tf_age(self) -> None:
        now_ns = 5_000_000_000
        result = self.tracker.evaluate_transform(
            now_ns=now_ns,
            node_uptime_s=5.0,
            stamp_sec=4,
            stamp_nanosec=600_000_000,
            x=1.0,
            y=2.0,
            yaw_rad=0.1,
        )
        self.assertAlmostEqual(result.tf_age_s, 0.4, places=6)
        self.assertTrue(result.valid)

    def test_tf_warmup_does_not_count_as_failure(self) -> None:
        result = self.tracker.evaluate_lookup_failure(
            node_uptime_s=0.5,
            exception_text="frame does not exist",
        )
        self.assertEqual(result.status, TRAJECTORY_TF_WARMING_UP)
        self.assertEqual(self.tracker.consecutive_invalid_count, 0)

        fail = self.tracker.evaluate_lookup_failure(
            node_uptime_s=3.0,
            exception_text="frame does not exist",
        )
        self.assertEqual(fail.status, TRAJECTORY_TF_LOOKUP_FAILED)
        self.assertEqual(self.tracker.consecutive_invalid_count, 1)

    def test_tf_age_warning_is_accepted(self) -> None:
        now_ns = 2_000_000_000
        result = self.tracker.evaluate_transform(
            now_ns=now_ns,
            node_uptime_s=5.0,
            stamp_sec=1,
            stamp_nanosec=200_000_000,
            x=0.0,
            y=0.0,
            yaw_rad=0.0,
        )
        self.assertEqual(result.status, TF_AGE_WARNING)
        self.assertTrue(result.valid)
        self.assertTrue(result.warning)

    def test_tf_over_hard_age_is_rejected(self) -> None:
        now_ns = 3_000_000_000
        result = self.tracker.evaluate_transform(
            now_ns=now_ns,
            node_uptime_s=5.0,
            stamp_sec=0,
            stamp_nanosec=0,
            x=0.0,
            y=0.0,
            yaw_rad=0.0,
        )
        self.assertEqual(result.status, TRAJECTORY_TF_STALE)
        self.assertFalse(result.valid)

    def test_small_future_clock_skew_is_clamped(self) -> None:
        now_ns = 1_000_000_000
        result = self.tracker.evaluate_transform(
            now_ns=now_ns,
            node_uptime_s=5.0,
            stamp_sec=1,
            stamp_nanosec=50_000_000,
            x=0.0,
            y=0.0,
            yaw_rad=0.0,
        )
        self.assertTrue(result.valid)
        self.assertEqual(result.tf_age_s, 0.0)

    def test_large_future_clock_skew_is_rejected(self) -> None:
        now_ns = 1_000_000_000
        result = self.tracker.evaluate_transform(
            now_ns=now_ns,
            node_uptime_s=5.0,
            stamp_sec=1,
            stamp_nanosec=200_000_000,
            x=0.0,
            y=0.0,
            yaw_rad=0.0,
        )
        self.assertEqual(result.status, TRAJECTORY_TF_CLOCK_SKEW)
        self.assertFalse(result.valid)

    def test_consecutive_valid_tf_sets_ready(self) -> None:
        now_ns = 2_000_000_000
        for _ in range(2):
            result = self.tracker.evaluate_transform(
                now_ns=now_ns,
                node_uptime_s=5.0,
                stamp_sec=1,
                stamp_nanosec=900_000_000,
                x=0.0,
                y=0.0,
                yaw_rad=0.0,
            )
            self.assertTrue(result.valid)
        self.assertTrue(self.tracker.trajectory_tf_ready)
        self.assertEqual(self.tracker.consecutive_valid_count, 2)

    def test_invalid_tf_clears_ready_after_configured_failures(self) -> None:
        now_ns = 2_000_000_000
        for _ in range(2):
            self.tracker.evaluate_transform(
                now_ns=now_ns,
                node_uptime_s=5.0,
                stamp_sec=1,
                stamp_nanosec=900_000_000,
                x=0.0,
                y=0.0,
                yaw_rad=0.0,
            )
        self.assertTrue(self.tracker.trajectory_tf_ready)
        self.tracker.evaluate_transform(
            now_ns=5_000_000_000,
            node_uptime_s=5.0,
            stamp_sec=0,
            stamp_nanosec=0,
            x=0.0,
            y=0.0,
            yaw_rad=0.0,
        )
        self.assertFalse(self.tracker.trajectory_tf_ready)
        self.assertEqual(self.tracker.consecutive_valid_count, 0)

    def test_odom_fallback_is_not_written_to_map_trajectory(self) -> None:
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as tmp:
            store = RobotTrajectoryStore(CFG, Path(tmp) / "trajectory_session.json", map_frame="map")
            self.tracker.odom_tf_available_for_diagnostics = True
            sample = store.ingest_tf_rejected(
                stamp_sec=1.0,
                rejection_reason=TRAJECTORY_TF_LOOKUP_FAILED,
                x=0.91,
                y=-0.10,
                yaw_rad=0.2,
            )
            self.assertFalse(sample.valid)
            self.assertEqual(len(store.session.vertices), 0)
            self.assertEqual(store.session.raw_samples[-1].x, 0.91)
            self.assertEqual(store.session.raw_samples[-1].rejection_reason, TRAJECTORY_TF_LOOKUP_FAILED)

    def test_tf_exception_text_is_preserved(self) -> None:
        exc = (
            "Lookup would require extrapolation into the future.  "
            "Latest transform was 0.123456s old when TF buffer was created"
        )
        result = self.tracker.evaluate_lookup_failure(
            node_uptime_s=5.0,
            exception_text=exc,
        )
        self.assertEqual(result.status, TRAJECTORY_TF_LOOKUP_FAILED)
        self.assertIn("extrapolation into the future", result.exception_text.lower())
        self.assertEqual(self.tracker.latest_tf_exception, exc)

    def test_valid_sample_status_ok_below_warn_threshold(self) -> None:
        now_ns = 1_500_000_000
        result = self.tracker.evaluate_transform(
            now_ns=now_ns,
            node_uptime_s=5.0,
            stamp_sec=1,
            stamp_nanosec=100_000_000,
            x=0.0,
            y=0.0,
            yaw_rad=0.0,
        )
        self.assertEqual(result.status, TF_OK)
        self.assertTrue(result.valid)


if __name__ == "__main__":
    unittest.main()
