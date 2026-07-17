#!/usr/bin/env python3
"""Progress watchdog unit tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import (  # noqa: E402
    ProgressWatchState,
    progress_timed_out,
    update_progress_watch,
)


class TestProgressWatchdog(unittest.TestCase):
    def test_feedback_only_no_distance_change_times_out(self) -> None:
        st = ProgressWatchState(last_progress_time=0.0)
        st.best_distance_remaining = 2.0
        st.last_progress_time = 100.0
        # Only feedback with same distance — update returns False
        progressed = update_progress_watch(
            st, now=110.0, distance_remaining=2.0, robot_xy=(0.0, 0.0)
        )
        self.assertFalse(progressed)
        self.assertTrue(progress_timed_out(st, now=131.0, timeout_s=30.0))

    def test_distance_drop_refreshes(self) -> None:
        st = ProgressWatchState(last_progress_time=100.0, best_distance_remaining=2.0)
        progressed = update_progress_watch(
            st, now=105.0, distance_remaining=1.85, robot_xy=None
        )
        self.assertTrue(progressed)
        self.assertEqual(st.last_progress_time, 105.0)
        self.assertFalse(progress_timed_out(st, now=120.0, timeout_s=30.0))

    def test_pose_move_refreshes(self) -> None:
        st = ProgressWatchState(
            last_progress_time=100.0,
            best_distance_remaining=2.0,
            last_progress_pose=(0.0, 0.0),
        )
        progressed = update_progress_watch(
            st, now=108.0, distance_remaining=2.0, robot_xy=(0.09, 0.0)
        )
        self.assertTrue(progressed)
        self.assertEqual(st.last_progress_time, 108.0)


if __name__ == "__main__":
    unittest.main()
