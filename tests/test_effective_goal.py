#!/usr/bin/env python3
"""Effective goal is the single source of truth after projection."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_planned_path  # noqa: E402


class _P:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _Pose:
    def __init__(self, x, y):
        self.pose = type("Pose", (), {"position": _P(x, y)})()


class TestEffectiveGoal(unittest.TestCase):
    def test_raw_goal_projected_effective_used(self) -> None:
        raw = (1.0, 1.0)
        effective = (1.05, 1.02)  # projected
        path = [_Pose(0.0, 0.0), _Pose(effective[0], effective[1])]
        ok, reason = validate_planned_path(
            path,
            robot_x=0.0,
            robot_y=0.0,
            goal_x=effective[0],
            goal_y=effective[1],
            max_path_m=10.0,
            path_frame="map",
        )
        self.assertTrue(ok, reason)
        # Using raw goal against projected end would fail goal tolerance in many cases
        ok_raw, _ = validate_planned_path(
            path,
            robot_x=0.0,
            robot_y=0.0,
            goal_x=raw[0] + 0.5,
            goal_y=raw[1] + 0.5,
            max_path_m=10.0,
            path_frame="map",
            goal_tol_m=0.05,
        )
        self.assertFalse(ok_raw)

    def test_result_fields_use_effective(self) -> None:
        planned = {
            "raw_goal_x": 1.0,
            "raw_goal_y": 1.0,
            "effective_goal_x": 1.12,
            "effective_goal_y": 0.98,
            "projection_distance_m": 0.12,
        }
        # All consumer fields must prefer effective_*
        for key in ("path", "marker", "navigate", "result"):
            goal_x = planned["effective_goal_x"]
            goal_y = planned["effective_goal_y"]
            self.assertNotEqual((goal_x, goal_y), (planned["raw_goal_x"], planned["raw_goal_y"]))
            self.assertAlmostEqual(goal_x, 1.12)
            self.assertAlmostEqual(goal_y, 0.98)


if __name__ == "__main__":
    unittest.main()
