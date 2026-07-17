#!/usr/bin/env python3
"""Tests for planned path validation."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_planned_path  # noqa: E402


def _pose(x: float, y: float):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))


class TestPathValidation(unittest.TestCase):
    def test_empty_path_rejected(self) -> None:
        ok, reason = validate_planned_path([], robot_x=0, robot_y=0, goal_x=1, goal_y=0, max_path_m=5)
        self.assertFalse(ok)
        self.assertIn("empty", reason)

    def test_nan_rejected(self) -> None:
        poses = [_pose(0, 0), _pose(float("nan"), 0)]
        ok, reason = validate_planned_path(
            poses, robot_x=0, robot_y=0, goal_x=1, goal_y=0, max_path_m=5
        )
        self.assertFalse(ok)
        self.assertIn("NaN", reason)

    def test_start_error_rejected(self) -> None:
        poses = [_pose(2, 0), _pose(1, 0)]
        ok, reason = validate_planned_path(
            poses, robot_x=0, robot_y=0, goal_x=1, goal_y=0, max_path_m=5, start_tol_m=0.5
        )
        self.assertFalse(ok)
        self.assertIn("start", reason)

    def test_goal_error_rejected(self) -> None:
        poses = [_pose(0, 0), _pose(0.5, 0)]
        ok, reason = validate_planned_path(
            poses, robot_x=0, robot_y=0, goal_x=1, goal_y=0, max_path_m=5, goal_tol_m=0.35
        )
        self.assertFalse(ok)
        self.assertIn("goal", reason)

    def test_too_long_rejected(self) -> None:
        poses = [_pose(0, 0), _pose(10, 0)]
        ok, reason = validate_planned_path(
            poses, robot_x=0, robot_y=0, goal_x=10, goal_y=0, max_path_m=5
        )
        self.assertFalse(ok)
        self.assertIn("long", reason)

    def test_valid_path_passes(self) -> None:
        poses = [_pose(0.05, 0), _pose(0.5, 0), _pose(0.95, 0)]
        ok, reason = validate_planned_path(
            poses,
            robot_x=0,
            robot_y=0,
            goal_x=1,
            goal_y=0,
            max_path_m=5,
            min_path_m=0.15,
        )
        self.assertTrue(ok, reason)


if __name__ == "__main__":
    unittest.main()
