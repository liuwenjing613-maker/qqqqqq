#!/usr/bin/env python3
"""Effective goal projection usage tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_planned_path  # noqa: E402


def _pose(x, y):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))


class TestGoalProjection(unittest.TestCase):
    def test_path_endpoint_uses_effective_not_raw(self) -> None:
        raw_x, raw_y = 1.0, 0.0
        effective_x, effective_y = 1.15, 0.0  # projected 0.15m
        poses = [_pose(0.0, 0.0), _pose(0.5, 0.0), _pose(1.12, 0.0)]
        ok, reason = validate_planned_path(
            poses,
            robot_x=0.0,
            robot_y=0.0,
            goal_x=effective_x,
            goal_y=effective_y,
            max_path_m=10.0,
            goal_tol_m=0.35,
        )
        self.assertTrue(ok, reason)
        poses2 = [_pose(0.0, 0.0), _pose(1.15, 0.0)]
        ok_raw, reason_raw = validate_planned_path(
            poses2,
            robot_x=0.0,
            robot_y=0.0,
            goal_x=raw_x,
            goal_y=raw_y,
            max_path_m=10.0,
            goal_tol_m=0.10,
        )
        self.assertFalse(ok_raw)
        ok_eff, reason_eff = validate_planned_path(
            poses2,
            robot_x=0.0,
            robot_y=0.0,
            goal_x=effective_x,
            goal_y=effective_y,
            max_path_m=10.0,
            goal_tol_m=0.10,
        )
        self.assertTrue(ok_eff, reason_eff)


if __name__ == "__main__":
    unittest.main()
