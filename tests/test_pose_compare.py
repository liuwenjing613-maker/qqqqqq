#!/usr/bin/env python3
"""Pose compare / odom hard-gate unit tests."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import evaluate_handoff_pose_gates  # noqa: E402


class TestPoseCompare(unittest.TestCase):
    def test_odom_still_map_small_ok(self) -> None:
        frozen_map = (0.0, 0.0, 0.0)
        live_map = (0.05, 0.02, math.radians(3.0))
        frozen_odom = (1.0, 2.0, 0.1)
        live_odom = (1.01, 2.01, 0.11)
        r = evaluate_handoff_pose_gates(
            frozen_map=frozen_map,
            live_map=live_map,
            frozen_odom=frozen_odom,
            live_odom=live_odom,
        )
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["odom_status"], "PASS")
        self.assertEqual(r["map_status"], "PASS")

    def test_odom_move_rejects(self) -> None:
        r = evaluate_handoff_pose_gates(
            frozen_map=(0.0, 0.0, 0.0),
            live_map=(0.0, 0.0, 0.0),
            frozen_odom=(0.0, 0.0, 0.0),
            live_odom=(0.5, 0.0, 0.0),
        )
        self.assertFalse(r["ok"])
        self.assertEqual(r["hard_fail"], "PHYSICAL_MOVE_DETECTED")

    def test_map_severe_jump_rejects(self) -> None:
        r = evaluate_handoff_pose_gates(
            frozen_map=(0.0, 0.0, 0.0),
            live_map=(1.0, 0.0, 0.0),
            frozen_odom=(0.0, 0.0, 0.0),
            live_odom=(0.01, 0.0, 0.0),
        )
        self.assertFalse(r["ok"])
        self.assertEqual(r["map_status"], "FAIL")
        self.assertEqual(r["hard_fail"], "MAP_POSE_JUMP")


if __name__ == "__main__":
    unittest.main()
