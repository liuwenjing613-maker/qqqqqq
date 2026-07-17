#!/usr/bin/env python3
"""Tests for circular yaw spread calculation."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "nav"))

from qwen_nav2_common import compute_yaw_spread_deg  # noqa: E402


class TestYawSpread(unittest.TestCase):
    def test_cluster_near_zero(self) -> None:
        yaws = [math.radians(d) for d in (0, 1, -1)]
        self.assertAlmostEqual(compute_yaw_spread_deg(yaws), 2.0, delta=0.2)

    def test_wrap_near_pi(self) -> None:
        yaws = [math.radians(d) for d in (179, -179)]
        self.assertAlmostEqual(compute_yaw_spread_deg(yaws), 2.0, delta=0.2)

    def test_quadrant_spread(self) -> None:
        yaws = [math.radians(d) for d in (0, 90)]
        self.assertAlmostEqual(compute_yaw_spread_deg(yaws), 90.0, delta=0.2)


if __name__ == "__main__":
    unittest.main()
