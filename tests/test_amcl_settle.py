#!/usr/bin/env python3
"""AMCL settle evaluation tests."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import (  # noqa: E402
    compute_yaw_spread_deg,
    evaluate_amcl_settle_window,
)


class TestAmclSettle(unittest.TestCase):
    def test_wrap_yaw_spread(self) -> None:
        yaws = [math.radians(179), math.radians(-179)]
        self.assertAlmostEqual(compute_yaw_spread_deg(yaws), 2.0, delta=0.2)

    def test_recent_window_pass(self) -> None:
        samples = [
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "x_cov": 0.1, "y_cov": 0.1, "yaw_cov": 0.05}
            for _ in range(8)
        ]
        ok, reasons, metrics = evaluate_amcl_settle_window(
            samples, scan_age_s=0.2, odom_age_s=0.1, map_tf_ok=True
        )
        self.assertTrue(ok, reasons)
        self.assertEqual(metrics["sample_count"], 8)

    def test_fail_reasons_actionable(self) -> None:
        samples = [
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "x_cov": 0.5, "y_cov": 0.1, "yaw_cov": 0.05}
        ] * 8
        ok, reasons, _ = evaluate_amcl_settle_window(
            samples, scan_age_s=2.0, odom_age_s=0.1, map_tf_ok=False
        )
        self.assertFalse(ok)
        joined = " ".join(reasons)
        self.assertIn("x_cov", joined)
        self.assertIn("scan_age", joined)
        self.assertIn("map_base_link", joined)


if __name__ == "__main__":
    unittest.main()
