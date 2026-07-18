#!/usr/bin/env python3
"""Sensor uniqueness / overall PASS policy tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import (  # noqa: E402
    decide_scan_filter_action,
    sensor_health_overall_pass_v2,
)


class TestSensorUniqueness(unittest.TestCase):
    def test_filter_count_matrix(self) -> None:
        self.assertEqual(decide_scan_filter_action(0, False)[0], "start_once")
        self.assertEqual(decide_scan_filter_action(1, True)[0], "reuse")
        self.assertEqual(decide_scan_filter_action(1, False)[0], "fail")
        self.assertEqual(decide_scan_filter_action(2, True)[0], "fail")

    def test_repair_requires_count_one_again(self) -> None:
        # After repair, overall PASS requires counts == 1
        self.assertFalse(
            sensor_health_overall_pass_v2(
                scan_ok=True,
                scan_filtered_ok=True,
                odom_ok=True,
                tf_ok=True,
                chassis_ok=True,
                lidar_count=1,
                scan_filter_count=0,
                chassis_count=1,
            )
        )
        self.assertTrue(
            sensor_health_overall_pass_v2(
                scan_ok=True,
                scan_filtered_ok=True,
                odom_ok=True,
                tf_ok=True,
                chassis_ok=True,
                lidar_count=1,
                scan_filter_count=1,
                chassis_count=1,
            )
        )

    def test_stale_single_must_not_stack(self) -> None:
        action, reason = decide_scan_filter_action(1, False)
        self.assertEqual(action, "fail")
        self.assertIn("stale", reason)

    def test_foxglove_false_does_not_block_overall(self) -> None:
        self.assertTrue(
            sensor_health_overall_pass_v2(
                scan_ok=True,
                scan_filtered_ok=True,
                odom_ok=True,
                tf_ok=True,
                chassis_ok=True,
                lidar_count=1,
                scan_filter_count=1,
                chassis_count=1,
                foxglove_ok=False,
            )
        )


if __name__ == "__main__":
    unittest.main()
