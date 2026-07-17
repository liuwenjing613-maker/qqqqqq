#!/usr/bin/env python3
"""Sensor health / repair policy unit tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import (  # noqa: E402
    decide_foxglove_action,
    decide_scan_filter_action,
    decide_static_tf_action,
    sensor_health_overall_pass,
)


class TestSensorHealthPolicy(unittest.TestCase):
    def test_foxglove_false_does_not_block_pass(self) -> None:
        self.assertTrue(
            sensor_health_overall_pass(
                scan_ok=True,
                scan_filtered_ok=True,
                odom_ok=True,
                tf_ok=True,
                chassis_ok=True,
                foxglove_ok=False,
            )
        )

    def test_filter_count_zero_starts(self) -> None:
        action, _ = decide_scan_filter_action(0, False)
        self.assertEqual(action, "start_once")

    def test_filter_count_one_fresh_reuses(self) -> None:
        action, _ = decide_scan_filter_action(1, True)
        self.assertEqual(action, "reuse")

    def test_filter_count_one_stale_fails(self) -> None:
        action, _ = decide_scan_filter_action(1, False)
        self.assertEqual(action, "fail")

    def test_filter_count_multiple_fails(self) -> None:
        action, _ = decide_scan_filter_action(2, True)
        self.assertEqual(action, "fail")

    def test_static_tf_exists_reuses(self) -> None:
        action, _ = decide_static_tf_action(tf_exists=True, owner_pid=1, owner_alive=True)
        self.assertEqual(action, "reuse")

    def test_static_tf_missing_owner_alive_fails(self) -> None:
        action, _ = decide_static_tf_action(tf_exists=False, owner_pid=9, owner_alive=True)
        self.assertEqual(action, "fail")

    def test_foxglove_port_listening_reuses(self) -> None:
        action, _ = decide_foxglove_action(port_listening=True, bridge_count=1)
        self.assertEqual(action, "reuse")

    def test_foxglove_no_bridge_starts(self) -> None:
        action, _ = decide_foxglove_action(port_listening=False, bridge_count=0)
        self.assertEqual(action, "start_once")


if __name__ == "__main__":
    unittest.main()
