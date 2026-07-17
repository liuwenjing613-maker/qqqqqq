#!/usr/bin/env python3
"""Nav2 owner PID reuse / kill safety tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_nav2_owner  # noqa: E402


class TestNav2Owner(unittest.TestCase):
    def test_pid_reuse_refuses_kill(self) -> None:
        owner = {
            "session_id": "JQS_A",
            "launch_pid": 42,
            "launch_pgid": 42,
            "start_ticks": 1000,
            "cmdline": "ros2 launch nav2_click_nav_bringup_launch.py",
        }
        with mock.patch("qwen_nav2_common.pid_alive", return_value=True), mock.patch(
            "qwen_nav2_common.read_proc_start_ticks", return_value=9999
        ), mock.patch(
            "qwen_nav2_common.read_proc_cmdline",
            return_value="ros2 launch nav2_click_nav_bringup_launch.py",
        ):
            ok, reason = validate_nav2_owner(owner, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("PID reuse", reason)

    def test_matching_owner_ok(self) -> None:
        owner = {
            "session_id": "JQS_A",
            "launch_pid": 42,
            "launch_pgid": 42,
            "start_ticks": 1000,
            "cmdline": "ros2 launch nav2_click_nav_bringup_launch.py",
        }
        with mock.patch("qwen_nav2_common.pid_alive", return_value=True), mock.patch(
            "qwen_nav2_common.read_proc_start_ticks", return_value=1000
        ), mock.patch(
            "qwen_nav2_common.read_proc_cmdline",
            return_value="ros2 launch configs/nav2_click_nav_bringup_launch.py",
        ):
            ok, reason = validate_nav2_owner(owner, expected_session_id="JQS_A")
        self.assertTrue(ok, reason)


if __name__ == "__main__":
    unittest.main()
