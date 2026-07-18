#!/usr/bin/env python3
"""Nav2 overlay preflight unit tests."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

import nav2_overlay_preflight as preflight  # noqa: E402
from qwen_nav2_common import atomic_write_json  # noqa: E402


class TestNav2Preflight(unittest.TestCase):
    def test_no_residual_pass(self) -> None:
        with mock.patch.object(preflight, "list_nav2_procs", return_value=[]):
            report = preflight.inspect(None)
        self.assertEqual(report["nav2_proc_count"], 0)
        self.assertEqual(report["clearable_pgids"], [])

    def test_unowned_nav2_inspect(self) -> None:
        proc = preflight.ProcInfo(
            pid=100,
            pgid=200,
            exe="/opt/ros/humble/lib/nav2_amcl/amcl",
            cmdline="amcl --ros-args",
            start_ticks=111,
            role="amcl",
        )
        with mock.patch.object(preflight, "list_nav2_procs", return_value=[proc]), mock.patch.object(
            preflight, "pgid_has_protected", return_value=(False, [])
        ):
            report = preflight.inspect(None)
        self.assertEqual(report["nav2_proc_count"], 1)
        self.assertIn(200, report["clearable_pgids"])

    def test_pgid_with_sensor_blocks_clear(self) -> None:
        proc = preflight.ProcInfo(
            pid=100,
            pgid=200,
            exe="/usr/bin/amcl",
            cmdline="amcl",
            start_ticks=1,
            role="amcl",
        )
        with mock.patch.object(preflight, "list_nav2_procs", return_value=[proc]), mock.patch.object(
            preflight, "pgid_has_protected", return_value=(True, ["ydlidar"])
        ):
            report = preflight.clear_unowned(None)
        self.assertEqual(report["status"], "REFUSED_PROTECTED")

    def test_stale_owner_auto_archive(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            runtime = Path(td)
            owner = runtime / "nav2_owner.json"
            atomic_write_json(
                owner,
                {
                    "session_id": "S1",
                    "launch_pid": 999999,
                    "start_ticks": 1,
                    "cmdline": "nav2_click_nav_bringup",
                },
            )
            with mock.patch("qwen_nav2_common.pid_alive", return_value=False):
                stale = preflight.archive_stale_owner(owner, runtime)
            self.assertIsNotNone(stale)
            self.assertFalse(owner.is_file())

    def test_start_ticks_change_refuses_kill(self) -> None:
        from qwen_nav2_common import validate_nav2_owner

        owner = {
            "session_id": "S1",
            "launch_pid": 1234,
            "start_ticks": 10,
            "cmdline": "nav2_click_nav_bringup_launch.py",
        }
        with mock.patch("qwen_nav2_common.pid_alive", return_value=True), mock.patch(
            "qwen_nav2_common.read_proc_start_ticks", return_value=99
        ), mock.patch("qwen_nav2_common.read_proc_cmdline", return_value="nav2_click_nav_bringup_launch.py"):
            ok, reason = validate_nav2_owner(owner, expected_session_id="S1")
        self.assertFalse(ok)
        self.assertIn("start_ticks", reason)


if __name__ == "__main__":
    unittest.main()
