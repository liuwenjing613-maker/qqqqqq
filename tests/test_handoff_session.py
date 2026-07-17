#!/usr/bin/env python3
"""Handoff session ack validation tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_handoff_ack  # noqa: E402


class TestHandoffSession(unittest.TestCase):
    def _base(self):
        req = {"session_id": "JQS_A", "requested_epoch": 100.0, "expected_cmdlines": {}}
        ack = {
            "session_id": "JQS_A",
            "state": "SENSOR_BASE_HELD",
            "completed_epoch": 101.0,
            "lidar_pid": None,
            "scan_filter_pid": None,
            "chassis_pid": None,
            "static_tf_pid": None,
            "foxglove_pid": None,
        }
        return req, ack

    def test_wrong_session_rejected(self) -> None:
        req, ack = self._base()
        ack["session_id"] = "OTHER"
        ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("session_id", reason)

    def test_old_timestamp_rejected(self) -> None:
        req, ack = self._base()
        ack["completed_epoch"] = 99.0
        ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("completed_epoch", reason)

    def test_pid_cmdline_mismatch_rejected(self) -> None:
        req, ack = self._base()
        req["expected_cmdlines"] = {"lidar_pid": "ydlidar"}
        ack["lidar_pid"] = 12345
        with mock.patch("qwen_nav2_common.pid_alive", return_value=True), mock.patch(
            "qwen_nav2_common.read_proc_cmdline", return_value="python other_thing"
        ):
            ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("cmdline", reason)

    def test_valid_ack_passes(self) -> None:
        req, ack = self._base()
        ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertTrue(ok, reason)


if __name__ == "__main__":
    unittest.main()
