#!/usr/bin/env python3
"""Handoff session ack validation tests (compat suite)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_handoff_ack  # noqa: E402


def _ident(role: str, pid: int) -> dict:
    scripts = {
        "lidar": ("", "ydlidar_ros2_driver_node --ros-args"),
        "scan_filter": ("simple_scan_filter.py", "python3 simple_scan_filter.py"),
        "chassis": ("m1_pwm_cmd_vel_bridge.py", "python3 m1_pwm_cmd_vel_bridge.py"),
        "static_tf": ("", "static_transform_publisher --frame-id base_link"),
    }
    base, cmd = scripts[role]
    return {
        "role": role,
        "pid": pid,
        "start_ticks": "7",
        "exe": "/usr/bin/python3",
        "script_basename": base,
        "cmdline": cmd,
    }


class TestHandoffSession(unittest.TestCase):
    def _base(self):
        req = {"session_id": "JQS_A", "requested_epoch": 100.0}
        ack = {
            "session_id": "JQS_A",
            "state": "SENSOR_BASE_HELD",
            "completed_epoch": 101.0,
            "processes": {
                "lidar": _ident("lidar", 1),
                "scan_filter": _ident("scan_filter", 2),
                "chassis": _ident("chassis", 3),
            },
            "stopped": {"joy": True, "teleop": True, "slam": True, "frontier": True},
            "static_tf_present_via_tf": True,
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

    def test_pid_identity_mismatch_rejected(self) -> None:
        req, ack = self._base()
        ack["processes"]["lidar"]["cmdline"] = "python other_thing"
        ack["processes"]["lidar"]["script_basename"] = "other.py"
        with mock.patch("qwen_nav2_common.pid_alive", return_value=True), mock.patch(
            "qwen_nav2_common.read_proc_start_ticks", return_value=7
        ):
            ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("identity", reason)

    def test_valid_ack_passes(self) -> None:
        req, ack = self._base()
        with mock.patch("qwen_nav2_common.pid_alive", return_value=True), mock.patch(
            "qwen_nav2_common.read_proc_start_ticks", return_value=7
        ):
            ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertTrue(ok, reason)


if __name__ == "__main__":
    unittest.main()
