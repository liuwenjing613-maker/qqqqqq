#!/usr/bin/env python3
"""Handoff ack identity validation tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_handoff_ack  # noqa: E402


def _ident(role: str, pid: int = 1000) -> dict:
    return {
        "role": role,
        "pid": pid,
        "start_ticks": "55",
        "exe": "/usr/bin/python3.10",
        "script_basename": {
            "lidar": "ydlidar_ros2_driver_node",
            "scan_filter": "simple_scan_filter.py",
            "chassis": "m1_pwm_cmd_vel_bridge.py",
            "static_tf": "static_transform_publisher",
        }.get(role, ""),
        "cmdline": {
            "lidar": "ydlidar_ros2_driver_node --ros-args",
            "scan_filter": "python3 simple_scan_filter.py",
            "chassis": "python3 m1_pwm_cmd_vel_bridge.py",
            "static_tf": "static_transform_publisher --frame-id base_link",
        }.get(role, role),
    }


class TestHandoffAck(unittest.TestCase):
    def _base(self):
        req = {"session_id": "JQS_A", "requested_epoch": 100.0}
        ack = {
            "session_id": "JQS_A",
            "state": "SENSOR_BASE_HELD",
            "completed_epoch": 101.0,
            "processes": {
                "lidar": _ident("lidar", 11),
                "scan_filter": _ident("scan_filter", 22),
                "chassis": _ident("chassis", 33),
                "static_tf": _ident("static_tf", 44),
            },
            "stopped": {"joy": True, "teleop": True, "slam": True, "frontier": True},
            "static_tf_present_via_tf": True,
        }
        return req, ack

    def _patch_alive(self):
        return mock.patch("qwen_nav2_common.pid_alive", return_value=True), mock.patch(
            "qwen_nav2_common.read_proc_start_ticks", return_value=55
        )

    def test_normal_identity_passes(self) -> None:
        req, ack = self._base()
        with self._patch_alive()[0], self._patch_alive()[1]:
            ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertTrue(ok, reason)

    def test_pid_reuse_rejected(self) -> None:
        req, ack = self._base()
        with mock.patch("qwen_nav2_common.pid_alive", return_value=True), mock.patch(
            "qwen_nav2_common.read_proc_start_ticks", return_value=999
        ):
            ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("PID reuse", reason)

    def test_wrong_session_rejected(self) -> None:
        req, ack = self._base()
        ack["session_id"] = "OTHER"
        with self._patch_alive()[0], self._patch_alive()[1]:
            ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("session_id", reason)

    def test_old_ack_rejected(self) -> None:
        req, ack = self._base()
        ack["completed_epoch"] = 99.0
        with self._patch_alive()[0], self._patch_alive()[1]:
            ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("completed_epoch", reason)

    def test_legacy_json_cannot_synthesize_formal_ack(self) -> None:
        req = {"session_id": "JQS_A", "requested_epoch": 100.0}
        ack = {
            "session_id": "JQS_A",
            "state": "SENSOR_BASE_HELD",
            "completed_epoch": 101.0,
            "lidar_pid": 1,
            "scan_filter_pid": 2,
            "chassis_pid": 3,
            "stopped": {"joy": True, "teleop": True, "slam": True, "frontier": True},
            "source": "legacy_sensor_base_stack",
        }
        ok, reason = validate_handoff_ack(req, ack, expected_session_id="JQS_A")
        self.assertFalse(ok)
        self.assertIn("legacy", reason.lower())


if __name__ == "__main__":
    unittest.main()
