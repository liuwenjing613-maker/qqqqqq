#!/usr/bin/env python3
"""EXIT ownership gating unit tests."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import (  # noqa: E402
    ownership_protects_sensors,
    write_ownership,
)


class TestExitOwnership(unittest.TestCase):
    def test_before_handoff_cleanup_mapping(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "ownership.json"
            write_ownership(p, "S1", "MAPPING_OWNS_SENSOR")
            self.assertFalse(ownership_protects_sensors(p))

    def test_after_handoff_must_not_kill_sensor(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "ownership.json"
            write_ownership(p, "S1", "NAV_SESSION_OWNS_SENSOR")
            self.assertTrue(ownership_protects_sensors(p))
            write_ownership(p, "S1", "NAV2_OWNS_MOTION")
            self.assertTrue(ownership_protects_sensors(p))

    def test_goal_accepted_exception_path_states(self) -> None:
        # Documented fault path: cancel + zero while sensors retained.
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "ownership.json"
            write_ownership(p, "S1", "NAV2_OWNS_MOTION")
            self.assertTrue(ownership_protects_sensors(p))
            # FINISHED still protects (session teardown must not kill sensor base)
            write_ownership(p, "S1", "FINISHED")
            self.assertTrue(ownership_protects_sensors(p))


if __name__ == "__main__":
    unittest.main()
