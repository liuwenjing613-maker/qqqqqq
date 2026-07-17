#!/usr/bin/env python3
"""Tests for Qwen Nav2 phase state machine ordering."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "nav"))

from qwen_nav2_common import NavPhase, TERMINAL_PHASES, can_advance_phase  # noqa: E402


class TestQwenNavStateMachine(unittest.TestCase):
    def test_happy_path_advances(self) -> None:
        cur = NavPhase.TARGET_RECEIVED
        nxt = NavPhase.CONTROL_STOPPED
        self.assertTrue(can_advance_phase(cur, nxt))

    def test_skip_phase_blocked(self) -> None:
        self.assertFalse(can_advance_phase(NavPhase.TARGET_RECEIVED, NavPhase.NAVIGATING))

    def test_terminal_blocks(self) -> None:
        for phase in TERMINAL_PHASES:
            self.assertFalse(can_advance_phase(phase, NavPhase.NAVIGATING))

    def test_full_chain(self) -> None:
        order = [
            NavPhase.TARGET_RECEIVED,
            NavPhase.CONTROL_STOPPED,
            NavPhase.MAPPING_HANDOFF_COMPLETE,
            NavPhase.SENSOR_BASE_VERIFIED,
            NavPhase.NAV2_OVERLAY_STARTED,
            NavPhase.LOCALIZATION_ACTIVE,
            NavPhase.LOCALIZATION_SETTLED,
            NavPhase.PATH_VALIDATED,
            NavPhase.GOAL_ACCEPTED,
            NavPhase.NAVIGATING,
        ]
        for cur, nxt in zip(order, order[1:]):
            self.assertTrue(can_advance_phase(cur, nxt), f"{cur} -> {nxt}")


if __name__ == "__main__":
    unittest.main()
