#!/usr/bin/env python3
"""Tests for region_history_store."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.region_history_store import ObservationPose, RegionHistoryStore  # noqa: E402


class TestRegionHistoryStore(unittest.TestCase):
    def test_history_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "region_history.json"
            store = RegionHistoryStore()
            store.add_observation_pose(
                ObservationPose("OP1", 1.0, 2.0, 90.0, "t", "RS_1", "T001", 100),
                max_poses=50,
            )
            store.save_atomic(path)
            loaded = RegionHistoryStore.load(path)
            self.assertEqual(len(loaded.observation_poses), 1)

    def test_atomic_history_write(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "region_history.json"
            store = RegionHistoryStore()
            store.save_atomic(path)
            self.assertTrue(path.is_file())
            self.assertFalse(path.with_suffix(path.suffix + ".tmp").exists())

    def test_blacklist_after_failures(self) -> None:
        store = RegionHistoryStore()
        store.update_track_stats("T001", navigation_failure=True, blacklist_after=3)
        store.update_track_stats("T001", navigation_failure=True, blacklist_after=3)
        store.update_track_stats("T001", navigation_failure=True, blacklist_after=3)
        self.assertTrue(store.track_stats["T001"]["blacklisted"])

    def test_history_corruption_safe_fallback(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "region_history.json"
            path.write_text("{not json", encoding="utf-8")
            loaded = RegionHistoryStore.load(path)
            self.assertEqual(len(loaded.observation_poses), 0)


if __name__ == "__main__":
    unittest.main()
