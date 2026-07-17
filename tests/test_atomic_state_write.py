#!/usr/bin/env python3
"""Tests for atomic JSON state writes."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "nav"))

from qwen_nav2_common import atomic_write_json  # noqa: E402


class TestAtomicStateWrite(unittest.TestCase):
    def test_write_creates_valid_json(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "nav2_state.json"
            atomic_write_json(path, {"state": "NAVIGATING", "session_id": "JQS_X"})
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(data["state"], "NAVIGATING")
            self.assertFalse(path.with_suffix(".json.tmp").exists())

    def test_replace_updates_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "nav2_state.json"
            atomic_write_json(path, {"v": 1})
            atomic_write_json(path, {"v": 2})
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["v"], 2)


if __name__ == "__main__":
    unittest.main()
