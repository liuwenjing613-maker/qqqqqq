#!/usr/bin/env python3
"""Map wait / metadata gate unit tests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_occupancy_grid_against_yaml  # noqa: E402


class TestMapWait(unittest.TestCase):
    def test_map_not_received_blocks_planning(self) -> None:
        # Empty / missing map is represented as failed validation before ComputePath.
        ok, reason = validate_occupancy_grid_against_yaml(
            frame_id="",
            width=0,
            height=0,
            resolution=0.0,
            data_len=0,
            origin_x=0.0,
            origin_y=0.0,
            yaml_resolution=0.05,
            yaml_origin_x=0.0,
            yaml_origin_y=0.0,
            yaml_width=10,
            yaml_height=10,
        )
        self.assertFalse(ok)

    def test_metadata_mismatch_rejects(self) -> None:
        ok, reason = validate_occupancy_grid_against_yaml(
            frame_id="map",
            width=10,
            height=10,
            resolution=0.05,
            data_len=100,
            origin_x=0.0,
            origin_y=0.0,
            yaml_resolution=0.05,
            yaml_origin_x=0.0,
            yaml_origin_y=0.0,
            yaml_width=20,
            yaml_height=10,
        )
        self.assertFalse(ok)
        self.assertIn("size mismatch", reason)

    def test_map_ok_allows_projection(self) -> None:
        ok, reason = validate_occupancy_grid_against_yaml(
            frame_id="map",
            width=10,
            height=10,
            resolution=0.05,
            data_len=100,
            origin_x=-1.0,
            origin_y=-2.0,
            yaml_resolution=0.05,
            yaml_origin_x=-1.0,
            yaml_origin_y=-2.0,
            yaml_width=10,
            yaml_height=10,
        )
        self.assertTrue(ok, reason)


if __name__ == "__main__":
    unittest.main()
