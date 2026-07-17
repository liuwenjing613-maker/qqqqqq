#!/usr/bin/env python3
"""Tests for map goal cell validation / projection."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np
from nav_msgs.msg import OccupancyGrid

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "nav"))

from qwen_nav2_common import validate_goal_on_map  # noqa: E402


def _grid(width: int, height: int, res: float = 0.05) -> OccupancyGrid:
    g = OccupancyGrid()
    g.header.frame_id = "map"
    g.info.width = width
    g.info.height = height
    g.info.resolution = res
    g.info.origin.position.x = 0.0
    g.info.origin.position.y = 0.0
    g.data = [0] * (width * height)
    return g


def _write_pgm(path: Path, width: int, height: int, fill: int = 254) -> None:
    import struct

    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + bytes([fill]) * (width * height))


class TestMapGoalCellValidation(unittest.TestCase):
    def test_free_goal_passes(self) -> None:
        grid = _grid(40, 40)
        map_yaml = PROJECT_ROOT / "tests" / "_fixture_map.yaml"
        pgm = PROJECT_ROOT / "tests" / "_fixture_map.pgm"
        map_yaml.write_text("image: _fixture_map.pgm\nresolution: 0.05\norigin: [0,0,0]\n", encoding="utf-8")
        _write_pgm(pgm, 40, 40)
        gx, gy = 0.5, 0.5
        x, y, reason = validate_goal_on_map(grid, gx, gy, 0.2, 0.2, map_yaml=map_yaml)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(x, gx, places=2)

    def test_occupied_center_fails_without_nearby_free(self) -> None:
        grid = _grid(20, 20)
        for idx in range(20 * 20):
            grid.data[idx] = 100  # all occupied
        map_yaml = PROJECT_ROOT / "tests" / "_fixture_map2.yaml"
        pgm = PROJECT_ROOT / "tests" / "_fixture_map2.pgm"
        map_yaml.write_text("image: _fixture_map2.pgm\nresolution: 0.05\norigin: [0,0,0]\n", encoding="utf-8")
        _write_pgm(pgm, 20, 20, fill=0)
        gx = 0.5
        gy = 0.5
        with self.assertRaises(ValueError):
            validate_goal_on_map(grid, gx, gy, 0.0, 0.0, map_yaml=map_yaml)


if __name__ == "__main__":
    unittest.main()
