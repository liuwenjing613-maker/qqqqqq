#!/usr/bin/env python3
"""Unit tests for exploration data contracts (Phase 3A.5). Not executed in code-only phase."""

from __future__ import annotations

import ast
import copy
import json
import os
import sys
import unittest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.exploration_contracts import (  # noqa: E402
    EXPLORATION_CONTRACT_VERSION,
    build_map_data_fingerprint,
    build_map_fingerprint,
    build_map_metadata_fingerprint,
    build_region_geometry_fingerprint,
    deep_copy_map_data,
    validate_contract_config,
    verify_map_data_unchanged,
)

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


def _default_contract_cfg() -> dict:
    return {
        "contracts": {"exploration_contract_version": EXPLORATION_CONTRACT_VERSION},
        "fingerprints": {
            "float_precision_digits": 6,
            "sort_region_cells": True,
            "include_map_stamp_in_map_fingerprint": True,
        },
        "safety": {
            "allow_motion": False,
            "allow_cmd_vel": False,
            "allow_nav2": False,
            "allow_goal_publish": False,
        },
    }


def _sample_map_data(length: int = 100) -> list:
    return [-1 if i % 3 else 0 for i in range(length)]


class TestMapFingerprint(unittest.TestCase):
    def test_same_map_produces_same_fingerprint(self) -> None:
        data = _sample_map_data()
        cfg = _default_contract_cfg()
        a = build_map_fingerprint(
            frame_id="map",
            width=10,
            height=10,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            map_data=data,
            map_stamp=100.0,
            cfg=cfg,
        )
        b = build_map_fingerprint(
            frame_id="map",
            width=10,
            height=10,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            map_data=data,
            map_stamp=100.0,
            cfg=cfg,
        )
        self.assertEqual(a, b)

    def test_changed_map_data_changes_fingerprint(self) -> None:
        cfg = _default_contract_cfg()
        data_a = _sample_map_data()
        data_b = list(data_a)
        data_b[0] = 100
        fp_a = build_map_data_fingerprint(data_a)
        fp_b = build_map_data_fingerprint(data_b)
        self.assertNotEqual(fp_a, fp_b)

    def test_changed_map_origin_changes_fingerprint(self) -> None:
        data = _sample_map_data()
        cfg = _default_contract_cfg()
        a = build_map_metadata_fingerprint(
            frame_id="map",
            width=10,
            height=10,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            map_stamp=100.0,
            cfg=cfg,
        )
        b = build_map_metadata_fingerprint(
            frame_id="map",
            width=10,
            height=10,
            resolution=0.05,
            origin_x=1.0,
            origin_y=0.0,
            origin_yaw=0.0,
            map_stamp=100.0,
            cfg=cfg,
        )
        self.assertNotEqual(a, b)

    def test_map_fingerprint_is_deterministic(self) -> None:
        data = _sample_map_data()
        cfg = _default_contract_cfg()
        fps = [
            build_map_fingerprint(
                frame_id="map",
                width=10,
                height=10,
                resolution=0.05,
                origin_x=0.0,
                origin_y=0.0,
                origin_yaw=0.0,
                map_data=data,
                map_stamp=100.0,
                cfg=cfg,
            )["map_fingerprint"]
            for _ in range(3)
        ]
        self.assertEqual(len(set(fps)), 1)

    def test_original_map_not_mutated(self) -> None:
        data = _sample_map_data()
        before = deep_copy_map_data(data)
        build_map_fingerprint(
            frame_id="map",
            width=10,
            height=10,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            map_data=data,
            map_stamp=100.0,
        )
        self.assertTrue(verify_map_data_unchanged(before, data))


class TestRegionGeometryFingerprint(unittest.TestCase):
    def test_same_region_geometry_produces_same_fingerprint(self) -> None:
        cfg = _default_contract_cfg()
        kwargs = dict(
            snapshot_id="RS_TEST",
            region_label="B",
            internal_region_id="R1",
            track_id="T1",
            frontier_cells_grid=[[1, 5], [2, 5]],
            frontier_points_map=[[0.27, 0.07], [0.27, 0.12]],
            centroid_x=0.2,
            centroid_y=0.2,
            bbox_grid=[1, 2, 5, 5],
            cfg=cfg,
        )
        self.assertEqual(
            build_region_geometry_fingerprint(**kwargs),
            build_region_geometry_fingerprint(**kwargs),
        )

    def test_changed_frontier_cell_changes_region_fingerprint(self) -> None:
        cfg = _default_contract_cfg()
        base = dict(
            snapshot_id="RS_TEST",
            region_label="B",
            internal_region_id="R1",
            track_id="T1",
            frontier_points_map=[[0.27, 0.07], [0.27, 0.12]],
            centroid_x=0.2,
            centroid_y=0.2,
            bbox_grid=[1, 2, 5, 5],
            cfg=cfg,
        )
        a = build_region_geometry_fingerprint(
            frontier_cells_grid=[[1, 5], [2, 5]], **base
        )
        b = build_region_geometry_fingerprint(
            frontier_cells_grid=[[1, 5], [2, 6]], **base
        )
        self.assertNotEqual(a, b)

    def test_region_cell_order_does_not_change_fingerprint(self) -> None:
        cfg = _default_contract_cfg()
        cells_a = [[2, 5], [1, 5]]
        cells_b = [[1, 5], [2, 5]]
        pts_a = [[0.27, 0.12], [0.27, 0.07]]
        pts_b = [[0.27, 0.07], [0.27, 0.12]]
        fp_a = build_region_geometry_fingerprint(
            snapshot_id="RS",
            region_label="B",
            internal_region_id="R1",
            track_id="T1",
            frontier_cells_grid=cells_a,
            frontier_points_map=pts_a,
            centroid_x=0.2,
            centroid_y=0.2,
            bbox_grid=[1, 2, 5, 5],
            cfg=cfg,
        )
        fp_b = build_region_geometry_fingerprint(
            snapshot_id="RS",
            region_label="B",
            internal_region_id="R1",
            track_id="T1",
            frontier_cells_grid=cells_b,
            frontier_points_map=pts_b,
            centroid_x=0.2,
            centroid_y=0.2,
            bbox_grid=[1, 2, 5, 5],
            cfg=cfg,
        )
        self.assertEqual(fp_a, fp_b)


class TestContractConfig(unittest.TestCase):
    def test_contract_config_rejects_unsafe_motion(self) -> None:
        cfg = _default_contract_cfg()
        cfg["safety"]["allow_nav2"] = True
        errors = validate_contract_config(cfg)
        self.assertTrue(any("allow_nav2" in e for e in errors))


class TestModuleSafety(unittest.TestCase):
    def test_no_ros_dependencies(self) -> None:
        path = os.path.join(PROJECT_ROOT, "src", "planning", "exploration_contracts.py")
        with open(path, encoding="utf-8") as fh:
            source = fh.read()
        self.assertNotIn("import rclpy", source)
        tree = ast.parse(source)
        modules = {
            node.module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        }
        self.assertNotIn("rclpy", modules)


if __name__ == "__main__":
    unittest.main()
