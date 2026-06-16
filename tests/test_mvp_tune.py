#!/usr/bin/env python3
"""configs/mvp_tune.yaml 与 load_mvp_tune 一致性测试。"""

import os
import sys
import tempfile
import unittest

import yaml

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.config.mvp_tune import (
    DEFAULT_TUNE_PATH,
    load_mvp_tune,
    shell_export,
)


class TestMvpTune(unittest.TestCase):
    def test_default_config_loads(self):
        tune = load_mvp_tune(DEFAULT_TUNE_PATH)
        self.assertTrue(os.path.isfile(tune["config_path"]))
        self.assertGreater(tune["yolo_min_score"], 0.0)
        self.assertGreater(tune["max_vx"], 0.0)

    def test_yolo_min_score_synced_to_node_and_mvp(self):
        tune = load_mvp_tune(DEFAULT_TUNE_PATH)
        self.assertEqual(tune["score_threshold"], tune["yolo_min_score"])
        self.assertEqual(tune["min_score"], tune["yolo_min_score"])

    def test_min_drive_vx_derived_from_max_vx(self):
        tune = load_mvp_tune(DEFAULT_TUNE_PATH)
        with open(DEFAULT_TUNE_PATH, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        ratio = float(raw.get("min_drive_vx_ratio", 0.9))
        floor = float(raw.get("min_drive_vx_floor", 0.03))
        expected = max(floor, tune["max_vx"] * ratio)
        self.assertAlmostEqual(tune["min_drive_vx"], expected)

    def test_shell_export_contains_stability_keys(self):
        tune = load_mvp_tune(DEFAULT_TUNE_PATH)
        text = shell_export(tune)
        for key in (
            "ENABLE_KICK_START",
            "CMD_SMOOTH_ALPHA",
            "RECOVERY_SCAN_WZ",
            "TURN_THRESHOLD",
            "FORWARD_THRESHOLD",
        ):
            self.assertIn(f"export {key}=", text)

    def test_stability_defaults_from_yaml(self):
        tune = load_mvp_tune(DEFAULT_TUNE_PATH)
        self.assertTrue(tune["enable_kick_start"])
        self.assertGreaterEqual(tune["kick_wz"], 0.24)
        self.assertLessEqual(tune["recovery_scan_wz"], tune["max_wz"])

    def test_shell_export_contains_yolo_and_chassis_keys(self):
        tune = load_mvp_tune(DEFAULT_TUNE_PATH)
        text = shell_export(tune)
        for key in (
            "SCORE_THRESHOLD",
            "MIN_SCORE",
            "MAX_VX",
            "MIN_DRIVE_VX",
            "CHASSIS_PORT",
            "LOST_FRAMES_LIMIT",
        ):
            self.assertIn(f"export {key}=", text)

    def test_custom_yaml_override(self):
        payload = {
            "yolo_min_score": 0.005,
            "max_vx": 0.04,
            "max_wz": 0.12,
            "kp_turn": 0.09,
            "center_threshold": 0.25,
            "arrive_area_ratio": 0.18,
            "stable_frames_required": 4,
            "lost_frames_limit": 12,
            "max_area_ratio": 0.12,
            "det_stale_sec": 0.8,
            "min_red_ratio": 0.05,
            "chassis_port": "/dev/ttyUSB1",
            "chassis_max_vx": 0.08,
            "chassis_max_wz": 0.18,
            "kick_vx": 0.10,
            "kick_wz": 0.24,
            "kick_duration": 0.20,
            "min_drive_vx_ratio": 0.8,
            "min_drive_vx_floor": 0.02,
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            yaml.dump(payload, f)
            path = f.name
        try:
            tune = load_mvp_tune(path)
            self.assertEqual(tune["yolo_min_score"], 0.005)
            self.assertEqual(tune["min_score"], 0.005)
            self.assertEqual(tune["score_threshold"], 0.005)
            self.assertEqual(tune["max_vx"], 0.04)
            self.assertEqual(tune["lost_frames_limit"], 12)
            self.assertAlmostEqual(tune["min_drive_vx"], max(0.02, 0.04 * 0.8))
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
