#!/usr/bin/env python3
import os
import math
import sys
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.apps.run_qwen_api_lidar_nav import (
    PHASE_ASK_QWEN_PATH,
    PHASE_PATH_YAW_STEP,
    PHASE_TARGET_SCAN_360,
    RunQwenApiLidarNav,
)


def test_no_safe_path_starts_path_yaw_not_full_scan():
    node = object.__new__(RunQwenApiLidarNav)
    node.path_confidence_threshold = 0.0
    node.explore_phase_reason = ""
    node.path_point = None
    node.path_confidence = 0.0
    node.phase = PHASE_ASK_QWEN_PATH
    node.path_yaw_step_deg = 90.0
    node.path_yaw_wz = 0.06
    node.path_yaw_target_rad = math.radians(90.0)
    node.path_yaw_integrated = 0.0
    node.path_yaw_start_time = None
    node._scan_odom_last_yaw = None
    node._scan_odom_fallback_time = None
    node.get_logger = lambda: type("L", (), {"info": lambda *a, **k: None, "warn": lambda *a, **k: None})()

    def publish_stop():
        node.phase_stopped = True

    node.publish_stop = publish_stop
    node._start_path_yaw_step = RunQwenApiLidarNav._start_path_yaw_step.__get__(node)

    RunQwenApiLidarNav._start_path_yaw_step(node)

    assert node.phase == PHASE_PATH_YAW_STEP
    assert node.phase != PHASE_TARGET_SCAN_360


def test_path_yaw_complete_advances_to_ask_path():
    node = object.__new__(RunQwenApiLidarNav)
    node.path_yaw_target_rad = math.radians(90.0)
    node.path_yaw_integrated = math.radians(91.0)
    node.path_yaw_start_time = time.time()
    node.path_yaw_max_sec = 20.0
    node.get_logger = lambda: type("L", (), {"warn": lambda *a, **k: None})()

    assert RunQwenApiLidarNav._path_yaw_turn_complete(node, time.time()) is True


if __name__ == "__main__":
    test_no_safe_path_starts_path_yaw_not_full_scan()
    test_path_yaw_complete_advances_to_ask_path()
    print("ok")
