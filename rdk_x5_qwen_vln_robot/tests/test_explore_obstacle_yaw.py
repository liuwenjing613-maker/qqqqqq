#!/usr/bin/env python3
import math
import os
import sys
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.apps.run_qwen_api_lidar_nav import (
    PHASE_ASK_QWEN_PATH,
    PHASE_EXPLORE_FORWARD,
    PHASE_PATH_YAW_STEP,
    PHASE_TARGET_SCAN_360,
    PHASE_TARGET_SERVO,
    RunQwenApiLidarNav,
    _EXPLORE_TARGET_WATCH_PHASES,
)


def _stub_node(**overrides):
    node = object.__new__(RunQwenApiLidarNav)
    node.require_lidar = False
    node.explore_vx = 0.06
    node.explore_distance_m = 1.0
    node.explore_max_sec = 40.0
    node.explore_hard_stop_distance = 0.45
    node.obstacle_yaw_step_deg = 90.0
    node.path_yaw_step_deg = 90.0
    node.path_yaw_wz = 0.06
    node.path_yaw_target_rad = math.radians(90.0)
    node.path_yaw_integrated = 0.0
    node.path_yaw_start_time = None
    node.path_point = (640.0, 360.0)
    node.path_confidence = 0.9
    node.image_width = 1280
    node.path_turn_threshold = 0.16
    node.path_kp_turn = 0.02
    node.full_scan_wz = 0.045
    node.phase = PHASE_EXPLORE_FORWARD
    node.explore_phase_reason = ""
    node.explore_start_time = time.time()
    node.explore_end_time = time.time() + 20.0
    node._scan_odom_last_yaw = None
    node._scan_odom_fallback_time = None
    node.stopped = False

    def publish_stop():
        node.stopped = True

    node.publish_stop = publish_stop
    node._set_motion_cmd = lambda cmd, reason, clear_angle=False: None
    node._publish_explore_state = lambda cmd: None
    node._scan_is_fresh = lambda: True
    node.get_logger = lambda: type("L", (), {"info": lambda *a, **k: None})()
    node._start_path_yaw_step = RunQwenApiLidarNav._start_path_yaw_step.__get__(node)
    fd = overrides.get("front_distance", 2.0)
    node.lidar = type("L", (), {"front_distance": lambda self: fd})()
    for key, val in overrides.items():
        setattr(node, key, val)
    return node


def test_obstacle_triggers_path_yaw_not_full_scan():
    node = _stub_node(front_distance=0.40)
    RunQwenApiLidarNav._explore_motion_tick(node, time.time())
    assert node.stopped is True
    assert node.phase == PHASE_PATH_YAW_STEP
    assert node.path_point is None
    assert node.explore_phase_reason == "obstacle_yaw_step_start"
    assert node.phase != PHASE_TARGET_SCAN_360


def test_explore_target_watch_phases_include_forward_and_align():
    assert PHASE_EXPLORE_FORWARD in _EXPLORE_TARGET_WATCH_PHASES
    assert PHASE_PATH_YAW_STEP in _EXPLORE_TARGET_WATCH_PHASES


def test_resolve_infer_mode_target_during_explore_forward():
    node = object.__new__(RunQwenApiLidarNav)
    node.explore_enable = True
    node.phase = PHASE_EXPLORE_FORWARD
    node.qwen_default_mode = "track"
    assert RunQwenApiLidarNav._resolve_infer_mode(node) == "target"


def test_resolve_infer_mode_path_only_when_asking_path():
    node = object.__new__(RunQwenApiLidarNav)
    node.explore_enable = True
    node.phase = PHASE_ASK_QWEN_PATH
    assert RunQwenApiLidarNav._resolve_infer_mode(node) == "path"


if __name__ == "__main__":
    test_obstacle_triggers_path_yaw_not_full_scan()
    test_explore_target_watch_phases_include_forward_and_align()
    test_resolve_infer_mode_target_during_explore_forward()
    test_resolve_infer_mode_path_only_when_asking_path()
    print("ok")
