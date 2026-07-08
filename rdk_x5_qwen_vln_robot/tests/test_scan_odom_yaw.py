import math
import sys
import os
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.apps.run_qwen_api_lidar_nav import (
    PHASE_TARGET_SCAN_360,
    RunQwenApiLidarNav,
)


def test_normalize_yaw_delta_wraps():
    prev = math.radians(350.0)
    current = math.radians(10.0)
    delta = RunQwenApiLidarNav._normalize_yaw_delta(prev, current)
    assert abs(delta - math.radians(20.0)) < 1e-6


def test_sync_scan_odom_yaw_accumulates_full_turn():
    node = object.__new__(RunQwenApiLidarNav)
    node.full_scan_use_odom_yaw = True
    node.phase = PHASE_TARGET_SCAN_360
    node.scan_yaw_integrated = 0.0
    node._scan_odom_last_yaw = None
    node._scan_odom_fallback_time = None
    node.full_scan_wz = 0.02
    node.chassis_max_wz = 0.02
    node.full_scan_odom_stale_sec = 0.5
    node.latest_odom_yaw = 0.0
    node.latest_odom_time = time.time()

    node._sync_scan_odom_yaw("ASK_QWEN_PATH")
    assert node._scan_odom_last_yaw == 0.0
    assert node.scan_yaw_integrated == 0.0

    node.latest_odom_yaw = math.pi
    node._sync_scan_odom_yaw(PHASE_TARGET_SCAN_360)
    assert abs(node.scan_yaw_integrated - math.pi) < 1e-6

    node.latest_odom_yaw = -math.pi + 0.1
    node._sync_scan_odom_yaw(PHASE_TARGET_SCAN_360)
    assert node.scan_yaw_integrated > math.pi
