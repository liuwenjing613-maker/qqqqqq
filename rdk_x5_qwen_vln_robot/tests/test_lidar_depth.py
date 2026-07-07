#!/usr/bin/env python3
import math
import os
import sys

import numpy as np
from sensor_msgs.msg import LaserScan

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.perception.lidar_depth import LidarDepthEstimator, fuse_target_distance, resolve_arrive_distance


def _make_scan(ranges, angle_min=-math.pi, angle_max=math.pi):
    msg = LaserScan()
    msg.angle_min = float(angle_min)
    msg.angle_max = float(angle_max)
    msg.angle_increment = (angle_max - angle_min) / max(len(ranges) - 1, 1)
    msg.ranges = [float(x) for x in ranges]
    return msg


def test_target_distance_uses_min_not_median():
    n = 361
    ranges = [5.0] * n
  # center index ~0 deg
    mid = n // 2
    ranges[mid] = 0.55
    ranges[mid + 1] = 3.0
    est = LidarDepthEstimator(
        min_range=0.08,
        max_range=12.0,
        target_window_deg=8.0,
        camera_lidar_yaw_offset_deg=0.0,
        target_distance_method="min",
    )
    est.update_scan(_make_scan(ranges))
    u = 640
    d_min = est.target_distance_at_u(u, 1280)
    est.target_distance_method = "median"
    d_med = est.target_distance_at_u(u, 1280)
    assert d_min is not None and abs(d_min - 0.55) < 0.01
    assert d_med is not None and d_med > 1.0


def test_fuse_front_when_centered():
    fused = fuse_target_distance(0.58, 0.72, ex=0.05, center_deadband=0.10)
    assert fused is not None and abs(fused - 0.58) < 1e-6
    off = fuse_target_distance(0.58, 0.72, ex=0.20, center_deadband=0.10)
    assert off is not None and abs(off - 0.72) < 1e-6


def test_fuse_min_always():
    fused = fuse_target_distance(0.58, 6.55, ex=0.20, fuse_min_always=True)
    assert fused is not None and abs(fused - 0.58) < 1e-6


def test_resolve_arrive_rejects_front_clutter():
    # step-14 case: front hits floor, bottle ray still ~2m
    arrive = resolve_arrive_distance(0.596, 2.17, arrive_threshold=0.6)
    assert arrive is not None and abs(arrive - 2.17) < 1e-6


def test_resolve_arrive_uses_front_when_ray_invalid():
    arrive = resolve_arrive_distance(0.55, 7.0, arrive_threshold=0.6)
    assert arrive is not None and abs(arrive - 0.55) < 1e-6


def test_resolve_arrive_min_when_both_close():
    arrive = resolve_arrive_distance(0.55, 0.62, arrive_threshold=0.6)
    assert arrive is not None and abs(arrive - 0.55) < 1e-6


if __name__ == "__main__":
    test_target_distance_uses_min_not_median()
    test_fuse_front_when_centered()
    test_fuse_min_always()
    test_resolve_arrive_rejects_front_clutter()
    test_resolve_arrive_uses_front_when_ray_invalid()
    test_resolve_arrive_min_when_both_close()
    print("PASS test_lidar_depth")
