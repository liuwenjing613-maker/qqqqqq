#!/usr/bin/env python3
import math
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.mapping.semantic_projection import (
    estimate_lidar_range_median,
    pixel_to_bearing_rad,
    project_object_xy,
)


def test_pixel_to_bearing_center():
    b = pixel_to_bearing_rad(320, 640, 70.0, 0.0)
    assert abs(b) < 1e-6


def test_pixel_to_bearing_offset():
    b = pixel_to_bearing_rad(640, 640, 70.0, 0.0)
    assert abs(b - math.radians(35.0)) < 1e-3


def test_project_object_xy():
    ox, oy = project_object_xy(0.0, 0.0, 0.0, 0.0, 1.0)
    assert abs(ox - 1.0) < 1e-6 and abs(oy) < 1e-6


def test_lidar_median_range():
    scan = {
        "angle_min": -math.pi,
        "angle_increment": math.pi / 4.0,
        "ranges": [0.05, 0.5, 1.0, 1.2, 0.4, 0.3, 0.2, 0.1, 0.05],
    }
    r = estimate_lidar_range_median(scan, 0.0, target_window_deg=45.0)
    assert r is not None
    assert 0.4 <= r <= 1.2


def test_lidar_no_valid_range():
    scan = {"angle_min": 0.0, "angle_increment": 0.1, "ranges": [float("inf"), float("nan")]}
    assert estimate_lidar_range_median(scan, 0.0) is None


if __name__ == "__main__":
    test_pixel_to_bearing_center()
    test_pixel_to_bearing_offset()
    test_project_object_xy()
    test_lidar_median_range()
    test_lidar_no_valid_range()
    print("PASS test_semantic_projection")
