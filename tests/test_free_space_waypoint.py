#!/usr/bin/env python3
import math
import os
import sys
from types import SimpleNamespace

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.perception.free_space_waypoint import FreeSpaceConfig, FreeSpaceWaypointProvider


def make_scan(default=1.5, overrides=None):
    n = 361
    angle_min = math.radians(-180)
    angle_increment = math.radians(1)
    ranges = [default] * n

    if overrides:
        for deg_min, deg_max, val in overrides:
            for deg in range(deg_min, deg_max + 1):
                idx = deg + 180
                if 0 <= idx < n:
                    ranges[idx] = val

    return SimpleNamespace(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
    )


def test_center_open():
    cfg = FreeSpaceConfig(camera_hfov_deg=70.0)
    p = FreeSpaceWaypointProvider(cfg)
    p.update_scan(make_scan(default=1.5))
    wp = p.get_waypoint(640, 480)
    assert wp["usable"]
    assert abs(wp["u"] - 320) < 80, wp


def test_left_open():
    cfg = FreeSpaceConfig(camera_hfov_deg=70.0)
    p = FreeSpaceWaypointProvider(cfg)
    scan = make_scan(default=0.25, overrides=[(-60, -20, 1.5)])
    p.update_scan(scan)
    wp = p.get_waypoint(640, 480)
    assert wp["usable"]
    assert wp["u"] < 320, wp


def test_right_open():
    cfg = FreeSpaceConfig(camera_hfov_deg=70.0)
    p = FreeSpaceWaypointProvider(cfg)
    scan = make_scan(default=0.25, overrides=[(20, 60, 1.5)])
    p.update_scan(scan)
    wp = p.get_waypoint(640, 480)
    assert wp["usable"]
    assert wp["u"] > 320, wp


def test_blocked():
    cfg = FreeSpaceConfig(camera_hfov_deg=70.0, min_clearance=0.45)
    p = FreeSpaceWaypointProvider(cfg)
    p.update_scan(make_scan(default=0.25))
    wp = p.get_waypoint(640, 480)
    assert not wp["usable"], wp


def test_target_distance_at_u_with_yaw_offset():
    cfg = FreeSpaceConfig(camera_hfov_deg=70.0, camera_lidar_yaw_offset_deg=90.0, target_window_deg=8.0)
    p = FreeSpaceWaypointProvider(cfg)
    # Bottle-like close return at camera-forward (+90° on scan index space used by make_scan)
    scan = make_scan(default=2.0, overrides=[(85, 95, 0.48)])
    p.update_scan(scan)
    u_center = 640.0
    u_right = 733.5
    front_min = p.front_min_distance()
    target_center = p.target_distance_at_u(u_center, 1280)
    target_right = p.target_distance_at_u(u_right, 1280)
    assert front_min is not None and front_min <= 0.55
    assert target_center is not None and target_center <= 0.55
    assert target_right is not None and target_right <= 0.55


def test_front_distance_is_front_min():
    cfg = FreeSpaceConfig(camera_hfov_deg=70.0, lidar_front_deg=20.0)
    p = FreeSpaceWaypointProvider(cfg)
    scan = make_scan(default=1.5, overrides=[(-2, 2, 0.42)])
    p.update_scan(scan)
    assert p.front_distance() == p.front_min_distance()
    wp = p.get_waypoint(640, 480)
    assert wp.get("front_distance") == p.front_min_distance()


if __name__ == "__main__":
    test_center_open()
    test_left_open()
    test_right_open()
    test_blocked()
    test_target_distance_at_u_with_yaw_offset()
    test_front_distance_is_front_min()
    print("PASS test_free_space_waypoint")
