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


if __name__ == "__main__":
    test_center_open()
    test_left_open()
    test_right_open()
    test_blocked()
    print("PASS test_free_space_waypoint")
