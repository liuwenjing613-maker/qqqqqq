#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.nav.lidar_distance import combine_lidar_distances


def test_combine_prefers_closer_reading():
    assert combine_lidar_distances(0.87, 3.82) == 0.87
    assert combine_lidar_distances(3.82, 0.87) == 0.87
    assert combine_lidar_distances(None, 0.62) == 0.62
    assert combine_lidar_distances(0.62, None) == 0.62
    assert combine_lidar_distances(None, None) is None


if __name__ == "__main__":
    test_combine_prefers_closer_reading()
    print("PASS test_lidar_distance")
