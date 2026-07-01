#!/usr/bin/env python3
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.nav.nav_video_overlay import parse_bbox_xyxy


def test_parse_bbox_xyxy():
    assert parse_bbox_xyxy([10, 20, 110, 220]) == (10, 20, 110, 220)
    assert parse_bbox_xyxy([250, 120, 140, 300]) == (250, 120, 390, 420)


if __name__ == "__main__":
    test_parse_bbox_xyxy()
    print("PASS test_capture_navigation_video")
