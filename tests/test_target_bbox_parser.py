#!/usr/bin/env python3
import json
import os
import sys
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.perception.target_bbox_parser import TargetBBoxParser, TargetBBoxParserConfig


def make_parser(**kwargs):
    defaults = dict(
        target_words=["green bottle", "bottle", "cup"],
        target_min_score=0.006,
        accept_unknown_class=True,
    )
    defaults.update(kwargs)
    cfg = TargetBBoxParserConfig(**defaults)
    return TargetBBoxParser(cfg)


def test_direct_bbox():
    p = make_parser()
    msg = {"visible": True, "bbox": [250, 120, 390, 420], "score": 0.8, "class": "bottle"}
    out = p.update_json(json.dumps(msg), 640, 480)
    assert out["visible"], out
    assert abs(out["u"] - 320) < 1, out


def test_target_bbox():
    p = make_parser()
    msg = {"target_visible": True, "target_bbox": [100, 100, 200, 300], "confidence": 0.5}
    out = p.update_json(json.dumps(msg), 640, 480)
    assert out["visible"], out
    assert abs(out["u"] - 150) < 1, out


def test_target_box():
    p = make_parser()
    msg = {"target_box": {"bbox": [100, 100, 200, 300], "score": 0.5, "class": "bottle"}}
    out = p.update_json(json.dumps(msg), 640, 480)
    assert out["visible"], out


def test_boxes_list():
    p = make_parser()
    msg = {
        "boxes": [
            {"bbox": [10, 10, 20, 20], "score": 0.5, "class": "chair"},
            {"bbox": [250, 120, 390, 420], "score": 0.02, "class": "bottle"},
        ]
    }
    out = p.update_json(json.dumps(msg), 640, 480)
    assert out["visible"], out
    assert out["class_name"] == "bottle", out


def test_low_score():
    p = make_parser()
    msg = {"bbox": [250, 120, 390, 420], "score": 0.001, "class": "bottle"}
    out = p.update_json(json.dumps(msg), 640, 480)
    assert not out["visible"], out


def test_xywh_bbox():
    p = make_parser()
    msg = {"bbox": [250, 120, 140, 300], "score": 0.8, "class": "bottle"}
    out = p.update_json(json.dumps(msg), 640, 480)
    assert out["visible"], out
    assert abs(out["u"] - 320) < 1, out
    assert abs(out["v"] - 270) < 1, out


def test_cx_cy_only():
    p = make_parser()
    msg = {"cx": 400, "cy": 240, "score": 0.5, "class": "bottle"}
    out = p.update_json(json.dumps(msg), 640, 480)
    assert out["visible"], out
    assert abs(out["u"] - 400) < 1, out


def test_live_preview_json():
    p = make_parser(target_min_score=0.002)
    msg = {
        "visible": True,
        "class_name": "bottle",
        "score": 0.018,
        "bbox": [200, 100, 120, 280],
        "cx": 260,
        "cy": 240,
        "area_ratio": 0.08,
    }
    out = p.update_json(json.dumps(msg), 640, 480)
    assert out["visible"], out
    assert out["class_name"] == "bottle"


def test_target_timeout():
    p = make_parser(target_lost_timeout_sec=0.05, target_memory_sec=0.1)
    msg = {"bbox": [250, 120, 390, 420], "score": 0.8, "class": "bottle"}
    p.update_json(json.dumps(msg), 640, 480)
    time.sleep(0.15)
    out = p.get_target(time.time(), 640, 480)
    assert not out["visible"]
    assert out["reason"] == "target_timeout"


if __name__ == "__main__":
    test_direct_bbox()
    test_target_bbox()
    test_target_box()
    test_boxes_list()
    test_low_score()
    test_xywh_bbox()
    test_cx_cy_only()
    test_live_preview_json()
    test_target_timeout()
    print("PASS test_target_bbox_parser")
