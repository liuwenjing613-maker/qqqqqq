#!/usr/bin/env python3
import json
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.perception.target_adapter import TargetAdapter


def test_yolo_bbox_xywh_to_point():
    adapter = TargetAdapter(image_width=640, image_height=480, target_words=["bottle"], min_score=0.01)
    target = adapter.update_yolo_bbox_json(
        json.dumps({"visible": True, "bbox": [250, 120, 140, 300], "score": 0.8, "class": "bottle"})
    )
    assert target.visible, target
    assert abs(target.u - 320.0) < 1.0
    assert abs(target.v - 270.0) < 1.0
    assert target.source == "yolo_bbox"


def test_yolo_bbox_xyxy_to_point():
    adapter = TargetAdapter(image_width=640, image_height=480, target_words=["bottle"], min_score=0.01)
    target = adapter.update_yolo_bbox_json(
        json.dumps({"visible": True, "bbox": [250, 120, 390, 420], "score": 0.8, "class": "bottle"})
    )
    assert target.visible, target
    assert abs(target.u - 320.0) < 1.0
    assert abs(target.v - 270.0) < 1.0


def test_stale_yolo_target_is_not_visible():
    adapter = TargetAdapter(image_width=640, image_height=480, target_words=["bottle"], min_score=0.01)
    target = adapter.update_yolo_bbox_json(
        json.dumps(
            {
                "visible": True,
                "bbox": [250, 120, 390, 420],
                "score": 0.8,
                "class": "bottle",
                "stale": True,
            }
        )
    )
    assert target.stale
    assert not target.visible
    current = adapter.current_yolo_target()
    assert current.stale
    assert not current.visible


def test_qwen_point():
    adapter = TargetAdapter()
    target = adapter.from_qwen_result({"target_point": [123, 234], "score": 0.7})
    assert target.visible
    assert target.u == 123.0
    assert target.v == 234.0
    assert target.source == "qwen_point"


if __name__ == "__main__":
    test_yolo_bbox_xywh_to_point()
    test_yolo_bbox_xyxy_to_point()
    test_stale_yolo_target_is_not_visible()
    test_qwen_point()
    print("PASS test_target_adapter")
