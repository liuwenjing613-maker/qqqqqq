#!/usr/bin/env python3
import os
import sys
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.mapping.semantic_config import load_semantic_config
from src.mapping.semantic_object_filter import SemanticObjectFilter


def _box(class_name="bottle", score=0.5, x=100, y=100, w=80, h=120):
    return {
        "class_name": class_name,
        "score": score,
        "bbox_xyxy": [x, y, x + w, y + h],
        "cx": x + w / 2,
        "cy": y + h / 2,
        "area_ratio": (w * h) / (640 * 480),
    }


def test_temporal_candidate_and_confirmed():
    cfg = load_semantic_config()
    filt = SemanticObjectFilter(cfg, 640, 480)
    t0 = time.time()
    states = []
    for i in range(3):
        track = filt.update(_box(), _box()["bbox_xyxy"], 140, 160, 0.03, t0 + i * 0.1)
        states.append(track)
    assert states[-1] is not None
    assert states[-1].is_candidate
    assert states[-1].is_confirmed


def test_dynamic_never_confirmed():
    cfg = load_semantic_config()
    filt = SemanticObjectFilter(cfg, 640, 480)
    t0 = time.time()
    track = None
    for i in range(5):
        track = filt.update(_box(class_name="person"), _box()["bbox_xyxy"], 140, 160, 0.03, t0 + i * 0.1)
    assert track is not None
    assert track.is_dynamic
    assert not filt.can_landmark(track)


if __name__ == "__main__":
    test_temporal_candidate_and_confirmed()
    test_dynamic_never_confirmed()
    print("PASS test_semantic_object_filter")
