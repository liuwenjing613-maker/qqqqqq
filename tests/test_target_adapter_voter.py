#!/usr/bin/env python3
import json
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.config.nav_voter import load_voter_config
from src.perception.target_adapter import TargetAdapter


def _bbox_msg(score=0.8):
    return json.dumps(
        {
            "visible": True,
            "bbox": [250, 120, 390, 420],
            "score": score,
            "class": "bottle",
        }
    )


def test_load_voter_config_enabled():
    cfg = {
        "voter": {
            "enabled": True,
            "window_size": 6,
            "min_votes": 2,
            "lost_hold_frames": 4,
        }
    }
    voter = load_voter_config(cfg)
    assert voter["enabled"] is True
    assert voter["window_size"] == 6
    assert voter["min_votes"] == 2
    assert voter["lost_hold_frames"] == 4


def test_voter_requires_min_votes_before_visible():
    adapter = TargetAdapter(
        image_width=640,
        image_height=480,
        target_words=["bottle"],
        min_score=0.01,
        voter_enabled=True,
        voter_window_size=3,
        voter_min_votes=2,
        voter_lost_hold_frames=2,
    )
    adapter.ingest_yolo_bbox_json(_bbox_msg())
    first = adapter.materialize_yolo_target(now=1.0)
    assert not first.visible
    assert first.reason == "waiting_multiframe_votes"

    adapter.ingest_yolo_bbox_json(_bbox_msg())
    second = adapter.materialize_yolo_target(now=1.1)
    assert second.visible, second
    assert second.vote_reason in ("confirmed_by_multiframe_votes", "latest_voted_candidate")


def test_voter_holds_last_target_on_missing_frames():
    adapter = TargetAdapter(
        image_width=640,
        image_height=480,
        target_words=["bottle"],
        min_score=0.01,
        voter_enabled=True,
        voter_window_size=3,
        voter_min_votes=2,
        voter_lost_hold_frames=2,
    )
    adapter.ingest_yolo_bbox_json(_bbox_msg())
    adapter.materialize_yolo_target(now=1.0)
    adapter.materialize_yolo_target(now=1.05)
    adapter.ingest_yolo_bbox_json(_bbox_msg())
    confirmed = adapter.materialize_yolo_target(now=1.1)
    assert confirmed.visible

    held = adapter.materialize_yolo_target(now=1.2)
    assert held.visible, held
    assert held.vote_reason == "hold_last_target"


def test_voter_disabled_passes_through_single_detection():
    adapter = TargetAdapter(
        image_width=640,
        image_height=480,
        target_words=["bottle"],
        min_score=0.01,
        voter_enabled=False,
    )
    target = adapter.update_yolo_bbox_json(_bbox_msg())
    assert target.visible


if __name__ == "__main__":
    test_load_voter_config_enabled()
    test_voter_requires_min_votes_before_visible()
    test_voter_holds_last_target_on_missing_frames()
    test_voter_disabled_passes_through_single_detection()
    print("PASS test_target_adapter_voter")
