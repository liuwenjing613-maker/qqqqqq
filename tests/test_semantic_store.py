#!/usr/bin/env python3
import json
import os
import sys
import tempfile
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.mapping.semantic_config import load_semantic_config
from src.mapping.semantic_projection import RobotPose
from src.mapping.semantic_store import SemanticStore
from src.mapping.semantic_types import SemanticObservation


def test_landmark_merge_and_save():
    cfg = load_semantic_config()
    with tempfile.TemporaryDirectory() as tmp:
        cfg["storage"]["root_dir"] = tmp
        cfg["storage"]["session_name"] = "test_session"
        store = SemanticStore(cfg, map_name="test_map")

        obs1 = SemanticObservation(
            obs_id="obs_001",
            stamp=time.time(),
            fixed_frame="map",
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            class_name="bottle",
            score=0.8,
            bbox_xyxy=[10, 10, 50, 90],
            u=30,
            v=50,
            area_ratio=0.01,
            bearing_rad=0.0,
            range_m=1.0,
            object_x=1.0,
            object_y=0.1,
            position_sigma=0.3,
            source="test",
            quality="confirmed_input",
            range_source="lidar_median",
        )
        store.add_observation(obs1)
        lm1 = store.fuse_landmark(obs1, small_objects={"bottle"}, large_objects=set())
        assert lm1 is not None

        obs2 = SemanticObservation(
            obs_id="obs_002",
            stamp=time.time() + 0.5,
            fixed_frame="map",
            robot_x=0.1,
            robot_y=0.0,
            robot_yaw=0.0,
            class_name="bottle",
            score=0.85,
            bbox_xyxy=[10, 10, 50, 90],
            u=30,
            v=50,
            area_ratio=0.01,
            bearing_rad=0.0,
            range_m=1.0,
            object_x=1.05,
            object_y=0.12,
            position_sigma=0.3,
            source="test",
            quality="confirmed_input",
            range_source="lidar_median",
        )
        lm2 = store.fuse_landmark(obs2, small_objects={"bottle"}, large_objects=set())
        assert lm2.landmark_id == lm1.landmark_id
        assert lm2.seen_count >= 2

        path = store.save_all(final=True)
        assert os.path.isfile(path)
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        assert data["map_name"] == "test_map"
        assert len(data["landmarks"]) >= 1


def test_loop_quality():
    cfg = load_semantic_config()
    with tempfile.TemporaryDirectory() as tmp:
        cfg["storage"]["root_dir"] = tmp
        store = SemanticStore(cfg)
        store.start_pose = RobotPose("map", 0.0, 0.0, 0.0)
        err = store.compute_loop_error(RobotPose("map", 0.08, 0.03, 0.05))
        assert err["quality"] == "pass"
        err2 = store.compute_loop_error(RobotPose("map", 0.5, 0.0, 0.5))
        assert err2["quality"] == "fail"


if __name__ == "__main__":
    test_landmark_merge_and_save()
    test_loop_quality()
    print("PASS test_semantic_store")
