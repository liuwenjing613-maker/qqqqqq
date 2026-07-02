#!/usr/bin/env python3
"""Persistent semantic map store: observations, landmarks, viewpoints, loop quality."""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from src.mapping.semantic_projection import RobotPose, normalize_angle
from src.mapping.semantic_types import (
    SemanticLandmark,
    SemanticObservation,
    ViewpointNode,
    json_dumps,
)


class SemanticStore:
    def __init__(self, cfg: Dict[str, Any], map_name: str = "joy_semantic_map"):
        self.cfg = cfg
        self.map_name = str(map_name)
        storage = cfg.get("storage", {})
        root = os.path.expanduser(str(storage.get("root_dir", "logs/semantic_mapping")))
        session_name = str(storage.get("session_name", "auto"))
        if session_name == "auto":
            session_name = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.session_id = session_name
        self.session_dir = os.path.join(root, session_name)
        os.makedirs(self.session_dir, exist_ok=True)
        self.keyframes_dir = os.path.join(self.session_dir, "keyframes")
        os.makedirs(self.keyframes_dir, exist_ok=True)

        self.observations: List[SemanticObservation] = []
        self.landmarks: Dict[str, SemanticLandmark] = {}
        self.viewpoints: List[ViewpointNode] = []
        self._obs_counter = 0
        self._lm_counter = 0
        self._vp_counter = 0

        self.start_pose: Optional[RobotPose] = None
        self.last_pose: Optional[RobotPose] = None
        self.last_stationary_time: Optional[float] = None
        self.last_motion_time: float = time.time()
        self.loop_quality: Dict[str, Any] = {}
        self.quality_result: str = "unknown"

        lf = cfg.get("landmark_fusion", {})
        self.confirm_seen_count = int(lf.get("confirm_seen_count", 3))
        self.confirm_min_duration_sec = float(lf.get("confirm_min_duration_sec", 1.0))
        self.max_position_sigma_m = float(lf.get("max_position_sigma_m", 0.80))

        vp = cfg.get("viewpoints", {})
        self.viewpoints_enabled = bool(vp.get("enable", True))
        self.min_translation_m = float(vp.get("min_translation_m", 0.25))
        self.min_rotation_deg = float(vp.get("min_rotation_deg", 20.0))
        self.view_range_m = float(vp.get("view_range_m", 3.0))
        self.view_fov_deg = float(vp.get("view_fov_deg", 70.0))
        self.save_keyframes = bool(vp.get("save_keyframes", True))

        lq = cfg.get("loop_quality", {})
        self.loop_enabled = bool(lq.get("enable", True))
        self.max_return_xy_error_m = float(lq.get("max_return_xy_error_m", 0.20))
        self.max_return_yaw_error_deg = float(lq.get("max_return_yaw_error_deg", 12.0))
        self.require_stationary_sec = float(lq.get("require_stationary_sec_before_save", 3.0))

        self._last_autosave = 0.0
        self.autosave_sec = float(storage.get("autosave_sec", 2.0))
        self.write_jsonl = bool(storage.get("write_jsonl", True))

    def _next_obs_id(self) -> str:
        self._obs_counter += 1
        return f"obs_{self._obs_counter:06d}"

    def _next_landmark_id(self) -> str:
        self._lm_counter += 1
        return f"lm_{self._lm_counter:04d}"

    def _next_viewpoint_id(self) -> str:
        self._vp_counter += 1
        return f"vp_{self._vp_counter:06d}"

    def add_observation(self, obs: SemanticObservation) -> None:
        self.observations.append(obs)
        if self.write_jsonl:
            path = os.path.join(self.session_dir, "observations.jsonl")
            with open(path, "a", encoding="utf-8") as f:
                f.write(json_dumps(obs.to_dict()) + "\n")

    def _merge_radius(self, class_name: str, small_objects: set, large_objects: set) -> float:
        lf = self.cfg.get("landmark_fusion", {})
        if class_name in large_objects:
            return float(lf.get("merge_radius_large_m", 0.80))
        return float(lf.get("merge_radius_small_m", 0.45))

    def fuse_landmark(
        self,
        obs: SemanticObservation,
        *,
        small_objects: set,
        large_objects: set,
    ) -> Optional[SemanticLandmark]:
        if obs.object_x is None or obs.object_y is None:
            return None

        merge_r = self._merge_radius(obs.class_name, small_objects, large_objects)
        best_id = None
        best_dist = merge_r

        for lm_id, lm in self.landmarks.items():
            if lm.class_name != obs.class_name:
                continue
            if lm.state in ("rejected", "blacklisted"):
                continue
            dist = math.hypot(lm.x - obs.object_x, lm.y - obs.object_y)
            if dist <= best_dist:
                best_dist = dist
                best_id = lm_id

        if best_id is None:
            lm = SemanticLandmark(
                landmark_id=self._next_landmark_id(),
                class_name=obs.class_name,
                frame_id=obs.fixed_frame,
                x=float(obs.object_x),
                y=float(obs.object_y),
                confidence=float(obs.score),
                seen_count=1,
                first_seen_time=obs.stamp,
                last_seen_time=obs.stamp,
                state="candidate",
                sources=[obs.source],
                obs_ids=[obs.obs_id],
                covariance_xy=[obs.position_sigma, 0.0, obs.position_sigma],
                best_image_path=obs.image_path,
            )
            self.landmarks[lm.landmark_id] = lm
            return lm

        lm = self.landmarks[best_id]
        n = max(1, lm.seen_count)
        lm.x = (lm.x * n + obs.object_x) / (n + 1)
        lm.y = (lm.y * n + obs.object_y) / (n + 1)
        lm.seen_count += 1
        lm.last_seen_time = obs.stamp
        lm.confidence = max(lm.confidence, obs.score)
        if obs.source not in lm.sources:
            lm.sources.append(obs.source)
        lm.obs_ids.append(obs.obs_id)
        if obs.image_path:
            lm.best_image_path = obs.image_path

        duration = lm.last_seen_time - lm.first_seen_time
        sigma = obs.position_sigma
        if (
            lm.seen_count >= self.confirm_seen_count
            and duration >= self.confirm_min_duration_sec
            and sigma <= self.max_position_sigma_m
        ):
            lm.state = "confirmed"
        elif lm.seen_count >= 2:
            lm.state = "candidate"
        return lm

    def update_pose(self, pose: RobotPose, stamp: float, odom_speed: float = 0.0) -> None:
        if self.start_pose is None and pose.frame_id == self.cfg.get("frames", {}).get("fixed_frame", "map"):
            self.start_pose = pose
        self.last_pose = pose

        if odom_speed < 0.02:
            if self.last_stationary_time is None:
                self.last_stationary_time = stamp
        else:
            self.last_stationary_time = None
            self.last_motion_time = stamp

        if self.loop_enabled and self.start_pose is not None:
            self.loop_quality = self.compute_loop_error(pose)

    def compute_loop_error(self, current: RobotPose) -> Dict[str, Any]:
        start = self.start_pose
        if start is None:
            return {}
        dx = current.x - start.x
        dy = current.y - start.y
        xy_error = math.hypot(dx, dy)
        yaw_error = math.degrees(
            abs(normalize_angle(current.yaw - start.yaw))
        )
        quality = "pass"
        if xy_error > self.max_return_xy_error_m or yaw_error > self.max_return_yaw_error_deg:
            quality = "fail"
        return {
            "start_pose_map": [start.x, start.y, start.yaw],
            "current_pose_map": [current.x, current.y, current.yaw],
            "xy_error_m": round(xy_error, 4),
            "yaw_error_deg": round(yaw_error, 2),
            "quality": quality,
        }

    def maybe_add_viewpoint(
        self,
        pose: RobotPose,
        observed_classes: List[str],
        stamp: float,
        image_path: Optional[str] = None,
    ) -> Optional[ViewpointNode]:
        if not self.viewpoints_enabled:
            return None
        if not observed_classes and not self.cfg.get("viewpoints", {}).get("save_on_detection", True):
            return None

        if self.viewpoints:
            last = self.viewpoints[-1]
            dist = math.hypot(pose.x - last.x, pose.y - last.y)
            dyaw = abs(math.degrees(normalize_angle(pose.yaw - last.yaw)))
            if dist < self.min_translation_m and dyaw < self.min_rotation_deg:
                last.observed_classes = sorted(set(last.observed_classes + observed_classes))
                last.stamp = stamp
                return None

        vp = ViewpointNode(
            node_id=self._next_viewpoint_id(),
            stamp=stamp,
            frame_id=pose.frame_id,
            x=pose.x,
            y=pose.y,
            yaw=pose.yaw,
            observed_classes=sorted(set(observed_classes)),
            image_path=image_path,
            view_fov_deg=self.view_fov_deg,
            view_range_m=self.view_range_m,
            visited=True,
        )
        self.viewpoints.append(vp)
        if self.write_jsonl:
            path = os.path.join(self.session_dir, "viewpoints.jsonl")
            with open(path, "a", encoding="utf-8") as f:
                f.write(json_dumps(vp.to_dict()) + "\n")
        return vp

    def build_semantic_map_json(self) -> Dict[str, Any]:
        lq = self.loop_quality or {}
        self.quality_result = str(lq.get("quality", "unknown"))
        return {
            "session_id": self.session_id,
            "frame_id": self.cfg.get("frames", {}).get("fixed_frame", "map"),
            "map_name": self.map_name,
            "quality": {
                "loop_xy_error_m": lq.get("xy_error_m"),
                "loop_yaw_error_deg": lq.get("yaw_error_deg"),
                "result": self.quality_result,
            },
            "landmarks": [lm.to_dict() for lm in self.landmarks.values()],
            "viewpoints": [vp.to_dict() for vp in self.viewpoints],
            "observation_count": len(self.observations),
        }

    def save_all(self, final: bool = False) -> str:
        semantic = self.build_semantic_map_json()
        out_path = os.path.join(self.session_dir, "semantic_map.json")
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(semantic, f, ensure_ascii=False, indent=2)

        landmarks_path = os.path.join(self.session_dir, "landmarks.json")
        with open(landmarks_path, "w", encoding="utf-8") as f:
            json.dump(
                [lm.to_dict() for lm in self.landmarks.values()],
                f,
                ensure_ascii=False,
                indent=2,
            )

        loop_path = os.path.join(self.session_dir, "loop_quality.json")
        with open(loop_path, "w", encoding="utf-8") as f:
            json.dump(self.loop_quality or {}, f, ensure_ascii=False, indent=2)

        self._last_autosave = time.time()
        return out_path

    def maybe_autosave(self) -> None:
        if time.time() - self._last_autosave >= self.autosave_sec:
            self.save_all(final=False)

    def keyframe_path(self, vp_id: str) -> str:
        return os.path.join(self.keyframes_dir, f"{vp_id}.jpg")
