#!/usr/bin/env python3
"""Temporal multi-object filter for semantic mapping detections."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Tuple

from src.perception.multi_frame_voter import bbox_iou, center_distance_ratio


@dataclass
class TrackState:
    track_id: int
    class_name: str
    bbox_xyxy: List[float]
    score: float
    area_ratio: float
    u: float
    v: float
    stamp: float
    vote_count: int = 0
    is_candidate: bool = False
    is_confirmed: bool = False
    is_dynamic: bool = False
    is_edge_box: bool = False
    hold_frames_left: int = 0
    last_seen_stamp: float = 0.0
    history: Deque[Dict[str, Any]] = field(default_factory=deque)
    raw: Dict[str, Any] = field(default_factory=dict)


class SemanticObjectFilter:
    def __init__(self, cfg: Dict[str, Any], image_width: int = 640, image_height: int = 480):
        self.cfg = cfg
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self._next_track_id = 1
        self._tracks: Dict[int, TrackState] = {}
        self._class_tracks: Dict[str, List[int]] = {}

        classes = cfg.get("classes", {})
        self.whitelist = set(classes.get("whitelist", []))
        self.dynamic_classes = set(classes.get("dynamic", []))
        self.small_objects = set(classes.get("small_objects", []))
        self.large_objects = set(classes.get("large_objects", []))

        det = cfg.get("detection_filter", {})
        self.min_score_default = float(det.get("min_score_default", 0.25))
        self.min_score_small = float(det.get("min_score_small_object", 0.20))
        self.min_score_observe = float(det.get("min_score_observation_only", 0.15))
        self.min_area_ratio = float(det.get("min_area_ratio", 0.0006))
        self.max_area_ratio = float(det.get("max_area_ratio", 0.45))
        self.edge_margin_px = float(det.get("edge_margin_px", 6.0))
        self.max_aspect = float(det.get("max_box_aspect_ratio", 6.0))
        self.reject_stale = bool(det.get("reject_stale", True))

        tv = cfg.get("temporal_vote", {})
        self.window_size = int(tv.get("window_size", 8))
        self.min_votes_candidate = int(tv.get("min_votes_candidate", 2))
        self.min_votes_confirmed = int(tv.get("min_votes_confirmed", 3))
        self.max_time_span_sec = float(tv.get("max_time_span_sec", 2.0))
        self.iou_threshold = float(tv.get("iou_threshold", 0.10))
        self.center_dist_threshold = float(tv.get("center_dist_threshold", 0.16))
        self.hold_frames = int(tv.get("hold_frames", 4))

        low = cfg.get("low_confidence_policy", {})
        self.min_score_can_observe = float(low.get("min_score_can_observe", 0.15))
        self.min_score_can_landmark = float(low.get("min_score_can_landmark", 0.20))

    def set_image_size(self, width: int, height: int) -> None:
        self.image_width = int(width)
        self.image_height = int(height)

    def pass_class_filter(self, class_name: str) -> bool:
        if class_name in self.dynamic_classes:
            return True
        return class_name in self.whitelist

    def _min_score_for_class(self, class_name: str) -> float:
        if class_name in self.small_objects:
            return self.min_score_small
        return self.min_score_default

    def pass_box_filter(
        self,
        box: Dict[str, Any],
        bbox_xyxy: List[float],
        area_ratio: float,
    ) -> Tuple[bool, bool]:
        """Returns (passes, is_edge_box)."""
        class_name = str(box.get("class_name", ""))
        score = float(box.get("score", 0.0))
        if self.reject_stale and bool(box.get("stale", False)):
            return False, False

        min_score = max(self.min_score_can_observe, self._min_score_for_class(class_name))
        if score < min_score:
            return False, False
        if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
            return False, False

        x1, y1, x2, y2 = bbox_xyxy
        w = max(1.0, x2 - x1)
        h = max(1.0, y2 - y1)
        aspect = max(w / h, h / w)
        if aspect > self.max_aspect:
            return False, False

        margin = self.edge_margin_px
        is_edge = (
            x1 <= margin
            or y1 <= margin
            or x2 >= self.image_width - margin
            or y2 >= self.image_height - margin
        )
        if bool(self.cfg.get("low_confidence_policy", {}).get("require_not_edge_box", True)) and is_edge and score < self.min_score_can_landmark:
            return False, is_edge
        return True, is_edge

    def can_landmark(self, track: TrackState) -> bool:
        if track.is_dynamic:
            return False
        if track.score < self.min_score_can_landmark:
            return False
        low = self.cfg.get("low_confidence_policy", {})
        if bool(low.get("require_not_edge_box", True)) and track.is_edge_box:
            return False
        return track.is_confirmed

    def _match_track(self, class_name: str, bbox_xyxy: List[float]) -> Optional[int]:
        candidates = self._class_tracks.get(class_name, [])
        best_id = None
        best_score = -1.0
        bbox_wh = [
            bbox_xyxy[0],
            bbox_xyxy[1],
            bbox_xyxy[2] - bbox_xyxy[0],
            bbox_xyxy[3] - bbox_xyxy[1],
        ]
        for tid in candidates:
            track = self._tracks.get(tid)
            if track is None:
                continue
            tb = track.bbox_xyxy
            tb_wh = [tb[0], tb[1], tb[2] - tb[0], tb[3] - tb[1]]
            iou = bbox_iou(bbox_wh, tb_wh)
            cdist = center_distance_ratio(
                bbox_wh, tb_wh, self.image_width, self.image_height
            )
            if iou >= self.iou_threshold or cdist <= self.center_dist_threshold:
                score = iou + (1.0 - min(cdist, 1.0))
                if score > best_score:
                    best_score = score
                    best_id = tid
        return best_id

    def update(
        self,
        box: Dict[str, Any],
        bbox_xyxy: List[float],
        u: float,
        v: float,
        area_ratio: float,
        stamp: float,
    ) -> Optional[TrackState]:
        class_name = str(box.get("class_name", ""))
        if not self.pass_class_filter(class_name):
            return None

        ok, is_edge = self.pass_box_filter(box, bbox_xyxy, area_ratio)
        if not ok:
            return None

        score = float(box.get("score", 0.0))
        is_dynamic = class_name in self.dynamic_classes

        tid = self._match_track(class_name, bbox_xyxy)
        if tid is None:
            tid = self._next_track_id
            self._next_track_id += 1
            track = TrackState(
                track_id=tid,
                class_name=class_name,
                bbox_xyxy=list(bbox_xyxy),
                score=score,
                area_ratio=area_ratio,
                u=u,
                v=v,
                stamp=stamp,
                is_dynamic=is_dynamic,
                is_edge_box=is_edge,
                raw=dict(box),
            )
            track.history = deque(maxlen=self.window_size)
            self._tracks[tid] = track
            self._class_tracks.setdefault(class_name, []).append(tid)
        else:
            track = self._tracks[tid]
            track.bbox_xyxy = list(bbox_xyxy)
            track.score = max(track.score, score)
            track.area_ratio = area_ratio
            track.u = u
            track.v = v
            track.stamp = stamp
            track.is_edge_box = is_edge
            track.raw = dict(box)
            track.hold_frames_left = 0

        track.last_seen_stamp = stamp
        track.history.append(
            {
                "stamp": stamp,
                "bbox_xyxy": list(bbox_xyxy),
                "score": score,
            }
        )

        stamps = [h["stamp"] for h in track.history]
        if stamps and (stamps[-1] - stamps[0]) > self.max_time_span_sec:
            while track.history and (stamps[-1] - track.history[0]["stamp"]) > self.max_time_span_sec:
                track.history.popleft()
                stamps = [h["stamp"] for h in track.history]

        track.vote_count = len(track.history)
        track.is_candidate = track.vote_count >= self.min_votes_candidate
        track.is_confirmed = track.vote_count >= self.min_votes_confirmed and not is_dynamic
        return track

    def tick_missing(self, stamp: float) -> None:
        for track in list(self._tracks.values()):
            if stamp - track.last_seen_stamp > 0.05:
                if track.hold_frames_left < self.hold_frames:
                    track.hold_frames_left += 1

    def merge_radius_for_class(self, class_name: str) -> float:
        lf = self.cfg.get("landmark_fusion", {})
        if class_name in self.large_objects:
            return float(lf.get("merge_radius_large_m", 0.80))
        return float(lf.get("merge_radius_small_m", 0.45))
