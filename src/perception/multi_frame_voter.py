#!/usr/bin/env python3
"""
Multi-frame voting and short-term holding for YOLO/MVP targets.

Purpose:
- YOLO-World on video may output target only in some frames.
- This module confirms a target if it appears in enough frames within a sliding window.
- It also holds the last confirmed target for a few missing frames.
- It smooths bbox to reduce jitter.

Input target example:
{
    "visible": True,
    "class_name": "bottle",
    "score": 0.0123,
    "bbox": [x, y, w, h],
    "area_ratio": 0.08,
}

Output target:
- Same dict, plus:
  "voted": True
  "vote_count": ...
  "vote_window": ...
  "vote_reason": ...
"""

from collections import deque
from copy import deepcopy


def _is_visible_target(target):
    return bool(target and target.get("visible", False) and target.get("bbox"))


def _bbox_to_xyxy(bbox):
    x, y, w, h = [float(v) for v in bbox]
    return x, y, x + w, y + h


def bbox_iou(a, b):
    if not a or not b:
        return 0.0

    ax1, ay1, ax2, ay2 = _bbox_to_xyxy(a)
    bx1, by1, bx2, by2 = _bbox_to_xyxy(b)

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih

    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter

    if union <= 1e-9:
        return 0.0
    return inter / union


def center_distance_ratio(a, b, image_width=1280, image_height=720):
    if not a or not b:
        return 999.0

    ax, ay, aw, ah = [float(v) for v in a]
    bx, by, bw, bh = [float(v) for v in b]

    acx = ax + aw / 2.0
    acy = ay + ah / 2.0
    bcx = bx + bw / 2.0
    bcy = by + bh / 2.0

    dx = (acx - bcx) / max(1.0, float(image_width))
    dy = (acy - bcy) / max(1.0, float(image_height))
    return (dx * dx + dy * dy) ** 0.5


def _same_class(a, b):
    ca = str((a or {}).get("class_name", "")).lower().strip()
    cb = str((b or {}).get("class_name", "")).lower().strip()
    if not ca or not cb:
        return True
    return ca == cb


def _smooth_bbox(old_bbox, new_bbox, alpha=0.65):
    if not old_bbox or not new_bbox:
        return new_bbox or old_bbox

    out = []
    for old, new in zip(old_bbox, new_bbox):
        out.append(int(round(alpha * float(old) + (1.0 - alpha) * float(new))))
    return out


def _bbox_center(bbox):
    x, y, w, h = [float(v) for v in bbox]
    return x + w / 2.0, y + h / 2.0


def _bbox_area_ratio(bbox, image_width=1280, image_height=720):
    x, y, w, h = [float(v) for v in bbox]
    return max(0.0, w * h) / max(1.0, float(image_width) * float(image_height))


class MultiFrameTargetVoter:
    """
    Sliding-window target voting.

    Default strategy:
    - In the last 10 frames, if at least 3 frames have a similar target, confirm it.
    - If confirmed target disappears briefly, hold it for 5 frames.
    """

    def __init__(
        self,
        window_size=10,
        min_votes=3,
        lost_hold_frames=5,
        iou_threshold=0.20,
        center_dist_threshold=0.18,
        smooth_alpha=0.65,
        image_width=1280,
        image_height=720,
        min_switch_score_ratio=1.8,
    ):
        self.window_size = int(window_size)
        self.min_votes = int(min_votes)
        self.lost_hold_frames = int(lost_hold_frames)
        self.iou_threshold = float(iou_threshold)
        self.center_dist_threshold = float(center_dist_threshold)
        self.smooth_alpha = float(smooth_alpha)
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.min_switch_score_ratio = float(min_switch_score_ratio)

        self.history = deque(maxlen=self.window_size)
        self.locked_target = None
        self.lost_count = 0
        self.total_updates = 0

    def reset(self):
        self.history.clear()
        self.locked_target = None
        self.lost_count = 0
        self.total_updates = 0

    def update(self, target):
        self.total_updates += 1

        if _is_visible_target(target):
            self.history.append(deepcopy(target))
        else:
            self.history.append(None)

        candidate, vote_count = self._best_voted_candidate()

        if candidate is not None and vote_count >= self.min_votes:
            self.lost_count = 0
            confirmed = self._merge_with_locked(candidate, vote_count)
            self.locked_target = deepcopy(confirmed)
            return confirmed

        if self.locked_target is not None:
            self.lost_count += 1

            if self.lost_count <= self.lost_hold_frames:
                held = deepcopy(self.locked_target)
                held["visible"] = True
                held["voted"] = True
                held["vote_count"] = vote_count
                held["vote_window"] = self.window_size
                held["vote_reason"] = "hold_last_target"
                held["reason"] = "hold_last_target"
                held["stale"] = True
                held["source"] = "held_old_target"
                return held

            # 超过保持帧数后，释放旧目标，允许重新识别
            self.locked_target = None
            self.history.clear()
            self.lost_count = 0

        return {
            "visible": False,
            "voted": False,
            "vote_count": vote_count,
            "vote_window": self.window_size,
            "reason": "waiting_multiframe_votes",
            "vote_reason": "waiting_multiframe_votes",
            "stale": False,
            "source": "no_target",
        }

    def _best_voted_candidate(self):
        visible = [deepcopy(t) for t in self.history if _is_visible_target(t)]
        if not visible:
            return None, 0

        clusters = []
        for target in visible:
            placed = False
            for cluster in clusters:
                ref = cluster[0]
                if self._is_same_object(ref, target):
                    cluster.append(target)
                    placed = True
                    break
            if not placed:
                clusters.append([target])

        best_cluster = None
        best_key = None
        for cluster in clusters:
            count = len(cluster)
            score_sum = sum(float(t.get("score", 0.0)) for t in cluster)
            score_max = max(float(t.get("score", 0.0)) for t in cluster)
            key = (count, score_sum, score_max)
            if best_key is None or key > best_key:
                best_key = key
                best_cluster = cluster

        if not best_cluster:
            return None, 0

        candidate = self._average_cluster(best_cluster)
        return candidate, len(best_cluster)

    def _is_same_object(self, a, b):
        if not _same_class(a, b):
            return False

        iou = bbox_iou(a.get("bbox"), b.get("bbox"))
        dist = center_distance_ratio(
            a.get("bbox"),
            b.get("bbox"),
            image_width=self.image_width,
            image_height=self.image_height,
        )

        return iou >= self.iou_threshold or dist <= self.center_dist_threshold

    def _average_cluster(self, cluster):
        """
        多帧投票只负责确认目标可信；
        控制和显示用 bbox 必须尽量来自最新一帧，避免小车运动时旧框滞后。
        """
        latest = deepcopy(cluster[-1])

        bbox = latest.get("bbox", [0, 0, 0, 0])
        latest["bbox"] = [int(round(float(v))) for v in bbox]
        latest["cx"], latest["cy"] = _bbox_center(latest["bbox"])
        latest["area_ratio"] = _bbox_area_ratio(
            latest["bbox"],
            image_width=self.image_width,
            image_height=self.image_height,
        )

        # 记录 cluster 里的最高分，但位置使用最新 bbox
        latest["score"] = max(float(t.get("score", 0.0)) for t in cluster)
        latest["latest_score"] = float(cluster[-1].get("score", 0.0))
        latest["stale"] = False
        latest["source"] = "latest_voted_candidate"

        return latest

    def _merge_with_locked(self, candidate, vote_count):
        out = deepcopy(candidate)

        if self.locked_target is not None:
            same = self._is_same_object(self.locked_target, candidate)

            if not same:
                # 新候选已经通过多帧投票，允许切换。
                # 不要因为旧目标 score 高就一直锁死旧目标。
                out["vote_reason"] = "switch_to_voted_candidate"
                out["reason"] = "switch_to_voted_candidate"
                out["stale"] = False
                out["source"] = "latest_voted_candidate"

            if same:
                smoothed_bbox = _smooth_bbox(
                    self.locked_target.get("bbox"),
                    candidate.get("bbox"),
                    alpha=self.smooth_alpha,
                )
                out["bbox"] = smoothed_bbox
                out["cx"], out["cy"] = _bbox_center(smoothed_bbox)
                out["area_ratio"] = _bbox_area_ratio(
                    smoothed_bbox,
                    image_width=self.image_width,
                    image_height=self.image_height,
                )

        out["visible"] = True
        out["voted"] = True
        out["vote_count"] = vote_count
        out["vote_window"] = self.window_size
        out["stale"] = False

        if "vote_reason" not in out:
            out["vote_reason"] = "confirmed_by_multiframe_votes"
        if "reason" not in out:
            out["reason"] = out["vote_reason"]
        if "source" not in out:
            out["source"] = "latest_voted_candidate"

        return out
