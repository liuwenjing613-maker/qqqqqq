#!/usr/bin/env python3
"""Project YOLO detections to map/odom coordinates using camera bearing and LiDAR."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class RobotPose:
    frame_id: str
    x: float
    y: float
    yaw: float
    provisional: bool = False


def normalize_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def pixel_to_bearing_rad(
    u: float,
    image_width: float,
    hfov_deg: float,
    yaw_offset_deg: float = 0.0,
) -> float:
    x_norm = (float(u) - image_width / 2.0) / max(image_width, 1.0)
    bearing_deg = x_norm * float(hfov_deg) + float(yaw_offset_deg)
    return math.radians(bearing_deg)


def project_object_xy(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    bearing_rad: float,
    range_m: float,
) -> Tuple[float, float]:
    angle = robot_yaw + bearing_rad
    ox = robot_x + range_m * math.cos(angle)
    oy = robot_y + range_m * math.sin(angle)
    return ox, oy


def _scan_angle_min(scan: Dict[str, Any]) -> float:
    return float(scan.get("angle_min", 0.0))


def _scan_angle_increment(scan: Dict[str, Any]) -> float:
    return float(scan.get("angle_increment", 0.0))


def _scan_ranges(scan: Dict[str, Any]) -> Sequence[float]:
    return scan.get("ranges") or []


def estimate_lidar_range_median(
    scan: Optional[Dict[str, Any]],
    bearing_rad: float,
    *,
    min_range_m: float = 0.18,
    max_range_m: float = 4.0,
    target_window_deg: float = 8.0,
    range_window_deg: Optional[float] = None,
    camera_to_laser_yaw_deg: float = 0.0,
    robot_yaw: float = 0.0,
) -> Optional[float]:
    """Estimate range in laser frame at bearing (robot-relative)."""
    if scan is None:
        return None

    ranges = _scan_ranges(scan)
    if not ranges:
        return None

    angle_min = _scan_angle_min(scan)
    angle_inc = _scan_angle_increment(scan)
    if abs(angle_inc) < 1e-9:
        return None

    # bearing_rad is camera/base_link relative; apply camera-laser yaw offset for LiDAR lookup.
    laser_bearing = normalize_angle(bearing_rad + math.radians(camera_to_laser_yaw_deg))
    window_deg = float(range_window_deg) if range_window_deg is not None else target_window_deg
    half_window = math.radians(window_deg / 2.0)
    samples: List[float] = []

    for i, raw in enumerate(ranges):
        if raw is None:
            continue
        try:
            r = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(r):
            continue
        if r < min_range_m or r > max_range_m:
            continue
        beam_angle = angle_min + i * angle_inc
        delta = abs(normalize_angle(beam_angle - laser_bearing))
        if delta <= half_window:
            samples.append(r)

    if not samples:
        return None
    return float(np.median(np.asarray(samples, dtype=np.float64)))


def parse_bbox_xyxy(box: Dict[str, Any]) -> Optional[List[float]]:
    if isinstance(box.get("bbox_xyxy"), (list, tuple)) and len(box["bbox_xyxy"]) == 4:
        return [float(v) for v in box["bbox_xyxy"]]
    bbox = box.get("bbox")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return None
    x1, y1, a, b = [float(v) for v in bbox]
    if a > x1 and b > y1:
        return [x1, y1, a, b]
    return [x1, y1, x1 + a, y1 + b]


def box_center_uv(box: Dict[str, Any], bbox_xyxy: List[float]) -> Tuple[float, float]:
    if box.get("cx") is not None and box.get("cy") is not None:
        return float(box["cx"]), float(box["cy"])
    if isinstance(box.get("center"), (list, tuple)) and len(box["center"]) >= 2:
        return float(box["center"][0]), float(box["center"][1])
    x1, y1, x2, y2 = bbox_xyxy
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def laser_scan_to_dict(msg: Any) -> Dict[str, Any]:
    return {
        "angle_min": float(msg.angle_min),
        "angle_increment": float(msg.angle_increment),
        "ranges": list(msg.ranges),
        "range_min": float(getattr(msg, "range_min", 0.0)),
        "range_max": float(getattr(msg, "range_max", 30.0)),
    }
