#!/usr/bin/env python3
"""LiDAR helper for Qwen-only navigation (aligned with YOLO free_space target ranging)."""
import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from sensor_msgs.msg import LaserScan


@dataclass
class LidarDepthState:
    front_distance: Optional[float] = None
    target_distance: Optional[float] = None
    target_distance_raw: Optional[float] = None
    target_angle_deg: Optional[float] = None
    valid: bool = False
    reason: str = ""


def fuse_target_distance(
    front_distance: Optional[float],
    target_distance: Optional[float],
    ex: Optional[float] = None,
    center_deadband: float = 0.10,
    fuse_when_centered: bool = True,
    fuse_min_always: bool = False,
) -> Optional[float]:
    """Fuse LiDAR ray with front sector for safety / slowdown."""
    if target_distance is None:
        return front_distance
    if front_distance is None:
        return target_distance
    if fuse_min_always:
        return min(float(front_distance), float(target_distance))
    if fuse_when_centered and ex is not None and abs(float(ex)) <= float(center_deadband):
        return min(float(front_distance), float(target_distance))
    return float(target_distance)


def resolve_arrive_distance(
    front_distance: Optional[float],
    target_distance: Optional[float],
    arrive_threshold: float = 0.6,
    front_margin_m: float = 0.3,
    ray_far_invalid_m: float = 3.0,
) -> Optional[float]:
    """Distance used for ARRIVED; avoid false stop when front hits floor/clutter."""
    if target_distance is None:
        return front_distance
    if front_distance is None:
        return target_distance
    front = float(front_distance)
    target = float(target_distance)
    # Bottle-direction ray still beyond arrive: front min is often floor, not the bottle.
    if target > float(arrive_threshold) + float(front_margin_m):
        if front <= float(arrive_threshold) and target > float(ray_far_invalid_m):
            return front
        return target
    return min(front, target)


class LidarDepthEstimator:
    def __init__(
        self,
        min_range=0.08,
        max_range=6.0,
        front_deg=18.0,
        target_window_deg=8.0,
        camera_hfov_deg=70.0,
        camera_lidar_yaw_offset_deg=0.0,
        target_distance_method: str = "min",
        front_distance_method: str = "min",
    ):
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.front_deg = float(front_deg)
        self.target_window_deg = float(target_window_deg)
        self.camera_hfov_deg = float(camera_hfov_deg)
        self.camera_lidar_yaw_offset_deg = float(camera_lidar_yaw_offset_deg)
        self.target_distance_method = str(target_distance_method)
        self.front_distance_method = str(front_distance_method)
        self.latest_scan: Optional[LaserScan] = None

    def update_scan(self, scan: LaserScan) -> None:
        self.latest_scan = scan

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _valid_ranges_near_angle(self, angle_rad: float, window_deg: float) -> List[float]:
        scan = self.latest_scan
        if scan is None:
            return []
        window_rad = math.radians(float(window_deg))
        angle_min = float(scan.angle_min)
        angle_inc = float(scan.angle_increment)
        if abs(angle_inc) < 1e-9:
            return []
        values = []
        for i, r in enumerate(scan.ranges):
            try:
                rv = float(r)
            except Exception:
                continue
            if not math.isfinite(rv):
                continue
            if rv < self.min_range or rv > self.max_range:
                continue
            a = angle_min + i * angle_inc
            if abs(self._angle_diff(a, angle_rad)) <= window_rad:
                values.append(rv)
        return values

    @staticmethod
    def _reduce_ranges(values: List[float], method: str) -> Optional[float]:
        if not values:
            return None
        arr = np.array(values, dtype=np.float32)
        m = str(method).strip().lower()
        if m == "min":
            return float(np.min(arr))
        if m == "median":
            return float(np.median(arr))
        if m in ("percentile25", "p25"):
            return float(np.percentile(arr, 25))
        return float(np.min(arr))

    def _distance_at_angle(self, angle_deg: float, window_deg: float, method: str) -> Optional[float]:
        center_rad = math.radians(float(angle_deg) + self.camera_lidar_yaw_offset_deg)
        return self._reduce_ranges(
            self._valid_ranges_near_angle(center_rad, window_deg),
            method,
        )

    def front_distance(self) -> Optional[float]:
        return self._distance_at_angle(0.0, self.front_deg, self.front_distance_method)

    def pixel_u_to_angle_deg(self, u: float, image_width: int) -> float:
        x_norm = (float(u) - float(image_width) / 2.0) / max(1.0, float(image_width))
        return float(x_norm * self.camera_hfov_deg)

    def target_distance_at_u(self, u: float, image_width: int) -> Optional[float]:
        angle_deg = self.pixel_u_to_angle_deg(float(u), int(image_width))
        return self._distance_at_angle(angle_deg, self.target_window_deg, self.target_distance_method)

    def estimate_for_point(self, u: Optional[float], image_width: int) -> LidarDepthState:
        if self.latest_scan is None:
            return LidarDepthState(valid=False, reason="no_scan")
        front = self.front_distance()
        if u is None:
            return LidarDepthState(front_distance=front, valid=front is not None, reason="no_u")
        angle_deg = self.pixel_u_to_angle_deg(float(u), int(image_width))
        target_raw = self.target_distance_at_u(float(u), int(image_width))
        valid = front is not None or target_raw is not None
        return LidarDepthState(
            front_distance=front,
            target_distance=target_raw,
            target_distance_raw=target_raw,
            target_angle_deg=angle_deg + self.camera_lidar_yaw_offset_deg,
            valid=valid,
            reason="ok" if valid else "no_valid_ranges",
        )
