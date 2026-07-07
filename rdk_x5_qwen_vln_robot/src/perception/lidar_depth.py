#!/usr/bin/env python3
"""LiDAR helper for Qwen-only navigation."""
import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from sensor_msgs.msg import LaserScan


@dataclass
class LidarDepthState:
    front_distance: Optional[float] = None
    target_distance: Optional[float] = None
    target_angle_deg: Optional[float] = None
    valid: bool = False
    reason: str = ""


class LidarDepthEstimator:
    def __init__(
        self,
        min_range=0.08,
        max_range=6.0,
        front_deg=18.0,
        target_window_deg=8.0,
        camera_hfov_deg=70.0,
        camera_lidar_yaw_offset_deg=0.0,
    ):
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.front_deg = float(front_deg)
        self.target_window_deg = float(target_window_deg)
        self.camera_hfov_deg = float(camera_hfov_deg)
        self.camera_lidar_yaw_offset_deg = float(camera_lidar_yaw_offset_deg)
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

    def _median_distance(self, angle_deg: float, window_deg: float) -> Optional[float]:
        vals = self._valid_ranges_near_angle(math.radians(angle_deg), window_deg)
        if not vals:
            return None
        return float(np.median(vals))

    def front_distance(self) -> Optional[float]:
        return self._median_distance(0.0, self.front_deg)

    def pixel_u_to_angle_deg(self, u: float, image_width: int) -> float:
        x_norm = (float(u) - float(image_width) / 2.0) / max(1.0, float(image_width))
        return float(x_norm * self.camera_hfov_deg + self.camera_lidar_yaw_offset_deg)

    def estimate_for_point(self, u: Optional[float], image_width: int) -> LidarDepthState:
        if self.latest_scan is None:
            return LidarDepthState(valid=False, reason="no_scan")
        front = self.front_distance()
        if u is None:
            return LidarDepthState(front_distance=front, valid=front is not None, reason="no_u")
        angle_deg = self.pixel_u_to_angle_deg(float(u), int(image_width))
        target = self._median_distance(angle_deg, self.target_window_deg)
        valid = front is not None or target is not None
        return LidarDepthState(front, target, angle_deg, valid, "ok" if valid else "no_valid_ranges")
