#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional, List

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
    """
    将 2D LaserScan 近似为：
    - 正前方距离 front_distance
    - 图像横向 u 对应方向的 target_distance

    注意：这不是像素级深度，只是水平角方向距离。
    """

    def __init__(
        self,
        min_range: float = 0.08,
        max_range: float = 6.0,
        front_deg: float = 18.0,
        target_window_deg: float = 8.0,
        camera_hfov_deg: float = 70.0,
        camera_lidar_yaw_offset_deg: float = 0.0,
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

    def _valid_ranges_near_angle(self, angle_rad: float, window_deg: float) -> List[float]:
        scan = self.latest_scan
        if scan is None:
            return []

        window_rad = math.radians(window_deg)
        values = []

        angle_min = float(scan.angle_min)
        angle_inc = float(scan.angle_increment)

        if abs(angle_inc) < 1e-9:
            return []

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < self.min_range or r > self.max_range:
                continue

            a = angle_min + i * angle_inc
            # wrap 到 [-pi, pi]
            da = math.atan2(math.sin(a - angle_rad), math.cos(a - angle_rad))
            if abs(da) <= window_rad:
                values.append(float(r))

        return values

    def _median_distance(self, angle_deg: float, window_deg: float) -> Optional[float]:
        vals = self._valid_ranges_near_angle(math.radians(angle_deg), window_deg)
        if not vals:
            return None
        return float(np.median(vals))

    def front_distance(self) -> Optional[float]:
        return self._median_distance(0.0, self.front_deg)

    def pixel_u_to_angle_deg(self, u: float, image_width: int) -> float:
        # u 在左边为负角，右边为正角。若你的雷达坐标反了，后面把符号反过来。
        x_norm = (float(u) - image_width / 2.0) / max(1.0, image_width)
        angle = x_norm * self.camera_hfov_deg + self.camera_lidar_yaw_offset_deg
        return float(angle)

    def estimate_for_point(self, u: Optional[float], image_width: int) -> LidarDepthState:
        if self.latest_scan is None:
            return LidarDepthState(valid=False, reason="no_scan")

        front = self.front_distance()

        if u is None:
            return LidarDepthState(
                front_distance=front,
                target_distance=None,
                target_angle_deg=None,
                valid=front is not None,
                reason="no_u",
            )

        angle_deg = self.pixel_u_to_angle_deg(float(u), image_width)
        target = self._median_distance(angle_deg, self.target_window_deg)

        return LidarDepthState(
            front_distance=front,
            target_distance=target,
            target_angle_deg=angle_deg,
            valid=(front is not None or target is not None),
            reason="ok" if (front is not None or target is not None) else "no_valid_ranges",
        )