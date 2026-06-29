import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class FreeSpaceConfig:
    lidar_min_range: float = 0.08
    lidar_max_range: float = 6.0
    lidar_front_deg: float = 18.0
    camera_hfov_deg: float = 70.0
    camera_lidar_yaw_offset_deg: float = 0.0

    sector_deg: float = 70.0
    step_deg: float = 5.0
    window_deg: float = 10.0
    min_clearance: float = 0.45
    good_clearance: float = 1.20
    smooth_alpha: float = 0.35

    waypoint_v_ratio: float = 0.62
    min_u_ratio: float = 0.20
    max_u_ratio: float = 0.80

    clearance_weight: float = 0.60
    center_weight: float = 0.30
    consistency_weight: float = 0.10
    target_window_deg: float = 8.0


class FreeSpaceWaypointProvider:
    def __init__(self, cfg: FreeSpaceConfig):
        self.cfg = cfg
        self.scan = None
        self.last_update_time = 0.0
        self.last_heading_deg: Optional[float] = None
        self.smoothed_heading_deg: Optional[float] = None

    def update_scan(self, scan_msg: Any) -> None:
        self.scan = scan_msg
        self.last_update_time = time.time()

    def has_scan(self) -> bool:
        return self.scan is not None

    def scan_age(self, now: Optional[float] = None) -> Optional[float]:
        if self.scan is None:
            return None
        return max(0.0, float(now if now is not None else time.time()) - self.last_update_time)

    def sector_clearance(
        self,
        heading_deg: float,
        window_deg: Optional[float] = None,
        method: str = "min",
    ) -> Optional[float]:
        win = float(window_deg if window_deg is not None else self.cfg.window_deg)
        return self._window_clearance(heading_deg, win, method=method)

    def front_min_distance(self, front_deg: Optional[float] = None) -> Optional[float]:
        """Safety: minimum range in front sector. Never use percentile for emergency."""
        if self.scan is None:
            return None
        deg = float(front_deg if front_deg is not None else self.cfg.lidar_front_deg)
        return self._window_clearance(0.0, deg, method="min")

    def left_clearance(self, heading_deg: float = 45.0, window_deg: Optional[float] = None) -> Optional[float]:
        return self.sector_clearance(heading_deg, window_deg, method="min")

    def right_clearance(self, heading_deg: float = -45.0, window_deg: Optional[float] = None) -> Optional[float]:
        return self.sector_clearance(heading_deg, window_deg, method="min")

    def pixel_u_to_heading_deg(self, u: float, image_width: int) -> float:
        u_ratio = float(u) / max(float(image_width), 1.0)
        return (u_ratio - 0.5) * self.cfg.camera_hfov_deg

    def target_distance_at_u(self, u: float, image_width: int) -> Optional[float]:
        """Lidar range in a narrow window around the image column of the target."""
        heading = self.pixel_u_to_heading_deg(u, image_width)
        return self.sector_clearance(heading, self.cfg.target_window_deg, method="min")

    def front_distance(self) -> Optional[float]:
        if self.scan is None:
            return None
        return self._window_clearance(0.0, self.cfg.lidar_front_deg, method="min_or_percentile")

    def get_waypoint(self, image_width: int, image_height: int) -> Dict[str, Any]:
        if self.scan is None:
            return {
                "usable": False,
                "mode": "no_scan",
                "reason": "no_scan",
                "front_distance": None,
            }

        front = self.front_distance()
        candidates = self._generate_candidates()

        best = None
        for heading_deg in candidates:
            clearance = self._window_clearance(heading_deg, self.cfg.window_deg, method="percentile25")
            if clearance is None or clearance < self.cfg.min_clearance:
                continue

            clearance_score = clamp(clearance / self.cfg.good_clearance, 0.0, 1.0)
            center_score = 1.0 - clamp(abs(heading_deg) / max(self.cfg.sector_deg, 1e-6), 0.0, 1.0)

            if self.last_heading_deg is None:
                consistency_score = 1.0
            else:
                consistency_score = 1.0 - clamp(
                    abs(heading_deg - self.last_heading_deg) / (2.0 * self.cfg.sector_deg),
                    0.0,
                    1.0,
                )

            score = (
                self.cfg.clearance_weight * clearance_score
                + self.cfg.center_weight * center_score
                + self.cfg.consistency_weight * consistency_score
            )

            item = {
                "heading_deg": heading_deg,
                "clearance": clearance,
                "score": score,
                "clearance_score": clearance_score,
                "center_score": center_score,
                "consistency_score": consistency_score,
            }
            if best is None or item["score"] > best["score"]:
                best = item

        if best is None:
            return {
                "usable": False,
                "mode": "blocked",
                "reason": "no_safe_direction",
                "front_distance": front,
            }

        heading = best["heading_deg"]

        if self.smoothed_heading_deg is None:
            self.smoothed_heading_deg = heading
        else:
            a = clamp(self.cfg.smooth_alpha, 0.0, 1.0)
            self.smoothed_heading_deg = (1.0 - a) * self.smoothed_heading_deg + a * heading

        self.last_heading_deg = heading
        used_heading = self.smoothed_heading_deg

        u = self._heading_to_pixel_u(used_heading, image_width)
        v = float(image_height) * self.cfg.waypoint_v_ratio

        reason = "center_open" if abs(used_heading) <= 8.0 else "side_open"

        return {
            "usable": True,
            "mode": "free_space",
            "u": float(u),
            "v": float(v),
            "heading_deg": float(used_heading),
            "raw_heading_deg": float(heading),
            "clearance": float(best["clearance"]),
            "front_distance": front,
            "score": float(best["score"]),
            "reason": reason,
        }

    def _generate_candidates(self) -> List[float]:
        s = float(self.cfg.sector_deg)
        step = max(float(self.cfg.step_deg), 1.0)

        values = []
        x = -s
        while x <= s + 1e-6:
            values.append(float(x))
            x += step

        values = sorted(values, key=lambda a: (abs(a), a))
        return values

    def _heading_to_pixel_u(self, heading_deg: float, image_width: int) -> float:
        u_ratio = 0.5 + heading_deg / max(self.cfg.camera_hfov_deg, 1e-6)
        u_ratio = clamp(u_ratio, self.cfg.min_u_ratio, self.cfg.max_u_ratio)
        return u_ratio * float(image_width)

    def _window_clearance(self, heading_deg: float, window_deg: float, method: str = "percentile25") -> Optional[float]:
        values = self._ranges_in_window(heading_deg, window_deg)
        if not values:
            return None

        arr = np.array(values, dtype=np.float32)

        if method == "min":
            return float(np.min(arr))
        if method == "median":
            return float(np.median(arr))
        if method == "percentile25":
            return float(np.percentile(arr, 25))
        if method == "min_or_percentile":
            return float(np.percentile(arr, 20))

        return float(np.percentile(arr, 25))

    def _ranges_in_window(self, heading_deg: float, window_deg: float) -> List[float]:
        scan = self.scan
        if scan is None:
            return []

        ranges = getattr(scan, "ranges", None)
        if ranges is None:
            return []

        angle_min = float(getattr(scan, "angle_min", 0.0))
        angle_inc = float(getattr(scan, "angle_increment", 0.0))
        if abs(angle_inc) < 1e-9:
            return []

        center_deg = heading_deg + self.cfg.camera_lidar_yaw_offset_deg
        half = window_deg / 2.0

        out = []
        for deg in np.linspace(center_deg - half, center_deg + half, num=max(3, int(window_deg) + 1)):
            rad = math.radians(float(deg))
            idx = int(round((rad - angle_min) / angle_inc))
            if idx < 0 or idx >= len(ranges):
                continue

            r = float(ranges[idx])
            if not math.isfinite(r):
                continue
            if r < self.cfg.lidar_min_range or r > self.cfg.lidar_max_range:
                continue
            out.append(r)

        return out
