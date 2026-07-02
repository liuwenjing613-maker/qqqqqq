#!/usr/bin/env python3
"""Path follower for astar explore mode."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple


def follow_path(
    path: List[Tuple[float, float]],
    robot_xy: Tuple[float, float],
    robot_yaw: float,
    lookahead_m: float = 0.35,
) -> Tuple[Optional[float], Optional[float], int]:
    """Return (bearing_rad, distance_m, waypoint_index) in robot frame."""
    if not path:
        return None, None, -1

    best_idx = 0
    best_dist = float("inf")
    for i, (px, py) in enumerate(path):
        d = math.hypot(px - robot_xy[0], py - robot_xy[1])
        if d < best_dist:
            best_dist = d
            best_idx = i

    target_idx = best_idx
    for i in range(best_idx, len(path)):
        px, py = path[i]
        d = math.hypot(px - robot_xy[0], py - robot_xy[1])
        if d >= lookahead_m:
            target_idx = i
            break
    else:
        target_idx = len(path) - 1

    tx, ty = path[target_idx]
    dx = tx - robot_xy[0]
    dy = ty - robot_xy[1]
    distance = math.hypot(dx, dy)
    world_bearing = math.atan2(dy, dx)
    bearing = (world_bearing - robot_yaw + math.pi) % (2 * math.pi) - math.pi
    return bearing, distance, target_idx
