#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Occupancy grid helpers for click-goal validation."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

from nav_msgs.msg import OccupancyGrid

# Match map yaml occupied_thresh / free_thresh defaults.
OCCUPIED_THRESH = 65
FREE_THRESH_MAX = 25
DEFAULT_ROBOT_RADIUS = 0.11
DEFAULT_PATH_SAMPLE_STEP = 0.05


def world_to_map(grid: OccupancyGrid, x: float, y: float) -> Optional[Tuple[int, int]]:
    res = grid.info.resolution
    if res <= 0.0:
        return None
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    mx = int((x - ox) / res)
    my = int((y - oy) / res)
    if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
        return None
    return mx, my


def map_cell_value(grid: OccupancyGrid, mx: int, my: int) -> int:
    idx = my * grid.info.width + mx
    return int(grid.data[idx])


def classify_map_cell(val: int) -> str:
    if val < 0:
        return 'unknown'
    if val >= OCCUPIED_THRESH:
        return 'occupied'
    if val <= FREE_THRESH_MAX:
        return 'free'
    return 'occupied'


def is_known_free(grid: OccupancyGrid, x: float, y: float) -> Tuple[bool, str]:
    cell = world_to_map(grid, x, y)
    if cell is None:
        return False, 'out_of_map'
    mx, my = cell
    val = map_cell_value(grid, mx, my)
    kind = classify_map_cell(val)
    if kind == 'free':
        return True, f'free(val={val})'
    return False, f'{kind}(val={val})'


def _footprint_offsets(radius_m: float, resolution: float) -> Iterable[Tuple[int, int]]:
    cells = max(1, int(math.ceil(radius_m / resolution)))
    radius_cells = radius_m / resolution
    r2 = radius_cells * radius_cells
    for dx in range(-cells, cells + 1):
        for dy in range(-cells, cells + 1):
            if (dx * dx + dy * dy) <= r2 + 1e-6:
                yield dx, dy


def is_footprint_known_free(
    grid: OccupancyGrid,
    x: float,
    y: float,
    robot_radius: float = DEFAULT_ROBOT_RADIUS,
) -> Tuple[bool, str]:
    """True only if the full robot disk lies on known-free (white) cells."""
    center = world_to_map(grid, x, y)
    if center is None:
        return False, 'out_of_map'
    mx0, my0 = center
    res = grid.info.resolution
    for dx, dy in _footprint_offsets(robot_radius, res):
        mx = mx0 + dx
        my = my0 + dy
        if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
            return False, 'footprint_out_of_map'
        val = map_cell_value(grid, mx, my)
        kind = classify_map_cell(val)
        if kind != 'free':
            return False, f'footprint_{kind}(val={val}) at cell ({mx},{my})'
    return True, 'ok'


def _sample_path_points(path_poses, sample_step: float) -> list[Tuple[float, float]]:
    if not path_poses:
        return []
    points: list[Tuple[float, float]] = []
    prev = path_poses[0].pose.position
    points.append((prev.x, prev.y))
    for pose in path_poses[1:]:
        cx = pose.pose.position.x
        cy = pose.pose.position.y
        px, py = prev.x, prev.y
        dist = math.hypot(cx - px, cy - py)
        steps = max(1, int(math.ceil(dist / sample_step)))
        for i in range(1, steps + 1):
            t = i / steps
            points.append((px + t * (cx - px), py + t * (cy - py)))
        prev = pose.pose.position
    return points


def path_stays_in_known_free(
    grid: OccupancyGrid,
    path_poses,
    robot_radius: float = DEFAULT_ROBOT_RADIUS,
    sample_step: float = DEFAULT_PATH_SAMPLE_STEP,
) -> Tuple[bool, str]:
    """Check path centerline samples; each sample uses robot footprint on white cells only."""
    points = _sample_path_points(path_poses, sample_step)
    if not points:
        return False, 'empty_path'
    for x, y in points:
        ok, reason = is_footprint_known_free(grid, x, y, robot_radius)
        if not ok:
            return False, f'path point ({x:.2f},{y:.2f}) -> {reason}'
    return True, 'ok'
