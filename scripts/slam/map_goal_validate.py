#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Occupancy grid helpers for click-goal validation."""

from __future__ import annotations

from typing import Optional, Tuple

from nav_msgs.msg import OccupancyGrid

# Match map yaml occupied_thresh / free_thresh defaults.
OCCUPIED_THRESH = 65
FREE_THRESH_MAX = 25


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


def path_stays_in_known_free(grid: OccupancyGrid, path_poses) -> Tuple[bool, str]:
    for pose in path_poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        ok, reason = is_known_free(grid, x, y)
        if not ok:
            return False, f'path point ({x:.2f},{y:.2f}) -> {reason}'
    return True, 'ok'
