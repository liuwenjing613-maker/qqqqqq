#!/usr/bin/env python3
"""Occupancy-grid checks: goals must lie on scanned (known-free) cells."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple


def _cell_value(grid: Any, mx: int, my: int, unknown_value: int) -> int:
    w = int(grid.info.width)
    h = int(grid.info.height)
    if mx < 0 or my < 0 or mx >= w or my >= h:
        return 100
    idx = my * w + mx
    if idx < 0 or idx >= len(grid.data):
        return 100
    return int(grid.data[idx])


def world_to_map(grid: Any, wx: float, wy: float) -> Tuple[int, int]:
    res = float(grid.info.resolution)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    return int((wx - ox) / res), int((wy - oy) / res)


def map_to_world(grid: Any, mx: int, my: int) -> Tuple[float, float]:
    res = float(grid.info.resolution)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    return ox + (mx + 0.5) * res, oy + (my + 0.5) * res


def is_known_free(grid: Any, mx: int, my: int, cfg: Dict[str, Any]) -> bool:
    unknown_value = int(cfg.get("unknown_value", -1))
    free_threshold = int(cfg.get("free_threshold", 20))
    occupied_threshold = int(cfg.get("occupied_threshold", 65))
    val = _cell_value(grid, mx, my, unknown_value)
    if val == unknown_value or val < 0:
        return False
    if val >= occupied_threshold:
        return False
    return val <= free_threshold


def is_unknown_cell(grid: Any, mx: int, my: int, cfg: Dict[str, Any]) -> bool:
    unknown_value = int(cfg.get("unknown_value", -1))
    val = _cell_value(grid, mx, my, unknown_value)
    return val == unknown_value or val < 0


def has_unknown_neighbor(grid: Any, mx: int, my: int, cfg: Dict[str, Any]) -> bool:
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            if is_unknown_cell(grid, mx + dx, my + dy, cfg):
                return True
    return False


def is_frontier_edge_cell(grid: Any, mx: int, my: int, cfg: Dict[str, Any]) -> bool:
    """Known-free cell touching unknown (scanned frontier edge)."""
    if not is_known_free(grid, mx, my, cfg):
        return False
    return has_unknown_neighbor(grid, mx, my, cfg)


def goal_on_scanned_map(
    grid: Any,
    wx: float,
    wy: float,
    cfg: Dict[str, Any],
) -> Tuple[bool, bool, bool]:
    """Return (on_known_free, is_frontier_edge, is_interior_scanned)."""
    mx, my = world_to_map(grid, wx, wy)
    if not is_known_free(grid, mx, my, cfg):
        return False, False, False
    edge = is_frontier_edge_cell(grid, mx, my, cfg)
    return True, edge, not edge


def snap_to_known_free(
    grid: Any,
    wx: float,
    wy: float,
    robot_xy: Tuple[float, float],
    cfg: Dict[str, Any],
    min_dist_m: float,
    max_dist_m: float,
    max_search_m: float = 1.2,
) -> Optional[Tuple[float, float]]:
    """Project a point onto the nearest known-free cell within distance limits."""
    if grid is None or not grid.data:
        return None
    res = float(grid.info.resolution)
    max_cells = max(1, int(math.ceil(max_search_m / res)))
    gx0, gy0 = world_to_map(grid, wx, wy)
    best: Optional[Tuple[float, float]] = None
    best_cost = float("inf")
    for radius in range(max_cells + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                mx, my = gx0 + dx, gy0 + dy
                if not is_known_free(grid, mx, my, cfg):
                    continue
                fx, fy = map_to_world(grid, mx, my)
                dist = math.hypot(fx - robot_xy[0], fy - robot_xy[1])
                if dist < min_dist_m or dist > max_dist_m:
                    continue
                cost = math.hypot(fx - wx, fy - wy) + 0.05 * dist
                if cost < best_cost:
                    best_cost = cost
                    best = (fx, fy)
        if best is not None:
            return best
    return None


def collect_scanned_free_goals(
    grid: Any,
    robot_xy: Tuple[float, float],
    cfg: Dict[str, Any],
    min_dist_m: float,
    max_dist_m: float,
    max_count: int = 10,
) -> List[Tuple[float, float, float, bool]]:
    """Known-free goals in range; prefer frontier-edge cells on the scanned side."""
    if grid is None or not grid.data or max_count <= 0:
        return []
    res = float(grid.info.resolution)
    rx, ry, _ = robot_xy
    min_cells = int(math.floor(min_dist_m / res))
    max_cells = int(math.ceil(max_dist_m / res))
    rmx, rmy = world_to_map(grid, rx, ry)
    w = int(grid.info.width)
    h = int(grid.info.height)
    interior: List[Tuple[float, float, float, bool]] = []
    edges: List[Tuple[float, float, float, bool]] = []
    for dy in range(-max_cells, max_cells + 1):
        for dx in range(-max_cells, max_cells + 1):
            dist_cells = math.hypot(dx, dy)
            if dist_cells < min_cells or dist_cells > max_cells:
                continue
            mx, my = rmx + dx, rmy + dy
            if mx < 0 or my < 0 or mx >= w or my >= h:
                continue
            if not is_known_free(grid, mx, my, cfg):
                continue
            fx, fy = map_to_world(grid, mx, my)
            dist = math.hypot(fx - rx, fy - ry)
            if dist < min_dist_m or dist > max_dist_m:
                continue
            yaw = math.atan2(fy - ry, fx - rx)
            edge = is_frontier_edge_cell(grid, mx, my, cfg)
            entry = (fx, fy, yaw, edge)
            if edge:
                edges.append(entry)
            else:
                interior.append(entry)
    edges.sort(key=lambda t: math.hypot(t[0] - rx, t[1] - ry))
    interior.sort(key=lambda t: math.hypot(t[0] - rx, t[1] - ry))
    merged = edges + interior
    return merged[:max_count]
