#!/usr/bin/env python3
"""Extract frontier clusters from OccupancyGrid and generate observation goals."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class FrontierGoal:
    frontier_id: str
    cluster_center: Tuple[float, float]
    goal_xy: Tuple[float, float]
    goal_yaw: float
    unknown_gain: float
    reachability: float
    distance_m: float
    cluster_size: int
    cells: List[Tuple[int, int]] = field(default_factory=list)


def _cell_value(grid: Any, mx: int, my: int, unknown_value: int) -> int:
    w = int(grid.info.width)
    h = int(grid.info.height)
    if mx < 0 or my < 0 or mx >= w or my >= h:
        return 100
    idx = my * w + mx
    if idx < 0 or idx >= len(grid.data):
        return 100
    return int(grid.data[idx])


def _world_to_map(grid: Any, wx: float, wy: float) -> Tuple[int, int]:
    res = float(grid.info.resolution)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    mx = int((wx - ox) / res)
    my = int((wy - oy) / res)
    return mx, my


def _map_to_world(grid: Any, mx: int, my: int) -> Tuple[float, float]:
    res = float(grid.info.resolution)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    return ox + (mx + 0.5) * res, oy + (my + 0.5) * res


def _is_free(val: int, free_threshold: int, occupied_threshold: int, unknown_value: int) -> bool:
    if val == unknown_value:
        return False
    if val < 0:
        return False
    return val <= free_threshold


def _is_unknown(val: int, unknown_value: int) -> bool:
    return val == unknown_value or val < 0


def _is_occupied(val: int, occupied_threshold: int) -> bool:
    return val >= occupied_threshold


def extract_frontiers(
    grid: Any,
    robot_xy: Tuple[float, float],
    cfg: Dict[str, Any],
    scan_ranges: Optional[List[float]] = None,
    scan_angles: Optional[List[float]] = None,
) -> List[FrontierGoal]:
    if grid is None or not grid.data:
        return []

    unknown_value = int(cfg.get("unknown_value", -1))
    free_threshold = int(cfg.get("free_threshold", 20))
    occupied_threshold = int(cfg.get("occupied_threshold", 65))
    min_cluster = int(cfg.get("min_cluster_cells", 4))
    max_frontiers = int(cfg.get("max_frontiers", 12))
    inflation_m = float(cfg.get("inflation_radius_m", 0.25))
    min_dist = float(cfg.get("min_distance_m", 0.45))
    max_dist = float(cfg.get("max_distance_m", 3.0))
    standoff_m = float(cfg.get("observation_standoff_m", 0.65))
    gain_radius_m = float(cfg.get("unknown_gain_radius_m", 0.6))

    res = float(grid.info.resolution)
    w = int(grid.info.width)
    h = int(grid.info.height)
    inflation_cells = max(1, int(math.ceil(inflation_m / res)))
    gain_cells = max(1, int(math.ceil(gain_radius_m / res)))

    frontier_cells: List[Tuple[int, int]] = []
    for my in range(h):
        for mx in range(w):
            val = _cell_value(grid, mx, my, unknown_value)
            if not _is_free(val, free_threshold, occupied_threshold, unknown_value):
                continue
            has_unknown = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nv = _cell_value(grid, mx + dx, my + dy, unknown_value)
                    if _is_unknown(nv, unknown_value):
                        has_unknown = True
                        break
                if has_unknown:
                    break
            if has_unknown:
                frontier_cells.append((mx, my))

    visited: set = set()
    clusters: List[List[Tuple[int, int]]] = []
    cell_set = set(frontier_cells)
    for start in frontier_cells:
        if start in visited:
            continue
        cluster: List[Tuple[int, int]] = []
        q = deque([start])
        visited.add(start)
        while q:
            cx, cy = q.popleft()
            cluster.append((cx, cy))
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (cx + dx, cy + dy)
                if nb in cell_set and nb not in visited:
                    visited.add(nb)
                    q.append(nb)
        if len(cluster) >= min_cluster:
            clusters.append(cluster)

    goals: List[FrontierGoal] = []
    for idx, cluster in enumerate(clusters[: max_frontiers * 2]):
        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)
        fwx, fwy = _map_to_world(grid, int(cx), int(cy))

        goal_xy = _find_standoff_goal(
            grid,
            fwx,
            fwy,
            robot_xy,
            standoff_m,
            inflation_cells,
            free_threshold,
            occupied_threshold,
            unknown_value,
        )
        if goal_xy is None:
            continue
        dist = math.hypot(goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1])
        if dist < min_dist or dist > max_dist:
            continue

        gmx, gmy = _world_to_map(grid, goal_xy[0], goal_xy[1])
        unknown_gain = _unknown_gain(
            grid, gmx, gmy, gain_cells, unknown_value, free_threshold, occupied_threshold
        )
        reach = _reachability_scan(goal_xy, robot_xy, scan_ranges, scan_angles)
        yaw = math.atan2(fwy - goal_xy[1], fwx - goal_xy[0])
        goals.append(
            FrontierGoal(
                frontier_id=f"frontier_{idx:03d}",
                cluster_center=(fwx, fwy),
                goal_xy=goal_xy,
                goal_yaw=yaw,
                unknown_gain=unknown_gain,
                reachability=reach,
                distance_m=dist,
                cluster_size=len(cluster),
                cells=cluster,
            )
        )

    goals.sort(key=lambda g: (-g.unknown_gain, -g.reachability, g.distance_m))
    return goals[:max_frontiers]


def _unknown_gain(
    grid: Any,
    mx: int,
    my: int,
    radius_cells: int,
    unknown_value: int,
    free_threshold: int,
    occupied_threshold: int,
) -> float:
    unknown_count = 0
    total = 0
    w = int(grid.info.width)
    h = int(grid.info.height)
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            nx, ny = mx + dx, my + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            total += 1
            val = _cell_value(grid, nx, ny, unknown_value)
            if _is_unknown(val, unknown_value):
                unknown_count += 1
    return unknown_count / max(total, 1)


def _find_standoff_goal(
    grid: Any,
    obj_x: float,
    obj_y: float,
    robot_xy: Tuple[float, float],
    standoff_m: float,
    inflation_cells: int,
    free_threshold: int,
    occupied_threshold: int,
    unknown_value: int,
    allow_unknown_neighbors: bool = True,
) -> Optional[Tuple[float, float]]:
    dx = robot_xy[0] - obj_x
    dy = robot_xy[1] - obj_y
    norm = math.hypot(dx, dy)
    if norm < 1e-3:
        dx, dy = 1.0, 0.0
        norm = 1.0
    ux, uy = dx / norm, dy / norm
    res = float(grid.info.resolution)
    w = int(grid.info.width)
    h = int(grid.info.height)

    for step in range(int(standoff_m / res), 0, -1):
        gx = obj_x + ux * step * res
        gy = obj_y + uy * step * res
        mx, my = _world_to_map(grid, gx, gy)
        if mx < 0 or my < 0 or mx >= w or my >= h:
            continue
        val = _cell_value(grid, mx, my, unknown_value)
        if not _is_free(val, free_threshold, occupied_threshold, unknown_value):
            continue
        if _inflated_occupied(grid, mx, my, inflation_cells, occupied_threshold, unknown_value):
            continue
        if not allow_unknown_neighbors and _has_unknown_neighbor(grid, mx, my, unknown_value):
            continue
        return gx, gy

    mx, my = _world_to_map(grid, obj_x, obj_y)
    if 0 <= mx < w and 0 <= my < h:
        val = _cell_value(grid, mx, my, unknown_value)
        if _is_free(val, free_threshold, occupied_threshold, unknown_value):
            return _map_to_world(grid, mx, my)
    return None


def _has_unknown_neighbor(grid: Any, mx: int, my: int, unknown_value: int) -> bool:
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nv = _cell_value(grid, mx + dx, my + dy, unknown_value)
            if _is_unknown(nv, unknown_value):
                return True
    return False


def _inflated_occupied(
    grid: Any,
    mx: int,
    my: int,
    inflation_cells: int,
    occupied_threshold: int,
    unknown_value: int,
) -> bool:
    w = int(grid.info.width)
    h = int(grid.info.height)
    for dy in range(-inflation_cells, inflation_cells + 1):
        for dx in range(-inflation_cells, inflation_cells + 1):
            nx, ny = mx + dx, my + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            val = _cell_value(grid, nx, ny, unknown_value)
            if _is_occupied(val, occupied_threshold):
                return True
    return False


def _reachability_scan(
    goal_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    scan_ranges: Optional[List[float]],
    scan_angles: Optional[List[float]],
) -> float:
    if not scan_ranges or not scan_angles:
        return 0.7
    bearing = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
    dist = math.hypot(goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1])
    best_clear = 0.0
    for r, a in zip(scan_ranges, scan_angles):
        if r <= 0.01 or math.isinf(r) or math.isnan(r):
            continue
        if abs((a - bearing + math.pi) % (2 * math.pi) - math.pi) < 0.35:
            best_clear = max(best_clear, r)
    if best_clear >= dist * 0.9:
        return 1.0
    if best_clear >= dist * 0.5:
        return 0.7
    if best_clear > 0.5:
        return 0.4
    return 0.1
