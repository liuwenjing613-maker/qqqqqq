#!/usr/bin/env python3
"""Grid A* path planner for semantic explore (optional P2)."""

from __future__ import annotations

import heapq
import math
from typing import Any, Dict, List, Optional, Tuple


def plan_path(
    grid: Any,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    cfg: Dict[str, Any],
) -> List[Tuple[float, float]]:
    if grid is None or not grid.data:
        return []

    unknown_value = int(cfg.get("unknown_value", -1))
    free_threshold = int(cfg.get("free_threshold", 20))
    occupied_threshold = int(cfg.get("occupied_threshold", 65))
    allow_unknown = bool(cfg.get("allow_unknown", False))
    inflation_m = float(cfg.get("inflation_radius_m", 0.25))
    res = float(grid.info.resolution)
    inflation_cells = max(1, int(math.ceil(inflation_m / res)))

    w = int(grid.info.width)
    h = int(grid.info.height)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)

    def to_map(wx: float, wy: float) -> Tuple[int, int]:
        return int((wx - ox) / res), int((wy - oy) / res)

    def to_world(mx: int, my: int) -> Tuple[float, float]:
        return ox + (mx + 0.5) * res, oy + (my + 0.5) * res

    def cell_val(mx: int, my: int) -> int:
        if mx < 0 or my < 0 or mx >= w or my >= h:
            return 100
        return int(grid.data[my * w + mx])

    def walkable(mx: int, my: int) -> bool:
        val = cell_val(mx, my)
        if val >= occupied_threshold:
            return False
        if val == unknown_value or val < 0:
            return allow_unknown
        if val > free_threshold:
            return False
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                nx, ny = mx + dx, my + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if cell_val(nx, ny) >= occupied_threshold:
                    return False
        return True

    start = to_map(*start_xy)
    goal = to_map(*goal_xy)
    if not walkable(*start) or not walkable(*goal):
        return []

    open_heap: List[Tuple[float, int, Tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, 0, start))
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}
    counter = 0

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            path_cells = [current]
            while current in came_from:
                current = came_from[current]
                path_cells.append(current)
            path_cells.reverse()
            return [to_world(mx, my) for mx, my in path_cells]

        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
            nb = (current[0] + dx, current[1] + dy)
            if not walkable(*nb):
                continue
            step = 1.414 if dx and dy else 1.0
            val = cell_val(*nb)
            near_cost = 0.0
            if val > free_threshold // 2:
                near_cost = 2.0
            tentative = g_score[current] + step + near_cost
            if tentative < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = tentative
                counter += 1
                f = tentative + heuristic(nb, goal)
                heapq.heappush(open_heap, (f, counter, nb))

    return []
