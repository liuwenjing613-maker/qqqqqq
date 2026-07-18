#!/usr/bin/env python3
"""Pure geometry/protocol helpers for the online full-flow V2 backend.

No ROS dependency is used here.  The real backend and the offline tests share
exactly the same frontier extraction, scoring, image rendering and Qwen output
validation code.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class RobotPose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GridMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "map"


@dataclass(frozen=True)
class FrontierConfig:
    free_max_value: int = 20
    occupied_min_value: int = 65
    obstacle_inflation_m: float = 0.30
    min_frontier_cells: int = 8
    information_radius_m: float = 0.85
    min_goal_distance_m: float = 0.50
    max_goal_distance_m: float = 2.60
    preferred_min_distance_m: float = 0.75
    preferred_max_distance_m: float = 1.80
    max_abs_relative_heading_deg: float = 150.0
    min_heading_separation_deg: float = 32.0
    max_candidates: int = 8
    stable_id_quantization_m: float = 0.15


@dataclass(frozen=True)
class FrontierCandidate:
    candidate_id: str
    x: float
    y: float
    yaw: float
    grid_x: int
    grid_y: int
    heading_deg: float
    distance_m: float
    information_gain: int
    clearance_m: float
    score: float
    cluster_cells: int

    def to_payload(self) -> Dict[str, Any]:
        return {
            "id": self.candidate_id,
            "candidate_id": self.candidate_id,
            "heading_deg": round(self.heading_deg, 3),
            "score": round(self.score, 5),
            "geometric_score": round(self.score, 5),
            "reachable": True,
            "visited": False,
            "status": "UNSEEN",
            "path_length": round(self.distance_m, 3),
            "distance_m": round(self.distance_m, 3),
            "information_gain": int(self.information_gain),
            "clearance_m": round(self.clearance_m, 3),
            "cluster_cells": int(self.cluster_cells),
            "pose": {
                "x": round(self.x, 5),
                "y": round(self.y, 5),
                "yaw": round(self.yaw, 6),
            },
            "goal_pose": {
                "x": round(self.x, 5),
                "y": round(self.y, 5),
                "yaw": round(self.yaw, 6),
            },
            "grid": {"x": int(self.grid_x), "y": int(self.grid_y)},
        }


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_grid(x: float, y: float, meta: GridMeta) -> Tuple[int, int]:
    gx = int(math.floor((x - meta.origin_x) / meta.resolution))
    gy = int(math.floor((y - meta.origin_y) / meta.resolution))
    return gx, gy


def grid_to_world(gx: int, gy: int, meta: GridMeta) -> Tuple[float, float]:
    return (
        meta.origin_x + (float(gx) + 0.5) * meta.resolution,
        meta.origin_y + (float(gy) + 0.5) * meta.resolution,
    )


def _nearest_true(mask: np.ndarray, gx: int, gy: int, radius_cells: int) -> Optional[Tuple[int, int]]:
    h, w = mask.shape
    if 0 <= gx < w and 0 <= gy < h and bool(mask[gy, gx]):
        return gx, gy
    best: Optional[Tuple[float, int, int]] = None
    for r in range(1, max(1, radius_cells) + 1):
        x0, x1 = max(0, gx - r), min(w - 1, gx + r)
        y0, y1 = max(0, gy - r), min(h - 1, gy + r)
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                if max(abs(x - gx), abs(y - gy)) != r or not bool(mask[y, x]):
                    continue
                d2 = float((x - gx) ** 2 + (y - gy) ** 2)
                if best is None or d2 < best[0]:
                    best = (d2, x, y)
        if best is not None:
            return best[1], best[2]
    return None


def _stable_candidate_id(x: float, y: float, cfg: FrontierConfig) -> str:
    q = max(0.05, cfg.stable_id_quantization_m)
    ix = int(round(x / q))
    iy = int(round(y / q))
    sx = f"m{abs(ix)}" if ix < 0 else f"p{ix}"
    sy = f"m{abs(iy)}" if iy < 0 else f"p{iy}"
    return f"F_{sx}_{sy}"


def _unknown_gain(unknown: np.ndarray, gx: int, gy: int, radius_cells: int) -> int:
    h, w = unknown.shape
    x0, x1 = max(0, gx - radius_cells), min(w, gx + radius_cells + 1)
    y0, y1 = max(0, gy - radius_cells), min(h, gy + radius_cells + 1)
    roi = unknown[y0:y1, x0:x1]
    yy, xx = np.ogrid[y0:y1, x0:x1]
    circle = (xx - gx) ** 2 + (yy - gy) ** 2 <= radius_cells ** 2
    return int(np.count_nonzero(roi & circle))


def _unknown_facing_yaw(
    unknown: np.ndarray,
    gx: int,
    gy: int,
    meta: GridMeta,
    fallback_yaw: float,
    radius_cells: int,
) -> float:
    h, w = unknown.shape
    x0, x1 = max(0, gx - radius_cells), min(w, gx + radius_cells + 1)
    y0, y1 = max(0, gy - radius_cells), min(h, gy + radius_cells + 1)
    ys, xs = np.nonzero(unknown[y0:y1, x0:x1])
    if len(xs) == 0:
        return fallback_yaw
    xs = xs.astype(np.float64) + x0
    ys = ys.astype(np.float64) + y0
    # Weight closer unknown cells more heavily so the final heading faces the
    # frontier opening instead of a remote unknown mass.
    dist = np.hypot(xs - gx, ys - gy)
    weights = 1.0 / np.maximum(1.0, dist)
    ux = float(np.average(xs, weights=weights))
    uy = float(np.average(ys, weights=weights))
    dx = (ux - gx) * meta.resolution
    dy = (uy - gy) * meta.resolution
    if math.hypot(dx, dy) < 1e-6:
        return fallback_yaw
    return math.atan2(dy, dx)


def _angle_difference_deg(a: float, b: float) -> float:
    return abs(math.degrees(wrap_angle(math.radians(a - b))))


def _build_nearest_gray_white_fallback_item(
    robot: RobotPose2D,
    meta: GridMeta,
    *,
    safe_free: np.ndarray,
    unknown: np.ndarray,
    free_labels: np.ndarray,
    robot_component: int,
    clearance_cells: np.ndarray,
    info_radius_cells: int,
) -> Optional[Dict[str, Any]]:
    """Pick the nearest gray/white-boundary cell when strict geometry filters find none.

    Gray-white means known-free (white) cells adjacent to unknown (gray). Distance and
    heading limits are intentionally not applied for this emergency fallback.
    """
    component_mask = (free_labels == robot_component) & safe_free
    unknown_near = cv2.dilate(
        unknown.astype(np.uint8), np.ones((3, 3), dtype=np.uint8), iterations=1
    ).astype(bool)
    gray_white_mask = component_mask & unknown_near
    search_mask = gray_white_mask if np.any(gray_white_mask) else component_mask
    if not np.any(search_mask):
        search_mask = safe_free
    if not np.any(search_mask):
        return None

    ys, xs = np.nonzero(search_mask)
    best_idx = int(
        np.argmin(
            [
                math.hypot(
                    grid_to_world(int(x), int(y), meta)[0] - robot.x,
                    grid_to_world(int(x), int(y), meta)[1] - robot.y,
                )
                for x, y in zip(xs, ys)
            ]
        )
    )
    gx, gy = int(xs[best_idx]), int(ys[best_idx])
    wx, wy = grid_to_world(gx, gy, meta)
    dx, dy = wx - robot.x, wy - robot.y
    distance_m = math.hypot(dx, dy)
    absolute_heading = math.atan2(dy, dx)
    relative_heading_deg = math.degrees(wrap_angle(absolute_heading - robot.yaw))
    gain = _unknown_gain(unknown, gx, gy, info_radius_cells)
    yaw = _unknown_facing_yaw(
        unknown,
        gx,
        gy,
        meta,
        fallback_yaw=absolute_heading,
        radius_cells=info_radius_cells,
    )
    return {
        "gx": gx,
        "gy": gy,
        "wx": wx,
        "wy": wy,
        "yaw": yaw,
        "distance": distance_m,
        "heading": relative_heading_deg,
        "gain": gain,
        "clearance": float(clearance_cells[gy, gx] * meta.resolution),
        "count": 1,
    }


def extract_frontier_candidates(
    occupancy: np.ndarray,
    meta: GridMeta,
    robot: RobotPose2D,
    cfg: FrontierConfig,
) -> Tuple[List[FrontierCandidate], Dict[str, Any]]:
    """Extract safe, connected and directionally diverse frontier candidates.

    Occupancy uses ROS OccupancyGrid semantics: ``-1`` unknown, ``0`` free,
    and larger positive values increasingly occupied.
    """
    if occupancy.shape != (meta.height, meta.width):
        raise ValueError(
            f"occupancy shape {occupancy.shape} != {(meta.height, meta.width)}"
        )
    if meta.resolution <= 0:
        raise ValueError("map resolution must be positive")

    occ = occupancy.astype(np.int16, copy=False)
    unknown = occ < 0
    free = (occ >= 0) & (occ <= cfg.free_max_value)
    blocked = occ >= cfg.occupied_min_value

    # Distance is measured from occupied cells. Unknown cells are not treated
    # as walls here, but goals still have to lie on known free cells.
    non_blocked_u8 = (~blocked).astype(np.uint8)
    clearance_cells = cv2.distanceTransform(non_blocked_u8, cv2.DIST_L2, 5)
    inflation_cells = max(1.0, cfg.obstacle_inflation_m / meta.resolution)
    safe_free = free & (clearance_cells >= inflation_cells)

    robot_gx, robot_gy = world_to_grid(robot.x, robot.y, meta)
    nearest = _nearest_true(
        safe_free,
        robot_gx,
        robot_gy,
        radius_cells=max(4, int(round(0.45 / meta.resolution))),
    )
    diagnostics: Dict[str, Any] = {
        "free_cells": int(np.count_nonzero(free)),
        "safe_free_cells": int(np.count_nonzero(safe_free)),
        "unknown_cells": int(np.count_nonzero(unknown)),
        "frontier_cells": 0,
        "raw_clusters": 0,
        "reachable_clusters": 0,
        "robot_grid": [robot_gx, robot_gy],
    }
    if nearest is None:
        diagnostics["failure"] = "robot_not_near_safe_free"
        return [], diagnostics
    robot_gx, robot_gy = nearest
    diagnostics["robot_grid_safe"] = [robot_gx, robot_gy]

    n_components, free_labels = cv2.connectedComponents(safe_free.astype(np.uint8), connectivity=8)
    robot_component = int(free_labels[robot_gy, robot_gx])
    if robot_component <= 0 or n_components <= 1:
        diagnostics["failure"] = "robot_safe_component_missing"
        return [], diagnostics

    unknown_near = cv2.dilate(
        unknown.astype(np.uint8), np.ones((3, 3), dtype=np.uint8), iterations=1
    ).astype(bool)
    frontier = safe_free & unknown_near
    diagnostics["frontier_cells"] = int(np.count_nonzero(frontier))
    n_frontier, frontier_labels, stats, _ = cv2.connectedComponentsWithStats(
        frontier.astype(np.uint8), connectivity=8
    )
    diagnostics["raw_clusters"] = max(0, int(n_frontier - 1))

    info_radius_cells = max(2, int(round(cfg.information_radius_m / meta.resolution)))
    raw: List[Dict[str, Any]] = []
    for label in range(1, n_frontier):
        count = int(stats[label, cv2.CC_STAT_AREA])
        if count < cfg.min_frontier_cells:
            continue
        ys, xs = np.nonzero(frontier_labels == label)
        if len(xs) == 0:
            continue
        # Keep only the part connected to the robot's known-free component.
        connected_mask = free_labels[ys, xs] == robot_component
        xs = xs[connected_mask]
        ys = ys[connected_mask]
        if len(xs) < cfg.min_frontier_cells:
            continue
        diagnostics["reachable_clusters"] += 1

        # One connected frontier can wrap around a large explored region. Using
        # one centroid for the whole component would collapse a real T-junction
        # into a single option. Split every component into robot-centric angular
        # groups, then choose one safe representative per group.
        sector_width = max(18.0, min(45.0, cfg.min_heading_separation_deg))
        groups: Dict[int, List[int]] = {}
        cell_headings: List[float] = []
        cell_distances: List[float] = []
        for cell_index, (cell_x, cell_y) in enumerate(zip(xs, ys)):
            wx_i, wy_i = grid_to_world(int(cell_x), int(cell_y), meta)
            dx_i, dy_i = wx_i - robot.x, wy_i - robot.y
            distance_i = math.hypot(dx_i, dy_i)
            heading_i = math.degrees(wrap_angle(math.atan2(dy_i, dx_i) - robot.yaw))
            cell_headings.append(heading_i)
            cell_distances.append(distance_i)
            if not (cfg.min_goal_distance_m <= distance_i <= cfg.max_goal_distance_m):
                continue
            if abs(heading_i) > cfg.max_abs_relative_heading_deg:
                continue
            sector = int(math.floor((heading_i + 180.0) / sector_width))
            groups.setdefault(sector, []).append(cell_index)

        for indices in groups.values():
            if len(indices) < max(2, cfg.min_frontier_cells // 3):
                continue
            group_xs = xs[indices]
            group_ys = ys[indices]
            cx, cy = float(np.mean(group_xs)), float(np.mean(group_ys))
            local_clearance = clearance_cells[group_ys, group_xs]
            centroid_cost = np.hypot(group_xs - cx, group_ys - cy)
            metric = local_clearance - 0.10 * centroid_cost
            local_idx = int(np.argmax(metric))
            source_idx = indices[local_idx]
            gx, gy = int(xs[source_idx]), int(ys[source_idx])
            wx, wy = grid_to_world(gx, gy, meta)
            dx, dy = wx - robot.x, wy - robot.y
            distance_m = math.hypot(dx, dy)
            absolute_heading = math.atan2(dy, dx)
            relative_heading_deg = math.degrees(wrap_angle(absolute_heading - robot.yaw))
            gain = _unknown_gain(unknown, gx, gy, info_radius_cells)
            yaw = _unknown_facing_yaw(
                unknown,
                gx,
                gy,
                meta,
                fallback_yaw=absolute_heading,
                radius_cells=info_radius_cells,
            )
            raw.append(
                {
                    "gx": gx,
                    "gy": gy,
                    "wx": wx,
                    "wy": wy,
                    "yaw": yaw,
                    "distance": distance_m,
                    "heading": relative_heading_deg,
                    "gain": gain,
                    "clearance": float(clearance_cells[gy, gx] * meta.resolution),
                    "count": len(indices),
                }
            )

    if not raw:
        fallback = _build_nearest_gray_white_fallback_item(
            robot,
            meta,
            safe_free=safe_free,
            unknown=unknown,
            free_labels=free_labels,
            robot_component=robot_component,
            clearance_cells=clearance_cells,
            info_radius_cells=info_radius_cells,
        )
        if fallback is None:
            diagnostics["failure"] = "no_candidate_after_geometry_filters"
            return [], diagnostics
        raw = [fallback]
        diagnostics["nearest_gray_white_fallback"] = True
        diagnostics["fallback_reason"] = "no_candidate_after_geometry_filters"

    max_gain = max(1, max(int(item["gain"]) for item in raw))
    max_clearance = max(0.05, max(float(item["clearance"]) for item in raw))
    scored: List[FrontierCandidate] = []
    for item in raw:
        distance = float(item["distance"])
        if cfg.preferred_min_distance_m <= distance <= cfg.preferred_max_distance_m:
            distance_pref = 1.0
        elif distance < cfg.preferred_min_distance_m:
            distance_pref = max(0.0, distance / max(cfg.preferred_min_distance_m, 1e-6))
        else:
            span = max(0.1, cfg.max_goal_distance_m - cfg.preferred_max_distance_m)
            distance_pref = max(0.0, 1.0 - (distance - cfg.preferred_max_distance_m) / span)
        info_norm = float(item["gain"]) / max_gain
        clearance_norm = min(1.0, float(item["clearance"]) / max_clearance)
        frontness = 1.0 - min(1.0, abs(float(item["heading"])) / 180.0)
        # Deliberately keep the score range compact. At a real branch, geometry
        # should nominate safe options and Qwen should resolve semantic ties.
        composite = 0.45 * info_norm + 0.25 * distance_pref + 0.20 * clearance_norm + 0.10 * frontness
        score = 0.50 + 0.18 * composite
        candidate_id = _stable_candidate_id(float(item["wx"]), float(item["wy"]), cfg)
        scored.append(
            FrontierCandidate(
                candidate_id=candidate_id,
                x=float(item["wx"]),
                y=float(item["wy"]),
                yaw=float(item["yaw"]),
                grid_x=int(item["gx"]),
                grid_y=int(item["gy"]),
                heading_deg=float(item["heading"]),
                distance_m=distance,
                information_gain=int(item["gain"]),
                clearance_m=float(item["clearance"]),
                score=float(score),
                cluster_cells=int(item["count"]),
            )
        )

    scored.sort(key=lambda c: (-c.score, c.distance_m, abs(c.heading_deg), c.candidate_id))
    diverse: List[FrontierCandidate] = []
    for candidate in scored:
        if any(
            _angle_difference_deg(candidate.heading_deg, kept.heading_deg)
            < cfg.min_heading_separation_deg
            for kept in diverse
        ):
            continue
        diverse.append(candidate)
        if len(diverse) >= cfg.max_candidates:
            break

    # A single geometric cluster can be represented by several nearby stable
    # cells after a map update; deduplicate IDs as the final guard.
    unique: Dict[str, FrontierCandidate] = {}
    for candidate in diverse:
        unique.setdefault(candidate.candidate_id, candidate)
    result = list(unique.values())
    diagnostics.update(
        {
            "filtered_clusters": len(raw),
            "candidate_count": len(result),
            "robot_component": robot_component,
        }
    )
    return result, diagnostics


def candidate_summary_payload(
    candidates: Sequence[FrontierCandidate],
    *,
    map_version: str,
    probe_id: Optional[str],
    diagnostics: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "schema_version": "online_frontier_candidate_summary_v2",
        "probe_id": probe_id,
        "map_version": map_version,
        "decision_distance_m": (
            None if not candidates else round(min(c.distance_m for c in candidates), 3)
        ),
        "returned_to_junction": False,
        "unseen_candidate_count": len(candidates),
        "candidates": [candidate.to_payload() for candidate in candidates],
        "diagnostics": diagnostics or {},
    }


def choose_geometric_candidate(candidates: Sequence[FrontierCandidate]) -> FrontierCandidate:
    if not candidates:
        raise ValueError("no candidate")
    return sorted(
        candidates,
        key=lambda c: (-c.score, c.distance_m, abs(c.heading_deg), c.candidate_id),
    )[0]


def parse_qwen_candidate_id(raw_text: str, allowed_ids: Iterable[str]) -> Tuple[str, Dict[str, Any]]:
    allowed = {str(value) for value in allowed_ids}
    text = str(raw_text or "").strip()
    if text.startswith("```"):
        text = text.strip("`")
        if text.lower().startswith("json"):
            text = text[4:].lstrip()
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("Qwen response has no JSON object")
    payload = json.loads(text[start : end + 1])
    candidate_id = str(
        payload.get("candidate_id", payload.get("selected_candidate_id", payload.get("id", "")))
    ).strip()
    if candidate_id not in allowed:
        raise ValueError(f"candidate_id={candidate_id!r} is not in allowed set")
    return candidate_id, payload


def render_candidate_map(
    occupancy: np.ndarray,
    meta: GridMeta,
    robot: RobotPose2D,
    candidates: Sequence[FrontierCandidate],
    selected_id: Optional[str] = None,
    max_side: int = 1280,
) -> np.ndarray:
    h, w = occupancy.shape
    image = np.full((h, w, 3), 128, dtype=np.uint8)
    image[occupancy >= 65] = (0, 0, 0)
    image[(occupancy >= 0) & (occupancy <= 20)] = (255, 255, 255)
    # OccupancyGrid row zero is the map's lower edge; images use top-left origin.
    image = np.flipud(image).copy()

    rgx, rgy = world_to_grid(robot.x, robot.y, meta)
    rpx, rpy = rgx, h - 1 - rgy
    radius = max(4, int(round(max(h, w) * 0.008)))
    if 0 <= rpx < w and 0 <= rpy < h:
        cv2.circle(image, (rpx, rpy), radius, (220, 90, 30), -1, cv2.LINE_AA)
        arrow_len = max(14, int(round(max(h, w) * 0.045)))
        ex = int(round(rpx + arrow_len * math.cos(robot.yaw)))
        ey = int(round(rpy - arrow_len * math.sin(robot.yaw)))
        cv2.arrowedLine(image, (rpx, rpy), (ex, ey), (30, 30, 220), 2, cv2.LINE_AA, tipLength=0.25)

    for index, candidate in enumerate(candidates, start=1):
        px, py = candidate.grid_x, h - 1 - candidate.grid_y
        selected = candidate.candidate_id == selected_id
        color = (40, 180, 40) if selected else (0, 205, 255)
        cv2.circle(image, (px, py), radius, color, -1, cv2.LINE_AA)
        label = f"{index}:{candidate.candidate_id}"
        cv2.putText(
            image,
            label,
            (px + radius + 2, py - radius - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            (30, 30, 30),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            label,
            (px + radius + 2, py - radius - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            color,
            1,
            cv2.LINE_AA,
        )

    longest = max(h, w)
    if longest > max_side:
        scale = float(max_side) / float(longest)
        image = cv2.resize(
            image,
            (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
            interpolation=cv2.INTER_AREA,
        )
    return image


def candidate_table(candidates: Sequence[FrontierCandidate]) -> str:
    lines = []
    for index, c in enumerate(candidates, start=1):
        direction = (
            "front"
            if abs(c.heading_deg) < 25
            else "left"
            if c.heading_deg > 0
            else "right"
        )
        lines.append(
            f"{index}. id={c.candidate_id}; direction={direction}; "
            f"relative_heading_deg={c.heading_deg:.1f}; distance_m={c.distance_m:.2f}; "
            f"information_gain={c.information_gain}; clearance_m={c.clearance_m:.2f}; "
            f"geometric_score={c.score:.3f}"
        )
    return "\n".join(lines)


def candidate_from_payload(payload: Dict[str, Any], meta: GridMeta) -> Optional[FrontierCandidate]:
    try:
        cid = str(payload.get("id", payload.get("candidate_id", ""))).strip()
        pose = payload.get("goal_pose", payload.get("pose", {})) or {}
        x = float(pose["x"])
        y = float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))
        gx, gy = world_to_grid(x, y, meta)
        return FrontierCandidate(
            candidate_id=cid,
            x=x,
            y=y,
            yaw=yaw,
            grid_x=gx,
            grid_y=gy,
            heading_deg=float(payload.get("heading_deg", 0.0) or 0.0),
            distance_m=float(payload.get("distance_m", payload.get("path_length", 0.0)) or 0.0),
            information_gain=int(payload.get("information_gain", 0) or 0),
            clearance_m=float(payload.get("clearance_m", 0.0) or 0.0),
            score=float(payload.get("score", payload.get("geometric_score", 0.5)) or 0.5),
            cluster_cells=int(payload.get("cluster_cells", 0) or 0),
        )
    except (KeyError, TypeError, ValueError):
        return None


def candidate_is_currently_safe(
    candidate: FrontierCandidate,
    occupancy: np.ndarray,
    meta: GridMeta,
    cfg: FrontierConfig,
) -> bool:
    gx, gy = world_to_grid(candidate.x, candidate.y, meta)
    if not (0 <= gx < meta.width and 0 <= gy < meta.height):
        return False
    value = int(occupancy[gy, gx])
    if value < 0 or value > cfg.free_max_value:
        return False
    blocked = occupancy >= cfg.occupied_min_value
    clearance = cv2.distanceTransform((~blocked).astype(np.uint8), cv2.DIST_L2, 5)
    return float(clearance[gy, gx] * meta.resolution) >= cfg.obstacle_inflation_m
