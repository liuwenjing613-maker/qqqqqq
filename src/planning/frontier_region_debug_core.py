#!/usr/bin/env python3
"""Pure-computation frontier region analysis for map exploration diagnostics."""

from __future__ import annotations

import copy
import math
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import cv2  # type: ignore

    _CV2_AVAILABLE = True
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    _CV2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class MapMetadata:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    stamp_sec: float


@dataclass
class RobotPose2D:
    x: float
    y: float
    yaw_rad: float


@dataclass
class MapHealthResult:
    ok: bool
    status: str
    errors: List[Dict[str, Any]] = field(default_factory=list)
    warnings: List[Dict[str, Any]] = field(default_factory=list)
    width: int = 0
    height: int = 0
    expected_cells: int = 0
    actual_cells: int = 0
    free_cells: int = 0
    occupied_cells: int = 0
    unknown_cells: int = 0
    other_cells: int = 0
    free_ratio: float = 0.0
    occupied_ratio: float = 0.0
    unknown_ratio: float = 0.0
    robot_grid_row: Optional[int] = None
    robot_grid_col: Optional[int] = None
    robot_cell_value: Optional[int] = None


@dataclass
class FrontierExtractionStats:
    free_cells_examined: int = 0
    raw_frontier_cells: int = 0
    removed_low_clearance: int = 0
    remaining_frontier_cells: int = 0
    raw_cluster_count: int = 0
    merge_pairs_considered: int = 0
    merge_pairs_accepted: int = 0
    merged_group_count: int = 0
    cluster_count_after_merge: int = 0
    clusters_too_small: int = 0
    accepted_region_count: int = 0
    rejected_region_count: int = 0
    accepted_regions_before_rank_limit: int = 0
    accepted_regions_after_rank_limit: int = 0
    analysis_time_ms: float = 0.0
    status: str = "OK"


@dataclass
class RegionRejection:
    code: str
    actual: Any
    threshold: Any
    detail: str = ""


@dataclass
class FrontierRegion:
    region_id: str
    accepted: bool
    rejection_reasons: List[RegionRejection] = field(default_factory=list)
    frontier_cell_count: int = 0
    frontier_length_m: float = 0.0
    row_min: int = 0
    row_max: int = 0
    col_min: int = 0
    col_max: int = 0
    centroid_row: float = 0.0
    centroid_col: float = 0.0
    centroid_x: float = 0.0
    centroid_y: float = 0.0
    nearest_row: int = 0
    nearest_col: int = 0
    nearest_x: float = 0.0
    nearest_y: float = 0.0
    distance_to_robot_m: float = 0.0
    bearing_global_deg: float = 0.0
    bearing_relative_deg: float = 0.0
    direction_label: str = ""
    unknown_gain_cells: int = 0
    unknown_gain_ratio: float = 0.0
    minimum_clearance_m: float = 0.0
    mean_clearance_m: float = 0.0
    visited_count: int = 0
    navigation_failure_count: int = 0
    blacklisted: bool = False
    diagnostic_priority: float = 0.0
    path_checked: bool = False
    reachable: Optional[bool] = None
    frontier_cells: List[Tuple[int, int]] = field(default_factory=list)
    merged: bool = False
    source_cluster_ids: List[str] = field(default_factory=list)
    merge_reasons: List[Dict[str, Any]] = field(default_factory=list)
    snapshot_eligible: bool = False
    track_id: str = ""
    stable: bool = False
    persistence_cycles: int = 0
    age_s: float = 0.0
    centroid_drift_m: float = 0.0
    bearing_drift_deg: float = 0.0
    guard_cell_count_change_ratio: float = 0.0
    stability_rejection_reasons: List[str] = field(default_factory=list)
    near_robot_penalty: float = 0.0
    recent_observation_penalty: float = 0.0
    distance_to_nearest_observation_pose_m: float = float("inf")
    nearest_trajectory_distance_m: float = float("inf")
    nearby_trajectory_vertex_count: int = 0
    nearby_recent_trajectory_count: int = 0
    last_nearby_visit_age_s: float = float("inf")
    trajectory_density_score: float = 0.0
    trajectory_novelty_score: float = 1.0
    trajectory_revisit_penalty: float = 0.0
    geo_score_before_trajectory: float = 0.0
    geo_score_after_trajectory: float = 0.0
    geo_score: float = 0.0
    geo_rank: int = 0
    score_components: Dict[str, float] = field(default_factory=dict)
    penalty_components: Dict[str, float] = field(default_factory=dict)
    score_explanation: str = ""


@dataclass
class MergeLogEntry:
    cycle_id: int
    source_clusters: List[str]
    frontier_gap_m: float
    centroid_distance_m: float
    bearing_difference_deg: float
    direction_labels: List[str]
    result_cluster: str
    result_frontier_cells: int
    decision: str = "MERGE"


@dataclass
class FrontierAnalysisResult:
    cycle_id: int
    map_health: MapHealthResult
    stats: FrontierExtractionStats
    regions: List[FrontierRegion] = field(default_factory=list)
    rejected_regions: List[FrontierRegion] = field(default_factory=list)
    raw_frontier_mask: Optional[np.ndarray] = None
    filtered_frontier_mask: Optional[np.ndarray] = None
    merge_log: List[MergeLogEntry] = field(default_factory=list)
    merge_reject_log: List[Dict[str, Any]] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Map classification helpers
# ---------------------------------------------------------------------------

NEIGH8 = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)


def _map_values_cfg(cfg: Dict[str, Any]) -> Tuple[int, int, int]:
    mv = cfg.get("map_values", {})
    unknown_value = int(mv.get("unknown_value", -1))
    free_max = int(mv.get("free_max", 20))
    occupied_min = int(mv.get("occupied_min", 65))
    return unknown_value, free_max, occupied_min


def classify_map_cells(
    data: np.ndarray,
    unknown_value: int,
    free_max: int,
    occupied_min: int,
) -> Tuple[np.ndarray, Dict[str, int]]:
    """Return category array: 0=free, 1=occupied, 2=unknown, 3=other."""
    flat = data.reshape(-1)
    cats = np.full(flat.shape[0], 3, dtype=np.int8)
    cats[flat == unknown_value] = 2
    cats[(flat >= 0) & (flat <= free_max)] = 0
    cats[flat >= occupied_min] = 1
    # unknown_value may overlap negative; re-apply
    cats[flat == unknown_value] = 2
    cats[(flat < 0) & (flat != unknown_value)] = 2
    counts = {
        "free": int(np.sum(cats == 0)),
        "occupied": int(np.sum(cats == 1)),
        "unknown": int(np.sum(cats == 2)),
        "other": int(np.sum(cats == 3)),
    }
    return cats.reshape(data.shape), counts


VISITED_OCCUPANCY_VALUE = 40
OCCUPIED_VIZ_VALUE = 99


def merge_map_with_visited_corridor(
    map_data: np.ndarray,
    visited_flat: Sequence[int],
    *,
    unknown_value: int = -1,
    free_max: int = 20,
    occupied_min: int = 65,
    visited_occ_value: int = VISITED_OCCUPANCY_VALUE,
    occupied_viz_value: int = OCCUPIED_VIZ_VALUE,
) -> List[int]:
    """Write visited corridor into map grid cells (free cells only)."""
    cats, _ = classify_map_cells(map_data, unknown_value, free_max, occupied_min)
    flat = map_data.reshape(-1)
    cat_flat = cats.reshape(-1)
    visited = list(visited_flat)
    if len(visited) != flat.size:
        raise ValueError(f"visited_flat length {len(visited)} != map cells {flat.size}")
    out: List[int] = []
    for i, _raw in enumerate(flat):
        if visited[i] >= 100 and cat_flat[i] == 0:
            out.append(int(visited_occ_value))
        elif cat_flat[i] == 0:
            out.append(0)
        elif cat_flat[i] == 1:
            out.append(int(occupied_viz_value))
        elif cat_flat[i] == 2:
            out.append(int(unknown_value))
        else:
            out.append(int(flat[i]))
    return out


def world_to_grid(
    x: float,
    y: float,
    meta: MapMetadata,
) -> Tuple[int, int]:
    if meta.resolution <= 0:
        raise ValueError("resolution must be > 0")
    col = int((x - meta.origin_x) / meta.resolution)
    row = int((y - meta.origin_y) / meta.resolution)
    return row, col


def grid_to_world(
    row: int,
    col: int,
    meta: MapMetadata,
) -> Tuple[float, float]:
    x = meta.origin_x + (col + 0.5) * meta.resolution
    y = meta.origin_y + (row + 0.5) * meta.resolution
    return x, y


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def relative_bearing_to_direction(relative_bearing_rad: float) -> str:
    deg = math.degrees(normalize_angle_rad(relative_bearing_rad))
    if -22.5 <= deg < 22.5:
        return "FRONT"
    if 22.5 <= deg < 67.5:
        return "FRONT_LEFT"
    if 67.5 <= deg < 112.5:
        return "LEFT"
    if 112.5 <= deg < 157.5:
        return "BACK_LEFT"
    if deg >= 157.5 or deg < -157.5:
        return "BACK"
    if -157.5 <= deg < -112.5:
        return "BACK_RIGHT"
    if -112.5 <= deg < -67.5:
        return "RIGHT"
    return "FRONT_RIGHT"


def _error(code: str, detail: str = "", **extra: Any) -> Dict[str, Any]:
    out: Dict[str, Any] = {"code": code, "detail": detail}
    out.update(extra)
    return out


def validate_map(
    data: np.ndarray,
    meta: MapMetadata,
    robot: Optional[RobotPose2D],
    cfg: Dict[str, Any],
) -> MapHealthResult:
    unknown_value, free_max, occupied_min = _map_values_cfg(cfg)
    mh_cfg = cfg.get("map_health", {})
    require_inside = bool(mh_cfg.get("require_robot_inside_map", True))
    reject_occ = bool(mh_cfg.get("reject_robot_cell_occupied", True))
    reject_unknown = bool(mh_cfg.get("reject_robot_cell_unknown", False))

    errors: List[Dict[str, Any]] = []
    warnings: List[Dict[str, Any]] = []

    if meta.width <= 0 or meta.height <= 0:
        errors.append(_error("MAP_INVALID_DIMENSIONS", width=meta.width, height=meta.height))
    if meta.resolution <= 0:
        errors.append(_error("MAP_INVALID_RESOLUTION", resolution=meta.resolution))

    expected = meta.width * meta.height
    actual = int(data.size)
    if expected != actual:
        errors.append(
            _error(
                "MAP_DATA_SIZE_MISMATCH",
                expected=expected,
                actual=actual,
            )
        )

    if data.size == 0:
        errors.append(_error("MAP_EMPTY"))
        return MapHealthResult(
            ok=False,
            status="REJECTED",
            errors=errors,
            warnings=warnings,
            width=meta.width,
            height=meta.height,
            expected_cells=expected,
            actual_cells=actual,
        )

    _, counts = classify_map_cells(data, unknown_value, free_max, occupied_min)
    total = max(actual, 1)
    result = MapHealthResult(
        ok=True,
        status="OK",
        errors=errors,
        warnings=warnings,
        width=meta.width,
        height=meta.height,
        expected_cells=expected,
        actual_cells=actual,
        free_cells=counts["free"],
        occupied_cells=counts["occupied"],
        unknown_cells=counts["unknown"],
        other_cells=counts["other"],
        free_ratio=counts["free"] / total,
        occupied_ratio=counts["occupied"] / total,
        unknown_ratio=counts["unknown"] / total,
    )

    if errors:
        result.ok = False
        result.status = "REJECTED"
        return result

    if robot is not None:
        row, col = world_to_grid(robot.x, robot.y, meta)
        result.robot_grid_row = row
        result.robot_grid_col = col
        if row < 0 or col < 0 or row >= meta.height or col >= meta.width:
            errors.append(
                _error(
                    "ROBOT_OUTSIDE_MAP",
                    row=row,
                    col=col,
                    robot_x=robot.x,
                    robot_y=robot.y,
                )
            )
        else:
            cell_val = int(data[row, col])
            result.robot_cell_value = cell_val
            cats, _ = classify_map_cells(
                np.array([[cell_val]], dtype=np.int16),
                unknown_value,
                free_max,
                occupied_min,
            )
            cat = int(cats[0, 0])
            if require_inside and (row < 0 or col < 0 or row >= meta.height or col >= meta.width):
                pass  # already flagged
            if cat == 1 and reject_occ:
                errors.append(_error("ROBOT_CELL_OCCUPIED", cell_value=cell_val))
            if cat == 2 and reject_unknown:
                errors.append(_error("ROBOT_CELL_UNKNOWN", cell_value=cell_val))
            if cat == 3:
                warnings.append(_error("ROBOT_CELL_OTHER", cell_value=cell_val))

    if errors:
        result.ok = False
        result.status = "REJECTED"
        result.errors = errors
    elif warnings:
        result.status = "WARN"

    return result


def extract_raw_frontier_mask(
    data: np.ndarray,
    cfg: Dict[str, Any],
) -> Tuple[np.ndarray, int]:
    """Return boolean mask (H,W) and free_cells_examined count."""
    unknown_value, free_max, occupied_min = _map_values_cfg(cfg)
    h, w = data.shape
    cats, _ = classify_map_cells(data, unknown_value, free_max, occupied_min)
    free_mask = cats == 0
    unknown_mask = cats == 2
    free_examined = int(np.sum(free_mask))
    frontier = np.zeros((h, w), dtype=bool)
    for row in range(h):
        for col in range(w):
            if not free_mask[row, col]:
                continue
            for dr, dc in NEIGH8:
                nr, nc = row + dr, col + dc
                if 0 <= nr < h and 0 <= nc < w and unknown_mask[nr, nc]:
                    frontier[row, col] = True
                    break
    return frontier, free_examined


def compute_obstacle_clearance(
    data: np.ndarray,
    meta: MapMetadata,
    cfg: Dict[str, Any],
) -> np.ndarray:
    """Distance to nearest occupied cell in meters; 0 on occupied cells."""
    unknown_value, free_max, occupied_min = _map_values_cfg(cfg)
    cats, _ = classify_map_cells(data, unknown_value, free_max, occupied_min)
    occupied = (cats == 1).astype(np.uint8)
    h, w = occupied.shape
    if h == 0 or w == 0:
        return np.zeros((h, w), dtype=np.float32)

    if _CV2_AVAILABLE and cv2 is not None:
        # distanceTransform: distance to nearest zero; invert so occupied=0, free=distance
        inv = (1 - occupied).astype(np.uint8)
        dist_cells = cv2.distanceTransform(inv, cv2.DIST_L2, 5)
        return dist_cells.astype(np.float32) * float(meta.resolution)

    return _clearance_bfs_numpy(occupied, meta.resolution)


def _clearance_bfs_numpy(occupied: np.ndarray, resolution: float) -> np.ndarray:
    h, w = occupied.shape
    dist = np.full((h, w), np.inf, dtype=np.float32)
    q: deque = deque()
    occ_rows, occ_cols = np.where(occupied > 0)
    for r, c in zip(occ_rows.tolist(), occ_cols.tolist()):
        dist[r, c] = 0.0
        q.append((r, c))
    if not q:
        return np.full((h, w), float("inf"), dtype=np.float32)
    while q:
        r, c = q.popleft()
        base = dist[r, c]
        for dr, dc in NEIGH8:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w:
                step = resolution if (dr == 0 or dc == 0) else resolution * math.sqrt(2.0)
                nd = base + step
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    q.append((nr, nc))
    return dist


def filter_frontier_by_clearance(
    frontier_mask: np.ndarray,
    clearance_m: np.ndarray,
    min_clearance_m: float,
) -> Tuple[np.ndarray, int]:
    if min_clearance_m <= 0:
        return frontier_mask.copy(), 0
    filtered = frontier_mask & (clearance_m >= min_clearance_m)
    removed = int(np.sum(frontier_mask & ~filtered))
    return filtered, removed


def connected_components_8(
    mask: np.ndarray,
) -> List[List[Tuple[int, int]]]:
    h, w = mask.shape
    visited = np.zeros((h, w), dtype=bool)
    components: List[List[Tuple[int, int]]] = []
    for row in range(h):
        for col in range(w):
            if not mask[row, col] or visited[row, col]:
                continue
            cells: List[Tuple[int, int]] = []
            q: deque = deque([(row, col)])
            visited[row, col] = True
            while q:
                r, c = q.popleft()
                cells.append((r, c))
                for dr, dc in NEIGH8:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        q.append((nr, nc))
            components.append(cells)
    return components


@dataclass
class _ClusterPreview:
    cluster_id: str
    cells: List[Tuple[int, int]]
    centroid_row: float
    centroid_col: float
    centroid_x: float
    centroid_y: float
    bearing_relative_deg: float
    direction_label: str


class _UnionFind:
    def __init__(self, size: int) -> None:
        self.parent = list(range(size))

    def find(self, x: int) -> int:
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a: int, b: int) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[rb] = ra


def _cluster_id(cycle_id: int, index: int) -> str:
    return f"C{cycle_id:03d}_{index:03d}"


def _min_frontier_gap_m(
    cells_a: List[Tuple[int, int]],
    cells_b: List[Tuple[int, int]],
    meta: MapMetadata,
) -> float:
    res = meta.resolution
    best = float("inf")
    for ra, ca in cells_a:
        ax, ay = grid_to_world(ra, ca, meta)
        for rb, cb in cells_b:
            bx, by = grid_to_world(rb, cb, meta)
            d = math.hypot(ax - bx, ay - by)
            if d < best:
                best = d
    return best


def _bearing_difference_deg(a_deg: float, b_deg: float) -> float:
    diff = abs(normalize_angle_rad(math.radians(a_deg - b_deg)))
    return math.degrees(diff)


def _build_cluster_preview(
    cells: List[Tuple[int, int]],
    cluster_id: str,
    meta: MapMetadata,
    robot: RobotPose2D,
) -> _ClusterPreview:
    rows = [c[0] for c in cells]
    cols = [c[1] for c in cells]
    centroid_row = sum(rows) / len(cells)
    centroid_col = sum(cols) / len(cells)
    cx, cy = grid_to_world(int(centroid_row), int(centroid_col), meta)
    global_bearing = math.atan2(cy - robot.y, cx - robot.x)
    relative_bearing = normalize_angle_rad(global_bearing - robot.yaw_rad)
    return _ClusterPreview(
        cluster_id=cluster_id,
        cells=cells,
        centroid_row=centroid_row,
        centroid_col=centroid_col,
        centroid_x=cx,
        centroid_y=cy,
        bearing_relative_deg=math.degrees(relative_bearing),
        direction_label=relative_bearing_to_direction(relative_bearing),
    )


def _merge_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("region_merge", {})


def _can_merge_previews(
    a: _ClusterPreview,
    b: _ClusterPreview,
    merge_cfg: Dict[str, Any],
    meta: MapMetadata,
) -> Tuple[bool, Dict[str, Any]]:
    max_gap = float(merge_cfg.get("max_frontier_gap_m", 0.20))
    max_centroid = float(merge_cfg.get("max_centroid_distance_m", 0.45))
    max_bearing = float(merge_cfg.get("max_bearing_difference_deg", 25.0))
    require_same_dir = bool(merge_cfg.get("require_same_direction_label", False))

    gap_m = _min_frontier_gap_m(a.cells, b.cells, meta)
    centroid_dist = math.hypot(a.centroid_x - b.centroid_x, a.centroid_y - b.centroid_y)
    bearing_diff = _bearing_difference_deg(a.bearing_relative_deg, b.bearing_relative_deg)

    info = {
        "cluster_a": a.cluster_id,
        "cluster_b": b.cluster_id,
        "frontier_gap_m": round(gap_m, 4),
        "centroid_distance_m": round(centroid_dist, 4),
        "bearing_difference_deg": round(bearing_diff, 4),
        "direction_labels": [a.direction_label, b.direction_label],
    }

    if gap_m > max_gap:
        info["reason"] = "FRONTIER_GAP_TOO_LARGE"
        info["threshold"] = max_gap
        return False, info
    if centroid_dist > max_centroid:
        info["reason"] = "CENTROID_DISTANCE_TOO_LARGE"
        info["threshold"] = max_centroid
        return False, info
    if bearing_diff > max_bearing:
        info["reason"] = "BEARING_DIFFERENCE_TOO_LARGE"
        info["threshold"] = max_bearing
        return False, info
    if require_same_dir and a.direction_label != b.direction_label:
        info["reason"] = "DIRECTION_LABEL_MISMATCH"
        return False, info
    return True, info


def merge_adjacent_clusters(
    raw_clusters: List[List[Tuple[int, int]]],
    meta: MapMetadata,
    robot: RobotPose2D,
    cfg: Dict[str, Any],
    cycle_id: int,
) -> Tuple[List[List[Tuple[int, int]]], List[str], List[MergeLogEntry], List[Dict[str, Any]], Dict[str, int]]:
    merge_cfg = _merge_cfg(cfg)
    stats = {
        "merge_pairs_considered": 0,
        "merge_pairs_accepted": 0,
        "merged_group_count": 0,
    }
    merge_log: List[MergeLogEntry] = []
    merge_reject_log: List[Dict[str, Any]] = []

    if not bool(merge_cfg.get("enabled", True)) or len(raw_clusters) <= 1:
        ids = [_cluster_id(cycle_id, i + 1) for i in range(len(raw_clusters))]
        return raw_clusters, ids, merge_log, merge_reject_log, stats

    previews = [
        _build_cluster_preview(cells, _cluster_id(cycle_id, i + 1), meta, robot)
        for i, cells in enumerate(raw_clusters)
    ]
    uf = _UnionFind(len(previews))
    accepted_pairs: List[Tuple[int, int, Dict[str, Any]]] = []

    for i in range(len(previews)):
        for j in range(i + 1, len(previews)):
            stats["merge_pairs_considered"] += 1
            ok, info = _can_merge_previews(previews[i], previews[j], merge_cfg, meta)
            if ok:
                uf.union(i, j)
                stats["merge_pairs_accepted"] += 1
                accepted_pairs.append((i, j, info))
            else:
                merge_reject_log.append(info)

    groups: Dict[int, List[int]] = {}
    for idx in range(len(previews)):
        root = uf.find(idx)
        groups.setdefault(root, []).append(idx)

    merged_clusters: List[List[Tuple[int, int]]] = []
    merged_ids: List[str] = []
    merge_group_idx = 0

    for indices in groups.values():
        combined: List[Tuple[int, int]] = []
        source_ids = [previews[i].cluster_id for i in indices]
        for i in indices:
            combined.extend(previews[i].cells)
        # dedupe while preserving order
        seen = set()
        unique_cells: List[Tuple[int, int]] = []
        for cell in combined:
            if cell not in seen:
                seen.add(cell)
                unique_cells.append(cell)

        if len(indices) > 1:
            stats["merged_group_count"] += 1
            merge_group_idx += 1
            result_id = f"M{cycle_id:03d}_{merge_group_idx:03d}"
            for i, j, info in accepted_pairs:
                if i in indices and j in indices:
                    merge_log.append(
                        MergeLogEntry(
                            cycle_id=cycle_id,
                            source_clusters=[previews[i].cluster_id, previews[j].cluster_id],
                            frontier_gap_m=float(info["frontier_gap_m"]),
                            centroid_distance_m=float(info["centroid_distance_m"]),
                            bearing_difference_deg=float(info["bearing_difference_deg"]),
                            direction_labels=list(info["direction_labels"]),
                            result_cluster=result_id,
                            result_frontier_cells=len(unique_cells),
                        )
                    )

        merged_clusters.append(unique_cells)
        merged_ids.append("+".join(source_ids) if len(source_ids) > 1 else source_ids[0])

    return merged_clusters, merged_ids, merge_log, merge_reject_log, stats


def _count_unknown_gain(
    data: np.ndarray,
    center_row: int,
    center_col: int,
    radius_m: float,
    meta: MapMetadata,
    cfg: Dict[str, Any],
) -> Tuple[int, float]:
    unknown_value, free_max, occupied_min = _map_values_cfg(cfg)
    cats, _ = classify_map_cells(data, unknown_value, free_max, occupied_min)
    radius_cells = max(1, int(math.ceil(radius_m / meta.resolution)))
    h, w = cats.shape
    unknown_count = 0
    total = 0
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            nr, nc = center_row + dr, center_col + dc
            if 0 <= nr < h and 0 <= nc < w:
                total += 1
                if cats[nr, nc] == 2:
                    unknown_count += 1
    ratio = unknown_count / max(total, 1)
    return unknown_count, ratio


def compute_region_metrics(
    cells: List[Tuple[int, int]],
    data: np.ndarray,
    clearance_m: np.ndarray,
    meta: MapMetadata,
    robot: RobotPose2D,
    cfg: Dict[str, Any],
    cycle_id: int,
    seq: int,
    source_cluster_ids: Optional[List[str]] = None,
    merged: bool = False,
    merge_reasons: Optional[List[Dict[str, Any]]] = None,
) -> FrontierRegion:
    region_cfg = cfg.get("region", {})
    gain_radius_m = float(region_cfg.get("unknown_gain_radius_m", 1.0))
    res = meta.resolution

    rows = [c[0] for c in cells]
    cols = [c[1] for c in cells]
    row_min, row_max = min(rows), max(rows)
    col_min, col_max = min(cols), max(cols)
    centroid_row = sum(rows) / len(cells)
    centroid_col = sum(cols) / len(cells)
    cx, cy = grid_to_world(int(centroid_row), int(centroid_col), meta)

    best_d = float("inf")
    nearest = cells[0]
    for r, c in cells:
        wx, wy = grid_to_world(r, c, meta)
        d = math.hypot(wx - robot.x, wy - robot.y)
        if d < best_d:
            best_d = d
            nearest = (r, c)
    nr, nc = nearest
    nx, ny = grid_to_world(nr, nc, meta)

    clear_vals = [float(clearance_m[r, c]) for r, c in cells]
    min_clear = min(clear_vals) if clear_vals else 0.0
    mean_clear = sum(clear_vals) / len(clear_vals) if clear_vals else 0.0

    cr = int(round(centroid_row))
    cc = int(round(centroid_col))
    gain_cells, gain_ratio = _count_unknown_gain(data, cr, cc, gain_radius_m, meta, cfg)

    global_bearing = math.atan2(cy - robot.y, cx - robot.x)
    relative_bearing = normalize_angle_rad(global_bearing - robot.yaw_rad)
    direction = relative_bearing_to_direction(relative_bearing)

    # frontier_length_m: perimeter estimate
    cell_set = set(cells)
    perimeter = 0
    for r, c in cells:
        for dr, dc in NEIGH8:
            if (r + dr, c + dc) not in cell_set:
                step = res if (dr == 0 or dc == 0) else res * math.sqrt(2.0)
                perimeter += step

    priority = gain_cells * 1.0 + len(cells) * 0.5 - best_d * 0.3

    return FrontierRegion(
        region_id=f"R{cycle_id:04d}_{seq:02d}",
        accepted=False,
        frontier_cell_count=len(cells),
        frontier_length_m=perimeter,
        row_min=row_min,
        row_max=row_max,
        col_min=col_min,
        col_max=col_max,
        centroid_row=centroid_row,
        centroid_col=centroid_col,
        centroid_x=cx,
        centroid_y=cy,
        nearest_row=nr,
        nearest_col=nc,
        nearest_x=nx,
        nearest_y=ny,
        distance_to_robot_m=best_d,
        bearing_global_deg=math.degrees(global_bearing),
        bearing_relative_deg=math.degrees(relative_bearing),
        direction_label=direction,
        unknown_gain_cells=gain_cells,
        unknown_gain_ratio=gain_ratio,
        minimum_clearance_m=min_clear,
        mean_clearance_m=mean_clear,
        diagnostic_priority=priority,
        path_checked=False,
        reachable=None,
        frontier_cells=list(cells),
        merged=merged,
        source_cluster_ids=list(source_cluster_ids or []),
        merge_reasons=list(merge_reasons or []),
        snapshot_eligible=False,
    )


def _evaluate_region(
    region: FrontierRegion,
    cfg: Dict[str, Any],
    meta: MapMetadata,
) -> List[RegionRejection]:
    rc = cfg.get("region", {})
    fc = cfg.get("frontier", {})
    min_cells = int(rc.get("min_frontier_cells", 8))
    min_dist = float(rc.get("min_distance_m", 0.50))
    max_dist = float(rc.get("max_distance_m", 4.00))
    min_gain = int(rc.get("min_unknown_gain_cells", 10))
    min_clear = float(fc.get("min_clearance_m", 0.35))

    reasons: List[RegionRejection] = []

    if region.frontier_cell_count < min_cells:
        reasons.append(
            RegionRejection(
                "REGION_TOO_SMALL",
                region.frontier_cell_count,
                min_cells,
                f"cells={region.frontier_cell_count} < {min_cells}",
            )
        )
    if region.distance_to_robot_m < min_dist:
        reasons.append(
            RegionRejection(
                "REGION_TOO_CLOSE",
                round(region.distance_to_robot_m, 3),
                min_dist,
            )
        )
    if region.distance_to_robot_m > max_dist:
        reasons.append(
            RegionRejection(
                "REGION_TOO_FAR",
                round(region.distance_to_robot_m, 3),
                max_dist,
            )
        )
    if region.unknown_gain_cells < min_gain:
        reasons.append(
            RegionRejection(
                "REGION_LOW_UNKNOWN_GAIN",
                region.unknown_gain_cells,
                min_gain,
            )
        )
    if region.minimum_clearance_m < min_clear:
        reasons.append(
            RegionRejection(
                "REGION_LOW_CLEARANCE",
                round(region.minimum_clearance_m, 3),
                min_clear,
            )
        )
    if not (
        0 <= region.centroid_row < meta.height and 0 <= region.centroid_col < meta.width
    ):
        reasons.append(
            RegionRejection(
                "REGION_OUTSIDE_MAP",
                (region.centroid_row, region.centroid_col),
                (meta.height, meta.width),
            )
        )
    if math.isnan(region.centroid_x) or math.isnan(region.centroid_y):
        reasons.append(RegionRejection("REGION_INVALID_CENTROID", None, None))

    return reasons


def _region_sort_key(r: FrontierRegion) -> Tuple[Any, ...]:
    return (
        -r.unknown_gain_cells,
        -r.frontier_cell_count,
        r.distance_to_robot_m,
        int(r.centroid_row),
        int(r.centroid_col),
    )


def analyze_frontier_regions(
    data: np.ndarray,
    meta: MapMetadata,
    robot: RobotPose2D,
    cfg: Dict[str, Any],
    cycle_id: int = 0,
) -> FrontierAnalysisResult:
    t0 = time.perf_counter()
    data_view = np.asarray(data, dtype=np.int16)
    if not data_view.flags.writeable:
        data_check = data_view.copy()
    else:
        data_check = data_view

    health = validate_map(data_check, meta, robot, cfg)
    stats = FrontierExtractionStats()
    empty_regions: List[FrontierRegion] = []

    if not health.ok:
        stats.status = "MAP_HEALTH_REJECTED"
        stats.analysis_time_ms = (time.perf_counter() - t0) * 1000.0
        return FrontierAnalysisResult(
            cycle_id=cycle_id,
            map_health=health,
            stats=stats,
            regions=empty_regions,
            rejected_regions=empty_regions,
        )

    min_clear = float(cfg.get("frontier", {}).get("min_clearance_m", 0.35))
    raw_mask, free_examined = extract_raw_frontier_mask(data_check, cfg)
    clearance = compute_obstacle_clearance(data_check, meta, cfg)
    filtered_mask, removed = filter_frontier_by_clearance(raw_mask, clearance, min_clear)

    stats.free_cells_examined = free_examined
    stats.raw_frontier_cells = int(np.sum(raw_mask))
    stats.removed_low_clearance = removed
    stats.remaining_frontier_cells = int(np.sum(filtered_mask))

    if stats.raw_frontier_cells == 0:
        stats.status = "NO_FRONTIER"
        stats.analysis_time_ms = (time.perf_counter() - t0) * 1000.0
        return FrontierAnalysisResult(
            cycle_id=cycle_id,
            map_health=health,
            stats=stats,
            regions=[],
            rejected_regions=[],
            raw_frontier_mask=raw_mask,
            filtered_frontier_mask=filtered_mask,
        )

    clusters = connected_components_8(filtered_mask)
    stats.raw_cluster_count = len(clusters)

    merged_clusters, cluster_id_strings, merge_log, merge_reject_log, merge_stats = (
        merge_adjacent_clusters(clusters, meta, robot, cfg, cycle_id)
    )
    stats.merge_pairs_considered = merge_stats["merge_pairs_considered"]
    stats.merge_pairs_accepted = merge_stats["merge_pairs_accepted"]
    stats.merged_group_count = merge_stats["merged_group_count"]
    stats.cluster_count_after_merge = len(merged_clusters)

    rc = cfg.get("region", {})
    min_cells = int(rc.get("min_frontier_cells", 8))
    max_regions = int(rc.get("max_regions", 12))

    candidates: List[FrontierRegion] = []
    seq = 0
    too_small = 0
    for cells, cid_str in zip(merged_clusters, cluster_id_strings):
        seq += 1
        source_ids = cid_str.split("+") if "+" in cid_str else [cid_str]
        is_merged = len(source_ids) > 1
        merge_reasons: List[Dict[str, Any]] = []
        if is_merged:
            for entry in merge_log:
                if set(entry.source_clusters).issubset(set(source_ids)):
                    merge_reasons.append(
                        {
                            "other_cluster": next(
                                c for c in entry.source_clusters if c in source_ids
                            ),
                            "frontier_gap_m": entry.frontier_gap_m,
                            "centroid_distance_m": entry.centroid_distance_m,
                            "bearing_difference_deg": entry.bearing_difference_deg,
                        }
                    )
        region = compute_region_metrics(
            cells,
            data_check,
            clearance,
            meta,
            robot,
            cfg,
            cycle_id,
            seq,
            source_cluster_ids=source_ids,
            merged=is_merged,
            merge_reasons=merge_reasons,
        )
        reasons = _evaluate_region(region, cfg, meta)
        region.rejection_reasons = reasons
        if region.frontier_cell_count < min_cells:
            too_small += 1
        if reasons:
            region.accepted = False
        else:
            region.accepted = True
            region.snapshot_eligible = True
        candidates.append(region)

    stats.clusters_too_small = too_small
    candidates.sort(key=_region_sort_key)

    accepted = [r for r in candidates if r.accepted]
    rejected = [r for r in candidates if not r.accepted]
    stats.accepted_regions_before_rank_limit = len(accepted)

    # Rank limit: excess accepted become rejected
    if len(accepted) > max_regions:
        for r in accepted[max_regions:]:
            r.accepted = False
            r.snapshot_eligible = False
            r.rejection_reasons.append(
                RegionRejection(
                    "REGION_RANK_LIMIT",
                    len(accepted),
                    max_regions,
                    detail=f"rank beyond max_regions={max_regions}",
                )
            )
            rejected.append(r)
        accepted = accepted[:max_regions]

    stats.accepted_regions_after_rank_limit = len(accepted)

    # Re-assign IDs in final sorted order
    for i, r in enumerate(sorted(accepted + rejected, key=_region_sort_key), start=1):
        r.region_id = f"R{cycle_id:04d}_{i:02d}"

    stats.accepted_region_count = len(accepted)
    stats.rejected_region_count = len(rejected)
    stats.analysis_time_ms = (time.perf_counter() - t0) * 1000.0
    if stats.remaining_frontier_cells == 0 and stats.raw_frontier_cells > 0:
        stats.status = "ALL_REMOVED_BY_CLEARANCE"
    elif not accepted and rejected:
        stats.status = "ALL_REGIONS_REJECTED"
    elif not accepted:
        stats.status = "NO_ACCEPTED_REGIONS"
    else:
        stats.status = "OK"

    return FrontierAnalysisResult(
        cycle_id=cycle_id,
        map_health=health,
        stats=stats,
        regions=accepted,
        rejected_regions=rejected,
        raw_frontier_mask=raw_mask,
        filtered_frontier_mask=filtered_mask,
        merge_log=merge_log,
        merge_reject_log=merge_reject_log,
    )


def _rejection_to_dict(r: RegionRejection) -> Dict[str, Any]:
    return {
        "code": r.code,
        "actual": r.actual,
        "threshold": r.threshold,
        "detail": r.detail,
    }


def _region_to_dict(r: FrontierRegion) -> Dict[str, Any]:
    d = asdict(r)
    d["rejection_reasons"] = [_rejection_to_dict(x) for x in r.rejection_reasons]
    d["frontier_cells"] = [[int(a), int(b)] for a, b in r.frontier_cells]
    return d


def result_to_dict(result: FrontierAnalysisResult) -> Dict[str, Any]:
    return {
        "cycle_id": result.cycle_id,
        "map_health": asdict(result.map_health),
        "stats": asdict(result.stats),
        "regions": [_region_to_dict(r) for r in result.regions],
        "rejected_regions": [_region_to_dict(r) for r in result.rejected_regions],
        "merge_log": [asdict(m) for m in result.merge_log],
        "merge_reject_log": result.merge_reject_log,
    }


def occupancy_grid_to_array(grid: Any) -> Tuple[np.ndarray, MapMetadata]:
    """Convert ROS OccupancyGrid-like message to 2D numpy array and metadata."""
    w = int(grid.info.width)
    h = int(grid.info.height)
    data = np.array(grid.data, dtype=np.int16).reshape((h, w))
    stamp = 0.0
    if hasattr(grid.header, "stamp"):
        stamp = float(grid.header.stamp.sec) + float(grid.header.stamp.nanosec) * 1e-9
    meta = MapMetadata(
        width=w,
        height=h,
        resolution=float(grid.info.resolution),
        origin_x=float(grid.info.origin.position.x),
        origin_y=float(grid.info.origin.position.y),
        frame_id=str(grid.header.frame_id),
        stamp_sec=stamp,
    )
    return data, meta


def validate_config(cfg: Dict[str, Any]) -> List[str]:
    """Return list of configuration errors; empty if valid."""
    errors: List[str] = []
    safety = cfg.get("safety", {})
    for key in ("motion_enabled", "qwen_enabled", "nav2_enabled"):
        if bool(safety.get(key, False)):
            errors.append(f"safety.{key} must be false for observation-only mode")
    if not bool(safety.get("observation_only", True)):
        errors.append("safety.observation_only must be true")

    node = cfg.get("node", {})
    period = float(node.get("analysis_period_s", 1.0))
    if period <= 0:
        errors.append("node.analysis_period_s must be > 0")

    region = cfg.get("region", {})
    min_d = float(region.get("min_distance_m", 0.5))
    max_d = float(region.get("max_distance_m", 4.0))
    if min_d >= max_d:
        errors.append("region.min_distance_m must be < region.max_distance_m")

    frontier = cfg.get("frontier", {})
    if float(frontier.get("min_clearance_m", 0.35)) < 0:
        errors.append("frontier.min_clearance_m must be >= 0")

    mv = cfg.get("map_values", {})
    if "unknown_value" not in mv:
        errors.append("map_values.unknown_value is required")
    if "free_max" not in mv:
        errors.append("map_values.free_max is required")
    if "occupied_min" not in mv:
        errors.append("map_values.occupied_min is required")

    merge = cfg.get("region_merge", {})
    if merge:
        if float(merge.get("max_frontier_gap_m", 0.20)) < 0:
            errors.append("region_merge.max_frontier_gap_m must be >= 0")
        if float(merge.get("max_centroid_distance_m", 0.45)) < 0:
            errors.append("region_merge.max_centroid_distance_m must be >= 0")
        bearing = float(merge.get("max_bearing_difference_deg", 25.0))
        if bearing < 0 or bearing > 180:
            errors.append("region_merge.max_bearing_difference_deg must be in [0, 180]")

    from src.planning.robot_trajectory_store import validate_trajectory_config

    errors.extend(validate_trajectory_config(cfg))

    return errors


def grid_row_to_image_y(row: int, height: int) -> int:
    """Convert occupancy grid row to image y (origin top-left)."""
    return height - 1 - row


def render_annotated_map(
    data: np.ndarray,
    meta: MapMetadata,
    robot: Optional[RobotPose2D],
    result: FrontierAnalysisResult,
    cycle_id: int,
    cfg: Optional[Dict[str, Any]] = None,
    trajectory_overlay: Optional[Dict[str, Any]] = None,
) -> Optional[np.ndarray]:
    if not _CV2_AVAILABLE or cv2 is None:
        return None
    unknown_value, free_max, occupied_min = _map_values_cfg(cfg or {})
    h, w = data.shape
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cats, _ = classify_map_cells(data, -1, 20, 65)

    img[cats == 0] = (240, 240, 240)
    img[cats == 1] = (40, 40, 40)
    img[cats == 2] = (180, 180, 180)
    img[cats == 3] = (0, 0, 180)

    if result.filtered_frontier_mask is not None:
        fm = result.filtered_frontier_mask
        img[fm] = (0, 255, 255)

    overlay = trajectory_overlay or {}
    visited_data = overlay.get("visited_area_data")
    if visited_data is not None and len(visited_data) == h * w:
        for idx, val in enumerate(visited_data):
            if val < 100:
                continue
            r = idx // w
            c = idx % w
            if cats[r, c] != 0:
                continue
            img[r, c] = (120, 180, 200)

    vertices = overlay.get("vertices") or []
    if len(vertices) >= 2:
        pts = []
        for v in vertices:
            if isinstance(v, dict):
                vx = float(v.get("x", 0.0))
                vy = float(v.get("y", 0.0))
            else:
                vx = float(v[0])
                vy = float(v[1])
            vr, vc = world_to_grid(float(vx), float(vy), meta)
            iy = grid_row_to_image_y(vr, h)
            pts.append((int(vc), int(iy)))
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], (0, 140, 255), 2, cv2.LINE_AA)

    obs_poses = overlay.get("observation_poses") or []
    for pose in obs_poses:
        ox = float(pose.get("x", 0.0))
        oy = float(pose.get("y", 0.0))
        orow, ocol = world_to_grid(ox, oy, meta)
        oiy = grid_row_to_image_y(orow, h)
        cv2.circle(img, (ocol, oiy), 4, (255, 0, 255), -1)

    for region in result.regions + result.rejected_regions:
        color = (0, 200, 0) if region.accepted else (0, 0, 255)
        cr, cc = int(round(region.centroid_row)), int(round(region.centroid_col))
        iy = grid_row_to_image_y(cr, h)
        cv2.circle(img, (cc, iy), 3, color, -1)
        label = f"{region.region_id} {region.direction_label[:2]}"
        cv2.putText(
            img,
            label,
            (cc + 4, max(10, iy - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            color,
            1,
            cv2.LINE_AA,
        )

    if robot is not None:
        rr, rc = world_to_grid(robot.x, robot.y, meta)
        iy = grid_row_to_image_y(rr, h)
        cv2.circle(img, (rc, iy), 5, (255, 0, 0), -1)
        arrow_len = 8
        ex = int(rc + arrow_len * math.cos(robot.yaw_rad))
        ey = int(iy - arrow_len * math.sin(robot.yaw_rad))
        cv2.arrowedLine(img, (rc, iy), (ex, ey), (255, 0, 0), 2, tipLength=0.3)

    legend_y = h - 5
    legend_lines = [
        "TRAVELED PATH",
        "VISITED CORRIDOR",
        "OBSERVATION POSE",
        "ROBOT",
        "CANDIDATE REGION",
    ]
    for i, line in enumerate(legend_lines):
        cv2.putText(
            img,
            line,
            (5, max(15, legend_y - (len(legend_lines) - i) * 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )

    traj_id = overlay.get("trajectory_session_id", "")
    traj_rev = overlay.get("trajectory_revision", 0)
    traj_vcount = overlay.get("trajectory_vertex_count", len(vertices))
    traj_len = overlay.get("trajectory_length_m", 0.0)
    header = (
        f"cycle={cycle_id} {meta.width}x{meta.height} res={meta.resolution:.3f} | "
        f"{traj_id} rev={traj_rev} vtx={traj_vcount} len={traj_len:.2f}m"
    )
    cv2.putText(
        img,
        header,
        (5, 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return img


def render_global_exploration_map(
    data: np.ndarray,
    meta: MapMetadata,
    robot: Optional[RobotPose2D],
    result: FrontierAnalysisResult,
    cycle_id: int,
    cfg: Optional[Dict[str, Any]] = None,
    trajectory_overlay: Optional[Dict[str, Any]] = None,
    *,
    panel_size_px: int = 900,
    content_padding_px: int = 40,
    grid_divisions: int = 10,
) -> Tuple[Optional[np.ndarray], Dict[str, Any]]:
    """Render map panel for GLOBAL_REGION_PROPOSAL (no A/B/C candidate labels)."""
    if not _CV2_AVAILABLE or cv2 is None:
        metadata = build_map_render_metadata(
            meta,
            snapshot_id="",
            panel_size_px=panel_size_px,
            content_padding_px=content_padding_px,
        )
        return None, metadata

    base = render_annotated_map(
        data,
        meta,
        robot,
        result,
        cycle_id,
        cfg,
        trajectory_overlay,
    )
    if base is None:
        metadata = build_map_render_metadata(
            meta,
            snapshot_id="",
            panel_size_px=panel_size_px,
            content_padding_px=content_padding_px,
        )
        return None, metadata

    h, w = base.shape[:2]
    # Strip candidate labels by re-rendering without region label loop
    img = np.zeros((h, w, 3), dtype=np.uint8)
    unknown_value, free_max, occupied_min = _map_values_cfg(cfg or {})
    cats, _ = classify_map_cells(data, unknown_value, free_max, occupied_min)
    img[cats == 0] = (240, 240, 240)
    img[cats == 1] = (40, 40, 40)
    img[cats == 2] = (180, 180, 180)
    img[cats == 3] = (0, 0, 180)
    if result.filtered_frontier_mask is not None:
        img[result.filtered_frontier_mask] = (0, 255, 255)

    overlay = trajectory_overlay or {}
    visited_data = overlay.get("visited_area_data")
    if visited_data is not None and len(visited_data) == h * w:
        for idx, val in enumerate(visited_data):
            if val < 100:
                continue
            r = idx // w
            c = idx % w
            if cats[r, c] != 0:
                continue
            img[r, c] = (120, 180, 200)

    vertices = overlay.get("vertices") or []
    if len(vertices) >= 2:
        pts = []
        for v in vertices:
            if isinstance(v, dict):
                vx = float(v.get("x", 0.0))
                vy = float(v.get("y", 0.0))
            else:
                vx = float(v[0])
                vy = float(v[1])
            vr, vc = world_to_grid(float(vx), float(vy), meta)
            iy = grid_row_to_image_y(vr, h)
            pts.append((int(vc), int(iy)))
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], (0, 140, 255), 2, cv2.LINE_AA)

    for pose in overlay.get("observation_poses") or []:
        ox = float(pose.get("x", 0.0))
        oy = float(pose.get("y", 0.0))
        orow, ocol = world_to_grid(ox, oy, meta)
        oiy = grid_row_to_image_y(orow, h)
        cv2.circle(img, (ocol, oiy), 4, (255, 0, 255), -1)

    if robot is not None:
        rr, rc = world_to_grid(robot.x, robot.y, meta)
        iy = grid_row_to_image_y(rr, h)
        cv2.circle(img, (rc, iy), 5, (255, 0, 0), -1)
        arrow_len = 8
        ex = int(rc + arrow_len * math.cos(robot.yaw_rad))
        ey = int(iy - arrow_len * math.sin(robot.yaw_rad))
        cv2.arrowedLine(img, (rc, iy), (ex, ey), (255, 0, 0), 2, tipLength=0.3)

    pad = int(content_padding_px)
    panel = np.full((panel_size_px, panel_size_px, 3), 255, dtype=np.uint8)
    content_max = panel_size_px - pad
    scale = min((content_max - pad) / max(w, 1), (content_max - pad) / max(h, 1))
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    resized = cv2.resize(img, (new_w, new_h))
    x_off = (panel_size_px - new_w) // 2
    y_off = (panel_size_px - new_h) // 2
    panel[y_off : y_off + new_h, x_off : x_off + new_w] = resized

    x_min = x_off
    y_min = y_off
    x_max = x_off + new_w
    y_max = y_off + new_h
    content_w = max(1, x_max - x_min)
    content_h = max(1, y_max - y_min)

    for i in range(1, grid_divisions):
        frac = i / grid_divisions
        gx = int(x_min + frac * content_w)
        gy = int(y_min + frac * content_h)
        cv2.line(panel, (gx, y_min), (gx, y_max), (200, 200, 200), 1, cv2.LINE_AA)
        cv2.line(panel, (x_min, gy), (x_max, gy), (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(
            panel,
            f"{frac:.1f}",
            (gx - 12, y_min - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.28,
            (120, 120, 120),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            panel,
            f"{frac:.1f}",
            (x_min - 28, gy + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.28,
            (120, 120, 120),
            1,
            cv2.LINE_AA,
        )

    legend_lines = [
        "FRONTIER BOUNDARY",
        "TRAVELED PATH",
        "VISITED CORRIDOR",
        "OBSERVATION POSE",
        "ROBOT",
        "FREE / OCCUPIED / UNKNOWN",
    ]
    for i, line in enumerate(legend_lines):
        cv2.putText(
            panel,
            line,
            (5, 15 + i * 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )

    metadata = build_map_render_metadata(
        meta,
        snapshot_id="",
        panel_size_px=panel_size_px,
        content_padding_px=content_padding_px,
        content_x_min_px=x_min,
        content_y_min_px=y_min,
        content_x_max_px=x_max,
        content_y_max_px=y_max,
    )
    return panel, metadata


def build_map_render_metadata(
    meta: MapMetadata,
    *,
    snapshot_id: str,
    panel_size_px: int = 900,
    content_padding_px: int = 40,
    content_x_min_px: Optional[int] = None,
    content_y_min_px: Optional[int] = None,
    content_x_max_px: Optional[int] = None,
    content_y_max_px: Optional[int] = None,
) -> Dict[str, Any]:
    pad = int(content_padding_px)
    size = int(panel_size_px)
    x_min = pad if content_x_min_px is None else int(content_x_min_px)
    y_min = pad if content_y_min_px is None else int(content_y_min_px)
    x_max = size - pad if content_x_max_px is None else int(content_x_max_px)
    y_max = size - pad if content_y_max_px is None else int(content_y_max_px)
    return {
        "snapshot_id": snapshot_id,
        "map_panel": {
            "image_width_px": size,
            "image_height_px": size,
            "content_x_min_px": x_min,
            "content_y_min_px": y_min,
            "content_x_max_px": x_max,
            "content_y_max_px": y_max,
        },
        "coordinate_space": "NORMALIZED_MAP_VIEWPORT",
        "u_direction": "LEFT_TO_RIGHT",
        "v_direction": "TOP_TO_BOTTOM",
        "map_grid": {
            "width": meta.width,
            "height": meta.height,
            "resolution": meta.resolution,
            "origin_x": meta.origin_x,
            "origin_y": meta.origin_y,
            "origin_yaw": 0.0,
        },
        "render_transform": {
            "grid_x_flipped": False,
            "grid_y_flipped": True,
            "rotation_deg": 0.0,
        },
    }


def deep_copy_grid_data(data: np.ndarray) -> np.ndarray:
    return copy.deepcopy(data)


def _snapshot_label_sort_key(region: FrontierRegion) -> Tuple[float, float, float, float]:
    return (
        region.bearing_relative_deg,
        region.distance_to_robot_m,
        region.centroid_x,
        region.centroid_y,
    )


def generate_snapshot_id(cycle_id: int, when: Optional[datetime] = None) -> str:
    ts = when or datetime.now(timezone.utc)
    return f"RS_{ts.strftime('%Y%m%dT%H%M%S')}_{cycle_id:04d}"


def labeled_top_regions_for_snapshot(
    result: FrontierAnalysisResult,
    *,
    top_k: int = 5,
    minimum_eligible_score: float = 0.40,
) -> List[Tuple[str, FrontierRegion]]:
    """Return (label, region) pairs using the same labeling as snapshot payload."""
    eligible = [
        r for r in result.regions
        if r.snapshot_eligible and r.stable and r.geo_score >= 0.0
    ]
    eligible.sort(key=lambda r: (r.geo_rank if r.geo_rank > 0 else 999, -r.geo_score))
    eligible = [r for r in eligible if r.geo_score >= minimum_eligible_score]
    if not eligible:
        eligible = [r for r in result.regions if r.snapshot_eligible]
    top = eligible[:top_k]
    top.sort(key=lambda r: (r.geo_rank if r.geo_rank > 0 else 999, r.bearing_relative_deg, r.distance_to_robot_m))
    labels = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    out: List[Tuple[str, FrontierRegion]] = []
    for idx, region in enumerate(top):
        label = labels[idx] if idx < len(labels) else f"R{idx}"
        out.append((label, region))
    return out


def build_region_geometry_payload(
    result: FrontierAnalysisResult,
    meta: MapMetadata,
    snapshot_id: str,
    *,
    top_k: int = 5,
    minimum_eligible_score: float = 0.40,
    map_fingerprints: Optional[Dict[str, str]] = None,
    contract_cfg: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Internal frontier geometry for safe viewpoint generation (not for Qwen prompts)."""
    from src.planning.exploration_contracts import (  # noqa: WPS433
        EXPLORATION_CONTRACT_VERSION,
        REGION_GEOMETRY_SCHEMA_VERSION,
        build_region_geometry_fingerprint_from_entry,
    )

    regions_out: Dict[str, Any] = {}
    for label, region in labeled_top_regions_for_snapshot(
        result, top_k=top_k, minimum_eligible_score=minimum_eligible_score
    ):
        frontier_points = [
            [round(x, 4), round(y, 4)] for x, y in (grid_to_world(r, c, meta) for r, c in region.frontier_cells)
        ]
        entry: Dict[str, Any] = {
            "internal_region_id": region.region_id,
            "track_id": region.track_id,
            "centroid_x": region.centroid_x,
            "centroid_y": region.centroid_y,
            "frontier_cells_grid": [[int(r), int(c)] for r, c in region.frontier_cells],
            "frontier_points_map": frontier_points,
            "bbox_grid": [region.row_min, region.row_max, region.col_min, region.col_max],
            "bearing_global_deg": region.bearing_global_deg,
            "unknown_gain_cells": region.unknown_gain_cells,
            "minimum_clearance_m": region.minimum_clearance_m,
            "trajectory_novelty_score": region.trajectory_novelty_score,
            "trajectory_revisit_penalty": region.trajectory_revisit_penalty,
            "distance_to_nearest_observation_pose_m": region.distance_to_nearest_observation_pose_m,
            "geo_score": region.geo_score,
            "stable": region.stable,
            "snapshot_eligible": region.snapshot_eligible,
            "blacklisted": region.blacklisted,
        }
        entry["region_geometry_fingerprint"] = build_region_geometry_fingerprint_from_entry(
            snapshot_id, label, entry, cfg=contract_cfg
        )
        regions_out[label] = entry
    payload: Dict[str, Any] = {
        "contract_version": EXPLORATION_CONTRACT_VERSION,
        "region_geometry_schema_version": REGION_GEOMETRY_SCHEMA_VERSION,
        "schema_version": REGION_GEOMETRY_SCHEMA_VERSION,
        "snapshot_id": snapshot_id,
        "regions": regions_out,
    }
    if map_fingerprints:
        payload["map_fingerprint"] = map_fingerprints.get("map_fingerprint", "")
        payload["map_metadata_fingerprint"] = map_fingerprints.get("map_metadata_fingerprint", "")
        payload["map_data_fingerprint"] = map_fingerprints.get("map_data_fingerprint", "")
    return payload


def build_region_snapshot_payload(
    result: FrontierAnalysisResult,
    meta: MapMetadata,
    robot: RobotPose2D,
    snapshot_id: str,
    capture_time: str,
    expires_after_s: float = 300.0,
    annotated_map_file: str = "",
    *,
    observation_meta: Optional[Dict[str, Any]] = None,
    top_k: int = 5,
    trajectory_meta: Optional[Dict[str, Any]] = None,
    map_data: Optional[Sequence[int]] = None,
    contract_cfg: Optional[Dict[str, Any]] = None,
    map_render_metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    from src.planning.exploration_contracts import (  # noqa: WPS433
        EXPLORATION_CONTRACT_VERSION,
        REGION_SNAPSHOT_SCHEMA_VERSION,
        TRAJECTORY_SCHEMA_VERSION,
        build_map_fingerprint,
        build_map_render_metadata_fingerprint,
    )
    eligible = [
        r for r in result.regions
        if r.snapshot_eligible and r.stable and r.geo_score >= 0.0
    ]
    eligible.sort(key=lambda r: (r.geo_rank if r.geo_rank > 0 else 999, -r.geo_score))
    gcfg_min = float((observation_meta or {}).get("minimum_eligible_score", 0.40))
    eligible = [r for r in eligible if r.geo_score >= gcfg_min]
    if not eligible:
        eligible = [r for r in result.regions if r.snapshot_eligible]
    top = eligible[:top_k]
    top.sort(key=lambda r: (r.geo_rank if r.geo_rank > 0 else 999, r.bearing_relative_deg, r.distance_to_robot_m))

    labels = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    region_entries: List[Dict[str, Any]] = []
    rejection_summary: Dict[str, int] = {}

    for region in result.regions + result.rejected_regions:
        for reason in region.rejection_reasons:
            rejection_summary[reason.code] = rejection_summary.get(reason.code, 0) + 1
        for code in region.stability_rejection_reasons:
            rejection_summary[code] = rejection_summary.get(code, 0) + 1

    for idx, region in enumerate(top):
        label = labels[idx] if idx < len(labels) else f"R{idx}"
        region_entries.append(
            {
                "label": label,
                "internal_region_id": region.region_id,
                "track_id": region.track_id,
                "source_cluster_ids": list(region.source_cluster_ids),
                "merged": region.merged,
                "merge_reasons": list(region.merge_reasons),
                "direction": region.direction_label,
                "centroid": {"x": region.centroid_x, "y": region.centroid_y},
                "distance_m": region.distance_to_robot_m,
                "unknown_gain_cells": region.unknown_gain_cells,
                "unknown_gain_ratio": region.unknown_gain_ratio,
                "minimum_clearance_m": region.minimum_clearance_m,
                "mean_clearance_m": region.mean_clearance_m,
                "frontier_cell_count": region.frontier_cell_count,
                "path_checked": region.path_checked,
                "reachable": region.reachable,
                "stable": region.stable,
                "snapshot_eligible": region.snapshot_eligible,
                "persistence_cycles": region.persistence_cycles,
                "centroid_drift_m": region.centroid_drift_m,
                "bearing_drift_deg": region.bearing_drift_deg,
                "distance_to_nearest_observation_pose_m": region.distance_to_nearest_observation_pose_m,
                "nearest_trajectory_distance_m": region.nearest_trajectory_distance_m,
                "nearby_trajectory_vertex_count": region.nearby_trajectory_vertex_count,
                "nearby_recent_trajectory_count": region.nearby_recent_trajectory_count,
                "trajectory_density_score": region.trajectory_density_score,
                "trajectory_novelty_score": region.trajectory_novelty_score,
                "trajectory_revisit_penalty": region.trajectory_revisit_penalty,
                "last_nearby_visit_age_s": region.last_nearby_visit_age_s,
                "geo_score_before_trajectory": region.geo_score_before_trajectory,
                "geo_score_after_trajectory": region.geo_score_after_trajectory,
                "near_robot_penalty": region.near_robot_penalty,
                "recent_observation_penalty": region.recent_observation_penalty,
                "visit_count": region.visited_count,
                "selection_count": 0,
                "navigation_failure_count": region.navigation_failure_count,
                "blacklisted": region.blacklisted,
                "geo_score": region.geo_score,
                "geo_rank": region.geo_rank,
                "score_components": dict(region.score_components),
                "penalty_components": dict(region.penalty_components),
            }
        )

    obs = observation_meta or {}
    traj = trajectory_meta or {}
    payload: Dict[str, Any] = {
        "contract_version": EXPLORATION_CONTRACT_VERSION,
        "snapshot_schema_version": REGION_SNAPSHOT_SCHEMA_VERSION,
        "schema_version": REGION_SNAPSHOT_SCHEMA_VERSION,
        "snapshot_id": snapshot_id,
        "cycle_id": result.cycle_id,
        "map_stamp": meta.stamp_sec,
        "capture_time": capture_time,
        "robot_pose": {
            "x": robot.x,
            "y": robot.y,
            "yaw_deg": math.degrees(robot.yaw_rad),
        },
        "map_metadata": asdict(meta),
        "analysis_status": result.stats.status,
        "accepted_regions": region_entries,
        "rejected_reason_summary": rejection_summary,
        "annotated_map_file": annotated_map_file,
        "expires_after_s": expires_after_s,
        "trajectory_session_id": traj.get("trajectory_session_id", ""),
        "trajectory_revision": traj.get("trajectory_revision", 0),
        "trajectory_length_m": traj.get("trajectory_length_m", 0.0),
        "trajectory_raw_sample_count": traj.get("trajectory_raw_sample_count", 0),
        "trajectory_vertex_count": traj.get("trajectory_vertex_count", 0),
        "visited_corridor_radius_m": traj.get("visited_corridor_radius_m", 0.35),
        "trajectory_path_topic": traj.get(
            "trajectory_path_topic", "/qwen_explore_debug/trajectory_path"
        ),
        "visited_area_grid_topic": traj.get(
            "visited_area_grid_topic", "/qwen_explore_debug/visited_area_grid"
        ),
        "observation_window_id": obs.get("observation_window_id"),
        "full_scan_completed": obs.get("full_scan_completed", False),
        "accumulated_rotation_deg": obs.get("accumulated_rotation_deg", 0.0),
        "map_stable": obs.get("map_stable", False),
        "robot_settled": obs.get("robot_settled", False),
        "geometric_scoring_version": "1.1",
        "history_version": obs.get("history_version", "1.0"),
        "merge_summary": {
            "raw_cluster_count": result.stats.raw_cluster_count,
            "merge_pairs_accepted": result.stats.merge_pairs_accepted,
            "merged_group_count": result.stats.merged_group_count,
            "cluster_count_after_merge": result.stats.cluster_count_after_merge,
        },
        "guard_summary": obs.get("guard_summary", {}),
    }
    if map_render_metadata:
        mrm = dict(map_render_metadata)
        mrm["snapshot_id"] = snapshot_id
        payload["map_render_metadata"] = mrm
        payload["map_render_metadata_fingerprint"] = build_map_render_metadata_fingerprint(
            mrm, cfg=contract_cfg
        )
    payload["trajectory_meta"] = dict(traj)
    payload["trajectory_schema_version"] = str(
        traj.get("trajectory_schema_version", TRAJECTORY_SCHEMA_VERSION)
    )
    if map_data is not None:
        fps = build_map_fingerprint(
            frame_id=meta.frame_id,
            width=meta.width,
            height=meta.height,
            resolution=meta.resolution,
            origin_x=meta.origin_x,
            origin_y=meta.origin_y,
            origin_yaw=float(getattr(meta, "origin_yaw", 0.0)),
            map_data=map_data,
            map_stamp=meta.stamp_sec,
            cfg=contract_cfg,
        )
        payload.update(fps)
    return payload
