#!/usr/bin/env python3
"""Pure safe viewpoint generation from map geometry — no ROS, no Nav2, no motion.

Region centroid is only a geometric summary of the frontier cluster, NOT a navigation
target. Viewpoints must be regenerated from known FREE cells on the free side of
frontier cells facing unknown space.
"""

from __future__ import annotations

import copy
import math
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

CELL_FREE = "FREE"
CELL_OCCUPIED = "OCCUPIED"
CELL_UNKNOWN = "UNKNOWN"
CELL_UNCERTAIN = "UNCERTAIN"

NEIGH8: Tuple[Tuple[int, int], ...] = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)

ALGORITHM_VERSION = "phase3a_v1"


@dataclass
class MapGridSnapshot:
    frame_id: str
    stamp_sec: float
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: List[int]
    free_threshold: int = 20
    occupied_threshold: int = 65

    def validate(self) -> List[str]:
        errors: List[str] = []
        if self.width <= 0:
            errors.append("MAP_WIDTH_INVALID")
        if self.height <= 0:
            errors.append("MAP_HEIGHT_INVALID")
        if self.resolution <= 0:
            errors.append("MAP_RESOLUTION_INVALID")
        if len(self.data) != self.width * self.height:
            errors.append("MAP_DATA_LENGTH_MISMATCH")
        for name, val in (
            ("origin_x", self.origin_x),
            ("origin_y", self.origin_y),
            ("origin_yaw", self.origin_yaw),
            ("resolution", self.resolution),
            ("stamp_sec", self.stamp_sec),
        ):
            if not math.isfinite(val):
                errors.append(f"MAP_{name.upper()}_NON_FINITE")
        return errors


@dataclass
class SelectedRegionGeometry:
    snapshot_id: str
    region_label: str
    internal_region_id: str
    track_id: str
    centroid_x: float
    centroid_y: float
    frontier_cells_grid: List[List[int]]
    frontier_points_map: List[List[float]]
    bbox_grid: List[int]
    bearing_global_deg: float
    unknown_gain_cells: int
    minimum_clearance_m: float
    trajectory_novelty_score: float = 1.0
    trajectory_revisit_penalty: float = 0.0
    distance_to_nearest_observation_pose_m: float = float("inf")
    geo_score: float = 0.0
    stable: bool = False
    snapshot_eligible: bool = False
    blacklisted: bool = False

    def validate(self) -> List[str]:
        errors: List[str] = []
        if not self.stable:
            errors.append("VIEWPOINT_REGION_UNSTABLE")
        if not self.snapshot_eligible:
            errors.append("VIEWPOINT_REGION_NOT_ELIGIBLE")
        if self.blacklisted:
            errors.append("VIEWPOINT_REGION_BLACKLISTED")
        if not self.frontier_cells_grid:
            errors.append("VIEWPOINT_REGION_GEOMETRY_EMPTY")
        return errors


@dataclass
class ViewpointCandidate:
    candidate_id: str
    region_label: str
    track_id: str
    grid_x: int
    grid_y: int
    x: float
    y: float
    yaw_rad: float
    yaw_deg: float
    source_frontier_grid_x: int
    source_frontier_grid_y: int
    source_frontier_x: float
    source_frontier_y: float
    stand_off_distance_m: float
    distance_to_robot_m: float
    distance_to_frontier_m: float
    clearance_m: float
    footprint_free_ratio: float
    local_free_ratio: float
    line_of_sight_clear: bool
    line_of_sight_length_m: float
    estimated_visible_unknown_cells: int
    trajectory_novelty_score: float
    recent_observation_penalty: float
    hard_gate_passed: bool
    rejection_reasons: List[str] = field(default_factory=list)
    score: Optional[float] = None
    score_components: Dict[str, float] = field(default_factory=dict)
    path_checked: bool = False
    reachable: Optional[bool] = None
    turn_from_robot_yaw_deg: float = 0.0
    line_of_sight_block_cell: Optional[List[int]] = None
    line_of_sight_block_value: Optional[int] = None
    footprint_unknown_count: int = 0
    footprint_occupied_count: int = 0


@dataclass
class ViewpointGenerationResult:
    snapshot_id: str
    region_label: str
    track_id: str
    map_stamp_sec: float
    generation_id: str
    raw_candidate_count: int
    accepted_candidate_count: int
    rejected_candidate_count: int
    accepted_candidates: List[ViewpointCandidate]
    rejected_candidates: List[ViewpointCandidate]
    rejection_reason_summary: Dict[str, int]
    selected_candidate_id: Optional[str]
    algorithm_version: str = ALGORITHM_VERSION


def _clamp(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


def _cfg_map(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("map", {})


def _cfg_robot(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("robot", {})


def _cfg_gen(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("viewpoint_generation", {})


def _cfg_val(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("viewpoint_validation", {})


def _cfg_score(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("viewpoint_scoring", {})


def _cfg_sel(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("viewpoint_selection", {})


def validate_safe_viewpoint_config(cfg: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    safety = cfg.get("safety", {})
    for key in ("allow_motion", "allow_cmd_vel", "allow_nav2", "allow_goal_publish"):
        if bool(safety.get(key, False)):
            errors.append(f"safety.{key} must be false")
    weights = _cfg_score(cfg).get("weights", {})
    if weights:
        total = sum(float(v) for v in weights.values())
        if abs(total - 1.0) > 0.01:
            errors.append("viewpoint_scoring.weights must sum to 1.0")
    robot = _cfg_robot(cfg)
    fp = float(robot.get("footprint_radius_m", 0.23))
    sm = float(robot.get("safety_margin_m", 0.10))
    rc = float(robot.get("required_clearance_m", 0.33))
    if rc < fp + sm - 1e-6:
        errors.append("robot.required_clearance_m should be >= footprint_radius + safety_margin")
    return errors


def classify_cell(value: int, cfg: Dict[str, Any]) -> str:
    mcfg = _cfg_map(cfg)
    unknown_value = int(mcfg.get("unknown_value", -1))
    free_max = int(mcfg.get("free_max_value", 20))
    occupied_min = int(mcfg.get("occupied_min_value", 65))
    if value == unknown_value or value < 0:
        return CELL_UNKNOWN
    if 0 <= value <= free_max:
        return CELL_FREE
    if value >= occupied_min:
        return CELL_OCCUPIED
    return CELL_UNCERTAIN


def grid_index(row: int, col: int, width: int) -> int:
    return row * width + col


def is_grid_inside(row: int, col: int, snap: MapGridSnapshot) -> bool:
    return 0 <= row < snap.height and 0 <= col < snap.width


def get_cell_value(snap: MapGridSnapshot, row: int, col: int) -> Optional[int]:
    if not is_grid_inside(row, col, snap):
        return None
    return int(snap.data[grid_index(row, col, snap.width)])


def grid_to_map(row: int, col: int, snap: MapGridSnapshot) -> Tuple[float, float]:
    """Map grid cell center to world coordinates; respects origin yaw."""
    local_x = (col + 0.5) * snap.resolution
    local_y = (row + 0.5) * snap.resolution
    cos_o = math.cos(snap.origin_yaw)
    sin_o = math.sin(snap.origin_yaw)
    x = snap.origin_x + cos_o * local_x - sin_o * local_y
    y = snap.origin_y + sin_o * local_x + cos_o * local_y
    return x, y


def map_to_grid(x: float, y: float, snap: MapGridSnapshot) -> Tuple[int, int]:
    """World coordinates to grid row/col; inverse of grid_to_map."""
    dx = x - snap.origin_x
    dy = y - snap.origin_y
    cos_o = math.cos(-snap.origin_yaw)
    sin_o = math.sin(-snap.origin_yaw)
    local_x = cos_o * dx - sin_o * dy
    local_y = sin_o * dx + cos_o * dy
    col = int(math.floor(local_x / snap.resolution - 0.5 + 1e-9))
    row = int(math.floor(local_y / snap.resolution - 0.5 + 1e-9))
    return row, col


def _normalize2(vx: float, vy: float) -> Tuple[float, float]:
    n = math.hypot(vx, vy)
    if n <= 1e-9:
        return 0.0, 0.0
    return vx / n, vy / n


def _perpendicular(vx: float, vy: float) -> Tuple[float, float]:
    return -vy, vx


def compute_frontier_free_direction(
    row: int,
    col: int,
    snap: MapGridSnapshot,
    cfg: Dict[str, Any],
) -> Tuple[Optional[Tuple[float, float]], Optional[str]]:
    """Return unit free_direction (map frame) pointing into known free space."""
    unknown_pts: List[Tuple[float, float]] = []
    free_pts: List[Tuple[float, float]] = []
    fx, fy = grid_to_map(row, col, snap)
    for dr, dc in NEIGH8:
        nr, nc = row + dr, col + dc
        val = get_cell_value(snap, nr, nc)
        if val is None:
            continue
        cat = classify_cell(val, cfg)
        nx, ny = grid_to_map(nr, nc, snap)
        if cat == CELL_UNKNOWN:
            unknown_pts.append((nx, ny))
        elif cat == CELL_FREE:
            free_pts.append((nx, ny))
    if not unknown_pts:
        return None, "FRONTIER_UNKNOWN_DIRECTION_UNDEFINED"
    ux = sum(p[0] for p in unknown_pts) / len(unknown_pts)
    uy = sum(p[1] for p in unknown_pts) / len(unknown_pts)
    udx, udy = _normalize2(ux - fx, uy - fy)
    if abs(udx) < 1e-9 and abs(udy) < 1e-9:
        return None, "FRONTIER_UNKNOWN_DIRECTION_UNDEFINED"
    free_dx, free_dy = -udx, -udy
    return (free_dx, free_dy), None


def _downsample_frontier(
    cells: Sequence[Sequence[int]],
    points_map: Sequence[Sequence[float]],
    spacing_m: float,
) -> List[Tuple[int, int, float, float]]:
    if not cells:
        return []
    spacing = max(spacing_m, 1e-6)
    kept: List[Tuple[int, int, float, float]] = []
    last_x = last_y = None
    for (row, col), pt in zip(cells, points_map):
        fx = float(pt[0]) if len(pt) >= 2 else 0.0
        fy = float(pt[1]) if len(pt) >= 2 else 0.0
        if last_x is None or math.hypot(fx - last_x, fy - last_y) >= spacing:
            kept.append((int(row), int(col), fx, fy))
            last_x, last_y = fx, fy
    return kept


def _downsample_frontier_cells(
    region: SelectedRegionGeometry,
    spacing_m: float,
    snap: MapGridSnapshot,
) -> List[Tuple[int, int, float, float]]:
    cells = region.frontier_cells_grid
    points = region.frontier_points_map
    if len(points) != len(cells):
        points = [[grid_to_map(c[0], c[1], snap)[0], grid_to_map(c[0], c[1], snap)[1]] for c in cells]
    return _downsample_frontier(cells, points, spacing_m)


def generate_raw_candidates(
    region: SelectedRegionGeometry,
    snap: MapGridSnapshot,
    cfg: Dict[str, Any],
) -> Tuple[List[Dict[str, Any]], List[str]]:
    gcfg = _cfg_gen(cfg)
    stand_offs = [float(x) for x in gcfg.get("stand_off_distances_m", [0.45, 0.60, 0.75, 0.90])]
    laterals = [float(x) for x in gcfg.get("lateral_offsets_m", [-0.30, -0.15, 0.0, 0.15, 0.30])]
    spacing = float(gcfg.get("frontier_sample_spacing_m", 0.20))
    max_raw = int(gcfg.get("max_raw_candidates", 500))

    samples = _downsample_frontier_cells(region, spacing, snap)
    raw: List[Dict[str, Any]] = []
    errors: List[str] = []

    for row, col, fx, fy in samples:
        free_dir, err = compute_frontier_free_direction(row, col, snap, cfg)
        if free_dir is None:
            if err:
                errors.append(err)
            continue
        fdx, fdy = free_dir
        ldx, ldy = _perpendicular(fdx, fdy)
        for stand in stand_offs:
            for lat in laterals:
                tx = fx + fdx * stand + ldx * lat
                ty = fy + fdy * stand + ldy * lat
                raw.append(
                    {
                        "theory_x": tx,
                        "theory_y": ty,
                        "source_row": row,
                        "source_col": col,
                        "source_fx": fx,
                        "source_fy": fy,
                        "stand_off": stand,
                        "lateral": lat,
                    }
                )
                if len(raw) >= max_raw:
                    return raw, errors
    return raw, errors


def _local_search_candidates(
    theory_x: float,
    theory_y: float,
    snap: MapGridSnapshot,
    cfg: Dict[str, Any],
) -> List[Tuple[int, int]]:
    gcfg = _cfg_gen(cfg)
    radius_m = float(gcfg.get("local_search_radius_m", 0.20))
    step_cells = int(gcfg.get("local_search_step_cells", 1))
    center_row, center_col = map_to_grid(theory_x, theory_y, snap)
    max_delta = int(math.ceil(radius_m / snap.resolution))
    cells: List[Tuple[int, int, float, float]] = []
    for dr in range(-max_delta, max_delta + 1, max(step_cells, 1)):
        for dc in range(-max_delta, max_delta + 1, max(step_cells, 1)):
            nr, nc = center_row + dr, center_col + dc
            if not is_grid_inside(nr, nc, snap):
                continue
            val = get_cell_value(snap, nr, nc)
            if val is None or classify_cell(val, cfg) != CELL_FREE:
                continue
            cx, cy = grid_to_map(nr, nc, snap)
            dist = math.hypot(cx - theory_x, cy - theory_y)
            if dist > radius_m + snap.resolution * 0.5:
                continue
            clearance = _estimate_clearance(nr, nc, snap, cfg)
            cells.append((nr, nc, dist, clearance))
    cells.sort(key=lambda t: (t[2], -t[3], t[0], t[1]))
    return [(r, c) for r, c, _, _ in cells]


def _estimate_clearance(row: int, col: int, snap: MapGridSnapshot, cfg: Dict[str, Any]) -> float:
    """Min distance to non-free cell within search radius."""
    max_r = int(math.ceil(float(_cfg_robot(cfg).get("required_clearance_m", 0.33)) / snap.resolution)) + 2
    best = float("inf")
    for dr in range(-max_r, max_r + 1):
        for dc in range(-max_r, max_r + 1):
            nr, nc = row + dr, col + dc
            val = get_cell_value(snap, nr, nc)
            if val is None:
                continue
            cat = classify_cell(val, cfg)
            if cat != CELL_FREE:
                d = math.hypot(dr, dc) * snap.resolution
                best = min(best, d)
    return best if math.isfinite(best) else 0.0


def _footprint_check(
    row: int,
    col: int,
    snap: MapGridSnapshot,
    cfg: Dict[str, Any],
) -> Tuple[float, int, int]:
    robot = _cfg_robot(cfg)
    radius = float(robot.get("required_clearance_m", 0.33))
    radius_cells = int(math.ceil(radius / snap.resolution))
    cx, cy = grid_to_map(row, col, snap)
    total = 0
    free = 0
    unknown_c = 0
    occupied_c = 0
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            nr, nc = row + dr, col + dc
            gx, gy = grid_to_map(nr, nc, snap)
            if math.hypot(gx - cx, gy - cy) > radius + snap.resolution * 0.5:
                continue
            total += 1
            val = get_cell_value(snap, nr, nc)
            if val is None:
                unknown_c += 1
                continue
            cat = classify_cell(val, cfg)
            if cat == CELL_FREE:
                free += 1
            elif cat == CELL_UNKNOWN:
                unknown_c += 1
            elif cat == CELL_OCCUPIED:
                occupied_c += 1
            else:
                unknown_c += 1
    ratio = free / total if total > 0 else 0.0
    return ratio, unknown_c, occupied_c


def _local_free_ratio(row: int, col: int, snap: MapGridSnapshot, cfg: Dict[str, Any]) -> float:
    vcfg = _cfg_val(cfg)
    radius_m = float(vcfg.get("local_free_radius_m", 0.45))
    radius_cells = int(math.ceil(radius_m / snap.resolution))
    cx, cy = grid_to_map(row, col, snap)
    total = free = 0
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            nr, nc = row + dr, col + dc
            gx, gy = grid_to_map(nr, nc, snap)
            if math.hypot(gx - cx, gy - cy) > radius_m + snap.resolution * 0.5:
                continue
            total += 1
            val = get_cell_value(snap, nr, nc)
            if val is not None and classify_cell(val, cfg) == CELL_FREE:
                free += 1
    return free / total if total > 0 else 0.0


def point_to_frontier_distance(
    x: float,
    y: float,
    frontier_points: Sequence[Sequence[float]],
) -> float:
    if not frontier_points:
        return float("inf")
    best = float("inf")
    for pt in frontier_points:
        if len(pt) < 2:
            continue
        best = min(best, math.hypot(x - float(pt[0]), y - float(pt[1])))
    return best


def raytrace_grid(
    row0: int,
    col0: int,
    row1: int,
    col1: int,
) -> List[Tuple[int, int]]:
    """Bresenham grid ray from (row0,col0) to (row1,col1) inclusive."""
    cells: List[Tuple[int, int]] = []
    dr = abs(row1 - row0)
    dc = abs(col1 - col0)
    sr = 1 if row0 < row1 else -1
    sc = 1 if col0 < col1 else -1
    err = dr - dc
    r, c = row0, col0
    while True:
        cells.append((r, c))
        if r == row1 and c == col1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


def check_line_of_sight(
    cand_row: int,
    cand_col: int,
    frontier_row: int,
    frontier_col: int,
    snap: MapGridSnapshot,
    cfg: Dict[str, Any],
) -> Tuple[bool, float, Optional[List[int]], Optional[int]]:
    path = raytrace_grid(cand_row, cand_col, frontier_row, frontier_col)
    length_m = 0.0
    prev_r, prev_c = path[0]
    for r, c in path[1:]:
        x0, y0 = grid_to_map(prev_r, prev_c, snap)
        x1, y1 = grid_to_map(r, c, snap)
        length_m += math.hypot(x1 - x0, y1 - y0)
        if (r, c) == (frontier_row, frontier_col):
            break
        val = get_cell_value(snap, r, c)
        if val is None:
            continue
        cat = classify_cell(val, cfg)
        if cat in (CELL_OCCUPIED, CELL_UNCERTAIN):
            return False, length_m, [r, c], val
        prev_r, prev_c = r, c
    return True, length_m, None, None


def estimate_visible_unknown_cells(
    x: float,
    y: float,
    yaw_rad: float,
    snap: MapGridSnapshot,
    cfg: Dict[str, Any],
) -> int:
    scfg = _cfg_score(cfg)
    fov_deg = float(scfg.get("sensor_fov_deg", 80.0))
    radius_m = float(scfg.get("visible_unknown_radius_m", 2.0))
    half_fov = math.radians(fov_deg / 2.0)
    radius_cells = int(math.ceil(radius_m / snap.resolution))
    row, col = map_to_grid(x, y, snap)
    count = 0
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            nr, nc = row + dr, col + dc
            nx, ny = grid_to_map(nr, nc, snap)
            dist = math.hypot(nx - x, ny - y)
            if dist > radius_m:
                continue
            ang = math.atan2(ny - y, nx - x)
            diff = abs((ang - yaw_rad + math.pi) % (2 * math.pi) - math.pi)
            if diff > half_fov:
                continue
            val = get_cell_value(snap, nr, nc)
            if val is not None and classify_cell(val, cfg) == CELL_UNKNOWN:
                count += 1
    return count


def _apply_hard_gates(
    cand: ViewpointCandidate,
    snap: MapGridSnapshot,
    region: SelectedRegionGeometry,
    robot_x: float,
    robot_y: float,
    cfg: Dict[str, Any],
) -> ViewpointCandidate:
    vcfg = _cfg_val(cfg)
    robot_cfg = _cfg_robot(cfg)
    reasons: List[str] = []

    if not is_grid_inside(cand.grid_y, cand.grid_x, snap):
        reasons.append("VIEWPOINT_OUTSIDE_MAP")
    val = get_cell_value(snap, cand.grid_y, cand.grid_x)
    if val is None:
        reasons.append("VIEWPOINT_OUTSIDE_MAP")
    else:
        cat = classify_cell(val, cfg)
        if cat == CELL_UNKNOWN:
            reasons.append("VIEWPOINT_UNKNOWN_CELL")
        elif cat == CELL_OCCUPIED:
            reasons.append("VIEWPOINT_OCCUPIED_CELL")
        elif cat == CELL_UNCERTAIN:
            reasons.append("VIEWPOINT_UNCERTAIN_CELL")

    req_clear = float(robot_cfg.get("required_clearance_m", 0.33))
    if cand.clearance_m < req_clear:
        reasons.append("VIEWPOINT_LOW_CLEARANCE")

    min_fp = float(vcfg.get("minimum_footprint_free_ratio", 1.0))
    if cand.footprint_free_ratio < min_fp:
        reasons.append("VIEWPOINT_FOOTPRINT_NOT_FREE")
    if cand.footprint_unknown_count > 0:
        reasons.append("VIEWPOINT_UNKNOWN_INSIDE_ROBOT_FOOTPRINT")
    if cand.footprint_occupied_count > 0:
        reasons.append("VIEWPOINT_OCCUPIED_INSIDE_ROBOT_FOOTPRINT")

    min_local = float(vcfg.get("minimum_local_free_ratio", 0.80))
    if cand.local_free_ratio < min_local:
        reasons.append("VIEWPOINT_LOCAL_SPACE_TOO_NARROW")

    min_fd = float(vcfg.get("min_frontier_distance_m", 0.35))
    max_fd = float(vcfg.get("max_frontier_distance_m", 1.10))
    if cand.distance_to_frontier_m < min_fd:
        reasons.append("VIEWPOINT_TOO_CLOSE_TO_FRONTIER")
    if cand.distance_to_frontier_m > max_fd:
        reasons.append("VIEWPOINT_TOO_FAR_FROM_FRONTIER")

    min_rd = float(vcfg.get("min_robot_distance_m", 0.50))
    max_rd = float(vcfg.get("max_robot_distance_m", 2.50))
    if cand.distance_to_robot_m < min_rd:
        reasons.append("VIEWPOINT_TOO_CLOSE_TO_ROBOT")
    if cand.distance_to_robot_m > max_rd:
        reasons.append("VIEWPOINT_TOO_FAR_FROM_ROBOT")

    if bool(vcfg.get("require_line_of_sight", True)) and not cand.line_of_sight_clear:
        reasons.append("VIEWPOINT_LINE_OF_SIGHT_BLOCKED")

    if not math.isfinite(cand.yaw_rad) or not math.isfinite(cand.x) or not math.isfinite(cand.y):
        reasons.append("VIEWPOINT_POSE_INVALID")

    cand.rejection_reasons = reasons
    cand.hard_gate_passed = len(reasons) == 0
    if not cand.hard_gate_passed:
        cand.score = None
    cand.path_checked = False
    cand.reachable = None
    return cand


def _score_candidate(
    cand: ViewpointCandidate,
    robot_yaw_rad: float,
    cfg: Dict[str, Any],
) -> ViewpointCandidate:
    scfg = _cfg_score(cfg)
    vcfg = _cfg_val(cfg)
    weights = scfg.get("weights", {})
    req_clear = float(_cfg_robot(cfg).get("required_clearance_m", 0.33))
    max_clear = req_clear + 0.5
    clearance_score = _clamp((cand.clearance_m - req_clear) / max(max_clear - req_clear, 1e-6))
    vis_score = _clamp(cand.estimated_visible_unknown_cells / 50.0)
    local_score = _clamp(cand.local_free_ratio)
    pref_fd = (float(vcfg.get("min_frontier_distance_m", 0.35)) + float(vcfg.get("max_frontier_distance_m", 1.10))) / 2.0
    fd_tol = float(vcfg.get("max_frontier_distance_m", 1.10)) - float(vcfg.get("min_frontier_distance_m", 0.35))
    frontier_dist_score = _clamp(1.0 - abs(cand.distance_to_frontier_m - pref_fd) / max(fd_tol, 1e-6))
    pref_rd = float(vcfg.get("preferred_robot_distance_m", 1.50))
    rd_tol = float(vcfg.get("max_robot_distance_m", 2.50)) - float(vcfg.get("min_robot_distance_m", 0.50))
    robot_dist_score = _clamp(1.0 - abs(cand.distance_to_robot_m - pref_rd) / max(rd_tol, 1e-6))
    novelty = _clamp(cand.trajectory_novelty_score)
    turn_diff = abs(cand.turn_from_robot_yaw_deg)
    turn_score = _clamp(1.0 - turn_diff / 180.0)

    components = {
        "clearance_score": round(clearance_score, 4),
        "visible_unknown_score": round(vis_score, 4),
        "local_free_score": round(local_score, 4),
        "frontier_distance_score": round(frontier_dist_score, 4),
        "robot_distance_score": round(robot_dist_score, 4),
        "trajectory_novelty_score": round(novelty, 4),
        "turn_cost_score": round(turn_score, 4),
    }
    score = (
        weights.get("clearance", 0.25) * clearance_score
        + weights.get("visible_unknown", 0.20) * vis_score
        + weights.get("local_free_space", 0.15) * local_score
        + weights.get("frontier_distance_preference", 0.15) * frontier_dist_score
        + weights.get("robot_distance_preference", 0.10) * robot_dist_score
        + weights.get("trajectory_novelty", 0.10) * novelty
        + weights.get("turn_cost", 0.05) * turn_score
    )
    obs_pen = _clamp(cand.recent_observation_penalty) * 0.05
    score = _clamp(score - obs_pen)
    cand.score_components = components
    cand.score = round(score, 6)
    return cand


def _candidate_sort_key(c: ViewpointCandidate) -> Tuple[Any, ...]:
    return (
        -(c.score or 0.0),
        -c.clearance_m,
        -c.estimated_visible_unknown_cells,
        c.distance_to_robot_m,
        c.grid_y,
        c.grid_x,
        c.candidate_id,
    )


def _deduplicate_candidates(
    candidates: List[ViewpointCandidate],
    cfg: Dict[str, Any],
) -> List[ViewpointCandidate]:
    scfg = _cfg_sel(cfg)
    dist_m = float(scfg.get("deduplication_distance_m", 0.20))
    yaw_deg = float(scfg.get("deduplication_yaw_difference_deg", 20.0))
    sorted_c = sorted(candidates, key=_candidate_sort_key)
    kept: List[ViewpointCandidate] = []
    for cand in sorted_c:
        duplicate = False
        for other in kept:
            if (
                math.hypot(cand.x - other.x, cand.y - other.y) <= dist_m
                and abs(cand.yaw_deg - other.yaw_deg) <= yaw_deg
            ):
                duplicate = True
                break
        if not duplicate:
            kept.append(cand)
    return kept


def _generate_id(prefix: str = "VP") -> str:
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    return f"{prefix}_{ts}"


def generate_safe_viewpoints(
    snap: MapGridSnapshot,
    region: SelectedRegionGeometry,
    robot_x: float,
    robot_y: float,
    robot_yaw_rad: float,
    cfg: Dict[str, Any],
    *,
    generation_id: str = "",
) -> ViewpointGenerationResult:
    """Generate ranked safe viewpoint candidates for a Qwen-selected region."""
    map_errors = snap.validate()
    region_errors = region.validate()
    rejection_summary: Dict[str, int] = {}

    def _count(reasons: Sequence[str]) -> None:
        for r in reasons:
            rejection_summary[r] = rejection_summary.get(r, 0) + 1

    if map_errors or region_errors:
        all_err = map_errors + region_errors
        _count(all_err)
        return ViewpointGenerationResult(
            snapshot_id=region.snapshot_id,
            region_label=region.region_label,
            track_id=region.track_id,
            map_stamp_sec=snap.stamp_sec,
            generation_id=generation_id or _generate_id(),
            raw_candidate_count=0,
            accepted_candidate_count=0,
            rejected_candidate_count=0,
            accepted_candidates=[],
            rejected_candidates=[],
            rejection_reason_summary=rejection_summary,
            selected_candidate_id=None,
        )

    raw_specs, gen_errors = generate_raw_candidates(region, snap, cfg)
    _count(gen_errors)

    accepted: List[ViewpointCandidate] = []
    rejected: List[ViewpointCandidate] = []
    cid = 0

    for spec in raw_specs:
        local_cells = _local_search_candidates(spec["theory_x"], spec["theory_y"], snap, cfg)
        if not local_cells:
            rej = ViewpointCandidate(
                candidate_id=f"C{cid:04d}",
                region_label=region.region_label,
                track_id=region.track_id,
                grid_x=-1,
                grid_y=-1,
                x=spec["theory_x"],
                y=spec["theory_y"],
                yaw_rad=0.0,
                yaw_deg=0.0,
                source_frontier_grid_x=spec["source_col"],
                source_frontier_grid_y=spec["source_row"],
                source_frontier_x=spec["source_fx"],
                source_frontier_y=spec["source_fy"],
                stand_off_distance_m=spec["stand_off"],
                distance_to_robot_m=math.hypot(spec["theory_x"] - robot_x, spec["theory_y"] - robot_y),
                distance_to_frontier_m=point_to_frontier_distance(
                    spec["theory_x"], spec["theory_y"], region.frontier_points_map
                ),
                clearance_m=0.0,
                footprint_free_ratio=0.0,
                local_free_ratio=0.0,
                line_of_sight_clear=False,
                line_of_sight_length_m=0.0,
                estimated_visible_unknown_cells=0,
                trajectory_novelty_score=region.trajectory_novelty_score,
                recent_observation_penalty=region.trajectory_revisit_penalty,
                hard_gate_passed=False,
                rejection_reasons=["VIEWPOINT_NO_LOCAL_FREE_CELL"],
            )
            rejected.append(rej)
            _count(rej.rejection_reasons)
            cid += 1
            continue

        for row, col in local_cells[:1]:
            x, y = grid_to_map(row, col, snap)
            yaw_rad = math.atan2(
                spec["source_fy"] - y,
                spec["source_fx"] - x,
            )
            yaw_deg = math.degrees(yaw_rad)
            turn = abs(math.degrees(normalize_angle_rad(yaw_rad - robot_yaw_rad)))
            clearance = _estimate_clearance(row, col, snap, cfg)
            fp_ratio, unk_c, occ_c = _footprint_check(row, col, snap, cfg)
            local_ratio = _local_free_ratio(row, col, snap, cfg)
            los_ok, los_len, block_cell, block_val = check_line_of_sight(
                row, col, spec["source_row"], spec["source_col"], snap, cfg
            )
            vis_unknown = estimate_visible_unknown_cells(x, y, yaw_rad, snap, cfg)
            cand = ViewpointCandidate(
                candidate_id=f"C{cid:04d}",
                region_label=region.region_label,
                track_id=region.track_id,
                grid_x=col,
                grid_y=row,
                x=x,
                y=y,
                yaw_rad=yaw_rad,
                yaw_deg=yaw_deg,
                source_frontier_grid_x=spec["source_col"],
                source_frontier_grid_y=spec["source_row"],
                source_frontier_x=spec["source_fx"],
                source_frontier_y=spec["source_fy"],
                stand_off_distance_m=spec["stand_off"],
                distance_to_robot_m=math.hypot(x - robot_x, y - robot_y),
                distance_to_frontier_m=point_to_frontier_distance(x, y, region.frontier_points_map),
                clearance_m=clearance,
                footprint_free_ratio=fp_ratio,
                local_free_ratio=local_ratio,
                line_of_sight_clear=los_ok,
                line_of_sight_length_m=los_len,
                estimated_visible_unknown_cells=vis_unknown,
                trajectory_novelty_score=region.trajectory_novelty_score,
                recent_observation_penalty=region.trajectory_revisit_penalty,
                hard_gate_passed=False,
                turn_from_robot_yaw_deg=turn,
                line_of_sight_block_cell=block_cell,
                line_of_sight_block_value=block_val,
                footprint_unknown_count=unk_c,
                footprint_occupied_count=occ_c,
            )
            cand = _apply_hard_gates(cand, snap, region, robot_x, robot_y, cfg)
            if cand.hard_gate_passed:
                cand = _score_candidate(cand, robot_yaw_rad, cfg)
                accepted.append(cand)
            else:
                rejected.append(cand)
                _count(cand.rejection_reasons)
            cid += 1

    accepted = _deduplicate_candidates(accepted, cfg)
    max_out = int(_cfg_sel(cfg).get("max_output_candidates", 5))
    accepted.sort(key=_candidate_sort_key)
    accepted = accepted[:max_out]

    selected_id = accepted[0].candidate_id if accepted else None
    return ViewpointGenerationResult(
        snapshot_id=region.snapshot_id,
        region_label=region.region_label,
        track_id=region.track_id,
        map_stamp_sec=snap.stamp_sec,
        generation_id=generation_id or _generate_id(),
        raw_candidate_count=len(raw_specs),
        accepted_candidate_count=len(accepted),
        rejected_candidate_count=len(rejected),
        accepted_candidates=accepted,
        rejected_candidates=rejected,
        rejection_reason_summary=rejection_summary,
        selected_candidate_id=selected_id,
    )


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def map_grid_from_snapshot_dict(
    snapshot: Dict[str, Any],
    occupancy_data: Sequence[int],
    *,
    origin_yaw: float = 0.0,
) -> MapGridSnapshot:
    meta = snapshot.get("map_metadata", {})
    mcfg = snapshot.get("map_config", {})
    return MapGridSnapshot(
        frame_id=str(meta.get("frame_id", "map")),
        stamp_sec=float(snapshot.get("map_stamp", 0.0)),
        width=int(meta.get("width", 0)),
        height=int(meta.get("height", 0)),
        resolution=float(meta.get("resolution", 0.05)),
        origin_x=float(meta.get("origin_x", 0.0)),
        origin_y=float(meta.get("origin_y", 0.0)),
        origin_yaw=float(origin_yaw),
        data=list(occupancy_data),
        free_threshold=int(mcfg.get("free_max_value", 20)),
        occupied_threshold=int(mcfg.get("occupied_min_value", 65)),
    )


def selected_region_from_geometry(
    geometry: Dict[str, Any],
    label: str,
    snapshot_id: str = "",
) -> SelectedRegionGeometry:
    regions = geometry.get("regions", {})
    entry = regions.get(label, {})
    return SelectedRegionGeometry(
        snapshot_id=snapshot_id or str(geometry.get("snapshot_id", "")),
        region_label=label,
        internal_region_id=str(entry.get("internal_region_id", "")),
        track_id=str(entry.get("track_id", "")),
        centroid_x=float(entry.get("centroid_x", 0.0)),
        centroid_y=float(entry.get("centroid_y", 0.0)),
        frontier_cells_grid=[list(c) for c in entry.get("frontier_cells_grid", [])],
        frontier_points_map=[list(p) for p in entry.get("frontier_points_map", [])],
        bbox_grid=list(entry.get("bbox_grid", [])),
        bearing_global_deg=float(entry.get("bearing_global_deg", 0.0)),
        unknown_gain_cells=int(entry.get("unknown_gain_cells", 0)),
        minimum_clearance_m=float(entry.get("minimum_clearance_m", 0.0)),
        trajectory_novelty_score=float(entry.get("trajectory_novelty_score", 1.0)),
        trajectory_revisit_penalty=float(entry.get("trajectory_revisit_penalty", 0.0)),
        distance_to_nearest_observation_pose_m=float(
            entry.get("distance_to_nearest_observation_pose_m", float("inf"))
        ),
        geo_score=float(entry.get("geo_score", 0.0)),
        stable=bool(entry.get("stable", False)),
        snapshot_eligible=bool(entry.get("snapshot_eligible", False)),
        blacklisted=bool(entry.get("blacklisted", False)),
    )


def result_to_dict(
    result: ViewpointGenerationResult,
    *,
    bundle_id: str = "",
    map_fingerprint: str = "",
    region_geometry_fingerprint: str = "",
    contract_version: str = "",
    safe_viewpoint_schema_version: str = "",
) -> Dict[str, Any]:
    from src.planning.exploration_contracts import (  # noqa: WPS433
        EXPLORATION_CONTRACT_VERSION,
        SAFE_VIEWPOINT_SCHEMA_VERSION,
    )

    return {
        "contract_version": contract_version or EXPLORATION_CONTRACT_VERSION,
        "safe_viewpoint_schema_version": safe_viewpoint_schema_version or SAFE_VIEWPOINT_SCHEMA_VERSION,
        "schema_version": safe_viewpoint_schema_version or SAFE_VIEWPOINT_SCHEMA_VERSION,
        "bundle_id": bundle_id,
        "map_fingerprint": map_fingerprint,
        "region_geometry_fingerprint": region_geometry_fingerprint,
        "snapshot_id": result.snapshot_id,
        "region_label": result.region_label,
        "track_id": result.track_id,
        "map_stamp_sec": result.map_stamp_sec,
        "generation_id": result.generation_id,
        "raw_candidate_count": result.raw_candidate_count,
        "accepted_candidate_count": result.accepted_candidate_count,
        "rejected_candidate_count": result.rejected_candidate_count,
        "accepted_candidates": [asdict(c) for c in result.accepted_candidates],
        "rejected_candidates": [asdict(c) for c in result.rejected_candidates],
        "rejection_reason_summary": dict(result.rejection_reason_summary),
        "selected_candidate_id": result.selected_candidate_id,
        "algorithm_selected_candidate": result.selected_candidate_id,
        "algorithm_version": result.algorithm_version,
        "path_checked": False,
        "reachable": None,
        "nav2_cost": None,
        "path_length_m": None,
    }


def verify_map_data_unchanged(before: Sequence[int], after: Sequence[int]) -> bool:
    return list(before) == list(after)


def deep_copy_map_data(data: Sequence[int]) -> List[int]:
    return copy.deepcopy(list(data))
