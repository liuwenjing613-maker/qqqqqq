"""Deterministic trigger math for the simplified EGO <-> MAP handoff.

This module intentionally has no ROS imports.  The two trigger families are:

A. execution failure: commanded translation/rotation produced almost no odometry
   progress, after local recovery (or two conservative fallback windows);
B. exploration exhaustion: the most recent 1.2 m-wide-view corridor adds too
   little previously unseen free-space coverage compared with older history.

Keeping the math pure lets us test the dangerous decision boundary without a
chassis connected. A small courtesy to the hardware, which humans occasionally
remember deserves one.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:  # The current backend already depends on OpenCV; keep a pure fallback for tests.
    import cv2  # type: ignore
except Exception:  # pragma: no cover - exercised only on minimal environments
    cv2 = None


@dataclass(frozen=True)
class TimedCommand:
    t: float
    vx: float
    wz: float


@dataclass(frozen=True)
class TimedPose:
    t: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ProgressThresholds:
    window_s: float = 8.0
    max_sample_gap_s: float = 0.35
    pose_sample_period_s: float = 0.25
    min_linear_cmd_mps: float = 0.020
    min_linear_active_s: float = 5.0
    min_commanded_linear_m: float = 0.18
    max_actual_path_m: float = 0.08
    min_angular_cmd_radps: float = 0.020
    min_angular_active_s: float = 5.0
    min_commanded_yaw_rad: float = 0.25
    max_actual_yaw_rad: float = 0.10


@dataclass(frozen=True)
class ProgressMetrics:
    window_covered_s: float
    linear_active_s: float
    commanded_linear_m: float
    actual_path_m: float
    angular_active_s: float
    commanded_yaw_rad: float
    actual_yaw_rad: float
    translation_failed: bool
    rotation_failed: bool

    @property
    def failed(self) -> bool:
        return self.translation_failed or self.rotation_failed


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0


@dataclass(frozen=True)
class CoverageThresholds:
    corridor_radius_m: float = 0.50
    recent_path_length_m: float = 1.20
    history_gap_length_m: float = 1.00
    min_history_path_length_m: float = 0.80
    free_cell_max_value: int = 20
    max_new_area_ratio: float = 0.20
    min_valid_recent_cells: int = 80


@dataclass(frozen=True)
class CoverageMetrics:
    available: bool
    reason: str
    total_path_m: float
    recent_path_m: float
    history_path_m: float
    recent_valid_cells: int
    history_valid_cells: int
    overlap_cells: int
    new_cells: int
    new_area_ratio: Optional[float]
    low_novelty: bool
    recent_mask: Optional[np.ndarray] = None
    history_mask: Optional[np.ndarray] = None
    new_mask: Optional[np.ndarray] = None
    overlap_mask: Optional[np.ndarray] = None
    recent_points: Tuple[Tuple[float, float], ...] = ()
    history_points: Tuple[Tuple[float, float], ...] = ()


def wrap_angle(angle: float) -> float:
    """Wrap radians to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _xy_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))


def polyline_length(points: Sequence[Sequence[float]]) -> float:
    return sum(_xy_distance(a, b) for a, b in zip(points, points[1:]))


def _window_items(items: Sequence, start_t: float) -> List:
    """Include one sample before start_t so interval integration is complete."""
    if not items:
        return []
    first_idx = 0
    for idx, item in enumerate(items):
        if float(item.t) >= start_t:
            first_idx = max(0, idx - 1)
            break
    else:
        first_idx = max(0, len(items) - 1)
    return list(items[first_idx:])


def evaluate_progress(
    commands: Sequence[TimedCommand],
    poses: Sequence[TimedPose],
    now: float,
    cfg: ProgressThresholds,
) -> ProgressMetrics:
    """Evaluate independent translational and rotational progress windows."""
    start_t = float(now) - float(cfg.window_s)
    cmd = _window_items(commands, start_t)
    odo = _window_items(poses, start_t)

    linear_active_s = 0.0
    commanded_linear_m = 0.0
    angular_active_s = 0.0
    commanded_yaw_rad = 0.0
    cmd_covered = 0.0
    for a, b in zip(cmd, cmd[1:]):
        seg_start = max(float(a.t), start_t)
        seg_end = min(float(b.t), float(now))
        dt = max(0.0, min(seg_end - seg_start, float(cfg.max_sample_gap_s)))
        if dt <= 0.0:
            continue
        cmd_covered += dt
        if abs(float(a.vx)) >= float(cfg.min_linear_cmd_mps):
            linear_active_s += dt
            commanded_linear_m += abs(float(a.vx)) * dt
        if abs(float(a.wz)) >= float(cfg.min_angular_cmd_radps):
            angular_active_s += dt
            commanded_yaw_rad += abs(float(a.wz)) * dt

    # Downsample odometry before summing path/yaw. Summing every 30 Hz encoder
    # jitter step can manufacture metres of imaginary travel while the chassis is
    # physically stuck. A 0.25 s spacing still resolves 0.04 m/s motion clearly.
    sampled_odo: List[TimedPose] = []
    for item in odo:
        if not sampled_odo or float(item.t) - float(sampled_odo[-1].t) >= float(cfg.pose_sample_period_s):
            sampled_odo.append(item)
    if odo and (not sampled_odo or sampled_odo[-1] is not odo[-1]):
        sampled_odo.append(odo[-1])

    actual_path_m = 0.0
    actual_yaw_rad = 0.0
    pose_covered = 0.0
    for a, b in zip(sampled_odo, sampled_odo[1:]):
        seg_start = max(float(a.t), start_t)
        seg_end = min(float(b.t), float(now))
        dt = max(0.0, seg_end - seg_start)
        if dt <= 0.0:
            continue
        pose_covered += dt
        actual_path_m += math.hypot(float(b.x) - float(a.x), float(b.y) - float(a.y))
        actual_yaw_rad += abs(wrap_angle(float(b.yaw) - float(a.yaw)))

    window_covered_s = min(cmd_covered, pose_covered)
    enough_window = window_covered_s >= max(0.0, float(cfg.window_s) * 0.75)
    translation_failed = bool(
        enough_window
        and linear_active_s >= float(cfg.min_linear_active_s)
        and commanded_linear_m >= float(cfg.min_commanded_linear_m)
        and actual_path_m <= float(cfg.max_actual_path_m)
    )
    rotation_failed = bool(
        enough_window
        and angular_active_s >= float(cfg.min_angular_active_s)
        and commanded_yaw_rad >= float(cfg.min_commanded_yaw_rad)
        and actual_yaw_rad <= float(cfg.max_actual_yaw_rad)
    )
    return ProgressMetrics(
        window_covered_s=window_covered_s,
        linear_active_s=linear_active_s,
        commanded_linear_m=commanded_linear_m,
        actual_path_m=actual_path_m,
        angular_active_s=angular_active_s,
        commanded_yaw_rad=commanded_yaw_rad,
        actual_yaw_rad=actual_yaw_rad,
        translation_failed=translation_failed,
        rotation_failed=rotation_failed,
    )


def _cumulative_lengths(points: Sequence[Sequence[float]]) -> List[float]:
    if not points:
        return []
    out = [0.0]
    for a, b in zip(points, points[1:]):
        out.append(out[-1] + _xy_distance(a, b))
    return out


def _point_at_distance(
    points: Sequence[Sequence[float]], cumulative: Sequence[float], distance: float
) -> Tuple[float, float]:
    if not points:
        raise ValueError("empty polyline")
    d = min(max(float(distance), 0.0), float(cumulative[-1]))
    for idx in range(1, len(points)):
        if cumulative[idx] >= d:
            lo = cumulative[idx - 1]
            hi = cumulative[idx]
            if hi <= lo + 1e-12:
                return float(points[idx][0]), float(points[idx][1])
            alpha = (d - lo) / (hi - lo)
            return (
                float(points[idx - 1][0])
                + alpha * (float(points[idx][0]) - float(points[idx - 1][0])),
                float(points[idx - 1][1])
                + alpha * (float(points[idx][1]) - float(points[idx - 1][1])),
            )
    return float(points[-1][0]), float(points[-1][1])


def slice_polyline(
    points: Sequence[Sequence[float]], start_distance: float, end_distance: float
) -> List[Tuple[float, float]]:
    """Return a polyline clipped to cumulative-distance bounds."""
    if len(points) < 2:
        return [(float(p[0]), float(p[1])) for p in points]
    cumulative = _cumulative_lengths(points)
    total = cumulative[-1]
    start = min(max(float(start_distance), 0.0), total)
    end = min(max(float(end_distance), start), total)
    result: List[Tuple[float, float]] = [_point_at_distance(points, cumulative, start)]
    for p, d in zip(points[1:-1], cumulative[1:-1]):
        if start < d < end:
            result.append((float(p[0]), float(p[1])))
    end_point = _point_at_distance(points, cumulative, end)
    if not result or _xy_distance(result[-1], end_point) > 1e-9:
        result.append(end_point)
    return result


def split_history_recent(
    points: Sequence[Sequence[float]],
    recent_length_m: float,
    gap_length_m: float,
) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]], float]:
    """Split old history and recent path with a deliberately excluded gap."""
    total = polyline_length(points)
    recent_start = max(0.0, total - float(recent_length_m))
    history_end = max(0.0, recent_start - float(gap_length_m))
    history = slice_polyline(points, 0.0, history_end) if history_end > 0.0 else []
    recent = slice_polyline(points, recent_start, total) if total > 0.0 else []
    return history, recent, total


def world_to_grid(x: float, y: float, spec: GridSpec) -> Tuple[float, float]:
    dx = float(x) - float(spec.origin_x)
    dy = float(y) - float(spec.origin_y)
    c = math.cos(float(spec.origin_yaw))
    s = math.sin(float(spec.origin_yaw))
    # Inverse rotation from world into map-grid axes.
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    return local_x / float(spec.resolution), local_y / float(spec.resolution)


def grid_to_world(col: float, row: float, spec: GridSpec) -> Tuple[float, float]:
    local_x = (float(col) + 0.5) * float(spec.resolution)
    local_y = (float(row) + 0.5) * float(spec.resolution)
    c = math.cos(float(spec.origin_yaw))
    s = math.sin(float(spec.origin_yaw))
    return (
        float(spec.origin_x) + c * local_x - s * local_y,
        float(spec.origin_y) + s * local_x + c * local_y,
    )


def rasterize_corridor(
    points: Sequence[Sequence[float]], spec: GridSpec, radius_m: float
) -> np.ndarray:
    """Rasterize a polyline corridor into a boolean map mask, without OpenCV."""
    mask = np.zeros((int(spec.height), int(spec.width)), dtype=np.bool_)
    if len(points) < 2 or spec.resolution <= 0.0:
        return mask
    radius_cells = max(0, int(math.ceil(float(radius_m) / float(spec.resolution))))

    if cv2 is not None:
        pixels = []
        for x, y in points:
            col_f, row_f = world_to_grid(float(x), float(y), spec)
            pixels.append((int(round(col_f)), int(round(row_f))))
        canvas = np.zeros(mask.shape, dtype=np.uint8)
        poly = np.asarray(pixels, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(
            canvas,
            [poly],
            isClosed=False,
            color=1,
            thickness=max(1, radius_cells * 2 + 1),
            lineType=cv2.LINE_8,
        )
        cv2.circle(canvas, pixels[0], radius_cells, 1, thickness=-1)
        cv2.circle(canvas, pixels[-1], radius_cells, 1, thickness=-1)
        return canvas.astype(np.bool_)
    disk: List[Tuple[int, int]] = []
    r2 = (float(radius_m) / float(spec.resolution)) ** 2 + 1e-9
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc <= r2:
                disk.append((dr, dc))
    sample_step = max(float(spec.resolution) * 0.45, 0.01)
    for a, b in zip(points, points[1:]):
        length = _xy_distance(a, b)
        count = max(1, int(math.ceil(length / sample_step)))
        for i in range(count + 1):
            alpha = i / count
            x = float(a[0]) + alpha * (float(b[0]) - float(a[0]))
            y = float(a[1]) + alpha * (float(b[1]) - float(a[1]))
            col_f, row_f = world_to_grid(x, y, spec)
            col = int(math.floor(col_f))
            row = int(math.floor(row_f))
            for dr, dc in disk:
                rr, cc = row + dr, col + dc
                if 0 <= rr < spec.height and 0 <= cc < spec.width:
                    mask[rr, cc] = True
    return mask


def evaluate_coverage(
    path_points: Sequence[Sequence[float]],
    occupancy: np.ndarray,
    spec: GridSpec,
    cfg: CoverageThresholds,
    *,
    include_masks: bool = False,
) -> CoverageMetrics:
    """Evaluate how much genuinely new free-space the recent path corridor adds."""
    if occupancy.shape != (int(spec.height), int(spec.width)):
        raise ValueError(
            f"occupancy shape {occupancy.shape} does not match grid "
            f"{spec.height}x{spec.width}"
        )
    history, recent, total = split_history_recent(
        path_points, cfg.recent_path_length_m, cfg.history_gap_length_m
    )
    history_m = polyline_length(history)
    recent_m = polyline_length(recent)
    required_total = (
        float(cfg.recent_path_length_m)
        + float(cfg.history_gap_length_m)
        + float(cfg.min_history_path_length_m)
    )
    if total + 1e-9 < required_total:
        return CoverageMetrics(
            available=False,
            reason="insufficient_total_path",
            total_path_m=total,
            recent_path_m=recent_m,
            history_path_m=history_m,
            recent_valid_cells=0,
            history_valid_cells=0,
            overlap_cells=0,
            new_cells=0,
            new_area_ratio=None,
            low_novelty=False,
            recent_points=tuple(recent),
            history_points=tuple(history),
        )
    if history_m + 1e-9 < float(cfg.min_history_path_length_m):
        return CoverageMetrics(
            available=False,
            reason="insufficient_history_path",
            total_path_m=total,
            recent_path_m=recent_m,
            history_path_m=history_m,
            recent_valid_cells=0,
            history_valid_cells=0,
            overlap_cells=0,
            new_cells=0,
            new_area_ratio=None,
            low_novelty=False,
            recent_points=tuple(recent),
            history_points=tuple(history),
        )

    free = (occupancy >= 0) & (occupancy <= int(cfg.free_cell_max_value))
    recent_mask_all = rasterize_corridor(recent, spec, cfg.corridor_radius_m)
    history_mask_all = rasterize_corridor(history, spec, cfg.corridor_radius_m)
    recent_mask = recent_mask_all & free
    history_mask = history_mask_all & free
    overlap = recent_mask & history_mask
    new = recent_mask & ~history_mask
    recent_valid = int(np.count_nonzero(recent_mask))
    history_valid = int(np.count_nonzero(history_mask))
    overlap_cells = int(np.count_nonzero(overlap))
    new_cells = int(np.count_nonzero(new))
    if recent_valid < int(cfg.min_valid_recent_cells):
        return CoverageMetrics(
            available=False,
            reason="insufficient_recent_free_cells",
            total_path_m=total,
            recent_path_m=recent_m,
            history_path_m=history_m,
            recent_valid_cells=recent_valid,
            history_valid_cells=history_valid,
            overlap_cells=overlap_cells,
            new_cells=new_cells,
            new_area_ratio=None,
            low_novelty=False,
            recent_mask=recent_mask if include_masks else None,
            history_mask=history_mask if include_masks else None,
            new_mask=new if include_masks else None,
            overlap_mask=overlap if include_masks else None,
            recent_points=tuple(recent),
            history_points=tuple(history),
        )
    ratio = float(new_cells) / float(recent_valid)
    return CoverageMetrics(
        available=True,
        reason="ok",
        total_path_m=total,
        recent_path_m=recent_m,
        history_path_m=history_m,
        recent_valid_cells=recent_valid,
        history_valid_cells=history_valid,
        overlap_cells=overlap_cells,
        new_cells=new_cells,
        new_area_ratio=ratio,
        low_novelty=ratio <= float(cfg.max_new_area_ratio),
        recent_mask=recent_mask if include_masks else None,
        history_mask=history_mask if include_masks else None,
        new_mask=new if include_masks else None,
        overlap_mask=overlap if include_masks else None,
        recent_points=tuple(recent),
        history_points=tuple(history),
    )


def trim_path_by_distance(
    points: Sequence[Sequence[float]], max_length_m: float
) -> List[Tuple[float, float]]:
    """Keep the newest max_length_m of a polyline, clipped at the boundary."""
    total = polyline_length(points)
    if total <= float(max_length_m):
        return [(float(p[0]), float(p[1])) for p in points]
    return slice_polyline(points, total - float(max_length_m), total)
