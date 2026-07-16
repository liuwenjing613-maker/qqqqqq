#!/usr/bin/env python3
"""Pure trajectory memory, polyline geometry, and region novelty scoring — no ROS."""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None  # type: ignore[assignment]

from src.planning.trajectory_tf_validation import validate_trajectory_tf_config


def _clamp(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


def _normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_diff_deg(a_rad: float, b_rad: float) -> float:
    diff = abs(math.degrees(_normalize_angle_rad(b_rad - a_rad)))
    return diff if diff <= 180.0 else 360.0 - diff


CREATION_REASONS = frozenset(
    {"FIRST_VERTEX", "DISTANCE_THRESHOLD", "YAW_THRESHOLD", "TIME_THRESHOLD"}
)


@dataclass
class TrajectoryPoseSample:
    stamp_sec: float
    x: float
    y: float
    yaw_rad: float
    tf_stamp_sec: float
    tf_age_s: float
    valid: bool
    rejection_reason: str = ""


@dataclass
class TrajectoryVertex:
    vertex_id: int
    stamp_sec: float
    x: float
    y: float
    yaw_rad: float
    distance_from_previous_m: float
    yaw_change_from_previous_deg: float
    elapsed_from_previous_s: float
    creation_reason: str


@dataclass
class TrajectorySession:
    trajectory_session_id: str
    start_time: str
    map_frame: str
    raw_samples: List[TrajectoryPoseSample] = field(default_factory=list)
    vertices: List[TrajectoryVertex] = field(default_factory=list)
    trajectory_length_m: float = 0.0
    revision: int = 0
    observation_pose_ids: List[str] = field(default_factory=list)


@dataclass
class TrajectoryRegionMetrics:
    nearest_trajectory_distance_m: float = float("inf")
    nearest_trajectory_segment_index: int = -1
    nearby_trajectory_vertex_count: int = 0
    nearby_recent_trajectory_count: int = 0
    distance_to_nearest_observation_pose_m: float = float("inf")
    last_nearby_visit_age_s: float = float("inf")
    trajectory_density_score: float = 0.0
    trajectory_novelty_score: float = 1.0
    trajectory_revisit_penalty: float = 0.0


def _trajectory_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("trajectory", {})


def validate_trajectory_config(cfg: Dict[str, Any]) -> List[str]:
    """Return configuration errors for trajectory section."""
    errors: List[str] = []
    tcfg = _trajectory_cfg(cfg)
    if not tcfg:
        return errors
    if not bool(tcfg.get("enabled", True)):
        return errors

    sample_period = float(tcfg.get("sample_period_s", 1.0))
    if sample_period <= 0:
        errors.append("trajectory.sample_period_s must be > 0")

    max_tf_age = float(tcfg.get("max_tf_age_s", 1.50))
    if max_tf_age <= 0:
        errors.append("trajectory.max_tf_age_s must be > 0")

    errors.extend(validate_trajectory_tf_config(cfg))

    min_dist = float(tcfg.get("min_vertex_distance_m", 0.05))
    if min_dist < 0:
        errors.append("trajectory.min_vertex_distance_m must be >= 0")

    min_yaw = float(tcfg.get("min_vertex_yaw_change_deg", 10.0))
    if min_yaw < 0:
        errors.append("trajectory.min_vertex_yaw_change_deg must be >= 0")

    max_interval = float(tcfg.get("max_vertex_interval_s", 5.0))
    if max_interval <= 0:
        errors.append("trajectory.max_vertex_interval_s must be > 0")

    min_time_dist = float(tcfg.get("min_time_vertex_distance_m", 0.01))
    if min_time_dist < 0:
        errors.append("trajectory.min_time_vertex_distance_m must be >= 0")

    min_time_yaw = float(tcfg.get("min_time_vertex_yaw_change_deg", 2.0))
    if min_time_yaw < 0:
        errors.append("trajectory.min_time_vertex_yaw_change_deg must be >= 0")

    visited_r = float(tcfg.get("visited_corridor_radius_m", 0.35))
    if visited_r < 0:
        errors.append("trajectory.visited_corridor_radius_m must be >= 0")

    local_r = float(tcfg.get("local_region_analysis_radius_m", 0.80))
    if local_r <= 0:
        errors.append("trajectory.local_region_analysis_radius_m must be > 0")

    strong_r = float(tcfg.get("strong_revisit_distance_m", 0.45))
    weak_r = float(tcfg.get("weak_revisit_distance_m", 1.20))
    if strong_r > weak_r:
        errors.append("trajectory.strong_revisit_distance_m must be <= weak_revisit_distance_m")

    return errors


def point_to_segment_distance(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    """Shortest distance from point (px, py) to segment (x1,y1)-(x2,y2)."""
    dx = x2 - x1
    dy = y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= 1e-12:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    t = _clamp(t, 0.0, 1.0)
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def point_to_polyline_distance(
    px: float,
    py: float,
    polyline: Sequence[Tuple[float, float]],
) -> Tuple[float, int]:
    """Return (min distance, segment index) for polyline as (x,y) pairs."""
    if not polyline:
        return float("inf"), -1
    if len(polyline) == 1:
        x0, y0 = polyline[0]
        return math.hypot(px - x0, py - y0), 0

    best_dist = float("inf")
    best_idx = 0
    for i in range(len(polyline) - 1):
        x1, y1 = polyline[i]
        x2, y2 = polyline[i + 1]
        d = point_to_segment_distance(px, py, x1, y1, x2, y2)
        if d < best_dist:
            best_dist = d
            best_idx = i
    return best_dist, best_idx


def nearest_trajectory_distance(
    px: float,
    py: float,
    vertices: Sequence[TrajectoryVertex],
) -> Tuple[float, int]:
    polyline = [(v.x, v.y) for v in vertices]
    return point_to_polyline_distance(px, py, polyline)


def polyline_length(vertices: Sequence[TrajectoryVertex]) -> float:
    if len(vertices) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(vertices)):
        total += math.hypot(
            vertices[i].x - vertices[i - 1].x,
            vertices[i].y - vertices[i - 1].y,
        )
    return total


def should_create_vertex(
    last_vertex: Optional[TrajectoryVertex],
    sample: TrajectoryPoseSample,
    cfg: Dict[str, Any],
) -> Optional[str]:
    """Return creation reason if a new vertex should be added, else None."""
    tcfg = _trajectory_cfg(cfg)
    if not sample.valid:
        return None
    if last_vertex is None:
        return "FIRST_VERTEX"

    dist = math.hypot(sample.x - last_vertex.x, sample.y - last_vertex.y)
    yaw_change = _yaw_diff_deg(last_vertex.yaw_rad, sample.yaw_rad)
    elapsed = sample.stamp_sec - last_vertex.stamp_sec

    if dist >= float(tcfg.get("min_vertex_distance_m", 0.05)):
        return "DISTANCE_THRESHOLD"
    if yaw_change >= float(tcfg.get("min_vertex_yaw_change_deg", 10.0)):
        return "YAW_THRESHOLD"
    if elapsed >= float(tcfg.get("max_vertex_interval_s", 5.0)):
        min_time_dist = float(tcfg.get("min_time_vertex_distance_m", 0.01))
        min_time_yaw = float(tcfg.get("min_time_vertex_yaw_change_deg", 2.0))
        if dist >= min_time_dist or yaw_change >= min_time_yaw:
            return "TIME_THRESHOLD"
    return None


def add_trajectory_vertex(
    session: TrajectorySession,
    sample: TrajectoryPoseSample,
    creation_reason: str,
) -> TrajectoryVertex:
    last = session.vertices[-1] if session.vertices else None
    dist_prev = 0.0 if last is None else math.hypot(sample.x - last.x, sample.y - last.y)
    yaw_prev = 0.0 if last is None else _yaw_diff_deg(last.yaw_rad, sample.yaw_rad)
    elapsed_prev = 0.0 if last is None else sample.stamp_sec - last.stamp_sec
    vid = len(session.vertices) + 1
    vertex = TrajectoryVertex(
        vertex_id=vid,
        stamp_sec=sample.stamp_sec,
        x=sample.x,
        y=sample.y,
        yaw_rad=sample.yaw_rad,
        distance_from_previous_m=dist_prev,
        yaw_change_from_previous_deg=yaw_prev,
        elapsed_from_previous_s=elapsed_prev,
        creation_reason=creation_reason,
    )
    session.vertices.append(vertex)
    session.trajectory_length_m = polyline_length(session.vertices)
    return vertex


def add_pose_sample(
    session: TrajectorySession,
    cfg: Dict[str, Any],
    *,
    stamp_sec: float,
    x: float,
    y: float,
    yaw_rad: float,
    tf_stamp_sec: float = 0.0,
    tf_age_s: float = 0.0,
    valid: bool = True,
    rejection_reason: str = "",
) -> Tuple[TrajectoryPoseSample, Optional[TrajectoryVertex]]:
    """Append raw sample; optionally create vertex. Returns (sample, vertex_or_none)."""
    tcfg = _trajectory_cfg(cfg)
    max_raw = int(tcfg.get("max_raw_samples", 20000))
    max_vertices = int(tcfg.get("max_vertices", 10000))

    sample = TrajectoryPoseSample(
        stamp_sec=stamp_sec,
        x=x,
        y=y,
        yaw_rad=yaw_rad,
        tf_stamp_sec=tf_stamp_sec,
        tf_age_s=tf_age_s,
        valid=valid,
        rejection_reason=rejection_reason,
    )
    session.raw_samples.append(sample)
    if len(session.raw_samples) > max_raw:
        session.raw_samples = session.raw_samples[-max_raw:]

    vertex: Optional[TrajectoryVertex] = None
    if valid:
        last_v = session.vertices[-1] if session.vertices else None
        reason = should_create_vertex(last_v, sample, cfg)
        if reason is not None and len(session.vertices) < max_vertices:
            vertex = add_trajectory_vertex(session, sample, reason)
            session.revision += 1
    return sample, vertex


def compute_trajectory_region_metrics(
    region_x: float,
    region_y: float,
    vertices: Sequence[TrajectoryVertex],
    observation_poses: Sequence[Dict[str, Any]],
    cfg: Dict[str, Any],
    now_s: float,
) -> TrajectoryRegionMetrics:
    """Compute trajectory density, novelty, and revisit penalty for a candidate region."""
    tcfg = _trajectory_cfg(cfg)
    metrics = TrajectoryRegionMetrics()

    if not vertices:
        metrics.trajectory_novelty_score = 1.0
        metrics.trajectory_revisit_penalty = 0.0
        metrics.trajectory_density_score = 0.0
        metrics.nearest_trajectory_distance_m = float("inf")
        metrics.nearest_trajectory_segment_index = -1
    else:
        nearest_dist, seg_idx = nearest_trajectory_distance(region_x, region_y, vertices)
        metrics.nearest_trajectory_distance_m = nearest_dist
        metrics.nearest_trajectory_segment_index = seg_idx

        local_r = float(tcfg.get("local_region_analysis_radius_m", 0.80))
        recent_window = float(tcfg.get("recent_path_window_s", 60.0))
        strong_r = float(tcfg.get("strong_revisit_distance_m", 0.45))
        weak_r = float(tcfg.get("weak_revisit_distance_m", 1.20))

        nearby = 0
        recent_nearby = 0
        last_visit_age = float("inf")
        for v in vertices:
            d = math.hypot(region_x - v.x, region_y - v.y)
            if d <= local_r:
                nearby += 1
                age = max(0.0, now_s - v.stamp_sec)
                last_visit_age = min(last_visit_age, age)
                if age <= recent_window:
                    recent_nearby += 1

        metrics.nearby_trajectory_vertex_count = nearby
        metrics.nearby_recent_trajectory_count = recent_nearby
        metrics.last_nearby_visit_age_s = last_visit_age

        # Density from nearest polyline distance and local vertex count
        if nearest_dist <= strong_r:
            dist_factor = 1.0
        elif nearest_dist >= weak_r:
            dist_factor = 0.0
        else:
            dist_factor = 1.0 - (nearest_dist - strong_r) / max(weak_r - strong_r, 1e-6)

        count_factor = _clamp(nearby / max(len(vertices), 1))
        recent_factor = _clamp(recent_nearby / max(nearby, 1)) if nearby > 0 else 0.0
        metrics.trajectory_density_score = _clamp(
            0.55 * dist_factor + 0.30 * count_factor + 0.15 * recent_factor
        )
        metrics.trajectory_novelty_score = _clamp(1.0 - metrics.trajectory_density_score)
        metrics.trajectory_revisit_penalty = _clamp(metrics.trajectory_density_score)

    # Observation pose proximity (separate from polyline)
    min_obs = float("inf")
    for pose in observation_poses:
        px = float(pose.get("x", 0.0))
        py = float(pose.get("y", 0.0))
        d = math.hypot(region_x - px, region_y - py)
        min_obs = min(min_obs, d)
    metrics.distance_to_nearest_observation_pose_m = min_obs

    return metrics


def _generate_session_id() -> str:
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    return f"TRJ_{ts}"


def _sample_from_dict(d: Dict[str, Any]) -> TrajectoryPoseSample:
    return TrajectoryPoseSample(
        stamp_sec=float(d.get("stamp_sec", 0.0)),
        x=float(d.get("x", 0.0)),
        y=float(d.get("y", 0.0)),
        yaw_rad=float(d.get("yaw_rad", 0.0)),
        tf_stamp_sec=float(d.get("tf_stamp_sec", 0.0)),
        tf_age_s=float(d.get("tf_age_s", 0.0)),
        valid=bool(d.get("valid", True)),
        rejection_reason=str(d.get("rejection_reason", "")),
    )


def _vertex_from_dict(d: Dict[str, Any]) -> TrajectoryVertex:
    return TrajectoryVertex(
        vertex_id=int(d.get("vertex_id", 0)),
        stamp_sec=float(d.get("stamp_sec", 0.0)),
        x=float(d.get("x", 0.0)),
        y=float(d.get("y", 0.0)),
        yaw_rad=float(d.get("yaw_rad", 0.0)),
        distance_from_previous_m=float(d.get("distance_from_previous_m", 0.0)),
        yaw_change_from_previous_deg=float(d.get("yaw_change_from_previous_deg", 0.0)),
        elapsed_from_previous_s=float(d.get("elapsed_from_previous_s", 0.0)),
        creation_reason=str(d.get("creation_reason", "")),
    )


def session_to_dict(session: TrajectorySession) -> Dict[str, Any]:
    return {
        "trajectory_session_id": session.trajectory_session_id,
        "start_time": session.start_time,
        "map_frame": session.map_frame,
        "raw_samples": [asdict(s) for s in session.raw_samples],
        "vertices": [asdict(v) for v in session.vertices],
        "trajectory_length_m": session.trajectory_length_m,
        "revision": session.revision,
        "observation_pose_ids": list(session.observation_pose_ids),
    }


def session_from_dict(raw: Dict[str, Any]) -> TrajectorySession:
    return TrajectorySession(
        trajectory_session_id=str(raw.get("trajectory_session_id", _generate_session_id())),
        start_time=str(raw.get("start_time", datetime.now(timezone.utc).isoformat())),
        map_frame=str(raw.get("map_frame", "map")),
        raw_samples=[_sample_from_dict(s) for s in raw.get("raw_samples", [])],
        vertices=[_vertex_from_dict(v) for v in raw.get("vertices", [])],
        trajectory_length_m=float(raw.get("trajectory_length_m", 0.0)),
        revision=int(raw.get("revision", 0)),
        observation_pose_ids=list(raw.get("observation_pose_ids", [])),
    )


def save_atomic(session: TrajectorySession, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    payload = session_to_dict(session)
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    with open(tmp, "rb") as fh:
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def load_safe(path: Path) -> Tuple[TrajectorySession, Dict[str, Any]]:
    """Load session; on corruption return empty session and diagnostic info."""
    path = Path(path)
    diag: Dict[str, Any] = {"load_status": "OK", "path": str(path)}
    if not path.is_file():
        diag["load_status"] = "MISSING"
        return _empty_session(), diag
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            raise ValueError("root not object")
        session = session_from_dict(raw)
        session.trajectory_length_m = polyline_length(session.vertices)
        diag["load_status"] = "OK"
        diag["vertex_count"] = len(session.vertices)
        return session, diag
    except (json.JSONDecodeError, OSError, TypeError, ValueError) as exc:
        diag["load_status"] = "CORRUPT"
        diag["error"] = str(exc)
        diag["note"] = "original file preserved; starting empty session"
        return _empty_session(), diag


def _empty_session(map_frame: str = "map") -> TrajectorySession:
    return TrajectorySession(
        trajectory_session_id=_generate_session_id(),
        start_time=datetime.now(timezone.utc).isoformat(),
        map_frame=map_frame,
    )


def vertices_from_xy(polyline: Sequence[Tuple[float, float]]) -> List[TrajectoryVertex]:
    """Build minimal TrajectoryVertex list for corridor rasterization."""
    verts: List[TrajectoryVertex] = []
    prev: Optional[Tuple[float, float]] = None
    for i, (x, y) in enumerate(polyline):
        dist = 0.0 if prev is None else math.hypot(x - prev[0], y - prev[1])
        verts.append(
            TrajectoryVertex(
                vertex_id=i + 1,
                stamp_sec=float(i),
                x=float(x),
                y=float(y),
                yaw_rad=0.0,
                distance_from_previous_m=dist,
                yaw_change_from_previous_deg=0.0,
                elapsed_from_previous_s=0.0,
                creation_reason="synthetic_polyline",
            )
        )
        prev = (x, y)
    return verts


def _free_mask_allows(free_mask: Optional[Any], row: int, col: int) -> bool:
    if free_mask is None:
        return True
    return bool(free_mask[row, col])


def rasterize_visited_corridor(
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    vertices: Sequence[TrajectoryVertex],
    corridor_radius_m: float,
    free_mask: Optional[Any] = None,
) -> List[int]:
    """Return flat OccupancyGrid data: 0=unvisited, 100=visited corridor.

    When free_mask is provided (H x W bool), visited stamps only apply on True cells.
    """
    data = [0] * (width * height)
    if not vertices or corridor_radius_m <= 0 or resolution <= 0:
        return data

    radius_cells = int(math.ceil(corridor_radius_m / resolution))
    polyline = [(v.x, v.y) for v in vertices]

    def world_to_col(x: float) -> int:
        return int((x - origin_x) / resolution)

    def world_to_row(y: float) -> int:
        return int((y - origin_y) / resolution)

    for seg_idx in range(max(len(polyline) - 1, 1)):
        if len(polyline) == 1:
            x1, y1 = polyline[0]
            x2, y2 = polyline[0]
        else:
            x1, y1 = polyline[seg_idx]
            x2, y2 = polyline[seg_idx + 1]

        seg_len = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(seg_len / resolution) + 1, radius_cells * 2 + 1)
        for step in range(steps + 1):
            t = step / max(steps, 1)
            wx = x1 + t * (x2 - x1)
            wy = y1 + t * (y2 - y1)
            cr = world_to_row(wy)
            cc = world_to_col(wx)
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    if dr * dr + dc * dc > radius_cells * radius_cells:
                        continue
                    r, c = cr + dr, cc + dc
                    if 0 <= r < height and 0 <= c < width:
                        if _free_mask_allows(free_mask, r, c):
                            data[r * width + c] = 100
    return data


class RobotTrajectoryStore:
    """In-memory trajectory session with optional persistence."""

    def __init__(
        self,
        cfg: Dict[str, Any],
        runtime_path: Path,
        *,
        map_frame: str = "map",
    ) -> None:
        self.cfg = cfg
        self.runtime_path = Path(runtime_path)
        self._last_save_s = 0.0
        tcfg = _trajectory_cfg(cfg)
        persist = bool(tcfg.get("persist_across_node_restart", False))
        if persist and bool(tcfg.get("enabled", True)):
            self.session, self._load_diag = load_safe(self.runtime_path)
            if self.session.map_frame != map_frame:
                self.session.map_frame = map_frame
        else:
            self.session = _empty_session(map_frame)
            self._load_diag = {"load_status": "NEW_SESSION"}
        self.session.map_frame = map_frame

    @property
    def trajectory_session_id(self) -> str:
        return self.session.trajectory_session_id

    @property
    def trajectory_revision(self) -> int:
        return self.session.revision

    @property
    def load_diagnostic(self) -> Dict[str, Any]:
        return dict(self._load_diag)

    def reset(self) -> None:
        map_frame = self.session.map_frame
        self.session = _empty_session(map_frame)
        self.session.revision = 0
        self._load_diag = {"load_status": "RESET"}

    def add_observation_pose_id(self, pose_id: str) -> None:
        if pose_id and pose_id not in self.session.observation_pose_ids:
            self.session.observation_pose_ids.append(pose_id)

    def ingest_tf_pose(
        self,
        *,
        stamp_sec: float,
        x: float,
        y: float,
        yaw_rad: float,
        tf_stamp_sec: float,
        tf_age_s: float,
        status: str = "OK",
    ) -> Tuple[TrajectoryPoseSample, Optional[TrajectoryVertex], str]:
        """Append a TF-validated map-frame pose sample."""
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(yaw_rad):
            sample, _ = add_pose_sample(
                self.session,
                self.cfg,
                stamp_sec=stamp_sec,
                x=x,
                y=y,
                yaw_rad=yaw_rad,
                tf_stamp_sec=tf_stamp_sec,
                tf_age_s=tf_age_s,
                valid=False,
                rejection_reason="TRAJECTORY_POSE_INVALID",
            )
            return sample, None, "TRAJECTORY_POSE_INVALID"

        return add_pose_sample(
            self.session,
            self.cfg,
            stamp_sec=stamp_sec,
            x=x,
            y=y,
            yaw_rad=yaw_rad,
            tf_stamp_sec=tf_stamp_sec,
            tf_age_s=tf_age_s,
            valid=True,
        ) + (status,)

    def ingest_tf_rejected(
        self,
        *,
        stamp_sec: float,
        rejection_reason: str,
        x: float = 0.0,
        y: float = 0.0,
        yaw_rad: float = 0.0,
        tf_stamp_sec: float = 0.0,
        tf_age_s: float = 0.0,
    ) -> TrajectoryPoseSample:
        sample, _ = add_pose_sample(
            self.session,
            self.cfg,
            stamp_sec=stamp_sec,
            x=x,
            y=y,
            yaw_rad=yaw_rad,
            tf_stamp_sec=tf_stamp_sec,
            tf_age_s=tf_age_s,
            valid=False,
            rejection_reason=rejection_reason,
        )
        return sample

    def maybe_save(self, now_s: Optional[float] = None) -> None:
        tcfg = _trajectory_cfg(self.cfg)
        interval = float(tcfg.get("atomic_save_interval_s", 5.0))
        ts = now_s if now_s is not None else time.time()
        if ts - self._last_save_s >= interval:
            save_atomic(self.session, self.runtime_path)
            self._last_save_s = ts

    def save_atomic(self) -> None:
        save_atomic(self.session, self.runtime_path)
        self._last_save_s = time.time()

    def to_dict(self) -> Dict[str, Any]:
        return session_to_dict(self.session)

    def trajectory_json_payload(
        self,
        tf_diagnostics: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        tcfg = _trajectory_cfg(self.cfg)
        latest = self.session.vertices[-1] if self.session.vertices else None
        latest_pose: Dict[str, Any] = {}
        if latest is not None:
            latest_pose = {
                "x": latest.x,
                "y": latest.y,
                "yaw_deg": math.degrees(latest.yaw_rad),
            }
        elif self.session.raw_samples:
            s = self.session.raw_samples[-1]
            latest_pose = {"x": s.x, "y": s.y, "yaw_deg": math.degrees(s.yaw_rad)}

        valid_count = sum(1 for s in self.session.raw_samples if s.valid)
        invalid_count = len(self.session.raw_samples) - valid_count
        rejection_counts: Dict[str, int] = {}
        for s in self.session.raw_samples:
            if not s.valid and s.rejection_reason:
                rejection_counts[s.rejection_reason] = rejection_counts.get(s.rejection_reason, 0) + 1

        payload = {
            "trajectory_session_id": self.session.trajectory_session_id,
            "trajectory_revision": self.session.revision,
            "map_frame": self.session.map_frame,
            "raw_sample_count": len(self.session.raw_samples),
            "valid_raw_count": valid_count,
            "invalid_raw_count": invalid_count,
            "vertex_count": len(self.session.vertices),
            "trajectory_length_m": round(self.session.trajectory_length_m, 4),
            "latest_pose": latest_pose,
            "rejection_reason_counts": rejection_counts,
            "visited_corridor_radius_m": float(tcfg.get("visited_corridor_radius_m", 0.35)),
            "runtime_validation": "NOT_RUN",
        }
        if tf_diagnostics:
            payload.update(tf_diagnostics)
        return payload

    def build_visited_area_data(
        self,
        *,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        free_mask: Optional[Any] = None,
    ) -> List[int]:
        tcfg = _trajectory_cfg(self.cfg)
        return rasterize_visited_corridor(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            vertices=self.session.vertices,
            corridor_radius_m=float(tcfg.get("visited_corridor_radius_m", 0.35)),
            free_mask=free_mask,
        )
