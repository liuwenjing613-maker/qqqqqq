#!/usr/bin/env python3
"""Cross-cycle region tracking, stability gates, near guard, and geometric scoring."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from src.planning.robot_trajectory_store import (
    TrajectorySession,
    TrajectoryVertex,
    compute_trajectory_region_metrics,
)


def _clamp(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


def _bearing_diff_deg(a: float, b: float) -> float:
    diff = abs(a - b) % 360.0
    return diff if diff <= 180.0 else 360.0 - diff


def _bbox_iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
    """IoU for (row_min, row_max, col_min, col_max) inclusive boxes."""
    ar0, ar1, ac0, ac1 = a
    br0, br1, bc0, bc1 = b
    ir0, ir1 = max(ar0, br0), min(ar1, br1)
    ic0, ic1 = max(ac0, bc0), min(ac1, bc1)
    if ir0 > ir1 or ic0 > ic1:
        return 0.0
    inter = (ir1 - ir0 + 1) * (ic1 - ic0 + 1)
    area_a = (ar1 - ar0 + 1) * (ac1 - ac0 + 1)
    area_b = (br1 - br0 + 1) * (bc1 - bc0 + 1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


@dataclass
class RegionTrack:
    track_id: str
    first_seen_cycle: int
    last_seen_cycle: int
    seen_cycles: int = 1
    consecutive_seen_cycles: int = 1
    missing_cycles: int = 0
    centroid_history: List[Tuple[float, float]] = field(default_factory=list)
    bearing_history: List[float] = field(default_factory=list)
    frontier_cell_count_history: List[int] = field(default_factory=list)
    unknown_gain_history: List[int] = field(default_factory=list)
    clearance_history: List[float] = field(default_factory=list)
    selection_count: int = 0
    visit_count: int = 0
    navigation_failure_count: int = 0
    last_selected_time: Optional[str] = None
    last_visited_time: Optional[str] = None
    blacklisted: bool = False
    first_seen_time_s: float = 0.0
    last_centroid: Tuple[float, float] = (0.0, 0.0)
    last_bearing_deg: float = 0.0
    last_bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
    last_region_id: str = ""
    last_source_cluster_ids: List[str] = field(default_factory=list)


@dataclass
class GuardedRegionMetrics:
    track_id: str = ""
    stable: bool = False
    snapshot_eligible: bool = False
    persistence_cycles: int = 0
    age_s: float = 0.0
    centroid_drift_m: float = 0.0
    bearing_drift_deg: float = 0.0
    cell_count_change_ratio: float = 0.0
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


def _tracking_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("region_tracking", {})


def _stability_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("region_stability", {})


def _near_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("near_region_guard", {})


def _geo_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return cfg.get("geometric_scoring", {})


def _region_bbox(region: Any) -> Tuple[int, int, int, int]:
    return (region.row_min, region.row_max, region.col_min, region.col_max)


def _match_score(
    track: RegionTrack,
    region: Any,
    tcfg: Dict[str, Any],
) -> Optional[float]:
    cx, cy = region.centroid_x, region.centroid_y
    dist = math.hypot(cx - track.last_centroid[0], cy - track.last_centroid[1])
    if dist > float(tcfg.get("max_centroid_match_distance_m", 0.35)):
        return None
    bearing_diff = _bearing_diff_deg(region.bearing_relative_deg, track.last_bearing_deg)
    if bearing_diff > float(tcfg.get("max_bearing_difference_deg", 25.0)):
        return None
    prev_cells = track.frontier_cell_count_history[-1] if track.frontier_cell_count_history else region.frontier_cell_count
    if prev_cells > 0:
        ratio = abs(region.frontier_cell_count - prev_cells) / prev_cells
        if ratio > float(tcfg.get("max_cell_count_change_ratio", 0.60)):
            return None
    iou = _bbox_iou(track.last_bbox, _region_bbox(region))
    if iou < float(tcfg.get("min_bbox_iou", 0.05)):
        return None
    return dist + bearing_diff * 0.01 + (1.0 - iou)


def match_regions_to_tracks(
    regions: Sequence[Any],
    tracks: Dict[str, RegionTrack],
    cycle_id: int,
    now_s: float,
    cfg: Dict[str, Any],
) -> Tuple[Dict[str, RegionTrack], Dict[int, str], int]:
    """Deterministic one-to-one matching. Returns tracks, region_index->track_id, next_id_num."""
    tcfg = _tracking_cfg(cfg)
    if not bool(tcfg.get("enabled", True)):
        return tracks, {}, _next_track_num(tracks)

    active = {tid: t for tid, t in tracks.items() if t.missing_cycles <= int(tcfg.get("max_missing_cycles", 2))}
    pairs: List[Tuple[float, str, int]] = []
    for idx, region in enumerate(regions):
        for tid, track in active.items():
            score = _match_score(track, region, tcfg)
            if score is not None:
                pairs.append((score, tid, idx))
    pairs.sort(key=lambda x: (x[0], x[1], x[2]))

    used_tracks: set[str] = set()
    used_regions: set[int] = set()
    mapping: Dict[int, str] = {}
    for _, tid, idx in pairs:
        if tid in used_tracks or idx in used_regions:
            continue
        used_tracks.add(tid)
        used_regions.add(idx)
        mapping[idx] = tid

    next_num = _next_track_num(tracks)
    for idx, region in enumerate(regions):
        if idx in mapping:
            continue
        tid = f"T{next_num:06d}"
        next_num += 1
        tracks[tid] = RegionTrack(
            track_id=tid,
            first_seen_cycle=cycle_id,
            last_seen_cycle=cycle_id,
            first_seen_time_s=now_s,
            last_centroid=(region.centroid_x, region.centroid_y),
            last_bearing_deg=region.bearing_relative_deg,
            last_bbox=_region_bbox(region),
            last_region_id=region.region_id,
            last_source_cluster_ids=list(region.source_cluster_ids),
        )
        mapping[idx] = tid

    for tid, track in list(tracks.items()):
        if tid not in used_tracks and tid in active:
            track.missing_cycles += 1
            track.consecutive_seen_cycles = 0

    for idx, tid in mapping.items():
        region = regions[idx]
        track = tracks[tid]
        if tid in used_tracks:
            track.last_seen_cycle = cycle_id
            track.seen_cycles += 1
            track.consecutive_seen_cycles += 1
            track.missing_cycles = 0
        track.centroid_history.append((region.centroid_x, region.centroid_y))
        track.bearing_history.append(region.bearing_relative_deg)
        track.frontier_cell_count_history.append(region.frontier_cell_count)
        track.unknown_gain_history.append(region.unknown_gain_cells)
        track.clearance_history.append(region.minimum_clearance_m)
        track.last_centroid = (region.centroid_x, region.centroid_y)
        track.last_bearing_deg = region.bearing_relative_deg
        track.last_bbox = _region_bbox(region)
        track.last_region_id = region.region_id
        track.last_source_cluster_ids = list(region.source_cluster_ids)

    return tracks, mapping, next_num


def _next_track_num(tracks: Dict[str, RegionTrack]) -> int:
    if not tracks:
        return 1
    nums = []
    for tid in tracks:
        if tid.startswith("T") and tid[1:].isdigit():
            nums.append(int(tid[1:]))
    return max(nums, default=0) + 1


def apply_stability_gate(
    region: Any,
    track: RegionTrack,
    now_s: float,
    cfg: Dict[str, Any],
) -> GuardedRegionMetrics:
    scfg = _stability_cfg(cfg)
    m = GuardedRegionMetrics(
        track_id=track.track_id,
        persistence_cycles=track.consecutive_seen_cycles,
        age_s=max(0.0, now_s - track.first_seen_time_s),
    )
    if not bool(scfg.get("enabled", True)):
        m.stable = True
        return m

    reasons: List[str] = []
    min_cycles = int(scfg.get("min_consecutive_cycles", 3))
    if track.consecutive_seen_cycles < min_cycles:
        reasons.append("REGION_NOT_PERSISTENT")
    if m.age_s < float(scfg.get("min_age_s", 2.0)):
        reasons.append("REGION_TOO_NEW")

    if len(track.centroid_history) >= 2:
        x0, y0 = track.centroid_history[0]
        x1, y1 = track.centroid_history[-1]
        m.centroid_drift_m = math.hypot(x1 - x0, y1 - y0)
        if m.centroid_drift_m > float(scfg.get("max_centroid_drift_m", 0.20)):
            reasons.append("REGION_CENTROID_UNSTABLE")

    if len(track.bearing_history) >= 2:
        m.bearing_drift_deg = _bearing_diff_deg(track.bearing_history[0], track.bearing_history[-1])
        if m.bearing_drift_deg > float(scfg.get("max_bearing_drift_deg", 20.0)):
            reasons.append("REGION_BEARING_UNSTABLE")

    if len(track.frontier_cell_count_history) >= 2:
        prev = track.frontier_cell_count_history[0]
        cur = track.frontier_cell_count_history[-1]
        if prev > 0:
            m.cell_count_change_ratio = abs(cur - prev) / prev
            if m.cell_count_change_ratio > float(scfg.get("max_cell_count_change_ratio", 0.40)):
                reasons.append("REGION_SIZE_UNSTABLE")

    m.stability_rejection_reasons = reasons
    m.stable = len(reasons) == 0
    return m


def apply_near_region_guard(
    region: Any,
    metrics: GuardedRegionMetrics,
    best_unknown_gain: int,
    cfg: Dict[str, Any],
) -> GuardedRegionMetrics:
    ncfg = _near_cfg(cfg)
    dist = region.distance_to_robot_m
    hard = float(ncfg.get("hard_reject_distance_m", 0.60))
    soft_end = float(ncfg.get("soft_penalty_end_distance_m", 1.00))
    soft_max = float(ncfg.get("soft_penalty_max", 0.45))

    if dist < hard:
        metrics.snapshot_eligible = False
        metrics.stability_rejection_reasons.append("REGION_TOO_CLOSE_HARD")
        metrics.near_robot_penalty = soft_max
        return metrics

    if dist < soft_end:
        span = max(soft_end - hard, 1e-6)
        metrics.near_robot_penalty = soft_max * (soft_end - dist) / span
        persist_ok = metrics.persistence_cycles >= int(ncfg.get("allow_near_region_if_persistent_cycles", 5))
        gain_ok = (
            best_unknown_gain > 0
            and region.unknown_gain_cells >= best_unknown_gain * float(ncfg.get("allow_near_region_if_unknown_gain_ratio_over_best", 0.85))
        )
        if not (persist_ok or gain_ok):
            metrics.snapshot_eligible = False
            metrics.stability_rejection_reasons.append("REGION_NEAR_ROBOT_TRANSIENT")
    return metrics


def apply_trajectory_metrics(
    region: Any,
    metrics: GuardedRegionMetrics,
    trajectory_session: Optional[TrajectorySession],
    observation_poses: Sequence[Dict[str, Any]],
    cfg: Dict[str, Any],
    now_s: float,
) -> GuardedRegionMetrics:
    tcfg = cfg.get("trajectory", {})
    if not bool(tcfg.get("enabled", True)):
        metrics.trajectory_novelty_score = 1.0
        metrics.trajectory_revisit_penalty = 0.0
        return metrics

    vertices: Sequence[TrajectoryVertex] = (
        trajectory_session.vertices if trajectory_session is not None else []
    )
    trm = compute_trajectory_region_metrics(
        region.centroid_x,
        region.centroid_y,
        vertices,
        observation_poses,
        cfg,
        now_s,
    )
    metrics.nearest_trajectory_distance_m = trm.nearest_trajectory_distance_m
    metrics.nearby_trajectory_vertex_count = trm.nearby_trajectory_vertex_count
    metrics.nearby_recent_trajectory_count = trm.nearby_recent_trajectory_count
    metrics.last_nearby_visit_age_s = trm.last_nearby_visit_age_s
    metrics.trajectory_density_score = trm.trajectory_density_score
    metrics.trajectory_novelty_score = trm.trajectory_novelty_score
    metrics.trajectory_revisit_penalty = trm.trajectory_revisit_penalty
    if trm.distance_to_nearest_observation_pose_m < metrics.distance_to_nearest_observation_pose_m:
        metrics.distance_to_nearest_observation_pose_m = trm.distance_to_nearest_observation_pose_m

    if bool(tcfg.get("hard_reject_heavily_revisited", False)):
        heavily = (
            metrics.trajectory_revisit_penalty >= 0.85
            and metrics.trajectory_density_score >= 0.75
            and metrics.nearby_recent_trajectory_count >= 3
            and metrics.nearest_trajectory_distance_m
            <= float(tcfg.get("strong_revisit_distance_m", 0.45))
        )
        if heavily:
            metrics.snapshot_eligible = False
            metrics.stability_rejection_reasons.append("REGION_HEAVILY_REVISITED")
    return metrics


def compute_geometric_score(
    region: Any,
    metrics: GuardedRegionMetrics,
    track: RegionTrack,
    cfg: Dict[str, Any],
    *,
    max_unknown_gain: float,
    max_frontier_cells: float,
    max_clearance: float,
) -> GuardedRegionMetrics:
    gcfg = _geo_cfg(cfg)
    if not bool(gcfg.get("enabled", True)):
        metrics.geo_score_before_trajectory = 0.5
        metrics.geo_score_after_trajectory = 0.5
        metrics.geo_score = 0.5
        return metrics

    weights = gcfg.get("weights", {})
    penalties = gcfg.get("penalties", {})
    pref_d = float(gcfg.get("preferred_distance_m", 1.50))
    pref_tol = float(gcfg.get("preferred_distance_tolerance_m", 1.00))

    ug = _clamp(region.unknown_gain_cells / max(max_unknown_gain, 1.0))
    cl = _clamp(region.minimum_clearance_m / max(max_clearance, 0.01))
    fs = _clamp(region.frontier_cell_count / max(max_frontier_cells, 1.0))
    dist_diff = abs(region.distance_to_robot_m - pref_d)
    dp = _clamp(1.0 - dist_diff / max(pref_tol, 0.01))
    pers = _clamp(metrics.persistence_cycles / 5.0)
    completeness = _clamp(min(1.0, region.unknown_gain_ratio if region.unknown_gain_ratio > 0 else ug))
    novelty = _clamp(metrics.trajectory_novelty_score)

    metrics.score_components = {
        "unknown_gain": round(ug, 4),
        "clearance": round(cl, 4),
        "frontier_size": round(fs, 4),
        "distance_preference": round(dp, 4),
        "persistence": round(pers, 4),
        "region_completeness": round(completeness, 4),
        "trajectory_novelty": round(novelty, 4),
    }

    sel_p = _clamp(track.selection_count * float(cfg.get("region_history", {}).get("selection_penalty_per_count", 0.08)))
    visit_p = _clamp(track.visit_count * float(cfg.get("region_history", {}).get("visit_penalty_per_count", 0.15)))
    nav_p = _clamp(track.navigation_failure_count * float(cfg.get("region_history", {}).get("navigation_failure_penalty_per_count", 0.25)))
    revisit_p = _clamp(metrics.trajectory_revisit_penalty)

    metrics.penalty_components = {
        "near_robot": round(metrics.near_robot_penalty, 4),
        "recent_observation": round(metrics.recent_observation_penalty, 4),
        "selection_count": round(sel_p, 4),
        "visit_count": round(visit_p, 4),
        "navigation_failure": round(nav_p, 4),
        "trajectory_revisit": round(revisit_p, 4),
    }

    pos_base = (
        weights.get("unknown_gain", 0.22) * ug
        + weights.get("clearance", 0.18) * cl
        + weights.get("frontier_size", 0.12) * fs
        + weights.get("distance_preference", 0.13) * dp
        + weights.get("persistence", 0.12) * pers
        + weights.get("region_completeness", 0.08) * completeness
    )
    pen_base = (
        penalties.get("near_robot", 0.15) * metrics.near_robot_penalty
        + penalties.get("recent_observation", 0.20) * metrics.recent_observation_penalty
        + penalties.get("selection_count", 0.15) * sel_p
        + penalties.get("visit_count", 0.15) * visit_p
        + penalties.get("navigation_failure", 0.20) * nav_p
    )
    metrics.geo_score_before_trajectory = _clamp(pos_base - pen_base)

    pos = pos_base + weights.get("trajectory_novelty", 0.15) * novelty
    pen = pen_base + penalties.get("trajectory_revisit", 0.15) * revisit_p
    metrics.geo_score = _clamp(pos - pen)
    metrics.geo_score_after_trajectory = metrics.geo_score
    metrics.score_explanation = (
        f"pos={pos:.3f} pen={pen:.3f} "
        f"before_traj={metrics.geo_score_before_trajectory:.3f}"
    )
    return metrics


def rank_geometric_candidates(
    items: List[Tuple[Any, GuardedRegionMetrics]],
    cfg: Dict[str, Any],
) -> List[Tuple[Any, GuardedRegionMetrics]]:
    gcfg = _geo_cfg(cfg)
    min_score = float(gcfg.get("minimum_eligible_score", 0.40))
    ranked = sorted(items, key=lambda x: (-x[1].geo_score, x[0].region_id))
    for rank, (region, m) in enumerate(ranked, start=1):
        m.geo_rank = rank
        if m.stable and m.snapshot_eligible and m.geo_score >= min_score:
            pass  # eligible stays
        elif m.snapshot_eligible:
            m.snapshot_eligible = False
            if m.geo_score < min_score:
                m.stability_rejection_reasons.append("GEO_SCORE_TOO_LOW")
    return ranked


def apply_history_penalties(
    region: Any,
    track: RegionTrack,
    metrics: GuardedRegionMetrics,
    observation_poses: Sequence[Dict[str, Any]],
    cfg: Dict[str, Any],
) -> GuardedRegionMetrics:
    hcfg = cfg.get("region_history", {})
    if not bool(hcfg.get("enabled", True)):
        return metrics

    if track.blacklisted or track.navigation_failure_count >= int(hcfg.get("blacklist_after_navigation_failures", 3)):
        track.blacklisted = True
        metrics.snapshot_eligible = False
        metrics.stability_rejection_reasons.append("REGION_BLACKLISTED")
        return metrics

    hard_r = float(hcfg.get("hard_reject_recent_pose_radius_m", 0.60))
    strong_r = float(hcfg.get("strong_penalty_radius_m", 0.90))
    weak_r = float(hcfg.get("weak_penalty_radius_m", 1.20))
    pen_max = float(hcfg.get("recent_pose_penalty_max", 0.50))

    min_dist = float("inf")
    cx, cy = region.centroid_x, region.centroid_y
    for pose in observation_poses:
        d = math.hypot(cx - float(pose.get("x", 0)), cy - float(pose.get("y", 0)))
        min_dist = min(min_dist, d)
    metrics.distance_to_nearest_observation_pose_m = min_dist

    if min_dist < hard_r:
        metrics.snapshot_eligible = False
        metrics.stability_rejection_reasons.append("REGION_NEAR_RECENT_OBSERVATION")
        metrics.recent_observation_penalty = pen_max
    elif min_dist < strong_r:
        metrics.recent_observation_penalty = pen_max * (strong_r - min_dist) / max(strong_r - hard_r, 1e-6)
    elif min_dist < weak_r:
        metrics.recent_observation_penalty = pen_max * 0.5 * (weak_r - min_dist) / max(weak_r - strong_r, 1e-6)

    return metrics


def guard_regions_for_cycle(
    regions: Sequence[Any],
    tracks: Dict[str, RegionTrack],
    cycle_id: int,
    now_s: float,
    cfg: Dict[str, Any],
    observation_poses: Sequence[Dict[str, Any]],
    trajectory_session: Optional[TrajectorySession] = None,
) -> Tuple[Dict[str, RegionTrack], List[Tuple[Any, GuardedRegionMetrics, RegionTrack]]]:
    """Full guard pipeline for accepted regions."""
    tracks, mapping, _ = match_regions_to_tracks(list(regions), tracks, cycle_id, now_s, cfg)
    best_gain = max((r.unknown_gain_cells for r in regions), default=0)
    max_cells = max((r.frontier_cell_count for r in regions), default=1)
    max_clear = max((r.minimum_clearance_m for r in regions), default=0.01)

    out: List[Tuple[Any, GuardedRegionMetrics, RegionTrack]] = []
    for idx, region in enumerate(regions):
        tid = mapping[idx]
        track = tracks[tid]
        m = apply_stability_gate(region, track, now_s, cfg)
        m.snapshot_eligible = region.accepted and m.stable
        m = apply_near_region_guard(region, m, best_gain, cfg)
        m = apply_history_penalties(region, track, m, observation_poses, cfg)
        m = apply_trajectory_metrics(
            region, m, trajectory_session, observation_poses, cfg, now_s
        )
        m = compute_geometric_score(
            region,
            m,
            track,
            cfg,
            max_unknown_gain=best_gain,
            max_frontier_cells=max_cells,
            max_clearance=max_clear,
        )
        out.append((region, m, track))

    ranked = rank_geometric_candidates([(r, m) for r, m, _ in out], cfg)
    rank_map = {id(r): m for r, m in ranked}
    final: List[Tuple[Any, GuardedRegionMetrics, RegionTrack]] = []
    for region, m, track in out:
        m.geo_rank = rank_map[id(region)].geo_rank
        m.geo_score = rank_map[id(region)].geo_score
        m.snapshot_eligible = rank_map[id(region)].snapshot_eligible
        final.append((region, m, track))
    return tracks, final


def accumulate_yaw_delta(prev_yaw_rad: float, curr_yaw_rad: float) -> float:
    """Signed smallest delta in radians."""
    dy = curr_yaw_rad - prev_yaw_rad
    while dy > math.pi:
        dy -= 2 * math.pi
    while dy < -math.pi:
        dy += 2 * math.pi
    return dy


def track_to_dict(t: RegionTrack) -> Dict[str, Any]:
    return asdict(t)


@dataclass
class ObservationWindow:
    state: str = "IDLE"
    window_id: str = ""
    start_x: float = 0.0
    start_y: float = 0.0
    start_yaw_rad: float = 0.0
    last_yaw_rad: float = 0.0
    accumulated_rotation_deg: float = 0.0
    translation_during_scan_m: float = 0.0
    settle_start_s: Optional[float] = None
    settle_elapsed_s: float = 0.0
    map_stable_cycle_count: int = 0
    last_frontier_count: int = 0
    last_accepted_count: int = 0
    last_known_cells: int = 0
    started_at_s: float = 0.0

    def start(self, window_id: str, robot: Any, now_s: float) -> None:
        self.state = "OBSERVING"
        self.window_id = window_id
        self.start_x = robot.x
        self.start_y = robot.y
        self.start_yaw_rad = robot.yaw_rad
        self.last_yaw_rad = robot.yaw_rad
        self.accumulated_rotation_deg = 0.0
        self.translation_during_scan_m = 0.0
        self.settle_start_s = None
        self.settle_elapsed_s = 0.0
        self.map_stable_cycle_count = 0
        self.started_at_s = now_s

    def update(
        self,
        robot: Any,
        now_s: float,
        result: Any,
        map_health: Any,
        cfg: Dict[str, Any],
    ) -> None:
        ocfg = cfg.get("observation_gate", {})
        if self.state in ("IDLE", "FAILED"):
            return
        if now_s - self.started_at_s > float(ocfg.get("observation_timeout_s", 120.0)):
            self.state = "FAILED"
            return

        dy = accumulate_yaw_delta(self.last_yaw_rad, robot.yaw_rad)
        self.accumulated_rotation_deg += abs(math.degrees(dy))
        self.last_yaw_rad = robot.yaw_rad
        self.translation_during_scan_m = max(
            self.translation_during_scan_m,
            math.hypot(robot.x - self.start_x, robot.y - self.start_y),
        )

        threshold = float(ocfg.get("full_scan_threshold_deg", 350.0))
        max_trans = float(ocfg.get("max_translation_during_scan_m", 0.20))

        if self.state == "OBSERVING":
            if self.translation_during_scan_m > max_trans:
                self.state = "FAILED"
                return
            if self.accumulated_rotation_deg >= threshold:
                self.state = "FULL_SCAN_COMPLETE"
                self.settle_start_s = now_s

        if self.state in ("FULL_SCAN_COMPLETE", "SETTLING"):
            if self.settle_start_s is None:
                self.settle_start_s = now_s
            self.settle_elapsed_s = now_s - self.settle_start_s
            self.state = "SETTLING"
            if self.settle_elapsed_s >= float(ocfg.get("settle_time_s", 1.5)):
                trans = math.hypot(robot.x - self.start_x, robot.y - self.start_y)
                if trans <= float(ocfg.get("settle_max_translation_m", 0.05)):
                    self.state = "MAP_STABILIZING"

        if self.state == "MAP_STABILIZING":
            fc = result.stats.remaining_frontier_cells
            ac = result.stats.accepted_region_count
            known = map_health.free_cells + map_health.occupied_cells
            stable = True
            if self.last_frontier_count > 0:
                ratio = abs(fc - self.last_frontier_count) / self.last_frontier_count
                if ratio > float(ocfg.get("max_frontier_count_change_ratio", 0.15)):
                    stable = False
            if abs(ac - self.last_accepted_count) > int(ocfg.get("max_accepted_region_count_change", 1)):
                stable = False
            if self.last_known_cells > 0:
                cr = abs(known - self.last_known_cells) / self.last_known_cells
                if cr > float(ocfg.get("max_map_cell_change_ratio", 0.05)):
                    stable = False
            if stable:
                self.map_stable_cycle_count += 1
            else:
                self.map_stable_cycle_count = 0
            self.last_frontier_count = fc
            self.last_accepted_count = ac
            self.last_known_cells = known
            if self.map_stable_cycle_count >= int(ocfg.get("map_stable_cycles", 3)):
                self.state = "READY_FOR_SNAPSHOT"

    def snapshot_gate_ready(self, stable_eligible_count: int) -> bool:
        return self.state == "READY_FOR_SNAPSHOT" and stable_eligible_count > 0

    def snapshot_block_reason(self, stable_eligible_count: int, require_window: bool) -> Optional[str]:
        if not require_window:
            return None if stable_eligible_count > 0 else "SNAPSHOT_NO_STABLE_REGION"
        if self.state == "IDLE":
            return "SNAPSHOT_OBSERVATION_NOT_STARTED"
        if self.state == "FAILED":
            return "SNAPSHOT_SCAN_TRANSLATION_TOO_LARGE"
        if self.state == "OBSERVING":
            return "SNAPSHOT_FULL_SCAN_NOT_COMPLETE"
        if self.state in ("FULL_SCAN_COMPLETE", "SETTLING"):
            return "SNAPSHOT_ROBOT_NOT_SETTLED"
        if self.state == "MAP_STABILIZING":
            return "SNAPSHOT_MAP_NOT_STABLE"
        if stable_eligible_count <= 0:
            return "SNAPSHOT_NO_STABLE_REGION"
        return None

