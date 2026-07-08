#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EXP3 STRICT v3 exploration goal selector, based on exp2-compatible interfaces.

This version keeps exp2/Foxglove-compatible fields and adds:
1) count-based sector selection with hysteresis, so active sector does not flicker;
2) goal latch, so an already issued goal is not changed every planner tick;
3) corridor reachability check using LaserScan before accepting far goals;
4) selected goal is always inside the active sector, with exp2 marker fields preserved.

The file is dependency-light and can replace/alias older exp2 selector call sites.
"""
from __future__ import annotations

import argparse
import copy
import dataclasses
import json
import math
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

EPS = 1e-9


def _now() -> float:
    return time.monotonic()


def _clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        if v is None:
            return default
        f = float(v)
        if math.isnan(f) or math.isinf(f):
            return default
        return f
    except Exception:
        return default


def _safe_bool(v: Any, default: bool = True) -> bool:
    if v is None:
        return default
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        return v.strip().lower() not in ("0", "false", "no", "off", "none")
    return bool(v)


def _get(obj: Any, key: str, default: Any = None) -> Any:
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _set(obj: Any, key: str, value: Any) -> None:
    try:
        if isinstance(obj, dict):
            obj[key] = value
        else:
            setattr(obj, key, value)
    except Exception:
        pass


def _pose_xyyaw(pose: Any) -> Tuple[float, float, float]:
    if pose is None:
        return 0.0, 0.0, 0.0
    if isinstance(pose, (list, tuple)):
        if len(pose) >= 3:
            return _safe_float(pose[0]), _safe_float(pose[1]), _safe_float(pose[2])
        if len(pose) >= 2:
            return _safe_float(pose[0]), _safe_float(pose[1]), 0.0
    x = _safe_float(_get(pose, "x", _get(pose, "px", _get(pose, "position_x", 0.0))))
    y = _safe_float(_get(pose, "y", _get(pose, "py", _get(pose, "position_y", 0.0))))
    yaw = _safe_float(_get(pose, "yaw", _get(pose, "theta", _get(pose, "heading", 0.0))))
    return x, y, yaw


@dataclass
class Exp3Config:
    enabled: bool = True

    # Sector policy. sector_00 is robot forward, sector_01 is left/front, ids increase CCW.
    sector_count: int = 8
    sector_choose_by_count: bool = True
    sector_count_distance_min_m: float = 0.70
    active_sector_min_candidates: int = 1
    active_sector_keep_seconds: float = 10.0
    active_sector_max_seconds: float = 45.0
    sector_switch_min_count_gain: int = 4
    keep_active_if_tied: bool = True
    force_goal_in_active_sector: bool = True

    # How many candidates must exist before accepting a sparse upstream set.
    min_candidates_total: int = 45
    min_candidates_per_active_sector: int = 2

    # Cone recovery is only used for marker/debug compatibility. Exact sector filtering is primary.
    active_cone_expand_deg: List[float] = field(default_factory=lambda: [22.5, 45.0, 67.5, 90.0, 135.0, 180.0])

    # LaserScan candidate generation.
    lidar_candidate_enable: bool = True
    lidar_angle_step_deg: float = 4.0
    lidar_max_candidates: int = 240
    lidar_min_range_m: float = 0.45
    lidar_max_range_m: float = 3.8
    lidar_obstacle_margin_m: float = 0.52
    lidar_unknown_ray_as_frontier: bool = False
    lidar_yaw_offset_deg: float = 0.0
    recovery_projection_radius_schedule_m: List[float] = field(default_factory=lambda: [0.9, 1.2, 1.6, 2.0, 2.5, 3.0])

    # Scoring.
    min_candidate_score: float = 0.12
    recovery_relax_score_schedule: List[float] = field(default_factory=lambda: [1.0, 0.6, 0.25, 0.0])
    near_reject_distance_m: float = 0.62
    preferred_min_distance_m: float = 1.10
    preferred_max_distance_m: float = 2.80
    max_goal_distance_m: float = 3.20
    min_unknown_gain: float = 0.0
    require_reachable: bool = True
    w_distance: float = 0.45
    w_unknown_gain: float = 0.18
    w_raw_score: float = 0.16
    w_sector_density: float = 0.16
    w_forward: float = 0.03
    w_hysteresis: float = 0.02
    near_penalty: float = 0.40
    visited_penalty: float = 0.45
    blacklist_penalty: float = 0.80

    # Direction preference. Kept deliberately weak: count wins, direction only breaks ties.
    prefer_initial_direction: bool = True
    initial_direction_hold_s: float = 55.0
    initial_direction_yaw_deg: Optional[float] = None
    initial_direction_tie_bonus: float = 0.25

    # Arrival behavior.
    observe_spin_on_arrival: bool = True
    observe_spin_duration_s: float = 5.8
    observe_spin_wz: float = 0.24
    observe_stop_after_spin_s: float = 0.35
    visited_cooldown_s: float = 90.0
    visited_radius_m: float = 0.65
    blacklist_radius_m: float = 0.55
    blacklist_ttl_s: float = 120.0

    # Stability: keep one issued goal until reached/blocked/timeout instead of reselecting every tick.
    goal_latch_enabled: bool = True
    goal_latch_min_seconds: float = 8.0
    goal_latch_max_seconds: float = 28.0
    goal_reached_radius_m: float = 0.38
    goal_switch_min_score_gain: float = 0.22

    # Reachability: cheap local corridor check from LaserScan. It is not global A*, but prevents
    # selecting far points behind nearby obstacles or narrow wall edges.
    scan_corridor_enable: bool = True
    scan_corridor_half_width_deg: float = 9.0
    scan_corridor_min_clear_fraction: float = 0.68
    scan_corridor_min_beams: int = 3
    scan_corridor_safety_margin_m: float = 0.22
    scan_corridor_unknown_is_clear: bool = False

    debug: bool = True

    @staticmethod
    def from_dict(d: Optional[Dict[str, Any]]) -> "Exp3Config":
        cfg = Exp3Config()
        if not d:
            return cfg
        flat: Dict[str, Any] = {}
        if isinstance(d, dict):
            for k in ("semantic_explore_exp3", "exp3", "explore_goal_selector", "direction_lock", "direction"):
                v = d.get(k)
                if isinstance(v, dict):
                    flat.update(v)
            flat.update({k: v for k, v in d.items() if not isinstance(v, dict) or hasattr(cfg, k)})
        for f in dataclasses.fields(cfg):
            if f.name not in flat:
                continue
            old = getattr(cfg, f.name)
            val = flat[f.name]
            try:
                if isinstance(old, bool):
                    setattr(cfg, f.name, _safe_bool(val, old))
                elif isinstance(old, int) and not isinstance(old, bool):
                    setattr(cfg, f.name, int(val))
                elif isinstance(old, float):
                    setattr(cfg, f.name, float(val))
                elif isinstance(old, list):
                    setattr(cfg, f.name, list(val) if isinstance(val, (list, tuple)) else old)
                else:
                    setattr(cfg, f.name, val)
            except Exception:
                setattr(cfg, f.name, old)
        cfg.sector_count = max(4, int(cfg.sector_count))
        cfg.active_cone_expand_deg = sorted(set(float(x) for x in cfg.active_cone_expand_deg))
        cfg.recovery_projection_radius_schedule_m = sorted(set(float(x) for x in cfg.recovery_projection_radius_schedule_m))
        cfg.recovery_relax_score_schedule = sorted(set(float(x) for x in cfg.recovery_relax_score_schedule), reverse=True)
        return cfg


@dataclass
class CandidateView:
    raw: Any
    x: float
    y: float
    raw_score: float = 0.0
    unknown_gain: float = 0.0
    reachable: bool = True
    source: str = "upstream"
    distance: float = 0.0
    bearing: float = 0.0
    sector_id: int = 0
    sector_center: float = 0.0
    sector_count: int = 0
    final_score: float = 0.0
    reason: str = ""

    def key(self) -> Tuple[int, int]:
        return int(round(self.x * 10.0)), int(round(self.y * 10.0))


class ExploreGoalSelectorExp3:
    def __init__(self, config: Optional[Dict[str, Any]] = None, logger: Any = None, **kwargs: Any) -> None:
        merged: Dict[str, Any] = {}
        if config:
            merged.update(config)
        merged.update(kwargs)
        self.cfg = Exp3Config.from_dict(merged)
        self.logger = logger
        self.active_sector_id: Optional[int] = None
        self.active_sector_since = _now()
        self.initial_yaw: Optional[float] = None
        self.task_start_time = _now()
        self.last_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_goal_key: Optional[Tuple[int, int]] = None
        self.latched_goal_raw: Any = None
        self.latched_goal_since: float = 0.0
        self.latched_goal_sector_id: Optional[int] = None
        self.visited: Deque[Tuple[float, float, float]] = deque(maxlen=160)
        self.blacklist: Deque[Tuple[float, float, float]] = deque(maxlen=160)
        self.observe_until = 0.0
        self.stop_until = 0.0
        self.last_debug: Dict[str, Any] = {}

    # ---------------- basic helpers ----------------
    def _log(self, msg: str) -> None:
        if not self.cfg.debug:
            return
        try:
            if self.logger is not None and hasattr(self.logger, "info"):
                self.logger.info(msg)
            elif self.logger is not None:
                self.logger(str(msg))
        except Exception:
            pass

    def _sector_id(self, bearing: float) -> int:
        """sector_00 centered at forward bearing 0; ids increase CCW."""
        width = 2.0 * math.pi / float(self.cfg.sector_count)
        return int(math.floor((_wrap_pi(bearing) + width / 2.0) / width)) % self.cfg.sector_count

    def _sector_center(self, sid: int) -> float:
        width = 2.0 * math.pi / float(self.cfg.sector_count)
        return _wrap_pi(float(sid % self.cfg.sector_count) * width)

    def get_active_sector_id(self) -> Optional[int]:
        return self.active_sector_id

    def get_active_sector_center(self) -> float:
        return self._sector_center(self.active_sector_id or 0)

    def get_debug_state(self) -> Dict[str, Any]:
        return copy.deepcopy(self.last_debug)

    def get_sector_boundaries(self, pose: Any = None, radius: float = 3.5) -> List[Dict[str, float]]:
        """Pure-data helper for Foxglove marker code if the old visualizer wants exact boundaries."""
        pose = self.last_pose if pose is None else pose
        rx, ry, ryaw = _pose_xyyaw(pose)
        width = 2.0 * math.pi / float(self.cfg.sector_count)
        out: List[Dict[str, float]] = []
        for sid in range(self.cfg.sector_count):
            center = self._sector_center(sid)
            for side, a_rel in (("left", center + width / 2.0), ("right", center - width / 2.0)):
                a = ryaw + a_rel
                out.append({
                    "sector_id": sid,
                    "side": 1 if side == "left" else -1,
                    "x0": rx,
                    "y0": ry,
                    "x1": rx + radius * math.cos(a),
                    "y1": ry + radius * math.sin(a),
                    "angle_deg": math.degrees(_wrap_pi(a_rel)),
                })
        return out

    # ---------------- candidate normalization ----------------
    def _candidate_xy(self, c: Any) -> Optional[Tuple[float, float]]:
        x = _get(c, "x", None)
        y = _get(c, "y", None)
        if x is not None and y is not None:
            return _safe_float(x), _safe_float(y)
        pos = _get(c, "position", None)
        if pos is not None:
            x = _get(pos, "x", None)
            y = _get(pos, "y", None)
            if x is not None and y is not None:
                return _safe_float(x), _safe_float(y)
        pose = _get(c, "pose", None)
        if pose is not None:
            got = self._candidate_xy(pose)
            if got is not None:
                return got
            pp = _get(pose, "pose", None)
            if pp is not None:
                got = self._candidate_xy(pp)
                if got is not None:
                    return got
        if isinstance(c, (list, tuple)) and len(c) >= 2:
            return _safe_float(c[0]), _safe_float(c[1])
        return None

    def _normalize_one(self, c: Any, pose: Any) -> Optional[CandidateView]:
        xy = self._candidate_xy(c)
        if xy is None:
            return None
        rx, ry, ryaw = _pose_xyyaw(pose)
        x, y = xy
        dx, dy = x - rx, y - ry
        dist = math.hypot(dx, dy)
        bearing = _wrap_pi(math.atan2(dy, dx) - ryaw)
        sid = self._sector_id(bearing)
        raw_score = _safe_float(_get(c, "score", _get(c, "frontier_score", _get(c, "candidate_score", 0.0))), 0.0)
        unknown_gain = _safe_float(_get(c, "unknown_gain", _get(c, "info_gain", _get(c, "gain", 0.0))), 0.0)
        reachable = _safe_bool(_get(c, "reachable", _get(c, "is_reachable", True)), True)
        source = str(_get(c, "source", "upstream"))
        return CandidateView(raw=c, x=x, y=y, raw_score=raw_score, unknown_gain=unknown_gain,
                             reachable=reachable, source=source, distance=dist, bearing=bearing,
                             sector_id=sid, sector_center=self._sector_center(sid))

    def normalize_candidates(self, candidates: Optional[Iterable[Any]], pose: Any = None) -> List[CandidateView]:
        pose = self.last_pose if pose is None else pose
        out: List[CandidateView] = []
        if not candidates:
            return out
        best_by_key: Dict[Tuple[int, int], CandidateView] = {}
        for c in candidates:
            cv = self._normalize_one(c, pose)
            if cv is None:
                continue
            k = cv.key()
            prev = best_by_key.get(k)
            if prev is None or (cv.raw_score + cv.unknown_gain + cv.distance * 0.01) > (prev.raw_score + prev.unknown_gain + prev.distance * 0.01):
                best_by_key[k] = cv
        out = list(best_by_key.values())
        return out

    # ---------------- lidar generation ----------------
    def generate_lidar_candidates(self, scan: Any, pose: Any = None, sector_id: Optional[int] = None) -> List[Dict[str, Any]]:
        if scan is None or not self.cfg.lidar_candidate_enable:
            return []
        pose = self.last_pose if pose is None else pose
        rx, ry, ryaw = _pose_xyyaw(pose)
        ranges = list(_get(scan, "ranges", []) or [])
        if not ranges:
            return []
        angle_min = _safe_float(_get(scan, "angle_min", -math.pi), -math.pi)
        angle_increment = _safe_float(_get(scan, "angle_increment", 2 * math.pi / max(1, len(ranges))), 2 * math.pi / max(1, len(ranges)))
        range_max = _safe_float(_get(scan, "range_max", self.cfg.lidar_max_range_m), self.cfg.lidar_max_range_m)
        range_min = _safe_float(_get(scan, "range_min", self.cfg.lidar_min_range_m), self.cfg.lidar_min_range_m)
        max_range = min(self.cfg.lidar_max_range_m, range_max if range_max > 0 else self.cfg.lidar_max_range_m)
        min_range = max(self.cfg.lidar_min_range_m, range_min if range_min > 0 else self.cfg.lidar_min_range_m)
        step = max(1, int(round(math.radians(max(1.0, self.cfg.lidar_angle_step_deg)) / max(abs(angle_increment), EPS))))
        yaw_offset = math.radians(self.cfg.lidar_yaw_offset_deg)
        out: List[Dict[str, Any]] = []
        for i in range(0, len(ranges), step):
            r = _safe_float(ranges[i], float("nan"))
            unknown = math.isnan(r) or math.isinf(r) or r <= 0.0
            if unknown:
                if not self.cfg.lidar_unknown_ray_as_frontier:
                    continue
                usable = max_range
                gain_base = 0.56
            else:
                usable = min(max_range, r - self.cfg.lidar_obstacle_margin_m)
                gain_base = 0.42 if r >= max_range * 0.80 else 0.22
            if usable < min_range:
                continue
            a_robot = _wrap_pi(angle_min + i * angle_increment + yaw_offset)
            sid = self._sector_id(a_robot)
            if sector_id is not None and sid != sector_id:
                continue
            world_a = ryaw + a_robot
            for rr in self.cfg.recovery_projection_radius_schedule_m:
                if rr < min_range or rr > usable:
                    continue
                x = rx + rr * math.cos(world_a)
                y = ry + rr * math.sin(world_a)
                score = _clip(0.18 + 0.68 * rr / max(self.cfg.preferred_max_distance_m, EPS), 0.0, 1.0)
                out.append({
                    "x": float(x),
                    "y": float(y),
                    "score": float(score),
                    "unknown_gain": float(_clip(gain_base + 0.18 * rr / max(self.cfg.preferred_max_distance_m, EPS), 0.0, 1.0)),
                    "reachable": True,
                    "source": "exp3_lidar_ray",
                    "ray_index": int(i),
                    "ray_range": None if unknown else float(r),
                    "projection_radius": float(rr),
                    "sector_id": int(sid),
                })
                if len(out) >= self.cfg.lidar_max_candidates:
                    return out
        return out

    def _scan_corridor_ok(self, scan: Any, bearing: float, distance: float) -> Tuple[bool, Dict[str, Any]]:
        if scan is None or not self.cfg.scan_corridor_enable:
            return True, {"checked": False, "reason": "no_scan_or_disabled"}
        ranges = list(_get(scan, "ranges", []) or [])
        if not ranges:
            return True, {"checked": False, "reason": "empty_scan"}
        angle_min = _safe_float(_get(scan, "angle_min", -math.pi), -math.pi)
        angle_increment = _safe_float(_get(scan, "angle_increment", 2 * math.pi / max(1, len(ranges))), 2 * math.pi / max(1, len(ranges)))
        range_max = _safe_float(_get(scan, "range_max", self.cfg.lidar_max_range_m), self.cfg.lidar_max_range_m)
        yaw_offset = math.radians(self.cfg.lidar_yaw_offset_deg)
        half = math.radians(max(1.0, self.cfg.scan_corridor_half_width_deg))
        need = max(0.0, float(distance) + float(self.cfg.scan_corridor_safety_margin_m))
        need = min(need, max(range_max, self.cfg.lidar_max_range_m))
        total = 0
        clear = 0
        blocked_min = float("inf")
        for i, r0 in enumerate(ranges):
            a_robot = _wrap_pi(angle_min + i * angle_increment + yaw_offset)
            if abs(_wrap_pi(a_robot - bearing)) > half:
                continue
            total += 1
            r = _safe_float(r0, float("nan"))
            unknown = math.isnan(r) or math.isinf(r) or r <= 0.0
            if unknown:
                if self.cfg.scan_corridor_unknown_is_clear:
                    clear += 1
                continue
            if r >= need:
                clear += 1
            else:
                blocked_min = min(blocked_min, r)
        if total < int(self.cfg.scan_corridor_min_beams):
            return True, {"checked": True, "reason": "too_few_beams", "beams": total, "need": round(need, 3)}
        frac = clear / max(total, 1)
        ok = frac >= float(self.cfg.scan_corridor_min_clear_fraction)
        return ok, {
            "checked": True,
            "beams": int(total),
            "clear": int(clear),
            "clear_fraction": round(frac, 3),
            "need": round(need, 3),
            "blocked_min": None if blocked_min == float("inf") else round(blocked_min, 3),
            "reason": "ok" if ok else "blocked_corridor",
        }

    def _apply_scan_reachability(self, cvs: List[CandidateView], scan: Any) -> Dict[str, Any]:
        checked = 0
        blocked = 0
        for cv in cvs:
            ok, info = self._scan_corridor_ok(scan, cv.bearing, cv.distance)
            if info.get("checked"):
                checked += 1
            if not ok:
                cv.reachable = False
                cv.reason += " corridor_blocked"
                blocked += 1
            else:
                cv.reason += " corridor_ok" if info.get("checked") else ""
        return {"reachability_checked": checked, "reachability_blocked": blocked}

    # ---------------- filters / scoring ----------------
    def _fresh_blacklist(self, now: float) -> List[Tuple[float, float, float]]:
        return [(x, y, t) for x, y, t in self.blacklist if now - t <= self.cfg.blacklist_ttl_s]

    def _near_records(self, cv: CandidateView, records: Iterable[Tuple[float, float, float]], radius: float) -> bool:
        for x, y, _t in records:
            if math.hypot(cv.x - x, cv.y - y) <= radius:
                return True
        return False

    def _hard_filter(self, cvs: List[CandidateView], now: float) -> List[CandidateView]:
        out: List[CandidateView] = []
        for cv in cvs:
            if self.cfg.require_reachable and not cv.reachable:
                continue
            if cv.distance > self.cfg.max_goal_distance_m:
                continue
            if cv.unknown_gain < self.cfg.min_unknown_gain:
                continue
            out.append(cv)
        return out

    def _sector_counts(self, cvs: List[CandidateView]) -> Dict[int, int]:
        counts: Dict[int, int] = defaultdict(int)
        for cv in cvs:
            if cv.distance >= self.cfg.sector_count_distance_min_m:
                counts[cv.sector_id] += 1
        if not counts:
            for cv in cvs:
                counts[cv.sector_id] += 1
        return dict(counts)

    def _initial_direction_sector(self, pose: Any) -> int:
        rx, ry, ryaw = _pose_xyyaw(pose)
        if self.initial_yaw is None:
            self.initial_yaw = ryaw
        if self.cfg.initial_direction_yaw_deg is not None:
            return self._sector_id(math.radians(float(self.cfg.initial_direction_yaw_deg)))
        return self._sector_id(_wrap_pi(self.initial_yaw - ryaw))

    def _choose_active_sector(self, cvs: List[CandidateView], pose: Any, now: float) -> int:
        counts = self._sector_counts(cvs)
        if not counts:
            self.active_sector_id = 0
            self.active_sector_since = now
            return 0

        # Count is primary. Direction preference is only a tie-breaker during the first seconds.
        initial_sid = self._initial_direction_sector(pose)
        best_sid = max(
            counts.keys(),
            key=lambda sid: (
                counts.get(sid, 0),
                self.cfg.initial_direction_tie_bonus if (
                    self.cfg.prefer_initial_direction
                    and now - self.task_start_time <= self.cfg.initial_direction_hold_s
                    and sid == initial_sid
                ) else 0.0,
                max((cv.distance for cv in cvs if cv.sector_id == sid), default=0.0),
                max((cv.raw_score + cv.unknown_gain for cv in cvs if cv.sector_id == sid), default=0.0),
                -sid,
            ),
        )

        if self.active_sector_id is None:
            self.active_sector_id = int(best_sid)
            self.active_sector_since = now
            return int(best_sid)

        active_sid = int(self.active_sector_id)
        active_count = counts.get(active_sid, 0)
        best_count = counts.get(best_sid, 0)
        active_age = now - self.active_sector_since

        # The important fix: do NOT switch just because another sector is slightly larger.
        # Keep current active sector while it still has usable candidates and has not timed out.
        if active_count >= int(self.cfg.active_sector_min_candidates) and active_age <= float(self.cfg.active_sector_max_seconds):
            if active_age <= float(self.cfg.active_sector_keep_seconds):
                return active_sid
            if best_count <= active_count + int(self.cfg.sector_switch_min_count_gain):
                return active_sid

        # Switch only when the current sector is exhausted/timed out or another sector is clearly better.
        if active_sid != int(best_sid):
            self.active_sector_id = int(best_sid)
            self.active_sector_since = now
        return int(self.active_sector_id)

    def _score_candidates(self, cvs: List[CandidateView], counts: Dict[int, int], now: float) -> None:
        max_count = max(counts.values(), default=1)
        fresh_black = self._fresh_blacklist(now)
        fresh_visit = [(x, y, t) for x, y, t in self.visited if now - t <= self.cfg.visited_cooldown_s]
        for cv in cvs:
            dist_norm = _clip((cv.distance - self.cfg.near_reject_distance_m) / max(self.cfg.preferred_max_distance_m - self.cfg.near_reject_distance_m, EPS), 0.0, 1.0)
            if cv.distance > self.cfg.preferred_max_distance_m:
                dist_norm *= max(0.0, 1.0 - 0.22 * (cv.distance - self.cfg.preferred_max_distance_m))
            raw_norm = _clip(cv.raw_score, 0.0, 1.0)
            gain_norm = _clip(cv.unknown_gain, 0.0, 1.0)
            density_norm = _clip(counts.get(cv.sector_id, 0) / max(max_count, 1), 0.0, 1.0)
            forward_norm = _clip(1.0 - abs(cv.bearing) / math.pi, 0.0, 1.0)
            hysteresis = 1.0 if self.last_goal_key is not None and cv.key() == self.last_goal_key else 0.0
            score = (
                self.cfg.w_distance * dist_norm +
                self.cfg.w_unknown_gain * gain_norm +
                self.cfg.w_raw_score * raw_norm +
                self.cfg.w_sector_density * density_norm +
                self.cfg.w_forward * forward_norm +
                self.cfg.w_hysteresis * hysteresis
            )
            if cv.distance < self.cfg.near_reject_distance_m:
                score -= self.cfg.near_penalty
                cv.reason += " near_penalty"
            if self._near_records(cv, fresh_visit, self.cfg.visited_radius_m):
                score -= self.cfg.visited_penalty
                cv.reason += " visited_penalty"
            if self._near_records(cv, fresh_black, self.cfg.blacklist_radius_m):
                score -= self.cfg.blacklist_penalty
                cv.reason += " blacklist_penalty"
            if not cv.reachable:
                score -= 0.25
                cv.reason += " maybe_unreachable"
            cv.sector_count = counts.get(cv.sector_id, 0)
            cv.final_score = float(score)

    def _export(self, cv: Optional[CandidateView], active_sector_id: Optional[int] = None, reason: str = "") -> Any:
        if cv is None:
            return None
        raw = cv.raw
        sid = int(cv.sector_id)
        active_sid = sid if active_sector_id is None else int(active_sector_id)
        label = f"EXP3 s{sid:02d} count={cv.sector_count} d={cv.distance:.2f} score={cv.final_score:.2f} {cv.source}"
        for k, v in {
            "exp3_score": float(cv.final_score),
            "exp3_distance": float(cv.distance),
            "exp3_bearing_deg": float(math.degrees(cv.bearing)),
            "exp3_sector_id": sid,
            "exp3_sector_count": int(cv.sector_count),
            "exp3_active_sector_id": active_sid,
            "exp3_in_active_sector": bool(sid == active_sid),
            "exp3_reason": (cv.reason + " " + reason).strip(),
            "exp3_reachable": bool(cv.reachable),
            "exp2_score": float(cv.final_score),
            "exp2_distance": float(cv.distance),
            "exp2_bearing_deg": float(math.degrees(cv.bearing)),
            "exp2_sector_id": sid,
            "exp2_sector_count": int(cv.sector_count),
            "exp2_active_sector_id": active_sid,
            "exp2_in_active_sector": bool(sid == active_sid),
            "sector_id": sid,
            "active_sector_id": active_sid,
            "reachable": bool(cv.reachable),
            "marker_label": label,
            "debug_label": label,
            "label": label,
        }.items():
            _set(raw, k, v)
        return raw

    # ---------------- goal latch ----------------
    def _clear_latch(self) -> None:
        self.latched_goal_raw = None
        self.latched_goal_since = 0.0
        self.latched_goal_sector_id = None

    def _maybe_return_latched_goal(self, pose: Any, scan: Any, now: float, debug: Dict[str, Any], force_reselect: bool = False) -> Optional[Tuple[Any, Dict[str, Any]]]:
        if force_reselect or not self.cfg.goal_latch_enabled or self.latched_goal_raw is None:
            return None
        cv = self._normalize_one(self.latched_goal_raw, pose)
        if cv is None:
            self._clear_latch()
            return None
        age = now - self.latched_goal_since
        if cv.distance <= float(self.cfg.goal_reached_radius_m):
            self.mark_reached(self.latched_goal_raw, now=now)
            self._clear_latch()
            debug["latch"] = {"kept": False, "reason": "goal_reached", "distance": round(cv.distance, 3)}
            return None
        if age > float(self.cfg.goal_latch_max_seconds):
            self._clear_latch()
            debug["latch"] = {"kept": False, "reason": "latch_timeout", "age": round(age, 2)}
            return None
        ok, info = self._scan_corridor_ok(scan, cv.bearing, cv.distance)
        if not ok:
            self.blacklist_goal(self.latched_goal_raw, now=now)
            self._clear_latch()
            debug["latch"] = {"kept": False, "reason": "latched_goal_blocked", "scan": info}
            return None
        # Keep the issued goal. This is the core anti-flicker fix.
        sid = cv.sector_id
        self.active_sector_id = sid
        cv.sector_count = int(_get(self.latched_goal_raw, "exp3_sector_count", _get(self.latched_goal_raw, "sector_count", 1)))
        self._score_candidates([cv], {sid: max(1, cv.sector_count)}, now)
        selected = self._export(cv, active_sector_id=sid, reason="latched_goal_keep")
        debug.update({
            "active_sector_id": int(sid),
            "active_sector_center_deg": round(math.degrees(self._sector_center(sid)), 1),
            "pool_reason": "latched_goal_keep",
            "score_threshold_used": "latch",
            "latch": {"kept": True, "age": round(age, 2), "distance": round(cv.distance, 3), "scan": info},
            "selected": {
                "x": round(cv.x, 3),
                "y": round(cv.y, 3),
                "distance": round(cv.distance, 3),
                "bearing_deg": round(math.degrees(cv.bearing), 1),
                "sector_id": int(sid),
                "active_sector_id": int(sid),
                "in_active_sector": True,
                "sector_count": int(cv.sector_count),
                "score": round(cv.final_score, 4),
                "source": cv.source,
                "reason": "latched_goal_keep",
            },
        })
        self.last_debug = debug
        return selected, debug

    # ---------------- primary selection ----------------
    def _prepare_candidates(self, candidates: Optional[Iterable[Any]], pose: Any, scan: Any, now: float) -> Tuple[List[CandidateView], Dict[str, Any]]:
        upstream = list(candidates or [])
        generated: List[Any] = []
        if self.cfg.lidar_candidate_enable and scan is not None and len(upstream) < self.cfg.min_candidates_total:
            generated = self.generate_lidar_candidates(scan, pose)
        cvs = self.normalize_candidates(upstream + generated, pose)
        raw_count = len(cvs)
        reach_debug = self._apply_scan_reachability(cvs, scan)
        cvs = self._hard_filter(cvs, now)
        counts = self._sector_counts(cvs)
        self._score_candidates(cvs, counts, now)
        debug = {
            "upstream_candidates": len(upstream),
            "generated_lidar_candidates": len(generated),
            "raw_candidates": raw_count,
            "after_hard_filter": len(cvs),
            "sector_counts": {int(k): int(v) for k, v in sorted(counts.items())},
            **reach_debug,
        }
        return cvs, debug

    def _pick_best_in_pool(self, pool: List[CandidateView], reason: str) -> Tuple[Optional[CandidateView], str, Any]:
        if not pool:
            return None, reason + ":empty", None
        best = None
        threshold_used: Any = None
        for ratio in self.cfg.recovery_relax_score_schedule:
            th = self.cfg.min_candidate_score * ratio
            good = [cv for cv in pool if cv.final_score >= th]
            if good:
                best = max(good, key=lambda cv: (cv.final_score, cv.distance, cv.raw_score + cv.unknown_gain))
                threshold_used = round(th, 4)
                break
        if best is None:
            best = max(pool, key=lambda cv: (cv.final_score, cv.distance, cv.raw_score + cv.unknown_gain))
            threshold_used = "fallback_best_no_threshold"
            best.reason += " threshold_empty_pick_best"
        return best, reason, threshold_used

    def _active_pool(self, cvs: List[CandidateView], pose: Any, scan: Any, now: float, debug: Dict[str, Any]) -> Tuple[List[CandidateView], int, str]:
        if not cvs:
            return [], self.active_sector_id or 0, "no_candidates"
        sid = self._choose_active_sector(cvs, pose, now)
        exact = [cv for cv in cvs if cv.sector_id == sid]
        if exact:
            return exact, sid, "exact_active_sector"

        # If exact active is empty, try generating lidar candidates only in this sector. This is the promised expansion.
        if scan is not None:
            generated = self.generate_lidar_candidates(scan, pose, sector_id=sid)
            more = self.normalize_candidates(generated, pose)
            more = self._hard_filter(more, now)
            counts = self._sector_counts(cvs + more)
            self._score_candidates(more, counts, now)
            exact_more = [cv for cv in more if cv.sector_id == sid]
            debug["active_sector_generated_candidates"] = len(exact_more)
            if exact_more:
                return exact_more, sid, "generated_inside_active_sector"

        # Last resort: choose global best, but switch active sector to selected sector later.
        return cvs, sid, "global_fallback_will_realign_active"

    def select_goal_with_debug(self,
                               candidates: Optional[Iterable[Any]] = None,
                               pose: Any = None,
                               scan: Any = None,
                               now: Optional[float] = None,
                               force_reselect: bool = False,
                               **kwargs: Any) -> Tuple[Any, Dict[str, Any]]:
        now = _now() if now is None else float(now)
        pose = self.last_pose if pose is None else pose
        self.last_pose = _pose_xyyaw(pose)

        cvs, debug = self._prepare_candidates(candidates, pose, scan, now)
        debug.update({"selected": None, "reason": ""})
        latched = self._maybe_return_latched_goal(pose, scan, now, debug, force_reselect=force_reselect)
        if latched is not None:
            return latched
        if not cvs:
            debug["reason"] = "no_candidate_even_after_lidar_generation"
            self.last_debug = debug
            return None, debug

        pool, active_sid, pool_reason = self._active_pool(cvs, pose, scan, now, debug)
        best, reason, threshold_used = self._pick_best_in_pool(pool, pool_reason)
        if best is None:
            self.last_debug = debug
            return None, debug

        # Critical guarantee: selected must be in active sector. If we had to fall back globally,
        # realign active sector to the selected point so Foxglove and behavior do not disagree.
        if best.sector_id != active_sid:
            active_sid = best.sector_id
            self.active_sector_id = best.sector_id
            self.active_sector_since = now
            reason += ";active_realigned_to_selected_sector"
        else:
            self.active_sector_id = active_sid

        self.last_goal_key = best.key()
        selected = self._export(best, active_sector_id=active_sid, reason=reason)
        if self.cfg.goal_latch_enabled:
            self.latched_goal_raw = selected
            self.latched_goal_since = now
            self.latched_goal_sector_id = int(active_sid)
        debug.update({
            "active_sector_id": int(active_sid),
            "active_sector_center_deg": round(math.degrees(self._sector_center(active_sid)), 1),
            "pool_reason": reason,
            "active_candidates": len([cv for cv in cvs if cv.sector_id == active_sid]),
            "score_threshold_used": threshold_used,
            "selected": {
                "x": round(best.x, 3),
                "y": round(best.y, 3),
                "distance": round(best.distance, 3),
                "bearing_deg": round(math.degrees(best.bearing), 1),
                "sector_id": int(best.sector_id),
                "active_sector_id": int(active_sid),
                "in_active_sector": bool(best.sector_id == active_sid),
                "sector_count": int(best.sector_count),
                "score": round(best.final_score, 4),
                "source": best.source,
                "reason": best.reason.strip(),
            },
        })
        self.last_debug = debug
        return selected, debug

    def select_goal(self, candidates: Optional[Iterable[Any]] = None, pose: Any = None, scan: Any = None, **kwargs: Any) -> Any:
        selected, _debug = self.select_goal_with_debug(candidates=candidates, pose=pose, scan=scan, **kwargs)
        return selected

    def select(self, candidates: Optional[Iterable[Any]] = None, pose: Any = None, scan: Any = None, **kwargs: Any) -> Any:
        return self.select_goal(candidates=candidates, pose=pose, scan=scan, **kwargs)

    def choose_goal(self, candidates: Optional[Iterable[Any]] = None, pose: Any = None, scan: Any = None, **kwargs: Any) -> Any:
        return self.select_goal(candidates=candidates, pose=pose, scan=scan, **kwargs)

    def choose(self, candidates: Optional[Iterable[Any]] = None, pose: Any = None, scan: Any = None, **kwargs: Any) -> Any:
        return self.select_goal(candidates=candidates, pose=pose, scan=scan, **kwargs)

    def __call__(self, candidates: Optional[Iterable[Any]] = None, pose: Any = None, scan: Any = None, **kwargs: Any) -> Any:
        return self.select_goal(candidates=candidates, pose=pose, scan=scan, **kwargs)

    def _filter_by_active_area(self,
                               candidates: Optional[Iterable[Any]],
                               pose: Any = None,
                               scan: Any = None,
                               return_debug: bool = False,
                               **kwargs: Any) -> Any:
        now = _now()
        pose = self.last_pose if pose is None else pose
        self.last_pose = _pose_xyyaw(pose)
        cvs, debug = self._prepare_candidates(candidates, pose, scan, now)
        if not cvs:
            self.last_debug = debug
            return ([], debug) if return_debug else []
        pool, active_sid, reason = self._active_pool(cvs, pose, scan, now, debug)
        if not pool:
            pool = cvs
            reason = "filter_global_fallback"
        # If fallback pool contains other sectors, choose best first, then return only its sector to keep marker coherent.
        if reason.startswith("global_fallback") or reason == "filter_global_fallback":
            best, _r, _t = self._pick_best_in_pool(pool, reason)
            if best is not None:
                active_sid = best.sector_id
                self.active_sector_id = active_sid
                self.active_sector_since = now
                pool = [cv for cv in cvs if cv.sector_id == active_sid] or [best]
        exported = [self._export(cv, active_sector_id=active_sid, reason=reason) for cv in sorted(pool, key=lambda c: (c.final_score, c.distance), reverse=True)]
        debug.update({
            "active_sector_id": int(active_sid),
            "active_sector_center_deg": round(math.degrees(self._sector_center(active_sid)), 1),
            "returned": len(exported),
            "pool_reason": reason,
        })
        self.last_debug = debug
        return (exported, debug) if return_debug else exported

    # ---------------- arrival / recovery state ----------------
    def mark_reached(self, goal: Any = None, now: Optional[float] = None) -> None:
        now = _now() if now is None else float(now)
        xy = self._candidate_xy(goal) if goal is not None else None
        if xy is None and self.last_goal_key is not None:
            xy = (self.last_goal_key[0] / 10.0, self.last_goal_key[1] / 10.0)
        if xy is not None:
            self.visited.append((float(xy[0]), float(xy[1]), now))
        self._clear_latch()
        if self.cfg.observe_spin_on_arrival:
            self.observe_until = now + self.cfg.observe_spin_duration_s
            self.stop_until = self.observe_until + self.cfg.observe_stop_after_spin_s

    def on_goal_reached(self, goal: Any = None, **kwargs: Any) -> None:
        self.mark_reached(goal=goal, **kwargs)

    def blacklist_goal(self, goal: Any, now: Optional[float] = None) -> None:
        now = _now() if now is None else float(now)
        xy = self._candidate_xy(goal)
        if xy is not None:
            self.blacklist.append((float(xy[0]), float(xy[1]), now))

    def update_after_navigation_result(self, success: bool, goal: Any = None, reason: str = "") -> None:
        if success:
            self.mark_reached(goal)
        else:
            self.blacklist_goal(goal)

    def is_observing(self, now: Optional[float] = None) -> bool:
        now = _now() if now is None else float(now)
        return now < self.observe_until or now < self.stop_until

    def get_observe_cmd(self, now: Optional[float] = None) -> Optional[Dict[str, float]]:
        now = _now() if now is None else float(now)
        if now < self.observe_until:
            return {"vx": 0.0, "vy": 0.0, "wz": float(self.cfg.observe_spin_wz), "mode": "observe_spin"}
        if now < self.stop_until:
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0, "mode": "observe_stop"}
        return None

    def reset_sector(self) -> None:
        self.active_sector_id = None
        self.active_sector_since = _now()

    def reset(self) -> None:
        self.active_sector_id = None
        self.active_sector_since = _now()
        self.initial_yaw = None
        self.task_start_time = _now()
        self.last_goal_key = None
        self._clear_latch()
        self.visited.clear()
        self.blacklist.clear()
        self.observe_until = 0.0
        self.stop_until = 0.0
        self.last_debug = {}


# Compatibility aliases for old call sites.
ExploreGoalSelector = ExploreGoalSelectorExp3
SemanticExploreGoalSelector = ExploreGoalSelectorExp3
FrontierGoalSelector = ExploreGoalSelectorExp3
Exp3GoalSelector = ExploreGoalSelectorExp3
ExploreGoalSelectorExp2 = ExploreGoalSelectorExp3
SemanticExploreGoalSelectorExp2 = ExploreGoalSelectorExp3
FrontierGoalSelectorExp2 = ExploreGoalSelectorExp3


def load_config_file(path: str) -> Dict[str, Any]:
    try:
        import yaml  # type: ignore
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def _fake_scan() -> Any:
    class Scan:
        pass
    s = Scan()
    s.angle_min = -math.pi
    s.angle_increment = math.radians(1.0)
    s.range_min = 0.05
    s.range_max = 4.0
    s.ranges = [3.5 for _ in range(360)]
    return s


def _self_test() -> int:
    sel = ExploreGoalSelectorExp3({
        "sector_count": 8,
        "prefer_initial_direction": False,
        "active_sector_keep_seconds": 10.0,
        "sector_switch_min_count_gain": 4,
    })
    assert sel._sector_id(0.0) == 0, "sector_00 must be forward"
    assert sel._sector_id(math.radians(46)) == 1, "sector ids must increase CCW"
    assert sel._sector_id(math.radians(-46)) == 7, "negative/right side should wrap to last sector"
    pose = (0.0, 0.0, 0.0)
    candidates = [
        {"x": 2.0, "y": 0.0, "score": 0.95, "unknown_gain": 0.1, "source": "test_s0"},
        {"x": 0.0, "y": 1.4, "score": 0.20, "unknown_gain": 0.2, "source": "test_s2a"},
        {"x": -0.2, "y": 1.6, "score": 0.15, "unknown_gain": 0.2, "source": "test_s2b"},
        {"x": 0.3, "y": 1.7, "score": 0.15, "unknown_gain": 0.2, "source": "test_s2c"},
    ]
    goal, debug = sel.select_goal_with_debug(candidates, pose=pose, scan=None)
    print(json.dumps({"goal": goal, "debug": debug}, ensure_ascii=False, indent=2))
    if goal is None:
        return 2
    if int(goal["exp3_sector_id"]) != int(debug["active_sector_id"]):
        return 3
    if int(debug["active_sector_id"]) != 2:
        return 4
    # Next tick gives many sector_0 points, but latch should keep previous goal/sector.
    stronger_new_sector = [{"x": 1.5 + 0.1*i, "y": 0.0, "score": 0.9, "unknown_gain": 0.1, "source": "new"} for i in range(8)]
    goal2, debug2 = sel.select_goal_with_debug(stronger_new_sector, pose=pose, scan=None)
    print(json.dumps({"goal2": goal2, "debug2": debug2}, ensure_ascii=False, indent=2))
    if not debug2.get("latch", {}).get("kept"):
        return 5
    if int(goal2["exp3_sector_id"]) != int(goal["exp3_sector_id"]):
        return 6
    sel.mark_reached(goal)
    print("observe_cmd=", sel.get_observe_cmd())
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="EXP3 STRICT v3 selector")
    parser.add_argument("--config", default="")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        raise SystemExit(_self_test())
    cfg = load_config_file(args.config) if args.config else {}
    sel = ExploreGoalSelectorExp3(cfg)
    print(json.dumps(dataclasses.asdict(sel.cfg), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
