#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared helpers for Qwen target -> Nav2 reuse pipeline."""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
_DEBUG_DIR = _SCRIPT_DIR.parent / "debug"
if str(_DEBUG_DIR) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(_DEBUG_DIR))

from qwen_map_goal_utils import load_map_yaml_meta  # noqa: E402

try:
    from scripts.slam.map_goal_validate import (  # type: ignore
        DEFAULT_ROBOT_RADIUS,
        is_footprint_known_free,
    )
except ImportError:
    _SLAM_DIR = _SCRIPT_DIR.parent / "slam"
    if str(_SLAM_DIR) not in __import__("sys").path:
        __import__("sys").path.insert(0, str(_SLAM_DIR))
    from map_goal_validate import DEFAULT_ROBOT_RADIUS, is_footprint_known_free  # noqa: E402


class NavPhase(str, Enum):
    TARGET_RECEIVED = "TARGET_RECEIVED"
    CONTROL_STOPPED = "CONTROL_STOPPED"
    MAPPING_HANDOFF_COMPLETE = "MAPPING_HANDOFF_COMPLETE"
    SENSOR_BASE_VERIFIED = "SENSOR_BASE_VERIFIED"
    NAV2_OVERLAY_STARTED = "NAV2_OVERLAY_STARTED"
    LOCALIZATION_ACTIVE = "LOCALIZATION_ACTIVE"
    LOCALIZATION_SETTLED = "LOCALIZATION_SETTLED"
    LOCALIZATION_UNSETTLED = "LOCALIZATION_UNSETTLED"
    MAP_NOT_RECEIVED = "MAP_NOT_RECEIVED"
    MAP_METADATA_MISMATCH = "MAP_METADATA_MISMATCH"
    PHYSICAL_MOVE_DETECTED = "PHYSICAL_MOVE_DETECTED"
    PATH_VALIDATED = "PATH_VALIDATED"
    GOAL_ACCEPTED = "GOAL_ACCEPTED"
    NAVIGATING = "NAVIGATING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    SENSOR_LOST = "SENSOR_LOST"
    NO_PROGRESS = "NO_PROGRESS"


TERMINAL_PHASES = frozenset(
    {
        NavPhase.SUCCEEDED,
        NavPhase.FAILED,
        NavPhase.CANCELED,
        NavPhase.LOCALIZATION_UNSETTLED,
        NavPhase.MAP_NOT_RECEIVED,
        NavPhase.MAP_METADATA_MISMATCH,
        NavPhase.PHYSICAL_MOVE_DETECTED,
        NavPhase.SENSOR_LOST,
        NavPhase.NO_PROGRESS,
    }
)

PHASE_ORDER = [
    NavPhase.TARGET_RECEIVED,
    NavPhase.CONTROL_STOPPED,
    NavPhase.MAPPING_HANDOFF_COMPLETE,
    NavPhase.SENSOR_BASE_VERIFIED,
    NavPhase.NAV2_OVERLAY_STARTED,
    NavPhase.LOCALIZATION_ACTIVE,
    NavPhase.LOCALIZATION_SETTLED,
    NavPhase.PATH_VALIDATED,
    NavPhase.GOAL_ACCEPTED,
    NavPhase.NAVIGATING,
]


def realpath(path: Path | str) -> Path:
    return Path(os.path.realpath(str(path)))


def atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    data = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(data)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def compute_yaw_spread_rad(yaws: Sequence[float]) -> float:
    """Circular yaw spread: 2*pi - max(gap), not max gap alone."""
    if len(yaws) < 2:
        return 0.0
    normalized = sorted(float(y) % (2.0 * math.pi) for y in yaws)
    gaps = [normalized[i + 1] - normalized[i] for i in range(len(normalized) - 1)]
    wrap_gap = (normalized[0] + 2.0 * math.pi) - normalized[-1]
    gaps.append(wrap_gap)
    return 2.0 * math.pi - max(gaps)


def compute_yaw_spread_deg(yaws: Sequence[float]) -> float:
    return math.degrees(compute_yaw_spread_rad(yaws))


@dataclass(frozen=True)
class ParsedGoal:
    session_id: str
    candidate_id: str
    map_yaml: Path
    goal_x: float
    goal_y: float
    goal_yaw: float
    bundle_fingerprint: str
    raw: Dict[str, Any]


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _extract_goal_pose(data: Dict[str, Any]) -> Tuple[float, float, float]:
    goal = data.get("goal") or data.get("goal_pose_map") or {}
    x = goal.get("x")
    y = goal.get("y")
    yaw = goal.get("yaw_rad", goal.get("yaw"))
    if yaw is None and "yaw_deg" in goal:
        yaw = math.radians(float(goal["yaw_deg"]))
    if not (_finite(x) and _finite(y) and _finite(yaw)):
        raise ValueError("goal x/y/yaw must be finite")
    return float(x), float(y), normalize_yaw(float(yaw))


def _extract_candidate_id(data: Dict[str, Any]) -> str:
    for key in ("candidate_id", "selected_candidate_id"):
        if key in data and data[key] is not None:
            return str(data[key])
    sel = data.get("qwen_selection") or {}
    if sel.get("selected_local_id") is not None:
        return str(sel["selected_local_id"])
    raise ValueError("candidate_id missing")


def load_goal_inputs_lenient(
    *,
    session_id: str,
    map_yaml: Path,
    goal_json: Path,
    candidate_bundle: Path,
    pose_json: Path,
) -> Tuple[ParsedGoal, str]:
    """Load goal for navigation with minimal checks (files + coordinates only)."""
    try:
        return validate_goal_inputs(
            session_id=session_id,
            map_yaml=map_yaml,
            goal_json=goal_json,
            candidate_bundle=candidate_bundle,
            pose_json=pose_json,
        )
    except ValueError as exc:
        map_yaml = realpath(map_yaml)
        goal_json = realpath(goal_json)
        if not map_yaml.is_file():
            raise ValueError(f"map_yaml not found: {map_yaml}") from exc
        if not goal_json.is_file():
            raise ValueError(f"goal_json not found: {goal_json}") from exc
        goal_data = json.loads(goal_json.read_text(encoding="utf-8"))
        gx, gy, gyaw = _extract_goal_pose(goal_data)
        try:
            candidate_id = _extract_candidate_id(goal_data)
        except ValueError:
            candidate_id = "lenient"
        return (
            ParsedGoal(
                session_id=session_id,
                candidate_id=candidate_id,
                map_yaml=map_yaml,
                goal_x=gx,
                goal_y=gy,
                goal_yaw=gyaw,
                bundle_fingerprint=str(goal_data.get("bundle_fingerprint", "")),
                raw=goal_data,
            ),
            f"lenient:{exc}",
        )


def validate_goal_inputs(
    *,
    session_id: str,
    map_yaml: Path,
    goal_json: Path,
    candidate_bundle: Path,
    pose_json: Path,
) -> Tuple[ParsedGoal, str]:
    """Validate session-bound files; return ParsedGoal or raise ValueError."""
    map_yaml = realpath(map_yaml)
    goal_json = realpath(goal_json)
    candidate_bundle = realpath(candidate_bundle)
    pose_json = realpath(pose_json)

    for label, path in (
        ("map_yaml", map_yaml),
        ("goal_json", goal_json),
        ("candidate_bundle", candidate_bundle),
        ("pose_json", pose_json),
    ):
        if not path.is_file():
            raise ValueError(f"{label} not found: {path}")

    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    pgm_name = str(map_data.get("image", map_yaml.with_suffix(".pgm").name))
    pgm_path = realpath(map_yaml.parent / pgm_name)
    if not pgm_path.is_file():
        raise ValueError(f"map pgm not found: {pgm_path}")

    goal_data = json.loads(goal_json.read_text(encoding="utf-8"))
    bundle_data = json.loads(candidate_bundle.read_text(encoding="utf-8"))
    pose_data = json.loads(pose_json.read_text(encoding="utf-8"))

    goal_session = str(goal_data.get("session_id", ""))
    if goal_session != session_id:
        raise ValueError(f"goal.session_id mismatch: {goal_session!r} != {session_id!r}")

    goal_map = realpath(Path(str(goal_data.get("map_yaml", ""))))
    if goal_map != map_yaml:
        raise ValueError(f"goal.map_yaml mismatch: {goal_map} != {map_yaml}")

    bundle_session = str(bundle_data.get("session_id", ""))
    if bundle_session and bundle_session != session_id:
        raise ValueError(f"bundle.session_id mismatch: {bundle_session!r}")

    pose_session = str(pose_data.get("session_id", ""))
    if pose_session and pose_session != session_id:
        raise ValueError(f"pose.session_id mismatch: {pose_session!r}")

    candidate_id = _extract_candidate_id(goal_data)
    candidates = bundle_data.get("candidates") or []
    cand_ids = {str(c.get("candidate_id", c.get("local_id", ""))) for c in candidates}
    if candidate_id not in cand_ids:
        raise ValueError(f"candidate_id {candidate_id!r} not in bundle")

    goal_fp = str(goal_data.get("bundle_fingerprint", ""))
    bundle_fp = str(bundle_data.get("bundle_fingerprint", ""))
    if goal_fp and bundle_fp and goal_fp != bundle_fp:
        raise ValueError("bundle_fingerprint mismatch")

    gx, gy, gyaw = _extract_goal_pose(goal_data)

    return (
        ParsedGoal(
            session_id=session_id,
            candidate_id=candidate_id,
            map_yaml=map_yaml,
            goal_x=gx,
            goal_y=gy,
            goal_yaw=gyaw,
            bundle_fingerprint=goal_fp or bundle_fp,
            raw=goal_data,
        ),
        "ok",
    )


def compute_max_path_length_m(
    robot_x: float,
    robot_y: float,
    goal_x: float,
    goal_y: float,
    *,
    cap: float = 20.0,
    floor: float = 3.0,
    margin: float = 2.0,
    scale: float = 3.0,
) -> float:
    straight = math.hypot(goal_x - robot_x, goal_y - robot_y)
    return min(cap, max(floor, straight * scale + margin))


def path_length_m(poses: Sequence[Any]) -> float:
    total = 0.0
    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position if hasattr(poses[i - 1], "pose") else poses[i - 1]
        p1 = poses[i].pose.position if hasattr(poses[i], "pose") else poses[i]
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


def validate_planned_path(
    path_poses: Sequence[Any],
    *,
    robot_x: float,
    robot_y: float,
    goal_x: float,
    goal_y: float,
    map_frame: str = "map",
    start_tol_m: float = 0.50,
    goal_tol_m: float = 0.35,
    min_path_m: float = 0.15,
    max_path_m: float,
    path_frame: Optional[str] = None,
) -> Tuple[bool, str]:
    if len(path_poses) < 2:
        return False, "empty path"

    for pose in path_poses:
        p = pose.pose.position if hasattr(pose, "pose") else pose
        if not (_finite(p.x) and _finite(p.y)):
            return False, "NaN in path"

    if path_frame is not None and path_frame != map_frame:
        return False, f"path frame {path_frame!r} != {map_frame!r}"

    length = path_length_m(path_poses)
    if length < min_path_m:
        return False, f"path too short ({length:.3f}m)"
    if length > max_path_m:
        return False, f"path too long ({length:.3f}m > {max_path_m:.3f}m)"

    start = path_poses[0].pose.position
    end = path_poses[-1].pose.position
    start_err = math.hypot(start.x - robot_x, start.y - robot_y)
    goal_err = math.hypot(end.x - goal_x, end.y - goal_y)
    if start_err > start_tol_m:
        return False, f"start error {start_err:.3f}m > {start_tol_m:.3f}m"
    if goal_err > goal_tol_m:
        return False, f"goal error {goal_err:.3f}m > {goal_tol_m:.3f}m"
    return True, "ok"


def project_goal_nearby(
    goal_x: float,
    goal_y: float,
    grid: Any,
    *,
    max_projection_m: float = 0.20,
    step_m: float = 0.05,
    robot_radius: float = DEFAULT_ROBOT_RADIUS,
) -> Tuple[Optional[Tuple[float, float]], str]:
    """Small safety projection around original candidate goal."""
    if is_footprint_known_free(grid, goal_x, goal_y, robot_radius)[0]:
        return (goal_x, goal_y), "ok"

    best: Optional[Tuple[float, float]] = None
    best_dist = max_projection_m + 1.0
    steps = int(math.ceil(max_projection_m / step_m))
    for ring in range(1, steps + 1):
        radius = ring * step_m
        n_angles = max(8, ring * 8)
        for k in range(n_angles):
            ang = 2.0 * math.pi * k / n_angles
            tx = goal_x + radius * math.cos(ang)
            ty = goal_y + radius * math.sin(ang)
            if is_footprint_known_free(grid, tx, ty, robot_radius)[0]:
                dist = math.hypot(tx - goal_x, ty - goal_y)
                if dist <= max_projection_m and dist < best_dist:
                    best = (tx, ty)
                    best_dist = dist
        if best is not None:
            return best, f"projected {best_dist:.3f}m"
    return None, "no safe projection within 0.20m"


def validate_goal_on_map(
    grid: Any,
    goal_x: float,
    goal_y: float,
    robot_x: float,
    robot_y: float,
    *,
    map_yaml: Path,
    max_projection_m: float = 0.20,
) -> Tuple[float, float, str]:
    meta = load_map_yaml_meta(map_yaml)
    if not (meta.origin_x <= goal_x <= meta.origin_x + meta.width * meta.resolution):
        raise ValueError("goal x out of map bounds")
    if not (meta.origin_y <= goal_y <= meta.origin_y + meta.height * meta.resolution):
        raise ValueError("goal y out of map bounds")

    dist = math.hypot(goal_x - robot_x, goal_y - robot_y)
    if dist > 50.0:
        raise ValueError(f"goal too far from robot ({dist:.1f}m)")

    projected, reason = project_goal_nearby(
        goal_x, goal_y, grid, max_projection_m=max_projection_m
    )
    if projected is None:
        raise ValueError(reason)
    return projected[0], projected[1], reason


def can_advance_phase(current: NavPhase, nxt: NavPhase) -> bool:
    if current in TERMINAL_PHASES:
        return False
    try:
        return PHASE_ORDER.index(nxt) == PHASE_ORDER.index(current) + 1
    except ValueError:
        return False


def write_nav2_state(
    runtime_dir: Path,
    *,
    session_id: str,
    state: NavPhase | str,
    map_yaml: Path,
    candidate_id: str,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    extra: Optional[Dict[str, Any]] = None,
) -> None:
    payload: Dict[str, Any] = {
        "session_id": session_id,
        "state": state.value if isinstance(state, NavPhase) else str(state),
        "updated_epoch": time_now(),
        "map_yaml": str(map_yaml),
        "candidate_id": candidate_id,
        "goal": {
            "x": goal_x,
            "y": goal_y,
            "yaw_deg": math.degrees(goal_yaw),
        },
    }
    if extra:
        payload.update(extra)
    atomic_write_json(runtime_dir / "nav2_state.json", payload)



def validate_occupancy_grid_against_yaml(
    *,
    frame_id: str,
    width: int,
    height: int,
    resolution: float,
    data_len: int,
    origin_x: float,
    origin_y: float,
    yaml_resolution: float,
    yaml_origin_x: float,
    yaml_origin_y: float,
    yaml_width: int,
    yaml_height: int,
) -> Tuple[bool, str]:
    if frame_id != "map":
        return False, f"frame_id={frame_id!r}"
    if width <= 0 or height <= 0 or resolution <= 0:
        return False, "invalid map dimensions"
    if data_len != width * height:
        return False, "map data length mismatch"
    if abs(yaml_resolution - resolution) > 1e-4:
        return False, f"resolution mismatch yaml={yaml_resolution} map={resolution}"
    if yaml_width != width or yaml_height != height:
        return False, f"size mismatch yaml=({yaml_width},{yaml_height}) map=({width},{height})"
    if abs(yaml_origin_x - origin_x) > 1e-3:
        return False, "origin x mismatch"
    if abs(yaml_origin_y - origin_y) > 1e-3:
        return False, "origin y mismatch"
    return True, "ok"


def time_now() -> float:
    import time

    return time.time()


# ---------------------------------------------------------------------------
# Process identity / handoff ack
# ---------------------------------------------------------------------------


def read_proc_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return ""
    return raw.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()


def read_proc_exe(pid: int) -> str:
    try:
        return os.readlink(f"/proc/{pid}/exe")
    except OSError:
        return ""


def pid_alive(pid: Optional[int]) -> bool:
    if pid is None:
        return False
    try:
        os.kill(int(pid), 0)
        return True
    except OSError:
        return False


def read_proc_start_ticks(pid: int) -> Optional[int]:
    try:
        fields = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()
        return int(fields[21])
    except (OSError, IndexError, ValueError):
        return None


def script_basename_from_cmdline(cmdline: str) -> str:
    for tok in cmdline.split():
        if tok.endswith(".py"):
            return Path(tok).name
    return ""


def build_process_identity(role: str, pid: Optional[int]) -> Optional[Dict[str, Any]]:
    if pid is None:
        return None
    pid_i = int(pid)
    if not pid_alive(pid_i):
        return None
    cmdline = read_proc_cmdline(pid_i)
    return {
        "role": role,
        "pid": pid_i,
        "start_ticks": str(read_proc_start_ticks(pid_i) or ""),
        "exe": read_proc_exe(pid_i),
        "script_basename": script_basename_from_cmdline(cmdline),
        "cmdline": cmdline,
    }


ROLE_MATCHERS: Dict[str, Dict[str, Any]] = {
    "lidar": {"script_substrings": ("ydlidar",), "exe_substrings": ("ydlidar",)},
    "scan_filter": {
        "script_basenames": ("simple_scan_filter.py",),
        "script_substrings": ("simple_scan_filter",),
    },
    "chassis": {
        "script_basenames": ("m1_pwm_cmd_vel_bridge.py",),
        "script_substrings": ("m1_pwm_cmd_vel_bridge",),
    },
    "static_tf": {
        "exe_substrings": ("static_transform_publisher",),
        "script_substrings": ("static_transform_publisher",),
    },
    "foxglove": {
        "script_substrings": ("foxglove_bridge",),
        "exe_substrings": ("foxglove_bridge",),
    },
}


def identity_matches_role(identity: Dict[str, Any], role: str) -> bool:
    rules = ROLE_MATCHERS.get(role) or {}
    exe = str(identity.get("exe", "")).lower()
    cmdline = str(identity.get("cmdline", "")).lower()
    basename = str(identity.get("script_basename", "")).lower()
    for b in rules.get("script_basenames", ()):
        if basename == b.lower():
            return True
    for s in rules.get("script_substrings", ()):
        if s.lower() in cmdline or s.lower() in basename:
            return True
    for s in rules.get("exe_substrings", ()):
        if s.lower() in exe:
            return True
    return False


def validate_process_identity(identity: Optional[Dict[str, Any]], role: str) -> Tuple[bool, str]:
    if identity is None:
        return False, f"{role} missing"
    pid = identity.get("pid")
    if not pid_alive(pid):
        return False, f"{role} pid={pid} not alive"
    recorded = str(identity.get("start_ticks", "") or "")
    current = read_proc_start_ticks(int(pid))
    if recorded and current is not None and recorded != str(current):
        return False, f"{role} PID reuse (start_ticks {recorded}!={current})"
    if not identity_matches_role(identity, role):
        return False, f"{role} identity mismatch exe/script"
    return True, "ok"


def validate_handoff_ack(
    request: Dict[str, Any],
    ack: Dict[str, Any],
    *,
    expected_session_id: str,
) -> Tuple[bool, str]:
    if str(ack.get("session_id", "")) != expected_session_id:
        return False, f"session_id mismatch ack={ack.get('session_id')!r}"
    if str(request.get("session_id", "")) != expected_session_id:
        return False, "request session_id mismatch"
    if str(ack.get("state", "")) != "SENSOR_BASE_HELD":
        return False, f"ack.state={ack.get('state')!r} expected SENSOR_BASE_HELD"
    try:
        requested = float(request["requested_epoch"])
        completed = float(ack["completed_epoch"])
    except (KeyError, TypeError, ValueError):
        return False, "missing requested_epoch/completed_epoch"
    if completed <= requested:
        return False, f"completed_epoch {completed} <= requested_epoch {requested}"

    stopped = ack.get("stopped") or {}
    for key in ("joy", "teleop", "slam", "frontier"):
        if not stopped.get(key, False):
            return False, f"stopped.{key} not confirmed"

    processes = ack.get("processes") or {}
    # Legacy flat pid fields are NOT accepted as formal success.
    if not processes and any(k.endswith("_pid") for k in ack.keys()):
        return False, "legacy flat-pid ack rejected; require processes{} identities"

    for role in ("lidar", "scan_filter", "chassis"):
        ok, reason = validate_process_identity(processes.get(role), role)
        if not ok:
            return False, reason

    static_id = processes.get("static_tf")
    if static_id is not None:
        ok, reason = validate_process_identity(static_id, "static_tf")
        if not ok:
            return False, reason
    elif not ack.get("static_tf_present_via_tf", False):
        return False, "static_tf missing and TF not confirmed"

    # foxglove optional
    return True, "ok"


OWNERSHIP_STATES = (
    "MAPPING_OWNS_SENSOR",
    "HANDOFF_REQUESTED",
    "NAV_SESSION_OWNS_SENSOR",
    "NAV2_OWNS_MOTION",
    "FINISHED",
)


def write_ownership(path: Path, session_id: str, state: str, extra: Optional[Dict[str, Any]] = None) -> None:
    if state not in OWNERSHIP_STATES:
        raise ValueError(f"invalid ownership state {state}")
    payload: Dict[str, Any] = {
        "session_id": session_id,
        "state": state,
        "updated_epoch": time_now(),
    }
    if extra:
        payload.update(extra)
    atomic_write_json(path, payload)


def read_ownership(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def ownership_protects_sensors(path: Path) -> bool:
    st = str(read_ownership(path).get("state", ""))
    return st in ("NAV_SESSION_OWNS_SENSOR", "NAV2_OWNS_MOTION", "FINISHED")


def validate_nav2_owner(
    owner: Dict[str, Any],
    *,
    expected_session_id: str,
) -> Tuple[bool, str]:
    if str(owner.get("session_id", "")) != expected_session_id:
        return False, "owner session_id mismatch"
    launch_pid = owner.get("launch_pid")
    if launch_pid is None:
        return False, "missing launch_pid"
    launch_pid = int(launch_pid)
    if not pid_alive(launch_pid):
        return False, f"launch_pid {launch_pid} not alive"
    recorded = owner.get("start_ticks")
    if recorded is not None:
        current = read_proc_start_ticks(launch_pid)
        if current is None or int(recorded) != int(current):
            return False, "PID reuse detected (start_ticks mismatch) — refuse kill"
    expected_cmd = str(owner.get("cmdline", "") or "")
    if expected_cmd:
        cmd = read_proc_cmdline(launch_pid)
        if expected_cmd[:80] not in cmd and "nav2_click_nav_bringup" not in cmd:
            return False, f"cmdline mismatch: {cmd!r}"
    return True, "ok"


def owner_is_stale(owner: Dict[str, Any]) -> bool:
    launch_pid = owner.get("launch_pid")
    if launch_pid is None:
        return True
    if not pid_alive(int(launch_pid)):
        return True
    recorded = owner.get("start_ticks")
    if recorded is not None:
        current = read_proc_start_ticks(int(launch_pid))
        if current is None or int(recorded) != int(current):
            return True
    return False


def pose_delta_ok(
    frozen_x: float,
    frozen_y: float,
    frozen_yaw: float,
    live_x: float,
    live_y: float,
    live_yaw: float,
    *,
    max_xy_m: float = 0.20,
    max_yaw_deg: float = 12.0,
) -> Tuple[bool, Dict[str, float]]:
    xy = math.hypot(live_x - frozen_x, live_y - frozen_y)
    yaw_err = abs(math.degrees(normalize_yaw(live_yaw - frozen_yaw)))
    metrics = {"xy_err_m": xy, "yaw_err_deg": yaw_err}
    return xy <= max_xy_m and yaw_err <= max_yaw_deg, metrics


def classify_map_pose_delta(xy_err: float, yaw_err_deg: float) -> str:
    if xy_err <= 0.20 and yaw_err_deg <= 12.0:
        return "PASS"
    if xy_err <= 0.50 and yaw_err_deg <= 30.0:
        return "WARN"
    return "FAIL"


def classify_odom_pose_delta(
    xy_err: float,
    yaw_err_deg: float,
    *,
    max_xy_m: float = 0.12,
    max_yaw_deg: float = 8.0,
) -> str:
    if xy_err <= max_xy_m and yaw_err_deg <= max_yaw_deg:
        return "PASS"
    return "PHYSICAL_MOVE_DETECTED"


def evaluate_handoff_pose_gates(
    *,
    frozen_map: Tuple[float, float, float],
    live_map: Optional[Tuple[float, float, float]],
    frozen_odom: Optional[Tuple[float, float, float]],
    live_odom: Optional[Tuple[float, float, float]],
) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "ok": True,
        "hard_fail": None,
        "map_status": "SKIP",
        "odom_status": "SKIP",
        "frozen_map_pose": {"x": frozen_map[0], "y": frozen_map[1], "yaw": frozen_map[2]},
        "thresholds": {
            "odom_xy_m": 0.12,
            "odom_yaw_deg": 8.0,
            "map_pass_xy_m": 0.20,
            "map_pass_yaw_deg": 12.0,
            "map_fail_xy_m": 0.50,
            "map_fail_yaw_deg": 30.0,
        },
    }
    if frozen_odom is not None and live_odom is not None:
        oxy = math.hypot(live_odom[0] - frozen_odom[0], live_odom[1] - frozen_odom[1])
        oyaw = abs(math.degrees(normalize_yaw(live_odom[2] - frozen_odom[2])))
        ostatus = classify_odom_pose_delta(oxy, oyaw)
        result["odom_status"] = ostatus
        result["frozen_odom_pose"] = {"x": frozen_odom[0], "y": frozen_odom[1], "yaw": frozen_odom[2]}
        result["current_odom_pose"] = {"x": live_odom[0], "y": live_odom[1], "yaw": live_odom[2]}
        result["odom_delta"] = {"xy_err_m": oxy, "yaw_err_deg": oyaw}
        if ostatus != "PASS":
            result["ok"] = False
            result["hard_fail"] = "PHYSICAL_MOVE_DETECTED"
    elif frozen_odom is None:
        result["ok"] = False
        result["odom_status"] = "MISSING_FROZEN_ODOM"
        result["hard_fail"] = "MISSING_FROZEN_ODOM"
    else:
        result["ok"] = False
        result["odom_status"] = "MISSING_LIVE_ODOM"
        result["hard_fail"] = "MISSING_LIVE_ODOM"

    if live_map is not None:
        mxy = math.hypot(live_map[0] - frozen_map[0], live_map[1] - frozen_map[1])
        myaw = abs(math.degrees(normalize_yaw(live_map[2] - frozen_map[2])))
        mstatus = classify_map_pose_delta(mxy, myaw)
        result["map_status"] = mstatus
        result["current_map_pose"] = {"x": live_map[0], "y": live_map[1], "yaw": live_map[2]}
        result["map_delta"] = {"xy_err_m": mxy, "yaw_err_deg": myaw}
        if mstatus == "FAIL":
            result["ok"] = False
            if result["hard_fail"] is None:
                result["hard_fail"] = "MAP_POSE_JUMP"
    return result


@dataclass
class ProgressWatchState:
    best_distance_remaining: Optional[float] = None
    last_progress_time: float = 0.0
    last_progress_pose: Optional[Tuple[float, float]] = None


def update_progress_watch(
    state: ProgressWatchState,
    *,
    now: float,
    distance_remaining: Optional[float],
    robot_xy: Optional[Tuple[float, float]],
    distance_delta_m: float = 0.10,
    move_delta_m: float = 0.08,
) -> bool:
    progressed = False
    if distance_remaining is not None:
        if state.best_distance_remaining is None:
            state.best_distance_remaining = float(distance_remaining)
        elif state.best_distance_remaining - float(distance_remaining) >= distance_delta_m:
            state.best_distance_remaining = float(distance_remaining)
            progressed = True
    if robot_xy is not None:
        if state.last_progress_pose is None:
            state.last_progress_pose = robot_xy
        else:
            moved = math.hypot(
                robot_xy[0] - state.last_progress_pose[0],
                robot_xy[1] - state.last_progress_pose[1],
            )
            if moved >= move_delta_m:
                state.last_progress_pose = robot_xy
                progressed = True
    if progressed:
        state.last_progress_time = now
    return progressed


def progress_timed_out(state: ProgressWatchState, now: float, timeout_s: float = 30.0) -> bool:
    if state.last_progress_time <= 0.0:
        return False
    return (now - state.last_progress_time) >= timeout_s


def evaluate_amcl_settle_window(
    samples: Sequence[Dict[str, float]],
    *,
    min_samples: int = 8,
    max_x_spread: float = 0.12,
    max_y_spread: float = 0.12,
    max_yaw_spread_deg: float = 10.0,
    max_x_cov: float = 0.30,
    max_y_cov: float = 0.30,
    max_yaw_cov: float = 0.20,
    scan_age_s: float,
    odom_age_s: float,
    map_tf_ok: bool,
    max_scan_age_s: float = 1.0,
    max_odom_age_s: float = 0.5,
) -> Tuple[bool, List[str], Dict[str, Any]]:
    reasons: List[str] = []
    if len(samples) < min_samples:
        reasons.append(f"sample_count={len(samples)}<{min_samples}")
    xs = [s["x"] for s in samples] if samples else []
    ys = [s["y"] for s in samples] if samples else []
    yaws = [s["yaw"] for s in samples] if samples else []
    last = samples[-1] if samples else {"x_cov": 0.0, "y_cov": 0.0, "yaw_cov": 0.0}
    x_spread = (max(xs) - min(xs)) if xs else 0.0
    y_spread = (max(ys) - min(ys)) if ys else 0.0
    yaw_spread = compute_yaw_spread_deg(yaws) if yaws else 0.0
    if xs and x_spread > max_x_spread:
        reasons.append(f"x_spread={x_spread:.3f}>{max_x_spread}")
    if ys and y_spread > max_y_spread:
        reasons.append(f"y_spread={y_spread:.3f}>{max_y_spread}")
    if yaws and yaw_spread > max_yaw_spread_deg:
        reasons.append(f"yaw_spread={yaw_spread:.2f}>{max_yaw_spread_deg}")
    if float(last.get("x_cov", 0.0)) > max_x_cov:
        reasons.append(f"x_cov={last['x_cov']:.3f}>{max_x_cov}")
    if float(last.get("y_cov", 0.0)) > max_y_cov:
        reasons.append(f"y_cov={last['y_cov']:.3f}>{max_y_cov}")
    if float(last.get("yaw_cov", 0.0)) > max_yaw_cov:
        reasons.append(f"yaw_cov={last['yaw_cov']:.3f}>{max_yaw_cov}")
    if scan_age_s > max_scan_age_s:
        reasons.append(f"scan_age={scan_age_s:.3f}>{max_scan_age_s}")
    if odom_age_s > max_odom_age_s:
        reasons.append(f"odom_age={odom_age_s:.3f}>{max_odom_age_s}")
    if not map_tf_ok:
        reasons.append("map_base_link_tf_missing")
    metrics = {
        "sample_count": len(samples),
        "x_spread_m": x_spread,
        "y_spread_m": y_spread,
        "yaw_spread_deg": yaw_spread,
        "x_cov": float(last.get("x_cov", 0.0)),
        "y_cov": float(last.get("y_cov", 0.0)),
        "yaw_cov": float(last.get("yaw_cov", 0.0)),
        "scan_age_s": scan_age_s,
        "odom_age_s": odom_age_s,
        "map_tf_ok": map_tf_ok,
        "reasons": reasons,
    }
    return len(reasons) == 0, reasons, metrics


def amcl_settle_near_threshold(metrics: Dict[str, Any]) -> bool:
    if int(metrics.get("sample_count", 0)) < 5:
        return False
    if not metrics.get("map_tf_ok"):
        return False
    if float(metrics.get("scan_age_s", 99)) > 1.0:
        return False
    if float(metrics.get("odom_age_s", 99)) > 0.5:
        return False
    return (
        float(metrics.get("x_spread_m", 9)) <= 0.18
        and float(metrics.get("y_spread_m", 9)) <= 0.18
        and float(metrics.get("yaw_spread_deg", 99)) <= 15.0
        and float(metrics.get("x_cov", 9)) <= 0.45
        and float(metrics.get("y_cov", 9)) <= 0.45
        and float(metrics.get("yaw_cov", 9)) <= 0.30
    )


def sensor_health_overall_pass(
    *,
    scan_ok: bool,
    scan_filtered_ok: bool,
    odom_ok: bool,
    tf_ok: bool,
    chassis_ok: bool,
    foxglove_ok: bool = False,
) -> bool:
    del foxglove_ok
    return all([scan_ok, scan_filtered_ok, odom_ok, tf_ok, chassis_ok])


def sensor_health_overall_pass_v2(
    *,
    scan_ok: bool,
    scan_filtered_ok: bool,
    odom_ok: bool,
    tf_ok: bool,
    chassis_ok: bool,
    lidar_count: int,
    scan_filter_count: int,
    chassis_count: int,
    foxglove_ok: bool = False,
) -> bool:
    del foxglove_ok
    return all(
        [
            scan_ok,
            scan_filtered_ok,
            odom_ok,
            tf_ok,
            chassis_ok,
            lidar_count == 1,
            scan_filter_count == 1,
            chassis_count == 1,
        ]
    )


def decide_scan_filter_action(count: int, topic_fresh: bool) -> Tuple[str, str]:
    if count == 0 and not topic_fresh:
        return "start_once", "no_filter_and_no_data"
    if count == 0 and topic_fresh:
        return "fail", "data_without_process"
    if count == 1 and topic_fresh:
        return "reuse", "single_fresh"
    if count == 1 and not topic_fresh:
        return "fail", "single_stale_do_not_restart"
    if count > 1:
        return "fail", "multiple_filters"
    return "fail", "unknown"


def decide_static_tf_action(
    *,
    tf_exists: bool,
    owner_pid: Optional[int],
    owner_alive: bool,
) -> Tuple[str, str]:
    if tf_exists:
        return "reuse", "tf_exists"
    if owner_pid is not None and owner_alive:
        return "fail", "owner_alive_but_tf_missing"
    return "start_once", "tf_missing_owner_absent"


def decide_foxglove_action(*, port_listening: bool, bridge_count: int) -> Tuple[str, str]:
    if port_listening:
        return "reuse", "port_listening"
    if bridge_count == 0:
        return "start_once", "no_bridge"
    return "warn", "bridge_exists_but_port_dead"
