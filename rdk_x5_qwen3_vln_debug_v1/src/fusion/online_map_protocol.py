"""Pure protocol and state logic for V1 <-> online map/Qwen/planner fusion.

This file deliberately contains no ROS imports.  The real bridge node is only a
thin transport wrapper around these deterministic functions, so the dangerous
parts can be tested without a chassis connected.  A surprisingly radical idea:
test the steering hand-off before asking the robot to demonstrate it physically.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


class ProtocolError(ValueError):
    """Raised when a message cannot safely be mapped into the fusion contract."""


ACTIVE_BACKEND_STATES = {
    "ACCEPTED",
    "RECEIVED",
    "EXTRACTING",
    "CANDIDATES_READY",
    "QWEN_SELECTING",
    "SELECTING",
    "PLANNING",
    "NAVIGATING",
    "RUNNING",
    "ACTIVE",
}
DONE_BACKEND_STATES = {
    "COMPLETED",
    "SUCCEEDED",
    "REACHED",
    "GOAL_REACHED",
    "ARRIVED",
    "ARRIVED_ALIGNED",
    "DONE",
}
FAIL_BACKEND_STATES = {
    "FAILED",
    "REJECTED",
    "NO_CANDIDATES",
    "NO_VALID_CANDIDATE",
    "PLAN_FAILED",
    "NAV_FAILED",
    "UNREACHABLE",
    "CANCELLED",
    "CANCELED",
    "ABORTED",
    "TIMEOUT",
    "ERROR",
}
TARGET_BACKEND_STATES = {"TARGET_VISIBLE", "TARGET_LOCKED"}


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on", "y"}:
        return True
    if text in {"0", "false", "no", "off", "n", ""}:
        return False
    return default


def _as_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _first(mapping: Mapping[str, Any], keys: Sequence[str], default: Any = None) -> Any:
    for key in keys:
        if key in mapping and mapping[key] is not None:
            return mapping[key]
    return default


def _candidate_pose(raw: Mapping[str, Any]) -> Optional[Dict[str, float]]:
    pose = raw.get("pose") or raw.get("goal_pose") or raw.get("world_pose")
    if isinstance(pose, Mapping):
        x = _as_float(_first(pose, ("x", "px")))
        y = _as_float(_first(pose, ("y", "py")))
        yaw = _as_float(_first(pose, ("yaw", "theta", "yaw_rad")), 0.0)
    elif isinstance(pose, (list, tuple)) and len(pose) >= 2:
        x = _as_float(pose[0])
        y = _as_float(pose[1])
        yaw = _as_float(pose[2], 0.0) if len(pose) >= 3 else 0.0
    else:
        x = _as_float(_first(raw, ("x", "goal_x")))
        y = _as_float(_first(raw, ("y", "goal_y")))
        yaw = _as_float(_first(raw, ("yaw", "theta", "goal_yaw")), 0.0)
    if x is None or y is None:
        return None
    return {"x": x, "y": y, "yaw": 0.0 if yaw is None else yaw}


def normalize_candidate_summary(payload: Mapping[str, Any]) -> Dict[str, Any]:
    """Normalize teammate candidate JSON into intervention-core schema.

    Aliases are accepted intentionally because the candidate extractor is owned
    by another module and is still evolving.  The output itself is strict and
    stable; only the bridge absorbs naming churn.
    """
    if not isinstance(payload, Mapping):
        raise ProtocolError("candidate summary must be a JSON object")

    raw_candidates = _first(
        payload,
        ("candidates", "candidate_points", "frontiers", "goals", "regions"),
        [],
    )
    if raw_candidates is None:
        raw_candidates = []
    if not isinstance(raw_candidates, list):
        raise ProtocolError("candidate list must be an array")

    normalized: List[Dict[str, Any]] = []
    seen_ids = set()
    for index, item in enumerate(raw_candidates):
        if not isinstance(item, Mapping):
            continue
        candidate_id = str(
            _first(item, ("id", "candidate_id", "frontier_id", "region_id"), f"C{index}")
        ).strip()
        if not candidate_id:
            candidate_id = f"C{index}"
        if candidate_id in seen_ids:
            candidate_id = f"{candidate_id}_{index}"
        seen_ids.add(candidate_id)

        heading = _as_float(
            _first(
                item,
                (
                    "heading_deg",
                    "relative_heading_deg",
                    "yaw_rel_deg",
                    "angle_deg",
                    "direction_deg",
                ),
            )
        )
        if heading is None:
            # A branch trigger needs a relative heading.  Do not invent one from
            # an absolute map pose without the robot pose; mark it neutral and
            # let failure-triggered requests still work.
            heading = 0.0

        status = str(_first(item, ("status", "state"), "UNSEEN")).strip().upper()
        visited = _as_bool(_first(item, ("visited", "is_visited", "explored")), False)
        reachable = _as_bool(
            _first(item, ("reachable", "is_reachable", "path_valid")), True
        )
        score = _as_float(
            _first(item, ("score", "final_score", "candidate_score", "utility"))
        )
        path_length = _as_float(
            _first(item, ("path_length", "path_cost", "distance", "travel_cost"))
        )

        candidate: Dict[str, Any] = {
            "id": candidate_id,
            "heading_deg": heading,
            "reachable": reachable,
            "visited": visited,
            "status": status,
        }
        if score is not None:
            candidate["score"] = score
        if path_length is not None:
            candidate["path_length"] = path_length
        pose = _candidate_pose(item)
        if pose is not None:
            candidate["pose"] = pose
        for key in ("information_gain", "semantic_score", "label", "reason"):
            if key in item:
                candidate[key] = item[key]
        normalized.append(candidate)

    decision_distance = _as_float(
        _first(
            payload,
            (
                "decision_distance_m",
                "distance_to_decision_m",
                "junction_distance_m",
                "nearest_branch_distance_m",
            ),
        )
    )
    result: Dict[str, Any] = {
        "map_version": str(
            _first(payload, ("map_version", "map_seq", "generation", "stamp"), "unknown")
        ),
        "returned_to_junction": _as_bool(
            _first(payload, ("returned_to_junction", "junction_revisit")), False
        ),
        "candidates": normalized,
    }
    if decision_distance is not None:
        result["decision_distance_m"] = decision_distance
    unseen_count = _first(payload, ("unseen_candidate_count", "unvisited_count"))
    if unseen_count is not None:
        try:
            result["unseen_candidate_count"] = int(unseen_count)
        except (TypeError, ValueError):
            pass
    junction_id = _first(payload, ("junction_id", "branch_node_id"))
    if junction_id is not None:
        result["junction_id"] = str(junction_id)
    if "stamp" in payload:
        result["source_stamp"] = payload["stamp"]
    return result


def normalize_backend_status(
    payload: Mapping[str, Any],
    *,
    active_request_id: Optional[str],
    require_final_orientation: bool,
) -> Optional[Dict[str, Any]]:
    """Normalize backend status for the existing intervention manager.

    Returns None for stale or structurally unsafe status messages.
    """
    if not isinstance(payload, Mapping):
        raise ProtocolError("backend status must be a JSON object")
    request_id = _first(payload, ("request_id", "session_id", "job_id"))
    if request_id is None or active_request_id is None:
        return None
    request_id = str(request_id)
    if request_id != str(active_request_id):
        return None

    raw_status = str(_first(payload, ("status", "state", "phase"), "")).strip().upper()
    if not raw_status:
        return None

    if raw_status in TARGET_BACKEND_STATES:
        status = raw_status
    elif raw_status in ACTIVE_BACKEND_STATES:
        status = "NAVIGATING" if raw_status in {"NAVIGATING", "RUNNING", "ACTIVE"} else "ACCEPTED"
    elif raw_status in DONE_BACKEND_STATES:
        aligned = _as_bool(
            _first(
                payload,
                (
                    "final_orientation_done",
                    "final_yaw_aligned",
                    "orientation_aligned",
                    "aligned",
                ),
            ),
            False,
        )
        if require_final_orientation and not aligned and raw_status != "ARRIVED_ALIGNED":
            # Keep MAP ownership and wait for the backend to finish the final yaw.
            return {
                "request_id": request_id,
                "status": "NAVIGATING",
                "backend_status": raw_status,
                "waiting_final_orientation": True,
            }
        status = "COMPLETED"
    elif raw_status in FAIL_BACKEND_STATES:
        status = "FAILED"
    else:
        return None

    result: Dict[str, Any] = {
        "request_id": request_id,
        "status": status,
        "backend_status": raw_status,
    }
    for key in (
        "candidate_id",
        "selected_candidate_id",
        "goal_pose",
        "goal_pose_map",
        "reason",
        "error",
        "final_orientation_done",
        "final_yaw_aligned",
    ):
        if key in payload:
            result[key] = payload[key]
    return result


def build_backend_request(
    intervention_request: Mapping[str, Any],
    *,
    instruction: str,
    map_topic: str,
    odom_topic: str,
    scan_topic: str,
    use_memory: bool,
) -> Dict[str, Any]:
    if not isinstance(intervention_request, Mapping):
        raise ProtocolError("intervention request must be a JSON object")
    request_id = str(intervention_request.get("request_id", "")).strip()
    if not request_id:
        raise ProtocolError("intervention request missing request_id")
    decision = str(intervention_request.get("decision", "")).strip().upper()
    if decision not in {"MAP_DIRECT", "MAP_QWEN"}:
        raise ProtocolError(f"unsupported decision: {decision!r}")

    candidate_ids = [str(x) for x in (intervention_request.get("candidate_ids") or [])]
    if decision == "MAP_DIRECT" and len(candidate_ids) == 1:
        operation = "NAVIGATE_CANDIDATE"
        selected_candidate_id: Optional[str] = candidate_ids[0]
    elif candidate_ids:
        operation = "SELECT_AND_NAVIGATE"
        selected_candidate_id = None
    else:
        operation = "EXTRACT_SELECT_AND_NAVIGATE"
        selected_candidate_id = None

    request: Dict[str, Any] = {
        "protocol_version": 1,
        "request_id": request_id,
        "operation": operation,
        "decision": decision,
        "reason_code": intervention_request.get("reason_code", "UNKNOWN"),
        "instruction": instruction,
        "candidate_ids": candidate_ids,
        "selected_candidate_id": selected_candidate_id,
        "robot_pose": intervention_request.get("robot_pose"),
        "evidence": intervention_request.get("evidence", {}),
        "live_inputs": {
            "map_topic": map_topic,
            "odom_topic": odom_topic,
            "scan_topic": scan_topic,
        },
        "options": {
            "use_live_map": True,
            "use_memory": bool(use_memory),
            "keep_mapping_alive": True,
            "return_full_goal_pose": True,
            "execute_navigation": True,
        },
    }
    return request


def clamp_twist_values(
    linear_x: float,
    angular_z: float,
    *,
    max_linear_x: float,
    max_angular_z: float,
) -> Tuple[float, float]:
    max_linear_x = abs(float(max_linear_x))
    max_angular_z = abs(float(max_angular_z))
    x = max(-max_linear_x, min(max_linear_x, float(linear_x)))
    z = max(-max_angular_z, min(max_angular_z, float(angular_z)))
    return x, z


@dataclass(frozen=True)
class BridgeConfig:
    request_timeout_sec: float = 8.0
    backend_heartbeat_timeout_sec: float = 4.0
    backend_cmd_timeout_sec: float = 0.45
    cancel_hold_sec: float = 0.40
    output_rate_hz: float = 20.0
    max_linear_x: float = 0.06
    max_angular_z: float = 0.06
    require_final_orientation: bool = False


@dataclass
class BridgeSession:
    """Small state machine used by the ROS bridge and unit tests."""

    cfg: BridgeConfig
    state: str = "IDLE"
    active_request_id: Optional[str] = None
    request_started: float = 0.0
    last_backend_status_at: Optional[float] = None
    backend_authorized: bool = False
    cancel_started: Optional[float] = None
    last_reason: str = "startup"
    backend_cmd_received_at: Optional[float] = None
    backend_cmd: Tuple[float, float] = (0.0, 0.0)

    def start(self, request_id: str, now: float) -> None:
        if not request_id:
            raise ProtocolError("empty request_id")
        if self.state != "IDLE":
            raise ProtocolError(f"bridge busy with {self.active_request_id}")
        self.state = "WAIT_BACKEND"
        self.active_request_id = str(request_id)
        self.request_started = float(now)
        self.last_backend_status_at = None
        self.backend_authorized = False
        self.cancel_started = None
        self.last_reason = "request_forwarded"
        self.backend_cmd_received_at = None
        self.backend_cmd = (0.0, 0.0)

    def on_backend_status(self, normalized: Mapping[str, Any], now: float) -> str:
        request_id = str(normalized.get("request_id", ""))
        if not self.active_request_id or request_id != self.active_request_id:
            return "IGNORED"
        status = str(normalized.get("status", "")).upper()
        self.last_backend_status_at = float(now)
        if status in {"ACCEPTED", "NAVIGATING"}:
            self.backend_authorized = True
            self.state = "ACTIVE"
            self.last_reason = status.lower()
            return "ACTIVE"
        if status in {"COMPLETED", "FAILED", "TARGET_VISIBLE", "TARGET_LOCKED"}:
            self.state = "FINISHING"
            self.backend_authorized = False
            self.backend_cmd = (0.0, 0.0)
            self.backend_cmd_received_at = None
            self.last_reason = status.lower()
            return status
        return "IGNORED"

    def on_backend_cmd(self, linear_x: float, angular_z: float, now: float) -> bool:
        if self.state != "ACTIVE" or not self.backend_authorized:
            return False
        self.backend_cmd = clamp_twist_values(
            linear_x,
            angular_z,
            max_linear_x=self.cfg.max_linear_x,
            max_angular_z=self.cfg.max_angular_z,
        )
        self.backend_cmd_received_at = float(now)
        self.last_backend_status_at = float(now)
        return True

    def cancel(self, now: float, reason: str) -> None:
        if self.state == "IDLE":
            return
        self.state = "CANCELLING"
        self.backend_authorized = False
        self.backend_cmd = (0.0, 0.0)
        self.backend_cmd_received_at = None
        self.cancel_started = float(now)
        self.last_reason = str(reason)

    def tick(self, now: float) -> Optional[str]:
        now = float(now)
        if self.state == "WAIT_BACKEND" and now - self.request_started > self.cfg.request_timeout_sec:
            self.state = "FINISHING"
            self.last_reason = "backend_request_timeout"
            return "FAILED"
        if (
            self.state == "ACTIVE"
            and self.last_backend_status_at is not None
            and now - self.last_backend_status_at > self.cfg.backend_heartbeat_timeout_sec
        ):
            self.state = "FINISHING"
            self.backend_authorized = False
            self.backend_cmd = (0.0, 0.0)
            self.last_reason = "backend_heartbeat_timeout"
            return "FAILED"
        if (
            self.state == "CANCELLING"
            and self.cancel_started is not None
            and now - self.cancel_started >= self.cfg.cancel_hold_sec
        ):
            self.reset("cancel_complete")
            return "CANCEL_COMPLETE"
        return None

    def output_cmd(self, now: float) -> Tuple[float, float, str]:
        if self.state != "ACTIVE" or not self.backend_authorized:
            return 0.0, 0.0, "hold_not_active"
        if self.backend_cmd_received_at is None:
            return 0.0, 0.0, "hold_no_backend_cmd"
        age = float(now) - self.backend_cmd_received_at
        if age > self.cfg.backend_cmd_timeout_sec:
            return 0.0, 0.0, "hold_stale_backend_cmd"
        return self.backend_cmd[0], self.backend_cmd[1], "backend_cmd"

    def reset(self, reason: str = "reset") -> None:
        self.state = "IDLE"
        self.active_request_id = None
        self.request_started = 0.0
        self.last_backend_status_at = None
        self.backend_authorized = False
        self.cancel_started = None
        self.last_reason = reason
        self.backend_cmd_received_at = None
        self.backend_cmd = (0.0, 0.0)
