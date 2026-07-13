#!/usr/bin/env python3
"""Pure region selection core — no ROS, no motion, no API calls."""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

FORBIDDEN_DECISION_FIELDS: frozenset[str] = frozenset(
    {
        "x",
        "y",
        "yaw",
        "u",
        "v",
        "waypoint_u",
        "waypoint_v",
        "speed",
        "velocity",
        "linear",
        "angular",
        "cmd_vel",
        "turn",
        "drive_distance",
        "linear_velocity",
        "angular_velocity",
        "distance_to_drive",
        "turn_angle",
    }
)

LABEL_PATTERN = re.compile(r"^[A-Z]{1,2}$")

UNSUPPORTED_VISUAL_CLAIM_KEYWORDS: Tuple[str, ...] = (
    "看到",
    "看见",
    "发现了",
    "桌子",
    "厨房",
    "瓶子",
    "绿色瓶子",
    "房间像",
    "已经看到",
    "visible bottle",
    "see a bottle",
    "kitchen",
    "table",
)

REASON_CODES_SUGGESTED: Tuple[str, ...] = (
    "UNEXPLORED_REGION_PRIORITY",
    "HIGHER_UNKNOWN_GAIN",
    "BETTER_CLEARANCE",
    "CLOSER_SAFE_REGION",
    "LARGER_FRONTIER",
    "MERGED_REGION_PREFERENCE",
    "FALLBACK_BALANCE",
)


@dataclass
class RegionCandidate:
    label: str
    internal_region_id: str
    direction: str
    distance_m: float
    unknown_gain_cells: int
    unknown_gain_ratio: float
    minimum_clearance_m: float
    mean_clearance_m: float
    frontier_cell_count: int
    merged: bool = False
    source_cluster_ids: List[str] = field(default_factory=list)
    path_checked: bool = False
    reachable: Optional[bool] = None


@dataclass
class RegionSelectionInput:
    snapshot_id: str
    cycle_id: int
    map_stamp: float
    capture_time: str
    target_instruction: str
    robot_pose: Dict[str, float]
    regions: List[RegionCandidate]
    annotated_map_file: str
    analysis_status: str = "OK"
    expires_after_s: float = 300.0


@dataclass
class RegionSelectionDecision:
    snapshot_id: str
    selected_region: Optional[str]
    fallback_regions: List[str]
    confidence: float
    reason_code: str
    evidence: List[str]


@dataclass
class DecisionValidationResult:
    decision_valid: bool
    errors: List[str]
    parsed: Optional[Dict[str, Any]] = None
    unsupported_visual_claims: List[str] = field(default_factory=list)


@dataclass
class QwenRequestManifest:
    run_id: str
    call_id: str
    snapshot_id: str
    model: str
    target_instruction: str
    valid_region_labels: List[str]
    image_file: str
    request_start_time: str


def _candidate_from_dict(data: Dict[str, Any]) -> RegionCandidate:
    unknown_gain_cells = int(data.get("unknown_gain_cells", 0))
    total_hint = float(data.get("unknown_gain_ratio", 0.0))
    if "unknown_gain_ratio" not in data and unknown_gain_cells > 0:
        total_hint = 0.0
    return RegionCandidate(
        label=str(data["label"]),
        internal_region_id=str(data["internal_region_id"]),
        direction=str(data.get("direction", "")),
        distance_m=float(data.get("distance_m", 0.0)),
        unknown_gain_cells=unknown_gain_cells,
        unknown_gain_ratio=float(total_hint),
        minimum_clearance_m=float(data.get("minimum_clearance_m", 0.0)),
        mean_clearance_m=float(data.get("mean_clearance_m", 0.0)),
        frontier_cell_count=int(data.get("frontier_cell_count", 0)),
        merged=bool(data.get("merged", False)),
        source_cluster_ids=list(data.get("source_cluster_ids", [])),
        path_checked=bool(data.get("path_checked", False)),
        reachable=data.get("reachable"),
    )


def load_region_snapshot(
    snapshot_path: Path,
    target_instruction: str = "",
) -> Tuple[RegionSelectionInput, Dict[str, Any]]:
    """Load snapshot JSON; returns input and raw dict."""
    path = Path(snapshot_path).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"SNAPSHOT_FILE_NOT_FOUND path={path}")
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"SNAPSHOT_JSON_INVALID detail={exc}") from exc

    if not isinstance(raw, dict):
        raise ValueError("SNAPSHOT_JSON_INVALID root must be object")

    regions_raw = raw.get("accepted_regions") or raw.get("regions") or []
    annotated = str(
        raw.get("annotated_map_file")
        or (path.parent / "annotated_map.png")
    )

    inp = RegionSelectionInput(
        snapshot_id=str(raw.get("snapshot_id", "")),
        cycle_id=int(raw.get("cycle_id", 0)),
        map_stamp=float(raw.get("map_stamp", 0.0)),
        capture_time=str(raw.get("capture_time", "")),
        target_instruction=target_instruction,
        robot_pose=dict(raw.get("robot_pose", {})),
        regions=[_candidate_from_dict(r) for r in regions_raw],
        annotated_map_file=annotated,
        analysis_status=str(raw.get("analysis_status", "OK")),
        expires_after_s=float(raw.get("expires_after_s", 300.0)),
    )
    return inp, raw


def validate_region_snapshot(inp: RegionSelectionInput) -> List[str]:
    """Return list of error codes; empty if valid."""
    errors: List[str] = []
    if not inp.snapshot_id.strip():
        errors.append("SNAPSHOT_ID_MISSING")
    if not inp.regions:
        errors.append("SNAPSHOT_REGIONS_EMPTY")

    labels: List[str] = []
    for region in inp.regions:
        if not LABEL_PATTERN.match(region.label):
            errors.append(f"SNAPSHOT_REGION_INVALID label_format={region.label}")
        labels.append(region.label)
        if not region.internal_region_id.strip():
            errors.append(f"SNAPSHOT_REGION_INVALID empty_internal_id label={region.label}")
        if region.distance_m < 0:
            errors.append(f"SNAPSHOT_REGION_INVALID negative_distance label={region.label}")
        if region.minimum_clearance_m < 0:
            errors.append(f"SNAPSHOT_REGION_INVALID negative_clearance label={region.label}")

    if len(labels) != len(set(labels)):
        errors.append("SNAPSHOT_DUPLICATE_LABEL")

    map_path = Path(inp.annotated_map_file)
    if not map_path.is_file():
        errors.append("SNAPSHOT_MAP_IMAGE_MISSING")
    elif map_path.stat().st_size <= 0:
        errors.append("SNAPSHOT_MAP_IMAGE_MISSING")

    return errors


def build_region_selection_prompt(inp: RegionSelectionInput) -> str:
    """Build strict prompt listing only available region labels."""
    labels = [r.label for r in inp.regions]
    label_list = ", ".join(labels)

    region_lines: List[str] = []
    for r in inp.regions:
        merged_note = "merged" if r.merged else "single_cluster"
        region_lines.append(
            f"- Label {r.label}: direction={r.direction}, distance_m={r.distance_m:.2f}, "
            f"unknown_gain_cells={r.unknown_gain_cells}, minimum_clearance_m={r.minimum_clearance_m:.2f}, "
            f"mean_clearance_m={r.mean_clearance_m:.2f}, frontier_cell_count={r.frontier_cell_count}, "
            f"{merged_note}, source_clusters={r.source_cluster_ids}, path_checked={r.path_checked}"
        )

    robot = inp.robot_pose
    regions_block = "\n".join(region_lines)

    return (
        "You are a region selection module for mobile robot exploration.\n"
        "You MUST follow these rules strictly:\n"
        "- You ONLY choose one exploration region label from the given list.\n"
        "- You MUST NOT output map coordinates (x, y, yaw).\n"
        "- You MUST NOT output velocity, turn angle, drive distance, or any chassis action.\n"
        "- You MUST NOT invent labels that are not in the candidate list.\n"
        "- Gray/unknown areas on the annotated map represent unexplored space.\n"
        "- Prefer regions with higher unknown gain, adequate clearance, and reasonable distance.\n"
        "- Do NOT treat path_checked=false as confirmed navigability.\n"
        "- Do NOT claim you saw specific objects (bottles, tables, kitchen) — only map metrics are provided.\n"
        "- Return ONLY valid JSON. No markdown. No extra text.\n"
        "\n"
        f"Target instruction: {inp.target_instruction}\n"
        f"Snapshot ID (echo exactly): {inp.snapshot_id}\n"
        f"Valid region labels ONLY: {label_list}\n"
        f"Robot pose: x={robot.get('x', 0):.3f}, y={robot.get('y', 0):.3f}, "
        f"yaw_deg={robot.get('yaw_deg', 0):.1f}\n"
        "\n"
        "Candidate regions:\n"
        f"{regions_block}\n"
        "\n"
        "Output exactly this JSON shape:\n"
        "{\n"
        f'  "snapshot_id": "{inp.snapshot_id}",\n'
        '  "selected_region": "<one of: '
        + label_list
        + ">\",\n"
        '  "fallback_regions": ["<optional other labels>"],\n'
        '  "confidence": 0.75,\n'
        '  "reason_code": "UNEXPLORED_REGION_PRIORITY",\n'
        '  "evidence": ["short metric-based reason 1", "short metric-based reason 2"]\n'
        "}\n"
    )


def extract_json_object(text: str) -> Dict[str, Any]:
    """Extract first JSON object from model response."""
    if not text or not text.strip():
        raise ValueError("QWEN_RESPONSE_EMPTY")
    cleaned = text.strip()
    if cleaned.startswith("```"):
        cleaned = re.sub(r"^```(?:json)?\s*", "", cleaned)
        cleaned = re.sub(r"\s*```$", "", cleaned)
    try:
        obj = json.loads(cleaned)
        if isinstance(obj, dict):
            return obj
    except json.JSONDecodeError:
        pass
    match = re.search(r"\{.*\}", cleaned, flags=re.DOTALL)
    if not match:
        raise ValueError("QWEN_RESPONSE_NOT_JSON")
    obj = json.loads(match.group(0))
    if not isinstance(obj, dict):
        raise ValueError("QWEN_RESPONSE_NOT_JSON")
    return obj


def _collect_forbidden_fields(obj: Dict[str, Any], prefix: str = "") -> List[str]:
    found: List[str] = []
    for key, value in obj.items():
        full = f"{prefix}.{key}" if prefix else key
        if key.lower() in FORBIDDEN_DECISION_FIELDS:
            found.append(full)
        if isinstance(value, dict):
            found.extend(_collect_forbidden_fields(value, full))
    return found


def audit_unsupported_visual_claims(evidence: Sequence[str]) -> List[str]:
    """Flag evidence that claims visual object detection without camera input."""
    flags: List[str] = []
    for line in evidence:
        lower = line.lower()
        for kw in UNSUPPORTED_VISUAL_CLAIM_KEYWORDS:
            if kw.lower() in lower or kw in line:
                flags.append(f"UNSUPPORTED_VISUAL_CLAIM: {line}")
                break
    return flags


def validate_qwen_decision(
    raw_response: str,
    expected_snapshot_id: str,
    valid_labels: Set[str],
) -> DecisionValidationResult:
    """Validate parsed Qwen JSON against snapshot constraints."""
    errors: List[str] = []
    parsed: Optional[Dict[str, Any]] = None
    unsupported: List[str] = []

    try:
        parsed = extract_json_object(raw_response)
    except ValueError as exc:
        code = str(exc)
        return DecisionValidationResult(
            decision_valid=False,
            errors=[code],
            parsed=None,
        )

    forbidden = _collect_forbidden_fields(parsed)
    if forbidden:
        errors.append("QWEN_FORBIDDEN_FIELD_PRESENT")
        errors.extend(forbidden)

    if str(parsed.get("snapshot_id", "")) != expected_snapshot_id:
        errors.append("QWEN_SNAPSHOT_ID_MISMATCH")

    selected = parsed.get("selected_region")
    if not isinstance(selected, str) or selected not in valid_labels:
        errors.append("QWEN_SELECTED_REGION_INVALID")

    fallbacks = parsed.get("fallback_regions", [])
    if fallbacks is None:
        fallbacks = []
    if not isinstance(fallbacks, list):
        errors.append("QWEN_FALLBACK_REGION_INVALID")
        fallbacks = []

    fb_seen: Set[str] = set()
    for fb in fallbacks:
        if not isinstance(fb, str) or fb not in valid_labels:
            errors.append("QWEN_FALLBACK_REGION_INVALID")
        elif fb in fb_seen:
            errors.append("QWEN_FALLBACK_DUPLICATE")
        elif isinstance(selected, str) and fb == selected:
            errors.append("QWEN_FALLBACK_REGION_INVALID")
        fb_seen.add(str(fb))

    confidence = parsed.get("confidence")
    try:
        conf_f = float(confidence)
        if conf_f < 0.0 or conf_f > 1.0:
            errors.append("QWEN_CONFIDENCE_INVALID")
    except (TypeError, ValueError):
        errors.append("QWEN_CONFIDENCE_INVALID")
        conf_f = 0.0

    reason_code = parsed.get("reason_code")
    if not isinstance(reason_code, str) or not reason_code.strip():
        errors.append("QWEN_REASON_CODE_MISSING")

    evidence = parsed.get("evidence", [])
    if isinstance(evidence, list):
        unsupported = audit_unsupported_visual_claims([str(e) for e in evidence])

    decision_valid = len(errors) == 0 and isinstance(selected, str)

    return DecisionValidationResult(
        decision_valid=decision_valid,
        errors=errors,
        parsed=parsed,
        unsupported_visual_claims=unsupported,
    )


def decision_from_validation(
    validation: DecisionValidationResult,
    expected_snapshot_id: str,
) -> RegionSelectionDecision:
    """Build decision object; null selected_region if invalid."""
    parsed = validation.parsed or {}
    selected: Optional[str] = None
    if validation.decision_valid:
        selected = str(parsed.get("selected_region"))

    fallbacks: List[str] = []
    raw_fb = parsed.get("fallback_regions", [])
    if isinstance(raw_fb, list):
        fallbacks = [str(x) for x in raw_fb]

    try:
        confidence = float(parsed.get("confidence", 0.0))
    except (TypeError, ValueError):
        confidence = 0.0

    return RegionSelectionDecision(
        snapshot_id=expected_snapshot_id,
        selected_region=selected,
        fallback_regions=fallbacks,
        confidence=confidence,
        reason_code=str(parsed.get("reason_code", "")),
        evidence=[str(e) for e in parsed.get("evidence", [])] if isinstance(parsed.get("evidence"), list) else [],
    )


def decision_to_dict(
    decision: RegionSelectionDecision,
    *,
    decision_valid: bool,
    motion_executed: bool = False,
    nav2_called: bool = False,
    validation_errors: Optional[List[str]] = None,
    unsupported_visual_claims: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """Serialize decision for logging and ROS topics."""
    return {
        "snapshot_id": decision.snapshot_id,
        "selected_region": decision.selected_region,
        "fallback_regions": list(decision.fallback_regions),
        "confidence": decision.confidence,
        "reason_code": decision.reason_code,
        "evidence": list(decision.evidence),
        "decision_valid": decision_valid,
        "motion_executed": motion_executed,
        "nav2_called": nav2_called,
        "validation_errors": list(validation_errors or []),
        "unsupported_visual_claims": list(unsupported_visual_claims or []),
    }


def input_to_dict(inp: RegionSelectionInput) -> Dict[str, Any]:
    d = asdict(inp)
    return d


def labels_from_input(inp: RegionSelectionInput) -> Set[str]:
    return {r.label for r in inp.regions}
