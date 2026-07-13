#!/usr/bin/env python3
"""Pure region selection core — no ROS, no motion, no API calls."""

from __future__ import annotations

import json
import math
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

ABSOLUTE_UNVISITED_KEYWORDS: Tuple[str, ...] = (
    "从未走过",
    "完全未访问",
    "never visited",
    "never been there",
    "完全没有轨迹",
)


def audit_trajectory_evidence_claims(
    evidence: Any,
    region_metrics: Optional[Dict[str, Dict[str, float]]] = None,
) -> List[str]:
    """Flag evidence that claims absolute unvisited status without metric support."""
    flags: List[str] = []
    if not isinstance(evidence, list) or not region_metrics:
        return flags
    for item in evidence:
        if not isinstance(item, dict):
            continue
        region = str(item.get("region", ""))
        metrics = region_metrics.get(region, {})
        novelty = float(metrics.get("trajectory_novelty_score", 1.0))
        density = float(metrics.get("trajectory_density_score", 0.0))
        factors = item.get("history_factors", [])
        if not isinstance(factors, list):
            continue
        for factor in factors:
            text = str(factor)
            lower = text.lower()
            if any(kw in text or kw in lower for kw in ABSOLUTE_UNVISITED_KEYWORDS):
                if novelty < 0.95 or density > 0.05:
                    flags.append(
                        f"UNSUPPORTED_TRAJECTORY_CLAIM region={region} factor={text}"
                    )
    return flags


REASON_CODES_SUGGESTED: Tuple[str, ...] = (
    "UNEXPLORED_REGION_PRIORITY",
    "HIGHER_UNKNOWN_GAIN",
    "BETTER_CLEARANCE",
    "CLOSER_SAFE_REGION",
    "LARGER_FRONTIER",
    "MERGED_REGION_PREFERENCE",
    "FALLBACK_BALANCE",
    "GEOMETRY_AND_VISUAL_CONTEXT_AGREE",
)

SELECTION_MODE_MAP_ONLY = "MAP_ONLY"
SELECTION_MODE_MAP_PLUS_VISUAL = "MAP_PLUS_VISUAL"


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
    track_id: str = ""
    stable: bool = False
    snapshot_eligible: bool = True
    blacklisted: bool = False
    geo_score: float = 0.0
    geo_rank: int = 0
    geo_score_before_trajectory: float = 0.0
    geo_score_after_trajectory: float = 0.0
    trajectory_novelty_score: float = 1.0
    trajectory_revisit_penalty: float = 0.0
    nearest_trajectory_distance_m: float = float("inf")
    nearby_trajectory_vertex_count: int = 0
    nearby_recent_trajectory_count: int = 0
    last_nearby_visit_age_s: float = float("inf")
    score_components: Dict[str, float] = field(default_factory=dict)
    penalty_components: Dict[str, float] = field(default_factory=dict)


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
    selection_mode: str = SELECTION_MODE_MAP_ONLY
    visual_context_id: str = ""
    visual_context_manifest: Optional[Dict[str, Any]] = None
    region_view_mapping: Optional[Dict[str, Any]] = None
    panorama_contact_sheet_file: str = ""
    decision_board_file: str = ""


@dataclass
class RegionSelectionDecision:
    snapshot_id: str
    selected_region: Optional[str]
    fallback_regions: List[str]
    confidence: float
    reason_code: str
    evidence: List[str]
    visual_context_id: str = ""


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
        track_id=str(data.get("track_id", "")),
        stable=bool(data.get("stable", False)),
        snapshot_eligible=bool(data.get("snapshot_eligible", True)),
        blacklisted=bool(data.get("blacklisted", False)),
        geo_score=float(data.get("geo_score", 0.0)),
        geo_rank=int(data.get("geo_rank", 0)),
        geo_score_before_trajectory=float(data.get("geo_score_before_trajectory", 0.0)),
        geo_score_after_trajectory=float(
            data.get("geo_score_after_trajectory", data.get("geo_score", 0.0))
        ),
        trajectory_novelty_score=float(data.get("trajectory_novelty_score", 1.0)),
        trajectory_revisit_penalty=float(data.get("trajectory_revisit_penalty", 0.0)),
        nearest_trajectory_distance_m=float(
            data.get("nearest_trajectory_distance_m", float("inf"))
        ),
        nearby_trajectory_vertex_count=int(data.get("nearby_trajectory_vertex_count", 0)),
        nearby_recent_trajectory_count=int(data.get("nearby_recent_trajectory_count", 0)),
        last_nearby_visit_age_s=float(data.get("last_nearby_visit_age_s", float("inf"))),
        score_components=dict(data.get("score_components", {})),
        penalty_components=dict(data.get("penalty_components", {})),
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


def resolve_selection_mode(
    inp: RegionSelectionInput,
    *,
    force_map_only: bool = False,
) -> str:
    if force_map_only:
        return SELECTION_MODE_MAP_ONLY
    manifest = inp.visual_context_manifest or {}
    if (
        inp.selection_mode == SELECTION_MODE_MAP_PLUS_VISUAL
        and inp.visual_context_id
        and manifest.get("capture_complete")
        and inp.region_view_mapping
        and inp.panorama_contact_sheet_file
    ):
        return SELECTION_MODE_MAP_PLUS_VISUAL
    return SELECTION_MODE_MAP_ONLY


def load_visual_context_bundle(
    manifest_path: Path,
    mapping_path: Optional[Path] = None,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    mp = Path(manifest_path).expanduser().resolve()
    if not mp.is_file():
        raise ValueError(f"VISUAL_MANIFEST_NOT_FOUND path={mp}")
    manifest = json.loads(mp.read_text(encoding="utf-8"))
    mapping: Dict[str, Any] = {}
    if mapping_path and Path(mapping_path).is_file():
        mapping = json.loads(Path(mapping_path).read_text(encoding="utf-8"))
    elif (mp.parent / "region_view_mapping.json").is_file():
        mapping = json.loads((mp.parent / "region_view_mapping.json").read_text(encoding="utf-8"))
    return manifest, mapping


def attach_visual_context_to_input(
    inp: RegionSelectionInput,
    manifest: Dict[str, Any],
    mapping: Dict[str, Any],
    *,
    contact_sheet_file: str = "",
    decision_board_file: str = "",
) -> RegionSelectionInput:
    inp.visual_context_id = str(manifest.get("visual_context_id", ""))
    inp.visual_context_manifest = manifest
    inp.region_view_mapping = mapping
    inp.panorama_contact_sheet_file = contact_sheet_file or str(
        manifest.get("contact_sheet_file", "")
    )
    inp.decision_board_file = decision_board_file or str(manifest.get("decision_board_file", ""))
    if manifest.get("capture_complete") and inp.visual_context_id:
        inp.selection_mode = SELECTION_MODE_MAP_PLUS_VISUAL
    else:
        inp.selection_mode = SELECTION_MODE_MAP_ONLY
    return inp


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
    """Build prompt for semantic re-ranking of algorithm-gated regions."""
    mode = resolve_selection_mode(inp)
    labels = [r.label for r in inp.regions]
    label_list = ", ".join(labels)

    region_lines: List[str] = []
    for r in sorted(inp.regions, key=lambda x: (x.geo_rank if x.geo_rank > 0 else 999, -x.geo_score)):
        region_lines.append(
            f"- Label {r.label}: geo_rank={r.geo_rank}, geo_score={r.geo_score:.3f}, "
            f"geo_before_traj={r.geo_score_before_trajectory:.3f}, "
            f"trajectory_novelty={r.trajectory_novelty_score:.3f}, "
            f"trajectory_revisit_penalty={r.trajectory_revisit_penalty:.3f}, "
            f"direction={r.direction}, distance_m={r.distance_m:.2f}, "
            f"unknown_gain_cells={r.unknown_gain_cells}, minimum_clearance_m={r.minimum_clearance_m:.2f}, "
            f"stable={r.stable}, track_id={r.track_id}, path_checked={r.path_checked}, "
            f"score_components={r.score_components}, penalties={r.penalty_components}"
        )

    robot = inp.robot_pose
    regions_block = "\n".join(region_lines)

    visual_block = ""
    mapping_block = ""
    if mode == SELECTION_MODE_MAP_PLUS_VISUAL:
        mapping = inp.region_view_mapping or {}
        assoc_lines: List[str] = []
        for item in mapping.get("associations", []):
            if not isinstance(item, dict):
                continue
            assoc_lines.append(
                f"- region {item.get('region_label')}: primary_view={item.get('primary_view_id')} "
                f"secondary={item.get('secondary_view_ids', [])} "
                f"bearing_deg={item.get('region_global_bearing_deg')}"
            )
        mapping_block = "\n".join(assoc_lines) if assoc_lines else "(no mappings)"
        visual_block = (
            f"Selection mode: MAP_PLUS_VISUAL\n"
            f"Visual context ID (echo exactly): {inp.visual_context_id}\n"
            f"Panorama contact sheet: {inp.panorama_contact_sheet_file}\n"
            f"Decision board: {inp.decision_board_file}\n"
            "Map + visual rules:\n"
            "1. Map evidence covers distance, clearance, unknown gain, trajectory history, stability.\n"
            "2. 360° images show visible scene per mapped view direction.\n"
            "3. Each region is bound to legal view_id via region_view_mapping.\n"
            "4. Reference ONLY view_ids present in the manifest.\n"
            "5. Do NOT guess which image matches a map direction.\n"
            "6. Do NOT claim objects not visible in provided images.\n"
            "7. Do NOT describe walls or closed obstacles as open paths.\n"
            "8. Do NOT override low geo_score or safety risks with visual interest.\n"
            "9. When map geometry is similar, prefer semantically relevant regions.\n"
            "10. path_checked=false means reachability is NOT confirmed.\n"
            "11. Echo snapshot_id and visual_context_id exactly.\n"
            "12. Rank ALL valid input labels exactly once.\n"
            "13. Do NOT output coordinates, pixel coords, velocities, angles, or motion commands.\n"
            "\n"
            f"Region-view mapping:\n{mapping_block}\n"
        )
    else:
        visual_block = (
            "Selection mode: MAP_ONLY\n"
            "No visual context is provided — do NOT claim seeing tables, rooms, bottles.\n"
        )

    return (
        "You are a candidate region semantic re-ranker for mobile robot exploration.\n"
        "The input regions already passed algorithm hard gates.\n"
        "geo_score and geo_rank are geometry-based scores from the map pipeline.\n"
        "Rules:\n"
        "1. Re-rank ONLY the given labels; do not invent labels.\n"
        "2. Do NOT output coordinates, velocities, or motion commands.\n"
        "3. Do NOT select snapshot_eligible=false or blacklisted=true regions.\n"
        "4. Do NOT claim path_checked=false regions are reachable.\n"
        "5. Do NOT choose solely by unknown_gain maximum.\n"
        "6. Balance distance, clearance, stability, history, trajectory novelty, and task relevance.\n"
        "8. Return ALL candidate labels in ranked_regions exactly once.\n"
        "9. selected_region MUST equal ranked_regions[0].\n"
        "10. fallback_regions MUST match ranked_regions[1:] in order.\n"
        "Trajectory legend on annotated map:\n"
        "1. TRAVELED PATH = robot actual traveled trajectory.\n"
        "2. VISITED CORRIDOR = corridor area already covered by trajectory.\n"
        "3. OBSERVATION POSE = positions where robot completed observation or scan.\n"
        "4. trajectory_novelty_score higher means fewer prior visits near candidate.\n"
        "5. trajectory_revisit_penalty higher means more repeated nearby visits.\n"
        "Region selection with trajectory history:\n"
        "1. When safety, stability, unknown gain, and distance are similar, prefer regions "
        "with lower trajectory coverage and fewer recent visits.\n"
        "2. Do NOT pick a region only because it has no trajectory if clearance, stability, "
        "or geometry score is worse.\n"
        "3. Do NOT reject a region only because it is near old trajectory.\n"
        "4. geo_score already includes trajectory novelty and revisit penalty.\n"
        "5. Qwen performs limited semantic re-ranking; final decision comes from fusion.\n"
        f"{visual_block}"
        "Return ONLY valid JSON.\n"
        "\n"
        f"Target instruction: {inp.target_instruction}\n"
        f"Snapshot ID (echo exactly): {inp.snapshot_id}\n"
        f"Valid labels ONLY: {label_list}\n"
        f"Robot pose: x={robot.get('x', 0):.3f}, y={robot.get('y', 0):.3f}, yaw_deg={robot.get('yaw_deg', 0):.1f}\n"
        "\n"
        "Candidate regions:\n"
        f"{regions_block}\n"
        "\n"
        "Output JSON shape:\n"
        "{\n"
        f'  "snapshot_id": "{inp.snapshot_id}",\n'
        + (
            f'  "visual_context_id": "{inp.visual_context_id}",\n'
            if mode == SELECTION_MODE_MAP_PLUS_VISUAL
            else ""
        )
        + '  "ranked_regions": ["<all labels ordered best-first>"],\n'
        '  "selected_region": "<ranked_regions[0]>",\n'
        '  "fallback_regions": ["<ranked_regions[1:]>"],\n'
        '  "confidence": 0.78,\n'
        '  "reason_code": "BALANCED_EXPLORATION_VALUE",\n'
        '  "evidence": [{"region":"B","map_factors":["..."],"history_factors":["..."],"visual_factors":[{"view_ids":["VIEW_135"],"observation":"..."}],"risks":["path_checked=false"]}]\n'
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


def audit_evidence_visual_claims(evidence: Any) -> List[str]:
    flags: List[str] = []
    if not isinstance(evidence, list):
        return flags
    for item in evidence:
        if isinstance(item, str):
            flags.extend(audit_unsupported_visual_claims([item]))
        elif isinstance(item, dict):
            vf = item.get("visual_factors", [])
            if isinstance(vf, list):
                flags.extend(audit_unsupported_visual_claims([str(x) for x in vf]))
            for key in ("map_factors", "risks"):
                vals = item.get(key, [])
                if isinstance(vals, list):
                    flags.extend(audit_unsupported_visual_claims([str(x) for x in vals if "看到" in str(x) or "bottle" in str(x).lower()]))
    return flags


def _region_view_allowlist(
    region_view_mapping: Optional[Dict[str, Any]],
) -> Dict[str, Set[str]]:
    allow: Dict[str, Set[str]] = {}
    if not region_view_mapping:
        return allow
    for item in region_view_mapping.get("associations", []):
        if not isinstance(item, dict):
            continue
        label = str(item.get("region_label", ""))
        views: Set[str] = set()
        pv = str(item.get("primary_view_id", ""))
        if pv:
            views.add(pv)
        for sv in item.get("secondary_view_ids", []):
            views.add(str(sv))
        if label:
            allow[label] = views
    return allow


def _valid_view_ids_from_manifest(
    visual_context_manifest: Optional[Dict[str, Any]],
) -> Set[str]:
    valid: Set[str] = set()
    if not visual_context_manifest:
        return valid
    for frame in visual_context_manifest.get("frames", []):
        if not isinstance(frame, dict):
            continue
        if frame.get("valid") and frame.get("view_id"):
            valid.add(str(frame["view_id"]))
    return valid


def audit_qwen_visual_evidence(
    evidence: Any,
    valid_labels: Set[str],
    *,
    expected_visual_context_id: str = "",
    region_view_mapping: Optional[Dict[str, Any]] = None,
    visual_context_manifest: Optional[Dict[str, Any]] = None,
    visual_mode: bool = False,
) -> List[str]:
    errors: List[str] = []
    if not visual_mode:
        return errors
    if not expected_visual_context_id:
        errors.append("QWEN_VISUAL_CONTEXT_ID_MISMATCH")
        return errors

    allow_by_region = _region_view_allowlist(region_view_mapping)
    valid_views = _valid_view_ids_from_manifest(visual_context_manifest)

    if not isinstance(evidence, list):
        return errors

    for item in evidence:
        if not isinstance(item, dict):
            continue
        region = str(item.get("region", ""))
        if region not in valid_labels:
            errors.append("QWEN_EVIDENCE_REGION_INVALID")
            continue
        allowed_views = allow_by_region.get(region, set())
        vf = item.get("visual_factors", [])
        if not isinstance(vf, list):
            continue
        for vf_item in vf:
            if isinstance(vf_item, str):
                errors.append("QWEN_EVIDENCE_VIEW_INVALID")
                continue
            if not isinstance(vf_item, dict):
                errors.append("QWEN_EVIDENCE_VIEW_INVALID")
                continue
            view_ids = vf_item.get("view_ids", [])
            if not isinstance(view_ids, list) or not view_ids:
                errors.append("QWEN_EVIDENCE_VIEW_MISSING")
                continue
            for vid in view_ids:
                vid_s = str(vid)
                if vid_s not in valid_views:
                    errors.append("QWEN_EVIDENCE_VIEW_MISSING")
                elif vid_s not in allowed_views:
                    errors.append("QWEN_EVIDENCE_VIEW_NOT_MAPPED")
    return errors


def validate_qwen_decision(
    raw_response: str,
    expected_snapshot_id: str,
    valid_labels: Set[str],
    geo_scores: Optional[Dict[str, float]] = None,
    region_trajectory_metrics: Optional[Dict[str, Dict[str, float]]] = None,
    *,
    expected_visual_context_id: str = "",
    visual_context_manifest: Optional[Dict[str, Any]] = None,
    region_view_mapping: Optional[Dict[str, Any]] = None,
    visual_mode: bool = False,
) -> DecisionValidationResult:
    """Validate parsed Qwen JSON against snapshot constraints."""
    errors: List[str] = []
    parsed: Optional[Dict[str, Any]] = None
    unsupported: List[str] = []

    try:
        parsed = extract_json_object(raw_response)
    except ValueError as exc:
        return DecisionValidationResult(decision_valid=False, errors=[str(exc)], parsed=None)

    forbidden = _collect_forbidden_fields(parsed)
    if forbidden:
        errors.append("QWEN_FORBIDDEN_FIELD_PRESENT")
        errors.extend(forbidden)

    if str(parsed.get("snapshot_id", "")) != expected_snapshot_id:
        errors.append("QWEN_SNAPSHOT_ID_MISMATCH")

    if visual_mode:
        if not expected_visual_context_id:
            errors.append("QWEN_VISUAL_CONTEXT_ID_MISMATCH")
        elif str(parsed.get("visual_context_id", "")) != expected_visual_context_id:
            errors.append("QWEN_VISUAL_CONTEXT_ID_MISMATCH")
    elif parsed.get("visual_context_id"):
        errors.append("QWEN_VISUAL_CONTEXT_ID_MISMATCH")

    ranked = parsed.get("ranked_regions")
    if isinstance(ranked, list) and ranked:
        if set(ranked) != valid_labels:
            errors.append("QWEN_RANKING_INCOMPLETE")
        if len(ranked) != len(set(ranked)):
            errors.append("QWEN_RANKING_DUPLICATE")
        selected = parsed.get("selected_region")
        if ranked and selected != ranked[0]:
            errors.append("QWEN_SELECTED_NOT_TOP_RANK")
        fallbacks = parsed.get("fallback_regions", [])
        if list(fallbacks) != list(ranked[1:]):
            errors.append("QWEN_FALLBACK_REGION_INVALID")
    else:
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
        if float(confidence) < 0.0 or float(confidence) > 1.0:
            errors.append("QWEN_CONFIDENCE_INVALID")
    except (TypeError, ValueError):
        errors.append("QWEN_CONFIDENCE_INVALID")

    reason_code = parsed.get("reason_code")
    if not isinstance(reason_code, str) or not reason_code.strip():
        errors.append("QWEN_REASON_CODE_MISSING")

    evidence = parsed.get("evidence", [])
    unsupported = audit_evidence_visual_claims(evidence)
    traj_flags = audit_trajectory_evidence_claims(evidence, region_trajectory_metrics)
    unsupported.extend(traj_flags)
    visual_errors = audit_qwen_visual_evidence(
        evidence,
        valid_labels,
        expected_visual_context_id=expected_visual_context_id,
        region_view_mapping=region_view_mapping,
        visual_context_manifest=visual_context_manifest,
        visual_mode=visual_mode,
    )
    for code in visual_errors:
        if code not in errors:
            errors.append(code)
    if unsupported:
        errors.append("QWEN_UNSUPPORTED_VISUAL_CLAIM")

    decision_valid = len(errors) == 0
    return DecisionValidationResult(
        decision_valid=decision_valid,
        errors=errors,
        parsed=parsed,
        unsupported_visual_claims=unsupported,
    )


QWEN_RANK_SCORES = [1.0, 0.65, 0.35, 0.20, 0.10]


def qwen_rank_score(rank_index: int) -> float:
    if rank_index < len(QWEN_RANK_SCORES):
        return QWEN_RANK_SCORES[rank_index]
    return max(0.05, QWEN_RANK_SCORES[-1] - 0.05 * (rank_index - len(QWEN_RANK_SCORES) + 1))


def fuse_geometric_and_qwen_ranking(
    regions: Sequence[RegionCandidate],
    parsed: Dict[str, Any],
    cfg: Dict[str, Any],
    *,
    has_visual_evidence: bool = False,
) -> Dict[str, Any]:
    fcfg = cfg.get("decision_fusion", {})
    if has_visual_evidence:
        gw = float(fcfg.get("visual_geo_weight", 0.65))
        qw = float(fcfg.get("visual_qwen_weight", 0.35))
    else:
        gw = float(fcfg.get("map_only_geo_weight", 0.85))
        qw = float(fcfg.get("map_only_qwen_weight", 0.15))

    min_geo = float(fcfg.get("reject_if_geo_score_below", 0.40))
    max_gap = float(fcfg.get("max_allowed_geo_gap_without_visual_evidence", 0.20))

    eligible = [
        r
        for r in regions
        if r.stable and r.snapshot_eligible and not r.blacklisted and r.geo_score >= min_geo
    ]
    if not eligible:
        eligible = list(regions)

    label_geo = {r.label: r.geo_score for r in eligible}
    ranked = parsed.get("ranked_regions") or []
    qwen_scores: Dict[str, float] = {}
    for i, lbl in enumerate(ranked):
        if isinstance(lbl, str):
            qwen_scores[lbl] = qwen_rank_score(i)

    fusion: List[Dict[str, Any]] = []
    for r in eligible:
        qs = qwen_scores.get(r.label, 0.0)
        final = gw * r.geo_score + qw * qs
        fusion.append({"label": r.label, "geo_score": r.geo_score, "qwen_rank_score": qs, "final_score": final})

    fusion.sort(key=lambda x: (-x["final_score"], x["label"]))
    best_geo_label = max(eligible, key=lambda r: r.geo_score).label
    qwen_pick = str(parsed.get("selected_region", ranked[0] if ranked else ""))
    algorithm_final = fusion[0]["label"]
    decision_source = "FUSION_BLEND"
    override_reason = ""

    if label_geo.get(qwen_pick, 0.0) < min_geo:
        algorithm_final = best_geo_label
        decision_source = "GEOMETRIC_MIN_SCORE_OVERRIDE"
        override_reason = "LOW_GEO_SCORE"
    elif (
        not has_visual_evidence
        and label_geo.get(best_geo_label, 0.0) - label_geo.get(qwen_pick, 0.0) > max_gap
    ):
        algorithm_final = best_geo_label
        decision_source = "GEOMETRIC_SAFETY_OVERRIDE"
        override_reason = "QWEN_OVERRIDE_REJECTED_LARGE_GEO_GAP"

    return {
        "fusion_scores": fusion,
        "qwen_recommended_region": qwen_pick,
        "algorithm_final_region": algorithm_final,
        "decision_source": decision_source,
        "override_reason": override_reason,
        "geo_weight": gw,
        "qwen_weight": qw,
        "ranked_regions": [str(x) for x in ranked if isinstance(x, str)],
    }


def revalidate_decision(
    snapshot_at_request: Dict[str, Any],
    snapshot_now: Dict[str, Any],
    selected_label: Optional[str],
    track_by_label: Dict[str, Dict[str, Any]],
    cfg: Dict[str, Any],
) -> List[str]:
    rcfg = cfg.get("decision_revalidation", {})
    errors: List[str] = []
    req_time = float(snapshot_at_request.get("map_stamp", 0))
    now_time = float(snapshot_now.get("map_stamp", 0))
    age = abs(now_time - req_time)
    if age > float(rcfg.get("max_snapshot_age_s", 20.0)):
        errors.append("DECISION_SNAPSHOT_STALE")

    rp0 = snapshot_at_request.get("robot_pose", {})
    rp1 = snapshot_now.get("robot_pose", {})
    trans = math.hypot(float(rp1.get("x", 0)) - float(rp0.get("x", 0)), float(rp1.get("y", 0)) - float(rp0.get("y", 0)))
    if trans > float(rcfg.get("max_robot_translation_m", 0.15)):
        errors.append("DECISION_ROBOT_MOVED")
    yaw0 = float(rp0.get("yaw_deg", 0))
    yaw1 = float(rp1.get("yaw_deg", 0))
    yaw_diff = abs(yaw1 - yaw0) % 360
    yaw_diff = yaw_diff if yaw_diff <= 180 else 360 - yaw_diff
    if yaw_diff > float(rcfg.get("max_robot_yaw_change_deg", 20.0)):
        errors.append("DECISION_ROBOT_ROTATED")

    k0 = int(snapshot_at_request.get("map_metadata", {}).get("width", 0)) * int(snapshot_at_request.get("map_metadata", {}).get("height", 0))
    k1 = int(snapshot_now.get("map_metadata", {}).get("width", 0)) * int(snapshot_now.get("map_metadata", {}).get("height", 0))
    if k0 > 0 and abs(k1 - k0) / k0 > float(rcfg.get("max_map_cell_change_ratio", 0.10)):
        errors.append("DECISION_MAP_CHANGED")

    if selected_label and bool(rcfg.get("require_selected_track_still_present", True)):
        track = track_by_label.get(selected_label)
        if track is None:
            errors.append("DECISION_REGION_DISAPPEARED")
        else:
            if not bool(track.get("stable", False)):
                errors.append("DECISION_REGION_NO_LONGER_STABLE")
            if not bool(track.get("snapshot_eligible", False)):
                errors.append("DECISION_REGION_NO_LONGER_ELIGIBLE")
    return errors


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
        visual_context_id=str(parsed.get("visual_context_id", "")),
    )


def decision_to_dict(
    decision: RegionSelectionDecision,
    *,
    decision_valid: bool,
    motion_executed: bool = False,
    nav2_called: bool = False,
    validation_errors: Optional[List[str]] = None,
    unsupported_visual_claims: Optional[List[str]] = None,
    decision_id: str = "",
    fusion: Optional[Dict[str, Any]] = None,
    map_fingerprint: str = "",
    trajectory_revision: int = 0,
    contract_version: str = "",
    qwen_decision_schema_version: str = "",
) -> Dict[str, Any]:
    """Serialize decision for logging and ROS topics."""
    from src.planning.exploration_contracts import (  # noqa: WPS433
        EXPLORATION_CONTRACT_VERSION,
        REGION_DECISION_SCHEMA_VERSION,
    )

    fusion = fusion or {}
    return {
        "contract_version": contract_version or EXPLORATION_CONTRACT_VERSION,
        "qwen_decision_schema_version": qwen_decision_schema_version or REGION_DECISION_SCHEMA_VERSION,
        "schema_version": qwen_decision_schema_version or REGION_DECISION_SCHEMA_VERSION,
        "decision_id": decision_id,
        "snapshot_id": decision.snapshot_id,
        "visual_context_id": decision.visual_context_id,
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
        "map_fingerprint": map_fingerprint,
        "trajectory_revision": trajectory_revision,
        "qwen_recommended_region": fusion.get("qwen_recommended_region"),
        "algorithm_final_region": fusion.get("algorithm_final_region"),
        "decision_source": fusion.get("decision_source"),
        "ranked_regions": fusion.get("ranked_regions") or [],
    }


def input_to_dict(inp: RegionSelectionInput) -> Dict[str, Any]:
    d = asdict(inp)
    return d


def labels_from_input(inp: RegionSelectionInput) -> Set[str]:
    return {r.label for r in inp.regions}
