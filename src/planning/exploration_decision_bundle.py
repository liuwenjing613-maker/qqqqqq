#!/usr/bin/env python3
"""Exploration decision bundle validation — pure Python, no ROS or motion interfaces."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from src.planning.exploration_contracts import (
    BUNDLE_SCHEMA_VERSION,
    EXPLORATION_CONTRACT_VERSION,
    REGION_DECISION_SCHEMA_VERSION,
    REGION_GEOMETRY_SCHEMA_VERSION,
    REGION_SNAPSHOT_SCHEMA_VERSION,
    SAFE_VIEWPOINT_SCHEMA_VERSION,
    TRAJECTORY_SCHEMA_VERSION,
    VISUAL_CONTEXT_SCHEMA_VERSION,
    build_map_fingerprint_from_snapshot,
    build_region_geometry_fingerprint_from_entry,
    validate_contract_config,
)

MIN_REGION_GEO_SCORE = 0.40


@dataclass
class BundleValidationError:
    code: str
    detail: str = ""


@dataclass
class BundleValidationResult:
    valid: bool
    errors: List[BundleValidationError] = field(default_factory=list)

    @property
    def error_codes(self) -> List[str]:
        return [e.code for e in self.errors]


@dataclass
class ExplorationDecisionBundle:
    contract_version: str
    bundle_schema_version: str
    bundle_id: str
    created_at: str
    snapshot_id: str
    snapshot_schema_version: str
    snapshot_capture_time: str
    snapshot_age_s: float
    map_frame: str
    map_stamp: float
    map_fingerprint: str
    map_metadata_fingerprint: str
    map_data_fingerprint: str
    trajectory_session_id: str
    trajectory_revision: int
    trajectory_schema_version: str
    selection_mode: str
    visual_context_id: Optional[str]
    visual_context_schema_version: Optional[str]
    visual_context_complete: bool
    qwen_decision_id: str
    qwen_decision_schema_version: str
    qwen_recommended_region: Optional[str]
    algorithm_final_region: Optional[str]
    decision_source: str
    decision_revalidation_passed: bool
    region_label: str
    internal_region_id: str
    track_id: str
    region_geometry_fingerprint: str
    region_geometry_schema_version: str
    region_stable: bool
    region_snapshot_eligible: bool
    region_blacklisted: bool
    region_geo_score: float
    configured_selection_strategy: str = "CANDIDATE_RANKING"
    effective_selection_strategy: str = "CANDIDATE_RANKING"
    region_source: str = "ALGORITHM_CANDIDATE"
    qwen_global_proposal_id: Optional[str] = None
    qwen_global_proposal_rank: Optional[int] = None
    proposal_validation_passed: bool = False
    proposal_validation_errors: List[str] = field(default_factory=list)
    map_render_metadata_fingerprint: str = ""
    path_checked: bool = False
    reachable: Optional[bool] = None
    safe_viewpoint_request_ready: bool = False
    validation_errors: List[str] = field(default_factory=list)


@dataclass
class SafeViewpointRequestEnvelope:
    bundle_id: str
    contract_version: str
    snapshot_id: str
    map_fingerprint: str
    region_label: str
    internal_region_id: str
    track_id: str
    region_geometry_fingerprint: str
    robot_pose: Dict[str, float]
    trajectory_session_id: str
    trajectory_revision: int
    selected_region_geometry: Dict[str, Any]
    safe_viewpoint_config_version: str
    path_checked: bool = False
    reachable: Optional[bool] = None
    request_ready: bool = True


def generate_bundle_id(
    snapshot_id: str,
    region_label: str,
    created_at: Optional[datetime] = None,
) -> str:
    ts = (created_at or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%S")
    parts = snapshot_id.split("_")
    short_id = parts[-1] if len(parts) >= 2 else snapshot_id[-4:]
    return f"EDB_{ts}_{short_id}_{region_label}"


def _add_error(errors: List[BundleValidationError], code: str, detail: str = "") -> None:
    errors.append(BundleValidationError(code=code, detail=detail))


def _cfg_bundle(cfg: Mapping[str, Any]) -> Dict[str, Any]:
    return dict(cfg.get("bundle_validation", {}))


def _parse_iso8601(value: str) -> Optional[datetime]:
    if not value:
        return None
    text = value.replace("Z", "+00:00")
    try:
        return datetime.fromisoformat(text)
    except ValueError:
        return None


def _snapshot_age_s(
    capture_time: str,
    current_time: datetime,
) -> float:
    captured = _parse_iso8601(capture_time)
    if captured is None:
        return float("inf")
    if captured.tzinfo is None:
        captured = captured.replace(tzinfo=timezone.utc)
    now = current_time if current_time.tzinfo else current_time.replace(tzinfo=timezone.utc)
    return max(0.0, (now - captured).total_seconds())


def _check_contract_versions(
    bundle: ExplorationDecisionBundle,
    sources: Mapping[str, Any],
    cfg: Mapping[str, Any],
    errors: List[BundleValidationError],
) -> None:
    expected_contract = str(cfg.get("contracts", {}).get("exploration_contract_version", EXPLORATION_CONTRACT_VERSION))
    if bundle.contract_version != expected_contract:
        _add_error(errors, "BUNDLE_CONTRACT_VERSION_MISMATCH", bundle.contract_version)
    supported = {BUNDLE_SCHEMA_VERSION, REGION_SNAPSHOT_SCHEMA_VERSION, REGION_GEOMETRY_SCHEMA_VERSION}
    for name, version in (
        ("bundle_schema_version", bundle.bundle_schema_version),
        ("snapshot_schema_version", bundle.snapshot_schema_version),
        ("region_geometry_schema_version", bundle.region_geometry_schema_version),
    ):
        if version not in supported:
            _add_error(errors, "BUNDLE_SCHEMA_VERSION_UNSUPPORTED", f"{name}={version}")
    snapshot = sources.get("region_snapshot") or {}
    if snapshot.get("contract_version") and snapshot.get("contract_version") != expected_contract:
        _add_error(errors, "BUNDLE_CONTRACT_VERSION_MISMATCH", "region_snapshot")


def validate_exploration_decision_bundle(
    bundle: ExplorationDecisionBundle,
    *,
    region_snapshot: Mapping[str, Any],
    region_geometry: Mapping[str, Any],
    qwen_decision: Mapping[str, Any],
    fusion: Mapping[str, Any],
    visual_manifest: Optional[Mapping[str, Any]] = None,
    region_view_mapping: Optional[Mapping[str, Any]] = None,
    cfg: Mapping[str, Any],
    current_time: Optional[datetime] = None,
    map_data: Optional[Sequence[int]] = None,
) -> BundleValidationResult:
    """Validate cross-stage consistency in fixed order."""
    errors: List[BundleValidationError] = []
    bcfg = _cfg_bundle(cfg)
    now = current_time or datetime.now(timezone.utc)

    _check_contract_versions(bundle, {"region_snapshot": region_snapshot}, cfg, errors)

    snapshot_id = str(bundle.snapshot_id)
    for label, obj in (
        ("region_snapshot", region_snapshot),
        ("region_geometry", region_geometry),
        ("qwen_decision", qwen_decision),
    ):
        if obj.get("snapshot_id") and str(obj.get("snapshot_id")) != snapshot_id:
            _add_error(errors, "BUNDLE_SNAPSHOT_ID_MISMATCH", label)
    if visual_manifest and visual_manifest.get("snapshot_id"):
        if str(visual_manifest.get("snapshot_id")) != snapshot_id:
            _add_error(errors, "BUNDLE_SNAPSHOT_ID_MISMATCH", "visual_manifest")
    if region_view_mapping and region_view_mapping.get("snapshot_id"):
        if str(region_view_mapping.get("snapshot_id")) != snapshot_id:
            _add_error(errors, "BUNDLE_SNAPSHOT_ID_MISMATCH", "region_view_mapping")

    algorithm_final = bundle.algorithm_final_region or fusion.get("algorithm_final_region")
    if not algorithm_final:
        _add_error(errors, "BUNDLE_FINAL_REGION_MISSING")
    else:
        if bundle.region_label != algorithm_final:
            _add_error(errors, "BUNDLE_REGION_LABEL_MISMATCH", "bundle.region_label")
        geo_regions = region_geometry.get("regions", {})
        if algorithm_final not in geo_regions:
            _add_error(errors, "BUNDLE_REGION_LABEL_MISMATCH", "region_geometry missing label")
        ranked = qwen_decision.get("ranked_regions") or []
        if ranked and algorithm_final not in ranked:
            _add_error(errors, "BUNDLE_REGION_LABEL_MISMATCH", "ranked_regions")

    geo_entry = region_geometry.get("regions", {}).get(bundle.region_label, {})
    if geo_entry:
        if str(geo_entry.get("internal_region_id", "")) != bundle.internal_region_id:
            _add_error(errors, "BUNDLE_INTERNAL_REGION_ID_MISMATCH")
        if str(geo_entry.get("track_id", "")) != bundle.track_id:
            _add_error(errors, "BUNDLE_TRACK_ID_MISMATCH")

    if map_data is not None and bool(bcfg.get("require_map_fingerprint", True)):
        computed = build_map_fingerprint_from_snapshot(region_snapshot, map_data, cfg=cfg)
        if bundle.map_fingerprint != computed["map_fingerprint"]:
            _add_error(errors, "BUNDLE_MAP_FINGERPRINT_MISMATCH")
        if bundle.map_metadata_fingerprint != computed["map_metadata_fingerprint"]:
            _add_error(errors, "BUNDLE_MAP_METADATA_MISMATCH")
        if bundle.map_data_fingerprint != computed["map_data_fingerprint"]:
            _add_error(errors, "BUNDLE_MAP_DATA_MISMATCH")
    else:
        snap_fp = region_snapshot.get("map_fingerprint")
        if snap_fp and snap_fp != bundle.map_fingerprint:
            _add_error(errors, "BUNDLE_MAP_FINGERPRINT_MISMATCH", "snapshot stored fp")

    if bool(bcfg.get("require_region_geometry_fingerprint", True)) and geo_entry:
        computed_geo = build_region_geometry_fingerprint_from_entry(
            snapshot_id, bundle.region_label, geo_entry, cfg=cfg
        )
        stored = str(geo_entry.get("region_geometry_fingerprint", bundle.region_geometry_fingerprint))
        if bundle.region_geometry_fingerprint != computed_geo or stored != computed_geo:
            _add_error(errors, "BUNDLE_REGION_GEOMETRY_FINGERPRINT_MISMATCH")

    if bool(bcfg.get("require_trajectory_revision_match", True)):
        snap_rev = int(region_snapshot.get("trajectory_revision", -1))
        if bundle.trajectory_revision != snap_rev:
            _add_error(errors, "BUNDLE_TRAJECTORY_REVISION_MISMATCH")
        if qwen_decision.get("trajectory_revision") is not None:
            if int(qwen_decision.get("trajectory_revision")) != bundle.trajectory_revision:
                _add_error(errors, "BUNDLE_TRAJECTORY_REVISION_MISMATCH", "qwen_decision")

    selection_mode = str(bundle.selection_mode or "MAP_ONLY").upper()
    if selection_mode == "MAP_PLUS_VISUAL":
        if bool(bcfg.get("require_visual_context_for_visual_mode", True)):
            if not bundle.visual_context_id:
                _add_error(errors, "BUNDLE_VISUAL_CONTEXT_REQUIRED")
            if not bundle.visual_context_complete:
                _add_error(errors, "BUNDLE_VISUAL_CONTEXT_INCOMPLETE")
            if visual_manifest:
                if str(visual_manifest.get("visual_context_id", "")) != str(bundle.visual_context_id or ""):
                    _add_error(errors, "BUNDLE_VISUAL_CONTEXT_ID_MISMATCH")
                if not bool(visual_manifest.get("capture_complete", False)):
                    _add_error(errors, "BUNDLE_VISUAL_CONTEXT_INCOMPLETE")
            mapping = region_view_mapping or {}
            assoc = mapping.get("associations") or visual_manifest.get("region_view_associations") if visual_manifest else []
            has_primary = False
            for item in assoc or []:
                if str(item.get("region_label", "")) == bundle.region_label and item.get("primary_view_id"):
                    has_primary = True
                    break
            if not has_primary:
                _add_error(errors, "BUNDLE_REGION_VIEW_MAPPING_MISSING")

    if bool(bcfg.get("require_region_stable", True)) and not bundle.region_stable:
        _add_error(errors, "BUNDLE_REGION_UNSTABLE")
    if bool(bcfg.get("require_region_snapshot_eligible", True)) and not bundle.region_snapshot_eligible:
        _add_error(errors, "BUNDLE_REGION_NOT_ELIGIBLE")
    if bool(bcfg.get("reject_blacklisted_region", True)) and bundle.region_blacklisted:
        _add_error(errors, "BUNDLE_REGION_BLACKLISTED")
    if not math.isfinite(bundle.region_geo_score):
        _add_error(errors, "BUNDLE_REGION_GEO_SCORE_INVALID")
    elif bundle.region_geo_score < MIN_REGION_GEO_SCORE:
        _add_error(errors, "BUNDLE_REGION_GEO_SCORE_INVALID", "below minimum")
    if bool(bcfg.get("require_decision_revalidation", True)) and not bundle.decision_revalidation_passed:
        _add_error(errors, "BUNDLE_DECISION_REVALIDATION_FAILED")

    age = _snapshot_age_s(bundle.snapshot_capture_time, now)
    max_age = float(bcfg.get("max_snapshot_age_s", 20.0))
    if age > max_age:
        _add_error(errors, "BUNDLE_SNAPSHOT_STALE", f"age={age:.1f}s")

    if bundle.path_checked:
        _add_error(errors, "BUNDLE_PREMATURE_REACHABILITY_CLAIM", "path_checked true")
    if bundle.reachable is True:
        nav2_called = bool(qwen_decision.get("nav2_called", False))
        if not nav2_called:
            _add_error(errors, "BUNDLE_PREMATURE_REACHABILITY_CLAIM", "reachable true without nav2")

    return BundleValidationResult(valid=len(errors) == 0, errors=errors)


def build_exploration_decision_bundle(
    *,
    region_snapshot: Mapping[str, Any],
    region_geometry: Mapping[str, Any],
    qwen_decision: Mapping[str, Any],
    fusion: Mapping[str, Any],
    selection_mode: str,
    decision_revalidation_passed: bool,
    visual_manifest: Optional[Mapping[str, Any]] = None,
    cfg: Mapping[str, Any],
    created_at: Optional[datetime] = None,
    current_time: Optional[datetime] = None,
) -> ExplorationDecisionBundle:
    algorithm_final = str(fusion.get("algorithm_final_region", ""))
    geo_entry = region_geometry.get("regions", {}).get(algorithm_final, {})
    meta = region_snapshot.get("map_metadata", {})
    created = created_at or datetime.now(timezone.utc)
    capture_time = str(region_snapshot.get("capture_time", ""))
    now = current_time or created
    age = _snapshot_age_s(capture_time, now)

    return ExplorationDecisionBundle(
        contract_version=str(cfg.get("contracts", {}).get("exploration_contract_version", EXPLORATION_CONTRACT_VERSION)),
        bundle_schema_version=BUNDLE_SCHEMA_VERSION,
        bundle_id=generate_bundle_id(str(region_snapshot.get("snapshot_id", "")), algorithm_final, created),
        created_at=created.isoformat(),
        snapshot_id=str(region_snapshot.get("snapshot_id", "")),
        snapshot_schema_version=str(region_snapshot.get("snapshot_schema_version", REGION_SNAPSHOT_SCHEMA_VERSION)),
        snapshot_capture_time=capture_time,
        snapshot_age_s=age,
        map_frame=str(meta.get("frame_id", "map")),
        map_stamp=float(region_snapshot.get("map_stamp", 0.0)),
        map_fingerprint=str(region_snapshot.get("map_fingerprint", "")),
        map_metadata_fingerprint=str(region_snapshot.get("map_metadata_fingerprint", "")),
        map_data_fingerprint=str(region_snapshot.get("map_data_fingerprint", "")),
        trajectory_session_id=str(region_snapshot.get("trajectory_session_id", "")),
        trajectory_revision=int(region_snapshot.get("trajectory_revision", 0)),
        trajectory_schema_version=str(region_snapshot.get("trajectory_schema_version", TRAJECTORY_SCHEMA_VERSION)),
        selection_mode=selection_mode,
        visual_context_id=(
            str(visual_manifest.get("visual_context_id", "")) if visual_manifest else qwen_decision.get("visual_context_id")
        ),
        visual_context_schema_version=(
            str(visual_manifest.get("visual_context_schema_version", VISUAL_CONTEXT_SCHEMA_VERSION))
            if visual_manifest
            else None
        ),
        visual_context_complete=bool(visual_manifest.get("capture_complete", False)) if visual_manifest else False,
        qwen_decision_id=str(qwen_decision.get("decision_id", "")),
        qwen_decision_schema_version=str(
            qwen_decision.get("qwen_decision_schema_version", REGION_DECISION_SCHEMA_VERSION)
        ),
        qwen_recommended_region=fusion.get("qwen_recommended_region"),
        algorithm_final_region=algorithm_final,
        decision_source=str(fusion.get("decision_source", "")),
        decision_revalidation_passed=decision_revalidation_passed,
        region_label=algorithm_final,
        internal_region_id=str(geo_entry.get("internal_region_id", "")),
        track_id=str(geo_entry.get("track_id", "")),
        region_geometry_fingerprint=str(geo_entry.get("region_geometry_fingerprint", "")),
        region_geometry_schema_version=str(
            region_geometry.get("region_geometry_schema_version", REGION_GEOMETRY_SCHEMA_VERSION)
        ),
        region_stable=bool(geo_entry.get("stable", False)),
        region_snapshot_eligible=bool(geo_entry.get("snapshot_eligible", False)),
        region_blacklisted=bool(geo_entry.get("blacklisted", False)),
        region_geo_score=float(geo_entry.get("geo_score", 0.0)),
        configured_selection_strategy=str(
            qwen_decision.get("configured_strategy", "CANDIDATE_RANKING")
        ),
        effective_selection_strategy=str(
            qwen_decision.get("effective_strategy", qwen_decision.get("configured_strategy", "CANDIDATE_RANKING"))
        ),
        region_source=str(qwen_decision.get("region_source", "ALGORITHM_CANDIDATE")),
        qwen_global_proposal_id=qwen_decision.get("qwen_global_proposal_id"),
        qwen_global_proposal_rank=qwen_decision.get("qwen_global_proposal_rank"),
        proposal_validation_passed=bool(qwen_decision.get("proposal_validation_passed", False)),
        proposal_validation_errors=list(qwen_decision.get("proposal_validation_errors", [])),
        map_render_metadata_fingerprint=str(
            region_snapshot.get("map_render_metadata_fingerprint", "")
        ),
        path_checked=False,
        reachable=None,
        safe_viewpoint_request_ready=False,
        validation_errors=[],
    )


def build_safe_viewpoint_request_envelope(
    bundle: ExplorationDecisionBundle,
    *,
    region_snapshot: Mapping[str, Any],
    region_geometry: Mapping[str, Any],
    qwen_decision: Mapping[str, Any],
    fusion: Mapping[str, Any],
    visual_manifest: Optional[Mapping[str, Any]] = None,
    region_view_mapping: Optional[Mapping[str, Any]] = None,
    cfg: Mapping[str, Any],
    map_data: Optional[Sequence[int]] = None,
    current_time: Optional[datetime] = None,
    safe_viewpoint_config_version: str = "phase3a_v1",
) -> Tuple[SafeViewpointRequestEnvelope, BundleValidationResult]:
    validation = validate_exploration_decision_bundle(
        bundle,
        region_snapshot=region_snapshot,
        region_geometry=region_geometry,
        qwen_decision=qwen_decision,
        fusion=fusion,
        visual_manifest=visual_manifest,
        region_view_mapping=region_view_mapping,
        cfg=cfg,
        current_time=current_time,
        map_data=map_data,
    )
    robot_pose = dict(region_snapshot.get("robot_pose", {}))
    geo_entry = region_geometry.get("regions", {}).get(bundle.algorithm_final_region or "", {})

    envelope = SafeViewpointRequestEnvelope(
        bundle_id=bundle.bundle_id,
        contract_version=bundle.contract_version,
        snapshot_id=bundle.snapshot_id,
        map_fingerprint=bundle.map_fingerprint,
        region_label=bundle.region_label,
        internal_region_id=bundle.internal_region_id,
        track_id=bundle.track_id,
        region_geometry_fingerprint=bundle.region_geometry_fingerprint,
        robot_pose=robot_pose,
        trajectory_session_id=bundle.trajectory_session_id,
        trajectory_revision=bundle.trajectory_revision,
        selected_region_geometry=dict(geo_entry),
        safe_viewpoint_config_version=safe_viewpoint_config_version,
        path_checked=False,
        reachable=None,
        request_ready=False,
    )

    if not validation.valid or not bundle.algorithm_final_region or not geo_entry:
        return envelope, validation

    envelope.request_ready = True
    return envelope, validation


def bundle_to_dict(bundle: ExplorationDecisionBundle) -> Dict[str, Any]:
    data = asdict(bundle)
    data["path_checked"] = False
    data["reachable"] = None
    return data


def envelope_to_dict(envelope: SafeViewpointRequestEnvelope) -> Dict[str, Any]:
    data = asdict(envelope)
    data["path_checked"] = False
    data["reachable"] = None
    return data
