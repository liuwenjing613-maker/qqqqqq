#!/usr/bin/env python3
"""Pure 360° visual context algorithms — no ROS, no motion control."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    _CV2_AVAILABLE = True
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    np = None  # type: ignore
    _CV2_AVAILABLE = False

VIEW_IDS: Tuple[str, ...] = (
    "VIEW_000",
    "VIEW_045",
    "VIEW_090",
    "VIEW_135",
    "VIEW_180",
    "VIEW_225",
    "VIEW_270",
    "VIEW_315",
)

VIEW_TARGET_ANGLES: Dict[str, float] = {
    "VIEW_000": 0.0,
    "VIEW_045": 45.0,
    "VIEW_090": 90.0,
    "VIEW_135": 135.0,
    "VIEW_180": 180.0,
    "VIEW_225": 225.0,
    "VIEW_270": 270.0,
    "VIEW_315": 315.0,
}

SUPPORTED_IMAGE_ENCODINGS = frozenset(
    {"bgr8", "rgb8", "mono8", "jpeg", "png", "jpg"}
)


def normalize_angle_deg(angle: float) -> float:
    a = angle % 360.0
    if a < 0:
        a += 360.0
    return a


def shortest_angular_distance_deg(from_deg: float, to_deg: float) -> float:
    diff = normalize_angle_deg(to_deg - from_deg)
    if diff > 180.0:
        diff -= 360.0
    return diff


def unwrap_yaw_sequence(yaws_deg: Sequence[float]) -> List[float]:
    if not yaws_deg:
        return []
    out = [float(yaws_deg[0])]
    for yaw in yaws_deg[1:]:
        delta = shortest_angular_distance_deg(out[-1], yaw)
        out.append(out[-1] + delta)
    return out


def accumulate_rotation_deg(yaws_deg: Sequence[float]) -> float:
    if len(yaws_deg) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(yaws_deg)):
        total += abs(shortest_angular_distance_deg(yaws_deg[i - 1], yaws_deg[i]))
    return total


def relative_angle_from_initial(initial_yaw_deg: float, current_yaw_deg: float) -> float:
    return normalize_angle_deg(current_yaw_deg - initial_yaw_deg)


def view_id_for_target_angle(target_deg: float) -> str:
    t = normalize_angle_deg(target_deg)
    for vid, ang in VIEW_TARGET_ANGLES.items():
        if abs(shortest_angular_distance_deg(ang, t)) < 0.01:
            return vid
    return f"VIEW_{int(round(t)):03d}"


@dataclass
class DirectionalFrameCandidate:
    image_stamp_sec: float
    tf_stamp_sec: float
    tf_age_s: float
    absolute_yaw_deg: float
    relative_yaw_deg: float
    image_reference: str
    width: int = 0
    height: int = 0
    encoding: str = ""
    valid: bool = True
    rejection_reasons: List[str] = field(default_factory=list)


@dataclass
class DirectionalFrame:
    view_id: str
    target_relative_angle_deg: float
    captured_relative_angle_deg: float
    absolute_yaw_deg: float
    angle_error_deg: float
    image_stamp_sec: float
    tf_stamp_sec: float
    tf_age_s: float
    image_file: str
    width: int = 0
    height: int = 0
    valid: bool = True
    rejection_reasons: List[str] = field(default_factory=list)


@dataclass
class RegionViewAssociation:
    region_label: str
    internal_region_id: str
    track_id: str
    region_global_bearing_deg: float
    region_relative_bearing_deg: float
    primary_view_id: str = ""
    secondary_view_ids: List[str] = field(default_factory=list)
    primary_angle_difference_deg: float = float("inf")
    mapping_valid: bool = False
    rejection_reasons: List[str] = field(default_factory=list)


@dataclass
class VisualCaptureSession:
    capture_session_id: str
    snapshot_id: str
    state: str = "IDLE"
    initial_robot_yaw_deg: float = 0.0
    final_robot_yaw_deg: float = 0.0
    accumulated_rotation_deg: float = 0.0
    capture_start_time: str = ""
    capture_end_time: str = ""
    candidates: List[DirectionalFrameCandidate] = field(default_factory=list)
    yaw_samples_deg: List[float] = field(default_factory=list)


@dataclass
class VisualContextManifest:
    visual_context_id: str
    capture_session_id: str
    snapshot_id: str
    capture_start_time: str
    capture_end_time: str
    initial_robot_yaw_deg: float
    final_robot_yaw_deg: float
    accumulated_rotation_deg: float
    target_view_angles_deg: List[float]
    frames: List[DirectionalFrame]
    region_view_associations: List[RegionViewAssociation]
    contact_sheet_file: str = ""
    decision_board_file: str = ""
    capture_complete: bool = False
    validation_errors: List[str] = field(default_factory=list)


@dataclass
class VisualContextValidationResult:
    valid: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


def validate_visual_context_config(cfg: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    safety = cfg.get("safety", {})
    for key in ("allow_motion", "allow_cmd_vel", "allow_nav2"):
        if bool(safety.get(key, False)):
            errors.append(f"safety.{key} must be false")

    vcfg = cfg.get("visual_context", {})
    if not vcfg:
        return errors

    angles = list(vcfg.get("target_relative_angles_deg", []))
    if len(angles) != len(set(angles)):
        errors.append("visual_context.target_relative_angles_deg contains duplicates")
    for ang in angles:
        try:
            a = float(ang)
        except (TypeError, ValueError):
            errors.append("visual_context.target_relative_angles_deg invalid angle")
            continue
        if a < -360.0 or a > 720.0:
            errors.append(f"visual_context.target_relative_angles_deg out of range: {a}")

    if float(vcfg.get("max_angle_error_deg", 12.0)) < 0:
        errors.append("visual_context.max_angle_error_deg must be >= 0")
    if float(vcfg.get("max_tf_age_s", 0.30)) < 0:
        errors.append("visual_context.max_tf_age_s must be >= 0")
    if float(vcfg.get("complete_rotation_threshold_deg", 350.0)) <= 0:
        errors.append("visual_context.complete_rotation_threshold_deg must be > 0")
    if float(vcfg.get("max_capture_duration_s", 90.0)) <= 0:
        errors.append("visual_context.max_capture_duration_s must be > 0")

    return errors


def _candidate_is_valid(
    candidate: DirectionalFrameCandidate,
    cfg: Dict[str, Any],
) -> Tuple[bool, List[str]]:
    vcfg = cfg.get("visual_context", {})
    reasons: List[str] = []
    if not candidate.valid:
        reasons.extend(candidate.rejection_reasons or ["FRAME_IMAGE_INVALID"])
        return False, reasons
    if not candidate.image_reference:
        reasons.append("FRAME_IMAGE_INVALID")
    if candidate.tf_age_s > float(vcfg.get("max_tf_age_s", 0.30)):
        reasons.append("FRAME_TF_STALE")
    enc = candidate.encoding.lower()
    if enc and enc not in SUPPORTED_IMAGE_ENCODINGS:
        reasons.append("FRAME_ENCODING_UNSUPPORTED")
    if candidate.width <= 0 or candidate.height <= 0:
        reasons.append("FRAME_IMAGE_INVALID")
    return len(reasons) == 0, reasons


def select_directional_frames(
    candidates: Sequence[DirectionalFrameCandidate],
    cfg: Dict[str, Any],
    *,
    initial_yaw_deg: float = 0.0,
) -> Tuple[List[DirectionalFrame], bool, List[str]]:
    """Select one frame per target view angle from passive candidates."""
    vcfg = cfg.get("visual_context", {})
    target_angles = [float(a) for a in vcfg.get("target_relative_angles_deg", list(VIEW_TARGET_ANGLES.values()))]
    max_err = float(vcfg.get("max_angle_error_deg", 12.0))
    allow_dup = bool(vcfg.get("allow_same_frame_for_multiple_views", False))
    complete_threshold = float(vcfg.get("complete_rotation_threshold_deg", 350.0))

    yaw_samples = [initial_yaw_deg]
    for c in candidates:
        if c.valid:
            yaw_samples.append(c.absolute_yaw_deg)
    accumulated = accumulate_rotation_deg(yaw_samples)

    assigned_refs: Set[str] = set()
    frames: List[DirectionalFrame] = []
    errors: List[str] = []

    for target in target_angles:
        vid = view_id_for_target_angle(target)
        best: Optional[Tuple[float, DirectionalFrameCandidate]] = None
        for cand in candidates:
            ok, rej = _candidate_is_valid(cand, cfg)
            if not ok:
                continue
            err = abs(shortest_angular_distance_deg(target, cand.relative_yaw_deg))
            if err > max_err:
                continue
            if not allow_dup and cand.image_reference in assigned_refs:
                continue
            if best is None or err < best[0]:
                best = (err, cand)

        if best is None:
            frames.append(
                DirectionalFrame(
                    view_id=vid,
                    target_relative_angle_deg=target,
                    captured_relative_angle_deg=float("nan"),
                    absolute_yaw_deg=float("nan"),
                    angle_error_deg=float("inf"),
                    image_stamp_sec=0.0,
                    tf_stamp_sec=0.0,
                    tf_age_s=0.0,
                    image_file="",
                    valid=False,
                    rejection_reasons=["FRAME_ANGLE_ERROR_TOO_LARGE"],
                )
            )
            errors.append(f"MISSING_VIEW:{vid}")
            continue

        err, cand = best
        if not allow_dup:
            assigned_refs.add(cand.image_reference)
        frames.append(
            DirectionalFrame(
                view_id=vid,
                target_relative_angle_deg=target,
                captured_relative_angle_deg=cand.relative_yaw_deg,
                absolute_yaw_deg=cand.absolute_yaw_deg,
                angle_error_deg=err,
                image_stamp_sec=cand.image_stamp_sec,
                tf_stamp_sec=cand.tf_stamp_sec,
                tf_age_s=cand.tf_age_s,
                image_file=cand.image_reference,
                width=cand.width,
                height=cand.height,
                valid=True,
            )
        )

    all_valid = all(f.valid for f in frames)
    rotation_ok = accumulated >= complete_threshold
    capture_complete = all_valid and rotation_ok
    if not rotation_ok:
        errors.append("INCOMPLETE_ROTATION")
    return frames, capture_complete, errors


def compute_region_global_bearing_deg(
    region: Dict[str, Any],
    robot_pose: Dict[str, float],
) -> Optional[float]:
    if "region_global_bearing_deg" in region:
        try:
            return normalize_angle_deg(float(region["region_global_bearing_deg"]))
        except (TypeError, ValueError):
            return None
    if "bearing_global_deg" in region:
        try:
            return normalize_angle_deg(float(region["bearing_global_deg"]))
        except (TypeError, ValueError):
            return None
    centroid = region.get("centroid") or {}
    try:
        cx = float(centroid.get("x", float("nan")))
        cy = float(centroid.get("y", float("nan")))
        rx = float(robot_pose.get("x", 0.0))
        ry = float(robot_pose.get("y", 0.0))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (cx, cy, rx, ry)):
        return None
    return normalize_angle_deg(math.degrees(math.atan2(cy - ry, cx - rx)))


def _adjacent_view_ids(primary_view_id: str) -> List[str]:
    if primary_view_id not in VIEW_IDS:
        return []
    idx = VIEW_IDS.index(primary_view_id)
    n = len(VIEW_IDS)
    return [VIEW_IDS[(idx - 1) % n], VIEW_IDS[(idx + 1) % n]]


def associate_regions_with_views(
    snapshot: Dict[str, Any],
    frames: Sequence[DirectionalFrame],
    initial_robot_yaw_deg: float,
    cfg: Dict[str, Any],
) -> Tuple[List[RegionViewAssociation], List[str]]:
    mcfg = cfg.get("region_view_mapping", {})
    max_primary = float(mcfg.get("max_primary_angle_difference_deg", 35.0))
    max_secondary = float(mcfg.get("max_secondary_angle_difference_deg", 70.0))
    include_adjacent = bool(mcfg.get("include_adjacent_views", True))

    snapshot_id = str(snapshot.get("snapshot_id", ""))
    robot_pose = dict(snapshot.get("robot_pose", {}))
    regions = list(snapshot.get("accepted_regions") or snapshot.get("regions") or [])

    valid_frames = {f.view_id: f for f in frames if f.valid}
    associations: List[RegionViewAssociation] = []
    errors: List[str] = []

    for region in regions:
        label = str(region.get("label", ""))
        assoc = RegionViewAssociation(
            region_label=label,
            internal_region_id=str(region.get("internal_region_id", "")),
            track_id=str(region.get("track_id", "")),
            region_global_bearing_deg=0.0,
            region_relative_bearing_deg=0.0,
        )
        bearing = compute_region_global_bearing_deg(region, robot_pose)
        if bearing is None:
            assoc.rejection_reasons.append("REGION_BEARING_MISSING")
            associations.append(assoc)
            errors.append(f"REGION_BEARING_MISSING:{label}")
            continue

        assoc.region_global_bearing_deg = bearing
        assoc.region_relative_bearing_deg = relative_angle_from_initial(
            initial_robot_yaw_deg, bearing
        )

        best_vid = ""
        best_err = float("inf")
        for vid, frame in valid_frames.items():
            view_bearing = relative_angle_from_initial(
                initial_robot_yaw_deg, frame.absolute_yaw_deg
            )
            err = abs(shortest_angular_distance_deg(assoc.region_relative_bearing_deg, view_bearing))
            if err < best_err:
                best_err = err
                best_vid = vid

        if not best_vid:
            assoc.rejection_reasons.append("NO_VALID_PRIMARY_VIEW")
            associations.append(assoc)
            errors.append(f"NO_VALID_PRIMARY_VIEW:{label}")
            continue

        assoc.primary_view_id = best_vid
        assoc.primary_angle_difference_deg = best_err
        if best_err > max_primary:
            assoc.rejection_reasons.append("PRIMARY_VIEW_ANGLE_TOO_LARGE")
            associations.append(assoc)
            errors.append(f"PRIMARY_VIEW_ANGLE_TOO_LARGE:{label}")
            continue

        if include_adjacent:
            for sid in _adjacent_view_ids(best_vid):
                sf = valid_frames.get(sid)
                if sf is None:
                    continue
                view_bearing = relative_angle_from_initial(
                    initial_robot_yaw_deg, sf.absolute_yaw_deg
                )
                err = abs(shortest_angular_distance_deg(assoc.region_relative_bearing_deg, view_bearing))
                if err <= max_secondary:
                    assoc.secondary_view_ids.append(sid)

        assoc.mapping_valid = True
        associations.append(assoc)

    if snapshot_id and not regions:
        errors.append("SNAPSHOT_REGIONS_EMPTY")
    return associations, errors


def generate_visual_context_id(when: Optional[datetime] = None) -> str:
    ts = when or datetime.now(timezone.utc)
    return f"VC_{ts.strftime('%Y%m%dT%H%M%S')}"


def build_visual_context_manifest(
    session: VisualCaptureSession,
    frames: Sequence[DirectionalFrame],
    associations: Sequence[RegionViewAssociation],
    cfg: Dict[str, Any],
    *,
    visual_context_id: str = "",
    validation_errors: Optional[List[str]] = None,
) -> VisualContextManifest:
    vcfg = cfg.get("visual_context", {})
    target_angles = [float(a) for a in vcfg.get("target_relative_angles_deg", list(VIEW_TARGET_ANGLES.values()))]
    _, capture_complete, frame_errors = select_directional_frames(
        session.candidates,
        cfg,
        initial_yaw_deg=session.initial_robot_yaw_deg,
    )
    all_errors = list(validation_errors or []) + frame_errors
    return VisualContextManifest(
        visual_context_id=visual_context_id or generate_visual_context_id(),
        capture_session_id=session.capture_session_id,
        snapshot_id=session.snapshot_id,
        capture_start_time=session.capture_start_time,
        capture_end_time=session.capture_end_time,
        initial_robot_yaw_deg=session.initial_robot_yaw_deg,
        final_robot_yaw_deg=session.final_robot_yaw_deg,
        accumulated_rotation_deg=session.accumulated_rotation_deg,
        target_view_angles_deg=target_angles,
        frames=list(frames),
        region_view_associations=list(associations),
        capture_complete=capture_complete,
        validation_errors=all_errors,
    )


def manifest_to_dict(
    manifest: VisualContextManifest,
    *,
    map_fingerprint: str = "",
    trajectory_revision: int = 0,
    contract_version: str = "",
    visual_context_schema_version: str = "",
) -> Dict[str, Any]:
    from src.planning.exploration_contracts import (  # noqa: WPS433
        EXPLORATION_CONTRACT_VERSION,
        VISUAL_CONTEXT_SCHEMA_VERSION,
    )

    return {
        "contract_version": contract_version or EXPLORATION_CONTRACT_VERSION,
        "visual_context_schema_version": visual_context_schema_version or VISUAL_CONTEXT_SCHEMA_VERSION,
        "schema_version": visual_context_schema_version or VISUAL_CONTEXT_SCHEMA_VERSION,
        "visual_context_id": manifest.visual_context_id,
        "capture_session_id": manifest.capture_session_id,
        "snapshot_id": manifest.snapshot_id,
        "capture_start_time": manifest.capture_start_time,
        "capture_end_time": manifest.capture_end_time,
        "initial_robot_yaw_deg": manifest.initial_robot_yaw_deg,
        "final_robot_yaw_deg": manifest.final_robot_yaw_deg,
        "accumulated_rotation_deg": manifest.accumulated_rotation_deg,
        "target_view_angles_deg": list(manifest.target_view_angles_deg),
        "frames": [asdict(f) for f in manifest.frames],
        "region_view_associations": [asdict(a) for a in manifest.region_view_associations],
        "contact_sheet_file": manifest.contact_sheet_file,
        "decision_board_file": manifest.decision_board_file,
        "capture_complete": manifest.capture_complete,
        "validation_errors": list(manifest.validation_errors),
        "map_fingerprint": map_fingerprint,
        "trajectory_revision": trajectory_revision,
    }


def region_view_mapping_to_dict(
    associations: Sequence[RegionViewAssociation],
    snapshot_id: str,
    visual_context_id: str,
    *,
    contract_version: str = "",
    visual_context_schema_version: str = "",
) -> Dict[str, Any]:
    from src.planning.exploration_contracts import (  # noqa: WPS433
        EXPLORATION_CONTRACT_VERSION,
        VISUAL_CONTEXT_SCHEMA_VERSION,
    )

    return {
        "contract_version": contract_version or EXPLORATION_CONTRACT_VERSION,
        "visual_context_schema_version": visual_context_schema_version or VISUAL_CONTEXT_SCHEMA_VERSION,
        "schema_version": visual_context_schema_version or VISUAL_CONTEXT_SCHEMA_VERSION,
        "snapshot_id": snapshot_id,
        "visual_context_id": visual_context_id,
        "associations": [asdict(a) for a in associations],
    }


def validate_visual_context_manifest(
    manifest: VisualContextManifest,
    expected_snapshot_id: str,
) -> VisualContextValidationResult:
    errors: List[str] = []
    if manifest.snapshot_id != expected_snapshot_id:
        errors.append("SNAPSHOT_ID_MISMATCH")
    if not manifest.frames:
        errors.append("FRAMES_EMPTY")
    for frame in manifest.frames:
        if not frame.valid:
            errors.append(f"FRAME_MISSING:{frame.view_id}")
    if not manifest.capture_complete:
        errors.append("CAPTURE_INCOMPLETE")
    return VisualContextValidationResult(valid=len(errors) == 0, errors=errors)


def render_panorama_contact_sheet(
    frames: Sequence[DirectionalFrame],
    associations: Sequence[RegionViewAssociation],
    output_path: Path,
    *,
    cell_w: int = 320,
    cell_h: int = 240,
) -> bool:
    """Render 2x4 contact sheet; missing views show MISSING VIEW placeholder."""
    if not _CV2_AVAILABLE or cv2 is None or np is None:
        return False

    label_by_view: Dict[str, List[str]] = {}
    for assoc in associations:
        if assoc.mapping_valid and assoc.primary_view_id:
            label_by_view.setdefault(assoc.primary_view_id, []).append(assoc.region_label)

    rows, cols = 2, 4
    canvas = np.zeros((rows * cell_h, cols * cell_w, 3), dtype=np.uint8)
    canvas[:] = (30, 30, 30)

    ordered = sorted(frames, key=lambda f: f.target_relative_angle_deg)
    for idx, frame in enumerate(ordered[:8]):
        r, c = divmod(idx, cols)
        y0, x0 = r * cell_h, c * cell_w
        cell = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
        cell[:] = (50, 50, 50)

        img_path = Path(frame.image_file) if frame.image_file else None
        if frame.valid and img_path and img_path.is_file():
            img = cv2.imread(str(img_path))
            if img is not None:
                cell = cv2.resize(img, (cell_w, cell_h))
        else:
            cv2.putText(
                cell,
                "MISSING VIEW",
                (20, cell_h // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        labels = ",".join(label_by_view.get(frame.view_id, []))
        caption = (
            f"{frame.view_id} tgt={frame.target_relative_angle_deg:.0f} "
            f"cap={frame.captured_relative_angle_deg:.1f} err={frame.angle_error_deg:.1f} "
            f"yaw={frame.absolute_yaw_deg:.1f} regions={labels}"
        )
        cv2.putText(
            cell,
            caption[:60],
            (5, cell_h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        canvas[y0 : y0 + cell_h, x0 : x0 + cell_w] = cell

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(output_path), canvas))


def render_decision_board(
    annotated_map_path: Path,
    contact_sheet_path: Path,
    output_path: Path,
    *,
    snapshot_id: str = "",
    visual_context_id: str = "",
    target_instruction: str = "",
    associations: Optional[Sequence[RegionViewAssociation]] = None,
) -> bool:
    """Combine annotated map (left) and contact sheet (right) into decision board."""
    if not _CV2_AVAILABLE or cv2 is None or np is None:
        return False

    left = cv2.imread(str(annotated_map_path)) if annotated_map_path.is_file() else None
    right = cv2.imread(str(contact_sheet_path)) if contact_sheet_path.is_file() else None
    if left is None:
        left = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(left, "annotated_map missing", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    if right is None:
        right = np.zeros((480, 1280, 3), dtype=np.uint8)
        cv2.putText(right, "contact_sheet missing", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    target_h = max(left.shape[0], right.shape[0])
    scale_l = target_h / left.shape[0]
    scale_r = target_h / right.shape[0]
    left_r = cv2.resize(left, (int(left.shape[1] * scale_l), target_h))
    right_r = cv2.resize(right, (int(right.shape[1] * scale_r), target_h))
    combined = np.hstack([left_r, right_r])

    footer_h = 80
    footer = np.zeros((footer_h, combined.shape[1], 3), dtype=np.uint8)
    mapping_lines = []
    for assoc in associations or []:
        if assoc.mapping_valid:
            mapping_lines.append(
                f"{assoc.region_label}->{assoc.primary_view_id}"
                + (f"+{','.join(assoc.secondary_view_ids)}" if assoc.secondary_view_ids else "")
            )
    footer_text = (
        f"snapshot={snapshot_id} vc={visual_context_id} target={target_instruction} "
        f"mapping={' | '.join(mapping_lines)}"
    )
    cv2.putText(
        footer,
        footer_text[:180],
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    board = np.vstack([combined, footer])
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(output_path), board))


def save_visual_context_artifacts(
    session_dir: Path,
    manifest: VisualContextManifest,
    associations: Sequence[RegionViewAssociation],
    *,
    annotated_map_path: Optional[Path] = None,
    target_instruction: str = "",
    cfg: Optional[Dict[str, Any]] = None,
) -> Dict[str, str]:
    """Write manifest JSON paths; image files expected pre-populated under session_dir/frames/."""
    session_dir = Path(session_dir)
    frames_dir = session_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    paths: Dict[str, str] = {}

    manifest_path = session_dir / "visual_context_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest_to_dict(manifest), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    paths["visual_context_manifest"] = str(manifest_path)

    mapping_path = session_dir / "region_view_mapping.json"
    mapping_path.write_text(
        json.dumps(
            region_view_mapping_to_dict(
                associations, manifest.snapshot_id, manifest.visual_context_id
            ),
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    paths["region_view_mapping"] = str(mapping_path)

    vcfg = (cfg or {}).get("visual_context", {})
    if bool(vcfg.get("generate_contact_sheet", True)):
        contact_path = session_dir / "panorama_contact_sheet.jpg"
        render_panorama_contact_sheet(manifest.frames, associations, contact_path)
        paths["contact_sheet_file"] = str(contact_path)
        manifest.contact_sheet_file = str(contact_path)

    if bool(vcfg.get("generate_decision_board", True)) and annotated_map_path:
        board_path = session_dir / "decision_board.jpg"
        render_decision_board(
            Path(annotated_map_path),
            Path(paths.get("contact_sheet_file", session_dir / "panorama_contact_sheet.jpg")),
            board_path,
            snapshot_id=manifest.snapshot_id,
            visual_context_id=manifest.visual_context_id,
            target_instruction=target_instruction,
            associations=associations,
        )
        paths["decision_board_file"] = str(board_path)
        manifest.decision_board_file = str(board_path)

    diag = {
        "state": session_dir.name,
        "capture_complete": manifest.capture_complete,
        "validation_errors": manifest.validation_errors,
    }
    diag_path = session_dir / "capture_diagnostics.json"
    diag_path.write_text(json.dumps(diag, ensure_ascii=False, indent=2), encoding="utf-8")
    paths["capture_diagnostics"] = str(diag_path)
    return paths
