#!/usr/bin/env python3
"""Exploration pipeline data contracts — pure Python, no ROS or motion interfaces."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union

EXPLORATION_CONTRACT_VERSION = "1.0"
REGION_SNAPSHOT_SCHEMA_VERSION = "1.0"
REGION_GEOMETRY_SCHEMA_VERSION = "1.0"
TRAJECTORY_SCHEMA_VERSION = "1.0"
VISUAL_CONTEXT_SCHEMA_VERSION = "1.0"
REGION_DECISION_SCHEMA_VERSION = "1.0"
SAFE_VIEWPOINT_SCHEMA_VERSION = "1.0"
BUNDLE_SCHEMA_VERSION = "1.0"

SUPPORTED_CONTRACT_VERSIONS = frozenset({EXPLORATION_CONTRACT_VERSION})
SUPPORTED_SCHEMA_VERSIONS = frozenset(
    {
        REGION_SNAPSHOT_SCHEMA_VERSION,
        REGION_GEOMETRY_SCHEMA_VERSION,
        TRAJECTORY_SCHEMA_VERSION,
        VISUAL_CONTEXT_SCHEMA_VERSION,
        REGION_DECISION_SCHEMA_VERSION,
        SAFE_VIEWPOINT_SCHEMA_VERSION,
        BUNDLE_SCHEMA_VERSION,
    }
)


def _sha256_hex(payload: Union[str, bytes]) -> str:
    if isinstance(payload, str):
        payload = payload.encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def serialize_float(value: float, precision: int = 6) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{float(value):.{precision}f}"


def validate_contract_config(cfg: Mapping[str, Any]) -> List[str]:
    errors: List[str] = []
    safety = cfg.get("safety", {})
    for key in ("allow_motion", "allow_cmd_vel", "allow_nav2", "allow_goal_publish"):
        if bool(safety.get(key, False)):
            errors.append(f"safety.{key} must be false")
    contracts = cfg.get("contracts", {})
    if contracts.get("exploration_contract_version") not in SUPPORTED_CONTRACT_VERSIONS:
        errors.append("contracts.exploration_contract_version unsupported")
    return errors


def _fingerprint_cfg(cfg: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    fp = (cfg or {}).get("fingerprints", {})
    return {
        "float_precision_digits": int(fp.get("float_precision_digits", 6)),
        "sort_region_cells": bool(fp.get("sort_region_cells", True)),
        "include_map_stamp_in_map_fingerprint": bool(
            fp.get("include_map_stamp_in_map_fingerprint", True)
        ),
    }


def build_map_metadata_fingerprint(
    *,
    frame_id: str,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    map_stamp: Optional[float] = None,
    cfg: Optional[Mapping[str, Any]] = None,
) -> str:
    fcfg = _fingerprint_cfg(cfg)
    precision = fcfg["float_precision_digits"]
    parts = [
        str(frame_id),
        str(int(width)),
        str(int(height)),
        serialize_float(resolution, precision),
        serialize_float(origin_x, precision),
        serialize_float(origin_y, precision),
        serialize_float(origin_yaw, precision),
    ]
    if fcfg["include_map_stamp_in_map_fingerprint"] and map_stamp is not None:
        parts.append(serialize_float(float(map_stamp), precision))
    return _sha256_hex("|".join(parts))


def build_map_data_fingerprint(map_data: Sequence[int]) -> str:
    payload = ",".join(str(int(v)) for v in map_data)
    return _sha256_hex(payload)


def build_map_fingerprint(
    *,
    frame_id: str,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    map_data: Sequence[int],
    map_stamp: Optional[float] = None,
    cfg: Optional[Mapping[str, Any]] = None,
) -> Dict[str, str]:
    metadata_fp = build_map_metadata_fingerprint(
        frame_id=frame_id,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        map_stamp=map_stamp,
        cfg=cfg,
    )
    data_fp = build_map_data_fingerprint(map_data)
    combined = _sha256_hex(f"{metadata_fp}|{data_fp}")
    return {
        "map_fingerprint": combined,
        "map_metadata_fingerprint": metadata_fp,
        "map_data_fingerprint": data_fp,
    }


def build_map_fingerprint_from_snapshot(
    region_snapshot: Mapping[str, Any],
    map_data: Sequence[int],
    *,
    origin_yaw: float = 0.0,
    cfg: Optional[Mapping[str, Any]] = None,
) -> Dict[str, str]:
    meta = region_snapshot.get("map_metadata", {})
    frame_id = str(meta.get("frame_id", region_snapshot.get("map_frame", "map")))
    return build_map_fingerprint(
        frame_id=frame_id,
        width=int(meta.get("width", 0)),
        height=int(meta.get("height", 0)),
        resolution=float(meta.get("resolution", 0.05)),
        origin_x=float(meta.get("origin_x", 0.0)),
        origin_y=float(meta.get("origin_y", 0.0)),
        origin_yaw=float(meta.get("origin_yaw", origin_yaw)),
        map_data=map_data,
        map_stamp=float(region_snapshot.get("map_stamp", meta.get("stamp_sec", 0.0))),
        cfg=cfg,
    )


def _normalize_frontier_cells(
    frontier_cells_grid: Sequence[Sequence[int]],
    sort_cells: bool,
) -> List[Tuple[int, int]]:
    cells = [(int(c[0]), int(c[1])) for c in frontier_cells_grid if len(c) >= 2]
    if sort_cells:
        cells.sort(key=lambda t: (t[0], t[1]))
    return cells


def build_region_geometry_fingerprint(
    *,
    snapshot_id: str,
    region_label: str,
    internal_region_id: str,
    track_id: str,
    frontier_cells_grid: Sequence[Sequence[int]],
    frontier_points_map: Sequence[Sequence[float]],
    centroid_x: float,
    centroid_y: float,
    bbox_grid: Sequence[int],
    cfg: Optional[Mapping[str, Any]] = None,
) -> str:
    fcfg = _fingerprint_cfg(cfg)
    precision = fcfg["float_precision_digits"]
    cells = _normalize_frontier_cells(frontier_cells_grid, fcfg["sort_region_cells"])
    cell_to_point: Dict[Tuple[int, int], Tuple[float, float]] = {}
    for cell, pt in zip(frontier_cells_grid, frontier_points_map):
        if len(cell) >= 2 and len(pt) >= 2:
            cell_to_point[(int(cell[0]), int(cell[1]))] = (float(pt[0]), float(pt[1]))
    point_parts: List[str] = []
    for cell in cells:
        pt = cell_to_point.get(cell)
        if pt is None:
            point_parts.append("na,na")
        else:
            point_parts.append(
                f"{serialize_float(pt[0], precision)},{serialize_float(pt[1], precision)}"
            )
    payload = "|".join(
        [
            str(snapshot_id),
            str(region_label),
            str(internal_region_id),
            str(track_id),
            ";".join(f"{r},{c}" for r, c in cells),
            ";".join(point_parts),
            f"{serialize_float(centroid_x, precision)},{serialize_float(centroid_y, precision)}",
            ",".join(str(int(v)) for v in bbox_grid),
        ]
    )
    return _sha256_hex(payload)


def build_region_geometry_fingerprint_from_entry(
    snapshot_id: str,
    region_label: str,
    entry: Mapping[str, Any],
    *,
    cfg: Optional[Mapping[str, Any]] = None,
) -> str:
    centroid = entry.get("centroid", {})
    if isinstance(centroid, Mapping):
        cx = float(centroid.get("x", entry.get("centroid_x", 0.0)))
        cy = float(centroid.get("y", entry.get("centroid_y", 0.0)))
    else:
        cx = float(entry.get("centroid_x", 0.0))
        cy = float(entry.get("centroid_y", 0.0))
    return build_region_geometry_fingerprint(
        snapshot_id=snapshot_id,
        region_label=region_label,
        internal_region_id=str(entry.get("internal_region_id", "")),
        track_id=str(entry.get("track_id", "")),
        frontier_cells_grid=entry.get("frontier_cells_grid", []),
        frontier_points_map=entry.get("frontier_points_map", []),
        centroid_x=cx,
        centroid_y=cy,
        bbox_grid=entry.get("bbox_grid", []),
        cfg=cfg,
    )


def contract_fields(
    *,
    contract_version: str = EXPLORATION_CONTRACT_VERSION,
    schema_version: str,
) -> Dict[str, str]:
    return {
        "contract_version": contract_version,
        "schema_version": schema_version,
    }


def verify_map_data_unchanged(before: Sequence[int], after: Sequence[int]) -> bool:
    return list(before) == list(after)


def deep_copy_map_data(data: Sequence[int]) -> List[int]:
    return copy.deepcopy(list(data))


def canonical_json_dumps(obj: Any) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
