#!/usr/bin/env python3
"""Validate Qwen global region proposals against occupancy grid and frontier geometry."""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from src.planning.frontier_region_debug_core import (
    FrontierAnalysisResult,
    FrontierRegion,
    MapMetadata,
    classify_map_cells,
    grid_row_to_image_y,
    grid_to_world,
    world_to_grid,
)
from src.vlm.qwen_global_region_selector_core import GlobalRegionProposal

REGION_SOURCE_QWEN_GLOBAL = "QWEN_GLOBAL_PROPOSAL"
REGION_SOURCE_ALGORITHM = "ALGORITHM_CANDIDATE"


@dataclass
class ProposalMapStatistics:
    free_ratio: float = 0.0
    unknown_ratio: float = 0.0
    occupied_ratio: float = 0.0
    uncertain_ratio: float = 0.0
    frontier_cell_count: int = 0
    total_cells: int = 0


@dataclass
class GlobalProposalValidationOutcome:
    validation_passed: bool
    rejection_reasons: List[str] = field(default_factory=list)
    matched_region: Optional[FrontierRegion] = None
    matched_frontier_cluster_id: str = ""
    nearest_frontier_distance_m: float = float("inf")
    mapped_grid_bounds: Tuple[int, int, int, int] = (0, 0, 0, 0)
    statistics: Optional[ProposalMapStatistics] = None
    formal_region_id: str = ""
    formal_region_label: str = ""


def _map_values_cfg(cfg: Dict[str, Any]) -> Tuple[int, int, int]:
    mv = cfg.get("map_values", {})
    return (
        int(mv.get("unknown_value", -1)),
        int(mv.get("free_max", 20)),
        int(mv.get("occupied_min", 65)),
    )


def normalized_viewport_to_panel_pixel(
    u: float,
    v: float,
    metadata: Dict[str, Any],
) -> Tuple[int, int]:
    panel = metadata.get("map_panel", {})
    x_min = int(panel.get("content_x_min_px", 0))
    y_min = int(panel.get("content_y_min_px", 0))
    x_max = int(panel.get("content_x_max_px", panel.get("image_width_px", 1)))
    y_max = int(panel.get("content_y_max_px", panel.get("image_height_px", 1)))
    content_w = max(1, x_max - x_min)
    content_h = max(1, y_max - y_min)
    px = int(round(x_min + float(u) * content_w))
    py = int(round(y_min + float(v) * content_h))
    return px, py


def panel_pixel_to_grid(
    px: int,
    py: int,
    metadata: Dict[str, Any],
    meta: MapMetadata,
) -> Tuple[int, int]:
    """Map panel pixel → grid (row, col)."""
    panel = metadata.get("map_panel", {})
    x_min = int(panel.get("content_x_min_px", 0))
    y_min = int(panel.get("content_y_min_px", 0))
    x_max = int(panel.get("content_x_max_px", meta.width))
    y_max = int(panel.get("content_y_max_px", meta.height))
    content_w = max(1, x_max - x_min)
    content_h = max(1, y_max - y_min)

    rel_x = (float(px) - x_min) / content_w
    rel_y = (float(py) - y_min) / content_h
    rel_x = max(0.0, min(1.0, rel_x))
    rel_y = max(0.0, min(1.0, rel_y))

    rt = metadata.get("render_transform", {})
    grid_y_flipped = bool(rt.get("grid_y_flipped", True))

    col = int(round(rel_x * (meta.width - 1)))
    if grid_y_flipped:
        image_y = int(round(rel_y * (meta.height - 1)))
        row = meta.height - 1 - image_y
    else:
        row = int(round(rel_y * (meta.height - 1)))

    row = max(0, min(meta.height - 1, row))
    col = max(0, min(meta.width - 1, col))
    return row, col


def normalized_viewport_to_grid(
    u: float,
    v: float,
    metadata: Dict[str, Any],
    meta: MapMetadata,
) -> Tuple[int, int]:
    px, py = normalized_viewport_to_panel_pixel(u, v, metadata)
    return panel_pixel_to_grid(px, py, metadata, meta)


def proposal_bbox_to_grid_bounds(
    proposal: GlobalRegionProposal,
    metadata: Dict[str, Any],
    meta: MapMetadata,
) -> Tuple[int, int, int, int]:
    """Return (row_min, row_max, col_min, col_max) inclusive."""
    r0, c0 = normalized_viewport_to_grid(proposal.bbox_u_min, proposal.bbox_v_min, metadata, meta)
    r1, c1 = normalized_viewport_to_grid(proposal.bbox_u_max, proposal.bbox_v_max, metadata, meta)
    row_min, row_max = sorted((r0, r1))
    col_min, col_max = sorted((c0, c1))
    return row_min, row_max, col_min, col_max


def compute_proposal_map_statistics(
    data: np.ndarray,
    row_min: int,
    row_max: int,
    col_min: int,
    col_max: int,
    cfg: Dict[str, Any],
    frontier_mask: Optional[np.ndarray] = None,
) -> ProposalMapStatistics:
    unknown_value, free_max, occupied_min = _map_values_cfg(cfg)
    h, w = data.shape
    r0 = max(0, row_min)
    r1 = min(h - 1, row_max)
    c0 = max(0, col_min)
    c1 = min(w - 1, col_max)
    if r1 < r0 or c1 < c0:
        return ProposalMapStatistics()

    patch = data[r0 : r1 + 1, c0 : c1 + 1]
    cats, _ = classify_map_cells(patch, unknown_value, free_max, occupied_min)
    total = int(cats.size)
    if total == 0:
        return ProposalMapStatistics()

    free_c = int(np.sum(cats == 0))
    occ_c = int(np.sum(cats == 1))
    unk_c = int(np.sum(cats == 2))
    unc_c = int(np.sum(cats == 3))
    frontier_c = 0
    if frontier_mask is not None:
        frontier_c = int(np.sum(frontier_mask[r0 : r1 + 1, c0 : c1 + 1]))

    return ProposalMapStatistics(
        free_ratio=free_c / total,
        unknown_ratio=unk_c / total,
        occupied_ratio=occ_c / total,
        uncertain_ratio=unc_c / total,
        frontier_cell_count=frontier_c,
        total_cells=total,
    )


def find_frontier_cells_in_proposal(
    frontier_mask: np.ndarray,
    row_min: int,
    row_max: int,
    col_min: int,
    col_max: int,
) -> List[Tuple[int, int]]:
    cells: List[Tuple[int, int]] = []
    h, w = frontier_mask.shape
    for r in range(max(0, row_min), min(h, row_max + 1)):
        for c in range(max(0, col_min), min(w, col_max + 1)):
            if frontier_mask[r, c]:
                cells.append((r, c))
    return cells


def _region_centroid_grid(region: FrontierRegion) -> Tuple[float, float]:
    return float(region.centroid_row), float(region.centroid_col)


def find_nearest_frontier_cluster(
    center_row: float,
    center_col: float,
    regions: Sequence[FrontierRegion],
    meta: MapMetadata,
) -> Tuple[Optional[FrontierRegion], float]:
    best: Optional[FrontierRegion] = None
    best_dist = float("inf")
    for region in regions:
        if not region.frontier_cells:
            continue
        dr = float(region.centroid_row) - center_row
        dc = float(region.centroid_col) - center_col
        dist_m = math.hypot(dr, dc) * meta.resolution
        if dist_m < best_dist:
            best_dist = dist_m
            best = region
    return best, best_dist


def validate_global_region_proposal(
    proposal: GlobalRegionProposal,
    *,
    data: np.ndarray,
    meta: MapMetadata,
    metadata: Dict[str, Any],
    result: FrontierAnalysisResult,
    cfg: Dict[str, Any],
    history_blacklist: Optional[Sequence[str]] = None,
) -> GlobalProposalValidationOutcome:
    """Validate one Qwen proposal against map geometry and frontier clusters."""
    gcfg = cfg.get("global_region_proposal", {})
    explore_cfg = cfg.get("region", {})
    frontier_cfg = cfg.get("frontier", {})

    rejections: List[str] = []
    center_row, center_col = normalized_viewport_to_grid(
        proposal.map_image_center_u,
        proposal.map_image_center_v,
        metadata,
        meta,
    )
    row_min, row_max, col_min, col_max = proposal_bbox_to_grid_bounds(proposal, metadata, meta)

    if row_min < 0 or col_min < 0 or row_max >= meta.height or col_max >= meta.width:
        rejections.append("GLOBAL_PROPOSAL_OUTSIDE_MAP")

    if row_max <= row_min or col_max <= col_min:
        rejections.append("GLOBAL_PROPOSAL_INVALID_BBOX")

    frontier_mask = result.filtered_frontier_mask
    if frontier_mask is None:
        frontier_mask = np.zeros(data.shape, dtype=bool)

    stats = compute_proposal_map_statistics(
        data, row_min, row_max, col_min, col_max, cfg, frontier_mask
    )

    max_occ = float(gcfg.get("maximum_occupied_ratio", 0.25))
    if stats.occupied_ratio > max_occ:
        rejections.append("GLOBAL_PROPOSAL_OCCUPIED_DOMINANT")

    min_overlap = float(gcfg.get("minimum_frontier_overlap_ratio", 0.10))
    min_frontier_cells = int(explore_cfg.get("min_frontier_cells", 8))
    max_snap = float(gcfg.get("max_frontier_snap_distance_m", 0.75))
    require_match = bool(gcfg.get("require_frontier_match", True))

    frontier_in_bbox = find_frontier_cells_in_proposal(
        frontier_mask, row_min, row_max, col_min, col_max
    )
    all_regions = list(result.regions) + [r for r in result.rejected_regions if r.frontier_cells]
    matched, nearest_m = find_nearest_frontier_cluster(
        float(center_row), float(center_col), all_regions, meta
    )

    overlap_ok = False
    if stats.total_cells > 0:
        overlap_ratio = stats.frontier_cell_count / stats.total_cells
        overlap_ok = overlap_ratio >= min_overlap or stats.frontier_cell_count >= min_frontier_cells

    snap_ok = nearest_m <= max_snap
    if require_match and not overlap_ok and not snap_ok:
        rejections.append("GLOBAL_PROPOSAL_NO_FRONTIER_MATCH")

    if matched is None:
        rejections.append("GLOBAL_PROPOSAL_NO_FRONTIER_MATCH")
    elif matched.frontier_cell_count < min_frontier_cells:
        rejections.append("GLOBAL_PROPOSAL_TINY_NOISE")

    min_clearance = float(frontier_cfg.get("min_clearance_m", 0.35))
    if matched is not None and matched.minimum_clearance_m < min_clearance:
        rejections.append("GLOBAL_PROPOSAL_LOW_CLEARANCE")

    if matched is not None and matched.blacklisted:
        rejections.append("GLOBAL_PROPOSAL_BLACKLISTED")

    if matched is not None and not matched.stable and bool(gcfg.get("require_region_stable", True)):
        rejections.append("GLOBAL_PROPOSAL_UNSTABLE")

    if history_blacklist and matched is not None and matched.track_id in history_blacklist:
        rejections.append("GLOBAL_PROPOSAL_BLACKLISTED")

    passed = len(rejections) == 0 and matched is not None
    formal_id = ""
    formal_label = ""
    if passed and matched is not None:
        formal_id = f"GQ_{proposal.proposal_id.replace('GP_', '').zfill(4)}"
        formal_label = formal_id

    return GlobalProposalValidationOutcome(
        validation_passed=passed,
        rejection_reasons=rejections,
        matched_region=matched,
        matched_frontier_cluster_id=matched.region_id if matched else "",
        nearest_frontier_distance_m=nearest_m,
        mapped_grid_bounds=(row_min, row_max, col_min, col_max),
        statistics=stats,
        formal_region_id=formal_id,
        formal_region_label=formal_label,
    )


def build_region_from_global_proposal(
    proposal: GlobalRegionProposal,
    outcome: GlobalProposalValidationOutcome,
    meta: MapMetadata,
    *,
    rank_index: int = 0,
) -> Dict[str, Any]:
    """Build formal region dict compatible with geometry bundle / Phase 3A."""
    region = outcome.matched_region
    if region is None:
        raise ValueError("cannot build region without matched frontier cluster")

    frontier_points = [
        [round(x, 4), round(y, 4)]
        for x, y in (grid_to_world(r, c, meta) for r, c in region.frontier_cells)
    ]
    return {
        "label": outcome.formal_region_label or f"GQ_{rank_index + 1:04d}",
        "internal_region_id": outcome.formal_region_id or region.region_id,
        "track_id": region.track_id,
        "region_source": REGION_SOURCE_QWEN_GLOBAL,
        "qwen_proposal_id": proposal.proposal_id,
        "qwen_proposal_rank": proposal.rank,
        "proposal_validated": True,
        "proposal_validation_passed": outcome.validation_passed,
        "proposal_validation_errors": list(outcome.rejection_reasons),
        "direction": proposal.direction_hint or region.direction_label,
        "distance_m": region.distance_to_robot_m,
        "unknown_gain_cells": region.unknown_gain_cells,
        "unknown_gain_ratio": region.unknown_gain_ratio,
        "minimum_clearance_m": region.minimum_clearance_m,
        "mean_clearance_m": region.mean_clearance_m,
        "frontier_cell_count": region.frontier_cell_count,
        "frontier_cells_grid": [[int(r), int(c)] for r, c in region.frontier_cells],
        "frontier_points_map": frontier_points,
        "centroid_x": region.centroid_x,
        "centroid_y": region.centroid_y,
        "bbox_grid": [region.row_min, region.row_max, region.col_min, region.col_max],
        "geo_score": region.geo_score,
        "geo_rank": region.geo_rank,
        "stable": region.stable,
        "snapshot_eligible": region.snapshot_eligible,
        "blacklisted": region.blacklisted,
        "trajectory_novelty_score": region.trajectory_novelty_score,
        "trajectory_revisit_penalty": region.trajectory_revisit_penalty,
        "path_checked": False,
        "reachable": None,
        "map_image_center_u": proposal.map_image_center_u,
        "map_image_center_v": proposal.map_image_center_v,
        "supporting_view_ids": list(proposal.supporting_view_ids),
        "confidence": proposal.confidence,
        "reason_code": proposal.reason_code,
    }


def build_selected_region_geometry_from_global(
    region_entry: Dict[str, Any],
    snapshot_id: str,
    *,
    contract_cfg: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    from src.planning.exploration_contracts import (  # noqa: WPS433
        build_region_geometry_fingerprint_from_entry,
    )

    label = str(region_entry.get("label", ""))
    entry = {
        "internal_region_id": region_entry.get("internal_region_id", ""),
        "track_id": region_entry.get("track_id", ""),
        "centroid_x": region_entry.get("centroid_x", 0.0),
        "centroid_y": region_entry.get("centroid_y", 0.0),
        "frontier_cells_grid": region_entry.get("frontier_cells_grid", []),
        "frontier_points_map": region_entry.get("frontier_points_map", []),
        "bbox_grid": region_entry.get("bbox_grid", []),
        "unknown_gain_cells": region_entry.get("unknown_gain_cells", 0),
        "minimum_clearance_m": region_entry.get("minimum_clearance_m", 0.0),
        "trajectory_novelty_score": region_entry.get("trajectory_novelty_score", 1.0),
        "trajectory_revisit_penalty": region_entry.get("trajectory_revisit_penalty", 0.0),
        "geo_score": region_entry.get("geo_score", 0.0),
        "stable": region_entry.get("stable", False),
        "snapshot_eligible": region_entry.get("snapshot_eligible", True),
        "blacklisted": region_entry.get("blacklisted", False),
        "region_source": REGION_SOURCE_QWEN_GLOBAL,
        "qwen_proposal_id": region_entry.get("qwen_proposal_id", ""),
    }
    entry["region_geometry_fingerprint"] = build_region_geometry_fingerprint_from_entry(
        snapshot_id, label, entry, cfg=contract_cfg
    )
    return entry


def deep_copy_map_data(data: np.ndarray) -> np.ndarray:
    return copy.deepcopy(data)
