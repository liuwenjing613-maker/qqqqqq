#!/usr/bin/env python3
"""Global map region proposal — pure data, prompts, and response validation (no ROS/motion)."""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

STRATEGY_CANDIDATE_RANKING = "CANDIDATE_RANKING"
STRATEGY_GLOBAL_REGION_PROPOSAL = "GLOBAL_REGION_PROPOSAL"
ALLOWED_STRATEGIES = frozenset({STRATEGY_CANDIDATE_RANKING, STRATEGY_GLOBAL_REGION_PROPOSAL})

COORDINATE_SPACE_NORMALIZED_MAP_VIEWPORT = "NORMALIZED_MAP_VIEWPORT"
REGION_TYPE_FRONTIER_EXPLORATION = "FRONTIER_EXPLORATION_REGION"

FALLBACK_USE_CANDIDATE_RANKING = "USE_CANDIDATE_RANKING"
FALLBACK_RESCAN = "RESCAN"

GLOBAL_FORBIDDEN_FIELDS: frozenset[str] = frozenset(
    {
        "x",
        "y",
        "yaw",
        "map_x",
        "map_y",
        "grid_x",
        "grid_y",
        "speed",
        "velocity",
        "linear",
        "angular",
        "cmd_vel",
        "goal_pose",
        "navigation_goal",
        "path_checked",
        "reachable",
        "waypoint",
        "linear_velocity",
        "angular_velocity",
        "turn_angle",
        "drive_distance",
    }
)

# Top-level keys allowed to contain u/v subfields
ALLOWED_UV_CONTAINERS = frozenset(
    {"map_image_center", "map_image_bbox", "center", "bbox"}
)


@dataclass
class GlobalRegionProposal:
    proposal_id: str
    rank: int
    region_type: str
    map_image_center_u: float
    map_image_center_v: float
    bbox_u_min: float
    bbox_v_min: float
    bbox_u_max: float
    bbox_v_max: float
    direction_hint: str = ""
    supporting_view_ids: List[str] = field(default_factory=list)
    confidence: float = 0.0
    reason_code: str = ""
    map_evidence: List[str] = field(default_factory=list)
    visual_evidence: List[Dict[str, Any]] = field(default_factory=list)
    risk_flags: List[str] = field(default_factory=list)
    proposal_validated: bool = False
    path_checked: bool = False
    reachable: Optional[bool] = None


@dataclass
class GlobalRegionProposalResponse:
    snapshot_id: str
    visual_context_id: str
    selection_strategy: str
    task_interpretation: str
    ranked_region_proposals: List[GlobalRegionProposal]
    fallback_recommendation: str = ""


@dataclass
class GlobalRegionPromptContext:
    snapshot_id: str
    visual_context_id: str
    target_instruction: str
    robot_pose: Dict[str, float]
    trajectory_summary: Dict[str, Any]
    map_metadata_summary: Dict[str, Any]
    view_ids: List[str]
    map_render_metadata: Dict[str, Any]
    decision_board_file: str = ""
    allow_map_only: bool = True


@dataclass
class GlobalRegionValidationResult:
    valid: bool
    errors: List[str] = field(default_factory=list)
    parsed: Optional[Dict[str, Any]] = None
    response: Optional[GlobalRegionProposalResponse] = None


def _region_selection_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return dict(cfg.get("region_selection", {}))


def _global_cfg(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return dict(cfg.get("global_region_proposal", {}))


def validate_region_selection_config(cfg: Dict[str, Any]) -> List[str]:
    """Validate strategy configuration; unknown strings must error when strict."""
    errors: List[str] = []
    rsc = _region_selection_cfg(cfg)
    gcfg = _global_cfg(cfg)
    strict = bool(rsc.get("strict_strategy_validation", True))

    strategy = str(rsc.get("strategy", STRATEGY_CANDIDATE_RANKING))
    fallback = str(rsc.get("fallback_strategy", STRATEGY_CANDIDATE_RANKING))

    if strategy not in ALLOWED_STRATEGIES:
        errors.append(f"region_selection.strategy invalid: {strategy}")
    if fallback not in ALLOWED_STRATEGIES:
        errors.append(f"region_selection.fallback_strategy invalid: {fallback}")

    if strategy == STRATEGY_GLOBAL_REGION_PROPOSAL:
        if not bool(gcfg.get("enabled", True)):
            errors.append("global_region_proposal.enabled must be true for GLOBAL_REGION_PROPOSAL")
        coord = str(gcfg.get("coordinate_space", COORDINATE_SPACE_NORMALIZED_MAP_VIEWPORT))
        if coord != COORDINATE_SPACE_NORMALIZED_MAP_VIEWPORT:
            errors.append(f"global_region_proposal.coordinate_space invalid: {coord}")

    for ratio_key in (
        "minimum_bbox_width_normalized",
        "minimum_bbox_height_normalized",
        "maximum_bbox_width_normalized",
        "maximum_bbox_height_normalized",
        "minimum_frontier_overlap_ratio",
        "maximum_occupied_ratio",
    ):
        if ratio_key in gcfg:
            val = float(gcfg[ratio_key])
            if val < 0.0 or val > 1.0:
                errors.append(f"global_region_proposal.{ratio_key} must be in [0,1]")

    if "max_frontier_snap_distance_m" in gcfg:
        if float(gcfg["max_frontier_snap_distance_m"]) <= 0:
            errors.append("global_region_proposal.max_frontier_snap_distance_m must be > 0")

    if strict and errors:
        return errors
    return errors


def resolve_effective_strategy(cfg: Dict[str, Any]) -> str:
    validate_region_selection_config(cfg)  # raises via caller checking errors
    return str(_region_selection_cfg(cfg).get("strategy", STRATEGY_CANDIDATE_RANKING))


def build_global_region_proposal_prompt(ctx: GlobalRegionPromptContext, cfg: Dict[str, Any]) -> str:
    """Build dedicated global region proposal prompt (not mixed with candidate ranking)."""
    gcfg = _global_cfg(cfg)
    max_props = int(gcfg.get("max_ranked_proposals", 3))
    robot = ctx.robot_pose
    traj = ctx.trajectory_summary
    meta = ctx.map_metadata_summary
    views = ", ".join(ctx.view_ids) if ctx.view_ids else "VIEW_000 ... VIEW_315"

    return f"""You are the high-level semantic exploration planner for a mobile robot.

Your responsibility is to inspect the complete occupancy map, the robot's
current position, traveled path, visited corridor, observation poses, and
the full 360-degree visual context, then propose the next map REGION that
is most valuable to explore.

You are not a motion controller.
You do not generate robot commands.
You do not generate a final navigation waypoint.
You only identify and rank exploration regions on the supplied map.

Map interpretation:

- FREE represents known traversable space.
- OCCUPIED represents walls or obstacles.
- UNKNOWN represents space that has not yet been mapped.
- FRONTIER BOUNDARY is the boundary between known FREE space and UNKNOWN space.
- ROBOT marks the robot's current map position and orientation.
- TRAVELED PATH shows where the robot has actually moved.
- VISITED CORRIDOR shows areas already covered by the robot's path.
- OBSERVATION POSE marks positions where the robot has already performed
  an observation or scan.

The map contains a normalized U/V reference grid:
- u=0.0 is the left edge of the MAP CONTENT.
- u=1.0 is the right edge of the MAP CONTENT.
- v=0.0 is the top edge of the MAP CONTENT.
- v=1.0 is the bottom edge of the MAP CONTENT.

Coordinates refer only to the map content panel, not the full decision-board image.

The visual panel contains eight direction-labeled images:

{views}

The view IDs describe relative directions captured around the robot.

Use the images only as visual semantic evidence.
Do not invent a correspondence between a map region and a view unless the
direction and supplied metadata support it.

Do not claim that a route is open only because an image looks visually open.
Map geometry and later path validation remain authoritative.

Your task is to select a REGION, not a navigation point.

A good exploration region should normally:

1. Be close to a real boundary between known FREE space and UNKNOWN space.
2. Represent a meaningful entrance, corridor extension, doorway, room opening,
   branch, or unexplored continuation.
3. Contain useful unknown-space information gain.
4. Be reasonably separated from walls and large occupied areas.
5. Be less covered by TRAVELED PATH and VISITED CORRIDOR when safety and
   information gain are otherwise similar.
6. Avoid regions that were very recently observed.
7. Prefer regions that may be relevant to the target instruction when supported
   by the 360-degree visual evidence.
8. Avoid tiny isolated map noise, narrow slivers, map borders, wall faces,
   fully enclosed unknown islands, and regions behind an obvious wall.

Use this strict decision order:

1. Reject obviously invalid map areas:
   - map legends or padding,
   - outside the map,
   - occupied wall areas,
   - tiny map noise,
   - isolated unknown areas with no visible FREE boundary.

2. Identify meaningful FREE-to-UNKNOWN boundaries.

3. Consider geometric exploration value:
   - unknown extent,
   - apparent entrance width,
   - distance from the robot,
   - surrounding free-space quality.

4. Consider exploration history:
   - traveled path,
   - visited corridor,
   - observation poses.

5. Consider 360-degree visual semantic evidence.

6. Rank up to {max_props} best regions.

Map safety and valid frontier structure take priority over semantic interest.
A visually interesting direction must not override an obviously invalid map region.

When an obstacle or wall occupies the central direction:
- Do not select the obstacle surface.
- Examine both sides of the obstacle.
- Prefer a side that forms a continuous known-FREE to UNKNOWN opening.

Exploration-history rules:
- Prefer less-traveled regions when geometric safety and information gain are similar.
- Do not reject a region merely because an old path passes nearby.
- Strongly penalize a region only when the region itself has already been
  repeatedly observed and offers little additional unknown information.

Never select:
- a wall surface,
- the center of an occupied block,
- a large uniform occupied or wall region,
- a tiny isolated unknown pixel group,
- a map border artifact,
- an area fully separated from known free space,
- the robot's immediate footprint.

For each proposal, output:
- one normalized center (u,v),
- one normalized bounding box,
- supporting view IDs when applicable.

The center and bounding box identify an exploration REGION only.
They are not a robot waypoint, Nav2 goal, or confirmed reachable coordinates.

All values must be between 0.0 and 1.0.
The bounding box must satisfy:
u_min < center_u < u_max
v_min < center_v < v_max

If no defensible region exists:
- return an empty ranked_region_proposals array,
- set fallback_recommendation to USE_CANDIDATE_RANKING or RESCAN.

Return only valid JSON. No markdown. No commentary outside JSON.
Do not output pixel coordinates.
Do not output map x/y coordinates.
Do not output robot speed, turn angle, drive distance, /cmd_vel, or Nav2 goal.
Do not claim path_checked or reachable.

Context:
snapshot_id={ctx.snapshot_id}
visual_context_id={ctx.visual_context_id}
target_instruction={ctx.target_instruction}
robot_pose={json.dumps(robot, ensure_ascii=False)}
trajectory_summary={json.dumps(traj, ensure_ascii=False)}
map_metadata={json.dumps(meta, ensure_ascii=False)}
coordinate_space={COORDINATE_SPACE_NORMALIZED_MAP_VIEWPORT}
decision_board_file={ctx.decision_board_file}

Required JSON schema:
{{
  "snapshot_id": "{ctx.snapshot_id}",
  "visual_context_id": "{ctx.visual_context_id}",
  "selection_strategy": "GLOBAL_REGION_PROPOSAL",
  "task_interpretation": "...",
  "ranked_region_proposals": [
    {{
      "proposal_id": "GP_1",
      "rank": 1,
      "region_type": "FRONTIER_EXPLORATION_REGION",
      "map_image_center": {{"u": 0.5, "v": 0.5}},
      "map_image_bbox": {{"u_min": 0.4, "v_min": 0.4, "u_max": 0.6, "v_max": 0.6}},
      "direction_hint": "RIGHT_FRONT",
      "supporting_view_ids": ["VIEW_090"],
      "confidence": 0.8,
      "reason_code": "UNVISITED_OPEN_FRONTIER",
      "map_evidence": ["..."],
      "visual_evidence": [{{"view_id": "VIEW_090", "observation": "..."}}],
      "risk_flags": ["区域尚未经过栅格前沿验证"]
    }}
  ],
  "fallback_recommendation": "USE_CANDIDATE_RANKING"
}}
"""


def _collect_global_forbidden_fields(obj: Any, path: str = "") -> List[str]:
    found: List[str] = []
    if isinstance(obj, dict):
        for key, val in obj.items():
            key_l = str(key).lower()
            full = f"{path}.{key}" if path else key
            if key_l in GLOBAL_FORBIDDEN_FIELDS:
                if key_l in ("u", "v") and any(p in path for p in ALLOWED_UV_CONTAINERS):
                    pass
                else:
                    found.append(full)
            found.extend(_collect_global_forbidden_fields(val, full))
    elif isinstance(obj, list):
        for i, item in enumerate(obj):
            found.extend(_collect_global_forbidden_fields(item, f"{path}[{i}]"))
    return found


def _parse_proposal(item: Dict[str, Any]) -> GlobalRegionProposal:
    center = item.get("map_image_center") or {}
    bbox = item.get("map_image_bbox") or {}
    return GlobalRegionProposal(
        proposal_id=str(item.get("proposal_id", "")),
        rank=int(item.get("rank", 0)),
        region_type=str(item.get("region_type", REGION_TYPE_FRONTIER_EXPLORATION)),
        map_image_center_u=float(center.get("u", -1)),
        map_image_center_v=float(center.get("v", -1)),
        bbox_u_min=float(bbox.get("u_min", -1)),
        bbox_v_min=float(bbox.get("v_min", -1)),
        bbox_u_max=float(bbox.get("u_max", -1)),
        bbox_v_max=float(bbox.get("v_max", -1)),
        direction_hint=str(item.get("direction_hint", "")),
        supporting_view_ids=[str(v) for v in item.get("supporting_view_ids", [])],
        confidence=float(item.get("confidence", 0.0)),
        reason_code=str(item.get("reason_code", "")),
        map_evidence=[str(x) for x in item.get("map_evidence", [])],
        visual_evidence=list(item.get("visual_evidence", [])),
        risk_flags=[str(x) for x in item.get("risk_flags", [])],
        proposal_validated=False,
        path_checked=False,
        reachable=None,
    )


def validate_global_region_proposal_response(
    raw_response: str,
    *,
    expected_snapshot_id: str,
    expected_visual_context_id: str,
    valid_view_ids: Set[str],
    cfg: Dict[str, Any],
) -> GlobalRegionValidationResult:
    """Validate Qwen global proposal JSON."""
    from src.vlm.qwen_region_selector_core import extract_json_object  # noqa: WPS433

    errors: List[str] = []
    try:
        parsed = extract_json_object(raw_response)
    except ValueError as exc:
        return GlobalRegionValidationResult(valid=False, errors=[str(exc)])

    forbidden = _collect_global_forbidden_fields(parsed)
    if forbidden:
        errors.append("GLOBAL_FORBIDDEN_FIELD_PRESENT")
        errors.extend(forbidden)

    if str(parsed.get("snapshot_id", "")) != expected_snapshot_id:
        errors.append("GLOBAL_SNAPSHOT_ID_MISMATCH")

    if expected_visual_context_id:
        if str(parsed.get("visual_context_id", "")) != expected_visual_context_id:
            errors.append("GLOBAL_VISUAL_CONTEXT_ID_MISMATCH")

    if str(parsed.get("selection_strategy", "")) != STRATEGY_GLOBAL_REGION_PROPOSAL:
        errors.append("GLOBAL_SELECTION_STRATEGY_MISMATCH")

    gcfg = _global_cfg(cfg)
    max_props = int(gcfg.get("max_ranked_proposals", 3))
    proposals_raw = parsed.get("ranked_region_proposals")
    if proposals_raw is None:
        errors.append("GLOBAL_PROPOSALS_MISSING")
        proposals_raw = []
    if not isinstance(proposals_raw, list):
        errors.append("GLOBAL_PROPOSALS_INVALID")
        proposals_raw = []

    if len(proposals_raw) > max_props:
        errors.append("GLOBAL_PROPOSAL_COUNT_EXCEEDED")

    proposal_ids: Set[str] = set()
    ranks: List[int] = []
    proposals: List[GlobalRegionProposal] = []

    min_w = float(gcfg.get("minimum_bbox_width_normalized", 0.04))
    min_h = float(gcfg.get("minimum_bbox_height_normalized", 0.04))
    max_w = float(gcfg.get("maximum_bbox_width_normalized", 0.45))
    max_h = float(gcfg.get("maximum_bbox_height_normalized", 0.45))

    for item in proposals_raw:
        if not isinstance(item, dict):
            errors.append("GLOBAL_PROPOSAL_INVALID")
            continue
        prop = _parse_proposal(item)
        if not prop.proposal_id:
            errors.append("GLOBAL_PROPOSAL_ID_MISSING")
        elif prop.proposal_id in proposal_ids:
            errors.append("GLOBAL_PROPOSAL_ID_DUPLICATE")
        proposal_ids.add(prop.proposal_id)

        if prop.rank <= 0:
            errors.append("GLOBAL_PROPOSAL_RANK_INVALID")
        ranks.append(prop.rank)

        if prop.region_type != REGION_TYPE_FRONTIER_EXPLORATION:
            errors.append("GLOBAL_PROPOSAL_REGION_TYPE_INVALID")

        for uv in (prop.map_image_center_u, prop.map_image_center_v):
            if uv < 0.0 or uv > 1.0:
                errors.append("GLOBAL_PROPOSAL_CENTER_OUT_OF_RANGE")
        for uv in (prop.bbox_u_min, prop.bbox_v_min, prop.bbox_u_max, prop.bbox_v_max):
            if uv < 0.0 or uv > 1.0:
                errors.append("GLOBAL_PROPOSAL_BBOX_OUT_OF_RANGE")

        if not (prop.bbox_u_min < prop.map_image_center_u < prop.bbox_u_max):
            errors.append("GLOBAL_PROPOSAL_CENTER_OUTSIDE_BBOX_U")
        if not (prop.bbox_v_min < prop.map_image_center_v < prop.bbox_v_max):
            errors.append("GLOBAL_PROPOSAL_CENTER_OUTSIDE_BBOX_V")
        if prop.bbox_u_max <= prop.bbox_u_min or prop.bbox_v_max <= prop.bbox_v_min:
            errors.append("GLOBAL_PROPOSAL_BBOX_ORDER_INVALID")

        bw = prop.bbox_u_max - prop.bbox_u_min
        bh = prop.bbox_v_max - prop.bbox_v_min
        if bw < min_w or bh < min_h:
            errors.append("GLOBAL_PROPOSAL_BBOX_TOO_SMALL")
        if bw > max_w or bh > max_h:
            errors.append("GLOBAL_PROPOSAL_BBOX_TOO_LARGE")

        if prop.confidence < 0.0 or prop.confidence > 1.0:
            errors.append("GLOBAL_PROPOSAL_CONFIDENCE_INVALID")

        for vid in prop.supporting_view_ids:
            if vid not in valid_view_ids:
                errors.append(f"GLOBAL_PROPOSAL_VIEW_INVALID view={vid}")

        proposals.append(prop)

    if ranks and sorted(ranks) != list(range(1, len(ranks) + 1)):
        errors.append("GLOBAL_PROPOSAL_RANK_NOT_CONTIGUOUS")
    if len(ranks) != len(set(ranks)):
        errors.append("GLOBAL_PROPOSAL_RANK_DUPLICATE")

    fb = str(parsed.get("fallback_recommendation", ""))
    if not proposals_raw and fb not in (FALLBACK_USE_CANDIDATE_RANKING, FALLBACK_RESCAN, ""):
        errors.append("GLOBAL_FALLBACK_RECOMMENDATION_INVALID")

    response = GlobalRegionProposalResponse(
        snapshot_id=str(parsed.get("snapshot_id", "")),
        visual_context_id=str(parsed.get("visual_context_id", "")),
        selection_strategy=STRATEGY_GLOBAL_REGION_PROPOSAL,
        task_interpretation=str(parsed.get("task_interpretation", "")),
        ranked_region_proposals=sorted(proposals, key=lambda p: p.rank),
        fallback_recommendation=fb,
    )
    return GlobalRegionValidationResult(
        valid=len(errors) == 0,
        errors=errors,
        parsed=parsed,
        response=response,
    )


def global_response_to_dict(response: GlobalRegionProposalResponse) -> Dict[str, Any]:
    return {
        "snapshot_id": response.snapshot_id,
        "visual_context_id": response.visual_context_id,
        "selection_strategy": response.selection_strategy,
        "task_interpretation": response.task_interpretation,
        "ranked_region_proposals": [
            {
                "proposal_id": p.proposal_id,
                "rank": p.rank,
                "region_type": p.region_type,
                "map_image_center": {"u": p.map_image_center_u, "v": p.map_image_center_v},
                "map_image_bbox": {
                    "u_min": p.bbox_u_min,
                    "v_min": p.bbox_v_min,
                    "u_max": p.bbox_u_max,
                    "v_max": p.bbox_v_max,
                },
                "direction_hint": p.direction_hint,
                "supporting_view_ids": list(p.supporting_view_ids),
                "confidence": p.confidence,
                "reason_code": p.reason_code,
                "map_evidence": list(p.map_evidence),
                "visual_evidence": list(p.visual_evidence),
                "risk_flags": list(p.risk_flags),
                "proposal_validated": p.proposal_validated,
                "path_checked": False,
                "reachable": None,
            }
            for p in response.ranked_region_proposals
        ],
        "fallback_recommendation": response.fallback_recommendation,
    }


def build_global_prompt_context_from_snapshot(
    raw_snapshot: Dict[str, Any],
    *,
    target_instruction: str,
    visual_context_id: str = "",
    map_render_metadata: Optional[Dict[str, Any]] = None,
    decision_board_file: str = "",
    view_ids: Optional[Sequence[str]] = None,
) -> GlobalRegionPromptContext:
    traj_meta = raw_snapshot.get("trajectory_meta") or {}
    return GlobalRegionPromptContext(
        snapshot_id=str(raw_snapshot.get("snapshot_id", "")),
        visual_context_id=visual_context_id,
        target_instruction=target_instruction,
        robot_pose=dict(raw_snapshot.get("robot_pose", {})),
        trajectory_summary={
            "trajectory_session_id": traj_meta.get("trajectory_session_id", ""),
            "trajectory_length_m": traj_meta.get("trajectory_length_m", 0.0),
            "vertex_count": traj_meta.get("trajectory_vertex_count", 0),
            "raw_sample_count": traj_meta.get("trajectory_raw_sample_count", 0),
        },
        map_metadata_summary=dict(raw_snapshot.get("map_metadata", {})),
        view_ids=list(view_ids or []),
        map_render_metadata=dict(map_render_metadata or raw_snapshot.get("map_render_metadata", {})),
        decision_board_file=decision_board_file,
    )
