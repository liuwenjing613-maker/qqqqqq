#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Live session: 真实小车位姿 + v6 程序候选 + Qwen 第二阶段选点 + map 坐标输出。

跨进程约定：
  - candidate_bundle.json 只含可序列化字段（无 _runtime）
  - 第二进程必须 rebuild_runtime_context()，禁止依赖 JSON 中不存在的 _runtime
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from qwen_map_goal_utils import (  # noqa: E402
    MapYamlMeta,
    atomic_write_json,
    build_free_mask_from_gray,
    build_visited_distance_field,
    compute_bundle_fingerprint,
    file_fingerprint,
    load_map_yaml_meta,
    paint_visited_on_bgr,
    parse_trajectory_line_bgr,
    pixel_to_map_xy,
    resolve_qwen_map_yaml,
    sha256_file,
    visited_mask_from_gray,
    world_to_pixel,
    write_foxglove_candidates_json,
    write_navigation_goal_proposal,
)


def _load_v6_module():
    path = SCRIPT_DIR / "qwen_five_frontier_cases_clean_output_v6.py"
    spec = importlib.util.spec_from_file_location("qwen_v6", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 v6 模块：{path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["qwen_v6"] = mod
    spec.loader.exec_module(mod)
    return mod


@dataclass
class RuntimeContext:
    """当前进程内存对象；禁止写入 candidate_bundle.json。"""

    semantic: Any
    pose: Any
    case_candidates: List[Any]
    nav_labels: Any
    visited_dist_field: Any
    v6_args: Any
    meta: MapYamlMeta
    longest: int
    map_yaml: Path
    pgm_path: Path
    rx: float
    ry: float
    yaw_rad: float
    yaw_deg: float
    robot_px: int
    robot_py: int
    candidates_path: Path
    foxglove_candidates_json: Path
    unknown_value: int
    vertices: List[Tuple[float, float]]
    unknown: Any
    blocked: Any
    navigable: Any


def load_pose_json(path: Path) -> Tuple[float, float, float]:
    data = json.loads(path.read_text(encoding="utf-8"))
    x = float(data.get("x", 0.0))
    y = float(data.get("y", 0.0))
    if "yaw" in data:
        yaw_rad = float(data["yaw"])
    else:
        qz = float(data.get("qz", 0.0))
        qw = float(data.get("qw", 1.0))
        qx = float(data.get("qx", 0.0))
        qy = float(data.get("qy", 0.0))
        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return x, y, yaw_rad


def load_trajectory_vertices(path: Path) -> List[Tuple[float, float]]:
    if not path.is_file():
        return []
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return []
    out: List[Tuple[float, float]] = []
    for v in raw.get("vertices") or []:
        if isinstance(v, dict):
            out.append((float(v.get("x", 0.0)), float(v.get("y", 0.0))))
        elif isinstance(v, (list, tuple)) and len(v) >= 2:
            out.append((float(v[0]), float(v[1])))
    return out


def resolve_robot_pixel(x, y, meta, navigable, search_radius=12):
    h, w = meta.height, meta.width
    px, py = world_to_pixel(x, y, meta)
    px = int(np.clip(px, 0, w - 1))
    py = int(np.clip(py, 0, h - 1))
    if navigable[py, px]:
        return px, py
    best = None
    for radius in range(1, search_radius + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                nx, ny = px + dx, py + dy
                if 0 <= nx < w and 0 <= ny < h and navigable[ny, nx]:
                    dist = math.hypot(dx, dy)
                    if best is None or dist < best[2]:
                        best = (nx, ny, dist)
        if best is not None:
            return best[0], best[1]
    raise RuntimeError(f"机器人 ({x:.3f},{y:.3f}) 附近找不到可通行白区")


def resolve_visited_mask(gray_nav, meta, map_yaml, qwen_map_yaml, traj_json, *, corridor_radius_m=0.35):
    h, w = gray_nav.shape
    qwen_yaml = qwen_map_yaml
    if qwen_yaml is None:
        auto = resolve_qwen_map_yaml(map_yaml)
        if auto.is_file():
            qwen_yaml = auto
    if qwen_yaml is not None and qwen_yaml.is_file():
        qdata = yaml.safe_load(qwen_yaml.read_text(encoding="utf-8"))
        qpgm = qwen_yaml.parent / str(qdata.get("image", ""))
        if not qpgm.is_file():
            qpgm = qwen_yaml.with_suffix(".pgm")
        qgray = cv2.imread(str(qpgm), cv2.IMREAD_GRAYSCALE)
        if qgray is not None and qgray.shape == gray_nav.shape:
            return visited_mask_from_gray(qgray)
    if traj_json is not None and traj_json.is_file():
        from src.planning.robot_trajectory_store import rasterize_visited_corridor, vertices_from_xy

        free_mask = build_free_mask_from_gray(gray_nav)
        verts = vertices_from_xy(load_trajectory_vertices(traj_json))
        if verts:
            visited_flat = rasterize_visited_corridor(
                width=meta.width,
                height=meta.height,
                resolution=meta.resolution,
                origin_x=meta.origin_x,
                origin_y=meta.origin_y,
                vertices=verts,
                corridor_radius_m=corridor_radius_m,
                free_mask=free_mask,
            )
            mask = np.zeros((h, w), dtype=bool)
            for idx, val in enumerate(visited_flat):
                if val >= 100:
                    r, c = divmod(idx, w)
                    mask[r, c] = True
            return mask
    return np.zeros((h, w), dtype=bool)


def draw_trajectory_overlay(semantic, vertices, meta):
    if len(vertices) < 2:
        return
    pts = []
    h, w = meta.height, meta.width
    for vx, vy in vertices:
        px, py = world_to_pixel(vx, vy, meta)
        pts.append((int(np.clip(px, 0, w - 1)), int(np.clip(py, 0, h - 1))))
    color = parse_trajectory_line_bgr()
    cv2.polylines(
        semantic,
        [np.array(pts, dtype=np.int32)],
        isClosed=False,
        color=color,
        thickness=max(2, int(round(max(h, w) * 0.003))),
        lineType=cv2.LINE_AA,
    )


def snapshot_mtime(path: Optional[Path]) -> Optional[float]:
    if path is None or not path.is_file():
        return None
    return float(path.stat().st_mtime)


def check_snapshot_time_delta(
    map_path: Path,
    pose_path: Path,
    traj_path: Optional[Path],
) -> Tuple[float, Dict[str, Any]]:
    map_t = snapshot_mtime(map_path)
    pose_t = snapshot_mtime(pose_path)
    traj_t = snapshot_mtime(traj_path)
    times = [t for t in (map_t, pose_t, traj_t) if t is not None]
    delta = (max(times) - min(times)) if len(times) >= 2 else 0.0
    max_delta = float(os.environ.get("SNAPSHOT_MAX_TIME_DELTA_S", "30"))
    info = {
        "map_saved_at": map_t,
        "pose_saved_at": pose_t,
        "trajectory_saved_at": traj_t,
        "snapshot_time_delta_s": delta,
        "snapshot_max_time_delta_s": max_delta,
    }
    if delta > max_delta:
        raise RuntimeError(
            f"会话快照时间差过大: {delta:.1f}s > {max_delta:.1f}s "
            f"(map/pose/trajectory 必须在同一 OK 冻结窗口内)"
        )
    return delta, info


def build_nav_goal_payload(**kwargs) -> Dict[str, Any]:
    session_id = kwargs["session_id"]
    map_yaml = kwargs["map_yaml"]
    meta = kwargs["meta"]
    robot_map = kwargs["robot_map"]
    chosen_pixel = kwargs["chosen_pixel"]
    result_meta = kwargs["result_meta"]
    output_dir = kwargs["output_dir"]
    bundle = kwargs["bundle"]
    bundle_path = kwargs["bundle_path"]
    goal_x, goal_y = pixel_to_map_xy(chosen_pixel[0], chosen_pixel[1], meta)
    robot_x = robot_map["x"]
    robot_y = robot_map["y"]
    heading = math.atan2(goal_y - robot_y, goal_x - robot_x)
    h, w = meta.height, meta.width
    selected_id = int(result_meta["selected_local_id"])
    return {
        "schema_version": "qwen_live_session_nav_goal_v1",
        "session_id": session_id,
        "candidate_bundle_path": str(bundle_path),
        "bundle_fingerprint": bundle.get("bundle_fingerprint"),
        "selected_candidate_id": selected_id,
        "selected_candidate_pixel": {"x": int(chosen_pixel[0]), "y": int(chosen_pixel[1])},
        "selected_candidate_map": {"x": goal_x, "y": goal_y},
        "map_frame": meta.frame_id,
        "map_yaml": str(map_yaml.resolve()),
        "robot_pose_map": {
            "x": robot_x,
            "y": robot_y,
            "z": 0.0,
            "yaw_rad": robot_map["yaw_rad"],
            "yaw_deg": robot_map["yaw_deg"],
            "pixel_x": robot_map["pixel_x"],
            "pixel_y": robot_map["pixel_y"],
            "u": robot_map["pixel_x"] / max(1, w - 1),
            "v": robot_map["pixel_y"] / max(1, h - 1),
        },
        "selection_status": "REGION_PROPOSED",
        "goal_pose_map": {
            "x": goal_x,
            "y": goal_y,
            "z": 0.0,
            "yaw_rad": heading,
            "yaw_deg": math.degrees(heading),
            "pixel_x": chosen_pixel[0],
            "pixel_y": chosen_pixel[1],
            "u": chosen_pixel[0] / max(1, w - 1),
            "v": chosen_pixel[1] / max(1, h - 1),
        },
        "region_center_map": {"x": goal_x, "y": goal_y, "z": 0.0},
        "qwen_selection": result_meta,
        "planner_output_dir": str(output_dir.resolve()),
        "safety": {
            "candidate_geometry_validated": True,
            "candidate_connected_to_robot": True,
            "path_checked": False,
            "reachability_validated": False,
            "ready_for_nav2": False,
            "note": "候选几何安全，但必须等待 Nav2 ComputePathToPose 成功。",
        },
    }


def bundle_inputs_fingerprint(map_yaml, pose_json, traj_json, qwen_map_yaml):
    items = {
        "map_yaml": file_fingerprint(map_yaml),
        "pose_json": file_fingerprint(pose_json),
        "trajectory_json": file_fingerprint(traj_json) if traj_json else {"exists": False},
        "qwen_map_yaml": file_fingerprint(qwen_map_yaml) if qwen_map_yaml else {"exists": False},
    }
    blob = json.dumps(items, sort_keys=True)
    return items, hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


def bundle_still_valid(bundle_path: Path, fingerprint: str) -> bool:
    if not bundle_path.is_file():
        return False
    try:
        data = json.loads(bundle_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return False
    if "_runtime" in data:
        return False
    return data.get("input_fingerprint") == fingerprint


def candidate_to_dict(c, meta, pose, longest, attrs: Optional[Dict[str, Any]] = None):
    mx, my = pixel_to_map_xy(c.x, c.y, meta)
    w, h = meta.width, meta.height
    dx = float(c.x - pose.x)
    dy = float(c.y - pose.y)
    distance_px = math.hypot(dx, dy)
    heading_goal = math.degrees(math.atan2(dy, dx)) if distance_px > 1e-6 else pose.yaw_deg
    heading_delta = ((heading_goal - pose.yaw_deg + 180.0) % 360.0) - 180.0
    out = {
        "candidate_id": None,
        "local_id": None,
        "global_id": int(c.global_id),
        "pixel_x": int(c.x),
        "pixel_y": int(c.y),
        "normalized_u": int(c.x) / max(1, w - 1),
        "normalized_v": int(c.y) / max(1, h - 1),
        "map_x": mx,
        "map_y": my,
        "distance_m": distance_px * meta.resolution,
        "heading_delta_deg": float(heading_delta),
        "obstacle_clearance_px": float(c.obstacle_clearance_px),
        "obstacle_clearance_m": float(c.obstacle_clearance_px) * meta.resolution,
        "unknown_distance_px": float(c.unknown_distance_px),
        "unknown_area_px": int(c.unknown_area_px),
        "component_id": int(c.component_id),
        "unexplored_gain": 0,
        "distance_to_visited_m": None,
        "inside_visited_corridor": False,
        "geometry_score": float(c.obstacle_clearance_px),
    }
    if attrs:
        out.update(attrs)
        if "geometry_score" not in attrs:
            out["geometry_score"] = float(c.obstacle_clearance_px) + 0.5 * float(
                attrs.get("unexplored_gain", 0)
            )
    return out


def dict_to_frontier_candidate(v6, d: Dict[str, Any]):
    return v6.FrontierCandidate(
        global_id=int(d["global_id"]),
        x=int(d["pixel_x"]),
        y=int(d["pixel_y"]),
        component_id=int(d.get("component_id", 0)),
        obstacle_clearance_px=float(d.get("obstacle_clearance_px", 0.0)),
        unknown_distance_px=float(d.get("unknown_distance_px", 0.0)),
        unknown_area_px=int(d.get("unknown_area_px", 0)),
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Live session: v6 候选 + Qwen 选点")
    p.add_argument("--map-yaml", required=True)
    p.add_argument("--qwen-map-yaml", default="")
    p.add_argument("--pose-json", required=True)
    p.add_argument("--trajectory-json", default="")
    p.add_argument("--output-dir", required=True)
    p.add_argument("--nav-goal-json", required=True)
    p.add_argument("--session-id", default="")
    p.add_argument("--task", default="优先探索尚未覆盖、最可能扩展地图的区域。")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--candidates-only", action="store_true")
    p.add_argument("--candidate-bundle", default="")
    p.add_argument("--force-regenerate-candidates", action="store_true")
    p.add_argument("--model", default=os.getenv("QWEN_MODEL", "qwen3-vl-flash"))
    p.add_argument("--base-url", default=os.getenv("QWEN_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"))
    p.add_argument("--api-key", default=None)
    p.add_argument("--max-candidates", type=int, default=int(os.getenv("QWEN_MAX_CANDIDATES", "8")))
    p.add_argument("--min-display-candidates", type=int, default=5)
    p.add_argument("--model-image-side", type=int, default=int(os.getenv("QWEN_MODEL_IMAGE_SIDE", "960")))
    p.add_argument("--temperature", type=float, default=0.0)
    p.add_argument("--timeout", type=float, default=float(os.getenv("QWEN_TIMEOUT_S", "60")))
    p.add_argument("--retries", type=int, default=int(os.getenv("QWEN_RETRIES", "1")))
    p.add_argument("--max-tokens", type=int, default=160)
    return p.parse_args()


def build_v6_args(live_args, pgm_path, output_dir):
    import argparse as ap

    return ap.Namespace(
        map=str(pgm_path),
        cases=1,
        output_dir=str(output_dir),
        model=live_args.model,
        base_url=live_args.base_url,
        api_key=live_args.api_key,
        seed=20260714,
        free_threshold=245,
        occupied_threshold=70,
        unknown_value=-1,
        unknown_tolerance=10,
        unknown_min_area=80,
        black_clearance_ratio=0.010,
        robot_clearance_ratio=0.012,
        frontier_gap_ratio=0.006,
        candidate_spacing_ratio=0.018,
        hard_black_clearance_m=float(
            os.getenv("CANDIDATE_HARD_BLACK_CLEARANCE_M", "0.35")
        ),
        hard_min_robot_distance_m=float(
            os.getenv("CANDIDATE_HARD_MIN_ROBOT_DISTANCE_M", "0.8")
        ),
        fallback_black_clearance_m=float(
            os.getenv("CANDIDATE_FALLBACK_BLACK_CLEARANCE_M", "0.2")
        ),
        fallback_min_robot_distance_m=float(
            os.getenv("CANDIDATE_FALLBACK_MIN_ROBOT_DISTANCE_M", "0.4")
        ),
        map_resolution=None,
        min_goal_distance_ratio=0.05,
        max_goal_distance_ratio=0.22,
        preferred_min_distance_ratio=0.07,
        preferred_max_distance_ratio=0.14,
        near_candidate_slack_ratio=0.05,
        front_cone_deg=90.0,
        min_display_candidates=max(5, int(live_args.min_display_candidates)),
        max_candidates=max(int(live_args.max_candidates), int(live_args.min_display_candidates)),
        pose_candidates=None,
        pose_temperature=0.85,
        model_image_side=live_args.model_image_side,
        temperature=live_args.temperature,
        max_tokens=live_args.max_tokens,
        timeout=live_args.timeout,
        retries=live_args.retries,
        dry_run=live_args.dry_run,
        hide_prompts=False,
        jpeg_quality=85,
    )


def _prepare_map_paths(args) -> Tuple[Path, Path, Optional[Path], Path, Path, Optional[Path]]:
    map_yaml = Path(args.map_yaml).expanduser().resolve()
    pose_json = Path(args.pose_json).expanduser().resolve()
    traj_json = Path(args.trajectory_json).expanduser().resolve() if args.trajectory_json else None
    output_dir = Path(args.output_dir).expanduser().resolve()
    nav_goal_json = Path(args.nav_goal_json).expanduser().resolve()
    qwen_map_yaml = Path(args.qwen_map_yaml).expanduser().resolve() if args.qwen_map_yaml else None
    return map_yaml, pose_json, traj_json, output_dir, nav_goal_json, qwen_map_yaml


def _generate_serializable_bundle(args, v6) -> Dict[str, Any]:
    map_yaml, pose_json, traj_json, output_dir, nav_goal_json, qwen_map_yaml = _prepare_map_paths(args)
    bundle_path = (
        Path(args.candidate_bundle).expanduser().resolve()
        if args.candidate_bundle
        else output_dir / "candidate_bundle.json"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    input_fps, fp_hash = bundle_inputs_fingerprint(map_yaml, pose_json, traj_json, qwen_map_yaml)
    _, snap_info = check_snapshot_time_delta(map_yaml, pose_json, traj_json)

    meta = load_map_yaml_meta(map_yaml)
    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    pgm_path = map_yaml.parent / str(map_data.get("image", ""))
    if not pgm_path.is_file():
        pgm_path = map_yaml.with_suffix(".pgm")
    gray = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise RuntimeError(f"无法读取地图：{pgm_path}")

    h, w = gray.shape
    if h != meta.height or w != meta.width:
        meta = MapYamlMeta(w, h, meta.resolution, meta.origin_x, meta.origin_y, meta.frame_id)

    v6_args = build_v6_args(args, pgm_path, output_dir)
    longest = max(h, w)
    v6_args.map_resolution = float(meta.resolution)
    v6_args.base_min_goal_distance_ratio = float(v6_args.min_goal_distance_ratio)
    v6_args.strict_black_clearance_m = float(v6_args.hard_black_clearance_m)
    v6_args.strict_min_robot_distance_m = float(v6_args.hard_min_robot_distance_m)
    rx, ry, yaw_rad = load_pose_json(pose_json)
    yaw_deg = math.degrees(yaw_rad)

    semantic, free, unknown, blocked, unknown_labels, unknown_areas, unknown_value = v6.build_semantic_masks(
        gray, v6_args
    )
    visited_mask = resolve_visited_mask(gray, meta, map_yaml, qwen_map_yaml, traj_json)
    visited_dist_field = build_visited_distance_field(visited_mask) if visited_mask.any() else None
    paint_visited_on_bgr(semantic, visited_mask)
    vertices = load_trajectory_vertices(traj_json) if traj_json else []
    draw_trajectory_overlay(semantic, vertices, meta)

    min_show = max(1, int(getattr(v6_args, "min_display_candidates", 5)))
    last_error: Optional[BaseException] = None
    case_candidates: List[Any] = []
    selection_meta: Dict[str, Any] = {}
    parameters: Dict[str, Any] = {}
    visit_attrs: Dict[int, Dict[str, Any]] = {}
    reject_stats: Dict[str, int] = {}
    tier_used = 1
    navigable = np.zeros_like(free, dtype=bool)
    nav_labels = np.zeros(free.shape, dtype=np.int32)
    frontier_mask = np.zeros_like(free, dtype=bool)
    robot_px = robot_py = 0
    pose = None
    geometry_tier = "strict"

    for tier_name, black_m, dist_m in v6.geometry_constraint_tiers(v6_args):
        v6.set_candidate_geometry_meters(
            v6_args,
            black_clearance_m=black_m,
            min_robot_distance_m=dist_m,
            longest=longest,
            resolution=meta.resolution,
        )
        try:
            candidates, navigable, nav_labels, frontier_mask, parameters = v6.generate_frontier_candidates(
                free, unknown, blocked, unknown_labels, unknown_areas, longest, v6_args,
                resolution=meta.resolution,
            )
            robot_px, robot_py = resolve_robot_pixel(rx, ry, meta, navigable)
            pose = v6.Pose(robot_px, robot_py, yaw_deg)
            case_candidates, selection_meta = v6.choose_case_candidates(
                pose, nav_labels, candidates, longest, v6_args,
                navigable=navigable, unknown=unknown, blocked=blocked,
                resolution=meta.resolution,
            )
            case_candidates, visit_attrs, reject_stats, tier_used = v6.apply_visited_candidate_policy(
                pose, case_candidates, nav_labels, longest, v6_args,
                visited_mask=visited_mask,
                visited_dist_field=visited_dist_field,
                unknown=unknown,
                blocked=blocked,
                resolution=meta.resolution,
            )
        except Exception as exc:
            last_error = exc
            case_candidates = []
            continue

        geometry_tier = tier_name
        selection_meta = dict(selection_meta)
        selection_meta["geometry_constraint_tier"] = tier_name
        selection_meta["geometry_black_clearance_m"] = float(black_m)
        selection_meta["geometry_min_robot_distance_m"] = float(dist_m)
        parameters = dict(parameters)
        parameters["geometry_constraint_tier"] = tier_name
        parameters["geometry_black_clearance_m"] = float(black_m)
        parameters["geometry_min_robot_distance_m"] = float(dist_m)

        if len(case_candidates) >= min_show:
            break
        # 不足 5 个时进入保护档（0.4m / 0.2m）；已是最后一档则接受现有结果。

    if pose is None or not case_candidates:
        raise RuntimeError(
            f"几何约束下无可用候选（含 protect_fallback 0.4m/0.2m）：{last_error}"
        )

    input_image, _ = v6.build_case_image(
        semantic, pose, case_candidates, v6_args.model_image_side,
        info_lines=[
            "LIVE SESSION INPUT",
            f"robot_map=({rx:.2f},{ry:.2f}) yaw={yaw_deg:.1f}",
            f"safe_frontier_candidates={len(case_candidates)} tier={tier_used} geom={geometry_tier}",
        ],
    )
    candidates_path = output_dir / "live_candidates.png"
    cv2.imwrite(str(candidates_path), input_image)

    foxglove_candidates_json = nav_goal_json.parent / "live_candidates_foxglove.json"
    write_foxglove_candidates_json(foxglove_candidates_json, meta, case_candidates, selected_local_id=None)

    cand_dicts = []
    for local_id, c in enumerate(case_candidates, start=1):
        d = candidate_to_dict(c, meta, pose, longest, visit_attrs.get(c.global_id))
        d["candidate_id"] = local_id
        d["local_id"] = local_id
        cand_dicts.append(d)

    session_id = args.session_id or f"live_{int(time.time())}"
    serializable_bundle: Dict[str, Any] = {
        "schema_version": "qwen_session_candidate_bundle_v2",
        "session_id": session_id,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "input_fingerprint": fp_hash,
        "input_files": input_fps,
        "clean_map_yaml": str(map_yaml),
        "clean_map_image": str(pgm_path),
        "map_yaml": str(map_yaml),
        "map_pgm": str(pgm_path),
        "map_width": meta.width,
        "map_height": meta.height,
        "resolution": meta.resolution,
        "map_resolution": meta.resolution,
        "origin": [meta.origin_x, meta.origin_y, 0.0],
        "map_origin": [meta.origin_x, meta.origin_y, 0.0],
        "robot_pose_map": {"x": rx, "y": ry, "yaw_rad": yaw_rad, "yaw_deg": yaw_deg},
        "robot_pose_pixel": {"x": robot_px, "y": robot_py},
        "robot_pixel": {"x": robot_px, "y": robot_py},
        "candidate_generation_parameters": parameters,
        "candidate_parameters": parameters,
        "selection_meta": selection_meta,
        "visited_summary": {
            "visited_cell_count": int(np.count_nonzero(visited_mask)),
            "trajectory_vertices": len(vertices),
        },
        "reject_reason_counts": reject_stats,
        "candidate_tier_used": tier_used,
        "geometry_constraint_tier": geometry_tier,
        "geometry_black_clearance_m": selection_meta.get("geometry_black_clearance_m"),
        "geometry_min_robot_distance_m": selection_meta.get("geometry_min_robot_distance_m"),
        "candidates": cand_dicts,
        "live_candidates_png": str(candidates_path),
        "live_candidates_foxglove_json": str(foxglove_candidates_json),
        "map_yaml_sha256": sha256_file(map_yaml),
        "map_pgm_sha256": sha256_file(pgm_path),
        "pose_snapshot_sha256": sha256_file(pose_json),
        "trajectory_snapshot_sha256": sha256_file(traj_json) if traj_json and traj_json.is_file() else "",
        **snap_info,
    }
    serializable_bundle["bundle_fingerprint"] = compute_bundle_fingerprint(serializable_bundle)
    atomic_write_json(bundle_path, serializable_bundle)
    return serializable_bundle


def load_or_generate_candidate_bundle(args, v6) -> Dict[str, Any]:
    """只返回可序列化 bundle（禁止附带 _runtime）。"""
    map_yaml, pose_json, traj_json, output_dir, _nav_goal, qwen_map_yaml = _prepare_map_paths(args)
    bundle_path = (
        Path(args.candidate_bundle).expanduser().resolve()
        if args.candidate_bundle
        else output_dir / "candidate_bundle.json"
    )
    args.candidate_bundle = str(bundle_path)
    input_fps, fp_hash = bundle_inputs_fingerprint(map_yaml, pose_json, traj_json, qwen_map_yaml)

    if (
        not args.force_regenerate_candidates
        and bundle_still_valid(bundle_path, fp_hash)
    ):
        serializable_bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
        if "_runtime" in serializable_bundle:
            raise RuntimeError("candidate_bundle.json 含非法 _runtime 字段")
        print(f"[CACHE] reuse candidate_bundle fingerprint={serializable_bundle.get('bundle_fingerprint')}")
        return serializable_bundle

    print("[CACHE] generate new candidate_bundle")
    return _generate_serializable_bundle(args, v6)


def rebuild_runtime_context(
    bundle: Dict[str, Any],
    *,
    clean_map_path: Path,
    visited_map_path: Optional[Path],
    pose_path: Path,
    trajectory_path: Optional[Path],
    v6_module,
    live_args: argparse.Namespace,
    output_dir: Path,
    nav_goal_json: Path,
) -> RuntimeContext:
    """根据缓存 bundle 和输入文件重新构建当前进程所需对象。"""
    if "_runtime" in bundle:
        raise RuntimeError("禁止从 bundle['_runtime'] 恢复；请使用 rebuild_runtime_context")

    map_yaml = Path(str(bundle.get("clean_map_yaml") or bundle.get("map_yaml") or clean_map_path)).resolve()
    if not map_yaml.is_file():
        map_yaml = clean_map_path.resolve()
    meta = load_map_yaml_meta(map_yaml)
    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    pgm_path = map_yaml.parent / str(map_data.get("image", ""))
    if not pgm_path.is_file():
        pgm_path = Path(str(bundle.get("clean_map_image") or bundle.get("map_pgm", ""))).expanduser()
    if not pgm_path.is_file():
        pgm_path = map_yaml.with_suffix(".pgm")
    gray = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise RuntimeError(f"无法读取地图：{pgm_path}")

    h, w = gray.shape
    if h != meta.height or w != meta.width:
        meta = MapYamlMeta(w, h, meta.resolution, meta.origin_x, meta.origin_y, meta.frame_id)

    v6_args = build_v6_args(live_args, pgm_path, output_dir)
    longest = max(h, w)
    v6_args.map_resolution = float(meta.resolution)
    v6_args.base_min_goal_distance_ratio = float(v6_args.min_goal_distance_ratio)
    v6_args.strict_black_clearance_m = float(v6_args.hard_black_clearance_m)
    v6_args.strict_min_robot_distance_m = float(v6_args.hard_min_robot_distance_m)
    # 重建时沿用 bundle 最终采用的几何档，避免 navigable 与候选不一致。
    geom_black = bundle.get("geometry_black_clearance_m")
    geom_dist = bundle.get("geometry_min_robot_distance_m")
    if geom_black is None or geom_dist is None:
        sel = bundle.get("selection_meta") or {}
        geom_black = sel.get("geometry_black_clearance_m", v6_args.hard_black_clearance_m)
        geom_dist = sel.get("geometry_min_robot_distance_m", v6_args.hard_min_robot_distance_m)
    v6_module.set_candidate_geometry_meters(
        v6_args,
        black_clearance_m=float(geom_black),
        min_robot_distance_m=float(geom_dist),
        longest=longest,
        resolution=meta.resolution,
    )
    rx, ry, yaw_rad = load_pose_json(pose_path)
    yaw_deg = math.degrees(yaw_rad)

    semantic, free, unknown, blocked, unknown_labels, unknown_areas, unknown_value = v6_module.build_semantic_masks(
        gray, v6_args
    )
    _cands, navigable, nav_labels, _frontier, _params = v6_module.generate_frontier_candidates(
        free, unknown, blocked, unknown_labels, unknown_areas, longest, v6_args,
        resolution=meta.resolution,
    )
    visited_mask = resolve_visited_mask(gray, meta, map_yaml, visited_map_path, trajectory_path)
    visited_dist_field = build_visited_distance_field(visited_mask) if visited_mask.any() else None
    paint_visited_on_bgr(semantic, visited_mask)
    vertices = load_trajectory_vertices(trajectory_path) if trajectory_path else []
    draw_trajectory_overlay(semantic, vertices, meta)

    robot_pixel = bundle.get("robot_pose_pixel") or bundle.get("robot_pixel") or {}
    robot_px = int(robot_pixel.get("x", 0))
    robot_py = int(robot_pixel.get("y", 0))
    if not navigable[robot_py, robot_px]:
        robot_px, robot_py = resolve_robot_pixel(rx, ry, meta, navigable)
    pose = v6_module.Pose(robot_px, robot_py, yaw_deg)

    case_candidates = [dict_to_frontier_candidate(v6_module, d) for d in (bundle.get("candidates") or [])]
    if not case_candidates:
        raise RuntimeError("candidate_bundle 中无候选")

    candidates_path = Path(str(bundle.get("live_candidates_png") or (output_dir / "live_candidates.png")))
    foxglove_candidates_json = Path(
        str(bundle.get("live_candidates_foxglove_json") or (nav_goal_json.parent / "live_candidates_foxglove.json"))
    )
    if not candidates_path.is_file():
        input_image, _ = v6_module.build_case_image(
            semantic, pose, case_candidates, v6_args.model_image_side,
            info_lines=["LIVE SESSION INPUT (rebuilt from bundle)"],
        )
        candidates_path = output_dir / "live_candidates.png"
        cv2.imwrite(str(candidates_path), input_image)

    return RuntimeContext(
        semantic=semantic,
        pose=pose,
        case_candidates=case_candidates,
        nav_labels=nav_labels,
        visited_dist_field=visited_dist_field,
        v6_args=v6_args,
        meta=meta,
        longest=longest,
        map_yaml=map_yaml,
        pgm_path=pgm_path,
        rx=rx,
        ry=ry,
        yaw_rad=yaw_rad,
        yaw_deg=yaw_deg,
        robot_px=robot_px,
        robot_py=robot_py,
        candidates_path=candidates_path,
        foxglove_candidates_json=foxglove_candidates_json,
        unknown_value=unknown_value,
        vertices=vertices,
        unknown=unknown,
        blocked=blocked,
        navigable=navigable,
    )


def main() -> int:
    args = parse_args()
    v6 = _load_v6_module()
    map_yaml, pose_json, traj_json, output_dir, nav_goal_json, qwen_map_yaml = _prepare_map_paths(args)
    output_dir.mkdir(parents=True, exist_ok=True)

    bundle_path = (
        Path(args.candidate_bundle).expanduser().resolve()
        if args.candidate_bundle
        else output_dir / "candidate_bundle.json"
    )
    args.candidate_bundle = str(bundle_path)

    serializable_bundle = load_or_generate_candidate_bundle(args, v6)
    if "_runtime" in serializable_bundle:
        raise RuntimeError("内部错误：serializable_bundle 不应含 _runtime")

    runtime_context = rebuild_runtime_context(
        serializable_bundle,
        clean_map_path=map_yaml,
        visited_map_path=qwen_map_yaml,
        pose_path=pose_json,
        trajectory_path=traj_json,
        v6_module=v6,
        live_args=args,
        output_dir=output_dir,
        nav_goal_json=nav_goal_json,
    )

    if args.candidates_only:
        print(f"[OK] 候选 bundle：{bundle_path}")
        print(f"[OK] 候选 Foxglove JSON：{runtime_context.foxglove_candidates_json}")
        print(f"[OK] 候选图：{runtime_context.candidates_path}")
        print(
            f"[OK] candidates_only=1 count={len(runtime_context.case_candidates)} "
            f"fingerprint={serializable_bundle.get('bundle_fingerprint')}"
        )
        return 0

    semantic = runtime_context.semantic
    pose = runtime_context.pose
    case_candidates = runtime_context.case_candidates
    nav_labels = runtime_context.nav_labels
    visited_dist_field = runtime_context.visited_dist_field
    v6_args = runtime_context.v6_args
    meta = runtime_context.meta
    longest = runtime_context.longest
    map_yaml = runtime_context.map_yaml
    w, h = meta.width, meta.height

    prompt = v6.GOAL_PROMPT_TEMPLATE_ZH.format(
        robot_u=pose.x / max(1, w - 1),
        robot_v=pose.y / max(1, h - 1),
        yaw_deg=pose.yaw_deg,
        task=args.task,
        min_distance_ratio=v6_args.min_goal_distance_ratio,
        max_distance_ratio=v6_args.max_goal_distance_ratio,
        preferred_min_ratio=v6_args.preferred_min_distance_ratio,
        preferred_max_ratio=v6_args.preferred_max_distance_ratio,
        nearest_distance_ratio=serializable_bundle["selection_meta"]["nearest_distance_ratio"],
        candidate_distance_limit_ratio=serializable_bundle["selection_meta"]["candidate_distance_limit_ratio"],
        heading_cone_deg=v6_args.front_cone_deg,
        min_display_candidates=v6_args.min_display_candidates,
        candidate_table=v6.candidate_table_text(
            pose, case_candidates, nav_labels, w, h, longest, v6_args,
            visited_dist_field=visited_dist_field,
        ),
    )
    (output_dir / "live_prompt.txt").write_text(prompt, encoding="utf-8")

    input_image, _ = v6.build_case_image(
        semantic, pose, case_candidates, v6_args.model_image_side,
        info_lines=["LIVE SESSION INPUT", f"task={args.task[:40]}"],
    )

    selected_local_id: int
    selected_by: str
    confidence = None
    reason = ""
    raw_response = ""
    latency_s = None
    error = ""

    def _pick_algorithm_ranked_final() -> int:
        """按算法等级选最优候选作为最终点（CLEAR/距离/朝向/已扫远离）。"""
        return int(
            v6.deterministic_fallback(
                pose,
                case_candidates,
                nav_labels,
                longest,
                v6_args,
                visited_dist_field=visited_dist_field,
            )
        )

    if v6_args.dry_run:
        selected_local_id = _pick_algorithm_ranked_final()
        selected_by = "python_dry_run"
        confidence = 1.0
        reason = "dry-run"
    else:
        api_key = v6.get_api_key(v6_args.api_key)
        qwen_ok = False
        if not api_key:
            error = "未设置 DASHSCOPE_API_KEY，改用算法等级选点"
            print(f"[WARN] {error}", file=sys.stderr)
        else:
            try:
                raw_response, latency_s = v6.call_qwen(
                    input_image, prompt, api_key, v6_args, image_format="jpeg"
                )
                (output_dir / "live_qwen_raw.txt").write_text(raw_response, encoding="utf-8")
                parsed = v6.extract_json(raw_response)
                if "candidate_id" not in parsed:
                    raise ValueError("Qwen 响应缺少 candidate_id")
                selected_local_id = int(parsed.get("candidate_id"))
                if not 1 <= selected_local_id <= len(case_candidates):
                    raise ValueError(f"candidate_id={selected_local_id} 超出范围")
                selected_by = "qwen"
                if parsed.get("confidence") is not None:
                    confidence = float(parsed["confidence"])
                reason = str(parsed.get("reason", "")).strip()
                qwen_ok = True
            except Exception as exc:
                error = str(exc)
                print(f"[WARN] Qwen 选点失败，改用算法等级选点：{error}", file=sys.stderr)

        if not qwen_ok:
            selected_local_id = _pick_algorithm_ranked_final()
            selected_by = "algorithm_rank"
            if not reason:
                reason = "algorithm_rank_final"
            confidence = 1.0 if confidence is None else confidence
            print(
                f"[OK] 算法等级最终点 candidate_id={selected_local_id} "
                f"(selected_by={selected_by})",
                flush=True,
            )

    chosen = case_candidates[selected_local_id - 1]
    write_foxglove_candidates_json(
        Path(runtime_context.foxglove_candidates_json), meta, case_candidates, selected_local_id=selected_local_id,
    )
    result_image, _ = v6.build_case_image(
        semantic, pose, case_candidates, v6_args.model_image_side,
        selected_local_id=selected_local_id,
        show_all_candidates=False,
        show_candidate_labels=False,
        info_lines=None,
    )
    cv2.imwrite(str(output_dir / "live_result.png"), result_image)

    goal_x, goal_y = pixel_to_map_xy(chosen.x, chosen.y, meta)
    report = {
        "mode": "live_session_v6_goal_only",
        "task": args.task,
        "bundle": str(bundle_path),
        "bundle_fingerprint": serializable_bundle.get("bundle_fingerprint"),
        "selected_local_id": selected_local_id,
        "selected_by": selected_by,
        "goal_map": {"x": goal_x, "y": goal_y},
        "error": error,
    }
    atomic_write_json(output_dir / "live_report.json", report)

    nav_payload = build_nav_goal_payload(
        session_id=serializable_bundle.get("session_id") or args.session_id,
        map_yaml=map_yaml,
        meta=meta,
        robot_map={
            "x": runtime_context.rx,
            "y": runtime_context.ry,
            "yaw_rad": runtime_context.yaw_rad,
            "yaw_deg": runtime_context.yaw_deg,
            "pixel_x": runtime_context.robot_px,
            "pixel_y": runtime_context.robot_py,
        },
        chosen_pixel=(chosen.x, chosen.y),
        result_meta={
            "selected_by": selected_by,
            "selected_local_id": selected_local_id,
            "selected_global_id": chosen.global_id,
            "confidence": confidence,
            "reason": reason,
        },
        output_dir=output_dir,
        bundle=serializable_bundle,
        bundle_path=bundle_path,
    )
    write_navigation_goal_proposal(nav_goal_json, nav_payload)

    print(f"[OK] 候选图：{runtime_context.candidates_path}")
    print(f"[OK] 结果图：{output_dir / 'live_result.png'}")
    print(f"[OK] 导航提案：{nav_goal_json}")
    print(
        f"[GOAL] map x={goal_x:.3f} y={goal_y:.3f} "
        f"(selected_by={selected_by}, candidate_id={selected_local_id}, "
        f"fingerprint={serializable_bundle.get('bundle_fingerprint')})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
