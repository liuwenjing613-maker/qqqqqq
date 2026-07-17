#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Qwen 图像归一化坐标与 map 坐标互转（供 Foxglove 标记与后续导航使用）。"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

# PGM 单通道编码：已扫过走廊（仅写在原 free 格；Nav2 用干净 PGM 不含此值）
VISITED_PGM_VALUE = 180
VISITED_BGR: Tuple[int, int, int] = (120, 180, 200)
# 实时 /map 叠层编码（Foxglove 用；仅 free 格）
VISITED_OCCUPANCY_VALUE = 50
OCCUPIED_VIZ_VALUE = 99
FREE_THRESHOLD = 245
DEFAULT_QWEN_REGION_CONFIG = (
    Path(__file__).resolve().parents[1] / "configs" / "qwen_region_explore_debug.yaml"
)


def load_visited_corridor_radius_m(
    config_path: Optional[Path] = None,
    *,
    default: float = 0.35,
) -> float:
    """走廊半径：环境变量 VISITED_CORRIDOR_RADIUS_M 优先，否则读 debug yaml。"""
    env = os.environ.get("VISITED_CORRIDOR_RADIUS_M", "").strip()
    if env:
        return float(env)
    path = config_path or DEFAULT_QWEN_REGION_CONFIG
    if not path.is_file():
        return float(default)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    traj = data.get("trajectory") or {}
    return float(traj.get("visited_corridor_radius_m", default))


@dataclass(frozen=True)
class MapYamlMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "map"


def load_map_yaml_meta(yaml_path: Path) -> MapYamlMeta:
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"invalid map yaml: {yaml_path}")
    origin = data.get("origin") or [0.0, 0.0, 0.0]
    image_name = str(data.get("image", ""))
    pgm_path = yaml_path.parent / image_name
    if not pgm_path.is_file():
        pgm_path = yaml_path.with_suffix(".pgm")
    if not pgm_path.is_file():
        raise FileNotFoundError(f"map pgm not found for {yaml_path}")

    try:
        from PIL import Image

        with Image.open(pgm_path) as img:
            width, height = img.size
    except Exception as exc:
        raise RuntimeError(f"cannot read map pgm size: {pgm_path}") from exc

    return MapYamlMeta(
        width=int(width),
        height=int(height),
        resolution=float(data.get("resolution", 0.05)),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
    )


def world_to_pixel(x: float, y: float, meta: MapYamlMeta) -> Tuple[int, int]:
    """map 坐标 → 图像像素（与 v6 / export 一致，图像 y 向下）。"""
    col = (float(x) - meta.origin_x) / meta.resolution
    row_ros = (float(y) - meta.origin_y) / meta.resolution
    px = int(round(col))
    py = int(round(meta.height - 1 - row_ros))
    return px, py


def pixel_to_map_xy(px: int, py: int, meta: MapYamlMeta) -> Tuple[float, float]:
    """图像像素 → map 坐标。"""
    row_ros = meta.height - 1 - int(py)
    x = meta.origin_x + float(px) * meta.resolution
    y = meta.origin_y + float(row_ros) * meta.resolution
    return x, y


def image_uv_to_map_xy(u: float, v: float, meta: MapYamlMeta) -> Tuple[float, float]:
    u = max(0.0, min(1.0, float(u)))
    v = max(0.0, min(1.0, float(v)))
    col = u * max(1, meta.width - 1)
    row = (1.0 - v) * max(1, meta.height - 1)
    x = meta.origin_x + col * meta.resolution
    y = meta.origin_y + row * meta.resolution
    return x, y


def map_xy_to_image_uv(x: float, y: float, meta: MapYamlMeta) -> Tuple[float, float]:
    col = (float(x) - meta.origin_x) / meta.resolution
    row = (float(y) - meta.origin_y) / meta.resolution
    u = col / max(1, meta.width - 1)
    v = 1.0 - (row / max(1, meta.height - 1))
    return max(0.0, min(1.0, u)), max(0.0, min(1.0, v))


def build_navigation_goal_proposal(
    *,
    map_yaml: Path,
    robot_u: float,
    robot_v: float,
    robot_yaw_deg: float,
    proposal: Optional[Dict[str, Any]],
    qwen_run_dir: Path,
    session_id: str = "",
) -> Dict[str, Any]:
    meta = load_map_yaml_meta(map_yaml)
    robot_x, robot_y = image_uv_to_map_xy(robot_u, robot_v, meta)
    payload: Dict[str, Any] = {
        "schema_version": "qwen_session_nav_goal_v1",
        "session_id": session_id,
        "map_frame": meta.frame_id,
        "map_yaml": str(map_yaml.resolve()),
        "robot_pose_map": {
            "x": robot_x,
            "y": robot_y,
            "yaw_rad": math.radians(robot_yaw_deg),
            "yaw_deg": robot_yaw_deg,
            "u": robot_u,
            "v": robot_v,
        },
        "selection_status": "NO_VALID_REGION",
        "goal_pose_map": None,
        "region_center_map": None,
        "test_point_image": None,
        "qwen_run_dir": str(qwen_run_dir.resolve()),
        "safety": {
            "path_checked": False,
            "reachability_validated": False,
            "ready_for_nav2": False,
            "note": "Qwen 图像提案，需 Nav2/几何校验后再作为真实导航目标",
        },
    }
    if not proposal:
        return payload

    payload["selection_status"] = "REGION_PROPOSED"
    center_u = float(proposal["center_u"])
    center_v = float(proposal["center_v"])
    test_u = float(proposal["test_point_u"])
    test_v = float(proposal["test_point_v"])
    goal_x, goal_y = image_uv_to_map_xy(test_u, test_v, meta)
    center_x, center_y = image_uv_to_map_xy(center_u, center_v, meta)
    heading = math.atan2(goal_y - robot_y, goal_x - robot_x)

    payload["goal_pose_map"] = {
        "x": goal_x,
        "y": goal_y,
        "z": 0.0,
        "yaw_rad": heading,
        "yaw_deg": math.degrees(heading),
    }
    payload["region_center_map"] = {"x": center_x, "y": center_y, "z": 0.0}
    payload["test_point_image"] = {"u": test_u, "v": test_v}
    payload["proposal_summary"] = {
        "proposal_id": proposal.get("proposal_id", "GP_1"),
        "confidence": proposal.get("confidence"),
        "reason_code": proposal.get("reason_code"),
        "center_u": center_u,
        "center_v": center_v,
    }
    return payload


def write_navigation_goal_proposal(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def build_free_mask_from_gray(gray: np.ndarray, free_threshold: int = FREE_THRESHOLD) -> np.ndarray:
    """PGM/灰度图：>= free_threshold 视为可通行白区。"""
    return gray >= int(free_threshold)


def visited_mask_from_gray(gray: np.ndarray, visited_value: int = VISITED_PGM_VALUE) -> np.ndarray:
    return gray == int(visited_value)


def apply_visited_to_pgm(
    nav_gray: np.ndarray,
    visited_flat: Sequence[int],
    free_mask: np.ndarray,
    *,
    visited_value: int = VISITED_PGM_VALUE,
) -> np.ndarray:
    """复制 nav PGM，仅在原 free 格写入 visited 编码。"""
    out = nav_gray.copy()
    h, w = out.shape
    if len(visited_flat) != h * w:
        raise ValueError(f"visited_flat length {len(visited_flat)} != {h * w}")
    for idx, val in enumerate(visited_flat):
        if val < 100:
            continue
        r, c = divmod(idx, w)
        if free_mask[r, c]:
            out[r, c] = int(visited_value)
    return out


def build_visited_distance_field(visited_mask: np.ndarray) -> np.ndarray:
    """每个像素到最近 visited 格的欧氏距离（像素）。"""
    inv = (~visited_mask.astype(bool)).astype(np.uint8)
    return cv2.distanceTransform(inv, cv2.DIST_L2, 3)


def distance_to_visited_px(
    x: int,
    y: int,
    visited_mask: np.ndarray,
    dist_field: Optional[np.ndarray] = None,
) -> float:
    h, w = visited_mask.shape
    xi = int(np.clip(x, 0, w - 1))
    yi = int(np.clip(y, 0, h - 1))
    if dist_field is not None:
        return float(dist_field[yi, xi])
    if visited_mask[yi, xi]:
        return 0.0
    field = build_visited_distance_field(visited_mask)
    return float(field[yi, xi])


def paint_visited_on_bgr(bgr: np.ndarray, visited_mask: np.ndarray) -> None:
    """在 BGR 语义图上绘制浅绿已扫区域（原地修改）。"""
    bgr[visited_mask] = VISITED_BGR


def write_foxglove_candidates_json(
    path: Path,
    meta: MapYamlMeta,
    candidates: Sequence[Any],
    *,
    selected_local_id: Optional[int] = None,
) -> None:
    """写入 Foxglove 候选点标记（始终保留全部候选；selected 仅用于高亮）。"""
    items: List[Dict[str, Any]] = []
    for local_id, candidate in enumerate(candidates, start=1):
        px = int(getattr(candidate, "x"))
        py = int(getattr(candidate, "y"))
        mx, my = pixel_to_map_xy(px, py, meta)
        items.append(
            {
                "local_id": local_id,
                "global_id": int(getattr(candidate, "global_id", local_id)),
                "map_x": mx,
                "map_y": my,
                "pixel_x": px,
                "pixel_y": py,
            }
        )
    payload: Dict[str, Any] = {
        "schema_version": "qwen_session_candidates_v1",
        "phase": "selected" if selected_local_id is not None else "pending",
        "selected_local_id": selected_local_id,
        "candidates": items,
    }
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def resolve_qwen_map_yaml(nav_map_yaml: Path) -> Path:
    """默认查找与 nav yaml 同目录的 {stem}_qwen.yaml。"""
    candidate = nav_map_yaml.with_name(f"{nav_map_yaml.stem}_qwen.yaml")
    return candidate


def proposal_dict_from_region_proposal(proposal: Any) -> Dict[str, Any]:
    return {
        "proposal_id": proposal.proposal_id,
        "center_u": proposal.center_u,
        "center_v": proposal.center_v,
        "test_point_u": proposal.test_point_u,
        "test_point_v": proposal.test_point_v,
        "confidence": proposal.confidence,
        "reason_code": proposal.reason_code,
    }
