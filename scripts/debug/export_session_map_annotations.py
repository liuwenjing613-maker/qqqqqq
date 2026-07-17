#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从已保存 SLAM 地图 + 轨迹 + 位姿，生成带已走路径与大号机器人箭头的标注图，
并输出归一化 u/v/yaw（供日志/Foxglove 参考）。

不启动 ROS，不发布 /cmd_vel。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from qwen_map_goal_utils import (  # noqa: E402
    MapYamlMeta,
    build_free_mask_from_gray,
    load_map_yaml_meta,
    load_visited_corridor_radius_m,
    paint_visited_on_bgr,
    parse_trajectory_line_bgr,
    world_to_pixel,
)


def load_pose(pose_json: Path) -> Tuple[float, float, float]:
    raw = json.loads(pose_json.read_text(encoding="utf-8"))
    x = float(raw.get("x", 0.0))
    y = float(raw.get("y", 0.0))
    if "yaw" in raw:
        yaw = float(raw["yaw"])
    else:
        qx = float(raw.get("qx", 0.0))
        qy = float(raw.get("qy", 0.0))
        qz = float(raw.get("qz", 0.0))
        qw = float(raw.get("qw", 1.0))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
    return x, y, yaw


def load_trajectory_vertices(traj_json: Path) -> List[Tuple[float, float]]:
    if not traj_json.is_file():
        return []
    try:
        raw = json.loads(traj_json.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return []
    out: List[Tuple[float, float]] = []
    for v in raw.get("vertices") or []:
        if isinstance(v, dict):
            out.append((float(v.get("x", 0.0)), float(v.get("y", 0.0))))
        elif isinstance(v, (list, tuple)) and len(v) >= 2:
            out.append((float(v[0]), float(v[1])))
    return out


def gray_to_bgr(gray: np.ndarray) -> np.ndarray:
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    free = gray >= 245
    unknown = (gray > 70) & (gray < 245)
    blocked = gray <= 70
    bgr[free] = (255, 255, 255)
    bgr[unknown] = (128, 128, 128)
    bgr[blocked] = (0, 0, 0)
    return bgr


def draw_robot_arrow(bgr: np.ndarray, px: int, py: int, yaw_rad: float) -> None:
    h, w = bgr.shape[:2]
    ref = max(h, w)
    radius = max(8, int(round(ref * 0.008)))
    arrow_len = max(40, int(round(ref * 0.04)))
    thickness = max(3, int(round(ref * 0.0025)))
    cv2.circle(bgr, (px, py), radius, (37, 99, 235), -1, lineType=cv2.LINE_AA)
    cv2.circle(bgr, (px, py), radius, (255, 255, 255), max(1, thickness // 2), lineType=cv2.LINE_AA)
    yaw_deg = math.degrees(yaw_rad)
    end = (
        int(round(px + arrow_len * math.cos(yaw_deg * math.pi / 180.0))),
        int(round(py - arrow_len * math.sin(yaw_deg * math.pi / 180.0))),
    )
    cv2.arrowedLine(
        bgr, (px, py), end, (38, 38, 220), thickness=thickness, tipLength=0.28, line_type=cv2.LINE_AA
    )


def draw_visited_corridor(
    bgr: np.ndarray,
    gray: np.ndarray,
    meta: MapYamlMeta,
    vertices: List[Tuple[float, float]],
    *,
    corridor_radius_m: float = 0.35,
) -> None:
    if len(vertices) < 1:
        return
    from src.planning.robot_trajectory_store import rasterize_visited_corridor, vertices_from_xy

    free_mask = build_free_mask_from_gray(gray)
    h, w = gray.shape
    verts = vertices_from_xy(vertices)
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
    visited_mask = np.zeros((h, w), dtype=bool)
    for idx, val in enumerate(visited_flat):
        if val >= 100:
            r, c = divmod(idx, w)
            visited_mask[r, c] = True
    paint_visited_on_bgr(bgr, visited_mask)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--pose-json", required=True)
    parser.add_argument("--trajectory-json", default="")
    parser.add_argument("--output-png", required=True)
    parser.add_argument("--output-pose-json", required=True)
    parser.add_argument("--copy-live-annotated", default="")
    parser.add_argument(
        "--corridor-radius-m",
        type=float,
        default=None,
        help="已扫走廊半径（米）；默认读 configs/qwen_region_explore_debug.yaml",
    )
    live = parser.parse_args()
    corridor_radius_m = (
        float(live.corridor_radius_m)
        if live.corridor_radius_m is not None
        else load_visited_corridor_radius_m()
    )

    map_yaml = Path(live.map_yaml).expanduser().resolve()
    pose_json = Path(live.pose_json).expanduser().resolve()
    traj_json = (
        Path(live.trajectory_json).expanduser().resolve()
        if live.trajectory_json
        else PROJECT_ROOT / "runtime/qwen_region_debug/trajectory_session.json"
    )
    out_png = Path(live.output_png).expanduser().resolve()
    out_pose = Path(live.output_pose_json).expanduser().resolve()

    meta = load_map_yaml_meta(map_yaml)
    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    pgm_path = map_yaml.parent / str(map_data.get("image", ""))
    if not pgm_path.is_file():
        pgm_path = map_yaml.with_suffix(".pgm")
    gray = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise SystemExit(f"无法读取地图：{pgm_path}")

    h, w = gray.shape
    if h != meta.height or w != meta.width:
        meta = MapYamlMeta(
            width=w, height=h, resolution=meta.resolution,
            origin_x=meta.origin_x, origin_y=meta.origin_y, frame_id=meta.frame_id,
        )

    bgr = gray_to_bgr(gray)
    vertices = load_trajectory_vertices(traj_json)
    draw_visited_corridor(bgr, gray, meta, vertices, corridor_radius_m=corridor_radius_m)
    if len(vertices) >= 2:
        pts = []
        for vx, vy in vertices:
            px, py = world_to_pixel(vx, vy, meta)
            pts.append([int(np.clip(px, 0, w - 1)), int(np.clip(py, 0, h - 1))])
        cv2.polylines(
            bgr, [np.array(pts, dtype=np.int32)], False, parse_trajectory_line_bgr(),
            max(2, int(round(max(h, w) * 0.003))), cv2.LINE_AA,
        )

    rx, ry, yaw_rad = load_pose(pose_json)
    px, py = world_to_pixel(rx, ry, meta)
    px = int(np.clip(px, 0, w - 1))
    py = int(np.clip(py, 0, h - 1))
    draw_robot_arrow(bgr, px, py, yaw_rad)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_png), bgr)

    u = px / max(1, w - 1)
    v = py / max(1, h - 1)
    yaw_deg = math.degrees(yaw_rad)
    payload: Dict[str, Any] = {
        "map_yaml": str(map_yaml),
        "map_pgm": str(pgm_path),
        "pose_json": str(pose_json),
        "trajectory_json": str(traj_json) if traj_json.is_file() else None,
        "trajectory_vertex_count": len(vertices),
        "corridor_radius_m": corridor_radius_m,
        "robot_map_pose": {"x": rx, "y": ry, "yaw_rad": yaw_rad},
        "robot_image_pose": {"u": u, "v": v, "yaw_deg": yaw_deg},
        "annotated_map_png": str(out_png),
        "live_annotated_reference": live.copy_live_annotated or None,
    }
    out_pose.parent.mkdir(parents=True, exist_ok=True)
    out_pose.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload["robot_image_pose"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
