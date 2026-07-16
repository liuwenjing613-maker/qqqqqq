#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 nav PGM + 轨迹生成 Qwen 专用 PGM（浅绿已扫区域编码为 VISITED_PGM_VALUE）。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

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
    FREE_THRESHOLD,
    VISITED_PGM_VALUE,
    apply_visited_to_pgm,
    build_free_mask_from_gray,
    load_map_yaml_meta,
)
from src.planning.robot_trajectory_store import rasterize_visited_corridor  # noqa: E402


def load_trajectory_vertices(traj_json: Path) -> List[Tuple[float, float, float]]:
    if not traj_json.is_file():
        return []
    try:
        raw = json.loads(traj_json.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return []
    out: List[Tuple[float, float, float]] = []
    for v in raw.get("vertices") or []:
        if isinstance(v, dict):
            out.append(
                (
                    float(v.get("x", 0.0)),
                    float(v.get("y", 0.0)),
                    float(v.get("yaw_rad", 0.0)),
                )
            )
        elif isinstance(v, (list, tuple)) and len(v) >= 2:
            yaw = float(v[2]) if len(v) >= 3 else 0.0
            out.append((float(v[0]), float(v[1]), yaw))
    return out


def vertices_to_store_format(vertices: List[Tuple[float, float, float]]):
    from src.planning.robot_trajectory_store import vertices_from_xy

    return vertices_from_xy([(x, y) for x, y, _ in vertices])


def export_qwen_map(
    *,
    map_yaml: Path,
    trajectory_json: Path,
    output_dir: Path,
    corridor_radius_m: float = 0.35,
    free_threshold: int = FREE_THRESHOLD,
) -> Dict[str, Any]:
    map_yaml = map_yaml.expanduser().resolve()
    trajectory_json = trajectory_json.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    meta = load_map_yaml_meta(map_yaml)
    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    nav_pgm_name = str(map_data.get("image", map_yaml.with_suffix(".pgm").name))
    nav_pgm_path = map_yaml.parent / nav_pgm_name
    if not nav_pgm_path.is_file():
        nav_pgm_path = map_yaml.with_suffix(".pgm")

    gray = cv2.imread(str(nav_pgm_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(f"无法读取 nav PGM: {nav_pgm_path}")

    stem = map_yaml.stem
    qwen_pgm_name = f"{stem}_qwen.pgm"
    qwen_yaml_name = f"{stem}_qwen.yaml"
    qwen_pgm_path = output_dir / qwen_pgm_name
    qwen_yaml_path = output_dir / qwen_yaml_name

    free_mask = build_free_mask_from_gray(gray, free_threshold)
    verts = vertices_to_store_format(load_trajectory_vertices(trajectory_json))
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
    qwen_gray = apply_visited_to_pgm(gray, visited_flat, free_mask)

    if not cv2.imwrite(str(qwen_pgm_path), qwen_gray):
        raise RuntimeError(f"无法写入 {qwen_pgm_path}")

    qwen_yaml = dict(map_data)
    qwen_yaml["image"] = qwen_pgm_name
    qwen_yaml_path.write_text(
        yaml.safe_dump(qwen_yaml, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    visited_count = int(np.sum(qwen_gray == VISITED_PGM_VALUE))
    return {
        "nav_map_yaml": str(map_yaml),
        "nav_pgm": str(nav_pgm_path),
        "qwen_map_yaml": str(qwen_yaml_path),
        "qwen_pgm": str(qwen_pgm_path),
        "visited_cell_count": visited_count,
        "trajectory_vertices": len(verts),
        "corridor_radius_m": corridor_radius_m,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="导出 Qwen 专用已扫区域 PGM")
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--trajectory-json", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--corridor-radius-m", type=float, default=0.35)
    args = parser.parse_args()

    meta = export_qwen_map(
        map_yaml=Path(args.map_yaml),
        trajectory_json=Path(args.trajectory_json),
        output_dir=Path(args.output_dir),
        corridor_radius_m=float(args.corridor_radius_m),
    )
    print(json.dumps(meta, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
