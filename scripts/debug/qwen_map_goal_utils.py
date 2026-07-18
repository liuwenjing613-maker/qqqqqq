#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Qwen 图像归一化坐标与 map 坐标互转（供 Foxglove 标记与后续导航使用）。"""

from __future__ import annotations

import hashlib
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


def atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    """Atomically write JSON so other processes never read a half-written file."""
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp_path, path)


def write_navigation_goal_proposal(path: Path, payload: Dict[str, Any]) -> None:
    atomic_write_json(path, payload)


def sha256_file(path: Path) -> str:
    path = path.expanduser().resolve()
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def compute_bundle_fingerprint(bundle: Dict[str, Any]) -> str:
    """Stable fingerprint over session inputs + candidate geometry (no timestamps)."""
    material = {
        "session_id": bundle.get("session_id"),
        "map_yaml_sha256": bundle.get("map_yaml_sha256"),
        "map_pgm_sha256": bundle.get("map_pgm_sha256"),
        "pose_snapshot_sha256": bundle.get("pose_snapshot_sha256"),
        "trajectory_snapshot_sha256": bundle.get("trajectory_snapshot_sha256"),
        "candidates": [
            {
                "candidate_id": c.get("candidate_id", c.get("local_id")),
                "pixel_x": c.get("pixel_x"),
                "pixel_y": c.get("pixel_y"),
                "map_x": c.get("map_x"),
                "map_y": c.get("map_y"),
            }
            for c in (bundle.get("candidates") or [])
        ],
    }
    blob = json.dumps(material, sort_keys=True, ensure_ascii=False)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:32]


def validate_proposal_against_bundle(
    proposal: Dict[str, Any],
    bundle: Dict[str, Any],
    *,
    expected_session_id: str = "",
    allow_fallback: bool = False,
    free_threshold: int = FREE_THRESHOLD,
    safety_radius_m: float = 0.15,
) -> Tuple[bool, str]:
    """Re-validate proposal vs candidate bundle before Nav2 auto-nav."""
    if expected_session_id and str(proposal.get("session_id", "")) != str(expected_session_id):
        return False, "proposal session_id 与本次会话不一致"
    if str(proposal.get("session_id", "")) != str(bundle.get("session_id", "")):
        return False, "proposal session_id 与 candidate_bundle 不一致"

    prop_fp = str(proposal.get("bundle_fingerprint", ""))
    bundle_fp = str(bundle.get("bundle_fingerprint", ""))
    if not prop_fp or not bundle_fp or prop_fp != bundle_fp:
        return False, "bundle_fingerprint 不匹配"

    sel = proposal.get("qwen_selection") or {}
    selected_by = str(sel.get("selected_by", ""))
    if selected_by == "python_fallback_after_qwen_error" and not allow_fallback:
        return False, "fallback 目标未显式允许自动导航 (--allow-nav-with-fallback)"

    cid = int(sel.get("selected_local_id") or proposal.get("selected_candidate_id") or 0)
    candidates = bundle.get("candidates") or []
    chosen = None
    for c in candidates:
        if int(c.get("candidate_id", c.get("local_id", -1))) == cid:
            chosen = c
            break
    if chosen is None:
        return False, f"selected_candidate_id={cid} 不存在于 candidate_bundle"

    goal = proposal.get("goal_pose_map") or {}
    sel_pix = proposal.get("selected_candidate_pixel") or {}
    px = int(sel_pix.get("x", goal.get("pixel_x", -1)))
    py = int(sel_pix.get("y", goal.get("pixel_y", -1)))
    if px != int(chosen["pixel_x"]) or py != int(chosen["pixel_y"]):
        return False, "proposal 像素坐标与候选不一致"

    sel_map = proposal.get("selected_candidate_map") or {}
    mx = float(sel_map.get("x", goal.get("x", float("nan"))))
    my = float(sel_map.get("y", goal.get("y", float("nan"))))
    if abs(mx - float(chosen["map_x"])) > 1e-4 or abs(my - float(chosen["map_y"])) > 1e-4:
        return False, "proposal map 坐标与候选不一致"

    map_yaml = Path(str(bundle.get("clean_map_yaml") or bundle.get("map_yaml", "")))
    if not map_yaml.is_file():
        return False, "bundle clean_map_yaml 不存在"
    if bundle.get("map_yaml_sha256") and sha256_file(map_yaml) != bundle["map_yaml_sha256"]:
        return False, "map YAML hash 与 bundle 不一致"
    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    pgm = map_yaml.parent / str(map_data.get("image", ""))
    if not pgm.is_file():
        pgm = map_yaml.with_suffix(".pgm")
    if bundle.get("map_pgm_sha256") and sha256_file(pgm) != bundle["map_pgm_sha256"]:
        return False, "map PGM hash 与 bundle 不一致"

    gray = cv2.imread(str(pgm), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        return False, "无法读取干净 PGM"
    h, w = gray.shape
    if not (0 <= px < w and 0 <= py < h):
        return False, "目标像素超出地图边界"
    if int(gray[py, px]) < free_threshold:
        return False, "目标栅格不是 free"

    # Occupied safety-radius recheck removed: candidate generation already enforces
    # circular black clearance; the old axis-aligned square (<=70) falsely rejected
    # valid candidates near diagonal walls.
    del safety_radius_m  # retained in signature for call-site compatibility

    free_mask = build_free_mask_from_gray(gray, free_threshold=free_threshold)
    num, labels = cv2.connectedComponents(free_mask.astype(np.uint8), connectivity=8)
    robot = bundle.get("robot_pose_pixel") or bundle.get("robot_pixel") or {}
    rpx = int(robot.get("x", robot.get("pixel_x", -1)))
    rpy = int(robot.get("y", robot.get("pixel_y", -1)))
    if not (0 <= rpx < w and 0 <= rpy < h and free_mask[rpy, rpx]):
        return False, "机器人位姿不在 free 区域"
    if int(labels[rpy, rpx]) == 0 or int(labels[py, px]) == 0:
        return False, "目标或机器人不在可通行连通域"
    if int(labels[rpy, rpx]) != int(labels[py, px]):
        return False, "目标与机器人不在同一安全连通域"
    if num < 2:
        return False, "地图无可通行连通域"
    return True, "ok"


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
    atomic_write_json(path, payload)


def resolve_qwen_map_yaml(nav_map_yaml: Path) -> Path:
    """默认查找与 nav yaml 同目录的 {stem}_qwen.yaml。"""
    candidate = nav_map_yaml.with_name(f"{nav_map_yaml.stem}_qwen.yaml")
    return candidate


def parse_trajectory_line_bgr() -> Tuple[int, int, int]:
    """轨迹中心线 BGR，默认绿色 (60, 170, 60)。"""
    raw = os.environ.get("TRAJECTORY_LINE_BGR", "60,170,60").strip()
    parts = [int(x.strip()) for x in raw.split(",")]
    if len(parts) != 3:
        return (60, 170, 60)
    return tuple(parts)  # type: ignore[return-value]


def file_fingerprint(path: Path) -> Dict[str, Any]:
    path = path.expanduser().resolve()
    if not path.is_file():
        return {"path": str(path), "exists": False}
    stat = path.stat()
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return {
        "path": str(path),
        "exists": True,
        "mtime": stat.st_mtime,
        "size": stat.st_size,
        "sha256": digest.hexdigest()[:16],
    }


def read_robot_pose_uv(pose_uv_json: Path) -> Tuple[float, float, float]:
    data = json.loads(pose_uv_json.read_text(encoding="utf-8"))
    pose = data["robot_image_pose"]
    return float(pose["u"]), float(pose["v"]), float(pose["yaw_deg"])


def validate_goal_for_nav_validation(
    goal_json: Path,
    *,
    expected_map_yaml: Path,
    allow_fallback: bool = False,
    free_threshold: int = FREE_THRESHOLD,
) -> Tuple[bool, str]:
    """检查是否允许启动 Nav2 进行路径验证（非 ready_for_nav2）。"""
    if not goal_json.is_file():
        return False, "navigation_goal_proposal.json 不存在"
    data = json.loads(goal_json.read_text(encoding="utf-8"))
    if data.get("selection_status") != "REGION_PROPOSED":
        return False, "selection_status 不是 REGION_PROPOSED"
    goal = data.get("goal_pose_map") or {}
    for key in ("x", "y", "yaw_rad", "yaw_deg"):
        if key not in goal or not math.isfinite(float(goal[key])):
            return False, f"goal_pose_map.{key} 无效"
    map_yaml = Path(str(data.get("map_yaml", ""))).resolve()
    if map_yaml != expected_map_yaml.resolve():
        return False, "map_yaml 与会话 MAP_YAML 不一致"
    safety = data.get("safety") or {}
    if not safety.get("candidate_geometry_validated", False):
        return False, "candidate_geometry_validated != true"
    sel = data.get("qwen_selection") or {}
    selected_by = str(sel.get("selected_by", ""))
    if selected_by == "python_fallback_after_qwen_error" and not allow_fallback:
        return False, "Qwen fallback 目标未显式允许导航 (--allow-nav-with-fallback)"
    local_id = int(sel.get("selected_local_id", 0))
    if local_id < 1:
        return False, "selected_local_id 无效"
    try:
        meta = load_map_yaml_meta(expected_map_yaml)
        pgm = expected_map_yaml.parent / yaml.safe_load(
            expected_map_yaml.read_text(encoding="utf-8")
        ).get("image", expected_map_yaml.with_suffix(".pgm").name)
        if not pgm.is_file():
            pgm = expected_map_yaml.with_suffix(".pgm")
        gray = cv2.imread(str(pgm), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return False, "无法读取干净 PGM"
        px = int(goal.get("pixel_x", -1))
        py = int(goal.get("pixel_y", -1))
        if not (0 <= px < meta.width and 0 <= py < meta.height):
            return False, "目标像素超出地图边界"
        if gray[py, px] < free_threshold:
            return False, "目标不在干净地图 free 区域"
    except Exception as exc:
        return False, f"几何校验异常: {exc}"
    return True, "ok"


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
