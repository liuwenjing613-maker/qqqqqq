#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在占据栅格地图上生成 5 个随机机器人位姿，并让 Qwen 从“程序严格验证且距离受限”
的灰白前沿候选中选择下一步探索目标。

本版保持地图分析、候选筛选、两阶段 Qwen 调用和 Python 回退流程不变，只调整图片输出：
1. 终端仅显示运行时间、当前进程和最终保存位置；
2. 每次运行前自动清理输出目录中的旧 PNG；
3. 每个案例保存一张带全部候选点的 Qwen 输入图：case_01_candidates.png ~ case_05_candidates.png；
4. 每个案例保存一张最终结果图：case_01_result.png ~ case_05_result.png；
5. 候选图显示机器人位置、朝向箭头、全部黄色编号候选点和必要信息框；
6. 最终图只显示机器人位置、朝向箭头和 Qwen 选出的绿色目标路径点；
7. Prompt、原始响应和 report.json 等文本记录继续保存，便于排查与复现。
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import random
import re
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import requests


DEFAULT_MAP = r"C:\Users\Acer\Desktop\x\joy_calibrated_corridor_map.png"
DEFAULT_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_MODEL = "qwen3-vl-flash"

# 候选点几何约束：
# - 默认严格档：离黑 0.35m、离车 0.8m
# - 展示候选不足 min_display_candidates 时，启用保护档：离黑 0.2m、离车 0.4m
DEFAULT_HARD_BLACK_CLEARANCE_M = 0.35
DEFAULT_HARD_MIN_ROBOT_DISTANCE_M = 0.8
DEFAULT_FALLBACK_BLACK_CLEARANCE_M = 0.2
DEFAULT_FALLBACK_MIN_ROBOT_DISTANCE_M = 0.4

FREE_BGR = (255, 255, 255)
UNKNOWN_BGR = (128, 128, 128)
BLOCKED_BGR = (0, 0, 0)
ROBOT_BGR = (235, 99, 37)       # 蓝色（OpenCV BGR）
HEADING_BGR = (38, 38, 220)     # 红色
CANDIDATE_BGR = (0, 210, 255)   # 黄色：探索目标候选
POSE_CANDIDATE_BGR = (200, 80, 200) # 紫色：机器人位置候选
GOAL_BGR = (74, 163, 22)        # 绿色
TEXT_BG_BGR = (245, 245, 245)


# 两阶段提示词均保持简洁：程序直接生成 5 个随机白区位置，第一阶段由 Qwen 分配随机朝向，第二阶段选择安全前沿。

POSE_PROMPT_TEMPLATE_ZH = """
你是占据栅格地图的随机朝向生成器。

颜色：白色=已知可通行，灰色=未知，黑色=墙体/障碍；紫色编号圆点是程序已经随机生成的 {case_count} 个机器人位置。
这些位置均位于安全白色区域，并且都能在规定距离内找到可达灰白前沿。

请为图中每一个位置编号分别生成一个随机 yaw_deg：
- 必须恰好输出 {case_count} 项；
- 必须覆盖下面列出的全部位置编号，每个编号使用一次且仅一次；
- yaw_deg 范围为 -180 到 180；
- 0° 向右，90° 向上，-90° 向下，±180° 向左；
- 朝向尽量多样，不要全部朝向同一方向。

随机扰动码：{nonce}
必须覆盖的位置编号：{position_ids}

只输出合法 JSON，不要 Markdown，不要解释：
{{"poses":[{{"position_id":1,"yaw_deg":30.0}}]}}
""".strip()

GOAL_PROMPT_TEMPLATE_ZH = """
你是二维占据栅格地图的近距离前沿探索目标选择器。

颜色：白色=已知可通行，灰色=未知待探索，黑色=墙体/障碍；浅绿色=小车已扫过/已通过区域（通行性等同白色，但表示已探索）；蓝点=机器人位置；红箭头=机器人朝向；黄色编号圆点=程序生成的安全候选目标。

程序已逐像素保证每个黄色候选：
- 目标点自身位于白色自由区；
- 紧邻连续灰色未知区，确实属于灰白交界；
- 安全邻域内没有黑色墙体或障碍；
- 与机器人处于同一安全白色连通区域，不需要穿墙或穿越灰区；
- 从小车指向候选点的方向与小车朝向夹角不超过 {heading_cone_deg:.0f}°（仅前方扇区）；
- 到小车直线距离已通过程序筛选：硬下限 ≥ 0.8m；不足 {min_display_candidates} 个时只会放宽距离上限，不会放宽该硬下限。

本案例距离规则（距离除以地图最长边得到 distance_ratio）：
- 硬下限：到小车直线距离 ≥ 0.8m（distance_ratio >= {min_distance_ratio:.3f}）
- 硬上限：distance_ratio <= {max_distance_ratio:.3f}
- 优选距离：{preferred_min_ratio:.3f} <= distance_ratio <= {preferred_max_ratio:.3f}
- 本图最近候选距离：{nearest_distance_ratio:.3f}
- 本图展示候选的最远距离：{candidate_distance_limit_ratio:.3f}

选择顺序：
1. 绝对禁止选择图中不存在的编号；
2. 所有候选均在小车朝向 ±{heading_cone_deg:.0f}° 前方扇区内，禁止选后方目标，优先考虑和小车朝向夹角小的候选；
3. 优先选择 direct_path=CLEAR；只有不存在任何 CLEAR 候选时，才允许使用 BLOCKED 候选；
4. BLOCKED 候选不得理解为可以穿墙，仅表示直线经过贴墙窄缝；
5. 同一方向层级内，优先选择距离黑色墙体较远的且在优选距离范围，最后选择distance_ratio 中等者，最后选择 distance_ratio 更大者；
6. 不要为了追求大灰区牺牲距离，不得选择展示范围之外的目标；
7. 优先选择远离浅绿色已扫区域的候选；同等条件下 visited_clearance_px 更大者优先；不要重复探索已扫过走廊附近。

机器人：u={robot_u:.6f}, v={robot_v:.6f}, yaw_deg={yaw_deg:.2f}

本次探索任务：
{task}

候选信息：
{candidate_table}

只输出一个合法 JSON，不要 Markdown，不要分析过程：
{{"candidate_id": 1, "confidence": 0.90, "reason": "一句简短中文原因，说明距离与朝向"}}
""".strip()


@dataclass(frozen=True)
class Pose:
    x: int
    y: int
    yaw_deg: float


@dataclass(frozen=True)
class RobotCandidate:
    position_id: int
    x: int
    y: int
    component_id: int


@dataclass(frozen=True)
class FrontierCandidate:
    global_id: int
    x: int
    y: int
    component_id: int
    obstacle_clearance_px: float
    unknown_distance_px: float
    unknown_area_px: int


@dataclass
class CaseResult:
    case_id: int
    robot_x: int
    robot_y: int
    robot_u: float
    robot_v: float
    yaw_deg: float
    candidate_count: int
    nearest_candidate_distance_ratio: Optional[float] = None
    candidate_distance_limit_ratio: Optional[float] = None
    goal_distance_ratio: Optional[float] = None
    goal_heading_delta_deg: Optional[float] = None
    selected_local_id: Optional[int] = None
    selected_global_id: Optional[int] = None
    goal_x: Optional[int] = None
    goal_y: Optional[int] = None
    goal_u: Optional[float] = None
    goal_v: Optional[float] = None
    confidence: Optional[float] = None
    reason: str = ""
    selected_by: str = ""
    latency_s: Optional[float] = None
    raw_response: str = ""
    error: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="随机生成 5 个机器人位姿，让 Qwen 选择严格安全的灰白前沿目标。"
    )
    parser.add_argument("--map", default=DEFAULT_MAP, help="地图 PNG/PGM/JPG 路径。")
    parser.add_argument("--cases", type=int, default=5, help="生成案例数量，默认 5。")
    parser.add_argument("--output-dir", default="qwen_five_frontier_nearby_v3_results")
    parser.add_argument("--model", default=os.getenv("QWEN_MODEL", DEFAULT_MODEL))
    parser.add_argument("--base-url", default=os.getenv("QWEN_BASE_URL", DEFAULT_BASE_URL))
    parser.add_argument("--api-key", default=None)
    parser.add_argument("--seed", type=int, default=20260714)

    parser.add_argument("--free-threshold", type=int, default=245)
    parser.add_argument("--occupied-threshold", type=int, default=70)
    parser.add_argument("--unknown-value", type=int, default=-1,
                        help="灰色未知值；-1 表示自动检测。")
    parser.add_argument("--unknown-tolerance", type=int, default=10)
    parser.add_argument("--unknown-min-area", type=int, default=80)

    parser.add_argument("--black-clearance-ratio", type=float, default=0.010,
                        help="候选点周围无黑色半径/地图最长边，默认 1%%。")
    parser.add_argument("--robot-clearance-ratio", type=float, default=0.012,
                        help="机器人离黑色的最小距离/地图最长边。")
    parser.add_argument("--frontier-gap-ratio", type=float, default=0.006,
                        help="候选白点到最近灰色的最大距离/地图最长边。")
    parser.add_argument("--candidate-spacing-ratio", type=float, default=0.018,
                        help="候选点之间最小间距/地图最长边。")
    parser.add_argument(
        "--hard-black-clearance-m",
        type=float,
        default=float(os.getenv("CANDIDATE_HARD_BLACK_CLEARANCE_M", str(DEFAULT_HARD_BLACK_CLEARANCE_M))),
        help="严格档：候选周围无黑障碍圆半径（米），默认 0.35；候选不足时再降到 fallback。",
    )
    parser.add_argument(
        "--hard-min-robot-distance-m",
        type=float,
        default=float(os.getenv("CANDIDATE_HARD_MIN_ROBOT_DISTANCE_M", str(DEFAULT_HARD_MIN_ROBOT_DISTANCE_M))),
        help="严格档：候选到小车直线距离下限（米），默认 0.8；候选不足时再降到 fallback。",
    )
    parser.add_argument(
        "--fallback-black-clearance-m",
        type=float,
        default=float(os.getenv("CANDIDATE_FALLBACK_BLACK_CLEARANCE_M", str(DEFAULT_FALLBACK_BLACK_CLEARANCE_M))),
        help="保护档：候选不足时的无黑圆半径（米），默认 0.2。",
    )
    parser.add_argument(
        "--fallback-min-robot-distance-m",
        type=float,
        default=float(os.getenv("CANDIDATE_FALLBACK_MIN_ROBOT_DISTANCE_M", str(DEFAULT_FALLBACK_MIN_ROBOT_DISTANCE_M))),
        help="保护档：候选不足时的离车直线距离下限（米），默认 0.4。",
    )
    parser.add_argument(
        "--map-resolution",
        type=float,
        default=None,
        help="地图分辨率 m/px；提供后启用米制硬约束（硬离黑 / 硬离车）。",
    )
    parser.add_argument("--min-goal-distance-ratio", type=float, default=0.1,
                        help="目标距离硬下限/地图最长边，默认 0.1。")
    parser.add_argument("--max-goal-distance-ratio", type=float, default=0.35,
                        help="目标距离硬上限/地图最长边，默认 0.35。")
    parser.add_argument("--preferred-min-distance-ratio", type=float, default=0.2,
                        help="优选距离下限，默认 0.2。")
    parser.add_argument("--preferred-max-distance-ratio", type=float, default=0.3,
                        help="优选距离上限，默认 0.3。")
    parser.add_argument("--near-candidate-slack-ratio", type=float, default=0.05,
                        help="候选最远距离最多比最近候选多出的比例，默认 0.05。")
    parser.add_argument("--front-cone-deg", type=float, default=90.0,
                        help="小车朝向扇区半角（度）；候选点与朝向夹角不得超过此值，默认 90。")
    parser.add_argument("--min-display-candidates", type=int, default=5,
                        help="每个案例至少展示给 Qwen 的候选数量（不足时自动放宽筛选），默认 5。")
    parser.add_argument("--max-candidates", type=int, default=12,
                        help="每个案例最多给 Qwen 展示多少个目标候选。")
    parser.add_argument("--pose-candidates", type=int, default=None,
                        help=argparse.SUPPRESS)
    parser.add_argument("--pose-temperature", type=float, default=0.85,
                        help="Qwen 随机选择机器人位姿时的温度。")

    parser.add_argument("--model-image-side", type=int, default=1600)
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--max-tokens", type=int, default=220)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true",
                        help="不调用 Qwen，使用确定性几何评分选点，用于检查地图处理。")
    parser.add_argument("--hide-prompts", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def get_api_key(cli_key: Optional[str]) -> Optional[str]:
    return cli_key or os.getenv("DASHSCOPE_API_KEY") or os.getenv("QWEN_API_KEY")


def detect_unknown_value(gray: np.ndarray, occupied_threshold: int,
                         free_threshold: int) -> int:
    middle = gray[(gray > occupied_threshold + 8) & (gray < free_threshold - 8)]
    if middle.size == 0:
        return 205
    hist = np.bincount(middle.ravel(), minlength=256)
    return int(np.argmax(hist))


def keep_large_components(mask: np.ndarray, minimum_area: int) -> Tuple[np.ndarray, np.ndarray, Dict[int, int]]:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    kept = np.zeros_like(mask, dtype=bool)
    relabeled = np.zeros_like(labels, dtype=np.int32)
    areas: Dict[int, int] = {}
    new_id = 1
    for old_id in range(1, count):
        area = int(stats[old_id, cv2.CC_STAT_AREA])
        if area >= minimum_area:
            component = labels == old_id
            kept |= component
            relabeled[component] = new_id
            areas[new_id] = area
            new_id += 1
    return kept, relabeled, areas


def build_semantic_masks(gray: np.ndarray, args: argparse.Namespace) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[int, int], int]:
    unknown_value = (
        detect_unknown_value(gray, args.occupied_threshold, args.free_threshold)
        if args.unknown_value < 0 else int(args.unknown_value)
    )

    free = gray >= args.free_threshold
    unknown_raw = np.abs(gray.astype(np.int16) - unknown_value) <= args.unknown_tolerance
    unknown_raw &= ~free
    unknown, unknown_labels, unknown_areas = keep_large_components(
        unknown_raw, args.unknown_min_area
    )

    blocked = ~(free | unknown)
    blocked |= gray <= args.occupied_threshold

    semantic = np.zeros((*gray.shape, 3), dtype=np.uint8)
    semantic[blocked] = BLOCKED_BGR
    semantic[unknown] = UNKNOWN_BGR
    semantic[free] = FREE_BGR
    return semantic, free, unknown, blocked, unknown_labels, unknown_areas, unknown_value


def disk_kernel(radius: int) -> np.ndarray:
    radius = max(1, int(radius))
    size = radius * 2 + 1
    kernel = np.zeros((size, size), dtype=np.uint8)
    cv2.circle(kernel, (radius, radius), radius, 1, thickness=-1)
    return kernel


def resolve_map_resolution(args: argparse.Namespace, resolution: Optional[float] = None) -> Optional[float]:
    if resolution is not None and float(resolution) > 0:
        return float(resolution)
    mapped = getattr(args, "map_resolution", None)
    if mapped is not None and float(mapped) > 0:
        return float(mapped)
    return None


def hard_black_clearance_m(args: argparse.Namespace) -> float:
    val = getattr(args, "hard_black_clearance_m", None)
    if val is not None:
        return float(val)
    return float(os.environ.get("CANDIDATE_HARD_BLACK_CLEARANCE_M", DEFAULT_HARD_BLACK_CLEARANCE_M))


def hard_min_robot_distance_m(args: argparse.Namespace) -> float:
    val = getattr(args, "hard_min_robot_distance_m", None)
    if val is not None:
        return float(val)
    return float(os.environ.get("CANDIDATE_HARD_MIN_ROBOT_DISTANCE_M", DEFAULT_HARD_MIN_ROBOT_DISTANCE_M))


def hard_black_clearance_px(args: argparse.Namespace, resolution: Optional[float]) -> Optional[int]:
    """候选周围无黑圆半径（像素）。有 resolution 时强制；否则返回 None。"""
    res = resolve_map_resolution(args, resolution)
    if res is None:
        return None
    return max(1, int(math.ceil(hard_black_clearance_m(args) / res)))


def hard_min_robot_distance_ratio(
    args: argparse.Namespace,
    longest: int,
    resolution: Optional[float],
) -> float:
    """把硬离车距离换成 distance_ratio；无 resolution 时退回 ratio 参数。"""
    res = resolve_map_resolution(args, resolution)
    if res is None:
        return float(args.min_goal_distance_ratio)
    px = hard_min_robot_distance_m(args) / res
    return max(float(args.min_goal_distance_ratio), px / float(max(longest, 1)))


def fallback_black_clearance_m(args: argparse.Namespace) -> float:
    val = getattr(args, "fallback_black_clearance_m", None)
    if val is not None:
        return float(val)
    return float(os.environ.get("CANDIDATE_FALLBACK_BLACK_CLEARANCE_M", DEFAULT_FALLBACK_BLACK_CLEARANCE_M))


def fallback_min_robot_distance_m(args: argparse.Namespace) -> float:
    val = getattr(args, "fallback_min_robot_distance_m", None)
    if val is not None:
        return float(val)
    return float(os.environ.get("CANDIDATE_FALLBACK_MIN_ROBOT_DISTANCE_M", DEFAULT_FALLBACK_MIN_ROBOT_DISTANCE_M))


def set_candidate_geometry_meters(
    args: argparse.Namespace,
    *,
    black_clearance_m: float,
    min_robot_distance_m: float,
    longest: int,
    resolution: Optional[float],
) -> None:
    """设置当前几何约束档位（米），并同步 min_goal_distance_ratio。"""
    args.hard_black_clearance_m = float(black_clearance_m)
    args.hard_min_robot_distance_m = float(min_robot_distance_m)
    base_ratio = float(getattr(args, "base_min_goal_distance_ratio", args.min_goal_distance_ratio))
    args.base_min_goal_distance_ratio = base_ratio
    res = resolve_map_resolution(args, resolution)
    if res is not None and res > 0:
        hard_min_ratio = (float(min_robot_distance_m) / res) / float(max(longest, 1))
        args.min_goal_distance_ratio = max(base_ratio, hard_min_ratio)
    else:
        args.min_goal_distance_ratio = base_ratio


def geometry_constraint_tiers(args: argparse.Namespace) -> Tuple[Tuple[str, float, float], ...]:
    """返回 (tier_name, black_clearance_m, min_robot_distance_m)。

    strict 使用初始严格值（不受 set_candidate_geometry_meters 覆盖影响）。
    """
    strict_black = float(
        getattr(args, "strict_black_clearance_m", None)
        if getattr(args, "strict_black_clearance_m", None) is not None
        else DEFAULT_HARD_BLACK_CLEARANCE_M
    )
    strict_dist = float(
        getattr(args, "strict_min_robot_distance_m", None)
        if getattr(args, "strict_min_robot_distance_m", None) is not None
        else DEFAULT_HARD_MIN_ROBOT_DISTANCE_M
    )
    return (
        ("strict", strict_black, strict_dist),
        ("protect_fallback", fallback_black_clearance_m(args), fallback_min_robot_distance_m(args)),
    )


def nearest_unknown_component(unknown_labels: np.ndarray, x: int, y: int,
                              search_radius: int) -> int:
    h, w = unknown_labels.shape
    x0, x1 = max(0, x - search_radius), min(w, x + search_radius + 1)
    y0, y1 = max(0, y - search_radius), min(h, y + search_radius + 1)
    crop = unknown_labels[y0:y1, x0:x1]
    values, counts = np.unique(crop[crop > 0], return_counts=True)
    if len(values) == 0:
        return 0
    return int(values[int(np.argmax(counts))])


def _scan_frontier_pool(
    free: np.ndarray,
    unknown: np.ndarray,
    blocked: np.ndarray,
    unknown_labels: np.ndarray,
    unknown_areas: Dict[int, int],
    longest: int,
    args: argparse.Namespace,
    *,
    black_clearance_px: int,
    robot_clearance_px: int,
    frontier_gap_px: int,
    unknown_min_area: int,
    spacing_px: int,
    max_global_candidates: int,
) -> Tuple[List[FrontierCandidate], np.ndarray, np.ndarray, np.ndarray]:
    obstacle_distance = cv2.distanceTransform((~blocked).astype(np.uint8), cv2.DIST_L2, 5)
    unknown_distance = cv2.distanceTransform((~unknown).astype(np.uint8), cv2.DIST_L2, 5)

    navigable = free & (obstacle_distance >= robot_clearance_px)
    frontier_mask = (
        navigable
        & (obstacle_distance >= black_clearance_px)
        & (unknown_distance <= frontier_gap_px)
    )
    _, nav_labels = cv2.connectedComponents(navigable.astype(np.uint8), connectivity=4)

    ys, xs = np.where(frontier_mask)
    if len(xs) == 0:
        return [], navigable, nav_labels, frontier_mask

    scored: List[Tuple[float, int, int]] = []
    for x, y in zip(xs.tolist(), ys.tolist()):
        score = float(obstacle_distance[y, x]) - 1.5 * float(unknown_distance[y, x])
        scored.append((score, x, y))
    scored.sort(reverse=True)

    selected_xy: List[Tuple[int, int]] = []
    candidates: List[FrontierCandidate] = []
    for _, x, y in scored:
        if any((x - sx) ** 2 + (y - sy) ** 2 < spacing_px ** 2 for sx, sy in selected_xy):
            continue
        component_id = int(nav_labels[y, x])
        if component_id <= 0:
            continue
        unknown_id = nearest_unknown_component(unknown_labels, x, y, frontier_gap_px + 2)
        unknown_area = int(unknown_areas.get(unknown_id, 0))
        if unknown_area < unknown_min_area:
            continue

        selected_xy.append((x, y))
        candidates.append(
            FrontierCandidate(
                global_id=len(candidates) + 1,
                x=x,
                y=y,
                component_id=component_id,
                obstacle_clearance_px=float(obstacle_distance[y, x]),
                unknown_distance_px=float(unknown_distance[y, x]),
                unknown_area_px=unknown_area,
            )
        )
        if len(candidates) >= max_global_candidates:
            break

    return candidates, navigable, nav_labels, frontier_mask


def generate_frontier_candidates(
    free: np.ndarray,
    unknown: np.ndarray,
    blocked: np.ndarray,
    unknown_labels: np.ndarray,
    unknown_areas: Dict[int, int],
    longest: int,
    args: argparse.Namespace,
    resolution: Optional[float] = None,
) -> Tuple[List[FrontierCandidate], np.ndarray, np.ndarray, np.ndarray, Dict[str, Any]]:
    hard_black_px = hard_black_clearance_px(args, resolution)
    black_clearance_px = max(3, int(round(longest * args.black_clearance_ratio)))
    if hard_black_px is not None:
        # 硬约束优先：周围 0.35m 圆内不得有黑障碍点。
        black_clearance_px = max(black_clearance_px, hard_black_px)
    robot_clearance_px = max(3, int(round(longest * args.robot_clearance_ratio)))
    frontier_gap_px = max(1, int(round(longest * args.frontier_gap_ratio)))
    spacing_px = max(5, int(round(longest * args.candidate_spacing_ratio)))
    min_pool = max(5, int(getattr(args, "min_display_candidates", 5)) * 3)
    max_global_candidates = max(80, args.max_candidates * 8)

    # 逐步放宽 gap / unknown / spacing，避免整张图只有 0~1 个全局候选。
    # 当前档位的 black clearance 米制下限（strict 0.35 / protect 0.2）不被 black_scale 放宽。
    relax_profiles = (
        (1.00, 1.00, 1.00, 1.00, 1.00),
        (0.85, 0.85, 1.40, 0.70, 0.85),
        (0.70, 0.70, 1.80, 0.45, 0.70),
        (0.55, 0.55, 2.20, 0.30, 0.55),
    )

    candidates: List[FrontierCandidate] = []
    navigable = np.zeros_like(free, dtype=bool)
    nav_labels = np.zeros(free.shape, dtype=np.int32)
    frontier_mask = np.zeros_like(free, dtype=bool)
    used_black = black_clearance_px
    used_robot = robot_clearance_px
    used_gap = frontier_gap_px
    used_unknown_area = args.unknown_min_area
    used_spacing = spacing_px

    for black_scale, robot_scale, gap_scale, area_scale, spacing_scale in relax_profiles:
        used_black = max(2, int(round(black_clearance_px * black_scale)))
        if hard_black_px is not None:
            used_black = max(used_black, hard_black_px)
        used_robot = max(2, int(round(robot_clearance_px * robot_scale)))
        used_gap = max(1, int(round(frontier_gap_px * gap_scale)))
        used_unknown_area = max(20, int(round(args.unknown_min_area * area_scale)))
        used_spacing = max(3, int(round(spacing_px * spacing_scale)))

        candidates, navigable, nav_labels, frontier_mask = _scan_frontier_pool(
            free,
            unknown,
            blocked,
            unknown_labels,
            unknown_areas,
            longest,
            args,
            black_clearance_px=used_black,
            robot_clearance_px=used_robot,
            frontier_gap_px=used_gap,
            unknown_min_area=used_unknown_area,
            spacing_px=used_spacing,
            max_global_candidates=max_global_candidates,
        )
        if len(candidates) >= min_pool:
            break

    if not candidates:
        hard_msg = ""
        if hard_black_px is not None:
            hard_msg = (
                f"硬约束要求候选周围 {hard_black_clearance_m(args):.2f}m 内无黑障碍"
                f"（约 {hard_black_px}px）。"
            )
        raise RuntimeError(
            "没有找到满足条件的灰白前沿（即使已自动放宽 clearance / gap / unknown 阈值）。"
            + hard_msg
            + "请确认地图存在灰白交界，或适当降低 --black-clearance-ratio / --robot-clearance-ratio。"
        )

    parameters = {
        "black_clearance_px": used_black,
        "robot_clearance_px": used_robot,
        "frontier_gap_px": used_gap,
        "candidate_spacing_px": used_spacing,
        "unknown_min_area_used": used_unknown_area,
        "hard_black_clearance_m": hard_black_clearance_m(args) if hard_black_px is not None else None,
        "hard_black_clearance_px": hard_black_px,
    }
    return candidates, navigable, nav_labels, frontier_mask, parameters


def generate_robot_position_candidates(
    navigable: np.ndarray,
    nav_labels: np.ndarray,
    frontier_mask: np.ndarray,
    frontier_candidates: Sequence[FrontierCandidate],
    desired_count: int,
    longest: int,
    args: argparse.Namespace,
    seed: int,
) -> List[RobotCandidate]:
    """直接生成实际需要数量的随机机器人位置。

    不再构造 36 个“供 Qwen 再筛选”的大候选池。这里只生成 desired_count 个位置。
    每个位置位于安全白色区域，并保证存在严格距离范围内的可达灰白前沿，
    从而后续每个案例都能正常生成目标点。
    """
    frontier_components = {c.component_id for c in frontier_candidates}
    component_mask = navigable & np.isin(nav_labels, list(frontier_components))

    # distanceTransform：非零像素到最近零像素的距离；~frontier_mask 在前沿处为 0。
    distance_to_frontier = cv2.distanceTransform(
        (~frontier_mask).astype(np.uint8), cv2.DIST_L2, 5
    )

    min_px = float(longest * args.min_goal_distance_ratio)
    max_px = float(longest * args.max_goal_distance_ratio)
    preferred_min_px = float(longest * args.preferred_min_distance_ratio)
    preferred_max_px = float(longest * args.preferred_max_distance_ratio)

    hard_mask = (
        component_mask
        & (distance_to_frontier >= min_px)
        & (distance_to_frontier <= max_px)
    )
    preferred_mask = (
        hard_mask
        & (distance_to_frontier >= preferred_min_px)
        & (distance_to_frontier <= preferred_max_px)
    )

    if int(np.count_nonzero(hard_mask)) == 0:
        raise RuntimeError(
            "没有机器人位置能在严格距离范围内到达灰白前沿。"
            "请适当增大 --max-goal-distance-ratio，或降低 --min-goal-distance-ratio。"
        )

    rng = random.Random(seed)
    h, w = navigable.shape
    spacing = max(10.0, 0.050 * math.hypot(w, h))
    chosen: List[Tuple[int, int]] = []

    def add_from_mask(mask: np.ndarray, current_spacing: float) -> None:
        ys, xs = np.where(mask)
        order = list(range(len(xs)))
        rng.shuffle(order)
        # 优选更接近优选区中心的位置，但保留随机性。
        target_px = longest * (
            args.preferred_min_distance_ratio + args.preferred_max_distance_ratio
        ) / 2.0
        order.sort(
            key=lambda idx: abs(float(distance_to_frontier[int(ys[idx]), int(xs[idx])]) - target_px)
            + rng.random() * max(1.0, longest * 0.015)
        )
        for idx in order:
            x, y = int(xs[idx]), int(ys[idx])
            if all(math.hypot(x - sx, y - sy) >= current_spacing for sx, sy in chosen):
                chosen.append((x, y))
                if len(chosen) >= desired_count:
                    return

    # 先从优选距离带抽样，再用严格距离带补足；绝不从 max 之外补点。
    add_from_mask(preferred_mask, spacing)
    current_spacing = spacing
    while len(chosen) < desired_count and current_spacing > 6.0:
        add_from_mask(hard_mask, current_spacing)
        current_spacing *= 0.78

    if len(chosen) < desired_count:
        # 分散间距只是美观偏好，不是硬限制。若仍未补足，就取消间距要求，
        # 从所有合格白区像素中随机补足互不重复的位置。
        ys, xs = np.where(hard_mask)
        remaining = [
            (int(x), int(y))
            for x, y in zip(xs.tolist(), ys.tolist())
            if (int(x), int(y)) not in chosen
        ]
        rng.shuffle(remaining)
        for point in remaining:
            chosen.append(point)
            if len(chosen) >= desired_count:
                break

    if len(chosen) < desired_count:
        raise RuntimeError(
            f"整张地图中只有 {len(chosen)} 个互不重复的有效白区像素能够在严格距离范围内找到前沿，"
            f"无法生成 {desired_count} 个机器人位置。"
        )

    return [
        RobotCandidate(i + 1, x, y, int(nav_labels[y, x]))
        for i, (x, y) in enumerate(chosen[:desired_count])
    ]


def fallback_select_poses(
    robot_candidates: Sequence[RobotCandidate],
    case_count: int,
    seed: int,
) -> List[Pose]:
    rng = random.Random(seed ^ 0x5A17)
    picked = rng.sample(list(robot_candidates), k=case_count)
    return [Pose(c.x, c.y, rng.uniform(-180.0, 180.0)) for c in picked]


def wrap_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def candidate_metrics(pose: Pose, candidate: FrontierCandidate, longest: int) -> Dict[str, float]:
    dx = candidate.x - pose.x
    dy_image = candidate.y - pose.y
    distance_ratio = math.hypot(dx, dy_image) / float(longest)

    # 地图图像 y 向下；数学 yaw 的正方向取逆时针，所以使用 -dy_image。
    bearing_deg = math.degrees(math.atan2(-dy_image, dx))
    heading_delta = wrap_angle_deg(bearing_deg - pose.yaw_deg)
    return {
        "distance_ratio": distance_ratio,
        "bearing_deg": bearing_deg,
        "heading_delta_deg": heading_delta,
        "forward_cos": math.cos(math.radians(heading_delta)),
    }


def direct_path_is_clear(
    pose: Pose,
    candidate: FrontierCandidate,
    nav_labels: np.ndarray,
) -> bool:
    """检查机器人到候选点的直线是否始终位于同一安全白色连通区。

    nav_labels 已由经过机器人安全距离腐蚀后的 navigable 区域生成，因此这里不仅能
    拦截穿过黑墙的直线，也会拦截穿越灰色未知区或贴墙过近的直线。
    """
    component_id = int(nav_labels[pose.y, pose.x])
    if component_id <= 0:
        return False

    dx = candidate.x - pose.x
    dy = candidate.y - pose.y
    steps = max(abs(dx), abs(dy), 1)
    xs = np.rint(np.linspace(pose.x, candidate.x, steps + 1)).astype(np.int32)
    ys = np.rint(np.linspace(pose.y, candidate.y, steps + 1)).astype(np.int32)
    xs = np.clip(xs, 0, nav_labels.shape[1] - 1)
    ys = np.clip(ys, 0, nav_labels.shape[0] - 1)
    return bool(np.all(nav_labels[ys, xs] == component_id))


def supplement_forward_candidates(
    pose: Pose,
    component_id: int,
    nav_labels: np.ndarray,
    navigable: np.ndarray,
    unknown: np.ndarray,
    blocked: np.ndarray,
    longest: int,
    args: argparse.Namespace,
    existing: Sequence[FrontierCandidate],
    need: int,
    resolution: Optional[float] = None,
) -> List[FrontierCandidate]:
    """当前方严格前沿不足时，从同一连通白区补充前方 navigable 点。

    补充点同样遵守硬约束：离黑 >= hard_black_clearance_m，离车 >= hard_min_robot_distance_m。
    """
    if need <= 0:
        return []

    obstacle_distance = cv2.distanceTransform((~blocked).astype(np.uint8), cv2.DIST_L2, 5)
    unknown_distance = cv2.distanceTransform((~unknown).astype(np.uint8), cv2.DIST_L2, 5)
    max_distance = args.max_goal_distance_ratio * 2.2
    hard_black_px = hard_black_clearance_px(args, resolution)
    hard_min_ratio = hard_min_robot_distance_ratio(args, longest, resolution)
    # 无 resolution 时仍用 ratio 黑净空作为补充点的安全下限。
    min_black_px = float(
        hard_black_px
        if hard_black_px is not None
        else max(2, int(round(longest * args.black_clearance_ratio)))
    )

    occupied_xy = {(c.x, c.y) for c in existing}
    next_global_id = max((c.global_id for c in existing), default=0)
    component_mask = navigable & (nav_labels == component_id)
    ys, xs = np.where(component_mask)

    supplement_tiers = (
        (max(2, int(round(longest * args.frontier_gap_ratio * 4.0))), max(3, int(round(longest * args.candidate_spacing_ratio * 0.55)))),
        (max(3, int(round(longest * args.frontier_gap_ratio * 8.0))), max(3, int(round(longest * args.candidate_spacing_ratio * 0.40)))),
        (max(9999, int(round(longest * args.frontier_gap_ratio * 20.0))), max(2, int(round(longest * args.candidate_spacing_ratio * 0.30)))),
    )

    added: List[FrontierCandidate] = []
    for max_unknown_gap, spacing_px in supplement_tiers:
        scored: List[Tuple[float, int, int]] = []
        for x, y in zip(xs.tolist(), ys.tolist()):
            if (x, y) in occupied_xy:
                continue
            if float(obstacle_distance[y, x]) < min_black_px - 1e-9:
                continue
            probe = FrontierCandidate(
                global_id=0,
                x=int(x),
                y=int(y),
                component_id=component_id,
                obstacle_clearance_px=float(obstacle_distance[y, x]),
                unknown_distance_px=float(unknown_distance[y, x]),
                unknown_area_px=0,
            )
            metrics = candidate_metrics(pose, probe, longest)
            if abs(metrics["heading_delta_deg"]) > args.front_cone_deg + 1e-9:
                continue
            if metrics["distance_ratio"] < hard_min_ratio - 1e-9:
                continue
            if metrics["distance_ratio"] > max_distance + 1e-9:
                continue
            if float(unknown_distance[y, x]) > float(max_unknown_gap):
                continue
            score = float(obstacle_distance[y, x]) - 1.2 * float(unknown_distance[y, x])
            scored.append((score, int(x), int(y)))
        scored.sort(reverse=True)

        for _, x, y in scored:
            if any((x - sx) ** 2 + (y - sy) ** 2 < spacing_px ** 2 for sx, sy in occupied_xy):
                continue
            next_global_id += 1
            candidate = FrontierCandidate(
                global_id=next_global_id,
                x=x,
                y=y,
                component_id=component_id,
                obstacle_clearance_px=float(obstacle_distance[y, x]),
                unknown_distance_px=float(unknown_distance[y, x]),
                unknown_area_px=0,
            )
            added.append(candidate)
            occupied_xy.add((x, y))
            if len(existing) + len(added) >= need:
                return added
    return added


def compute_candidate_visit_attributes(
    candidate: FrontierCandidate,
    *,
    visited_mask: np.ndarray,
    visited_dist_field: Optional[np.ndarray],
    unknown: np.ndarray,
    resolution: float,
    min_obstacle_clearance_px: float,
) -> Dict[str, Any]:
    h, w = visited_mask.shape
    x, y = candidate.x, candidate.y
    inside = bool(visited_mask[y, x]) if 0 <= y < h and 0 <= x < w else False
    dist_px = (
        float(visited_dist_field[y, x])
        if visited_dist_field is not None
        else float("inf")
    )
    dist_m = dist_px * resolution
    y0, y1 = max(0, y - 3), min(h, y + 4)
    x0, x1 = max(0, x - 3), min(w, x + 4)
    patch_unknown = unknown[y0:y1, x0:x1]
    unexplored_gain = int(np.count_nonzero(patch_unknown))
    frontier_area = int(candidate.unknown_area_px)
    geometry_score = float(candidate.obstacle_clearance_px) + 0.5 * unexplored_gain
    overlap_ratio = 0.0
    if visited_mask.any():
        ring = visited_mask[max(0, y - 2) : min(h, y + 3), max(0, x - 2) : min(w, x + 3)]
        overlap_ratio = float(np.count_nonzero(ring)) / max(1, ring.size)
    return {
        "distance_to_visited_m": dist_m,
        "inside_visited_corridor": inside,
        "visited_overlap_ratio": overlap_ratio,
        "unexplored_gain": unexplored_gain,
        "frontier_area": frontier_area,
        "geometry_score": geometry_score,
        "obstacle_clearance_ok": candidate.obstacle_clearance_px >= min_obstacle_clearance_px,
    }


def apply_visited_candidate_policy(
    pose: Pose,
    candidates: Sequence[FrontierCandidate],
    nav_labels: np.ndarray,
    longest: int,
    args: argparse.Namespace,
    *,
    visited_mask: np.ndarray,
    visited_dist_field: Optional[np.ndarray],
    unknown: np.ndarray,
    blocked: np.ndarray,
    resolution: float,
) -> Tuple[List[FrontierCandidate], Dict[int, Dict[str, Any]], Dict[str, int], int]:
    """按已走区域策略分层过滤/重排候选（几何安全永不放宽）。

    Tier 1：严格安全 + 严格未探索收益
    Tier 2：保持障碍/连通安全，只放宽 visited 惩罚
    Tier 3：保持几何安全，选择最优安全候选

    硬约束（永不放宽）：
    - 候选周围 hard_black_clearance_m 圆内无黑障碍
    - 到小车直线距离 >= hard_min_robot_distance_m

    返回 (kept, attrs, reject_reason_counts, tier_used)
    """
    hard_black_px = hard_black_clearance_px(args, resolution)
    min_clear = float(longest) * float(args.black_clearance_ratio)
    if hard_black_px is not None:
        min_clear = max(min_clear, float(hard_black_px))
    hard_min_m = hard_min_robot_distance_m(args)
    # 将像素阈值换成与 resolution 相关的面积：约 0.0075 m^2 等价于 0.05m 栅格下 ~3 像素
    min_unexplored_gain_px = max(
        1,
        int(round(float(os.environ.get("CANDIDATE_MIN_UNEXPLORED_AREA_M2", "0.0075")) / max(resolution ** 2, 1e-9))),
    )
    reject_counts: Dict[str, int] = {
        "clearance": 0,
        "disconnected": 0,
        "inside_visited": 0,
        "low_unknown_gain": 0,
        "too_close": 0,
        "too_far": 0,
        "map_edge": 0,
        "narrow_passage": 0,
    }

    robot_comp = int(nav_labels[pose.y, pose.x])
    h, w = nav_labels.shape
    edge_margin = max(2, int(round(0.15 / max(resolution, 1e-6))))

    scored: List[Tuple[FrontierCandidate, Dict[str, Any]]] = []
    for candidate in candidates:
        if int(nav_labels[candidate.y, candidate.x]) != robot_comp:
            reject_counts["disconnected"] += 1
            continue
        if (
            candidate.x < edge_margin
            or candidate.y < edge_margin
            or candidate.x >= w - edge_margin
            or candidate.y >= h - edge_margin
        ):
            reject_counts["map_edge"] += 1
            continue
        dist_m = candidate_metrics(pose, candidate, longest)["distance_ratio"] * float(longest) * float(resolution)
        if dist_m < hard_min_m - 1e-9:
            reject_counts["too_close"] += 1
            continue
        a = compute_candidate_visit_attributes(
            candidate,
            visited_mask=visited_mask,
            visited_dist_field=visited_dist_field,
            unknown=unknown,
            resolution=resolution,
            min_obstacle_clearance_px=min_clear,
        )
        if not a["obstacle_clearance_ok"]:
            reject_counts["clearance"] += 1
            continue
        # 窄通道：障碍 clearance 仅略高于阈值时记入统计，但仍允许进入后续几何层
        if candidate.obstacle_clearance_px < min_clear * 1.15:
            reject_counts["narrow_passage"] += 1
        scored.append((candidate, a))

    def filter_tier(tier: int) -> List[Tuple[FrontierCandidate, Dict[str, Any]]]:
        out: List[Tuple[FrontierCandidate, Dict[str, Any]]] = []
        for candidate, a in scored:
            if tier == 1:
                if a["inside_visited_corridor"] and a["unexplored_gain"] < min_unexplored_gain_px:
                    reject_counts["inside_visited"] += 1
                    reject_counts["low_unknown_gain"] += 1
                    continue
                if a["unexplored_gain"] < min_unexplored_gain_px:
                    reject_counts["low_unknown_gain"] += 1
                    continue
            elif tier == 2:
                # 放宽 visited 惩罚：允许 inside_visited，但仍要求一定未知收益
                if a["unexplored_gain"] < max(1, min_unexplored_gain_px // 3):
                    reject_counts["low_unknown_gain"] += 1
                    continue
            # tier 3: 仅几何安全（已在 scored 中保证）
            out.append((candidate, a))
        return out

    kept_pairs: List[Tuple[FrontierCandidate, Dict[str, Any]]] = []
    tier_used = 1
    for tier in (1, 2, 3):
        kept_pairs = filter_tier(tier)
        if kept_pairs:
            tier_used = tier
            break

    if not kept_pairs:
        raise RuntimeError("已走区域策略过滤后无可用候选（三层回退后仍为空）。")

    attrs: Dict[int, Dict[str, Any]] = {c.global_id: a for c, a in kept_pairs}
    kept: List[FrontierCandidate] = [c for c, _ in kept_pairs]

    def rank_key(c: FrontierCandidate) -> Tuple[float, float, float, float, float]:
        m = candidate_metrics(pose, c, longest)
        a = attrs[c.global_id]
        path_clear = 0.0 if direct_path_is_clear(pose, c, nav_labels) else 1.0
        front_bonus = 0.0 if abs(m["heading_delta_deg"]) <= args.front_cone_deg * 0.6 else 0.5
        visited_penalty = 0.0
        if visited_dist_field is not None and tier_used == 1:
            visited_penalty = max(0.0, 1.5 - a["distance_to_visited_m"] / max(resolution, 0.05))
        elif visited_dist_field is not None and tier_used == 2:
            visited_penalty = 0.35 * max(0.0, 1.5 - a["distance_to_visited_m"] / max(resolution, 0.05))
        unknown_bonus = -min(1.0, a["unexplored_gain"] / 20.0)
        return (path_clear, visited_penalty + front_bonus, -a["geometry_score"], m["distance_ratio"], unknown_bonus)

    kept.sort(key=rank_key)
    return kept, attrs, reject_counts, tier_used


def choose_case_candidates(
    pose: Pose,
    nav_labels: np.ndarray,
    candidates: Sequence[FrontierCandidate],
    longest: int,
    args: argparse.Namespace,
    *,
    navigable: Optional[np.ndarray] = None,
    unknown: Optional[np.ndarray] = None,
    blocked: Optional[np.ndarray] = None,
    resolution: Optional[float] = None,
) -> Tuple[List[FrontierCandidate], Dict[str, float]]:
    """返回给 Qwen 展示的前方扇区候选与本案例距离元数据。

    约束：候选与小车朝向夹角不超过 front_cone_deg（默认 90°）。
    若严格距离下不足 min_display_candidates，会逐级放宽距离上限；
    若仍不足，由上层 geometry protect 降到 0.4m/0.2m 再重跑。
    当前档位内：离车直线距离不得低于 hard_min_robot_distance_m。
    """
    component_id = int(nav_labels[pose.y, pose.x])
    reachable = [c for c in candidates if c.component_id == component_id]
    if not reachable:
        raise RuntimeError("该机器人位置所在白色连通区没有安全前沿。")

    metrics_by_id: Dict[int, Dict[str, float]] = {}
    path_clear_by_id: Dict[int, bool] = {}
    for candidate in reachable:
        metrics = candidate_metrics(pose, candidate, longest)
        metrics_by_id[candidate.global_id] = metrics
        path_clear_by_id[candidate.global_id] = direct_path_is_clear(
            pose, candidate, nav_labels
        )

    heading_limit = float(args.front_cone_deg)
    hard_min_ratio = hard_min_robot_distance_ratio(args, longest, resolution)
    hard_min_m = hard_min_robot_distance_m(args)
    forward = [
        c for c in reachable
        if abs(metrics_by_id[c.global_id]["heading_delta_deg"]) <= heading_limit + 1e-9
        and metrics_by_id[c.global_id]["distance_ratio"] >= hard_min_ratio - 1e-9
    ]
    if not forward:
        raise RuntimeError(
            f"小车前方 ±{heading_limit:.0f}° 内没有距车直线距离 ≥ {hard_min_m:.2f}m 的可达灰白前沿。"
            "请调整朝向或继续建图扩大前方白色区域。"
        )

    min_show = max(1, int(getattr(args, "min_display_candidates", 5)))
    if (
        len(forward) < min_show
        and navigable is not None
        and unknown is not None
        and blocked is not None
    ):
        extra = supplement_forward_candidates(
            pose,
            component_id,
            nav_labels,
            navigable,
            unknown,
            blocked,
            longest,
            args,
            forward,
            min_show,
            resolution=resolution,
        )
        if extra:
            forward = list(forward) + extra
            for candidate in extra:
                metrics_by_id[candidate.global_id] = candidate_metrics(
                    pose, candidate, longest
                )
                path_clear_by_id[candidate.global_id] = direct_path_is_clear(
                    pose, candidate, nav_labels
                )
            # 补充点已按硬下限过滤，这里再保险一次。
            forward = [
                c for c in forward
                if metrics_by_id[c.global_id]["distance_ratio"] >= hard_min_ratio - 1e-9
            ]

    display_cap = max(min_show, args.max_candidates)

    # 距离 tier 可放宽上限，但下限永不低于硬离车距离。
    distance_tiers = (
        (max(hard_min_ratio, args.min_goal_distance_ratio), args.max_goal_distance_ratio),
        (max(hard_min_ratio, args.min_goal_distance_ratio * 0.75), args.max_goal_distance_ratio * 1.20),
        (max(hard_min_ratio, args.min_goal_distance_ratio * 0.50), args.max_goal_distance_ratio * 1.50),
        (hard_min_ratio, args.max_goal_distance_ratio * 1.80),
    )
    pool: List[FrontierCandidate] = []
    distance_limit = args.max_goal_distance_ratio
    for min_r, max_r in distance_tiers:
        tier = [
            c for c in forward
            if min_r - 1e-9 <= metrics_by_id[c.global_id]["distance_ratio"] <= max_r + 1e-9
        ]
        if tier:
            pool = tier
            distance_limit = max_r
        if len(pool) >= min_show:
            break
    if not pool:
        pool = [
            c for c in forward
            if metrics_by_id[c.global_id]["distance_ratio"] >= hard_min_ratio - 1e-9
        ]
        if not pool:
            raise RuntimeError(
                f"没有满足硬离车距离 ≥ {hard_min_m:.2f}m 的前方候选。"
            )
        distance_limit = max(
            args.max_goal_distance_ratio * 1.80,
            max(metrics_by_id[c.global_id]["distance_ratio"] for c in pool),
        )

    nearest_distance = min(metrics_by_id[c.global_id]["distance_ratio"] for c in pool)

    front_clear = [
        c for c in pool
        if path_clear_by_id[c.global_id]
        and abs(metrics_by_id[c.global_id]["heading_delta_deg"]) <= heading_limit
    ]
    front_blocked = [
        c for c in pool
        if not path_clear_by_id[c.global_id]
        and abs(metrics_by_id[c.global_id]["heading_delta_deg"]) <= heading_limit
    ]

    def preferred_penalty(distance_ratio: float) -> float:
        if distance_ratio < args.preferred_min_distance_ratio:
            return args.preferred_min_distance_ratio - distance_ratio
        if distance_ratio > args.preferred_max_distance_ratio:
            return distance_ratio - args.preferred_max_distance_ratio
        return 0.0

    def rank_key(candidate: FrontierCandidate) -> Tuple[float, float, float, float]:
        m = metrics_by_id[candidate.global_id]
        abs_delta = abs(m["heading_delta_deg"])
        path_clear = path_clear_by_id[candidate.global_id]
        direction_bucket = 0.0 if path_clear else 1.0
        return (
            direction_bucket,
            preferred_penalty(m["distance_ratio"]),
            m["distance_ratio"],
            abs_delta,
        )

    ranked = sorted(pool, key=rank_key)

    target_count = min(display_cap, max(min_show, len(ranked)))
    selected: List[FrontierCandidate] = []
    min_bearing_sep = 18.0
    for candidate in ranked:
        if len(selected) >= target_count:
            break
        angle = metrics_by_id[candidate.global_id]["bearing_deg"]
        if len(selected) < min_show:
            selected.append(candidate)
            continue
        if all(
            abs(wrap_angle_deg(angle - metrics_by_id[s.global_id]["bearing_deg"])) >= min_bearing_sep
            for s in selected
        ):
            selected.append(candidate)

    while len(selected) < target_count:
        min_bearing_sep = max(8.0, min_bearing_sep * 0.75)
        added = False
        for candidate in ranked:
            if candidate in selected:
                continue
            angle = metrics_by_id[candidate.global_id]["bearing_deg"]
            if all(
                abs(wrap_angle_deg(angle - metrics_by_id[s.global_id]["bearing_deg"])) >= min_bearing_sep
                for s in selected
            ):
                selected.append(candidate)
                added = True
                if len(selected) >= target_count:
                    break
        if not added:
            for candidate in ranked:
                if candidate not in selected:
                    selected.append(candidate)
                    if len(selected) >= target_count:
                        break
            break

    meta = {
        "nearest_distance_ratio": nearest_distance,
        "candidate_distance_limit_ratio": distance_limit,
        "strict_min_distance_ratio": max(args.min_goal_distance_ratio, hard_min_ratio),
        "strict_max_distance_ratio": args.max_goal_distance_ratio,
        "preferred_min_distance_ratio": args.preferred_min_distance_ratio,
        "preferred_max_distance_ratio": args.preferred_max_distance_ratio,
        "hard_min_robot_distance_m": hard_min_m,
        "hard_min_distance_ratio": hard_min_ratio,
        "front_clear_candidate_count": float(len(front_clear)),
        "front_blocked_candidate_count": float(len(front_blocked)),
        "forward_candidate_count": float(len(forward)),
        "display_candidate_count": float(len(selected)),
        "heading_cone_deg": heading_limit,
    }
    return selected, meta


def resize_nearest(image: np.ndarray, longest_side: int) -> Tuple[np.ndarray, float]:
    h, w = image.shape[:2]
    scale = longest_side / float(max(h, w))
    new_size = (max(1, int(round(w * scale))), max(1, int(round(h * scale))))
    return cv2.resize(image, new_size, interpolation=cv2.INTER_NEAREST), scale


def draw_pose(image: np.ndarray, pose: Pose, scale: float) -> None:
    x = int(round(pose.x * scale))
    y = int(round(pose.y * scale))
    ref = max(image.shape[:2])
    radius = max(8, int(round(ref * 0.006)))
    arrow_len = max(45, int(round(ref * 0.035)))
    thickness = max(3, int(round(ref * 0.0025)))

    cv2.circle(image, (x, y), radius, ROBOT_BGR, thickness=-1, lineType=cv2.LINE_AA)
    cv2.circle(image, (x, y), radius, (255, 255, 255), thickness=max(1, thickness // 2), lineType=cv2.LINE_AA)
    yaw = math.radians(pose.yaw_deg)
    end = (
        int(round(x + arrow_len * math.cos(yaw))),
        int(round(y - arrow_len * math.sin(yaw))),
    )
    cv2.arrowedLine(image, (x, y), end, HEADING_BGR, thickness=thickness,
                    tipLength=0.28, line_type=cv2.LINE_AA)


def draw_candidate(image: np.ndarray, candidate: FrontierCandidate, local_id: int,
                   scale: float, selected: bool = False,
                   show_label: bool = True) -> None:
    x = int(round(candidate.x * scale))
    y = int(round(candidate.y * scale))
    ref = max(image.shape[:2])
    radius = max(11, int(round(ref * (0.009 if selected else 0.007))))
    thickness = max(2, int(round(ref * 0.0018)))
    color = GOAL_BGR if selected else CANDIDATE_BGR

    cv2.circle(image, (x, y), radius, color, thickness=-1, lineType=cv2.LINE_AA)
    cv2.circle(image, (x, y), radius, (0, 0, 0), thickness=thickness, lineType=cv2.LINE_AA)
    if show_label:
        label = str(local_id)
        font_scale = max(0.45, ref / 1500.0 * 0.62)
        text_thickness = max(1, int(round(ref / 900.0)))
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thickness
        )
        cv2.putText(
            image, label, (x - tw // 2, y + th // 2),
            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0),
            text_thickness, cv2.LINE_AA,
        )


def draw_robot_position_candidate(image: np.ndarray, candidate: RobotCandidate, scale: float) -> None:
    x = int(round(candidate.x * scale))
    y = int(round(candidate.y * scale))
    ref = max(image.shape[:2])
    radius = max(11, int(round(ref * 0.007)))
    thickness = max(2, int(round(ref * 0.0018)))
    cv2.circle(image, (x, y), radius, POSE_CANDIDATE_BGR, thickness=-1, lineType=cv2.LINE_AA)
    cv2.circle(image, (x, y), radius, (0, 0, 0), thickness=thickness, lineType=cv2.LINE_AA)
    label = str(candidate.position_id)
    font_scale = max(0.45, ref / 1500.0 * 0.62)
    text_thickness = max(1, int(round(ref / 900.0)))
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thickness)
    cv2.putText(image, label, (x - tw // 2, y + th // 2),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0),
                text_thickness, cv2.LINE_AA)


def build_pose_selection_image(
    semantic: np.ndarray,
    robot_candidates: Sequence[RobotCandidate],
    model_image_side: int,
) -> np.ndarray:
    image, scale = resize_nearest(semantic, model_image_side)
    for candidate in robot_candidates:
        draw_robot_position_candidate(image, candidate, scale)
    put_info_box(image, [
        "QWEN POSE SELECTION",
        f"safe_robot_positions={len(robot_candidates)}",
        "purple IDs = validated white-area positions",
    ])
    return image


def put_info_box(image: np.ndarray, lines: Sequence[str]) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    ref = max(image.shape[:2])
    font_scale = max(0.42, ref / 1600.0 * 0.55)
    thickness = max(1, int(round(ref / 1100.0)))
    line_h = max(20, int(round(30 * ref / 1600.0)))
    max_w = 0
    for line in lines:
        (tw, _), _ = cv2.getTextSize(line, font, font_scale, thickness)
        max_w = max(max_w, tw)
    box_h = 18 + line_h * len(lines)
    overlay = image.copy()
    cv2.rectangle(overlay, (8, 8), (28 + max_w, 8 + box_h), TEXT_BG_BGR, thickness=-1)
    cv2.addWeighted(overlay, 0.90, image, 0.10, 0, image)
    cv2.rectangle(image, (8, 8), (28 + max_w, 8 + box_h), (60, 60, 60), thickness=2)
    y = 8 + line_h
    for line in lines:
        cv2.putText(image, line, (18, y), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
        y += line_h


def build_case_image(
    semantic: np.ndarray,
    pose: Pose,
    case_candidates: Sequence[FrontierCandidate],
    model_image_side: int,
    selected_local_id: Optional[int] = None,
    info_lines: Optional[Sequence[str]] = None,
    show_all_candidates: bool = True,
    show_candidate_labels: bool = True,
) -> Tuple[np.ndarray, float]:
    image, scale = resize_nearest(semantic, model_image_side)
    for local_id, candidate in enumerate(case_candidates, start=1):
        if show_all_candidates or local_id == selected_local_id:
            draw_candidate(
                image,
                candidate,
                local_id,
                scale,
                selected=(local_id == selected_local_id),
                show_label=show_candidate_labels,
            )
    draw_pose(image, pose, scale)
    if info_lines:
        put_info_box(image, info_lines)
    return image, scale


def candidate_table_text(
    pose: Pose,
    case_candidates: Sequence[FrontierCandidate],
    nav_labels: np.ndarray,
    width: int,
    height: int,
    longest: int,
    args: argparse.Namespace,
    *,
    visited_dist_field: Optional[np.ndarray] = None,
) -> str:
    rows = []
    for local_id, candidate in enumerate(case_candidates, start=1):
        m = candidate_metrics(pose, candidate, longest)
        distance = m["distance_ratio"]
        distance_tag = (
            "PREFERRED"
            if args.preferred_min_distance_ratio <= distance <= args.preferred_max_distance_ratio
            else "ALLOWED"
        )
        direction_tag = (
            "FORWARD_HEMISPHERE"
            if abs(m["heading_delta_deg"]) <= args.front_cone_deg + 1e-9
            else "OUT_OF_CONE"
        )
        direct_path_tag = "CLEAR" if direct_path_is_clear(pose, candidate, nav_labels) else "BLOCKED"
        visited_clearance = (
            float(visited_dist_field[candidate.y, candidate.x])
            if visited_dist_field is not None
            else float("inf")
        )
        visited_part = (
            f", visited_clearance_px={visited_clearance:.1f}"
            if visited_dist_field is not None
            else ""
        )
        rows.append(
            f"id={local_id}, u={candidate.x / max(1, width - 1):.6f}, "
            f"v={candidate.y / max(1, height - 1):.6f}, "
            f"distance_ratio={distance:.3f}, distance_tag={distance_tag}, "
            f"heading_delta={m['heading_delta_deg']:.1f}deg, direction_tag={direction_tag}, "
            f"direct_path={direct_path_tag}, "
            f"black_clearance_px={candidate.obstacle_clearance_px:.1f}, "
            f"unknown_area={candidate.unknown_area_px}{visited_part}"
        )
    return "\n".join(rows)


def image_to_data_url(image_bgr: np.ndarray) -> str:
    ok, encoded = cv2.imencode(".png", image_bgr)
    if not ok:
        raise RuntimeError("无法编码输入图像。")
    data = base64.b64encode(encoded.tobytes()).decode("ascii")
    return f"data:image/png;base64,{data}"


def image_to_jpeg_data_url(image_bgr: np.ndarray, quality: int = 85) -> str:
    ok, encoded = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("无法编码 JPEG 输入图像。")
    data = base64.b64encode(encoded.tobytes()).decode("ascii")
    return f"data:image/jpeg;base64,{data}"


def call_qwen(image_bgr: np.ndarray, prompt: str, api_key: str,
              args: argparse.Namespace, *, temperature: Optional[float] = None,
              max_tokens: Optional[int] = None,
              image_format: str = "png") -> Tuple[str, float]:
    endpoint = args.base_url.rstrip("/") + "/chat/completions"
    if image_format == "jpeg":
        image_url = image_to_jpeg_data_url(image_bgr, quality=int(getattr(args, "jpeg_quality", 85)))
    else:
        image_url = image_to_data_url(image_bgr)
    payload = {
        "model": args.model,
        "messages": [
            {
                "role": "system",
                "content": (
                    "你必须只从图中已有候选编号中选择。"
                    f"所有候选已限制在小车朝向 ±{args.front_cone_deg:.0f}° 前方扇区。"
                    "优先 direct_path=CLEAR；无 CLEAR 时可用 BLOCKED。"
                    "浅绿色为已扫过区域，通行等同白色，但应优先选远离浅绿的候选。"
                    "只输出合法 JSON。"
                ),
            },
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": image_url}},
                    {"type": "text", "text": prompt},
                ],
            },
        ],
        "temperature": args.temperature if temperature is None else temperature,
        "max_tokens": args.max_tokens if max_tokens is None else max_tokens,
    }
    headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}

    started = time.perf_counter()
    last_error: Optional[Exception] = None
    for attempt in range(args.retries + 1):
        try:
            response = requests.post(endpoint, headers=headers, json=payload, timeout=args.timeout)
            if response.status_code >= 400:
                raise RuntimeError(f"HTTP {response.status_code}: {response.text[:800]}")
            data = response.json()
            content = data["choices"][0]["message"]["content"]
            if isinstance(content, list):
                content = "\n".join(str(item.get("text", "")) for item in content if isinstance(item, dict))
            return str(content), time.perf_counter() - started
        except (requests.RequestException, RuntimeError, ValueError, KeyError) as exc:
            last_error = exc
            if attempt < args.retries:
                time.sleep(1.2 * (2 ** attempt))
    raise RuntimeError(f"Qwen API 调用失败：{last_error}") from last_error


def extract_json(text: str) -> Dict[str, Any]:
    cleaned = re.sub(r"^```(?:json)?\s*|\s*```$", "", text.strip(), flags=re.IGNORECASE)
    start = cleaned.find("{")
    if start < 0:
        raise ValueError("响应中没有 JSON 对象。")
    decoder = json.JSONDecoder()
    obj, _ = decoder.raw_decode(cleaned[start:])
    if not isinstance(obj, dict):
        raise ValueError("响应 JSON 不是对象。")
    return obj


def parse_qwen_poses(
    raw: str,
    robot_candidates: Sequence[RobotCandidate],
    case_count: int,
) -> List[Pose]:
    data = extract_json(raw)
    items = data.get("poses")
    if not isinstance(items, list) or len(items) != case_count:
        raise ValueError(f"poses 必须恰好包含 {case_count} 项。")
    lookup = {c.position_id: c for c in robot_candidates}
    used: set[int] = set()
    poses: List[Pose] = []
    for item in items:
        if not isinstance(item, dict):
            raise ValueError("poses 中存在非对象项。")
        position_id = int(item.get("position_id"))
        yaw_deg = float(item.get("yaw_deg"))
        if position_id not in lookup:
            raise ValueError(f"position_id={position_id} 不存在。")
        if position_id in used:
            raise ValueError(f"position_id={position_id} 重复。")
        if not -180.0 <= yaw_deg <= 180.0:
            raise ValueError(f"yaw_deg={yaw_deg} 越界。")
        used.add(position_id)
        c = lookup[position_id]
        poses.append(Pose(c.x, c.y, yaw_deg))
    return poses


def deterministic_fallback(
    pose: Pose,
    candidates: Sequence[FrontierCandidate],
    nav_labels: np.ndarray,
    longest: int,
    args: argparse.Namespace,
    *,
    visited_dist_field: Optional[np.ndarray] = None,
) -> int:
    """Qwen 异常时，在前方扇区内按 CLEAR > BLOCKED 确定性选点。"""
    best_id = 1
    best_key: Optional[Tuple[float, float, float, float]] = None
    for local_id, candidate in enumerate(candidates, start=1):
        m = candidate_metrics(pose, candidate, longest)
        distance = m["distance_ratio"]
        abs_delta = abs(m["heading_delta_deg"])
        if abs_delta > args.front_cone_deg + 1e-9:
            continue
        path_clear = direct_path_is_clear(pose, candidate, nav_labels)
        preferred_penalty = 0.0
        if distance < args.preferred_min_distance_ratio:
            preferred_penalty = args.preferred_min_distance_ratio - distance
        elif distance > args.preferred_max_distance_ratio:
            preferred_penalty = distance - args.preferred_max_distance_ratio

        direction_bucket = 0.0 if path_clear else 1.0
        if visited_dist_field is not None:
            visited_clearance = float(visited_dist_field[candidate.y, candidate.x])
            visited_penalty = -visited_clearance
        else:
            visited_penalty = 0.0
        key = (direction_bucket, preferred_penalty, distance, abs_delta, visited_penalty)
        if best_key is None or key < best_key:
            best_key = key
            best_id = local_id
    return best_id


def format_elapsed(seconds: float) -> str:
    return f"{seconds:7.3f}s"


def print_progress(
    run_started: float,
    current: int,
    total: int,
    message: str,
) -> None:
    elapsed = time.perf_counter() - run_started
    print(f"[{format_elapsed(elapsed)}] 进程 {current}/{total}：{message}", flush=True)


def main() -> int:
    run_started = time.perf_counter()
    args = parse_args()
    if args.cases <= 0:
        print("[错误] --cases 必须大于 0。", file=sys.stderr)
        return 2
    if args.max_candidates < 1:
        print("[错误] --max-candidates 至少为 1。", file=sys.stderr)
        return 2
    if args.min_display_candidates < 1:
        print("[错误] --min-display-candidates 至少为 1。", file=sys.stderr)
        return 2
    if args.max_candidates < args.min_display_candidates:
        print(
            "[错误] --max-candidates 不能小于 --min-display-candidates。",
            file=sys.stderr,
        )
        return 2
    if not (0.0 < args.front_cone_deg <= 90.0):
        print("[错误] --front-cone-deg 必须在 (0, 90] 范围内。", file=sys.stderr)
        return 2
    if not (
        0.0 < args.min_goal_distance_ratio
        <= args.preferred_min_distance_ratio
        <= args.preferred_max_distance_ratio
        <= args.max_goal_distance_ratio
        <= 1.0
    ):
        print(
            "[错误] 距离参数必须满足：0 < min <= preferred_min <= preferred_max <= max <= 1。",
            file=sys.stderr,
        )
        return 2
    if args.near_candidate_slack_ratio < 0.0:
        print("[错误] --near-candidate-slack-ratio 不能小于 0。", file=sys.stderr)
        return 2

    map_path = Path(args.map).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    gray = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        print(f"[错误] 无法读取地图：{map_path}", file=sys.stderr)
        return 2

    h, w = gray.shape
    longest = max(h, w)

    try:
        semantic, free, unknown, blocked, unknown_labels, unknown_areas, unknown_value = build_semantic_masks(gray, args)
        candidates, navigable, nav_labels, frontier_mask, parameters = generate_frontier_candidates(
            free, unknown, blocked, unknown_labels, unknown_areas, longest, args,
            resolution=getattr(args, "map_resolution", None),
        )
        robot_candidates = generate_robot_position_candidates(
            navigable=navigable,
            nav_labels=nav_labels,
            frontier_mask=frontier_mask,
            frontier_candidates=candidates,
            desired_count=args.cases,
            longest=longest,
            args=args,
            seed=args.seed,
        )
    except Exception as exc:
        print(f"[错误] 地图分析失败：{exc}", file=sys.stderr)
        return 2

    total_steps = args.cases + 2
    print_progress(run_started, 1, total_steps, "地图分析与安全前沿生成完成")

    (output_dir / "prompt_templates_zh.txt").write_text(
        "【阶段一：程序随机生成位置，Qwen 分配随机朝向】\n" + POSE_PROMPT_TEMPLATE_ZH +
        "\n\n【阶段二：Qwen 选择灰白前沿目标】\n" + GOAL_PROMPT_TEMPLATE_ZH,
        encoding="utf-8",
    )

    api_key = get_api_key(args.api_key)
    if not args.dry_run and not api_key:
        print(
            "[错误] 未检测到 API Key。PowerShell 中先执行：\n"
            '$env:DASHSCOPE_API_KEY="你的 API Key"',
            file=sys.stderr,
        )
        return 2

    # 确保本次运行结束后，目录中只保留本次新生成的候选图和最终结果图。
    for old_png in output_dir.glob("*.png"):
        try:
            old_png.unlink()
        except OSError as exc:
            print(f"[错误] 无法清理旧图片 {old_png.name}：{exc}", file=sys.stderr)
            return 2

    pose_selection_image = build_pose_selection_image(
        semantic, robot_candidates, args.model_image_side
    )
    pose_prompt = POSE_PROMPT_TEMPLATE_ZH.format(
        case_count=args.cases,
        nonce=random.Random(args.seed).randrange(100000, 999999),
        position_ids=", ".join(str(c.position_id) for c in robot_candidates),
    )
    (output_dir / "pose_selection_prompt.txt").write_text(pose_prompt, encoding="utf-8")

    pose_selection_source = ""
    pose_selection_error = ""
    pose_selection_raw = ""
    pose_selection_latency_s: Optional[float] = None
    if args.dry_run:
        poses = fallback_select_poses(robot_candidates, args.cases, args.seed)
        pose_selection_source = "python_dry_run"
    else:
        try:
            pose_selection_raw, pose_selection_latency_s = call_qwen(
                pose_selection_image, pose_prompt, api_key or "", args,
                temperature=args.pose_temperature, max_tokens=max(280, args.cases * 70),
            )
            (output_dir / "pose_selection_raw.txt").write_text(
                pose_selection_raw, encoding="utf-8"
            )
            poses = parse_qwen_poses(pose_selection_raw, robot_candidates, args.cases)
            pose_selection_source = "qwen"
        except Exception as exc:
            pose_selection_error = str(exc)
            poses = fallback_select_poses(robot_candidates, args.cases, args.seed)
            pose_selection_source = "python_fallback_after_qwen_error"

    if pose_selection_source == "qwen":
        pose_message = f"Qwen 生成 5 个机器人朝向完成（API {pose_selection_latency_s:.3f}s）"
    elif pose_selection_source == "python_dry_run":
        pose_message = "DRY-RUN：Python 生成 5 个机器人朝向完成"
    else:
        pose_message = "Qwen 朝向调用失败，Python 回退生成朝向完成"
    print_progress(run_started, 2, total_steps, pose_message)

    results: List[CaseResult] = []

    for case_id, pose in enumerate(poses, start=1):
        prefix = f"case_{case_id:02d}"
        try:
            case_candidates, selection_meta = choose_case_candidates(
                pose,
                nav_labels,
                candidates,
                longest,
                args,
                navigable=navigable,
                unknown=unknown,
                blocked=blocked,
                resolution=getattr(args, "map_resolution", None),
            )
        except Exception as exc:
            print(f"[错误] case {case_id}: {exc}", file=sys.stderr)
            continue

        input_image, _ = build_case_image(
            semantic, pose, case_candidates, args.model_image_side,
            info_lines=[
                f"Case {case_id:02d} INPUT",
                f"robot=({pose.x},{pose.y}) yaw={pose.yaw_deg:.1f} deg",
                f"safe_frontier_candidates={len(case_candidates)}",
            ],
        )
        candidate_image_path = output_dir / f"{prefix}_candidates.png"
        if not cv2.imwrite(str(candidate_image_path), input_image):
            print(f"[错误] 无法保存候选图：{candidate_image_path}", file=sys.stderr)
            continue

        robot_u = pose.x / max(1, w - 1)
        robot_v = pose.y / max(1, h - 1)
        prompt = GOAL_PROMPT_TEMPLATE_ZH.format(
            robot_u=robot_u,
            robot_v=robot_v,
            yaw_deg=pose.yaw_deg,
            task="",
            min_distance_ratio=args.min_goal_distance_ratio,
            max_distance_ratio=args.max_goal_distance_ratio,
            preferred_min_ratio=args.preferred_min_distance_ratio,
            preferred_max_ratio=args.preferred_max_distance_ratio,
            nearest_distance_ratio=selection_meta["nearest_distance_ratio"],
            candidate_distance_limit_ratio=selection_meta["candidate_distance_limit_ratio"],
            heading_cone_deg=args.front_cone_deg,
            min_display_candidates=args.min_display_candidates,
            candidate_table=candidate_table_text(
                pose, case_candidates, nav_labels, w, h, longest, args
            ),
        )
        (output_dir / f"{prefix}_prompt.txt").write_text(prompt, encoding="utf-8")

        result = CaseResult(
            case_id=case_id,
            robot_x=pose.x,
            robot_y=pose.y,
            robot_u=robot_u,
            robot_v=robot_v,
            yaw_deg=pose.yaw_deg,
            candidate_count=len(case_candidates),
            nearest_candidate_distance_ratio=selection_meta["nearest_distance_ratio"],
            candidate_distance_limit_ratio=selection_meta["candidate_distance_limit_ratio"],
        )

        selected_local_id: int
        if args.dry_run:
            selected_local_id = deterministic_fallback(
                pose, case_candidates, nav_labels, longest, args
            )
            result.selected_by = "python_dry_run"
            result.confidence = 1.0
            result.reason = "未调用 API；在前方扇区内按 CLEAR > BLOCKED 选择。"
        else:
            try:
                raw, latency = call_qwen(input_image, prompt, api_key or "", args)
                result.raw_response = raw
                result.latency_s = round(latency, 3)
                (output_dir / f"{prefix}_raw.txt").write_text(raw, encoding="utf-8")
                parsed = extract_json(raw)
                selected_local_id = int(parsed.get("candidate_id"))
                if not 1 <= selected_local_id <= len(case_candidates):
                    raise ValueError(
                        f"candidate_id={selected_local_id} 不在 1~{len(case_candidates)} 范围内。"
                    )
                result.selected_by = "qwen"
                if parsed.get("confidence") is not None:
                    result.confidence = float(parsed["confidence"])
                result.reason = str(parsed.get("reason", "")).strip()
            except Exception as exc:
                # 只在 Qwen 响应异常时回退；回退点仍来自同一组已严格验证候选。
                selected_local_id = deterministic_fallback(
                    pose, case_candidates, nav_labels, longest, args
                )
                result.selected_by = "python_fallback_after_qwen_error"
                result.error = str(exc)
                result.reason = "Qwen 响应异常，使用同一安全候选集中的确定性最优点。"

        chosen = case_candidates[selected_local_id - 1]
        result.selected_local_id = selected_local_id
        result.selected_global_id = chosen.global_id
        result.goal_x = chosen.x
        result.goal_y = chosen.y
        result.goal_u = chosen.x / max(1, w - 1)
        result.goal_v = chosen.y / max(1, h - 1)
        chosen_metrics = candidate_metrics(pose, chosen, longest)
        result.goal_distance_ratio = chosen_metrics["distance_ratio"]
        result.goal_heading_delta_deg = chosen_metrics["heading_delta_deg"]

        result_image, _ = build_case_image(
            semantic,
            pose,
            case_candidates,
            args.model_image_side,
            selected_local_id=selected_local_id,
            show_all_candidates=False,
            show_candidate_labels=False,
            info_lines=None,
        )
        result_path = output_dir / f"{prefix}_result.png"
        if not cv2.imwrite(str(result_path), result_image):
            print(f"[错误] 无法保存最终结果图：{result_path}", file=sys.stderr)
            continue
        results.append(result)

        if result.selected_by == "qwen":
            case_message = f"Case {case_id:02d} Qwen 选点完成（API {result.latency_s:.3f}s）"
        elif result.selected_by == "python_dry_run":
            case_message = f"Case {case_id:02d} DRY-RUN 选点完成"
        else:
            case_message = f"Case {case_id:02d} Qwen 调用失败，Python 回退选点完成"
        print_progress(run_started, case_id + 2, total_steps, case_message)

    (output_dir / "report.json").write_text(
        json.dumps(
            {
                "map": str(map_path),
                "map_size": [w, h],
                "unknown_value": unknown_value,
                "safety_parameters": parameters,
                "distance_parameters": {
                    "min_goal_distance_ratio": args.min_goal_distance_ratio,
                    "max_goal_distance_ratio": args.max_goal_distance_ratio,
                    "preferred_min_distance_ratio": args.preferred_min_distance_ratio,
                    "preferred_max_distance_ratio": args.preferred_max_distance_ratio,
                    "near_candidate_slack_ratio": args.near_candidate_slack_ratio,
                    "front_cone_deg": args.front_cone_deg,
                },
                "global_candidate_count": len(candidates),
                "generated_robot_position_count": len(robot_candidates),
                "pose_selection_source": pose_selection_source,
                "pose_selection_latency_s": pose_selection_latency_s,
                "pose_selection_error": pose_selection_error,
                "pose_selection_raw": pose_selection_raw,
                "results": [asdict(item) for item in results],
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    total_elapsed = time.perf_counter() - run_started
    if len(results) == args.cases:
        print(f"[{format_elapsed(total_elapsed)}] 最终保存位置：{output_dir}", flush=True)
    else:
        print(
            f"[{format_elapsed(total_elapsed)}] 最终保存位置：{output_dir} "
            f"（成功生成 {len(results)}/{args.cases} 张）",
            flush=True,
        )
    return 0 if len(results) == args.cases else 3


if __name__ == "__main__":
    raise SystemExit(main())
