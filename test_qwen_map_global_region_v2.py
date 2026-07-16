#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Qwen 地图单图全局探索区域测试脚本。

用途：
1. 读取一张地图图片；
2. 按手动给定的归一化位置和朝向，在地图上标记假定机器人初始位姿；
3. 将标注后的地图发送给 Qwen-VL；
4. 要求 Qwen 只依据地图提出下一步探索区域和一个测试标记点；
5. 校验 JSON，并将 Qwen 结果画回地图。

安全边界：
- 不使用 ROS；
- 不发布 /cmd_vel；
- 不调用 Nav2；
- 输出点只是图像上的测试标记，不是导航目标；
- 不验证真实可达性、footprint、clearance 或路径。

坐标约定：
- u=0 为地图图片最左侧，u=1 为最右侧；
- v=0 为地图图片最上侧，v=1 为最下侧；
- yaw_deg=0 指向图片右侧；
- yaw_deg=90 指向图片上侧；
- yaw_deg=-90 指向图片下侧；
- yaw_deg=180/-180 指向图片左侧。
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import re
import sys
import textwrap
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import requests
except ImportError as exc:
    raise SystemExit("缺少 requests。请先安装：python3 -m pip install requests") from exc

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:
    raise SystemExit("缺少 Pillow。请先安装：python3 -m pip install pillow") from exc


ALLOWED_RISK_FLAGS = {
    "NOT_GEOMETRICALLY_VALIDATED",
    "PATH_NOT_CHECKED",
    "REACHABILITY_UNKNOWN",
    "MAP_IMAGE_ONLY",
}

FORBIDDEN_KEYS = {
    "map_x",
    "map_y",
    "grid_x",
    "grid_y",
    "goal_pose",
    "navigation_goal",
    "cmd_vel",
    "speed",
    "velocity",
    "linear",
    "angular",
    "path_checked",
    "reachable",
}


@dataclass(frozen=True)
class RobotPoseImage:
    u: float
    v: float
    yaw_deg: float


@dataclass(frozen=True)
class RegionProposal:
    proposal_id: str
    center_u: float
    center_v: float
    bbox_u_min: float
    bbox_v_min: float
    bbox_u_max: float
    bbox_v_max: float
    test_point_u: float
    test_point_v: float
    confidence: float
    reason_code: str
    map_evidence: Tuple[str, ...]
    risk_flags: Tuple[str, ...]
    center_distance_norm: float
    test_point_distance_norm: float


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def ensure_finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是数字")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数字")
    return result


def ensure_unit_interval(value: Any, name: str) -> float:
    result = ensure_finite_number(value, name)
    if not 0.0 <= result <= 1.0:
        raise ValueError(f"{name} 必须位于 [0, 1]，实际为 {result}")
    return result


def load_font(size: int) -> ImageFont.ImageFont:
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for candidate in candidates:
        if Path(candidate).exists():
            try:
                return ImageFont.truetype(candidate, size=size)
            except OSError:
                pass
    return ImageFont.load_default()


def normalized_to_pixel(u: float, v: float, width: int, height: int) -> Tuple[int, int]:
    x = int(round(clamp01(u) * max(0, width - 1)))
    y = int(round(clamp01(v) * max(0, height - 1)))
    return x, y


def normalized_image_distance(
    u1: float,
    v1: float,
    u2: float,
    v2: float,
    width: int,
    height: int,
) -> float:
    """返回两点像素距离占整张图像对角线的比例。"""
    dx_px = (u2 - u1) * max(1, width - 1)
    dy_px = (v2 - v1) * max(1, height - 1)
    diagonal = math.hypot(max(1, width - 1), max(1, height - 1))
    return math.hypot(dx_px, dy_px) / diagonal


def _draw_text_box(
    draw: ImageDraw.ImageDraw,
    xy: Tuple[int, int],
    text: str,
    font: ImageFont.ImageFont,
    image_size: Tuple[int, int],
    *,
    fill: Tuple[int, int, int, int],
    background: Tuple[int, int, int, int] = (255, 255, 255, 210),
    padding: int = 2,
) -> None:
    width, height = image_size
    bbox = draw.textbbox((0, 0), text, font=font)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]
    x = min(max(padding, xy[0]), max(padding, width - tw - padding * 2))
    y = min(max(padding, xy[1]), max(padding, height - th - padding * 2))
    draw.rectangle(
        [x - padding, y - padding, x + tw + padding, y + th + padding],
        fill=background,
    )
    draw.text((x, y), text, fill=fill, font=font)


def draw_reference_grid(
    image: Image.Image,
    divisions: int = 10,
    label_every: int = 2,
) -> None:
    draw = ImageDraw.Draw(image, "RGBA")
    width, height = image.size
    font = load_font(max(9, min(12, min(width, height) // 55)))

    for i in range(divisions + 1):
        u = i / divisions
        x = int(round(u * (width - 1)))
        draw.line([(x, 0), (x, height - 1)], fill=(30, 80, 180, 58), width=1)
        if i % label_every == 0:
            _draw_text_box(
                draw,
                (x + 2, 2),
                f"{u:.1f}",
                font,
                image.size,
                fill=(20, 50, 130, 220),
                background=(255, 255, 255, 175),
                padding=1,
            )

    for i in range(divisions + 1):
        v = i / divisions
        y = int(round(v * (height - 1)))
        draw.line([(0, y), (width - 1, y)], fill=(30, 80, 180, 58), width=1)
        if i % label_every == 0:
            _draw_text_box(
                draw,
                (2, y + 2),
                f"{v:.1f}",
                font,
                image.size,
                fill=(20, 50, 130, 220),
                background=(255, 255, 255, 175),
                padding=1,
            )


def draw_robot_pose(image: Image.Image, pose: RobotPoseImage) -> None:
    draw = ImageDraw.Draw(image, "RGBA")
    width, height = image.size
    x, y = normalized_to_pixel(pose.u, pose.v, width, height)

    radius = max(7, min(width, height) // 65)
    arrow_len = max(28, min(width, height) // 8)
    arrow_width = max(2, min(width, height) // 190)

    angle_rad = math.radians(pose.yaw_deg)
    dx = math.cos(angle_rad) * arrow_len
    dy = -math.sin(angle_rad) * arrow_len
    end_x = x + dx
    end_y = y + dy

    draw.ellipse(
        [x - radius, y - radius, x + radius, y + radius],
        fill=(0, 120, 255, 235),
        outline=(255, 255, 255, 255),
        width=max(2, arrow_width),
    )
    draw.line([(x, y), (end_x, end_y)], fill=(0, 90, 230, 255), width=arrow_width)

    head_len = max(9, arrow_len * 0.27)
    for offset_deg in (150, -150):
        head_angle = angle_rad + math.radians(offset_deg)
        hx = end_x + math.cos(head_angle) * head_len
        hy = end_y - math.sin(head_angle) * head_len
        draw.line([(end_x, end_y), (hx, hy)], fill=(0, 90, 230, 255), width=arrow_width)

    font = load_font(max(10, min(14, min(width, height) // 45)))
    label_x = x + radius + 5 if x < width * 0.72 else x - radius - 58
    label_y = y - radius - 18 if y > 28 else y + radius + 5
    _draw_text_box(
        draw,
        (int(label_x), int(label_y)),
        "ROBOT",
        font,
        image.size,
        fill=(0, 60, 160, 255),
        background=(255, 255, 255, 210),
        padding=2,
    )


def create_qwen_input_image(
    map_path: Path,
    output_path: Path,
    pose: RobotPoseImage,
    grid_divisions: int,
    grid_label_every: int,
) -> None:
    with Image.open(map_path) as source:
        image = source.convert("RGB")
    draw_reference_grid(image, divisions=grid_divisions, label_every=grid_label_every)
    draw_robot_pose(image, pose)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path, quality=95)


def image_to_data_url(image_path: Path) -> str:
    suffix = image_path.suffix.lower()
    mime = "image/png" if suffix == ".png" else "image/jpeg"
    encoded = base64.b64encode(image_path.read_bytes()).decode("ascii")
    return f"data:{mime};base64,{encoded}"


def build_prompt(
    task: str,
    pose: RobotPoseImage,
    preferred_min_distance: float,
    preferred_max_distance: float,
    hard_max_distance: float,
    hard_max_test_distance: float,
) -> str:
    return f"""
You are the high-level semantic exploration planner for a mobile robot.

This is a MAP-ONLY TEST. You receive one occupancy-map image that already contains:
- a blue ROBOT marker,
- a blue heading arrow,
- a normalized 10x10 U/V reference grid.

Robot pose supplied by the test script:
- robot_u = {pose.u:.6f}
- robot_v = {pose.v:.6f}
- robot_yaw_deg = {pose.yaw_deg:.3f}

Target instruction:
{task}

Your task:
Inspect the complete map and propose exactly ONE next exploration REGION.
Also provide ONE test marker point inside that region. The test point is only
for drawing a marker on the image; it is NOT a robot waypoint or Nav2 goal.

MAP INTERPRETATION:
- In a conventional occupancy map, white or very light areas usually represent known FREE space.
- Black or very dark areas usually represent OCCUPIED walls or obstacles.
- Gray areas usually represent UNKNOWN, unmapped space.
- If the supplied map contains a legend, follow that legend instead.
- ROBOT is the assumed current position.
- The arrow is the assumed current heading.

NORMALIZED COORDINATES:
- u=0.0 is the left edge of the map image.
- u=1.0 is the right edge.
- v=0.0 is the top edge.
- v=1.0 is the bottom edge.
- All coordinates refer to the entire supplied map image.

SELECT A GOOD EXPLORATION REGION:
1. Prefer a meaningful boundary or continuation between known FREE space and UNKNOWN space.
2. Prefer a doorway, corridor continuation, room opening, branch, or broad unexplored boundary that could reveal useful new space.
3. Prefer a SHORT, incremental exploration step rather than jumping to the farthest frontier.
4. Distance is measured as a fraction of the full image diagonal. Prefer a region center approximately between {preferred_min_distance:.3f} and {preferred_max_distance:.3f} from the robot. The region center must not be farther than {hard_max_distance:.3f}, and the test point must not be farther than {hard_max_test_distance:.3f}.
5. Among similarly useful frontiers, choose the nearest one that extends the map.
6. Consider the robot's current heading and avoid unnecessary reversal when exploration value is otherwise similar.
5. Avoid walls, occupied blocks, map padding, legends, text labels, tiny noise, isolated unknown pixels, and unknown areas fully separated from known free space.
6. If a central wall or obstacle blocks the direct direction, inspect both sides and choose a real free-to-unknown opening rather than the obstacle surface.
7. Do not select a region solely because it is a large gray area. It must appear connected to known FREE space through a plausible frontier.
8. The suggested test point should lie near the FREE side of the selected exploration boundary, not deep inside UNKNOWN and not on a wall.
9. Do not claim reachability or path validity. Geometry and Nav2 have not checked this result.
10. If the map does not contain a defensible exploration region, set selection_status to \"NO_VALID_REGION\" and use null for region and test_point.

OUTPUT:
Return ONLY one valid JSON object. No markdown and no extra text.

Use exactly this shape:
{{
  \"selection_status\": \"REGION_PROPOSED\",
  \"selection_strategy\": \"MAP_ONLY_GLOBAL_REGION_TEST\",
  \"robot_pose_ack\": {{
    \"u\": {pose.u:.6f},
    \"v\": {pose.v:.6f},
    \"yaw_deg\": {pose.yaw_deg:.3f}
  }},
  \"selected_region\": {{
    \"proposal_id\": \"GP_1\",
    \"center\": {{\"u\": 0.0, \"v\": 0.0}},
    \"bbox\": {{
      \"u_min\": 0.0,
      \"v_min\": 0.0,
      \"u_max\": 0.0,
      \"v_max\": 0.0
    }},
    \"direction_hint\": \"FRONT|RIGHT_FRONT|RIGHT|RIGHT_BACK|BACK|LEFT_BACK|LEFT|LEFT_FRONT\",
    \"confidence\": 0.0,
    \"reason_code\": \"MEANINGFUL_FREE_UNKNOWN_BOUNDARY\",
    \"map_evidence\": [\"brief evidence 1\", \"brief evidence 2\"],
    \"risk_flags\": [
      \"NOT_GEOMETRICALLY_VALIDATED\",
      \"PATH_NOT_CHECKED\",
      \"REACHABILITY_UNKNOWN\",
      \"MAP_IMAGE_ONLY\"
    ]
  }},
  \"test_point\": {{
    \"u\": 0.0,
    \"v\": 0.0,
    \"purpose\": \"VISUAL_TEST_MARKER_ONLY\"
  }}
}}

For a valid proposal:
- every u/v must be in [0,1];
- u_min < center.u < u_max;
- v_min < center.v < v_max;
- the test point must be inside the bounding box;
- confidence must be in [0,1].

For no valid region, return:
{{
  \"selection_status\": \"NO_VALID_REGION\",
  \"selection_strategy\": \"MAP_ONLY_GLOBAL_REGION_TEST\",
  \"robot_pose_ack\": {{
    \"u\": {pose.u:.6f},
    \"v\": {pose.v:.6f},
    \"yaw_deg\": {pose.yaw_deg:.3f}
  }},
  \"selected_region\": null,
  \"test_point\": null,
  \"reason\": \"brief reason\"
}}

FORBIDDEN:
- pixel coordinates;
- map x/y coordinates;
- grid coordinates;
- speed, velocity, turn angle, drive distance;
- /cmd_vel;
- goal_pose or navigation_goal;
- path_checked;
- reachable;
- statements claiming the point is safe or reachable.
""".strip()


def strip_json_fences(text: str) -> str:
    cleaned = text.strip()
    fence = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", cleaned, flags=re.IGNORECASE | re.DOTALL)
    return fence.group(1).strip() if fence else cleaned


def parse_response_json(raw_text: str) -> Dict[str, Any]:
    cleaned = strip_json_fences(raw_text)
    try:
        parsed = json.loads(cleaned)
    except json.JSONDecodeError:
        first = cleaned.find("{")
        last = cleaned.rfind("}")
        if first < 0 or last <= first:
            raise ValueError("Qwen响应中未找到JSON对象")
        parsed = json.loads(cleaned[first:last + 1])
    if not isinstance(parsed, dict):
        raise ValueError("Qwen响应顶层必须是JSON对象")
    return parsed


def recursively_find_forbidden_keys(value: Any, path: str = "$") -> List[str]:
    findings: List[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            key_lower = str(key).lower()
            if key_lower in FORBIDDEN_KEYS:
                findings.append(f"{path}.{key}")
            findings.extend(recursively_find_forbidden_keys(child, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            findings.extend(recursively_find_forbidden_keys(child, f"{path}[{index}]"))
    return findings


def validate_qwen_result(
    payload: Dict[str, Any],
    pose: RobotPoseImage,
    image_size: Tuple[int, int],
    hard_max_distance: float,
    hard_max_test_distance: float,
) -> Optional[RegionProposal]:
    forbidden = recursively_find_forbidden_keys(payload)
    if forbidden:
        raise ValueError(f"响应包含禁止字段：{', '.join(forbidden)}")

    if payload.get("selection_strategy") != "MAP_ONLY_GLOBAL_REGION_TEST":
        raise ValueError("selection_strategy不正确")

    status = payload.get("selection_status")
    if status == "NO_VALID_REGION":
        if payload.get("selected_region") is not None:
            raise ValueError("NO_VALID_REGION时selected_region必须为null")
        if payload.get("test_point") is not None:
            raise ValueError("NO_VALID_REGION时test_point必须为null")
        return None

    if status != "REGION_PROPOSED":
        raise ValueError(f"未知selection_status：{status!r}")

    robot_ack = payload.get("robot_pose_ack")
    if not isinstance(robot_ack, dict):
        raise ValueError("缺少robot_pose_ack")
    ack_u = ensure_unit_interval(robot_ack.get("u"), "robot_pose_ack.u")
    ack_v = ensure_unit_interval(robot_ack.get("v"), "robot_pose_ack.v")
    ack_yaw = ensure_finite_number(robot_ack.get("yaw_deg"), "robot_pose_ack.yaw_deg")
    if abs(ack_u - pose.u) > 1e-3 or abs(ack_v - pose.v) > 1e-3:
        raise ValueError("Qwen没有原样确认机器人位置")
    if abs(ack_yaw - pose.yaw_deg) > 1e-2:
        raise ValueError("Qwen没有原样确认机器人朝向")

    selected = payload.get("selected_region")
    point = payload.get("test_point")
    if not isinstance(selected, dict) or not isinstance(point, dict):
        raise ValueError("REGION_PROPOSED时必须提供selected_region和test_point")
    if selected.get("proposal_id") != "GP_1":
        raise ValueError("proposal_id必须为GP_1")

    center = selected.get("center")
    bbox = selected.get("bbox")
    if not isinstance(center, dict) or not isinstance(bbox, dict):
        raise ValueError("selected_region.center/bbox格式错误")

    center_u = ensure_unit_interval(center.get("u"), "center.u")
    center_v = ensure_unit_interval(center.get("v"), "center.v")
    u_min = ensure_unit_interval(bbox.get("u_min"), "bbox.u_min")
    v_min = ensure_unit_interval(bbox.get("v_min"), "bbox.v_min")
    u_max = ensure_unit_interval(bbox.get("u_max"), "bbox.u_max")
    v_max = ensure_unit_interval(bbox.get("v_max"), "bbox.v_max")
    if not u_min < center_u < u_max:
        raise ValueError("center.u必须严格位于bbox横向范围内")
    if not v_min < center_v < v_max:
        raise ValueError("center.v必须严格位于bbox纵向范围内")

    test_u = ensure_unit_interval(point.get("u"), "test_point.u")
    test_v = ensure_unit_interval(point.get("v"), "test_point.v")
    if not u_min <= test_u <= u_max or not v_min <= test_v <= v_max:
        raise ValueError("test_point必须位于selected_region.bbox内部")
    if point.get("purpose") != "VISUAL_TEST_MARKER_ONLY":
        raise ValueError("test_point.purpose不正确")

    confidence = ensure_unit_interval(selected.get("confidence"), "selected_region.confidence")
    reason_code = selected.get("reason_code")
    if not isinstance(reason_code, str) or not reason_code.strip():
        raise ValueError("reason_code必须是非空字符串")

    evidence = selected.get("map_evidence")
    if not isinstance(evidence, list) or not evidence or not all(isinstance(item, str) and item.strip() for item in evidence):
        raise ValueError("map_evidence必须是非空字符串列表")

    risk_flags = selected.get("risk_flags")
    if not isinstance(risk_flags, list):
        raise ValueError("risk_flags必须是列表")
    unknown_flags = [flag for flag in risk_flags if flag not in ALLOWED_RISK_FLAGS]
    if unknown_flags:
        raise ValueError(f"存在未知risk_flags：{unknown_flags}")

    width, height = image_size
    center_distance_norm = normalized_image_distance(
        pose.u, pose.v, center_u, center_v, width, height
    )
    test_point_distance_norm = normalized_image_distance(
        pose.u, pose.v, test_u, test_v, width, height
    )
    if center_distance_norm > hard_max_distance:
        raise ValueError(
            "Qwen选择的区域中心过远："
            f"distance={center_distance_norm:.3f} > hard_max={hard_max_distance:.3f}"
        )
    if test_point_distance_norm > hard_max_test_distance:
        raise ValueError(
            "Qwen选择的测试点过远："
            f"distance={test_point_distance_norm:.3f} > "
            f"hard_max={hard_max_test_distance:.3f}"
        )

    return RegionProposal(
        proposal_id="GP_1",
        center_u=center_u,
        center_v=center_v,
        bbox_u_min=u_min,
        bbox_v_min=v_min,
        bbox_u_max=u_max,
        bbox_v_max=v_max,
        test_point_u=test_u,
        test_point_v=test_v,
        confidence=confidence,
        reason_code=reason_code.strip(),
        map_evidence=tuple(item.strip() for item in evidence),
        risk_flags=tuple(risk_flags),
        center_distance_norm=center_distance_norm,
        test_point_distance_norm=test_point_distance_norm,
    )


def call_qwen(
    image_path: Path,
    prompt: str,
    api_key: str,
    base_url: str,
    model: str,
    timeout_s: float,
    temperature: float,
) -> str:
    endpoint = base_url.rstrip("/") + "/chat/completions"
    payload = {
        "model": model,
        "messages": [
            {
                "role": "system",
                "content": "You are a precise mobile-robot map reasoning assistant. Follow the JSON schema and safety constraints exactly.",
            },
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": image_to_data_url(image_path)}},
                    {"type": "text", "text": prompt},
                ],
            },
        ],
        "temperature": temperature,
        "max_tokens": 1800,
    }

    response = requests.post(
        endpoint,
        headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
        json=payload,
        timeout=timeout_s,
    )
    if not response.ok:
        raise RuntimeError(f"Qwen API调用失败：HTTP {response.status_code}\n{response.text[:4000]}")

    data = response.json()
    try:
        content = data["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as exc:
        raise RuntimeError(
            "Qwen API响应缺少 choices[0].message.content：\n"
            + json.dumps(data, ensure_ascii=False, indent=2)[:4000]
        ) from exc

    if isinstance(content, str):
        return content
    if isinstance(content, list):
        text_parts: List[str] = []
        for part in content:
            if isinstance(part, dict) and isinstance(part.get("text"), str):
                text_parts.append(part["text"])
        if text_parts:
            return "\n".join(text_parts)
    raise RuntimeError(f"无法解析Qwen消息内容：{content!r}")


def draw_result(
    input_image_path: Path,
    output_path: Path,
    proposal: Optional[RegionProposal],
    no_region_reason: Optional[str],
) -> None:
    with Image.open(input_image_path) as source:
        map_image = source.convert("RGB")

    width, map_height = map_image.size
    font = load_font(max(11, min(15, min(width, map_height) // 42)))
    small_font = load_font(max(9, min(12, min(width, map_height) // 52)))
    panel_height = max(120, int(map_height * 0.25))

    canvas = Image.new("RGB", (width, map_height + panel_height), "white")
    canvas.paste(map_image, (0, 0))
    draw = ImageDraw.Draw(canvas, "RGBA")

    if proposal is None:
        draw.rectangle(
            [0, map_height, width - 1, map_height + panel_height - 1],
            fill=(255, 250, 250, 255),
            outline=(180, 0, 0, 255),
            width=2,
        )
        reason = no_region_reason or "No defensible exploration region was returned."
        lines = ["QWEN: NO VALID REGION"] + textwrap.wrap(reason, width=max(35, width // 8))
        for index, line in enumerate(lines[:5]):
            draw.text(
                (10, map_height + 10 + index * 22),
                line,
                fill=(180, 0, 0, 255),
                font=font if index == 0 else small_font,
            )
    else:
        x0, y0 = normalized_to_pixel(
            proposal.bbox_u_min, proposal.bbox_v_min, width, map_height
        )
        x1, y1 = normalized_to_pixel(
            proposal.bbox_u_max, proposal.bbox_v_max, width, map_height
        )
        cx, cy = normalized_to_pixel(
            proposal.center_u, proposal.center_v, width, map_height
        )
        tx, ty = normalized_to_pixel(
            proposal.test_point_u, proposal.test_point_v, width, map_height
        )

        box_width = max(2, min(width, map_height) // 180)
        draw.rectangle(
            [x0, y0, x1, y1],
            outline=(255, 120, 0, 255),
            fill=(255, 150, 0, 28),
            width=box_width,
        )

        center_radius = max(5, min(width, map_height) // 95)
        draw.ellipse(
            [cx - center_radius, cy - center_radius, cx + center_radius, cy + center_radius],
            fill=(255, 140, 0, 230),
            outline=(255, 255, 255, 255),
            width=2,
        )

        point_radius = max(6, min(width, map_height) // 80)
        draw.line(
            [(tx - point_radius, ty), (tx + point_radius, ty)],
            fill=(220, 0, 60, 255),
            width=2,
        )
        draw.line(
            [(tx, ty - point_radius), (tx, ty + point_radius)],
            fill=(220, 0, 60, 255),
            width=2,
        )
        draw.ellipse(
            [tx - point_radius, ty - point_radius, tx + point_radius, ty + point_radius],
            outline=(220, 0, 60, 255),
            width=2,
        )

        _draw_text_box(
            draw,
            (x0 + 3, max(3, y0 - 18)),
            "REGION GP_1",
            small_font,
            (width, map_height),
            fill=(150, 60, 0, 255),
            background=(255, 255, 255, 215),
            padding=2,
        )
        _draw_text_box(
            draw,
            (tx + 7, ty + 5),
            "TEST",
            small_font,
            (width, map_height),
            fill=(180, 0, 40, 255),
            background=(255, 255, 255, 215),
            padding=2,
        )

        draw.rectangle(
            [0, map_height, width - 1, map_height + panel_height - 1],
            fill=(255, 252, 247, 255),
            outline=(255, 120, 0, 255),
            width=2,
        )

        summary = [
            f"QWEN REGION GP_1   confidence={proposal.confidence:.2f}",
            (
                f"center=({proposal.center_u:.3f}, {proposal.center_v:.3f})  "
                f"test=({proposal.test_point_u:.3f}, {proposal.test_point_v:.3f})"
            ),
            (
                f"distance: center={proposal.center_distance_norm:.3f}, "
                f"test={proposal.test_point_distance_norm:.3f} "
                "(fraction of image diagonal)"
            ),
            f"reason={proposal.reason_code}",
        ]
        evidence_text = " | ".join(proposal.map_evidence)
        wrapped_evidence = textwrap.wrap(evidence_text, width=max(42, width // 7))[:2]
        lines = summary + [f"evidence: {line}" for line in wrapped_evidence]
        lines.append("VISUAL TEST ONLY — NOT A NAVIGATION GOAL")

        y = map_height + 8
        for index, line in enumerate(lines):
            draw.text(
                (10, y),
                line,
                fill=(180, 0, 40, 255) if index == len(lines) - 1 else (110, 55, 15, 255),
                font=font if index == 0 else small_font,
            )
            y += 20 if index else 23
            if y > map_height + panel_height - 18:
                break

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output_path, quality=95)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="让Qwen只根据地图图片提出下一步探索区域并画回地图。")
    parser.add_argument("--map", required=True, help="输入地图PNG/JPG路径")
    parser.add_argument("--robot-u", type=float, required=True, help="假定初始u，[0,1]")
    parser.add_argument("--robot-v", type=float, required=True, help="假定初始v，[0,1]")
    parser.add_argument("--robot-yaw-deg", type=float, required=True, help="假定朝向：0右、90上、-90下、180左")
    parser.add_argument("--task", default="优先探索尚未覆盖、最可能扩展地图的区域。", help="给Qwen的任务说明")
    parser.add_argument("--output-dir", default="logs/qwen_map_only_test", help="输出根目录")
    parser.add_argument("--model", default=os.getenv("QWEN_MODEL", "qwen3-vl-flash"), help="模型名称")
    parser.add_argument(
        "--base-url",
        default=os.getenv("QWEN_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"),
        help="OpenAI兼容base URL",
    )
    parser.add_argument("--api-key-env", default="DASHSCOPE_API_KEY", help="API Key环境变量名称")
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--temperature", type=float, default=0.1)
    parser.add_argument("--grid-divisions", type=int, default=10)
    parser.add_argument(
        "--grid-label-every",
        type=int,
        default=2,
        help="每隔多少条网格线显示一次数字，默认2，即0.0/0.2/…/1.0",
    )
    parser.add_argument(
        "--preferred-min-distance",
        type=float,
        default=0.08,
        help="偏好的最小区域距离，占图像对角线比例",
    )
    parser.add_argument(
        "--preferred-max-distance",
        type=float,
        default=0.28,
        help="偏好的最大区域距离，占图像对角线比例",
    )
    parser.add_argument(
        "--hard-max-distance",
        type=float,
        default=0.40,
        help="区域中心硬最大距离，占图像对角线比例",
    )
    parser.add_argument(
        "--hard-max-test-distance",
        type=float,
        default=0.36,
        help="测试点硬最大距离，占图像对角线比例",
    )
    parser.add_argument("--dry-run", action="store_true", help="只生成Qwen输入图和prompt，不调用API")
    parser.add_argument("--mock-response", help="读取本地模拟Qwen JSON/文本，跳过API调用")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    map_path = Path(args.map).expanduser().resolve()
    if not map_path.is_file():
        raise SystemExit(f"地图文件不存在：{map_path}")

    pose = RobotPoseImage(
        u=ensure_unit_interval(args.robot_u, "robot_u"),
        v=ensure_unit_interval(args.robot_v, "robot_v"),
        yaw_deg=ensure_finite_number(args.robot_yaw_deg, "robot_yaw_deg"),
    )
    if args.grid_divisions < 2 or args.grid_divisions > 20:
        raise SystemExit("--grid-divisions建议位于2到20")
    if args.grid_label_every < 1 or args.grid_label_every > args.grid_divisions:
        raise SystemExit("--grid-label-every必须位于1到grid-divisions之间")
    distance_values = [
        args.preferred_min_distance,
        args.preferred_max_distance,
        args.hard_max_distance,
        args.hard_max_test_distance,
    ]
    if any((not math.isfinite(v)) or v < 0.0 or v > 1.0 for v in distance_values):
        raise SystemExit("所有距离参数必须位于[0,1]")
    if not args.preferred_min_distance <= args.preferred_max_distance <= args.hard_max_distance:
        raise SystemExit(
            "距离关系必须满足 preferred-min <= preferred-max <= hard-max-distance"
        )

    with Image.open(map_path) as source_image:
        image_size = source_image.size

    run_id = datetime.now(timezone.utc).strftime("MAPQ_%Y%m%dT%H%M%SZ")
    output_dir = Path(args.output_dir).expanduser().resolve() / run_id
    output_dir.mkdir(parents=True, exist_ok=False)

    input_image_path = output_dir / "map_input_for_qwen.png"
    prompt_path = output_dir / "prompt.txt"
    raw_response_path = output_dir / "qwen_raw_response.txt"
    parsed_response_path = output_dir / "qwen_parsed_response.json"
    result_image_path = output_dir / "qwen_selected_region.png"
    run_meta_path = output_dir / "run_meta.json"

    create_qwen_input_image(
        map_path,
        input_image_path,
        pose,
        args.grid_divisions,
        args.grid_label_every,
    )
    prompt = build_prompt(
        args.task,
        pose,
        args.preferred_min_distance,
        args.preferred_max_distance,
        args.hard_max_distance,
        args.hard_max_test_distance,
    )
    prompt_path.write_text(prompt, encoding="utf-8")

    run_meta = {
        "run_id": run_id,
        "input_map": str(map_path),
        "input_for_qwen": str(input_image_path),
        "model": args.model,
        "base_url": args.base_url,
        "robot_pose": {"u": pose.u, "v": pose.v, "yaw_deg": pose.yaw_deg},
        "task": args.task,
        "distance_policy": {
            "metric": "fraction_of_image_diagonal",
            "preferred_min": args.preferred_min_distance,
            "preferred_max": args.preferred_max_distance,
            "hard_max_region_center": args.hard_max_distance,
            "hard_max_test_point": args.hard_max_test_distance,
        },
        "dry_run": bool(args.dry_run),
        "mock_response": args.mock_response,
        "safety": {
            "ros_used": False,
            "cmd_vel_published": False,
            "nav2_invoked": False,
            "robot_moved": False,
            "result_is_navigation_goal": False,
        },
    }
    run_meta_path.write_text(json.dumps(run_meta, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[INFO] 输出目录：{output_dir}")
    print(f"[INFO] Qwen输入图：{input_image_path}")
    print(f"[INFO] 提示词：{prompt_path}")

    if args.dry_run:
        print("[PASS] dry-run完成：未调用Qwen API")
        return 0

    if args.mock_response:
        raw_text = Path(args.mock_response).expanduser().read_text(encoding="utf-8")
    else:
        api_key = os.getenv(args.api_key_env, "").strip()
        if not api_key:
            raise SystemExit(f"未找到API Key。请先设置：export {args.api_key_env}='你的Key'")
        raw_text = call_qwen(
            image_path=input_image_path,
            prompt=prompt,
            api_key=api_key,
            base_url=args.base_url,
            model=args.model,
            timeout_s=args.timeout_s,
            temperature=args.temperature,
        )

    raw_response_path.write_text(raw_text, encoding="utf-8")
    parsed = parse_response_json(raw_text)
    proposal = validate_qwen_result(
        parsed,
        pose,
        image_size,
        args.hard_max_distance,
        args.hard_max_test_distance,
    )
    parsed_response_path.write_text(json.dumps(parsed, ensure_ascii=False, indent=2), encoding="utf-8")

    no_region_reason = parsed.get("reason") if proposal is None and isinstance(parsed.get("reason"), str) else None
    draw_result(input_image_path, result_image_path, proposal, no_region_reason)

    if proposal is None:
        print("[RESULT] Qwen未找到可信区域")
    else:
        print("[RESULT] Qwen提出探索区域")
        print(f"  center=({proposal.center_u:.3f}, {proposal.center_v:.3f})")
        print(f"  test_point=({proposal.test_point_u:.3f}, {proposal.test_point_v:.3f})")
        print(f"  confidence={proposal.confidence:.3f}")
        print(f"  reason_code={proposal.reason_code}")
        print(f"  center_distance_norm={proposal.center_distance_norm:.3f}")
        print(f"  test_point_distance_norm={proposal.test_point_distance_norm:.3f}")

    print(f"[PASS] 结果图：{result_image_path}")
    print(f"[PASS] 解析JSON：{parsed_response_path}")
    print("[SAFETY] 该点仅为图像测试标记，不是导航目标。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[CANCELED] 用户中断", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"[FAIL] {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1)
