#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从已保存地图 + 轨迹 + 位姿，合成带「已通过路径」和大号朝向箭头的标注图。

供 run_joy_mapping_qwen_plan_session.sh 调用；不启动 ROS、不发布 /cmd_vel。
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Tuple

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:
    raise SystemExit("缺少 Pillow。请先安装：python3 -m pip install pillow") from exc


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


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


def load_map_yaml(yaml_path: Path) -> Dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise SystemExit("缺少 PyYAML。请先安装：python3 -m pip install pyyaml") from exc

    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"无效的 map yaml: {yaml_path}")
    return data


def world_to_pixel(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> Tuple[int, int]:
    col = (x - origin_x) / resolution
    row = (y - origin_y) / resolution
    px = int(round(col))
    py = int(round(height - 1 - row))
    return px, py


def world_to_uv(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> Tuple[float, float]:
    col = (x - origin_x) / resolution
    row = (y - origin_y) / resolution
    u = col / max(1, width - 1)
    img_row = height - 1 - row
    v = img_row / max(1, height - 1)
    return clamp01(u), clamp01(v)


def extract_path_points(trajectory: Dict[str, Any]) -> List[Tuple[float, float]]:
    vertices = trajectory.get("vertices") or []
    if isinstance(vertices, list) and len(vertices) >= 2:
        points: List[Tuple[float, float]] = []
        for item in vertices:
            if isinstance(item, dict):
                points.append((float(item.get("x", 0.0)), float(item.get("y", 0.0))))
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                points.append((float(item[0]), float(item[1])))
        if len(points) >= 2:
            return points

    samples = trajectory.get("raw_samples") or []
    points: List[Tuple[float, float]] = []
    last_xy: Tuple[float, float] | None = None
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        if sample.get("valid") is False:
            continue
        x = float(sample.get("x", 0.0))
        y = float(sample.get("y", 0.0))
        xy = (x, y)
        if last_xy is not None and abs(x - last_xy[0]) < 1e-4 and abs(y - last_xy[1]) < 1e-4:
            continue
        points.append(xy)
        last_xy = xy

    if len(points) >= 2:
        return points

    for sample in samples:
        if not isinstance(sample, dict):
            continue
        x = float(sample.get("x", 0.0))
        y = float(sample.get("y", 0.0))
        xy = (x, y)
        if points and abs(x - points[-1][0]) < 1e-4 and abs(y - points[-1][1]) < 1e-4:
            continue
        points.append(xy)
    return points


def resolve_robot_pose(
    pose_json: Dict[str, Any] | None,
    trajectory: Dict[str, Any] | None,
) -> Tuple[float, float, float]:
    if pose_json:
        x = float(pose_json.get("x", 0.0))
        y = float(pose_json.get("y", 0.0))
        if "yaw" in pose_json:
            yaw = float(pose_json["yaw"])
        else:
            qx = float(pose_json.get("qx", 0.0))
            qy = float(pose_json.get("qy", 0.0))
            qz = float(pose_json.get("qz", 0.0))
            qw = float(pose_json.get("qw", 1.0))
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw

    if trajectory:
        samples = trajectory.get("raw_samples") or []
        for sample in reversed(samples):
            if not isinstance(sample, dict):
                continue
            if sample.get("valid") is False:
                continue
            return (
                float(sample.get("x", 0.0)),
                float(sample.get("y", 0.0)),
                float(sample.get("yaw_rad", 0.0)),
            )
        vertices = trajectory.get("vertices") or []
        if vertices:
            last = vertices[-1]
            if isinstance(last, dict):
                return (
                    float(last.get("x", 0.0)),
                    float(last.get("y", 0.0)),
                    float(last.get("yaw_rad", 0.0)),
                )

    raise ValueError("无法从 pose 或 trajectory 解析机器人位姿")


def draw_path(
    draw: ImageDraw.ImageDraw,
    points: List[Tuple[int, int]],
    width: int,
) -> None:
    if len(points) < 2:
        return
    line_w = max(3, width // 180)
    for i in range(len(points) - 1):
        draw.line([points[i], points[i + 1]], fill=(255, 140, 0, 230), width=line_w)


def draw_robot_arrow(
    draw: ImageDraw.ImageDraw,
    px: int,
    py: int,
    yaw_rad: float,
    image_size: Tuple[int, int],
    arrow_scale: float,
) -> None:
    width, height = image_size
    radius = max(10, int(min(width, height) // 40 * arrow_scale))
    arrow_len = max(45, int(min(width, height) // 7 * arrow_scale))
    arrow_width = max(4, int(min(width, height) // 120 * arrow_scale))

    dx = math.cos(yaw_rad) * arrow_len
    dy = -math.sin(yaw_rad) * arrow_len
    end_x = px + dx
    end_y = py + dy

    draw.ellipse(
        [px - radius, py - radius, px + radius, py + radius],
        fill=(0, 120, 255, 235),
        outline=(255, 255, 255, 255),
        width=max(2, arrow_width // 2),
    )
    draw.line([(px, py), (end_x, end_y)], fill=(0, 70, 220, 255), width=arrow_width)

    head_len = max(14, arrow_len * 0.32)
    for offset_deg in (150, -150):
        head_angle = yaw_rad + math.radians(offset_deg)
        hx = end_x + math.cos(head_angle) * head_len
        hy = end_y - math.sin(head_angle) * head_len
        draw.line([(end_x, end_y), (hx, hy)], fill=(0, 70, 220, 255), width=arrow_width)

    font = load_font(max(14, min(width, height) // 38))
    label = "ROBOT (last pose)"
    draw.rectangle([8, 8, 8 + 220, 34], fill=(255, 255, 255, 210))
    draw.text((12, 10), label, fill=(0, 60, 160, 255), font=font)


def compose_map(
    map_image_path: Path,
    map_yaml_path: Path,
    trajectory_path: Path | None,
    pose_path: Path | None,
    output_path: Path,
    pose_uv_path: Path | None,
    arrow_scale: float,
) -> Dict[str, Any]:
    yaml_data = load_map_yaml(map_yaml_path)
    resolution = float(yaml_data.get("resolution", 0.05))
    origin = yaml_data.get("origin") or [0.0, 0.0, 0.0]
    origin_x = float(origin[0])
    origin_y = float(origin[1])

    with Image.open(map_image_path) as source:
        image = source.convert("RGBA")
    width, height = image.size
    draw = ImageDraw.Draw(image, "RGBA")

    trajectory: Dict[str, Any] = {}
    if trajectory_path and trajectory_path.is_file():
        trajectory = json.loads(trajectory_path.read_text(encoding="utf-8"))

    pose_json: Dict[str, Any] | None = None
    if pose_path and pose_path.is_file():
        pose_json = json.loads(pose_path.read_text(encoding="utf-8"))

    path_points = extract_path_points(trajectory) if trajectory else []
    pixel_path = [
        world_to_pixel(x, y, origin_x, origin_y, resolution, width, height)
        for x, y in path_points
    ]
    draw_path(draw, pixel_path, width)

    rx, ry, ryaw = resolve_robot_pose(pose_json, trajectory if trajectory else None)
    rpx, rpy = world_to_pixel(rx, ry, origin_x, origin_y, resolution, width, height)
    draw_robot_arrow(draw, rpx, rpy, ryaw, (width, height), arrow_scale)

    ru, rv = world_to_uv(rx, ry, origin_x, origin_y, resolution, width, height)
    yaw_deg = math.degrees(ryaw)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.convert("RGB").save(output_path, quality=95)

    meta = {
        "map_image": str(map_image_path.resolve()),
        "map_yaml": str(map_yaml_path.resolve()),
        "output": str(output_path.resolve()),
        "path_point_count": len(path_points),
        "robot_pose_map": {"x": rx, "y": ry, "yaw_rad": ryaw, "yaw_deg": yaw_deg},
        "robot_pose_image": {"u": ru, "v": rv, "yaw_deg": yaw_deg},
        "map_origin": [origin_x, origin_y],
        "map_resolution": resolution,
        "image_size": [width, height],
    }

    if pose_uv_path is not None:
        pose_uv_path.write_text(json.dumps(meta["robot_pose_image"], indent=2) + "\n", encoding="utf-8")

    return meta


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="合成建图会话标注图（路径 + 大号机器人箭头）")
    parser.add_argument("--map-image", required=True, help="地图 PNG/PGM 路径")
    parser.add_argument("--map-yaml", required=True, help="地图 YAML 路径")
    parser.add_argument("--trajectory-json", help="trajectory_session.json 路径")
    parser.add_argument("--pose-json", help="last_pose_map.json 路径")
    parser.add_argument("--output", required=True, help="输出标注 PNG 路径")
    parser.add_argument("--pose-uv-json", help="输出 robot u/v/yaw JSON 路径")
    parser.add_argument("--arrow-scale", type=float, default=2.5, help="箭头放大倍数")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    map_image = Path(args.map_image).expanduser().resolve()
    map_yaml = Path(args.map_yaml).expanduser().resolve()
    trajectory = Path(args.trajectory_json).expanduser().resolve() if args.trajectory_json else None
    pose = Path(args.pose_json).expanduser().resolve() if args.pose_json else None
    output = Path(args.output).expanduser().resolve()
    pose_uv = Path(args.pose_uv_json).expanduser().resolve() if args.pose_uv_json else None

    if not map_image.is_file():
        raise SystemExit(f"地图图片不存在：{map_image}")
    if not map_yaml.is_file():
        raise SystemExit(f"地图 YAML 不存在：{map_yaml}")

    meta = compose_map(
        map_image_path=map_image,
        map_yaml_path=map_yaml,
        trajectory_path=trajectory,
        pose_path=pose,
        output_path=output,
        pose_uv_path=pose_uv,
        arrow_scale=max(1.0, float(args.arrow_scale)),
    )
    print(json.dumps(meta, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
