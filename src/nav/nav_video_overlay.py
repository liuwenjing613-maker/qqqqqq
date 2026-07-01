#!/usr/bin/env python3
"""Shared navigation overlay: target, FSM state, u/v, velocity."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import cv2


def front_min_from_nav_state(nav_state: Optional[Dict[str, Any]]) -> Optional[float]:
    """Read nav LiDAR distance from /nav_state (target column when tracking, else front-min)."""
    if not isinstance(nav_state, dict):
        return None
    for key in ("front_distance", "target_distance", "front_min_distance"):
        val = nav_state.get(key)
        if val is not None:
            return float(val)
    safety = nav_state.get("safety")
    if isinstance(safety, dict):
        for key in ("front_distance", "target_distance", "front_min_distance"):
            val = safety.get(key)
            if val is not None:
                return float(val)
    return None


def safe_json_load(text: str) -> Dict[str, Any]:
    try:
        data = json.loads(text)
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def parse_bbox_xyxy(bbox: Any) -> Optional[Tuple[int, int, int, int]]:
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return None
    x, y, a, b = [float(v) for v in bbox]
    if a > x and b > y:
        x1, y1, x2, y2 = x, y, a, b
    else:
        x1, y1, x2, y2 = x, y, x + a, y + b
    if x2 <= x1 or y2 <= y1:
        return None
    return int(x1), int(y1), int(x2), int(y2)


def target_info_from_dict(
    target_bbox: Optional[Dict[str, Any]] = None,
    target_point: Optional[Dict[str, Any]] = None,
    nav_state: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    bbox = target_bbox if isinstance(target_bbox, dict) else {}
    point = target_point if isinstance(target_point, dict) else {}
    nav = nav_state if isinstance(nav_state, dict) else {}

    u: Optional[float] = None
    v: Optional[float] = None
    for source in (bbox, point, nav.get("target", {}) or {}):
        if not isinstance(source, dict):
            continue
        su = source.get("u", source.get("cx"))
        sv = source.get("v", source.get("cy"))
        if su is not None and sv is not None:
            u, v = float(su), float(sv)
            break

    visible = bool(bbox.get("visible", False))
    class_name = str(bbox.get("class_name", bbox.get("class", "")) or "-")
    score = bbox.get("score", bbox.get("confidence"))
    reason = str(bbox.get("reason", bbox.get("vote_reason", "not_visible")) or "not_visible")
    area_ratio = bbox.get("area_ratio")
    raw_bbox = bbox.get("bbox_xyxy") or bbox.get("bbox") or bbox.get("bbox_xywh")

    return {
        "visible": visible,
        "class_name": class_name,
        "score": score,
        "reason": reason,
        "u": u,
        "v": v,
        "area_ratio": area_ratio,
        "bbox": raw_bbox,
    }


@dataclass
class NavOverlayContext:
    nav_state: Dict[str, Any] = field(default_factory=dict)
    target_bbox: Dict[str, Any] = field(default_factory=dict)
    target_point: Dict[str, Any] = field(default_factory=dict)
    cmd_vx: float = 0.0
    cmd_wz: float = 0.0
    last_fsm_mode: str = ""
    state_transition: str = ""
    state_transition_time: float = 0.0

    def update_nav_state(self, data: Dict[str, Any], now: Optional[float] = None) -> None:
        if not data:
            return
        now = float(now if now is not None else time.time())
        fsm_mode = str(data.get("fsm_mode", data.get("mode", "")) or "")
        if fsm_mode and fsm_mode != self.last_fsm_mode:
            if self.last_fsm_mode:
                self.state_transition = f"{self.last_fsm_mode} -> {fsm_mode}"
                self.state_transition_time = now
            self.last_fsm_mode = fsm_mode
        if data.get("safe_cmd_vx") is not None:
            self.cmd_vx = float(data.get("safe_cmd_vx", self.cmd_vx))
        if data.get("safe_cmd_wz") is not None:
            self.cmd_wz = float(data.get("safe_cmd_wz", self.cmd_wz))
        self.nav_state = data

    def update_cmd(self, vx: float, wz: float) -> None:
        self.cmd_vx = float(vx)
        self.cmd_wz = float(wz)

    def target_info(self, override: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        if override is not None:
            merged = dict(override)
            if merged.get("visible") and merged.get("bbox") is None and merged.get("bbox_xyxy"):
                merged["bbox"] = merged.get("bbox_xyxy")
            return target_info_from_dict(merged, self.target_point, self.nav_state)
        return target_info_from_dict(self.target_bbox, self.target_point, self.nav_state)

    def fsm_mode(self) -> str:
        return str(self.nav_state.get("fsm_mode", self.nav_state.get("mode", "")) or "")

    def fsm_reason(self) -> str:
        return str(self.nav_state.get("fsm_reason", self.nav_state.get("desired_reason", "")) or "")


def build_overlay_lines(
    ctx: NavOverlayContext,
    target_override: Optional[Dict[str, Any]] = None,
    header_lines: Optional[List[str]] = None,
    now: Optional[float] = None,
) -> List[str]:
    now = float(now if now is not None else time.time())
    target = ctx.target_info(target_override)
    lines: List[str] = list(header_lines or [])

    if target["visible"]:
        score_txt = "-"
        if target["score"] is not None:
            score_txt = f"{float(target['score']):.3f}"
        area_txt = "-"
        if target["area_ratio"] is not None:
            area_txt = f"{float(target['area_ratio']):.4f}"
        lines.append(f"[1 TARGET] class={target['class_name']} score={score_txt} area={area_txt}")
    else:
        lines.append(f"[1 TARGET] LOST reason={target['reason']}")

    transition = ""
    if ctx.state_transition and now - ctx.state_transition_time <= 3.0:
        transition = f" switch={ctx.state_transition}"
    lines.append(f"[2 STATE] {ctx.fsm_mode() or '-'} ({ctx.fsm_reason() or '-'}){transition}")

    if target["u"] is not None and target["v"] is not None:
        lines.append(f"[3 UV] u={target['u']:.1f} v={target['v']:.1f}")
    else:
        lines.append("[3 UV] u=- v=-")

    lines.append(f"[4 VEL] vx={ctx.cmd_vx:+.3f} wz={ctx.cmd_wz:+.3f}")

    front = front_min_from_nav_state(ctx.nav_state)
    if front is not None:
        lines.append(f"front={front:.2f}m")
    return lines


def draw_text_block(
    img,
    lines: List[str],
    origin: Tuple[int, int] = (12, 12),
    line_height: int = 24,
    font_scale: float = 0.58,
):
    if not lines:
        return img

    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 1
    max_width = 0
    for line in lines:
        (tw, _), _ = cv2.getTextSize(line, font, font_scale, thickness)
        max_width = max(max_width, tw)

    panel_w = max_width + 24
    panel_h = line_height * len(lines) + 16
    x0, y0 = origin
    overlay = img.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.58, img, 0.42, 0, img)

    y = y0 + 20
    for line in lines:
        cv2.putText(img, line, (x0 + 10, y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
        y += line_height
    return img


def annotate_nav_frame(
    frame,
    ctx: NavOverlayContext,
    *,
    target_override: Optional[Dict[str, Any]] = None,
    header_lines: Optional[List[str]] = None,
    status_banner: Optional[str] = None,
    draw_target_graphics: bool = True,
    draw_center_line: bool = True,
) -> Any:
    vis = frame.copy()
    h, w = vis.shape[:2]
    target = ctx.target_info(target_override)

    if draw_target_graphics:
        bbox_xyxy = parse_bbox_xyxy(target.get("bbox"))
        if target["visible"] and bbox_xyxy is not None:
            x1, y1, x2, y2 = bbox_xyxy
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if target["u"] is not None and target["v"] is not None:
                cx, cy = int(target["u"]), int(target["v"])
                cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(
                    vis,
                    f"u={cx} v={cy}",
                    (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
        if draw_center_line:
            cv2.line(vis, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)

    banner = status_banner if status_banner is not None else (ctx.fsm_mode() or "NAV")
    fsm_mode = ctx.fsm_mode()
    if fsm_mode == "SUCCESS":
        banner_color = (0, 200, 0)
    elif fsm_mode in ("BLOCKED", "FAILED"):
        banner_color = (0, 0, 220)
    else:
        banner_color = (0, 180, 255)

    cv2.putText(
        vis,
        banner,
        (max(10, w - 220), 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        banner_color,
        2,
        cv2.LINE_AA,
    )

    lines = build_overlay_lines(ctx, target_override=target_override, header_lines=header_lines)
    draw_text_block(vis, lines, origin=(12, 12))
    return vis
