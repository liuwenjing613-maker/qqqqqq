#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cloud Qwen visual point client for rdk_x5_qwen_vln_robot.

Explicit TARGET / PATH / NONE modes so target points and path waypoints are never mixed.
"""

import base64
import http.client
import json
import os
import re
import time
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import urlparse

import cv2

_JSON_SHAPE = (
    '{"mode":"TARGET|PATH|NONE",'
    '"target_visible":true,'
    '"u":0.0,'
    '"v":0.0,'
    '"waypoint_visible":false,'
    '"waypoint_u":null,'
    '"waypoint_v":null,'
    '"confidence":0.0,'
    '"reason":""}'
)


def parse_instruction_sequence(instruction: str) -> List[str]:
    instruction = (instruction or "").strip()
    if not instruction:
        return ["bottle"]

    parts = re.split(r"\b(?:first|then|next|finally|and then)\b|,", instruction, flags=re.IGNORECASE)
    targets: List[str] = []
    for part in parts:
        part = part.strip()
        if not part:
            continue
        m = re.search(r"(?:find|go\s+to|look\s+for)\s+(?:a\s+|an\s+|the\s+)?(.+)", part, re.IGNORECASE)
        target = m.group(1).strip() if m else part
        target = re.sub(r"[.!?]+$", "", target).strip()
        if target:
            targets.append(target)
    return targets or [instruction]


def select_current_target(targets: List[str], target_index: int) -> str:
    if not targets:
        return "bottle"
    target_index = max(0, min(int(target_index), len(targets) - 1))
    return targets[target_index]


def build_prompt(
    target_name: str,
    mode: str = "target",
    first_request: bool = False,
    image_width: Optional[int] = None,
    image_height: Optional[int] = None,
) -> str:
    """Build prompt for task mode ``target`` (bottle search) or ``path`` (free-space waypoint)."""
    mode = (mode or "target").strip().lower()
    legacy = {"track": "target", "search": "target", "scan": "target"}
    mode = legacy.get(mode, mode)
    if mode not in {"target", "path"}:
        mode = "target"

    size_line = ""
    if image_width and image_height:
        size_line = (
            f"The image shown to you has size {int(image_width)} x {int(image_height)} pixels "
            "(width x height). However, your output u/v and waypoint_u/waypoint_v must still be "
            "normalized 0.0 to 1.0.\n"
        )

    base_format = (
        "Return ONLY valid JSON. No markdown. No extra text.\n"
        + size_line
        + "Output exactly this JSON object shape:\n"
        + _JSON_SHAPE
        + "\n"
        "Global rules:\n"
        '- mode must be exactly "TARGET", "PATH", or "NONE".\n'
        "- u and v are normalized horizontal/vertical coordinates from 0.0 to 1.0.\n"
        "- waypoint_u and waypoint_v use the same normalized coordinate system.\n"
        "- Do not output pixel coordinates such as 320 or 480.\n"
        "- Never output robot speed or movement commands.\n"
        "- Never guess a hidden target location.\n"
        "- Never mix a target point and a path waypoint in one response.\n"
        "\n"
        "Decision order (strict, very important):\n"
        "1. Run the visual gates for this task mode first.\n"
        "2. Only if ALL required gates pass, output TARGET or PATH.\n"
        "3. If ANY required gate fails, output mode=NONE immediately. Do not guess.\n"
        "\n"
        "Visual gate (shared):\n"
        "- GATE_IMAGE: the image has enough visible scene detail to judge (not blank, blurred, or overexposed).\n"
        "\n"
        "Object naming rules:\n"
        "- If target is bottle, accept water bottle, plastic bottle, drink bottle, or mineral water bottle.\n"
        "- If target is cup, accept cup, mug, or paper cup, but do not confuse cup with bottle.\n"
        "- If target is chair, accept chair or seat, but do not confuse chair with table.\n"
        "- If target is person, locate the visible center of the person's body, not only the face.\n"
        "- Find the exact requested object category. Do not lock onto a similar but wrong object.\n"
    )

    target_rules = (
        "TARGET mode rules (center-strict):\n"
        '- Use mode="TARGET" ONLY when the exact requested target is clearly visible AND all TARGET visual gates pass.\n'
        "- Set target_visible=true and output u/v as the visual center of the target object body.\n"
        "- Internally estimate the visible bounding box of the exact target; output the geometric center of that bbox. Do not output the bbox.\n"
        "- The point must lie on the visible object body, near its geometric center, not a random interior point.\n"
        "- Do not output an edge, corner, cap, handle, label, top, bottom, shadow, reflection, floor point, path point, or navigation waypoint.\n"
        "- Do not choose the image center unless the target object center is actually at the image center.\n"
        "- If the target is partly visible, output the center of the visible part of the target object.\n"
        "- For bottle or cup, place the point at the center of the main body; ignore cap, strap, rope, handle, logo, and label when choosing the horizontal center.\n"
        "- Do not output the ground area in front of the target or the route toward the target.\n"
        "- waypoint_visible must be false; waypoint_u and waypoint_v must be null.\n"
        '- If the target is not clearly visible or any TARGET gate fails, use mode="NONE" with all coordinates null.\n'
        "- Do NOT return PATH mode or any path waypoint in target task.\n"
        "- For mode=TARGET, confidence should usually be 0.8 to 1.0.\n"
        "\n"
        "TARGET visual gates (ALL must pass for mode=TARGET):\n"
        "- GATE_CATEGORY: the EXACT requested target category is clearly visible.\n"
        "- GATE_ON_BODY: u/v lies on the visible target object body, not floor, shadow, neighbor object, or wall.\n"
        "- GATE_CENTER_STRICT: the point is the geometric center of the visible target area.\n"
        "- GATE_NOT_PATH: the point is NOT a navigation waypoint or search direction.\n"
        "If any gate fails: mode=NONE (do NOT output PATH in target task).\n"
    )

    path_rules = (
        "PATH mode rules (state-safe):\n"
        '- Use mode="PATH" ONLY when a safe traversable path is visible, the target is NOT visible, AND all PATH visual gates pass.\n'
        "- Do NOT guess where the hidden target is.\n"
        "- Set target_visible=false; u and v must be null.\n"
        "- Set waypoint_visible=true and output waypoint_u/waypoint_v on the center of the safe free path.\n"
        "- Use PATH only for a visible traversable floor, corridor, doorway, aisle, open passage, or largest connected free-floor region.\n"
        "- Choose a visible traversable route that can lead the robot to another area.\n"
        "- The waypoint should be near the centerline of that visible route, in the middle of the obstacle-free path.\n"
        "- If multiple routes are visible, prefer the route with the widest clearance and clearest forward continuation.\n"
        "- Prefer the higher half of the image: ground, floor, corridor centerline, or largest open region.\n"
        "- Avoid obstacles, walls, furniture, object bodies, clutter, narrow gaps, and floor regions immediately blocked by objects.\n"
        "- A large uniform wall-like surface, cabinet face, door face, vertical plane, or close obstacle is NOT a traversable path.\n"
        '- If most of the image is one uniform wall-like color/texture and no clear free-floor region is visible, use mode="NONE".\n'
        "- If unsure whether a region is floor or wall, prefer mode=\"NONE\" instead of placing a waypoint on a wall.\n"
        '- If no safe traversable path is visible, use mode="NONE" with all coordinates null.\n'
        "- Do NOT return TARGET mode or invent a target point in path task.\n"
        "- Do NOT output motion commands such as rotate or turn; only return PATH or NONE.\n"
        "- For mode=PATH, confidence means how clear, safe, and useful the navigable path is.\n"
        "\n"
        "PATH visual gates (ALL must pass for mode=PATH):\n"
        "- GATE_FLOOR: the higher half shows a connected traversable floor/ground region, not only a vertical wall.\n"
        "- GATE_ROUTE: a clear corridor, doorway, aisle, or open passage is visible.\n"
        "- GATE_NOT_WALL: waypoint is NOT on wall, cabinet, door, furniture, legs, clutter, shadow, border, or vertical plane.\n"
        "- GATE_CENTERLINE: waypoint is on the centerline of the widest obstacle-free path.\n"
        "- GATE_STATE_SAFE: if the view is dominated by one uniform wall-like surface with no floor/path boundary, the gate fails.\n"
        "If any gate fails: mode=NONE; set reason to gate_floor_failed, gate_not_wall, wall_like_no_floor, or no_traversable_path.\n"
        "Wall-like recovery rule: a large uniform same-color vertical region is NOT a path. "
        "If unsure whether it is floor or wall, choose NONE rather than placing a waypoint.\n"
    )

    none_rules = (
        "NONE mode rules:\n"
        '- Use mode="NONE" when the requested output for this task is not available or any visual gate fails.\n'
        "- Set target_visible=false and waypoint_visible=false.\n"
        "- All coordinate fields must be null.\n"
        "- reason may name the failed gate or scene condition, e.g. gate_category_failed, gate_floor_failed, wall_like_no_floor.\n"
    )

    if mode == "target":
        prefix = "TARGET_TASK_FIRST" if first_request else "TARGET_TASK"
        extra = "Reason may contain one short phrase.\n" if first_request else "Keep reason empty or very short.\n"
        task = (
            f"Task: {prefix}.\n"
            f'Target object: "{target_name}".\n'
            "Run TARGET visual gates first.\n"
            "If all gates pass and the exact target is clearly visible: mode=TARGET with object center u/v.\n"
            "If any gate fails or the target is not visible: mode=NONE. Do not output a path waypoint.\n"
            + extra
        )
        return base_format + "\n" + target_rules + "\n" + none_rules + "\n" + task

    prefix = "PATH_TASK_FIRST" if first_request else "PATH_TASK"
    extra = (
        "Reason may contain one short phrase such as 'safe free-space corridor visible ahead' "
        "or a gate failure such as 'wall_like_no_floor'.\n"
        if first_request
        else "Keep reason empty or very short.\n"
    )
    task = (
        f"Task: {prefix}.\n"
        f'Original mission target (NOT visible requirement): "{target_name}".\n'
        "The target is not required to be visible. Analyze only traversable free space.\n"
        "Run PATH visual gates first.\n"
        "If all gates pass and a safe path exists: mode=PATH with waypoint_u/waypoint_v at the path center.\n"
        "If any gate fails or no safe path exists: mode=NONE.\n"
        "Do not guess target location.\n"
        + extra
    )
    return base_format + "\n" + path_rules + "\n" + none_rules + "\n" + task


def _extract_json(text: str) -> Dict[str, Any]:
    text = (text or "").strip()
    try:
        return json.loads(text)
    except Exception:
        pass
    match = re.search(r"\{.*\}", text, re.S)
    if not match:
        raise RuntimeError("No JSON found in model output: " + text)
    return json.loads(match.group(0))


def _clamp_float(value: Any, lo: float, hi: float, default: float) -> float:
    try:
        x = float(value)
    except Exception:
        return default
    return max(lo, min(hi, x))


def _safe_optional_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _map_coord_pair(
    u_raw: Any,
    v_raw: Any,
    image_info: Dict[str, Any],
) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float], bool, str]:
    """Map one u/v pair to pixel + normalized coords. Returns (u_px, v_px, u_norm, v_norm, valid, reason)."""
    if u_raw is None or v_raw is None:
        return None, None, None, None, False, "missing_point"

    u_val = _safe_optional_float(u_raw)
    v_val = _safe_optional_float(v_raw)
    if u_val is None or v_val is None:
        return None, None, None, None, False, "invalid_non_numeric_coordinate"

    u_norm: Optional[float] = None
    v_norm: Optional[float] = None
    valid = False
    reason = ""

    if 0.0 <= u_val <= 1.0 and 0.0 <= v_val <= 1.0:
        u_norm, v_norm = u_val, v_val
        valid = True
    else:
        sent_w = image_info.get("sent_w")
        sent_h = image_info.get("sent_h")
        orig_w = image_info.get("orig_w")
        orig_h = image_info.get("orig_h")
        if sent_w and sent_h and 0.0 <= u_val <= float(sent_w) and 0.0 <= v_val <= float(sent_h):
            u_norm = u_val / float(sent_w)
            v_norm = v_val / float(sent_h)
            valid = True
            reason = "pixel_coord_converted_from_sent_image"
        elif orig_w and orig_h and 0.0 <= u_val <= float(orig_w) and 0.0 <= v_val <= float(orig_h):
            u_norm = u_val / max(1.0, float(orig_w) - 1.0)
            v_norm = v_val / max(1.0, float(orig_h) - 1.0)
            valid = True
            reason = "pixel_coord_converted_from_orig_image"

    if not valid or u_norm is None or v_norm is None:
        return None, None, None, None, False, "coordinate_out_of_range"

    orig_w = int(image_info["orig_w"])
    orig_h = int(image_info["orig_h"])
    u_px = float(u_norm * max(1, orig_w - 1))
    v_px = float(v_norm * max(1, orig_h - 1))
    return u_px, v_px, u_norm, v_norm, True, reason


def _legacy_mode_from_status(raw: Dict[str, Any], task_mode: str) -> str:
    status = str(raw.get("status", "")).strip().lower()
    if status == "locked":
        return "TARGET"
    if status == "inferred":
        return "PATH" if task_mode == "path" else "NONE"
    return "NONE"


def _check_env() -> Tuple[str, str, str]:
    api_key = os.getenv("DASHSCOPE_API_KEY")
    base_url = os.getenv("QWEN_BASE_URL")
    model = os.getenv("QWEN_MODEL")
    if not api_key:
        raise RuntimeError("Missing DASHSCOPE_API_KEY")
    if not base_url:
        raise RuntimeError("Missing QWEN_BASE_URL")
    if not model:
        raise RuntimeError("Missing QWEN_MODEL")
    return api_key, base_url.rstrip("/"), model


class KeepAliveHTTPClient:
    def __init__(self, api_key: str, base_url: str, timeout: float = 15.0):
        self.api_key = api_key
        self.base_url = base_url.rstrip("/")
        self.timeout = float(timeout)
        parsed = urlparse(self.base_url)
        if parsed.scheme != "https":
            raise RuntimeError("Only https QWEN_BASE_URL is supported")
        self.host = parsed.netloc
        self.base_path = parsed.path.rstrip("/")
        self.conn: Optional[http.client.HTTPSConnection] = None

    def connect(self) -> None:
        if self.conn is None:
            self.conn = http.client.HTTPSConnection(self.host, timeout=self.timeout)

    def close(self) -> None:
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None

    def post_json(self, endpoint: str, payload: Dict[str, Any]) -> str:
        path = self.base_path + endpoint
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Authorization": "Bearer " + self.api_key,
            "Connection": "keep-alive",
        }
        last_error = None
        for attempt in range(2):
            try:
                self.connect()
                self.conn.request("POST", path, body=body, headers=headers)
                response = self.conn.getresponse()
                text = response.read().decode("utf-8", errors="ignore")
                if response.status >= 400:
                    raise RuntimeError(f"HTTP {response.status} {response.reason}: {text}")
                return text
            except Exception as e:
                last_error = e
                self.close()
                if attempt == 0:
                    continue
        raise RuntimeError(f"post_json failed: {repr(last_error)}")


class QwenDashScopeClient:
    def __init__(
        self,
        timeout: float = 15.0,
        resize_width: int = 640,
        jpeg_quality: int = 80,
        min_confidence: float = 0.60,
        max_tokens: int = 120,
    ):
        api_key, base_url, model = _check_env()
        self.model = model
        self.timeout = float(timeout)
        self.resize_width = int(resize_width)
        self.jpeg_quality = int(jpeg_quality)
        self.min_confidence = float(min_confidence)
        self.max_tokens = int(max_tokens)
        self.http = KeepAliveHTTPClient(api_key=api_key, base_url=base_url, timeout=timeout)

    def close(self) -> None:
        self.http.close()

    def _resize_frame(self, frame_bgr):
        orig_h, orig_w = frame_bgr.shape[:2]
        if self.resize_width <= 0 or orig_w <= self.resize_width:
            return frame_bgr, orig_w, orig_h, orig_w, orig_h
        scale = self.resize_width / float(orig_w)
        new_w = self.resize_width
        new_h = int(orig_h * scale)
        resized = cv2.resize(frame_bgr, (new_w, new_h))
        return resized, orig_w, orig_h, new_w, new_h

    def _frame_to_data_url(self, frame_bgr) -> Dict[str, Any]:
        model_frame, orig_w, orig_h, sent_w, sent_h = self._resize_frame(frame_bgr)
        ok, buf = cv2.imencode(".jpg", model_frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        image_bytes = buf.tobytes()
        image_b64 = base64.b64encode(image_bytes).decode("utf-8")
        return {
            "data_url": "data:image/jpeg;base64," + image_b64,
            "orig_w": orig_w,
            "orig_h": orig_h,
            "sent_w": sent_w,
            "sent_h": sent_h,
            "sent_bytes": len(image_bytes),
        }

    def _build_payload(
        self,
        data_url: str,
        instruction: str,
        *,
        mode: str = "target",
        first_request: bool = False,
        target_index: int = 0,
        sent_w: Optional[int] = None,
        sent_h: Optional[int] = None,
    ) -> Dict[str, Any]:
        targets = parse_instruction_sequence(instruction)
        target_name = select_current_target(targets, target_index)
        prompt = build_prompt(
            target_name,
            mode=mode,
            first_request=first_request,
            image_width=sent_w,
            image_height=sent_h,
        )
        return {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": data_url}},
                        {"type": "text", "text": prompt},
                    ],
                }
            ],
            "temperature": 0,
            "max_tokens": self.max_tokens,
        }

    def _validate_and_map(
        self,
        raw: Dict[str, Any],
        image_info: Dict[str, Any],
        *,
        task_mode: str = "target",
    ) -> Dict[str, Any]:
        task_mode = (task_mode or "target").strip().lower()
        legacy = {"track": "target", "search": "target", "scan": "target"}
        task_mode = legacy.get(task_mode, task_mode)

        confidence = _clamp_float(raw.get("confidence", 0.0), 0.0, 1.0, 0.0)
        reason = "" if raw.get("reason") is None else str(raw.get("reason", ""))

        mode = str(raw.get("mode", "")).strip().upper()
        if mode not in ("TARGET", "PATH", "NONE"):
            mode = _legacy_mode_from_status(raw, task_mode)

        target_visible = bool(raw.get("target_visible", mode == "TARGET"))
        waypoint_visible = bool(raw.get("waypoint_visible", mode == "PATH"))

        target_u_px = target_v_px = target_u_norm = target_v_norm = None
        waypoint_u_px = waypoint_v_px = waypoint_u_norm = waypoint_v_norm = None
        coord_reason = mode

        if mode == "TARGET":
            u_px, v_px, u_norm, v_norm, valid, cr = _map_coord_pair(raw.get("u"), raw.get("v"), image_info)
            if valid and target_visible:
                target_u_px, target_v_px = u_px, v_px
                target_u_norm, target_v_norm = u_norm, v_norm
            else:
                mode = "NONE"
                target_visible = False
                coord_reason = cr if not valid else "target_not_visible"
        elif mode == "PATH":
            wu_px, wv_px, wu_norm, wv_norm, valid, cr = _map_coord_pair(
                raw.get("waypoint_u"), raw.get("waypoint_v"), image_info
            )
            if valid and waypoint_visible:
                waypoint_u_px, waypoint_v_px = wu_px, wv_px
                waypoint_u_norm, waypoint_v_norm = wu_norm, wv_norm
            else:
                mode = "NONE"
                waypoint_visible = False
                coord_reason = cr if not valid else "waypoint_not_visible"
        else:
            mode = "NONE"
            target_visible = False
            waypoint_visible = False
            coord_reason = "none"

        usable = False
        if mode == "TARGET" and target_u_px is not None and target_v_px is not None:
            usable = confidence >= self.min_confidence
            if not usable:
                coord_reason = f"low_confidence:{confidence:.3f}"
        elif mode == "PATH" and waypoint_u_px is not None and waypoint_v_px is not None:
            usable = True

        # Backward-compat aliases for older nav code paths.
        status = "locked" if mode == "TARGET" else ("inferred" if mode == "PATH" else "searching")
        u_out = target_u_px if mode == "TARGET" else None
        v_out = target_v_px if mode == "TARGET" else None
        direction_valid = mode in ("TARGET", "PATH") and (
            (mode == "TARGET" and target_u_px is not None) or (mode == "PATH" and waypoint_u_px is not None)
        )

        return {
            "mode": mode,
            "target_visible": target_visible,
            "waypoint_visible": waypoint_visible,
            "u": u_out,
            "v": v_out,
            "waypoint_u": waypoint_u_px,
            "waypoint_v": waypoint_v_px,
            "cx": u_out,
            "usable": usable,
            "_point_valid": usable and mode == "TARGET",
            "direction_valid": direction_valid,
            "status": status,
            "_raw_u": target_u_norm,
            "_raw_v": target_v_norm,
            "_raw_waypoint_u": waypoint_u_norm,
            "_raw_waypoint_v": waypoint_v_norm,
            "confidence": confidence,
            "reason": reason,
            "_coord_reason": coord_reason,
        }

    def infer_navigation(
        self,
        frame_bgr,
        instruction: str,
        *,
        mode: str = "target",
        first_request: bool = False,
        target_index: int = 0,
    ) -> Dict[str, Any]:
        start = time.perf_counter()
        image_info = self._frame_to_data_url(frame_bgr)
        payload = self._build_payload(
            image_info["data_url"],
            instruction,
            mode=mode,
            first_request=first_request,
            target_index=target_index,
            sent_w=image_info["sent_w"],
            sent_h=image_info["sent_h"],
        )
        response_text = self.http.post_json("/chat/completions", payload)
        response_json = json.loads(response_text)
        raw_content = response_json["choices"][0]["message"]["content"]
        raw_json = _extract_json(raw_content)
        result = self._validate_and_map(raw_json, image_info, task_mode=mode)
        end = time.perf_counter()
        result.update({
            "_raw_json": raw_json,
            "_raw_model_output": raw_content,
            "_latency_sec": end - start,
            "_api_model": self.model,
            "_qwen_mode": mode,
            "_first_request": bool(first_request),
            "_sent_w": image_info["sent_w"],
            "_sent_h": image_info["sent_h"],
            "_sent_bytes": image_info["sent_bytes"],
            "_orig_image_width": image_info["orig_w"],
            "_orig_image_height": image_info["orig_h"],
        })
        return result
