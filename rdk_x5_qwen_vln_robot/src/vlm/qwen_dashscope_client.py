#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cloud Qwen visual point client for rdk_x5_qwen_vln_robot.

Center-strict prompt semantics (locked object center vs inferred path waypoint).

Environment variables required:
    DASHSCOPE_API_KEY
    QWEN_BASE_URL
    QWEN_MODEL
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
    mode: str = "track",
    first_request: bool = False,
    image_width: Optional[int] = None,
    image_height: Optional[int] = None,
) -> str:
    mode = (mode or "track").strip().lower()
    if mode not in {"track", "search", "scan"}:
        mode = "track"

    size_line = ""
    if image_width and image_height:
        size_line = (
            f"The image shown to you has size {int(image_width)} x {int(image_height)} pixels "
            "(width x height). However, your output u and v must still be normalized 0.0 to 1.0.\n"
        )

    base_format = (
        "Return ONLY valid JSON. No markdown. No extra text.\n"
        + size_line
        + "Output exactly this JSON object shape:\n"
        '{"status":"locked|inferred","u":0.0,"v":0.0,"confidence":0.0,"reason":""}\n'
        "\n"
        "Coordinate rules:\n"
        "- u and v must be normalized coordinates from 0.0 to 1.0.\n"
        "- u is horizontal position from left to right.\n"
        "- v is vertical position from top to bottom.\n"
        "- Do not output pixel coordinates such as 320 or 480.\n"
        "- Output only ONE point.\n"
        "- Never output robot speed or movement commands.\n"
        "\n"
        "Decision order, very important:\n"
        "1. First decide whether the EXACT requested target object is clearly visible.\n"
        "2. If it is clearly visible, use status='locked' and output the target object center.\n"
        "3. If it is not clearly visible, use status='inferred' and output a safe navigable path waypoint.\n"
        "4. Do not use status='locked' for a guessed direction, a likely area, a route, or a similar wrong object.\n"
    )

    object_rules = (
        "Object naming rules:\n"
        "- If target is bottle, accept water bottle, plastic bottle, drink bottle, or mineral water bottle.\n"
        "- If target is cup, accept cup, mug, or paper cup, but do not confuse cup with bottle.\n"
        "- If target is chair, accept chair or seat, but do not confuse chair with table.\n"
        "- If target is person, locate the visible center of the person's body, not only the face.\n"
        "- Find the exact requested object category. Do not lock onto a similar but wrong object.\n"
    )

    locked_rules = (
        "Locked status rules:\n"
        "- Use status='locked' ONLY when the exact requested target is clearly visible.\n"
        "- If status='locked', u and v MUST be the visual center of the target object itself.\n"
        "- Internally estimate the visible bounding box of the exact target; output the center of that visible bounding box. Do not output the bounding box.\n"
        "- The point must lie on the visible object body.\n"
        "- The point must be near the geometric center of the visible target area, not a random point inside the object.\n"
        "- Do not output an edge, corner, cap, handle, label, top, bottom, shadow, reflection, floor point, path point, or navigation waypoint.\n"
        "- Do not output a point above, below, in front of, or behind the object.\n"
        "- Do not choose the image center unless the target object center is actually at the image center.\n"
        "- If the target is partly visible, output the center of the visible part of the target object.\n"
        "- For status='locked', confidence should usually be 0.8 to 1.0.\n"
        "- If you cannot identify the object center, do NOT output locked; output inferred instead.\n"
    )

    inferred_rules = (
        "Inferred status rules:\n"
        "- Use status='inferred' when the exact target is not clearly visible.\n"
        "- For status='inferred', u and v are a safe navigable path waypoint, not an object center.\n"
        "- Do not pretend the target is visible when it is not visible.\n"
        "- Do not imagine or invent a target location, room, table, shelf, container, or random place.\n"
        "- Choose a visible traversable route that can lead the robot to another area, such as an open corridor, doorway, passage, or clear open floor path.\n"
        "- The waypoint should be near the centerline of that visible route, approximately in the middle of the obstacle-free path.\n"
        "- Avoid obstacles, walls, furniture, object bodies, clutter, narrow gaps, and floor regions immediately blocked by objects.\n"
        "- If multiple routes are visible, prefer the route with the widest clearance and clearest forward continuation.\n"
        "- If no clear route is visible, choose the safest visible open-floor continuation with low confidence, not a random semantic area.\n"
        "- For status='inferred', confidence means how clear, safe, and useful that navigable path is.\n"
    )

    if mode == "track":
        prefix = "TRACK_FIRST" if first_request else "TRACK"
        extra_reason = "Reason may contain one short phrase.\n" if first_request else "Keep reason empty or very short.\n"
        task = (
            f"Mode: {prefix}.\n"
            f'Target: "{target_name}".\n'
            "If the exact target is clearly visible, output status='locked' and the center of the visible target object itself.\n"
            "The point must be on the object body itself.\n"
            "Do not choose the ground area in front of the target.\n"
            "Do not choose the route toward the target.\n"
            "Do not choose the image center unless the target object center is actually there.\n"
            "If the target is small but clearly visible, still output status='locked'.\n"
            "If the target is not clearly visible, output status='inferred' with a safe waypoint near the center of a visible obstacle-free path that can lead to another area, and use low confidence.\n"
            + extra_reason
        )
    elif mode == "search":
        prefix = "SEARCH_FIRST" if first_request else "SEARCH"
        extra_reason = (
            "Reason may contain one short phrase explaining the path clue, such as 'clear path ahead', 'open corridor center', or 'doorway path'.\n"
            if first_request
            else "Use scene context only; do not invent a visible target. Keep reason empty or very short.\n"
        )
        task = (
            f"Mode: {prefix}.\n"
            f'Target: "{target_name}".\n'
            "If the exact target is clearly visible, output status='locked' and the center of the visible target object itself.\n"
            "When locked, the point must be the object center, not the likely search direction.\n"
            "If the exact target is not clearly visible, output status='inferred' and choose a safe waypoint near the center of a visible obstacle-free route that can lead to another area.\n"
            "For inferred, u and v should point to the center of the navigable path, not a guessed object location or random search area.\n"
            + extra_reason
        )
    else:
        prefix = "SCAN_FIRST" if first_request else "SCAN"
        extra_reason = (
            "Reason may contain one short phrase, such as 'clear path ahead', 'open corridor center', or 'doorway path'.\n"
            if first_request
            else "Do not output a fake object center. Keep reason empty or very short.\n"
        )
        task = (
            f"Mode: {prefix}.\n"
            f'Target: "{target_name}".\n'
            "If the exact target is clearly visible, output status='locked' and the center of the visible target object itself.\n"
            "When locked, the point must be the object center, not the scan direction.\n"
            "If the exact target is not clearly visible, output status='inferred' and choose a safe waypoint near the center of a visible obstacle-free route that can lead to another area.\n"
            "For inferred, u and v should point to the center of the navigable path, not a fake object center or random scan area.\n"
            + extra_reason
        )

    return base_format + "\n" + object_rules + "\n" + locked_rules + "\n" + inferred_rules + "\n" + task


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
        max_tokens: int = 80,
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
        mode: str = "track",
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

    def _validate_and_map(self, raw: Dict[str, Any], image_info: Dict[str, Any]) -> Dict[str, Any]:
        status = str(raw.get("status", "inferred")).strip().lower()
        if status not in ("locked", "inferred"):
            status = "searching"

        confidence = _clamp_float(raw.get("confidence", 0.0), 0.0, 1.0, 0.0)
        reason = "" if raw.get("reason") is None else str(raw.get("reason", ""))
        u_raw = raw.get("u")
        v_raw = raw.get("v")

        if u_raw is None or v_raw is None:
            return {
                "usable": False,
                "_point_valid": False,
                "direction_valid": False,
                "status": status,
                "u": None,
                "v": None,
                "cx": None,
                "_raw_u": None,
                "_raw_v": None,
                "confidence": confidence,
                "reason": reason,
                "_coord_reason": "missing_point",
            }

        try:
            u_val = float(u_raw)
            v_val = float(v_raw)
        except Exception:
            return {
                "usable": False,
                "_point_valid": False,
                "direction_valid": False,
                "status": "searching",
                "u": None,
                "v": None,
                "cx": None,
                "_raw_u": u_raw,
                "_raw_v": v_raw,
                "confidence": 0.0,
                "reason": reason or "invalid coordinate",
                "_coord_reason": "invalid_non_numeric_coordinate",
            }

        valid = False
        u_norm: Optional[float] = None
        v_norm: Optional[float] = None
        if 0.0 <= u_val <= 1.0 and 0.0 <= v_val <= 1.0:
            u_norm, v_norm = u_val, v_val
            valid = True
        else:
            sent_w = image_info.get("sent_w")
            sent_h = image_info.get("sent_h")
            if sent_w and sent_h and 0.0 <= u_val <= float(sent_w) and 0.0 <= v_val <= float(sent_h):
                u_norm = u_val / float(sent_w)
                v_norm = v_val / float(sent_h)
                reason = reason or "pixel coordinate converted to normalized"
                valid = True

        if not valid or u_norm is None or v_norm is None:
            return {
                "usable": False,
                "_point_valid": False,
                "direction_valid": False,
                "status": "searching",
                "u": None,
                "v": None,
                "cx": None,
                "_raw_u": u_val,
                "_raw_v": v_val,
                "confidence": 0.0,
                "reason": reason or "coordinate out of range",
                "_coord_reason": "coordinate_out_of_range",
            }

        orig_w = int(image_info["orig_w"])
        orig_h = int(image_info["orig_h"])
        u_px = float(u_norm * max(1, orig_w - 1))
        v_px = float(v_norm * max(1, orig_h - 1))

        usable = bool(status == "locked" and confidence >= self.min_confidence)
        direction_valid = bool(status in ("locked", "inferred") and valid)
        coord_reason = status
        if status == "locked" and confidence < self.min_confidence:
            coord_reason = f"low_confidence:{confidence:.3f}"
        elif status == "inferred":
            coord_reason = "inferred_waypoint"

        u_out = v_out = cx_out = None
        if usable:
            u_out, v_out, cx_out = u_px, v_px, u_px
        elif status == "inferred" and direction_valid:
            u_out, v_out, cx_out = u_px, v_px, u_px

        return {
            "usable": usable,
            "_point_valid": usable,
            "direction_valid": direction_valid,
            "status": status,
            "u": u_out,
            "v": v_out,
            "cx": cx_out,
            "_raw_u": u_norm,
            "_raw_v": v_norm,
            "confidence": confidence,
            "reason": reason,
            "_coord_reason": coord_reason,
        }

    def infer_navigation(
        self,
        frame_bgr,
        instruction: str,
        *,
        mode: str = "track",
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
        result = self._validate_and_map(raw_json, image_info)
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
