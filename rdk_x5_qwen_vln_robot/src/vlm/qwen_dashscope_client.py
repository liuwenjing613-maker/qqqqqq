#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cloud Qwen visual point client for rdk_x5_qwen_vln_robot.

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
from typing import Any, Dict, Optional
from urllib.parse import urlparse

import cv2


DEFAULT_PROMPT_TEMPLATE = """
Return ONLY valid JSON. No markdown. No extra text.
Output format:
{{"status":"locked|inferred|searching","u":0.0,"v":0.0,"confidence":0.0,"reason":""}}

Coordinate rules:
- u and v must be normalized coordinates from 0.0 to 1.0.
- u is horizontal position from left to right.
- v is vertical position from top to bottom.
- Do not output pixel coordinates such as 320 or 480.
- Output only ONE point.
- For status='locked', the point must be the visual center of the target object itself.
- For status='locked', the point must lie on the visible object body.
- For status='locked', do not output a floor point.
- For status='locked', do not output a path point.
- For status='locked', do not output a navigation waypoint.
- If the target is partly visible, output the center of the visible part of the target object.
- If the exact target is not visible, use status='searching' with u=null and v=null.
- Never output robot speed.

Object naming rules:
- If target is bottle, accept water bottle, plastic bottle, drink bottle, or mineral water bottle.
- If target is cup, accept cup, mug, or paper cup, but do not confuse cup with bottle.
- Find the exact requested object category.

Task: Find target: {target_name}.
If the target is visible, output status='locked' and the center of the visible target object itself.
The point must be on the object body itself.
Do not choose the ground area in front of the target.
Do not choose the route toward the target.
Do not choose the image center unless the target object center is actually there.
If the target is small but clearly visible, still output status='locked'.
If not visible, output status='searching' with u=null and v=null.
Keep reason empty or very short.
"""


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


def _check_env():
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

    def connect(self):
        if self.conn is None:
            self.conn = http.client.HTTPSConnection(self.host, timeout=self.timeout)

    def close(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None

    def post_json(self, endpoint: str, payload: Dict[str, Any]) -> str:
        path = self.base_path + endpoint
        body = json.dumps(payload).encode("utf-8")
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

    def close(self):
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

    def _frame_to_data_url(self, frame_bgr):
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

    def _build_payload(self, data_url: str, instruction: str) -> Dict[str, Any]:
        target_name = (instruction or "find bottle").strip()
        prompt = DEFAULT_PROMPT_TEMPLATE.format(target_name=target_name)
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
        status = str(raw.get("status", "searching")).strip().lower()
        if status not in ("locked", "inferred", "searching"):
            status = "searching"
        confidence = _clamp_float(raw.get("confidence", 0.0), 0.0, 1.0, 0.0)
        reason = "" if raw.get("reason") is None else str(raw.get("reason"))
        u_raw = raw.get("u", None)
        v_raw = raw.get("v", None)
        coord_reason = status

        if u_raw is None or v_raw is None:
            return {
                "usable": False, "_point_valid": False, "status": status,
                "u": None, "v": None, "cx": None, "_raw_u": None, "_raw_v": None,
                "confidence": confidence, "reason": reason, "_coord_reason": "missing_point",
            }
        try:
            u_norm = float(u_raw)
            v_norm = float(v_raw)
        except Exception:
            return {
                "usable": False, "_point_valid": False, "status": "searching",
                "u": None, "v": None, "cx": None, "_raw_u": u_raw, "_raw_v": v_raw,
                "confidence": 0.0, "reason": "invalid coordinate",
                "_coord_reason": "invalid_non_numeric_coordinate",
            }
        if not (0.0 <= u_norm <= 1.0 and 0.0 <= v_norm <= 1.0):
            return {
                "usable": False, "_point_valid": False, "status": "searching",
                "u": None, "v": None, "cx": None, "_raw_u": u_norm, "_raw_v": v_norm,
                "confidence": 0.0, "reason": "coordinate out of normalized range",
                "_coord_reason": "coordinate_out_of_range",
            }
        orig_w = int(image_info["orig_w"])
        orig_h = int(image_info["orig_h"])
        u_px = float(u_norm * max(1, orig_w - 1))
        v_px = float(v_norm * max(1, orig_h - 1))
        usable = status == "locked" and confidence >= self.min_confidence
        if status != "locked":
            coord_reason = f"not_locked:{status}"
        elif confidence < self.min_confidence:
            coord_reason = f"low_confidence:{confidence:.3f}"
        return {
            "usable": usable, "_point_valid": usable, "status": status,
            "u": u_px if usable else None, "v": v_px if usable else None,
            "cx": u_px if usable else None, "_raw_u": u_norm, "_raw_v": v_norm,
            "confidence": confidence, "reason": reason, "_coord_reason": coord_reason,
        }

    def infer_navigation(self, frame_bgr, instruction: str) -> Dict[str, Any]:
        start = time.perf_counter()
        image_info = self._frame_to_data_url(frame_bgr)
        payload = self._build_payload(image_info["data_url"], instruction)
        response_text = self.http.post_json("/chat/completions", payload)
        response_json = json.loads(response_text)
        raw_content = response_json["choices"][0]["message"]["content"]
        raw_json = _extract_json(raw_content)
        result = self._validate_and_map(raw_json, image_info)
        end = time.perf_counter()
        result.update({
            "_raw_json": raw_json, "_raw_model_output": raw_content,
            "_latency_sec": end - start, "_api_model": self.model,
            "_sent_w": image_info["sent_w"], "_sent_h": image_info["sent_h"],
            "_sent_bytes": image_info["sent_bytes"],
            "_orig_image_width": image_info["orig_w"], "_orig_image_height": image_info["orig_h"],
        })
        return result
