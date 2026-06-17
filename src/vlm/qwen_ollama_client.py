#!/usr/bin/env python3
import base64
import json
import logging
import re
import subprocess
import time
from typing import Any, Dict, Optional, Tuple

import cv2
import requests

logger = logging.getLogger(__name__)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _safe_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _normalize_status(raw_status: Any, target_visible: bool, point_valid: bool, stop: bool) -> str:
    status = str(raw_status or "searching").strip()
    if "/" not in status:
        return status

    # Model echoed the enum template; infer a concrete status.
    if stop:
        return "success"
    if target_visible and point_valid:
        return "target_locked"
    if "unsafe" in status and stop:
        return "unsafe"
    return "searching"


def parse_nav_result(
    raw: Dict[str, Any],
    orig_w: int,
    orig_h: int,
    model_w: int,
    model_h: int,
    sx: float,
    sy: float,
) -> Dict[str, Any]:
    """
    解析千问 JSON，只保留 u/v 点坐标（原始图像像素系）。
    """
    target_visible = bool(raw.get("target_visible", False))
    confidence = _safe_float(raw.get("confidence"))
    if confidence is None:
        confidence = 0.0
    stop = bool(raw.get("stop", False))

    u_raw = _safe_float(raw.get("u"))
    v_raw = _safe_float(raw.get("v"))

    u: Optional[float] = None
    v: Optional[float] = None
    coords_scaled = False

    if u_raw is not None and v_raw is not None:
        if u_raw <= model_w and v_raw <= model_h and (sx > 1.01 or sy > 1.01):
            logger.warning(
                "u,v look like model-image coords (u=%.1f v=%.1f model=%dx%d), scaling to orig",
                u_raw,
                v_raw,
                model_w,
                model_h,
            )
            u_raw *= sx
            v_raw *= sy
            coords_scaled = True

        u = _clamp(u_raw, 0, max(0, orig_w - 1))
        v = _clamp(v_raw, 0, max(0, orig_h - 1))

    point_valid = u is not None and v is not None
    status = _normalize_status(raw.get("status", "searching"), target_visible, point_valid, stop)

    return {
        "status": status,
        "target_visible": target_visible and point_valid,
        "u": u,
        "v": v,
        "cx": u,
        "confidence": confidence,
        "stop": stop,
        "_coords_scaled_from_model": coords_scaled,
        "_point_valid": point_valid,
    }


class QwenOllamaClient:
    """
    本地 Ollama Qwen2.5-VL 客户端。
    输入 BGR 图像 + 指令，输出 u/v 点伺服 JSON。
    """

    def __init__(
        self,
        model: str = "qwen2.5vl:3b",
        url: str = "http://127.0.0.1:11434/api/generate",
        timeout: float = 900.0,
        resize_width: int = 256,
        jpeg_quality: int = 60,
        num_predict: int = 80,
        num_ctx: int = 2048,
    ):
        self.model = model
        self.url = url
        self.timeout = float(timeout)
        self.resize_width = int(resize_width)
        self.jpeg_quality = int(jpeg_quality)
        self.num_predict = int(num_predict)
        self.num_ctx = int(num_ctx)

    def _build_prompt(self, instruction: str, orig_w: int, orig_h: int) -> str:
        return f"""
你是移动机器人视觉导航模块。只输出严格 JSON，不要输出 Markdown，不要解释。

用户目标：{instruction}

原始图像尺寸：
width={orig_w}, height={orig_h}

请判断图中是否能看到用户目标，并输出目标中心像素点。

输出 JSON 格式必须为：
{{
  "status": "target_locked/searching/success/unsafe",
  "target_visible": true,
  "u": 0,
  "v": 0,
  "confidence": 0.0,
  "stop": false
}}

规则：
1. 如果清楚看到目标，status="target_locked"，target_visible=true。
2. u 是目标中心点横坐标，范围 0 到 {orig_w - 1}。
3. v 是目标中心点纵坐标，范围 0 到 {orig_h - 1}。
4. 如果看不到目标，status="searching"，target_visible=false，u=null，v=null。
5. 如果目标已经非常近或占画面很大，status="success"，stop=true。
6. 不要输出 bbox，不要输出描述文字，不要输出 reason。
7. 坐标必须是原始图像坐标，不是缩放图坐标。
""".strip()

    def _resize_frame(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        if w <= self.resize_width:
            return frame_bgr.copy(), 1.0, 1.0

        scale = self.resize_width / float(w)
        new_w = self.resize_width
        new_h = int(round(h * scale))
        resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
        sx = w / float(new_w)
        sy = h / float(new_h)
        return resized, sx, sy

    def _frame_to_base64(self, frame_bgr) -> Tuple[str, int, int, float, float]:
        resized, sx, sy = self._resize_frame(frame_bgr)
        ok, buf = cv2.imencode(
            ".jpg",
            resized,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("cv2.imencode failed")

        img64 = base64.b64encode(buf.tobytes()).decode("utf-8")
        rh, rw = resized.shape[:2]
        return img64, rw, rh, sx, sy

    def _extract_json(self, text: str) -> Dict[str, Any]:
        text = (text or "").strip()

        try:
            return json.loads(text)
        except Exception:
            pass

        match = re.search(r"\{[\s\S]*\}", text)
        if not match:
            raise RuntimeError(f"No JSON found in model output: {text[:500]}")

        return json.loads(match.group(0))

    def recover_stuck_server(self) -> None:
        """Kill llama-server processes stuck on vision encoding."""
        script = "/root/rdk_x5_vln_robot/scripts/ollama_recover.sh"
        try:
            subprocess.run(["bash", script], check=False, timeout=15)
            return
        except Exception:
            pass

        try:
            out = subprocess.check_output(
                ["ps", "-eo", "pid=,pcpu=,cmd="],
                text=True,
                timeout=5,
            )
        except Exception:
            return

        for line in out.splitlines():
            if "llama-server" not in line:
                continue
            parts = line.split(None, 2)
            if len(parts) < 3:
                continue
            pid, pcpu_str = parts[0], parts[1]
            try:
                pcpu = float(pcpu_str)
            except ValueError:
                continue
            if pcpu >= 200.0:
                subprocess.run(["kill", "-9", pid], check=False)

    def check_health(self, timeout: float = 5.0) -> bool:
        try:
            resp = requests.get(
                self.url.replace("/api/generate", "/api/tags"),
                timeout=timeout,
            )
            resp.raise_for_status()
            models = resp.json().get("models", [])
            return any(m.get("name", "").startswith(self.model.split(":")[0]) for m in models)
        except Exception:
            return False

    def warmup(self, timeout: Optional[float] = None) -> float:
        """Load LLM weights with a tiny text-only request."""
        payload = {
            "model": self.model,
            "prompt": "ok",
            "stream": False,
            "options": {
                "num_predict": 1,
                "num_ctx": 256,
            },
        }
        t0 = time.time()
        print("[QwenOllama] warmup text start...", flush=True)
        resp = requests.post(
            self.url,
            json=payload,
            timeout=timeout or self.timeout,
        )
        resp.raise_for_status()
        dt = time.time() - t0
        print(f"[QwenOllama] warmup text done in {dt:.1f}s", flush=True)
        return dt

    def warmup_vision(self, timeout: Optional[float] = None) -> float:
        """Preload vision encoder with a minimal image request."""
        import numpy as np

        blank = np.zeros((128, 128, 3), dtype=np.uint8)
        img64, model_w, model_h, _, _ = self._frame_to_base64(blank)
        payload = {
            "model": self.model,
            "prompt": (
                'Return JSON only: {"status":"searching","target_visible":false,'
                '"u":null,"v":null,"confidence":0.0,"stop":false}'
            ),
            "images": [img64],
            "stream": False,
            "format": "json",
            "options": {
                "temperature": 0,
                "num_predict": 16,
                "num_ctx": 512,
            },
        }
        t0 = time.time()
        print(
            f"[QwenOllama] warmup vision start image={model_w}x{model_h}...",
            flush=True,
        )
        resp = requests.post(
            self.url,
            json=payload,
            timeout=timeout or self.timeout,
        )
        resp.raise_for_status()
        dt = time.time() - t0
        print(f"[QwenOllama] warmup vision done in {dt:.1f}s", flush=True)
        return dt

    def warmup_full(self, timeout: Optional[float] = None) -> float:
        """Text + vision warmup so the first real infer skips cold-start."""
        t0 = time.time()
        self.warmup(timeout=timeout)
        self.warmup_vision(timeout=timeout)
        dt = time.time() - t0
        print(f"[QwenOllama] warmup_full total {dt:.1f}s", flush=True)
        return dt

    def infer_navigation(self, frame_bgr, instruction: str) -> Dict[str, Any]:
        img64, model_w, model_h, sx, sy = self._frame_to_base64(frame_bgr)
        orig_h, orig_w = frame_bgr.shape[:2]
        prompt = self._build_prompt(instruction, orig_w, orig_h)

        payload = {
            "model": self.model,
            "prompt": prompt,
            "images": [img64],
            "stream": False,
            "format": "json",
            "options": {
                "temperature": 0,
                "num_predict": self.num_predict,
                "num_ctx": self.num_ctx,
            },
        }

        t0 = time.time()
        print(
            f"[QwenOllama] infer start model={self.model} "
            f"image={model_w}x{model_h} orig={orig_w}x{orig_h} timeout={self.timeout:.0f}s",
            flush=True,
        )
        try:
            resp = requests.post(self.url, json=payload, timeout=self.timeout)
        except requests.exceptions.ReadTimeout as exc:
            raise RuntimeError(
                f"Ollama vision timed out after {self.timeout:.0f}s. "
                "On RDK X5, qwen2.5vl first infer can take several minutes. "
                "Retry with --timeout 900 --prep. "
                "If stuck at 'encoding image slice', run bash scripts/ollama_recover.sh"
            ) from exc
        except requests.exceptions.ConnectionError as exc:
            raise RuntimeError(
                "Ollama connection dropped mid-request. "
                "Most likely llama-server was killed by OOM on this 7GB board. "
                "Run: sudo bash scripts/setup_ollama_memory.sh && "
                "bash scripts/ollama_prep_infer.sh qwen2.5vl:3b"
            ) from exc
        dt = time.time() - t0
        resp.raise_for_status()

        raw_text = resp.json().get("response", "")
        raw_json = self._extract_json(raw_text)
        result = parse_nav_result(raw_json, orig_w, orig_h, model_w, model_h, sx, sy)

        result["_raw_text"] = raw_text
        result["_raw_json"] = raw_json
        result["_latency_sec"] = dt
        result["_model_image_width"] = model_w
        result["_model_image_height"] = model_h
        result["_orig_image_width"] = orig_w
        result["_orig_image_height"] = orig_h
        result["_scale_x_to_orig"] = sx
        result["_scale_y_to_orig"] = sy

        return result
