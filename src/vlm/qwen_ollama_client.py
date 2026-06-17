#!/usr/bin/env python3
import base64
import json
import logging
import math
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


def smart_resize(
    height: int,
    width: int,
    min_pixels: int = 512 * 28 * 28,
    max_pixels: int = 2048 * 28 * 28,
    factor: int = 28,
) -> Tuple[int, int]:
    """
    Qwen2.5-VL / Ollama vision preprocessor: snap to multiples of 28 and
    clamp total pixels to [min_pixels, max_pixels].
    Returns (height, width).
    """
    h, w = float(height), float(width)

    def round_by_factor(x: float) -> int:
        return int(round(x / factor) * factor)

    def ceil_by_factor(x: float) -> int:
        return int(math.ceil(x / factor) * factor)

    def floor_by_factor(x: float) -> int:
        return int(math.floor(x / factor) * factor)

    h_bar = max(factor, round_by_factor(h))
    w_bar = max(factor, round_by_factor(w))
    if h_bar * w_bar > max_pixels:
        beta = math.sqrt((height * width) / max_pixels)
        h_bar = floor_by_factor(height / beta)
        w_bar = floor_by_factor(width / beta)
    elif h_bar * w_bar < min_pixels:
        beta = math.sqrt(min_pixels / (height * width))
        h_bar = ceil_by_factor(height * beta)
        w_bar = ceil_by_factor(width * beta)
    return h_bar, w_bar


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
    解析千问 JSON，只输出 u/v（原图像素系）。
    """
    u_raw = _safe_float(raw.get("u"))
    v_raw = _safe_float(raw.get("v"))

    u: Optional[float] = None
    v: Optional[float] = None
    coords_scaled = False
    coords_from_ollama_internal = False
    ollama_internal_h, ollama_internal_w = smart_resize(model_h, model_w)

    if u_raw is not None and v_raw is not None:
        if u_raw <= model_w and v_raw <= model_h and (sx > 1.01 or sy > 1.01):
            logger.warning(
                "u,v look like sent-jpeg coords (u=%.1f v=%.1f sent=%dx%d), scaling to orig",
                u_raw,
                v_raw,
                model_w,
                model_h,
            )
            u_raw *= sx
            v_raw *= sy
            coords_scaled = True
        elif (
            not coords_scaled
            and model_w < 256
            and u_raw <= ollama_internal_w
            and v_raw <= ollama_internal_h
            and (u_raw > model_w or v_raw > model_h)
        ):
            logger.warning(
                "u,v look like Ollama/Qwen internal coords "
                "(u=%.1f v=%.1f internal=%dx%d sent=%dx%d), scaling to orig",
                u_raw,
                v_raw,
                ollama_internal_w,
                ollama_internal_h,
                model_w,
                model_h,
            )
            u_raw = u_raw / ollama_internal_w * orig_w
            v_raw = v_raw / ollama_internal_h * orig_h
            coords_scaled = True
            coords_from_ollama_internal = True

        u = _clamp(u_raw, 0, max(0, orig_w - 1))
        v = _clamp(v_raw, 0, max(0, orig_h - 1))

    point_valid = u is not None and v is not None

    return {
        "u": u,
        "v": v,
        "cx": u,
        "_point_valid": point_valid,
        "_coords_scaled_from_model": coords_scaled,
        "_coords_scaled_from_ollama_internal": coords_from_ollama_internal,
        "_ollama_internal_width": ollama_internal_w,
        "_ollama_internal_height": ollama_internal_h,
    }

    # --- 旧版：含 status / target_visible / confidence / stop ---
    # target_visible = bool(raw.get("target_visible", False))
    # confidence = _safe_float(raw.get("confidence")) or 0.0
    # stop = bool(raw.get("stop", False))
    # status = _normalize_status(raw.get("status", "searching"), target_visible, point_valid, stop)
    # return {
    #     "status": status,
    #     "target_visible": target_visible and point_valid,
    #     "u": u, "v": v, "cx": u,
    #     "confidence": confidence,
    #     "stop": stop,
    #     ...
    # }


def _normalize_keep_alive(value: Any) -> Any:
    """Ollama rejects string '-1'; use integer -1 for keep loaded indefinitely."""
    if value is None:
        return "30m"
    if isinstance(value, str) and value.strip() in ("-1", "inf", "infinite", "forever"):
        return -1
    if isinstance(value, int) and value == -1:
        return -1
    return value


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
        resize_width: int = 96,
        jpeg_quality: int = 60,
        num_predict: int = 80,
        num_ctx: int = 2048,
        keep_alive: Any = "1h",
    ):
        self.model = model
        self.url = url
        self.timeout = float(timeout)
        self.resize_width = int(resize_width)
        self.jpeg_quality = int(jpeg_quality)
        self.num_predict = int(num_predict)
        self.num_ctx = int(num_ctx)
        self.keep_alive = _normalize_keep_alive(keep_alive)

    def _build_prompt(self, instruction: str, orig_w: int, orig_h: int) -> str:
        return f"""
你是移动机器人视觉模块。只输出严格 JSON，不要 Markdown，不要解释。

用户目标：{instruction}

坐标系：
原始图像 width={orig_w}, height={orig_h}。
u/v 必须按这个原始图像像素坐标输出，原点在左上角。

输出格式（仅此两个字段）：
{{"u": 0, "v": 0}}

规则：
1. u 是目标中心横坐标，范围 0 到 {orig_w - 1}。
2. v 是目标中心纵坐标，范围 0 到 {orig_h - 1}。
3. 看得到目标时输出整数像素点；看不到时输出 {{"u": null, "v": null}}。
4. 不要输出 status、confidence、bbox 或其他字段。
""".strip()

        # --- 旧版 prompt（含 status/confidence/stop + 原图尺寸约束）---
        # return f"""
        # 你是移动机器人视觉导航模块。只输出严格 JSON，不要输出 Markdown，不要解释。
        #
        # 用户目标：{instruction}
        #
        # 原始图像尺寸：
        # width={orig_w}, height={orig_h}
        #
        # 输出 JSON 格式必须为：
        # {{
        #   "status": "target_locked/searching/success/unsafe",
        #   "target_visible": true,
        #   "u": 0,
        #   "v": 0,
        #   "confidence": 0.0,
        #   "stop": false
        # }}
        # ...
        # """.strip()

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

    @staticmethod
    def _attach_ollama_timing(result: Dict[str, Any], data: Dict[str, Any]) -> None:
        result["_ollama_total_ms"] = data.get("total_duration", 0) / 1e6
        result["_ollama_load_ms"] = data.get("load_duration", 0) / 1e6
        result["_ollama_prompt_eval_ms"] = data.get("prompt_eval_duration", 0) / 1e6
        result["_ollama_eval_ms"] = data.get("eval_duration", 0) / 1e6
        result["_ollama_eval_count"] = data.get("eval_count", 0)

    def _post_generate(self, payload: Dict[str, Any], timeout: Optional[float] = None) -> Dict[str, Any]:
        payload["keep_alive"] = _normalize_keep_alive(
            payload.get("keep_alive", self.keep_alive)
        )
        resp = requests.post(self.url, json=payload, timeout=timeout or self.timeout)
        if resp.status_code >= 400:
            detail = resp.text[:500]
            raise requests.HTTPError(
                f"{resp.status_code} Client Error for {self.url}: {detail}",
                response=resp,
            )
        return resp.json()

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
            "keep_alive": self.keep_alive,
            "options": {
                "num_predict": 1,
                "num_ctx": 256,
            },
        }
        t0 = time.time()
        print("[QwenOllama] warmup text start...", flush=True)
        data = self._post_generate(payload, timeout=timeout)
        dt = time.time() - t0
        load_ms = data.get("load_duration", 0) / 1e6
        print(
            f"[QwenOllama] warmup text done in {dt:.1f}s "
            f"(ollama_load_ms={load_ms:.0f})",
            flush=True,
        )
        return dt

    def warmup_vision(self, timeout: Optional[float] = None) -> float:
        """Preload vision encoder with a minimal image request."""
        import numpy as np

        blank = np.zeros((128, 128, 3), dtype=np.uint8)
        img_b64, model_w, model_h, _, _ = self._frame_to_base64(blank)
        '''payload = {
            "model": self.model,
            "prompt": (
                'Return JSON only: {"status":"searching","target_visible":false,'
                '"u":null,"v":null,"confidence":0.0,"stop":false}'
            ),
            "images": [img_b64],
            "stream": False,
            "format": "json",
            "keep_alive": self.keep_alive,
            "options": {
                "temperature": 0,
                "num_predict": 16,
                "num_ctx": 512,
            },
        }'''
        payload = {
            "model": self.model,
            "prompt": (
                'Return JSON only: {"u":null,"v":null}'
            ),
            "images": [img_b64],
            "stream": False,
            "format": "json",
            "keep_alive": self.keep_alive,
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
        data = self._post_generate(payload, timeout=timeout)
        dt = time.time() - t0
        load_ms = data.get("load_duration", 0) / 1e6
        print(
            f"[QwenOllama] warmup vision done in {dt:.1f}s "
            f"(ollama_load_ms={load_ms:.0f})",
            flush=True,
        )
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
        img_b64, model_w, model_h, sx, sy = self._frame_to_base64(frame_bgr)
        orig_h, orig_w = frame_bgr.shape[:2]
        prompt = self._build_prompt(instruction, orig_w, orig_h)

        payload = {
            "model": self.model,
            "prompt": prompt,
            "images": [img_b64],
            "stream": False,
            "format": "json",
            "keep_alive": self.keep_alive,
            "options": {
                "temperature": 0,
                "num_predict": self.num_predict,
                "num_ctx": self.num_ctx,
            },
        }

        t0 = time.time()
        print(
            f"[QwenOllama] infer start model={self.model} "
            f"image={model_w}x{model_h} orig={orig_w}x{orig_h} "
            f"keep_alive={self.keep_alive} timeout={self.timeout:.0f}s",
            flush=True,
        )
        try:
            data = self._post_generate(payload, timeout=self.timeout)
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

        raw_text = data.get("response", "")
        raw_json = self._extract_json(raw_text)
        result = parse_nav_result(raw_json, orig_w, orig_h, model_w, model_h, sx, sy)

        result["_raw_text"] = raw_text
        result["_raw_json"] = raw_json
        result["_latency_sec"] = dt
        self._attach_ollama_timing(result, data)
        result["_model_image_width"] = model_w
        result["_model_image_height"] = model_h
        result["_orig_image_width"] = orig_w
        result["_orig_image_height"] = orig_h
        result["_scale_x_to_orig"] = sx
        result["_scale_y_to_orig"] = sy

        print(
            f"[QwenOllama] infer done latency={dt:.1f}s "
            f"ollama_total_ms={result['_ollama_total_ms']:.0f} "
            f"load_ms={result['_ollama_load_ms']:.0f} "
            f"prompt_eval_ms={result['_ollama_prompt_eval_ms']:.0f} "
            f"eval_ms={result['_ollama_eval_ms']:.0f}",
            flush=True,
        )

        return result
