#!/usr/bin/env python3
import base64
import json
import logging
import os
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


def _scalar_coord(value: Any) -> Optional[float]:
    """
    Normalize one coordinate field from model JSON.
    Scalar -> as-is; list/tuple -> center (mean of numeric elements).
    """
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        nums = [_safe_float(x) for x in value]
        nums = [n for n in nums if n is not None]
        if not nums:
            return None
        return sum(nums) / len(nums)
    return _safe_float(value)


def _extract_uv_fields(raw: Dict[str, Any]) -> Tuple[Optional[float], Optional[float]]:
    """Read u/v from model JSON; fall back to x/y or point list for compatibility."""
    u = _scalar_coord(raw.get("u"))
    v = _scalar_coord(raw.get("v"))
    if u is not None and v is not None:
        return u, v

    x = _scalar_coord(raw.get("x"))
    y = _scalar_coord(raw.get("y"))
    if x is not None and y is not None:
        return x, y

    p = raw.get("target_point") or raw.get("point")
    if isinstance(p, (list, tuple)) and len(p) >= 2:
        return _scalar_coord(p[0]), _scalar_coord(p[1])

    return None, None


def parse_nav_result(
    raw: Dict[str, Any],
    orig_w: int,
    orig_h: int,
    model_w: int,
    model_h: int,
    sx: float,
    sy: float,
    coord_mode: str = "norm1000",
    min_confidence: float = 0.0,
) -> Dict[str, Any]:
    """
    Parse Qwen JSON with explicit coord_mode mapping to original-image pixels.
    Model output is u/v only; null u or v means not visible.
    """
    raw_u, raw_v = _extract_uv_fields(raw)

    confidence = _safe_float(raw.get("confidence"))
    conf_ok = confidence is None or confidence >= min_confidence

    mapped_u: Optional[float] = None
    mapped_v: Optional[float] = None
    coord_invalid = False
    coord_reason = ""

    if raw_u is None or raw_v is None:
        coord_reason = "missing_point"
    elif not conf_ok:
        coord_invalid = True
        coord_reason = f"low_confidence:{confidence}"
    elif coord_mode == "norm1000":
        if not (0 <= raw_u <= 1000 and 0 <= raw_v <= 1000):
            coord_invalid = True
            coord_reason = f"norm1000_out_of_range:{raw_u},{raw_v}"
        else:
            mapped_u = raw_u / 1000.0 * (orig_w - 1)
            mapped_v = raw_v / 1000.0 * (orig_h - 1)

    elif coord_mode == "model":
        if not (0 <= raw_u < model_w and 0 <= raw_v < model_h):
            coord_invalid = True
            coord_reason = f"model_coord_out_of_range:{raw_u},{raw_v},model={model_w}x{model_h}"
        else:
            mapped_u = raw_u * sx
            mapped_v = raw_v * sy

    elif coord_mode == "original":
        if not (0 <= raw_u < orig_w and 0 <= raw_v < orig_h):
            coord_invalid = True
            coord_reason = f"orig_coord_out_of_range:{raw_u},{raw_v},orig={orig_w}x{orig_h}"
        else:
            mapped_u = raw_u
            mapped_v = raw_v

    else:
        coord_invalid = True
        coord_reason = f"unknown_coord_mode:{coord_mode}"

    if coord_invalid or coord_reason == "missing_point":
        mapped_u = None
        mapped_v = None

    point_valid = mapped_u is not None and mapped_v is not None
    usable = point_valid and not coord_invalid

    return {
        "u": mapped_u,
        "v": mapped_v,
        "cx": mapped_u,
        "usable": usable,
        "_point_valid": point_valid,
        "_raw_u": raw_u,
        "_raw_v": raw_v,
        "_coord_mode": coord_mode,
        "_coord_invalid": coord_invalid,
        "_coord_reason": coord_reason,
    }


def save_qwen_coord_debug(
    debug_dir: str,
    prefix: str,
    frame_bgr,
    model_bgr,
    result: Dict[str, Any],
    coord_mode: str,
) -> Dict[str, str]:
    """Save input / raw-point / mapped-point debug images."""
    os.makedirs(debug_dir, exist_ok=True)
    paths: Dict[str, str] = {}

    input_path = os.path.join(debug_dir, f"{prefix}_qwen_input.jpg")
    cv2.imwrite(input_path, model_bgr)
    paths["debug_qwen_input"] = input_path

    raw_u = result.get("_raw_u")
    raw_v = result.get("_raw_v")
    mh, mw = model_bgr.shape[:2]

    raw_vis = model_bgr.copy()
    if raw_u is not None and raw_v is not None:
        if coord_mode == "model":
            px, py = int(raw_u), int(raw_v)
        elif coord_mode == "norm1000":
            px = int(round(raw_u / 1000.0 * (mw - 1)))
            py = int(round(raw_v / 1000.0 * (mh - 1)))
        else:
            px = int(round(float(raw_u) * mw / max(1, result.get("_orig_image_width", mw) - 1)))
            py = int(round(float(raw_v) * mh / max(1, result.get("_orig_image_height", mh) - 1)))

        if 0 <= px < mw and 0 <= py < mh:
            cv2.circle(raw_vis, (px, py), 8, (0, 0, 255), 2)
        cv2.putText(
            raw_vis,
            f"raw({raw_u:.1f},{raw_v:.1f})",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )
    else:
        raw_json = result.get("_raw_json") or {}
        cv2.putText(
            raw_vis,
            f"no point: {json.dumps(raw_json, ensure_ascii=False)[:80]}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
        )

    raw_path = os.path.join(debug_dir, f"{prefix}_qwen_input_raw_point.jpg")
    cv2.imwrite(raw_path, raw_vis)
    paths["debug_qwen_input_raw_point"] = raw_path

    mapped = frame_bgr.copy()
    u = result.get("u")
    v = result.get("v")
    if u is not None and v is not None:
        cv2.circle(mapped, (int(u), int(v)), 18, (0, 0, 255), 4)
        cv2.putText(
            mapped,
            f"mapped({u:.1f},{v:.1f})",
            (int(u) + 20, max(30, int(v) - 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
        )
    elif result.get("_coord_reason"):
        cv2.putText(
            mapped,
            f"unusable: {result.get('_coord_reason')}",
            (12, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )

    mapped_path = os.path.join(debug_dir, f"{prefix}_orig_mapped_point.jpg")
    cv2.imwrite(mapped_path, mapped)
    paths["debug_orig_mapped_point"] = mapped_path

    return paths


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
    输入 BGR 图像 + 指令，输出 u/v 点伺服 JSON（原图像素系）。
    """

    def __init__(
        self,
        model: str = "qwen2.5vl:3b",
        url: str = "http://127.0.0.1:11434/api/generate",
        timeout: float = 900.0,
        resize_width: int = 192,
        jpeg_quality: int = 60,
        num_predict: int = 80,
        num_ctx: int = 2048,
        keep_alive: Any = "1h",
        coord_mode: str = "norm1000",
        debug_dir: Optional[str] = None,
        save_debug: bool = False,
        min_confidence: float = 0.0,
    ):
        self.model = model
        self.url = url
        self.timeout = float(timeout)
        self.resize_width = int(resize_width)
        self.jpeg_quality = int(jpeg_quality)
        self.num_predict = int(num_predict)
        self.num_ctx = int(num_ctx)
        self.keep_alive = _normalize_keep_alive(keep_alive)
        self.coord_mode = str(coord_mode)
        self.debug_dir = debug_dir
        self.save_debug = bool(save_debug)
        self.min_confidence = float(min_confidence)

    def _build_prompt(
        self,
        instruction: str,
        orig_w: int,
        orig_h: int,
        model_w: int,
        model_h: int,
    ) -> str:
        target = instruction.strip()

        if self.coord_mode == "norm1000":
            return (
                f"Target: {target}\n"
                "Return ONLY one JSON object. No markdown. No explanation.\n"
                "The JSON must contain exactly two fields: u and v.\n"
                "u and v are normalized coordinates from 0 to 1000.\n"
                "u=0 means the left edge of the image, u=1000 means the right edge.\n"
                "v=0 means the top edge of the image, v=1000 means the bottom edge.\n"
                "\n"
                "First decide whether the target is clearly visible.\n"
                "\n"
                "If the target is clearly visible:\n"
                "Locate the CENTER of the visible target object itself.\n"
                "Do NOT place the point on the floor, shadow, background, chair leg, table leg, wheel, label, strap, rope, handle, or nearby area.\n"
                "If the target is a bottle or cup, place the point at the center of the main bottle/cup body.\n"
                "For a bottle or cup, ignore the cap, strap, rope, handle, logo, and label when choosing the horizontal center.\n"
                "The u coordinate must be halfway between the left and right visible edges of the main object body.\n"
                "The v coordinate should be around the vertical center of the main visible object body.\n"
                "If multiple matching targets are visible, choose the clearest and largest matching target.\n"
                "\n"
                "Only if the target is NOT clearly visible:\n"
                "Return a conservative navigation waypoint, not an estimated target position.\n"
                "The waypoint must be on the safest visible free floor where the robot can move next.\n"
                "Do NOT guess where the hidden target is. First choose a safe path point.\n"
                "\n"
                "SEARCH WAYPOINT RULES:\n"
                "1. Prefer the centerline of the largest connected visible free-floor region.\n"
                "2. If the forward center floor is open, choose a point near the horizontal center of the image.\n"
                "3. In a corridor, hallway, aisle, or open passage, choose the center of the passage on the floor, not the left or right side.\n"
                "4. The search waypoint should usually have u between 400 and 600 when the central path is open.\n"
                "5. Only choose u < 350 or u > 650 if the center path is physically blocked by an obstacle.\n"
                "6. The waypoint should be far enough ahead to guide motion, but not too close to the bottom edge.\n"
                "7. Prefer v between 550 and 700 for normal indoor floor navigation.\n"
                "8. Avoid v > 750 unless there is no farther safe floor visible.\n"
                "9. Avoid v < 450 unless the visible free floor is very short or blocked.\n"
                "\n"
                "DO NOT place the search waypoint on:\n"
                "- walls, doors, cabinets, furniture, chair legs, table legs, wheels, object bodies, shadows, reflections, image borders, or obstacles.\n"
                "- the left/right edge of the floor when the middle floor is open.\n"
                "- a nearby floor patch at the bottom of the image if a safer central floor point exists farther ahead.\n"
                "\n"
                "For target-not-visible cases, safety and path center are more important than semantic guessing.\n"
                "If there is a clear open corridor or floor ahead, output a centered floor waypoint such as the middle of that open path.\n"
                "If no safe visible free-floor waypoint exists, return null values.\n"
                "\n"
                "Priority rule:\n"
                "If any matching target is clearly visible, ALWAYS output the target center and ignore all search waypoint rules.\n"
                "Use the search waypoint rules ONLY when no matching target is clearly visible.\n"
                "When using search waypoint rules, prefer a centered safe floor point over a semantically guessed side point.\n"
                "\n"
                'Target visible output: {"u":500,"v":500}\n'
                'Target not visible and no safe waypoint output: {"u":null,"v":null}\n'
            )

        if self.coord_mode == "model":
            return (
                f"Target: {target}\n"
                f"The image you see is {model_w}x{model_h} pixels.\n"
                "Return ONLY one JSON object. No markdown. No explanation.\n"
                "Use pixel coordinates in the image you see.\n"
                f"u must be 0 to {model_w - 1}; v must be 0 to {model_h - 1}.\n"
                'If visible: {"u":100,"v":100}\n'
                'If not visible: {"u":null,"v":null}\n'
            )

        if self.coord_mode == "original":
            return (
                f"Target: {target}\n"
                f"Original image size is {orig_w}x{orig_h}.\n"
                "Return ONLY one JSON object. No markdown. No explanation.\n"
                "Use pixel coordinates in the original image.\n"
                f"u must be 0 to {orig_w - 1}; v must be 0 to {orig_h - 1}.\n"
                'If visible: {"u":100,"v":100}\n'
                'If not visible: {"u":null,"v":null}\n'
            )

        raise ValueError(f"unknown coord_mode: {self.coord_mode}")

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

    def _frame_to_base64(self, frame_bgr) -> Tuple[str, Any, int, int, float, float]:
        resized, sx, sy = self._resize_frame(frame_bgr)
        ok, buf = cv2.imencode(
            ".jpg",
            resized,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("cv2.imencode failed")

        img64 = base64.b64encode(buf.tobytes()).decode("utf-8")
        model_h, model_w = resized.shape[:2]
        return img64, resized, model_w, model_h, sx, sy

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
                "num_ctx": self.num_ctx,
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

    def warmup_vision(self, frame_bgr=None, timeout: Optional[float] = None) -> float:
        """Preload vision encoder using the same resize/ctx path as infer_navigation."""
        import numpy as np

        if frame_bgr is None:
            # 4:3 placeholder matching typical USB camera aspect.
            blank_h = max(8, int(round(self.resize_width * 0.75)))
            frame_bgr = np.zeros((blank_h, self.resize_width, 3), dtype=np.uint8)

        img_b64, _, model_w, model_h, _, _ = self._frame_to_base64(frame_bgr)
        payload = {
            "model": self.model,
            "prompt": 'Return JSON only: {"u":null,"v":null}',
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
            f"[QwenOllama] warmup vision start image={model_w}x{model_h} "
            f"num_ctx={self.num_ctx} num_predict={self.num_predict}...",
            flush=True,
        )
        data = self._post_generate(payload, timeout=timeout)
        dt = time.time() - t0
        load_ms = data.get("load_duration", 0) / 1e6
        prompt_eval_ms = data.get("prompt_eval_duration", 0) / 1e6
        print(
            f"[QwenOllama] warmup vision done in {dt:.1f}s "
            f"(ollama_load_ms={load_ms:.0f} prompt_eval_ms={prompt_eval_ms:.0f})",
            flush=True,
        )
        return dt

    def warmup_full(self, frame_bgr=None, timeout: Optional[float] = None) -> float:
        """Text + vision warmup so the first real infer skips cold-start."""
        t0 = time.time()
        self.warmup(timeout=timeout)
        self.warmup_vision(frame_bgr=frame_bgr, timeout=timeout)
        dt = time.time() - t0
        print(f"[QwenOllama] warmup_full total {dt:.1f}s", flush=True)
        return dt

    def warmup_on_camera_frame(self, frame_bgr, timeout: Optional[float] = None) -> float:
        """
        Warm up under the same memory layout as live navigation (camera already running).
        Uses the actual camera frame and the same resize/ctx settings as infer_navigation.
        """
        return self.warmup_full(frame_bgr=frame_bgr, timeout=timeout)

    def infer_navigation(self, frame_bgr, instruction: str) -> Dict[str, Any]:
        img_b64, model_bgr, model_w, model_h, sx, sy = self._frame_to_base64(frame_bgr)
        orig_h, orig_w = frame_bgr.shape[:2]
        prompt = self._build_prompt(instruction, orig_w, orig_h, model_w, model_h)

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
            f"coord_mode={self.coord_mode} "
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
        result = parse_nav_result(
            raw_json,
            orig_w,
            orig_h,
            model_w,
            model_h,
            sx,
            sy,
            coord_mode=self.coord_mode,
            min_confidence=self.min_confidence,
        )

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

        if self.save_debug and self.debug_dir:
            prefix = f"qwen_{int(time.time() * 1000)}"
            debug_paths = save_qwen_coord_debug(
                self.debug_dir,
                prefix,
                frame_bgr,
                model_bgr,
                result,
                self.coord_mode,
            )
            result.update(debug_paths)

        print(
            f"[QwenOllama] infer done latency={dt:.1f}s "
            f"usable={result.get('usable')} "
            f"u={result.get('u')} v={result.get('v')} "
            f"raw=({result.get('_raw_u')},{result.get('_raw_v')}) "
            f"ollama_total_ms={result['_ollama_total_ms']:.0f} "
            f"load_ms={result['_ollama_load_ms']:.0f} "
            f"prompt_eval_ms={result['_ollama_prompt_eval_ms']:.0f} "
            f"eval_ms={result['_ollama_eval_ms']:.0f}",
            flush=True,
        )

        return result
