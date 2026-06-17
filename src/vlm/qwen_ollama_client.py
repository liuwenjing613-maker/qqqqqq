#!/usr/bin/env python3
import base64
import json
import re
import subprocess
import time
from typing import Any, Dict, Optional, Tuple

import cv2
import requests


class QwenOllamaClient:
    """
    本地 Ollama Qwen2.5-VL 客户端。
    输入 BGR 图像 + 指令，输出严格 JSON。
    """

    def __init__(
        self,
        model: str = "moondream:latest",
        url: str = "http://127.0.0.1:11434/api/generate",
        timeout: float = 600.0,
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
        """Load model into memory with a tiny text-only request."""
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
        resp = requests.post(
            self.url,
            json=payload,
            timeout=timeout or self.timeout,
        )
        resp.raise_for_status()
        return time.time() - t0

    def _moondream_to_nav_json(self, text: str, instruction: str) -> Dict[str, Any]:
        text_l = (text or "").lower()
        target_words = [w.strip() for w in re.split(r"[,/\s]+", instruction.lower()) if w.strip()]
        if not target_words:
            target_words = ["bottle"]

        visible = any(w in text_l for w in target_words if len(w) > 2)
        if not visible and any(w in text_l for w in ("bottle", "cup", "container")):
            visible = "bottle" in instruction.lower() or "cup" in instruction.lower()

        if visible:
            return {
                "status": "target_locked",
                "target_visible": True,
                "target_category": "bottle" if "bottle" in instruction.lower() else "unknown",
                "target_description": text.strip(),
                "target_bbox": None,
                "target_point": None,
                "confidence": 0.6,
                "search_direction": "unknown",
                "action_hint": "approach",
                "stop": False,
                "reason": "moondream text detection",
            }

        return {
            "status": "searching",
            "target_visible": False,
            "target_category": "unknown",
            "target_description": text.strip() or "target not described",
            "target_bbox": None,
            "target_point": None,
            "confidence": 0.2,
            "search_direction": "unknown",
            "action_hint": "search",
            "stop": False,
            "reason": "moondream did not mention target",
        }

    def infer_navigation(self, frame_bgr, instruction: str) -> Dict[str, Any]:
        img64, model_w, model_h, sx, sy = self._frame_to_base64(frame_bgr)
        orig_h, orig_w = frame_bgr.shape[:2]
        is_moondream = "moondream" in self.model.lower()

        if is_moondream:
            prompt = (
                f"Instruction: {instruction}. "
                "Describe what you see and whether the target is visible."
            )
            num_predict = max(self.num_predict, 40)
        else:
            prompt = (
                f"Find: {instruction}. Image {model_w}x{model_h}. "
                'JSON only: {"status":"target_locked|searching",'
                '"target_visible":true,"target_bbox":[x1,y1,x2,y2],'
                '"target_point":[cx,cy],"confidence":0.0,"stop":false}'
            )
            num_predict = self.num_predict

        payload = {
            "model": self.model,
            "prompt": prompt,
            "images": [img64],
            "stream": False,
            "options": {
                "temperature": 0.1 if is_moondream else 0,
                "num_predict": num_predict,
                "num_ctx": self.num_ctx,
            },
        }

        t0 = time.time()
        print(
            f"[QwenOllama] infer start model={self.model} "
            f"image={model_w}x{model_h} timeout={self.timeout:.0f}s",
            flush=True,
        )
        try:
            resp = requests.post(self.url, json=payload, timeout=self.timeout)
        except requests.exceptions.ReadTimeout as exc:
            raise RuntimeError(
                f"Ollama vision timed out after {self.timeout:.0f}s. "
                "On RDK X5, first moondream inference can take 2-4 min. "
                "Retry with --timeout 900. "
                "If qwen2.5vl hangs at 'encoding image slice', run "
                "bash scripts/ollama_recover.sh"
            ) from exc
        except requests.exceptions.ConnectionError as exc:
            raise RuntimeError(
                "Ollama connection dropped mid-request. "
                "Most likely llama-server was killed by OOM on this 7GB board. "
                "Run: sudo bash scripts/setup_ollama_memory.sh && "
                "bash scripts/ollama_prep_infer.sh moondream:latest"
            ) from exc
        dt = time.time() - t0
        resp.raise_for_status()

        raw_text = resp.json().get("response", "")
        if is_moondream:
            result = self._moondream_to_nav_json(raw_text, instruction)
        else:
            result = self._extract_json(raw_text)

        result["_raw_text"] = raw_text
        result["_latency_sec"] = dt
        result["_model_image_width"] = model_w
        result["_model_image_height"] = model_h
        result["_orig_image_width"] = orig_w
        result["_orig_image_height"] = orig_h
        result["_scale_x_to_orig"] = sx
        result["_scale_y_to_orig"] = sy

        return result