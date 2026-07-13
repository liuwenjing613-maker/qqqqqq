from __future__ import annotations

import base64
import json
import os
import re
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import cv2

from .types import ModelResult, PixelPoint, PromptMode


_ROLE_BY_RESULT = {
    "TARGET_VISIBLE": "target",
    "TARGET_INFERRED": "search",
    "VERIFY_SUCCESS": "verify",
    "VERIFY_FAILED": "search",
}

_STATUS_TO_RESULT = {
    "V": "TARGET_VISIBLE",
    "I": "TARGET_INFERRED",
    "S": "VERIFY_SUCCESS",
    "F": "VERIFY_FAILED",
}

_ALLOWED_STATUS_BY_MODE = {
    PromptMode.OBSERVE: {"V", "I"},
    PromptMode.TRACK: {"V", "I"},
    PromptMode.SEARCH: {"V", "I"},
    PromptMode.VERIFY: {"S", "F"},
}


@dataclass
class ClientConfig:
    model: str = "qwen3-vl-flash"
    api_key_env: str = "DASHSCOPE_API_KEY"
    base_url_env: str = "QWEN_BASE_URL"
    default_base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1"
    timeout_sec: float = 20.0
    max_retries: int = 0
    temperature: float = 0.0
    max_tokens: int = 96
    enable_thinking: bool = False
    jpeg_quality: int = 72
    min_pixels: int = 65536
    max_pixels: int = 442368
    vl_high_resolution_images: bool = False


class QwenVisionClient:
    def __init__(self, config: ClientConfig):
        self.config = config
        api_key = os.getenv(config.api_key_env, "").strip()
        if not api_key:
            raise RuntimeError(f"Environment variable {config.api_key_env} is empty")
        base_url = os.getenv(config.base_url_env, "").strip() or config.default_base_url
        model_override = os.getenv("QWEN_MODEL", "").strip()
        if model_override:
            self.config.model = model_override
        try:
            from openai import OpenAI
        except ImportError as exc:
            raise RuntimeError("Missing dependency: pip install openai") from exc
        self._client = OpenAI(
            api_key=api_key,
            base_url=base_url,
            timeout=config.timeout_sec,
            max_retries=config.max_retries,
        )

    def infer(self, image_bgr, prompt: str, mode: PromptMode, request_id: int = 0) -> ModelResult:
        height, width = image_bgr.shape[:2]

        encode_started = time.perf_counter()
        ok, encoded = cv2.imencode(
            ".jpg",
            image_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.jpeg_quality)],
        )
        encode_ms = (time.perf_counter() - encode_started) * 1000.0
        if not ok:
            raise RuntimeError("Failed to encode image as JPEG")

        jpeg_kb = len(encoded) / 1024.0
        data_url = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
        # Keep the image payload minimal. Attaching min_pixels/max_pixels on the
        # image item can trigger server-side smart-resize that no longer matches
        # the exact JPEG we encode and draw on, which reintroduces point offset.
        image_item: Dict[str, Any] = {
            "type": "image_url",
            "image_url": {"url": data_url},
        }

        api_started = time.perf_counter()
        completion = self._client.chat.completions.create(
            model=self.config.model,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        image_item,
                    ],
                }
            ],
            response_format={"type": "json_object"},
            temperature=float(self.config.temperature),
            max_tokens=int(self.config.max_tokens),
            extra_body={
                "enable_thinking": bool(self.config.enable_thinking),
                "vl_high_resolution_images": bool(self.config.vl_high_resolution_images),
            },
        )
        api_ms = (time.perf_counter() - api_started) * 1000.0

        usage = getattr(completion, "usage", None)
        prompt_tokens = getattr(usage, "prompt_tokens", None)
        completion_tokens = getattr(usage, "completion_tokens", None)
        cached_tokens = None
        details = getattr(usage, "prompt_tokens_details", None)
        if details is not None:
            cached_tokens = getattr(details, "cached_tokens", None)

        raw_content = completion.choices[0].message.content
        raw_text = raw_content if isinstance(raw_content, str) else json.dumps(raw_content, ensure_ascii=False)
        try:
            result = parse_model_output(raw_text or "", mode, width, height)
        except Exception as exc:
            preview = (raw_text or "").replace("\n", " ")[:600]
            raise ValueError(f"{exc}; raw_output={preview!r}") from exc

        raw_point = "null"
        try:
            raw_json = _extract_json(raw_text or "")
            raw_point = repr(raw_json.get("p", raw_json.get("point")))
        except Exception:  # noqa: BLE001
            pass
        pixel_point = (
            "null"
            if result.point is None
            else f"[{result.point.x},{result.point.y}]"
        )
        print(
            f"[QWEN_PROFILE] "
            f"req={request_id} mode={mode.value} "
            f"shape={width}x{height} "
            f"jpeg_kb={jpeg_kb:.1f} "
            f"encode_ms={encode_ms:.1f} "
            f"api_ms={api_ms:.1f} "
            f"raw_p={raw_point} "
            f"pixel_p={pixel_point} "
            f"prompt_tokens={prompt_tokens} "
            f"completion_tokens={completion_tokens} "
            f"cached_tokens={cached_tokens}",
            flush=True,
        )

        result.raw_text = raw_text or ""
        # Keep ModelResult.latency_ms as the pure API wait (exclude local JPEG encode).
        result.latency_ms = api_ms
        result.request_id = request_id
        result.image_width = width
        result.image_height = height
        return result

    def warmup(self) -> float:
        """Run one tiny vision request before the real task.

        Call once after Client construction on node startup. Failures must not
        abort node startup; callers should catch and log warnings.
        """
        import numpy as np

        dummy = np.full((256, 256, 3), 127, dtype=np.uint8)
        ok, encoded = cv2.imencode(
            ".jpg",
            dummy,
            [int(cv2.IMWRITE_JPEG_QUALITY), 60],
        )
        if not ok:
            raise RuntimeError("Failed to encode warmup image")

        data_url = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
        started = time.perf_counter()
        self._client.chat.completions.create(
            model=self.config.model,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": 'Return exactly this JSON: {"ok":true}',
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": data_url},
                            "min_pixels": 65536,
                            "max_pixels": 65536,
                        },
                    ],
                }
            ],
            response_format={"type": "json_object"},
            temperature=0.0,
            max_tokens=16,
            extra_body={
                "enable_thinking": False,
                "vl_high_resolution_images": False,
            },
        )
        return (time.perf_counter() - started) * 1000.0


def _extract_json(text: str) -> Dict[str, Any]:
    cleaned = (text or "").strip()
    if not cleaned:
        raise ValueError("Model returned empty content")
    try:
        value = json.loads(cleaned)
        if not isinstance(value, dict):
            raise ValueError("Top-level JSON must be an object")
        return value
    except json.JSONDecodeError:
        # Prefer a complete {...} span when present.
        match = re.search(r"\{.*\}", cleaned, flags=re.DOTALL)
        if match:
            value = json.loads(match.group(0))
            if not isinstance(value, dict):
                raise ValueError("Top-level JSON must be an object")
            return value
        # Compact protocol recovery for truncated tails such as:
        # {"s":"V","p":[450, 430]
        repaired = _repair_compact_json(cleaned)
        if repaired is not None:
            return repaired
        raise ValueError(f"No JSON object found in model output: {cleaned[:300]}")


_COMPACT_RE = re.compile(
    r'"s"\s*:\s*"([VvIiSsFf])"'
    r'.*?'
    r'"p"\s*:\s*\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\]',
    flags=re.DOTALL,
)


def _repair_compact_json(text: str) -> Optional[Dict[str, Any]]:
    """Recover compact {"s","p"} even when the model truncates the closing brace."""
    match = _COMPACT_RE.search(text or "")
    if not match:
        return None
    status, x_raw, y_raw = match.groups()
    return {
        "s": status.upper(),
        "p": [float(x_raw), float(y_raw)],
    }


_NORM1000_MAX = 1000.0


def _looks_like_unit_interval(x: float, y: float) -> bool:
    """Reject 0-1 normalized coords that would collapse under norm1000 mapping."""
    if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
        return False
    # Exact 0/1 integers are valid edge cells on the 0-1000 grid.
    return not (x in (0.0, 1.0) and y in (0.0, 1.0) and float(x).is_integer() and float(y).is_integer())


def _norm1000_to_pixel(coord: float, size: int) -> int:
    """Map Qwen3-VL relative coord in [0, 1000] onto pixel index in [0, size-1]."""
    if size <= 0:
        raise ValueError(f"image size must be positive, got {size}")
    pixel = int(round(coord / _NORM1000_MAX * (size - 1)))
    return min(max(pixel, 0), size - 1)


def pixel_to_norm1000(pixel: int, size: int) -> int:
    """Inverse map for feeding previous points back into prompts."""
    if size <= 1:
        return 0
    value = int(round(float(pixel) / float(size - 1) * _NORM1000_MAX))
    return min(max(value, 0), int(_NORM1000_MAX))


def _parse_point(value: Any, width: int, height: int) -> Optional[PixelPoint]:
    """Parse model point as Qwen3-VL 0-1000 relative coords, then convert to pixels."""
    if value is None:
        return None
    if isinstance(value, dict):
        x_raw, y_raw = value.get("x"), value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        x_raw, y_raw = value
    else:
        raise ValueError("point must be null, {x,y}, or [x,y]")
    if isinstance(x_raw, bool) or isinstance(y_raw, bool):
        raise ValueError("point coordinates cannot be booleans")
    try:
        x_norm, y_norm = float(x_raw), float(y_raw)
    except (TypeError, ValueError) as exc:
        raise ValueError("point coordinates must be numeric") from exc
    if _looks_like_unit_interval(x_norm, y_norm):
        raise ValueError(
            f"point ({x_norm}, {y_norm}) looks like normalized 0-1 coordinates; "
            "expected Qwen3-VL relative coordinates in [0, 1000]"
        )
    if not (0.0 <= x_norm <= _NORM1000_MAX and 0.0 <= y_norm <= _NORM1000_MAX):
        raise ValueError(
            f"point ({x_norm}, {y_norm}) is outside the Qwen3-VL relative grid [0, 1000]"
        )
    return PixelPoint(
        x=_norm1000_to_pixel(x_norm, width),
        y=_norm1000_to_pixel(y_norm, height),
    )


def parse_model_output(raw_text: str, mode: PromptMode, image_width: int, image_height: int) -> ModelResult:
    """Parse compact realtime JSON: {"s":"V|I|S|F","p":[x,y]}."""
    data = _extract_json(raw_text)
    required = {"s", "p"}
    missing = sorted(required.difference(data))
    if missing:
        raise ValueError(f"Missing required JSON fields: {missing}")

    status = str(data.get("s", "")).strip().upper()
    if status not in _STATUS_TO_RESULT:
        raise ValueError(f"Unknown status code s={status!r}; allowed=V|I|S|F")
    if status not in _ALLOWED_STATUS_BY_MODE[mode]:
        raise ValueError(
            f"Status s={status!r} is not allowed in mode {mode.value}; "
            f"allowed={sorted(_ALLOWED_STATUS_BY_MODE[mode])}"
        )

    result = _STATUS_TO_RESULT[status]
    point = _parse_point(data.get("p"), image_width, image_height)
    if point is None:
        raise ValueError(f"s={status} requires point p=[x,y]")

    role = _ROLE_BY_RESULT[result]
    return ModelResult(
        result=result,
        point=point,
        point_role=role,
        label="",
        reason_code=f"compact_{status.lower()}",
    )
