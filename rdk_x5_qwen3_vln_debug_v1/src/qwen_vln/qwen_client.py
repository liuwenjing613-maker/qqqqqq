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


_ALLOWED_BY_MODE = {
    PromptMode.OBSERVE: {"TARGET_VISIBLE", "TARGET_NOT_VISIBLE"},
    PromptMode.TRACK: {"TARGET_VISIBLE", "TARGET_NOT_VISIBLE"},
    PromptMode.SEARCH: {"TARGET_VISIBLE", "SEARCH_HINT", "SEARCH_NO_HINT"},
    PromptMode.VERIFY: {"VERIFY_SUCCESS", "VERIFY_FAILED"},
}
_POINT_REQUIRED = {"TARGET_VISIBLE", "SEARCH_HINT", "VERIFY_SUCCESS"}
_POINT_FORBIDDEN = {"TARGET_NOT_VISIBLE", "SEARCH_NO_HINT", "VERIFY_FAILED"}
_ROLE_BY_RESULT = {
    "TARGET_VISIBLE": "target",
    "TARGET_NOT_VISIBLE": "none",
    "SEARCH_HINT": "search",
    "SEARCH_NO_HINT": "none",
    "VERIFY_SUCCESS": "verify",
    "VERIFY_FAILED": "none",
}


@dataclass
class ClientConfig:
    model: str = "qwen3-vl-flash"
    api_key_env: str = "DASHSCOPE_API_KEY"
    base_url_env: str = "QWEN_BASE_URL"
    default_base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1"
    timeout_sec: float = 35.0
    max_retries: int = 1
    temperature: float = 0.1
    max_tokens: int = 420
    enable_thinking: bool = False
    jpeg_quality: int = 88


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
        ok, encoded = cv2.imencode(
            ".jpg",
            image_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.jpeg_quality)],
        )
        if not ok:
            raise RuntimeError("Failed to encode image as JPEG")
        data_url = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
        started = time.perf_counter()
        completion = self._client.chat.completions.create(
            model=self.config.model,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": data_url}},
                        {"type": "text", "text": prompt},
                    ],
                }
            ],
            response_format={"type": "json_object"},
            temperature=self.config.temperature,
            max_tokens=self.config.max_tokens,
            extra_body={"enable_thinking": bool(self.config.enable_thinking)},
        )
        latency_ms = (time.perf_counter() - started) * 1000.0
        raw_content = completion.choices[0].message.content
        raw_text = raw_content if isinstance(raw_content, str) else json.dumps(raw_content, ensure_ascii=False)
        try:
            result = parse_model_output(raw_text or "", mode, width, height)
        except Exception as exc:
            preview = (raw_text or "").replace("\n", " ")[:600]
            raise ValueError(f"{exc}; raw_output={preview!r}") from exc
        result.raw_text = raw_text or ""
        result.latency_ms = latency_ms
        result.request_id = request_id
        result.image_width = width
        result.image_height = height
        return result


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
        match = re.search(r"\{.*\}", cleaned, flags=re.DOTALL)
        if not match:
            raise ValueError(f"No JSON object found in model output: {cleaned[:300]}")
        value = json.loads(match.group(0))
        if not isinstance(value, dict):
            raise ValueError("Top-level JSON must be an object")
        return value


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
    data = _extract_json(raw_text)
    required = {"result", "point", "point_role", "confidence", "label", "reason_code"}
    missing = sorted(required.difference(data))
    if missing:
        raise ValueError(f"Missing required JSON fields: {missing}")
    result = str(data.get("result", "")).strip().upper()
    if result not in _ALLOWED_BY_MODE[mode]:
        raise ValueError(
            f"Result {result!r} is not allowed in mode {mode.value}; "
            f"allowed={sorted(_ALLOWED_BY_MODE[mode])}"
        )
    point = _parse_point(data.get("point"), image_width, image_height)
    if result in _POINT_REQUIRED and point is None:
        raise ValueError(f"{result} requires a pixel point")
    if result in _POINT_FORBIDDEN and point is not None:
        raise ValueError(f"{result} requires point=null")
    expected_role = _ROLE_BY_RESULT[result]
    role = str(data.get("point_role", expected_role)).strip().lower()
    if role != expected_role:
        raise ValueError(
            f"point_role={role!r} conflicts with result={result}; expected {expected_role!r}"
        )
    if isinstance(data.get("confidence"), bool):
        raise ValueError("confidence cannot be a boolean")
    try:
        confidence = float(data["confidence"])
    except (TypeError, ValueError) as exc:
        raise ValueError("confidence must be numeric") from exc
    if not 0.0 <= confidence <= 1.0:
        raise ValueError(f"confidence must be within [0, 1], got {confidence}")
    reason_code = str(data["reason_code"]).strip().lower()
    if not reason_code:
        raise ValueError("reason_code must be a non-empty machine-readable string")
    return ModelResult(
        result=result,
        point=point,
        point_role=role,
        confidence=confidence,
        label=str(data.get("label", "")).strip(),
        reason_code=reason_code,
    )
