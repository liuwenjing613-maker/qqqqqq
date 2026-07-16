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
    PromptMode.SPAWN_SCAN: {"V", "I"},
    PromptMode.OBSERVE: {"V", "I"},
    PromptMode.TRACK: {"V", "I"},
    PromptMode.SEARCH: {"V", "I"},
    PromptMode.VERIFY: {"S", "F"},
}
_ACTIONS = {"POINT", "TURN_LEFT", "TURN_RIGHT", "STOP"}
_ALLOWED_ACTION_BY_STATUS = {
    "V": {"POINT"},
    # Target is not visible. Either follow a visible continuation, adjust view,
    # or accept legacy STOP output from older prompts.
    "I": {"POINT", "TURN_LEFT", "TURN_RIGHT", "STOP"},
    # New prompt asks S+POINT; STOP remains accepted for old-output
    # compatibility. The high-level FSM enters SUCCESS either way.
    "S": {"POINT", "STOP"},
    "F": {"POINT", "TURN_LEFT", "TURN_RIGHT", "STOP"},
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
    # SPAWN_SCAN {"p","t","r","q"} needs more tokens than action JSON.
    max_tokens_spawn_scan: int = 48
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

    def infer(
        self,
        image_bgr,
        prompt: str,
        mode: PromptMode,
        request_id: int = 0,
    ) -> ModelResult:
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
        data_url = "data:image/jpeg;base64," + base64.b64encode(
            encoded.tobytes()
        ).decode("ascii")
        # Freeze server-side smart_resize to this exact canvas: callers must
        # already smart_resize via image_prep.resize_for_api so draw == model.
        # min/max_pixels are siblings of image_url (DashScope OpenAI-compatible).
        pixel_count = int(width) * int(height)
        image_item: Dict[str, Any] = {
            "type": "image_url",
            "image_url": {"url": data_url},
            "min_pixels": pixel_count,
            "max_pixels": pixel_count,
        }

        max_tokens = (
            int(self.config.max_tokens_spawn_scan)
            if mode == PromptMode.SPAWN_SCAN
            else int(self.config.max_tokens)
        )
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
            max_tokens=max_tokens,
            extra_body={
                "enable_thinking": bool(self.config.enable_thinking),
                "vl_high_resolution_images": bool(
                    self.config.vl_high_resolution_images
                ),
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
        raw_text = (
            raw_content
            if isinstance(raw_content, str)
            else json.dumps(raw_content, ensure_ascii=False)
        )
        try:
            result = parse_model_output(raw_text or "", mode, width, height)
        except Exception as exc:
            preview = (raw_text or "").replace("\n", " ")[:600]
            raise ValueError(f"{exc}; raw_output={preview!r}") from exc

        raw_point = "null"
        raw_action = "?"
        try:
            raw_json = _extract_json(raw_text or "")
            raw_point = repr(raw_json.get("p", raw_json.get("point")))
            raw_action = str(raw_json.get("a", "legacy"))
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
            f"action={raw_action} raw_p={raw_point} pixel_p={pixel_point} "
            f"c={result.confidence:.0f} "
            f"prompt_tokens={prompt_tokens} "
            f"completion_tokens={completion_tokens} "
            f"cached_tokens={cached_tokens}",
            flush=True,
        )
        result.raw_text = raw_text or ""
        result.latency_ms = api_ms
        result.request_id = request_id
        result.image_width = width
        result.image_height = height
        return result

    def warmup(self) -> float:
        """Run one tiny vision request before the real task."""
        import numpy as np

        dummy = np.full((256, 256, 3), 127, dtype=np.uint8)
        ok, encoded = cv2.imencode(
            ".jpg", dummy, [int(cv2.IMWRITE_JPEG_QUALITY), 60]
        )
        if not ok:
            raise RuntimeError("Failed to encode warmup image")
        data_url = "data:image/jpeg;base64," + base64.b64encode(
            encoded.tobytes()
        ).decode("ascii")
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
        match = re.search(r"\{.*\}", cleaned, flags=re.DOTALL)
        if match:
            value = json.loads(match.group(0))
            if not isinstance(value, dict):
                raise ValueError("Top-level JSON must be an object")
            return value
        repaired = _repair_compact_json(cleaned)
        if repaired is not None:
            return repaired
        raise ValueError(f"No JSON object found in model output: {cleaned[:300]}")


_STATUS_RE = re.compile(r'"s"\s*:\s*"([VvIiSsFf])"')
_ACTION_RE = re.compile(
    r'"a"\s*:\s*"(POINT|TURN_LEFT|TURN_RIGHT|STOP)"', re.IGNORECASE
)
_POINT_RE = re.compile(
    r'"p"\s*:\s*\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\]',
    re.DOTALL,
)
_POINT_NULL_RE = re.compile(r'"p"\s*:\s*null', re.IGNORECASE)
_CONF_RE = re.compile(r'"c"\s*:\s*(-?\d+(?:\.\d+)?)')


def _repair_compact_json(text: str) -> Optional[Dict[str, Any]]:
    """Recover compact action JSON when only a closing brace was truncated."""
    status_match = _STATUS_RE.search(text or "")
    if not status_match:
        return None
    action_match = _ACTION_RE.search(text or "")
    point_match = _POINT_RE.search(text or "")
    point_is_null = bool(_POINT_NULL_RE.search(text or ""))
    if not point_match and not point_is_null:
        return None
    data: Dict[str, Any] = {"s": status_match.group(1).upper()}
    if action_match:
        data["a"] = action_match.group(1).upper()
    if point_match:
        data["p"] = [float(point_match.group(1)), float(point_match.group(2))]
    else:
        data["p"] = None
    confidence_match = _CONF_RE.search(text or "")
    if confidence_match:
        data["c"] = float(confidence_match.group(1))
    return data


_NORM1000_MAX = 1000.0


def _looks_like_unit_interval(x: float, y: float) -> bool:
    if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
        return False
    return not (
        x in (0.0, 1.0)
        and y in (0.0, 1.0)
        and float(x).is_integer()
        and float(y).is_integer()
    )


def _norm1000_to_pixel(coord: float, size: int) -> int:
    """Official Qwen3-VL mapping: pixel = round(coord / 1000 * size)."""
    if size <= 0:
        raise ValueError(f"image size must be positive, got {size}")
    pixel = int(round(coord / _NORM1000_MAX * float(size)))
    return min(max(pixel, 0), size - 1)


def pixel_to_norm1000(pixel: int, size: int) -> int:
    """Inverse of official Qwen3-VL mapping for previous-point feedback."""
    if size <= 0:
        return 0
    value = int(round(float(pixel) / float(size) * _NORM1000_MAX))
    return min(max(value, 0), int(_NORM1000_MAX))


def _parse_point(value: Any, width: int, height: int) -> Optional[PixelPoint]:
    if value is None:
        return None
    if isinstance(value, dict):
        x_raw, y_raw = value.get("x"), value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        x_raw, y_raw = value
    else:
        raise ValueError("p must be null, {x,y}, or [x,y]")
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
    if not (
        0.0 <= x_norm <= _NORM1000_MAX
        and 0.0 <= y_norm <= _NORM1000_MAX
    ):
        raise ValueError(
            f"point ({x_norm}, {y_norm}) is outside [0, 1000]"
        )
    return PixelPoint(
        x=_norm1000_to_pixel(x_norm, width),
        y=_norm1000_to_pixel(y_norm, height),
    )


def _parse_confidence(value: Any) -> float:
    # c is published for logging only. No controller gate uses it in V3.
    if value is None:
        return 0.0
    if isinstance(value, bool):
        raise ValueError("c cannot be boolean")
    try:
        confidence = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("c must be numeric in [0,100]") from exc
    # Truncation with max_tokens can turn {"c":95} into "c":950 without '}'.
    # Models may also emit a 0-1000 style score. Both map cleanly by /10.
    if 100.0 < confidence <= 1000.0:
        confidence = confidence / 10.0
    if not 0.0 <= confidence <= 100.0:
        raise ValueError(f"c={confidence} is outside [0,100]")
    return confidence


def _parse_unit_score(value: Any, name: str) -> float:
    if value is None:
        raise ValueError(f"Missing required JSON field: {name}")
    if isinstance(value, bool):
        raise ValueError(f"{name} cannot be boolean")
    try:
        score = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric in [0,1]") from exc
    if not 0.0 <= score <= 1.0:
        raise ValueError(f"{name}={score} is outside [0,1]")
    return score


def _parse_spawn_scan_output(
    data: Dict[str, Any],
    image_width: int,
    image_height: int,
) -> ModelResult:
    """Parse SPAWN_SCAN protocol: {"p","t","r","q"} (no s / no motion action).

    Visibility is inferred from p only: non-null => TARGET_VISIBLE, null => TARGET_INFERRED.
    """
    required = {"p", "t", "r", "q"}
    missing = sorted(required.difference(data))
    if missing:
        raise ValueError(f"Missing required JSON fields: {missing}")

    point = _parse_point(data.get("p"), image_width, image_height)
    if point is not None:
        result_name = "TARGET_VISIBLE"
        action = "POINT"
        role = "target"
        reason = "spawn_visible"
    else:
        result_name = "TARGET_INFERRED"
        action = "STOP"
        role = "none"
        reason = "spawn_inferred"

    score_t = _parse_unit_score(data.get("t"), "t")
    score_r = _parse_unit_score(data.get("r"), "r")
    score_q = _parse_unit_score(data.get("q"), "q")
    return ModelResult(
        result=result_name,
        point=point,
        point_role=role,
        label="",
        reason_code=reason,
        action=action,
        confidence=0.0,
        score_t=score_t,
        score_r=score_r,
        score_q=score_q,
    )


def parse_model_output(
    raw_text: str,
    mode: PromptMode,
    image_width: int,
    image_height: int,
) -> ModelResult:
    """Parse V3 compact protocol.

    Preferred: {"s":"I","a":"TURN_RIGHT","p":null}
    Legacy:    {"s":"I","p":[800,650]} -> inferred as a=POINT
    SPAWN_SCAN: {"p":null,"t":0.1,"r":0.2,"q":0.3}
    Optional:  "c" remains accepted if the model still emits it.
    """
    data = _extract_json(raw_text)
    if mode == PromptMode.SPAWN_SCAN:
        return _parse_spawn_scan_output(data, image_width, image_height)

    required = {"s", "p"}
    missing = sorted(required.difference(data))
    if missing:
        raise ValueError(f"Missing required JSON fields: {missing}")

    status = str(data.get("s", "")).strip().upper()
    if status not in _STATUS_TO_RESULT:
        raise ValueError(f"Unknown status s={status!r}; allowed=V|I|S|F")
    if status not in _ALLOWED_STATUS_BY_MODE[mode]:
        raise ValueError(
            f"Status s={status!r} is not allowed in mode {mode.value}; "
            f"allowed={sorted(_ALLOWED_STATUS_BY_MODE[mode])}"
        )

    raw_point = data.get("p")
    # Backward compatibility is deliberate: if the model occasionally emits
    # the old two-field protocol, a non-null p remains a POINT action.
    action_raw = data.get("a")
    if action_raw is None:
        action = "POINT" if raw_point is not None else "STOP"
    else:
        action = str(action_raw).strip().upper()
    if action not in _ACTIONS:
        raise ValueError(
            f"Unknown action a={action!r}; allowed={sorted(_ACTIONS)}"
        )
    if action not in _ALLOWED_ACTION_BY_STATUS[status]:
        raise ValueError(
            f"Action a={action!r} is incompatible with s={status}; "
            f"allowed={sorted(_ALLOWED_ACTION_BY_STATUS[status])}"
        )

    point = _parse_point(raw_point, image_width, image_height)
    if action == "POINT" and point is None:
        raise ValueError("a=POINT requires p=[x,y]")
    if action != "POINT" and point is not None:
        raise ValueError(f"a={action} requires p=null")

    result_name = _STATUS_TO_RESULT[status]
    role = _ROLE_BY_RESULT[result_name] if action == "POINT" else "none"
    confidence = _parse_confidence(data.get("c"))
    return ModelResult(
        result=result_name,
        point=point,
        point_role=role,
        label="",
        reason_code=f"compact_{status.lower()}_{action.lower()}",
        action=action,
        confidence=confidence,
    )
