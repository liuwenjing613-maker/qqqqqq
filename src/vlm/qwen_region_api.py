#!/usr/bin/env python3
"""Thin Qwen VL API adapter for region selection — no motion side effects."""

from __future__ import annotations

import base64
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


@dataclass
class QwenApiResult:
    raw_response: str
    request_latency_ms: float
    model: str
    prompt_tokens: Optional[int] = None
    completion_tokens: Optional[int] = None
    total_tokens: Optional[int] = None
    retry_count: int = 0
    error_code: Optional[str] = None
    error_message: Optional[str] = None


def _resolve_model(cfg: Dict[str, Any]) -> str:
    qwen = cfg.get("qwen", {})
    env_name = str(qwen.get("model_env", "QWEN_MODEL"))
    default = str(qwen.get("default_model", "qwen3-vl-flash"))
    return os.getenv(env_name, "").strip() or default


def _resolve_api_key(cfg: Dict[str, Any]) -> str:
    qwen = cfg.get("qwen", {})
    env_name = str(qwen.get("api_key_env", "DASHSCOPE_API_KEY"))
    return os.getenv(env_name, "").strip()


def _resolve_base_url(cfg: Dict[str, Any]) -> str:
    qwen = cfg.get("qwen", {})
    env_name = str(qwen.get("base_url_env", "QWEN_BASE_URL"))
    default = str(
        qwen.get(
            "default_base_url",
            "https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
    )
    return os.getenv(env_name, "").strip() or default


def _encode_image(image_path: Path) -> str:
    data = image_path.read_bytes()
    b64 = base64.b64encode(data).decode("ascii")
    suffix = image_path.suffix.lower()
    mime = "image/png" if suffix == ".png" else "image/jpeg"
    return f"data:{mime};base64,{b64}"


def call_qwen_region_selection(
    prompt: str,
    image_path: Path,
    cfg: Dict[str, Any],
) -> QwenApiResult:
    """Call Qwen VL with text + annotated map image."""
    qwen_cfg = cfg.get("qwen", {})
    api_key = _resolve_api_key(cfg)
    if not api_key:
        return QwenApiResult(
            raw_response="",
            request_latency_ms=0.0,
            model=_resolve_model(cfg),
            error_code="API_KEY_MISSING",
            error_message=f"Environment variable {qwen_cfg.get('api_key_env', 'DASHSCOPE_API_KEY')} is empty",
        )

    model = _resolve_model(cfg)
    timeout_s = float(qwen_cfg.get("request_timeout_s", 30))
    max_retries = int(qwen_cfg.get("max_retries", 1))
    temperature = 0.0 if bool(qwen_cfg.get("use_low_randomness", True)) else 0.2
    max_tokens = int(qwen_cfg.get("max_tokens", 256))

    if not image_path.is_file():
        return QwenApiResult(
            raw_response="",
            request_latency_ms=0.0,
            model=model,
            error_code="API_REQUEST_FAILED",
            error_message=f"Image not found: {image_path}",
        )

    try:
        from openai import OpenAI
    except ImportError as exc:
        return QwenApiResult(
            raw_response="",
            request_latency_ms=0.0,
            model=model,
            error_code="API_REQUEST_FAILED",
            error_message=f"Missing openai package: {exc}",
        )

    data_url = _encode_image(image_path)
    client = OpenAI(
        api_key=api_key,
        base_url=_resolve_base_url(cfg),
        timeout=timeout_s,
        max_retries=0,
    )

    payload_messages = [
        {
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url": data_url}},
            ],
        }
    ]

    retry_count = 0
    last_error: Optional[Exception] = None
    t0 = time.perf_counter()

    for attempt in range(max_retries + 1):
        retry_count = attempt
        try:
            completion = client.chat.completions.create(
                model=model,
                messages=payload_messages,
                temperature=temperature,
                max_tokens=max_tokens,
                response_format={"type": "json_object"},
            )
            latency_ms = (time.perf_counter() - t0) * 1000.0
            content = completion.choices[0].message.content
            raw = content if isinstance(content, str) else str(content)
            if not raw.strip():
                return QwenApiResult(
                    raw_response="",
                    request_latency_ms=latency_ms,
                    model=model,
                    retry_count=retry_count,
                    error_code="MODEL_RESPONSE_EMPTY",
                    error_message="Empty model content",
                )
            usage = getattr(completion, "usage", None)
            return QwenApiResult(
                raw_response=raw,
                request_latency_ms=latency_ms,
                model=model,
                prompt_tokens=getattr(usage, "prompt_tokens", None),
                completion_tokens=getattr(usage, "completion_tokens", None),
                total_tokens=getattr(usage, "total_tokens", None),
                retry_count=retry_count,
            )
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            err_name = type(exc).__name__
            if "Timeout" in err_name or "timeout" in str(exc).lower():
                code = "API_TIMEOUT"
            else:
                code = "API_REQUEST_FAILED"
            if attempt < max_retries:
                continue
            latency_ms = (time.perf_counter() - t0) * 1000.0
            return QwenApiResult(
                raw_response="",
                request_latency_ms=latency_ms,
                model=model,
                retry_count=retry_count,
                error_code=code,
                error_message=str(last_error),
            )

    latency_ms = (time.perf_counter() - t0) * 1000.0
    return QwenApiResult(
        raw_response="",
        request_latency_ms=latency_ms,
        model=model,
        retry_count=retry_count,
        error_code="API_REQUEST_FAILED",
        error_message=str(last_error),
    )
