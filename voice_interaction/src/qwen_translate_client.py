#!/usr/bin/env python3
from __future__ import annotations

import os

from openai import OpenAI


DEFAULT_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_MODEL = "qwen-mt-flash"


def _get_client() -> OpenAI:
    api_key = os.getenv("DASHSCOPE_API_KEY")
    if not api_key:
        raise RuntimeError("没有配置 DASHSCOPE_API_KEY")

    # Prefer voice-module URL; fall back to nav-style QWEN_BASE_URL if set.
    base_url = (
        os.getenv("DASHSCOPE_BASE_URL")
        or os.getenv("QWEN_BASE_URL")
        or DEFAULT_BASE_URL
    ).rstrip("/")

    return OpenAI(
        api_key=api_key,
        base_url=base_url,
        timeout=15.0,
        max_retries=1,
    )


def translate_instruction(text: str) -> str | None:
    """
    Translate Chinese ASR text into an English navigation instruction.

    Example:
        帮我找一个瓶子。 -> Help me find a bottle.
    """
    source_text = (text or "").strip()
    if not source_text:
        return None

    client = _get_client()
    response = client.chat.completions.create(
        model=os.getenv("QWEN_TRANSLATE_MODEL", DEFAULT_MODEL),
        messages=[
            {
                "role": "user",
                "content": source_text,
            }
        ],
        extra_body={
            "translation_options": {
                "source_lang": "Chinese",
                "target_lang": "English",
            }
        },
    )

    translated = response.choices[0].message.content
    if not translated:
        return None
    translated = translated.strip()
    return translated or None


if __name__ == "__main__":
    from pathlib import Path

    from dotenv import load_dotenv

    load_dotenv(Path(__file__).resolve().parents[1] / ".env")
    test_text = "帮我找一个瓶子。"
    result = translate_instruction(test_text)
    print(f"中文：{test_text}")
    print(f"英文：{result}")
