from __future__ import annotations

import base64
import mimetypes
import os
from pathlib import Path

from openai import OpenAI


DEFAULT_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
DEFAULT_MODEL = "qwen3-asr-flash"


class AsrConfigurationError(RuntimeError):
    """Raised when the ASR client is not configured correctly."""


def _to_data_uri(audio_path: Path) -> str:
    mime_type, _ = mimetypes.guess_type(audio_path.name)
    if mime_type is None:
        mime_type = "audio/wav"
    encoded = base64.b64encode(audio_path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{encoded}"


def transcribe_file(audio_path: str | Path) -> str:
    path = Path(audio_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Audio file does not exist: {path}")

    api_key = os.getenv("DASHSCOPE_API_KEY") or os.getenv("QWEN_API_KEY")
    if not api_key:
        raise AsrConfigurationError(
            "DASHSCOPE_API_KEY or QWEN_API_KEY is not set."
        )

    base_url = os.getenv(
        "DASHSCOPE_BASE_URL", DEFAULT_BASE_URL
    ).rstrip("/")
    model = os.getenv("QWEN_ASR_MODEL", DEFAULT_MODEL)
    language = os.getenv("QWEN_ASR_LANGUAGE", "zh").strip()

    asr_options: dict[str, object] = {"enable_itn": True}
    if language:
        asr_options["language"] = language

    client = OpenAI(api_key=api_key, base_url=base_url)
    completion = client.chat.completions.create(
        model=model,
        messages=[
            {
                "role": "user",
                "content": [
                    {
                        "type": "input_audio",
                        "input_audio": {"data": _to_data_uri(path)},
                    }
                ],
            }
        ],
        stream=False,
        extra_body={"asr_options": asr_options},
    )

    text = completion.choices[0].message.content
    if not isinstance(text, str) or not text.strip():
        raise RuntimeError("ASR returned an empty transcript.")
    return text.strip()
