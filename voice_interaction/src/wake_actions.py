#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import time
from datetime import datetime
from pathlib import Path

from dotenv import load_dotenv

from audio_utils import record_wav
from qwen_asr_client import transcribe_file


ROOT_DIR = Path(__file__).resolve().parents[1]
load_dotenv(ROOT_DIR / ".env")


def optional_int(value: str | None) -> int | None:
    if value is None or not value.strip():
        return None
    return int(value)


DEVICE_INDEX = optional_int(os.getenv("VOICE_DEVICE_INDEX", "0"))
RECORD_SECONDS = float(os.getenv("VOICE_RECORD_SECONDS", "10"))
ACK_GAP_SECONDS = float(os.getenv("VOICE_ACK_GAP_SECONDS", "0.3"))

ack_file_value = os.getenv("VOICE_ACK_FILE", "assets/i_am_here.wav")
ACK_FILE = Path(ack_file_value)
if not ACK_FILE.is_absolute():
    ACK_FILE = ROOT_DIR / ACK_FILE


def play_ack() -> None:
    """Play the fixed acknowledgment clip through the default speaker."""
    if not ACK_FILE.is_file():
        raise FileNotFoundError(f"没有找到提示音文件：{ACK_FILE}")

    print("[VOICE] 播放反馈：我在", flush=True)
    result = subprocess.run(["aplay", "-q", str(ACK_FILE)], check=False)
    if result.returncode != 0:
        raise RuntimeError(f"播放提示音失败，aplay 返回码：{result.returncode}")


def record_and_transcribe() -> str:
    """Record a fixed-length command and send it to Qwen ASR."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = ROOT_DIR / "recordings" / f"command_{timestamp}.wav"

    print(
        f"[VOICE] 开始录音 {RECORD_SECONDS:.1f} 秒，请说出命令。",
        flush=True,
    )
    wav_path = record_wav(
        output_path=output_path,
        seconds=RECORD_SECONDS,
        device_index=DEVICE_INDEX,
    )
    print(f"[VOICE] 录音完成：{wav_path}", flush=True)
    print("[ASR] 正在发送给千问进行识别……", flush=True)
    text = transcribe_file(wav_path)
    print(f"[ASR] 识别结果：{text}", flush=True)
    return text


def handle_wake() -> str | None:
    """
    Full post-wake action sequence.

    Caller must release the microphone before invoking this function.
    """
    print("=" * 50, flush=True)
    print("[WAKE] 检测到唤醒词：小车你好", flush=True)

    try:
        play_ack()
        if ACK_GAP_SECONDS > 0:
            print(
                f"[VOICE] 等待 {ACK_GAP_SECONDS:.1f} 秒后开始录音。",
                flush=True,
            )
            time.sleep(ACK_GAP_SECONDS)
        return record_and_transcribe()
    except Exception as exc:
        print(f"[VOICE][ERROR] {exc}", flush=True)
        return None
    finally:
        print("[VOICE] 本轮语音交互结束。", flush=True)
        print("=" * 50, flush=True)
