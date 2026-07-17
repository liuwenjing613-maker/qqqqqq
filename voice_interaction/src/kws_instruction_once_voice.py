#!/usr/bin/env python3
"""Wait for one valid wake/ASR/translation result and hand it to navigation.

This is an additive one-shot adapter around the existing voice_interaction
pipeline. It intentionally reuses the tested KWS, post-wake recording, Qwen ASR,
and Qwen translation modules instead of duplicating their implementations.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path
from types import SimpleNamespace

from dotenv import load_dotenv

from kws_test import create_keyword_spotter, listen_until_wake
from qwen_translate_client import translate_instruction
from wake_actions import handle_wake

ROOT_DIR = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="单次语音指令：唤醒 -> 录音 -> ASR -> 英译 -> 写入导航指令文件"
    )
    parser.add_argument("--device", default=os.getenv("VOICE_ALSA_DEVICE", "plughw:0,0"))
    parser.add_argument("--encoder", required=True)
    parser.add_argument("--decoder", required=True)
    parser.add_argument("--joiner", required=True)
    parser.add_argument("--tokens", required=True)
    parser.add_argument("--keywords-file", required=True)
    parser.add_argument("--num-threads", type=int, default=2)
    parser.add_argument("--output-file", required=True)
    parser.add_argument("--min-length", type=int, default=3)
    return parser.parse_args()


def normalize_instruction(value: str | None) -> str | None:
    """Convert model output to one safe, single-line instruction."""
    if value is None:
        return None

    text = value.strip()
    if not text:
        return None

    # Translation models occasionally wrap a short answer in quotes or fences.
    text = re.sub(r"^```(?:text)?\s*", "", text, flags=re.IGNORECASE)
    text = re.sub(r"\s*```$", "", text)
    text = text.strip().strip('"').strip("'").strip()
    text = " ".join(text.split())
    return text or None


def write_instruction_atomic(path: Path, instruction: str) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    temporary.write_text(instruction + "\n", encoding="utf-8")
    os.replace(temporary, path)


def main() -> int:
    args = parse_args()
    load_dotenv(ROOT_DIR / ".env", override=False)

    spotter_args = SimpleNamespace(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        keywords_file=args.keywords_file,
        num_threads=args.num_threads,
    )
    keyword_spotter = create_keyword_spotter(spotter_args)

    output_file = Path(args.output_file)
    try:
        output_file.unlink(missing_ok=True)
    except OSError as exc:
        print(f"[VOICE][ERROR] 无法清理旧指令文件：{exc}", file=sys.stderr, flush=True)
        return 2

    print("=" * 64, flush=True)
    print("[VOICE->NAV] 单次语音导航入口已启动", flush=True)
    print(f"[VOICE->NAV] 指令输出文件：{output_file}", flush=True)
    print(
        f"[VOICE->NAV] 录音时长：{os.getenv('VOICE_RECORD_SECONDS', '10')} 秒",
        flush=True,
    )
    print("[VOICE->NAV] 得到有效英文指令后将自动退出并启动导航。", flush=True)
    print("=" * 64, flush=True)

    try:
        while True:
            listen_until_wake(keyword_spotter, args.device)
            original_text = handle_wake()
            if not original_text or not original_text.strip():
                print("[VOICE->NAV] 本轮没有有效 ASR 文本，重新等待唤醒。", flush=True)
                continue

            original_text = original_text.strip()
            print(f"[CONTROL PANEL] 用户语音（原文）：{original_text}", flush=True)
            print("[TRANSLATE] 正在转换为英文 instruction...", flush=True)

            try:
                translated = translate_instruction(original_text)
            except Exception as exc:  # Keep the wake loop alive on cloud failures.
                print(f"[TRANSLATE][ERROR] {exc}", flush=True)
                print("[VOICE->NAV] 翻译失败，重新等待唤醒。", flush=True)
                continue

            instruction = normalize_instruction(translated)
            if instruction is None or len(instruction) < args.min_length:
                print("[TRANSLATE] 未得到有效英文结果，重新等待唤醒。", flush=True)
                continue

            print(f"[INSTRUCTION] 英文：{instruction}", flush=True)
            write_instruction_atomic(output_file, instruction)
            print("[VOICE->NAV] 指令已交付，准备启动视觉语言导航。", flush=True)
            return 0

    except KeyboardInterrupt:
        print("\n[VOICE->NAV] 用户终止，未启动导航。", flush=True)
        return 130
    except Exception as exc:
        print(f"[VOICE->NAV][FATAL] {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
