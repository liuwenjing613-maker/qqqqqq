#!/usr/bin/env python3
"""Persistent wake-word + fixed-function ASR event server.

It deliberately reuses the repository's tested KWS and post-wake ASR path:
  listen_until_wake() -> handle_wake()
The wake event is written *before* the 5 s recording starts, allowing the demo
hub to stop the current robot motion immediately after the wake word.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

from dotenv import load_dotenv
from kws_test import create_keyword_spotter, listen_until_wake
from wake_actions import handle_wake

ROOT_DIR = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="持续语音功能选择事件服务")
    parser.add_argument("--device", default=os.getenv("VOICE_ALSA_DEVICE", "plughw:0,0"))
    parser.add_argument("--encoder", required=True)
    parser.add_argument("--decoder", required=True)
    parser.add_argument("--joiner", required=True)
    parser.add_argument("--tokens", required=True)
    parser.add_argument("--keywords-file", required=True)
    parser.add_argument("--num-threads", type=int, default=2)
    parser.add_argument("--event-file", required=True)
    return parser.parse_args()


def emit(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    line = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    with path.open("a", encoding="utf-8", buffering=1) as f:
        f.write(line + "\n")
        f.flush()
        os.fsync(f.fileno())


def normalize_text(value: str | None) -> str:
    if value is None:
        return ""
    return " ".join(value.strip().split())


def main() -> int:
    args = parse_args()
    load_dotenv(ROOT_DIR / ".env", override=False)
    event_file = Path(args.event_file).expanduser().resolve()
    event_file.parent.mkdir(parents=True, exist_ok=True)
    # The hub owns truncation at startup. A KWS restart must never erase unread events.
    event_file.touch(exist_ok=True)

    spotter_args = SimpleNamespace(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        keywords_file=args.keywords_file,
        num_threads=args.num_threads,
    )
    keyword_spotter = create_keyword_spotter(spotter_args)
    seq = 0
    print("[VOICE-MENU] KWS 已常驻加载，持续等待唤醒。", flush=True)
    print(f"[VOICE-MENU] 功能录音时长：{os.getenv('VOICE_RECORD_SECONDS', '5')} 秒", flush=True)

    try:
        while True:
            listen_until_wake(keyword_spotter, args.device)
            seq += 1
            emit(event_file, {"seq": seq, "event": "wake", "time": time.time()})
            print(f"[VOICE-MENU] wake seq={seq}，已通知总控立即停车。", flush=True)
            try:
                original = normalize_text(handle_wake())
            except Exception as exc:  # Keep the KWS server alive for the demo.
                emit(
                    event_file,
                    {"seq": seq, "event": "error", "time": time.time(), "error": repr(exc)},
                )
                print(f"[VOICE-MENU][ERROR] 本轮 ASR 失败：{exc}", file=sys.stderr, flush=True)
                continue

            emit(
                event_file,
                {
                    "seq": seq,
                    "event": "command",
                    "time": time.time(),
                    "text": original,
                    "valid": bool(original),
                },
            )
            if original:
                print(f"[VOICE-MENU] 用户功能命令：{original}", flush=True)
            else:
                print("[VOICE-MENU] 本轮无有效文本，保持待机。", flush=True)
    except KeyboardInterrupt:
        print("\n[VOICE-MENU] 退出。", flush=True)
        return 130
    except Exception as exc:
        emit(event_file, {"seq": seq, "event": "fatal", "time": time.time(), "error": repr(exc)})
        print(f"[VOICE-MENU][FATAL] {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
