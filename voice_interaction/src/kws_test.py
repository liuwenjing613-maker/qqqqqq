#!/usr/bin/env python3
"""Local wake-word detection with post-wake ASR via Qwen."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import sherpa_onnx

from qwen_translate_client import translate_instruction
from wake_actions import handle_wake


SAMPLE_RATE = 16000
FRAME_SECONDS = 0.1
FRAME_SAMPLES = int(SAMPLE_RATE * FRAME_SECONDS)
FRAME_BYTES = FRAME_SAMPLES * 2


def require_file(path: str) -> str:
    file_path = Path(path)
    if not file_path.is_file():
        raise FileNotFoundError(f"文件不存在：{file_path}")
    return str(file_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RDK X5 本地唤醒 + 千问 ASR 闭环"
    )
    parser.add_argument(
        "--device",
        default="plughw:0,0",
        help="ALSA 录音设备，例如 plughw:0,0",
    )
    parser.add_argument("--encoder", required=True)
    parser.add_argument("--decoder", required=True)
    parser.add_argument("--joiner", required=True)
    parser.add_argument("--tokens", required=True)
    parser.add_argument("--keywords-file", required=True)
    parser.add_argument("--num-threads", type=int, default=2)
    return parser.parse_args()


def create_keyword_spotter(
    args: argparse.Namespace,
) -> sherpa_onnx.KeywordSpotter:
    return sherpa_onnx.KeywordSpotter(
        tokens=require_file(args.tokens),
        encoder=require_file(args.encoder),
        decoder=require_file(args.decoder),
        joiner=require_file(args.joiner),
        keywords_file=require_file(args.keywords_file),
        num_threads=args.num_threads,
        max_active_paths=4,
        keywords_score=1.0,
        keywords_threshold=0.25,
        num_trailing_blanks=1,
        provider="cpu",
    )


def start_arecord(device: str) -> subprocess.Popen:
    command = [
        "arecord",
        "-q",
        "-D",
        device,
        "-t",
        "raw",
        "-f",
        "S16_LE",
        "-r",
        str(SAMPLE_RATE),
        "-c",
        "1",
    ]
    print("启动麦克风：", " ".join(command), flush=True)
    return subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=None,
    )


def stop_arecord(recorder: subprocess.Popen | None) -> None:
    if recorder is None:
        return
    if recorder.stdout is not None:
        recorder.stdout.close()
    if recorder.poll() is None:
        recorder.terminate()
        try:
            recorder.wait(timeout=2)
        except subprocess.TimeoutExpired:
            recorder.kill()
            recorder.wait()


def listen_until_wake(
    keyword_spotter: sherpa_onnx.KeywordSpotter,
    device: str,
) -> str:
    """
    Open the microphone and listen until a wake word is detected.
    Always releases arecord before returning.
    """
    stream = keyword_spotter.create_stream()
    recorder: subprocess.Popen | None = None

    try:
        recorder = start_arecord(device)
        if recorder.stdout is None:
            raise RuntimeError("无法读取 arecord 输出。")

        print("[KWS] 等待唤醒词：小车你好", flush=True)

        while True:
            raw = recorder.stdout.read(FRAME_BYTES)
            if not raw:
                raise RuntimeError(
                    "麦克风数据流中断，请检查 USB 连接。"
                )
            if len(raw) != FRAME_BYTES:
                continue

            samples = np.frombuffer(raw, dtype="<i2").astype(np.float32)
            samples /= 32768.0
            stream.accept_waveform(SAMPLE_RATE, samples)

            while keyword_spotter.is_ready(stream):
                keyword_spotter.decode_stream(stream)

            result = keyword_spotter.get_result(stream)
            if result:
                print(f"[KWS] 检测结果：{result}", flush=True)
                return result
    finally:
        stop_arecord(recorder)


def main() -> int:
    args = parse_args()
    keyword_spotter = create_keyword_spotter(args)

    print("[SYSTEM] 语音交互程序启动", flush=True)

    try:
        while True:
            listen_until_wake(keyword_spotter, args.device)
            text = handle_wake()
            if text:
                print(f"[CONTROL PANEL] 用户语音（原文）：{text}", flush=True)
                print(
                    "[TRANSLATE] 正在转换为英文 instruction...",
                    flush=True,
                )
                try:
                    instruction = translate_instruction(text)
                except Exception as exc:
                    print(f"[TRANSLATE][ERROR] {exc}", flush=True)
                    instruction = None
                if instruction is None:
                    print(
                        "[TRANSLATE] 未得到有效英文结果",
                        flush=True,
                    )
                else:
                    print(
                        f"[INSTRUCTION] 英文：{instruction}",
                        flush=True,
                    )
            else:
                print(
                    "[CONTROL PANEL] 本轮没有得到有效文字。",
                    flush=True,
                )
            print("[KWS] 重新进入待唤醒状态。", flush=True)
    except KeyboardInterrupt:
        print("\n[SYSTEM] 程序已停止。", flush=True)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"错误：{exc}", file=sys.stderr)
        raise SystemExit(1)
