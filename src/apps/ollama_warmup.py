#!/usr/bin/env python3
"""Warm up Ollama Qwen model (text + vision) before real inference."""
import argparse
import os
import sys

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_ollama_client import QwenOllamaClient


def main():
    parser = argparse.ArgumentParser(description="Warm up Qwen VLM before inference")
    parser.add_argument("--model", default="qwen2.5vl:3b")
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--resize-width", type=int, default=192)
    parser.add_argument("--num-ctx", type=int, default=256)
    parser.add_argument("--num-predict", type=int, default=16)
    parser.add_argument("--keep-alive", default=-1)
    parser.add_argument("--text-only", action="store_true", help="only text warmup")
    parser.add_argument("--vision-only", action="store_true", help="only vision warmup")
    args = parser.parse_args()

    client = QwenOllamaClient(
        model=args.model,
        timeout=args.timeout,
        resize_width=args.resize_width,
        num_ctx=args.num_ctx,
        num_predict=args.num_predict,
        keep_alive=args.keep_alive,
    )
    if not client.check_health():
        raise RuntimeError(f"Ollama/model unavailable: {args.model}")

    if args.text_only:
        client.warmup(timeout=args.timeout)
    elif args.vision_only:
        client.warmup_vision(timeout=args.timeout)
    else:
        client.warmup_full(timeout=args.timeout)


if __name__ == "__main__":
    main()
