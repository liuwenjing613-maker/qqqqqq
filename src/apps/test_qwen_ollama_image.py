#!/usr/bin/env python3
import argparse
import json
import os
import sys

import cv2

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.vlm.qwen_ollama_client import QwenOllamaClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--instruction", default="find the bottle")
    parser.add_argument("--model", default="moondream:latest")
    parser.add_argument("--resize-width", type=int, default=256)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--warmup", action="store_true", help="preload model before vision infer")
    parser.add_argument("--recover", action="store_true", help="kill stuck llama-server before infer")
    parser.add_argument(
        "--prep",
        action="store_true",
        help="run ollama_prep_infer.sh (swap + unload models) before infer",
    )
    args = parser.parse_args()

    img = cv2.imread(os.path.expanduser(args.image))
    if img is None:
        raise RuntimeError(f"failed to read image: {args.image}")

    client = QwenOllamaClient(
        model=args.model,
        resize_width=args.resize_width,
        timeout=args.timeout,
    )

    if args.prep:
        import subprocess

        prep = os.path.join(PROJECT_ROOT, "scripts/ollama_prep_infer.sh")
        subprocess.run(["bash", prep, args.model], check=True)

    if not client.check_health():
        raise RuntimeError(
            f"Ollama not reachable or model missing at {client.url}. "
            f"Run: ollama serve && ollama pull {args.model}"
        )

    if args.recover:
        client.recover_stuck_server()

    if args.warmup:
        dt = client.warmup()
        print(f"[warmup] model loaded in {dt:.1f}s", flush=True)

    result = client.infer_navigation(img, args.instruction)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
