#!/usr/bin/env python3
"""Offline smoke test for src/vlm/qwen_dashscope_client.py."""
import argparse
import json
import os
import sys

import cv2

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_dashscope_client import QwenDashScopeClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--instruction", default="find bottle")
    parser.add_argument("--mode", default="track", choices=["track", "search", "scan"])
    parser.add_argument("--first-request", action="store_true")
    args = parser.parse_args()

    frame = cv2.imread(args.image)
    if frame is None:
        raise RuntimeError("Cannot read image: " + args.image)

    client = QwenDashScopeClient(timeout=15, resize_width=640, jpeg_quality=80, min_confidence=0.60)
    result = client.infer_navigation(
        frame,
        args.instruction,
        mode=args.mode,
        first_request=args.first_request,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    client.close()


if __name__ == "__main__":
    main()
