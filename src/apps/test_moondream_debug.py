#!/usr/bin/env python3
"""
Moondream 调试工具：仅输出自然语言描述，不参与正式导航闭环。
"""
import argparse
import base64
import json
import os
import sys
import time

import cv2
import requests

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)


def main():
    parser = argparse.ArgumentParser(description="Moondream debug (not for competition nav)")
    parser.add_argument("--image", required=True)
    parser.add_argument("--prompt", default="Describe what you see in one sentence.")
    parser.add_argument("--model", default="moondream:latest")
    parser.add_argument("--resize-width", type=int, default=256)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--num-predict", type=int, default=40)
    args = parser.parse_args()

    img = cv2.imread(os.path.expanduser(args.image))
    if img is None:
        raise RuntimeError(f"failed to read image: {args.image}")

    h, w = img.shape[:2]
    if w > args.resize_width:
        scale = args.resize_width / float(w)
        img = cv2.resize(img, (args.resize_width, int(round(h * scale))))

    _, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    b64 = base64.b64encode(buf.tobytes()).decode()

    payload = {
        "model": args.model,
        "prompt": args.prompt,
        "images": [b64],
        "stream": False,
        "options": {"num_predict": args.num_predict, "num_ctx": 2048, "temperature": 0.1},
    }

    t0 = time.time()
    resp = requests.post("http://127.0.0.1:11434/api/generate", json=payload, timeout=args.timeout)
    resp.raise_for_status()
    data = resp.json()
    dt = time.time() - t0

    print(json.dumps(
        {
            "model": args.model,
            "response": data.get("response", ""),
            "eval_count": data.get("eval_count"),
            "latency_sec": dt,
        },
        ensure_ascii=False,
        indent=2,
    ))


if __name__ == "__main__":
    main()
