#!/usr/bin/env python3
"""Diagnose Qwen infer speed without restarting camera or Ollama."""
import argparse
import os
import subprocess
import sys
import time

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_ollama_client import QwenOllamaClient


def read_mem():
    with open("/proc/meminfo") as f:
        info = {}
        for line in f:
            k, v = line.split(":", 1)
            info[k.strip()] = v.strip()
    avail_kb = int(info["MemAvailable"].split()[0])
    swap_kb = int(info["SwapTotal"].split()[0]) - int(info["SwapFree"].split()[0])
    return avail_kb / 1024 / 1024, swap_kb / 1024 / 1024


def llama_cpu_pct():
    try:
        out = subprocess.check_output(
            ["ps", "-o", "pcpu=", "-p", subprocess.check_output(
                ["pgrep", "-f", "llama-server"], text=True
            ).strip().split("\n")[0]],
            text=True,
        )
        return float(out.strip())
    except Exception:
        return -1.0


def grab_camera_frame(topic: str, timeout_sec: float = 8.0):
    class _Grab(Node):
        def __init__(self):
            super().__init__("diag_grab_frame")
            self.bridge = CvBridge()
            self.frame = None
            self.sub = self.create_subscription(Image, topic, self._cb, 10)

        def _cb(self, msg):
            if self.frame is None:
                self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    rclpy.init()
    node = _Grab()
    t0 = time.time()
    try:
        while node.frame is None and time.time() - t0 < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        frame = node.frame.copy() if node.frame is not None else None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return frame


def run_case(name: str, frame, client: QwenOllamaClient, instruction: str):
    h, w = frame.shape[:2]
    avail_gb, swap_gb = read_mem()
    cpu = llama_cpu_pct()
    print(f"\n=== CASE: {name} ===")
    print(f"frame={w}x{h} resize_width={client.resize_width} num_ctx={client.num_ctx}")
    print(f"mem_available={avail_gb:.2f}Gi swap_used={swap_gb:.0f}Mi llama_cpu={cpu:.0f}%")

    t0 = time.time()
    result = client.infer_navigation(frame, instruction)
    dt = time.time() - t0

    print(
        f"RESULT latency={dt:.1f}s "
        f"prompt_eval_ms={result.get('_ollama_prompt_eval_ms', 0):.0f} "
        f"eval_ms={result.get('_ollama_eval_ms', 0):.0f} "
        f"load_ms={result.get('_ollama_load_ms', 0):.0f} "
        f"usable={result.get('usable')}"
    )
    return {
        "name": name,
        "latency_sec": dt,
        "prompt_eval_ms": result.get("_ollama_prompt_eval_ms"),
        "eval_ms": result.get("_ollama_eval_ms"),
        "load_ms": result.get("_ollama_load_ms"),
        "frame": f"{w}x{h}",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs/qwen_lidar_nav.yaml"))
    parser.add_argument("--instruction", default="green bottle")
    parser.add_argument("--bench-image", default=os.path.join(PROJECT_ROOT, "test_qwen/raw_png/no_target_2.png"))
    parser.add_argument("--image-topic", default="/image_raw")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    client = QwenOllamaClient(
        model=cfg["model"],
        timeout=cfg["qwen_timeout_sec"],
        resize_width=cfg["qwen_resize_width"],
        jpeg_quality=cfg.get("qwen_jpeg_quality", 45),
        num_predict=cfg.get("qwen_num_predict", 16),
        num_ctx=cfg.get("qwen_num_ctx", 256),
        keep_alive=cfg.get("qwen_keep_alive", -1),
        coord_mode=cfg.get("qwen_coord_mode", "norm1000"),
        save_debug=False,
    )

    print("=== Qwen infer speed diagnostic (camera+ollama kept running) ===")
    avail_gb, swap_gb = read_mem()
    print(f"startup mem_available={avail_gb:.2f}Gi swap_used={swap_gb:.0f}Mi")

    rows = []

    bench = cv2.imread(args.bench_image)
    if bench is not None:
        rows.append(run_case("static_bench_png", bench, client, args.instruction))
    else:
        print(f"WARN: bench image missing: {args.bench_image}")

    cam = grab_camera_frame(args.image_topic)
    if cam is not None:
        rows.append(run_case("live_camera_frame", cam, client, args.instruction))
    else:
        print(f"ERROR: no frame from {args.image_topic}")

    print("\n=== SUMMARY ===")
    for r in rows:
        pe = r["prompt_eval_ms"] or 0
        tag = "HOT" if pe < 5000 else ("WARM" if pe < 60000 else "COLD/SWAP")
        print(
            f"{r['name']}: {r['latency_sec']:.1f}s prompt_eval={pe:.0f}ms "
            f"frame={r['frame']} [{tag}]"
        )

    if len(rows) == 2:
        r0, r1 = rows
        pe0 = r0["prompt_eval_ms"] or 0
        pe1 = r1["prompt_eval_ms"] or 0
        if pe0 < 5000 and pe1 > 60000:
            print("BOTTLENECK: live camera path much slower than static -> likely memory pressure + swap")
        elif pe0 > 60000 and pe1 > 60000:
            print("BOTTLENECK: both cold -> model not hot OR severe memory/swap thrashing")
        elif pe0 < 5000 and pe1 < 5000:
            print("OK: both hot -> nav slowness was likely duplicate processes or skipped warmup timing")


if __name__ == "__main__":
    main()
