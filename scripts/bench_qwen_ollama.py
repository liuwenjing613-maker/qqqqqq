#!/usr/bin/env python3
"""
Qwen VLM 参数 benchmark：单 client、一次 warmup、按可行性排序逐组 infer。

排序优先级（从轻到重）：
  width -> num_ctx(512 最后) -> num_predict -> jpeg_quality
"""
import argparse
import csv
import itertools
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import cv2

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.vlm.qwen_ollama_client import QwenOllamaClient

FIELDNAMES = [
    "model",
    "width",
    "jpeg_quality",
    "num_predict",
    "num_ctx",
    "repeat_id",
    "ok",
    "latency_sec",
    "u",
    "v",
    "ollama_total_ms",
    "ollama_load_ms",
    "ollama_prompt_eval_ms",
    "ollama_eval_ms",
    "error",
]


def _num_ctx_sort_key(num_ctx: int) -> Tuple[int, int]:
    """512 ctx 对 qwen-VL 往往不够，排到最后探测。"""
    if num_ctx <= 512:
        return (1, num_ctx)
    return (0, num_ctx)


def iter_configs_sorted(
    widths: Iterable[int],
    qualities: Iterable[int],
    num_predicts: Iterable[int],
    num_ctxs: Iterable[int],
) -> List[Tuple[int, int, int, int]]:
    combos = list(itertools.product(widths, qualities, num_predicts, num_ctxs))
    # (width, quality, num_predict, num_ctx) -> sort by feasibility
    combos.sort(
        key=lambda c: (
            c[0],  # width asc
            _num_ctx_sort_key(c[3]),
            c[3],  # num_ctx asc within tier
            c[2],  # num_predict asc
            c[1],  # jpeg_quality asc
        )
    )
    return combos


def _apply_client_params(
    client: QwenOllamaClient,
    width: int,
    quality: int,
    num_predict: int,
    num_ctx: int,
) -> None:
    client.resize_width = int(width)
    client.jpeg_quality = int(quality)
    client.num_predict = int(num_predict)
    client.num_ctx = int(num_ctx)


def _append_row(out_path: Path, row: Dict[str, Any], write_header: bool) -> None:
    with out_path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(
        description="Benchmark Qwen VLM params (single warmup, sorted light-to-heavy)",
    )
    parser.add_argument("--image", required=True)
    parser.add_argument("--instruction", default="find the red backpack")
    parser.add_argument("--model", default="qwen2.5vl:3b")
    parser.add_argument("--widths", nargs="+", type=int, default=[96])
    parser.add_argument("--qualities", nargs="+", type=int, default=[35, 45, 60])
    parser.add_argument("--num-predicts", nargs="+", type=int, default=[24, 32, 48])
    parser.add_argument(
        "--num-ctxs",
        nargs="+",
        type=int,
        default=[1024, 768, 512],
        help="512 会自动排到最后；推荐优先 1024/768",
    )
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument(
        "--keep-alive",
        default=-1,
        help='Ollama keep_alive: -1 (int) forever, or duration like "30m"',
    )
    parser.add_argument("--no-warmup", action="store_true", help="skip one-time warmup_full")
    parser.add_argument("--out", default="qwen_bench.csv")
    args = parser.parse_args()

    frame = cv2.imread(args.image)
    if frame is None:
        raise RuntimeError(f"Failed to read image: {args.image}")

    configs = iter_configs_sorted(
        args.widths, args.qualities, args.num_predicts, args.num_ctxs
    )
    if not configs:
        raise RuntimeError("empty parameter grid")

    out_path = Path(args.out)
    if out_path.exists():
        out_path.unlink()

    first_w, first_q, first_np, first_nc = configs[0]
    client = QwenOllamaClient(
        model=args.model,
        resize_width=first_w,
        jpeg_quality=first_q,
        num_predict=first_np,
        num_ctx=first_nc,
        timeout=args.timeout,
        keep_alive=args.keep_alive,
    )

    print(f"[bench] configs={len(configs)} repeat={args.repeat} out={out_path}")
    print(f"[bench] sort order: width -> num_ctx(512 last) -> num_predict -> quality")
    print(f"[bench] first config: w={first_w} q={first_q} np={first_np} ctx={first_nc}")

    if not args.no_warmup:
        print("[bench] one-time warmup_full...", flush=True)
        try:
            dt = client.warmup_full(timeout=args.timeout)
            print(f"[bench] warmup done in {dt:.1f}s", flush=True)
        except Exception as e:
            print(f"[bench] WARN warmup failed: {e}", flush=True)

    rows: List[Dict[str, Any]] = []
    write_header = True

    for idx, (width, quality, num_predict, num_ctx) in enumerate(configs, start=1):
        _apply_client_params(client, width, quality, num_predict, num_ctx)
        print(
            f"\n=== [{idx}/{len(configs)}] "
            f"width={width} quality={quality} "
            f"num_predict={num_predict} num_ctx={num_ctx} ===",
            flush=True,
        )

        for i in range(args.repeat):
            t0 = time.time()
            ok = True
            err = ""
            result: Dict[str, Any] = {}

            try:
                result = client.infer_navigation(frame, args.instruction)
            except Exception as e:
                ok = False
                err = str(e)

            latency = time.time() - t0
            row = {
                "model": args.model,
                "width": width,
                "jpeg_quality": quality,
                "num_predict": num_predict,
                "num_ctx": num_ctx,
                "repeat_id": i,
                "ok": ok,
                "latency_sec": latency,
                "u": result.get("u"),
                "v": result.get("v"),
                "ollama_total_ms": result.get("_ollama_total_ms"),
                "ollama_load_ms": result.get("_ollama_load_ms"),
                "ollama_prompt_eval_ms": result.get("_ollama_prompt_eval_ms"),
                "ollama_eval_ms": result.get("_ollama_eval_ms"),
                "error": err[:200],
            }
            rows.append(row)
            _append_row(out_path, row, write_header=write_header)
            write_header = False
            print(row, flush=True)

    ok_count = sum(1 for r in rows if r.get("ok"))
    print(f"\nSaved benchmark results to {out_path}")
    print(f"Summary: ok={ok_count}/{len(rows)} failed={len(rows) - ok_count}")

    usable = [
        r for r in rows
        if r.get("ok") and r.get("u") is not None and r.get("v") is not None
    ]
    if usable:
        best = min(usable, key=lambda r: float(r.get("latency_sec") or 1e9))
        print(
            "Fastest usable: "
            f"w={best['width']} q={best['jpeg_quality']} "
            f"np={best['num_predict']} ctx={best['num_ctx']} "
            f"latency={best['latency_sec']:.1f}s "
            f"u={best['u']} v={best['v']}"
        )


if __name__ == "__main__":
    main()
