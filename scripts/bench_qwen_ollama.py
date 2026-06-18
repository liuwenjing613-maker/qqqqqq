#!/usr/bin/env python3
"""
Qwen VLM 参数 benchmark：单 client、一次 warmup、按可行性排序逐组 infer。

排序优先级（从轻到重）：
  width -> num_ctx -> num_predict -> jpeg_quality
"""
import argparse
import csv
import itertools
import json
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
    "coord_mode",
    "repeat_id",
    "ok",
    "usable",
    "latency_sec",
    "u",
    "v",
    "raw_u",
    "raw_v",
    "coord_invalid",
    "coord_reason",
    "orig_w",
    "orig_h",
    "model_w",
    "model_h",
    "scale_x",
    "scale_y",
    "ollama_total_ms",
    "ollama_load_ms",
    "ollama_prompt_eval_ms",
    "ollama_eval_ms",
    "debug_qwen_input",
    "debug_qwen_input_raw_point",
    "debug_orig_mapped_point",
    "raw_json",
    "error",
]


def iter_configs_sorted(
    widths: Iterable[int],
    qualities: Iterable[int],
    num_predicts: Iterable[int],
    num_ctxs: Iterable[int],
) -> List[Tuple[int, int, int, int]]:
    combos = list(itertools.product(widths, qualities, num_predicts, num_ctxs))
    combos.sort(
        key=lambda c: (
            c[0],  # width asc
            c[3],  # num_ctx asc
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
    parser.add_argument("--widths", nargs="+", type=int, default=[192])
    parser.add_argument("--qualities", nargs="+", type=int, default=[35, 45, 60])
    parser.add_argument("--num-predicts", nargs="+", type=int, default=[24, 32, 48])
    parser.add_argument(
        "--num-ctxs",
        nargs="+",
        type=int,
        default=[512],
        help="RDK X5 + qwen2.5vl 推荐先用 512；1024 可能触发极慢 prompt_eval",
    )
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument(
        "--keep-alive",
        default=-1,
        help='Ollama keep_alive: -1 (int) forever, or duration like "30m"',
    )
    parser.add_argument(
        "--coord-mode",
        default="norm1000",
        choices=["norm1000", "model", "original"],
    )
    parser.add_argument("--debug-dir", default="debug_qwen")
    parser.add_argument("--save-debug", action="store_true")
    parser.add_argument("--min-confidence", type=float, default=0.0)
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
        coord_mode=args.coord_mode,
        debug_dir=args.debug_dir,
        save_debug=args.save_debug,
        min_confidence=args.min_confidence,
    )

    print(f"[bench] configs={len(configs)} repeat={args.repeat} out={out_path}")
    print(f"[bench] coord_mode={args.coord_mode} save_debug={args.save_debug}")
    print(f"[bench] sort order: width -> num_ctx -> num_predict -> quality")
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
                "coord_mode": args.coord_mode,
                "repeat_id": i,
                "ok": ok,
                "usable": bool(result.get("usable", False)),
                "latency_sec": latency,
                "u": result.get("u"),
                "v": result.get("v"),
                "raw_u": result.get("_raw_u"),
                "raw_v": result.get("_raw_v"),
                "coord_invalid": result.get("_coord_invalid"),
                "coord_reason": result.get("_coord_reason"),
                "orig_w": result.get("_orig_image_width"),
                "orig_h": result.get("_orig_image_height"),
                "model_w": result.get("_model_image_width"),
                "model_h": result.get("_model_image_height"),
                "scale_x": result.get("_scale_x_to_orig"),
                "scale_y": result.get("_scale_y_to_orig"),
                "ollama_total_ms": result.get("_ollama_total_ms"),
                "ollama_load_ms": result.get("_ollama_load_ms"),
                "ollama_prompt_eval_ms": result.get("_ollama_prompt_eval_ms"),
                "ollama_eval_ms": result.get("_ollama_eval_ms"),
                "debug_qwen_input": result.get("debug_qwen_input"),
                "debug_qwen_input_raw_point": result.get("debug_qwen_input_raw_point"),
                "debug_orig_mapped_point": result.get("debug_orig_mapped_point"),
                "raw_json": json.dumps(result.get("_raw_json", {}), ensure_ascii=False)[:500],
                "error": err[:300],
            }
            rows.append(row)
            _append_row(out_path, row, write_header=write_header)
            write_header = False
            print(row, flush=True)

    ok_count = sum(1 for r in rows if r.get("ok"))
    usable_count = sum(1 for r in rows if r.get("usable"))
    print(f"\nSaved benchmark results to {out_path}")
    print(f"Summary: ok={ok_count}/{len(rows)} usable={usable_count}/{len(rows)}")

    usable = [r for r in rows if r.get("ok") and r.get("usable") is True]
    if usable:
        best = min(usable, key=lambda r: float(r.get("latency_sec") or 1e9))
        print(
            "Fastest usable: "
            f"w={best['width']} q={best['jpeg_quality']} "
            f"np={best['num_predict']} ctx={best['num_ctx']} "
            f"coord_mode={best['coord_mode']} "
            f"latency={best['latency_sec']:.1f}s "
            f"raw=({best['raw_u']},{best['raw_v']}) "
            f"u={best['u']} v={best['v']}"
        )


if __name__ == "__main__":
    main()
