#!/usr/bin/env python3
"""Test Qwen VLM u/v JSON on a static image (defaults from qwen_lidar_nav.yaml)."""
import argparse
import json
import os
import subprocess
import sys
from typing import Any, Dict, Optional

import cv2
import yaml

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.vlm.qwen_ollama_client import QwenOllamaClient

DEFAULT_CONFIG = os.path.join(PROJECT_ROOT, "configs/qwen_lidar_nav.yaml")


def load_yaml(path: str) -> Dict[str, Any]:
    path = os.path.expanduser(path)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise RuntimeError(f"Config did not parse into a dict: {path}")
    return cfg


def _resolve_debug_dir(cfg: Dict[str, Any], override: Optional[str]) -> str:
    if override:
        return os.path.expanduser(override)
    debug_dir_cfg = cfg.get("debug_dir", "debug_qwen")
    if os.path.isabs(debug_dir_cfg):
        return debug_dir_cfg
    return os.path.join(PROJECT_ROOT, debug_dir_cfg)


def build_client(cfg: Dict[str, Any], args: argparse.Namespace) -> QwenOllamaClient:
    """CLI overrides take precedence over config file values."""
    return QwenOllamaClient(
        model=args.model or cfg["model"],
        timeout=float(args.timeout if args.timeout is not None else cfg["qwen_timeout_sec"]),
        resize_width=int(
            args.resize_width if args.resize_width is not None else cfg["qwen_resize_width"]
        ),
        jpeg_quality=int(
            args.jpeg_quality if args.jpeg_quality is not None else cfg.get("qwen_jpeg_quality", 45)
        ),
        num_predict=int(
            args.num_predict if args.num_predict is not None else cfg.get("qwen_num_predict", 16)
        ),
        num_ctx=int(args.num_ctx if args.num_ctx is not None else cfg.get("qwen_num_ctx", 256)),
        keep_alive=args.keep_alive if args.keep_alive is not None else cfg.get("qwen_keep_alive", -1),
        coord_mode=args.coord_mode or cfg.get("qwen_coord_mode", "norm1000"),
        debug_dir=_resolve_debug_dir(cfg, args.debug_dir),
        save_debug=args.save_debug if args.save_debug is not None else bool(cfg.get("save_debug", False)),
        min_confidence=float(
            args.min_confidence if args.min_confidence is not None else cfg.get("min_confidence", 0.0)
        ),
        url=args.url,
    )


def _print_client_params(client: QwenOllamaClient, cfg_path: str) -> None:
    print(
        "[test_qwen] config="
        f"{cfg_path} model={client.model} resize_width={client.resize_width} "
        f"jpeg_quality={client.jpeg_quality} num_predict={client.num_predict} "
        f"num_ctx={client.num_ctx} keep_alive={client.keep_alive} "
        f"coord_mode={client.coord_mode} min_confidence={client.min_confidence} "
        f"timeout={client.timeout}s save_debug={client.save_debug}",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Test Qwen VLM u/v JSON on a static image (Qwen params from yaml + CLI overrides)",
    )
    parser.add_argument("--image", required=True)
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG,
        help=f"Qwen parameter defaults (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument("--instruction", default=None)
    parser.add_argument("--model", default=None)
    parser.add_argument("--url", default="http://127.0.0.1:11434/api/generate")
    parser.add_argument("--resize-width", type=int, default=None)
    parser.add_argument("--jpeg-quality", type=int, default=None)
    parser.add_argument("--num-predict", type=int, default=None)
    parser.add_argument("--num-ctx", type=int, default=None)
    parser.add_argument(
        "--keep-alive",
        default=None,
        help='Ollama keep_alive, e.g. -1 or "1h" (default from config)',
    )
    parser.add_argument("--min-confidence", type=float, default=None)
    parser.add_argument("--coord-mode", default=None, choices=["norm1000", "model", "original"])
    parser.add_argument("--debug-dir", default=None)
    parser.add_argument("--save-debug", action="store_true", default=None)
    parser.add_argument("--no-save-debug", action="store_false", dest="save_debug")
    parser.add_argument("--timeout", type=float, default=None, help="infer timeout seconds")
    parser.add_argument(
        "--warmup-timeout",
        type=float,
        default=None,
        help="warmup timeout seconds (default: qwen_warmup_timeout_sec from config)",
    )
    parser.add_argument(
        "--warmup",
        action="store_true",
        help="optional warmup before infer (default: off, for measuring raw infer latency)",
    )
    parser.add_argument(
        "--no-warmup",
        action="store_true",
        help="explicitly skip warmup (default; use when benchmarking cold/hot infer)",
    )
    parser.add_argument("--recover", action="store_true", help="kill stuck llama-server before infer")
    parser.add_argument(
        "--prep",
        action="store_true",
        help="run ollama_prep_infer.sh (swap + unload models) before infer",
    )
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    instruction = args.instruction or "find the bottle"
    warmup_timeout = float(
        args.warmup_timeout
        if args.warmup_timeout is not None
        else cfg.get("qwen_warmup_timeout_sec", cfg.get("qwen_timeout_sec", 900.0))
    )

    img = cv2.imread(os.path.expanduser(args.image))
    if img is None:
        raise RuntimeError(f"failed to read image: {args.image}")

    client = build_client(cfg, args)
    _print_client_params(client, args.config)

    if args.prep:
        prep = os.path.join(PROJECT_ROOT, "scripts/ollama_prep_infer.sh")
        subprocess.run(["bash", prep, client.model], check=True)

    if not client.check_health():
        raise RuntimeError(
            f"Ollama not reachable or model missing at {client.url}. "
            f"Run: ollama serve && ollama pull {client.model}"
        )

    if args.recover:
        client.recover_stuck_server()

    do_warmup = args.warmup and not args.no_warmup
    if do_warmup:
        dt = client.warmup_full(
            frame_bgr=img,
            timeout=warmup_timeout,
            instruction=instruction,
        )
        print(f"[warmup] total {dt:.1f}s timeout={warmup_timeout}s", flush=True)
    else:
        print("[test_qwen] warmup skipped (default); infer latency is raw/cold/hot as-is", flush=True)

    result = client.infer_navigation(img, instruction)
    print(json.dumps(result, ensure_ascii=False, indent=2))

    print("\n--- summary ---")
    print(f"instruction={instruction}")
    print(f"usable={result.get('usable')} u={result.get('u')} v={result.get('v')}")
    print(f"raw=({result.get('_raw_u')},{result.get('_raw_v')}) coord_mode={client.coord_mode}")
    print(f"latency={result.get('_latency_sec', 0):.1f}s")
    print(
        f"ollama: total_ms={result.get('_ollama_total_ms', 0):.0f} "
        f"load_ms={result.get('_ollama_load_ms', 0):.0f} "
        f"prompt_eval_ms={result.get('_ollama_prompt_eval_ms', 0):.0f} "
        f"eval_ms={result.get('_ollama_eval_ms', 0):.0f} "
        f"eval_count={result.get('_ollama_eval_count', 0)}"
    )


if __name__ == "__main__":
    main()
