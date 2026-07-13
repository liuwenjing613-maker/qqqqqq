#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List

import cv2
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from qwen_vln.prompt_manager import PromptManager
from qwen_vln.qwen_client import ClientConfig, QwenVisionClient
from qwen_vln.types import PromptMode, VlnState
from qwen_vln.visualizer import ResultVisualizer


def resize_for_api(image, max_width: int, max_height: int = 0):
    height, width = image.shape[:2]
    scale = 1.0
    if max_width > 0 and width > max_width:
        scale = min(scale, max_width / float(width))
    if max_height > 0 and height > max_height:
        scale = min(scale, max_height / float(height))
    if scale >= 0.999:
        return image
    new_w = max(1, int(round(width * scale)))
    new_h = max(1, int(round(height * scale)))
    return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)


def _section(config: dict, *names: str) -> dict:
    for name in names:
        value = config.get(name)
        if isinstance(value, dict):
            return value
    return {}


def _state_for_result(result_name: str) -> VlnState:
    if result_name == "TARGET_VISIBLE":
        return VlnState.TARGET_LOCKED
    if result_name in {"TARGET_INFERRED", "VERIFY_FAILED"}:
        return VlnState.TARGET_INFERRED
    if result_name == "VERIFY_SUCCESS":
        return VlnState.SUCCESS
    return VlnState.SEARCHING


def _collect_images(paths: List[str]) -> List[Path]:
    images: List[Path] = []
    for raw in paths:
        path = Path(raw).expanduser().resolve()
        if path.is_dir():
            found = sorted(
                [
                    p
                    for p in path.iterdir()
                    if p.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
                ]
            )
            if not found:
                raise RuntimeError(f"No images found in directory: {path}")
            images.extend(found)
        elif path.is_file():
            images.append(path)
        else:
            raise RuntimeError(f"Image path not found: {path}")
    return images


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run continuous Qwen VL inference on multiple local images with one Client."
    )
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs/qwen3_vln_debug.yaml"))
    parser.add_argument(
        "--images",
        nargs="+",
        required=True,
        help="Image files and/or directories. Directories expand to sorted image files.",
    )
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--mode", choices=[m.value for m in PromptMode], default="search")
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs/batch"))
    parser.add_argument(
        "--warmup",
        choices=["auto", "on", "off"],
        default="auto",
        help="auto = follow yaml warmup.enabled; on/off force behavior",
    )
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    api_cfg = _section(config, "qwen", "api")
    camera_cfg = _section(config, "camera", "image")
    if not api_cfg or not camera_cfg:
        raise KeyError("config needs qwen/camera sections (or legacy api/image)")

    image_paths = _collect_images(args.images)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    client = QwenVisionClient(
        ClientConfig(
            model=api_cfg["model"],
            api_key_env=api_cfg["api_key_env"],
            base_url_env=api_cfg["base_url_env"],
            default_base_url=api_cfg["default_base_url"],
            timeout_sec=float(api_cfg["timeout_sec"]),
            max_retries=int(api_cfg["max_retries"]),
            temperature=float(api_cfg["temperature"]),
            max_tokens=int(api_cfg["max_tokens"]),
            enable_thinking=bool(api_cfg.get("enable_thinking", False)),
            jpeg_quality=int(api_cfg.get("jpeg_quality", 72)),
            min_pixels=int(api_cfg.get("min_pixels", 65536)),
            max_pixels=int(api_cfg.get("max_pixels", 442368)),
            vl_high_resolution_images=bool(api_cfg.get("vl_high_resolution_images", False)),
        )
    )

    do_warmup = (
        True
        if args.warmup == "on"
        else False
        if args.warmup == "off"
        else bool(config.get("warmup", {}).get("enabled", False))
    )
    if do_warmup:
        try:
            warmup_ms = client.warmup()
            print(f"[QWEN_WARMUP] completed: {warmup_ms:.0f} ms", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[QWEN_WARMUP] failed: {exc}", flush=True)

    mode = PromptMode(args.mode)
    prompt_manager = PromptManager(
        str(PROJECT_ROOT / str(config.get("prompts", {}).get("directory", "prompts")))
    )
    max_w = int(camera_cfg.get("api_max_width", 960))
    max_h = int(camera_cfg.get("api_max_height", 0))

    summaries = []
    latencies: List[float] = []

    for index, image_path in enumerate(image_paths, start=1):
        print(f"\n===== [{index}/{len(image_paths)}] {image_path.name} =====", flush=True)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Cannot read image: {image_path}")
        image = resize_for_api(image, max_w, max_h)
        height, width = image.shape[:2]
        prompt = prompt_manager.build(mode, args.instruction, width, height)
        result = client.infer(image, prompt, mode, request_id=index)
        latencies.append(float(result.latency_ms))

        stem = image_path.stem
        result_path = output_dir / f"{stem}_result.json"
        annotated_path = output_dir / f"{stem}_annotated.jpg"
        result_path.write_text(
            json.dumps(result.to_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        visualizer = ResultVisualizer(history_length=1)
        visualizer.add_result(result)
        annotated = visualizer.draw(
            image,
            _state_for_result(result.result),
            args.instruction,
            result,
            False,
            frame_note=f"batch {index}/{len(image_paths)}: {image_path.name}",
        )
        cv2.imwrite(str(annotated_path), annotated)

        # Keep latest_* aliases pointing at the most recent frame.
        (output_dir / "latest_result.json").write_text(
            result_path.read_text(encoding="utf-8"),
            encoding="utf-8",
        )
        cv2.imwrite(str(output_dir / "latest_annotated.jpg"), annotated)

        item = {
            "index": index,
            "image": str(image_path),
            "result": result.result,
            "point": None if result.point is None else {"x": result.point.x, "y": result.point.y},
            "confidence": result.confidence,
            "latency_ms": result.latency_ms,
            "image_width": result.image_width,
            "image_height": result.image_height,
            "result_json": str(result_path),
            "annotated_jpg": str(annotated_path),
        }
        summaries.append(item)
        print(json.dumps(item, ensure_ascii=False, indent=2), flush=True)

    steady = latencies[1:] if len(latencies) > 1 else []
    summary = {
        "instruction": args.instruction,
        "mode": mode.value,
        "count": len(summaries),
        "first_latency_ms": latencies[0] if latencies else None,
        "steady_avg_latency_ms": (
            sum(steady) / len(steady) if steady else None
        ),
        "all_avg_latency_ms": (
            sum(latencies) / len(latencies) if latencies else None
        ),
        "items": summaries,
    }
    summary_path = output_dir / "batch_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("\n===== BATCH SUMMARY =====", flush=True)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    print(f"saved: {summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
