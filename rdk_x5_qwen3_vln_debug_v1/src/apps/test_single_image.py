#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from qwen_vln.prompt_manager import PromptManager
from qwen_vln.qwen_client import ClientConfig, QwenVisionClient
from qwen_vln.types import PromptMode, VlnState
from qwen_vln.visualizer import ResultVisualizer


def resize_for_api(image, max_width: int):
    height, width = image.shape[:2]
    if width <= max_width:
        return image
    scale = max_width / float(width)
    return cv2.resize(image, (max_width, max(1, int(round(height * scale)))), interpolation=cv2.INTER_AREA)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs/qwen3_vln_debug.yaml"))
    parser.add_argument("--image", required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--mode", choices=[m.value for m in PromptMode], default="observe")
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs"))
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Cannot read image: {args.image}")
    image = resize_for_api(image, int(config["image"]["api_max_width"]))
    height, width = image.shape[:2]
    api_cfg = config["api"]
    client = QwenVisionClient(ClientConfig(
        model=api_cfg["model"], api_key_env=api_cfg["api_key_env"], base_url_env=api_cfg["base_url_env"],
        default_base_url=api_cfg["default_base_url"], timeout_sec=float(api_cfg["timeout_sec"]),
        max_retries=int(api_cfg["max_retries"]), temperature=float(api_cfg["temperature"]),
        max_tokens=int(api_cfg["max_tokens"]), enable_thinking=bool(api_cfg["enable_thinking"]),
        jpeg_quality=int(config["image"]["jpeg_quality"]),
    ))
    mode = PromptMode(args.mode)
    prompt_dir = PROJECT_ROOT / str(config.get("prompts", {}).get("directory", "prompts"))
    prompt = PromptManager(str(prompt_dir)).build(mode, args.instruction, width, height)
    result = client.infer(image, prompt, mode, request_id=1)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    result_path = output_dir / "latest_result.json"
    image_path = output_dir / "latest_annotated.jpg"
    result_path.write_text(json.dumps(result.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
    state_by_mode = {
        PromptMode.OBSERVE: VlnState.TARGET_LOCKED if result.result == "TARGET_VISIBLE" else VlnState.OBSERVE,
        PromptMode.TRACK: VlnState.TARGET_LOCKED if result.result == "TARGET_VISIBLE" else VlnState.SEARCHING,
        PromptMode.SEARCH: (
            VlnState.TARGET_LOCKED
            if result.result == "TARGET_VISIBLE"
            else VlnState.TARGET_INFERRED
            if result.result == "SEARCH_HINT"
            else VlnState.SEARCHING
        ),
        PromptMode.VERIFY: VlnState.SUCCESS if result.result == "VERIFY_SUCCESS" else VlnState.VERIFY,
    }
    visualizer = ResultVisualizer(history_length=1)
    visualizer.add_result(result)
    annotated = visualizer.draw(
        image,
        state_by_mode[mode],
        args.instruction,
        result,
        False,
        frame_note="exact local input image",
    )
    cv2.imwrite(str(image_path), annotated)
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
    print(f"saved: {result_path}")
    print(f"saved: {image_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
