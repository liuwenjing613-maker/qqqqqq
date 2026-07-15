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
from qwen_vln.visualizer import ResultVisualizer, SpawnScanHud


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs/qwen3_vln_debug.yaml"))
    parser.add_argument("--image", required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--mode", choices=[m.value for m in PromptMode], default="observe")
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs"))
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))

    def section(*names: str) -> dict:
        for name in names:
            value = config.get(name)
            if isinstance(value, dict):
                return value
        return {}

    api_cfg = section("qwen", "api")
    camera_cfg = section("camera", "image")
    if not api_cfg or not camera_cfg:
        raise KeyError("config needs qwen/camera sections (or legacy api/image)")

    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Cannot read image: {args.image}")
    image = resize_for_api(
        image,
        int(camera_cfg.get("api_max_width", 960)),
        int(camera_cfg.get("api_max_height", 0)),
    )
    height, width = image.shape[:2]
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
            max_tokens_spawn_scan=int(
                api_cfg.get(
                    "max_tokens_spawn_scan",
                    api_cfg["max_tokens"],
                )
            ),
            enable_thinking=bool(api_cfg["enable_thinking"]),
            jpeg_quality=int(api_cfg.get("jpeg_quality", 72)),
            min_pixels=int(api_cfg.get("min_pixels", 65536)),
            max_pixels=int(api_cfg.get("max_pixels", 442368)),
            vl_high_resolution_images=bool(api_cfg.get("vl_high_resolution_images", False)),
        )
    )
    mode = PromptMode(args.mode)
    prompt_dir = PROJECT_ROOT / str(config.get("prompts", {}).get("directory", "prompts"))
    prompt = PromptManager(str(prompt_dir)).build(mode, args.instruction, width, height)
    result = client.infer(image, prompt, mode, request_id=1)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    result_path = output_dir / "latest_result.json"
    image_path = output_dir / "latest_annotated.jpg"
    result_path.write_text(json.dumps(result.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")

    def _state_for_result(result_name: str) -> VlnState:
        if result_name == "TARGET_VISIBLE":
            return VlnState.TARGET_LOCKED
        if result_name == "TARGET_INFERRED":
            return VlnState.TARGET_INFERRED
        if result_name == "VERIFY_SUCCESS":
            return VlnState.SUCCESS
        if result_name == "VERIFY_FAILED":
            return VlnState.TARGET_INFERRED
        return VlnState.SEARCHING

    state_by_mode = {
        PromptMode.SPAWN_SCAN: VlnState.SPAWN_SCAN,
        PromptMode.OBSERVE: _state_for_result(result.result),
        PromptMode.TRACK: _state_for_result(result.result),
        PromptMode.SEARCH: _state_for_result(result.result),
        PromptMode.VERIFY: _state_for_result(result.result),
    }
    visualizer = ResultVisualizer(history_length=1)
    visualizer.add_result(result)
    spawn_hud = None
    if mode == PromptMode.SPAWN_SCAN:
        spawn_hud = SpawnScanHud(
            phase="DWELL",
            sector=0,
            scores=[float(result.score_q)],
            best_sector=0,
            sector_deg=60.0,
        )
    annotated = visualizer.draw(
        image,
        state_by_mode[mode],
        args.instruction,
        result,
        False,
        spawn_scan=spawn_hud,
    )
    cv2.imwrite(str(image_path), annotated)
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
    print(f"saved: {result_path}")
    print(f"saved: {image_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
