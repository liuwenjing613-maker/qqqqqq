#!/usr/bin/env python3
"""Offline dry-run CLI for Qwen region selection."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from src.vlm.qwen_region_api import call_qwen_region_selection  # noqa: E402
from src.vlm.qwen_region_selector_core import (  # noqa: E402
    QwenRequestManifest,
    build_region_selection_prompt,
    decision_from_validation,
    decision_to_dict,
    input_to_dict,
    labels_from_input,
    load_region_snapshot,
    validate_qwen_decision,
    validate_region_snapshot,
)


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _load_config(path: Path) -> Dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _run_single_call(
    *,
    call_id: str,
    run_id: str,
    snapshot_path: Path,
    instruction: str,
    cfg: Dict[str, Any],
    output_dir: Path,
    mock_response: Optional[str] = None,
    no_api: bool = False,
) -> Dict[str, Any]:
    call_dir = output_dir / call_id
    call_dir.mkdir(parents=True, exist_ok=True)
    t_total_start = time.perf_counter()

    inp, raw_snap = load_region_snapshot(snapshot_path, instruction)
    snap_errors = validate_region_snapshot(inp)
    if snap_errors:
        err = {
            "call_id": call_id,
            "errors": snap_errors,
            "decision_valid": False,
        }
        _write_json(call_dir / "error.json", err)
        _write_json(call_dir / "validation.json", err)
        _write_json(
            call_dir / "decision.json",
            {
                "snapshot_id": inp.snapshot_id if inp.snapshot_id else "",
                "selected_region": None,
                "fallback_regions": [],
                "confidence": 0.0,
                "reason_code": "",
                "evidence": [],
                "decision_valid": False,
                "motion_executed": False,
                "nav2_called": False,
                "validation_errors": snap_errors,
            },
        )
        return {"exit_code": 2, "summary": err}

    _write_json(call_dir / "snapshot_input.json", {**input_to_dict(inp), "raw_snapshot": raw_snap})
    (call_dir / "target_instruction.txt").write_text(instruction, encoding="utf-8")

    map_src = Path(inp.annotated_map_file)
    if map_src.is_file():
        shutil.copy2(map_src, call_dir / "annotated_map.png")

    prompt = build_region_selection_prompt(inp)
    if cfg.get("logging", {}).get("save_prompt", True):
        (call_dir / "prompt.txt").write_text(prompt, encoding="utf-8")

    valid_labels = sorted(labels_from_input(inp))
    manifest = QwenRequestManifest(
        run_id=run_id,
        call_id=call_id,
        snapshot_id=inp.snapshot_id,
        model=str(cfg.get("qwen", {}).get("default_model", "")),
        target_instruction=instruction,
        valid_region_labels=valid_labels,
        image_file=str(map_src),
        request_start_time=_utc_now_iso(),
    )

    api_result = None
    raw_response = ""
    request_latency_ms = 0.0
    retry_count = 0

    if mock_response is not None:
        raw_response = Path(mock_response).read_text(encoding="utf-8")
    elif no_api:
        raw_response = ""
    else:
        api_result = call_qwen_region_selection(prompt, map_src, cfg)
        manifest.model = api_result.model
        raw_response = api_result.raw_response
        request_latency_ms = api_result.request_latency_ms
        retry_count = api_result.retry_count
        if api_result.error_code:
            err_payload = {
                "error_code": api_result.error_code,
                "error_message": api_result.error_message,
                "decision_valid": False,
            }
            _write_json(call_dir / "error.json", err_payload)
            _write_json(call_dir / "request_manifest.json", manifest.__dict__)
            _write_json(
                call_dir / "timing.json",
                {
                    "request_latency_ms": request_latency_ms,
                    "total_latency_ms": (time.perf_counter() - t_total_start) * 1000.0,
                    "retry_count": retry_count,
                },
            )
            return {"exit_code": 3, "summary": err_payload, "call_dir": str(call_dir)}

    if cfg.get("logging", {}).get("save_request_manifest", True):
        _write_json(call_dir / "request_manifest.json", manifest.__dict__)

    (call_dir / "raw_response.txt").write_text(raw_response, encoding="utf-8")

    validation = validate_qwen_decision(
        raw_response,
        inp.snapshot_id,
        labels_from_input(inp),
    )
    decision = decision_from_validation(validation, inp.snapshot_id)
    decision_dict = decision_to_dict(
        decision,
        decision_valid=validation.decision_valid,
        validation_errors=validation.errors,
        unsupported_visual_claims=validation.unsupported_visual_claims,
    )

    parsed_path = call_dir / "parsed_response.json"
    if validation.parsed is not None:
        _write_json(parsed_path, validation.parsed)
    else:
        _write_json(parsed_path, {})

    _write_json(
        call_dir / "validation.json",
        {
            "decision_valid": validation.decision_valid,
            "errors": validation.errors,
            "unsupported_visual_claims": validation.unsupported_visual_claims,
        },
    )
    _write_json(call_dir / "decision.json", decision_dict)

    timing = {
        "request_latency_ms": request_latency_ms,
        "total_latency_ms": (time.perf_counter() - t_total_start) * 1000.0,
        "retry_count": retry_count,
    }
    if api_result is not None:
        timing["prompt_tokens"] = api_result.prompt_tokens
        timing["completion_tokens"] = api_result.completion_tokens
        timing["total_tokens"] = api_result.total_tokens
    _write_json(call_dir / "timing.json", timing)

    exit_code = 0 if validation.decision_valid else 4
    return {
        "exit_code": exit_code,
        "summary": decision_dict,
        "validation": validation,
        "timing": timing,
        "call_dir": str(call_dir),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Qwen region selector dry-run CLI")
    parser.add_argument("--snapshot-json", required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--config", default="configs/qwen_region_selector.yaml")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--mock-response", default="")
    parser.add_argument("--no-api", action="store_true")
    args = parser.parse_args()

    cfg = _load_config((PROJECT_ROOT / args.config).resolve())
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = cfg.get("logging", {}).get("root_dir", "logs/qwen_region_selection")
    output_dir = Path(args.output_dir) if args.output_dir else PROJECT_ROOT / root / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "calls").mkdir(exist_ok=True)

    meta = {
        "run_id": run_id,
        "mode": cfg.get("selector", {}).get("mode", "dry_run"),
        "snapshot_json": str(Path(args.snapshot_json).resolve()),
        "instruction": args.instruction,
        "repeat": args.repeat,
        "start_time": _utc_now_iso(),
    }
    _write_json(output_dir / "run_meta.json", meta)
    shutil.copy2(PROJECT_ROOT / args.config, output_dir / "resolved_config.yaml")

    mock = args.mock_response.strip() or None
    results: List[Dict[str, Any]] = []
    worst_exit = 0

    for i in range(args.repeat):
        call_id = f"call_{i + 1:03d}"
        result = _run_single_call(
            call_id=call_id,
            run_id=run_id,
            snapshot_path=Path(args.snapshot_json),
            instruction=args.instruction,
            cfg=cfg,
            output_dir=output_dir / "calls",
            mock_response=mock,
            no_api=args.no_api,
        )
        results.append(result)
        worst_exit = max(worst_exit, int(result.get("exit_code", 1)))
        summary = result.get("summary", {})
        _write_json(output_dir / "latest_decision.json", summary)
        _write_json(
            output_dir / "latest_validation.json",
            result.get("validation", {}).__dict__
            if hasattr(result.get("validation"), "__dict__")
            else result.get("summary", {}),
        )
        with (output_dir / "decision_history.jsonl").open("a", encoding="utf-8") as fh:
            fh.write(json.dumps({"call_id": call_id, **summary}, ensure_ascii=False) + "\n")

    _write_json(output_dir / "batch_summary.json", {"results": [r.get("summary") for r in results]})
    print(f"[CLI] run_id={run_id} output_dir={output_dir} calls={args.repeat} worst_exit={worst_exit}")
    return worst_exit


if __name__ == "__main__":
    raise SystemExit(main())
