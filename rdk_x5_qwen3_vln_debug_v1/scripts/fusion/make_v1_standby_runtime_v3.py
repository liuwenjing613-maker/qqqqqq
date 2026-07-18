#!/usr/bin/env python3
"""Create standby-only runtime copies without modifying checked-in configs."""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, MutableMapping

import yaml


def load_mapping(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise SystemExit(f"YAML root must be a mapping: {path}")
    return value


def require_mapping(parent: MutableMapping[str, Any], key: str) -> MutableMapping[str, Any]:
    value = parent.get(key)
    if value is None:
        value = {}
        parent[key] = value
    if not isinstance(value, dict):
        raise SystemExit(f"YAML key must be a mapping: {key}")
    return value


def write_yaml(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(value, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--qwen-input", required=True)
    parser.add_argument("--qwen-output", required=True)
    parser.add_argument("--simple-input", required=True)
    parser.add_argument("--simple-output", required=True)
    args = parser.parse_args()

    qwen_input = Path(args.qwen_input).resolve()
    qwen_output = Path(args.qwen_output).resolve()
    simple_input = Path(args.simple_input).resolve()
    simple_output = Path(args.simple_output).resolve()

    for path in (qwen_input, simple_input):
        if not path.is_file():
            raise SystemExit(f"missing input file: {path}")

    qwen = load_mapping(qwen_input)
    warmup = require_mapping(qwen, "warmup")
    warmup["enabled"] = True
    state_machine = require_mapping(qwen, "state_machine")
    state_machine["initial_instruction"] = "standby"
    state_machine["auto_enter_search"] = False

    simple = load_mapping(simple_input)
    simple_root_raw = simple.get("simple_handoff_v2", simple)
    if not isinstance(simple_root_raw, dict):
        raise SystemExit("simple_handoff_v2 must be a mapping")
    cmd_mux = require_mapping(simple_root_raw, "cmd_mux")
    cmd_mux["default_mode"] = "HOLD"

    write_yaml(qwen_output, qwen)
    write_yaml(simple_output, simple)

    print(f"qwen_runtime={qwen_output}")
    print(f"simple_runtime={simple_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
