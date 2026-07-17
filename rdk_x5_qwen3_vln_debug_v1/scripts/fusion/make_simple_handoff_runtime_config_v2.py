#!/usr/bin/env python3
"""Create runtime copies without rewriting the proven checked-in configs."""
from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict

import yaml


def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    out = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = deepcopy(value)
    return out


def load_mapping(path: str) -> Dict[str, Any]:
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise SystemExit(f"YAML root must be a mapping: {path}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--servo-config", required=True)
    parser.add_argument("--fusion-config", required=True)
    parser.add_argument("--simple-config", required=True)
    parser.add_argument("--backend-config", required=True)
    parser.add_argument("--servo-output", required=True)
    parser.add_argument("--backend-output", required=True)
    parser.add_argument("--backend-debug-dir", required=True)
    args = parser.parse_args()

    servo = load_mapping(args.servo_config)
    fusion_root = load_mapping(args.fusion_config)
    simple_root = load_mapping(args.simple_config)
    backend = load_mapping(args.backend_config)

    fusion = deepcopy(fusion_root.get("online_map_plan_fusion", {}) or {})
    simple = deepcopy(simple_root.get("simple_handoff_v2", simple_root) or {})
    if not isinstance(fusion, dict) or not isinstance(simple, dict):
        raise SystemExit("fusion/simple config roots must be mappings")

    topics = deepcopy(simple.get("topics", {}) or {})
    servo_topics = servo.setdefault("topics", {})
    servo_topics["cmd_output"] = str(topics.get("ego_cmd", "/cmd_vel_ego"))

    # The mux already exists and is battle-tested. We enable only its config;
    # the old complex intervention node is deliberately not launched.
    third = deepcopy(servo.get("third_view_intervention", {}) or {})
    third["enabled"] = True
    third_topics = deepcopy(third.get("topics", {}) or {})
    third_topics.update(
        {
            "servo_status": str(
                topics.get("servo_status", "/qwen_vln/servo/status")
            ),
            "odom": str(topics.get("odom", "/odom")),
            "request": str(
                topics.get("intervention_request", "/third_view/intervention/request")
            ),
            "cancel": str(
                topics.get("intervention_cancel", "/third_view/intervention/cancel")
            ),
            "navigation_status": str(
                topics.get("navigation_status", "/third_view/navigation_status")
            ),
            "control_mode": str(
                topics.get("control_mode", "/third_view/intervention/control_mode")
            ),
            "qwen_command": str(topics.get("qwen_command", "/qwen_vln/command")),
            "ego_cmd": str(topics.get("ego_cmd", "/cmd_vel_ego")),
            "map_cmd": str(topics.get("map_cmd", "/cmd_vel_map")),
            "mux_output": str(topics.get("mux_output", "/cmd_vel_autonomy")),
        }
    )
    third["topics"] = third_topics
    third["cmd_mux"] = deep_merge(
        third.get("cmd_mux", {}) or {}, simple.get("cmd_mux", {}) or {}
    )
    servo["third_view_intervention"] = third
    fusion = deep_merge(servo.get("online_map_plan_fusion", {}) or {}, fusion)
    probe = fusion.setdefault("candidate_probe", {})
    probe["enabled"] = bool(
        (simple.get("handoff", {}) or {}).get(
            "backend_candidate_probe_enabled", False
        )
    )
    servo["online_map_plan_fusion"] = fusion
    servo["simple_handoff_v2"] = simple

    backend_root = backend.setdefault("online_map_plan_fullflow_v2", {})
    if not isinstance(backend_root, dict):
        raise SystemExit("online_map_plan_fullflow_v2 must be a mapping")
    backend_root["debug_dir"] = str(Path(args.backend_debug_dir).resolve())

    servo_out = Path(args.servo_output)
    backend_out = Path(args.backend_output)
    servo_out.parent.mkdir(parents=True, exist_ok=True)
    backend_out.parent.mkdir(parents=True, exist_ok=True)
    servo_out.write_text(
        yaml.safe_dump(servo, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    backend_out.write_text(
        yaml.safe_dump(backend, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(servo_out)
    print(backend_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
