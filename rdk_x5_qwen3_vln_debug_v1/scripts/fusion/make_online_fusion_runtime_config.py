#!/usr/bin/env python3
"""Merge the separate fusion overlay into a temporary servo configuration."""
from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict

import yaml


def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--servo-config", required=True)
    parser.add_argument("--fusion-config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--enable", choices=("true", "false"), default="true")
    args = parser.parse_args()

    servo_path = Path(args.servo_config)
    fusion_path = Path(args.fusion_config)
    base = yaml.safe_load(servo_path.read_text(encoding="utf-8")) or {}
    fusion_root = yaml.safe_load(fusion_path.read_text(encoding="utf-8")) or {}
    fusion = deepcopy(fusion_root.get("online_map_plan_fusion", {}) or {})

    if not isinstance(base, dict) or not isinstance(fusion, dict):
        raise SystemExit("config roots must be mappings")

    # The existing intervention block remains the source of its tuned trigger
    # parameters.  This runtime copy only enables it and aligns its topics with
    # the bridge.  The checked-in V1 config is never rewritten.
    enabled = args.enable == "true"
    fusion["enabled"] = enabled

    third = deepcopy(base.get("third_view_intervention", {}) or {})
    third["enabled"] = enabled
    third_topics = deepcopy(third.get("topics", {}) or {})
    fusion_topics = fusion.get("topics", {}) or {}
    third_topics.update(
        {
            "candidate_summary": fusion_topics.get(
                "intervention_candidate_summary", "/third_view/candidate_summary"
            ),
            "navigation_status": fusion_topics.get(
                "intervention_navigation_status", "/third_view/navigation_status"
            ),
            "request": fusion_topics.get(
                "intervention_request", "/third_view/intervention/request"
            ),
            "cancel": fusion_topics.get(
                "intervention_cancel", "/third_view/intervention/cancel"
            ),
            "map_cmd": fusion_topics.get("map_cmd_output", "/cmd_vel_map"),
        }
    )
    third["topics"] = third_topics
    base["third_view_intervention"] = third
    base["online_map_plan_fusion"] = deep_merge(
        base.get("online_map_plan_fusion", {}) or {}, fusion
    )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(base, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(output)


if __name__ == "__main__":
    main()
