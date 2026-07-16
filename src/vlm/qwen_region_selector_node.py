#!/usr/bin/env python3
"""ROS 2 dry-run node: Qwen region selection from frozen snapshots only."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from src.vlm.qwen_region_api import call_qwen_region_selection  # noqa: E402
from src.vlm.qwen_region_selector_core import (  # noqa: E402
    QwenRequestManifest,
    STRATEGY_CANDIDATE_RANKING,
    STRATEGY_GLOBAL_REGION_PROPOSAL,
    build_prompt_for_strategy,
    build_region_selection_prompt,
    decision_from_validation,
    decision_to_dict,
    fuse_geometric_and_qwen_ranking,
    input_to_dict,
    labels_from_input,
    load_region_snapshot,
    revalidate_decision,
    select_exploration_region,
    unified_decision_dict,
    validate_qwen_decision,
    validate_region_snapshot,
)
from src.vlm.qwen_global_region_selector_core import validate_region_selection_config  # noqa: E402


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class QwenRegionSelectorNode(Node):
    NODE_NAME = "qwen_region_selector"

    def __init__(self, cfg: Dict[str, Any], run_dir: Path) -> None:
        super().__init__(self.NODE_NAME)
        self.cfg = cfg
        self.run_dir = run_dir
        ros_cfg = cfg.get("ros", {})
        self.max_snapshot_age_s = float(ros_cfg.get("max_snapshot_age_s", 300))

        self._latest_snapshot: Optional[Dict[str, Any]] = None
        self._latest_snapshot_time: float = 0.0
        self._snapshot_at_request: Optional[Dict[str, Any]] = None
        self._latest_instruction: str = ""
        self._request_lock = threading.Lock()
        self._request_running = False

        snap_topic = str(ros_cfg.get("snapshot_topic", "/qwen_explore_debug/region_snapshot_json"))
        instr_topic = str(ros_cfg.get("instruction_topic", "/qwen_explore/target_instruction"))
        self.create_subscription(String, snap_topic, self._snapshot_cb, 10)
        self.create_subscription(String, instr_topic, self._instruction_cb, 10)

        self.pub_status = self.create_publisher(
            String, str(ros_cfg.get("status_topic", "/qwen_explore/region_selection_status")), 10
        )
        self.pub_decision = self.create_publisher(
            String, str(ros_cfg.get("decision_topic", "/qwen_explore/region_decision_json")), 10
        )
        self.pub_debug = self.create_publisher(
            String, str(ros_cfg.get("debug_topic", "/qwen_explore/region_selection_debug_json")), 10
        )

        service_name = str(ros_cfg.get("select_service", "/qwen_explore/select_region"))
        self.create_service(Trigger, service_name, self._select_cb)

        self.get_logger().info(
            f"[SELECTOR] dry_run=true motion_enabled=false nav2_enabled=false run_dir={run_dir}"
        )
        cfg_errors = validate_region_selection_config(cfg)
        if cfg_errors:
            self.get_logger().error(f"[SELECTOR] config errors: {cfg_errors}")

    def _snapshot_cb(self, msg: String) -> None:
        try:
            self._latest_snapshot = json.loads(msg.data)
            self._latest_snapshot_time = time.time()
        except json.JSONDecodeError:
            self.get_logger().warn("[SELECTOR] invalid snapshot JSON on topic")

    def _instruction_cb(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if text:
            self._latest_instruction = text
            self.get_logger().info(f"[SELECTOR] instruction_received len={len(text)}")

    def _publish_status(self, payload: Dict[str, Any]) -> None:
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub_status.publish(m)

    def _select_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if not self._request_lock.acquire(blocking=False):
            resp.success = False
            resp.message = "SELECTOR_REQUEST_ALREADY_RUNNING"
            return resp

        self._request_running = True
        try:
            return self._run_selection(resp)
        finally:
            self._request_running = False
            self._request_lock.release()

    def _run_selection(self, resp: Trigger.Response) -> Trigger.Response:
        if self._latest_snapshot is None:
            resp.success = False
            resp.message = "SELECTOR_NO_SNAPSHOT"
            return resp
        if not self._latest_instruction.strip():
            resp.success = False
            resp.message = "SELECTOR_NO_INSTRUCTION"
            return resp

        age = time.time() - self._latest_snapshot_time
        if age > self.max_snapshot_age_s:
            resp.success = False
            resp.message = f"SELECTOR_SNAPSHOT_STALE age_s={age:.1f}"
            return resp

        snap = self._latest_snapshot
        snapshot_id = str(snap.get("snapshot_id", ""))
        annotated = str(snap.get("annotated_map_file", ""))

        # Write temp snapshot file for loader
        tmp_snap = self.run_dir / "latest_snapshot_for_select.json"
        tmp_snap.write_text(json.dumps(snap, ensure_ascii=False, indent=2), encoding="utf-8")

        try:
            inp, raw_snap = load_region_snapshot(tmp_snap, self._latest_instruction)
        except ValueError as exc:
            resp.success = False
            resp.message = str(exc)
            return resp

        snap_errors = validate_region_snapshot(inp)
        if snap_errors:
            resp.success = False
            resp.message = snap_errors[0]
            return resp

        strategy = str(
            self.cfg.get("region_selection", {}).get("strategy", STRATEGY_CANDIDATE_RANKING)
        )
        map_render_metadata = raw_snap.get("map_render_metadata") or {}
        decision_board = str(raw_snap.get("decision_board_file") or inp.decision_board_file or "")
        image_path = Path(decision_board if strategy == STRATEGY_GLOBAL_REGION_PROPOSAL and decision_board else (annotated or inp.annotated_map_file))
        if not image_path.is_file():
            image_path = Path(annotated or inp.annotated_map_file)
        if not image_path.is_file():
            resp.success = False
            resp.message = "SELECTOR_SNAPSHOT_FILE_MISSING"
            return resp

        call_id = f"ros_{datetime.now().strftime('%H%M%S')}"
        call_dir = self.run_dir / "calls" / call_id
        call_dir.mkdir(parents=True, exist_ok=True)
        prompt = build_prompt_for_strategy(
            strategy,
            inp,
            self.cfg,
            raw_snap,
            map_render_metadata=map_render_metadata,
            visual_context_id=inp.visual_context_id,
        )
        prompt_name = (
            "global_region_prompt.txt"
            if strategy == STRATEGY_GLOBAL_REGION_PROPOSAL
            else "prompt.txt"
        )
        (call_dir / prompt_name).write_text(prompt, encoding="utf-8")
        (call_dir / "target_instruction.txt").write_text(self._latest_instruction, encoding="utf-8")
        shutil.copy2(image_path, call_dir / image_path.name)

        t0 = time.perf_counter()
        self._snapshot_at_request = json.loads(json.dumps(snap))
        api_result = call_qwen_region_selection(prompt, image_path, self.cfg)
        if api_result.error_code:
            err = {
                "error_code": api_result.error_code,
                "error_message": api_result.error_message,
            }
            (call_dir / "error.json").write_text(json.dumps(err, indent=2), encoding="utf-8")
            resp.success = False
            resp.message = f"SELECTOR_API_FAILED code={api_result.error_code}"
            return resp

        (call_dir / "raw_response.txt").write_text(api_result.raw_response, encoding="utf-8")
        if strategy == STRATEGY_GLOBAL_REGION_PROPOSAL and raw_snap.get("map_data"):
            from src.planning.frontier_region_debug_core import MapMetadata, analyze_frontier_regions  # noqa: E402
            import numpy as np

            meta_d = raw_snap.get("map_metadata", {})
            meta = MapMetadata(
                width=int(meta_d.get("width", 0)),
                height=int(meta_d.get("height", 0)),
                resolution=float(meta_d.get("resolution", 0.05)),
                origin_x=float(meta_d.get("origin_x", 0.0)),
                origin_y=float(meta_d.get("origin_y", 0.0)),
                frame_id=str(meta_d.get("frame_id", "map")),
                stamp_sec=float(meta_d.get("stamp_sec", 0.0)),
            )
            data = np.array(raw_snap["map_data"], dtype=np.int16).reshape(meta.height, meta.width)
            robot = inp.robot_pose
            from src.planning.frontier_region_debug_core import RobotPose2D  # noqa: E402

            analysis = analyze_frontier_regions(
                data,
                meta,
                RobotPose2D(float(robot.get("x", 0)), float(robot.get("y", 0)), math.radians(float(robot.get("yaw_deg", 0)))),
                self.cfg,
                cycle_id=int(raw_snap.get("cycle_id", 0)),
            )
            pipeline = select_exploration_region(
                strategy,
                inp=inp,
                raw_snapshot=raw_snap,
                raw_response=api_result.raw_response,
                cfg=self.cfg,
                map_data=raw_snap["map_data"],
                analysis_result=analysis,
                map_meta=meta,
                map_render_metadata=map_render_metadata,
            )
            validation = pipeline.get("json_validation") or pipeline.get("validation")
            decision_dict = unified_decision_dict(
                pipeline,
                snapshot_id=inp.snapshot_id,
                visual_context_id=inp.visual_context_id,
            )
            decision_valid = bool(
                pipeline.get("proposal_validated")
                or (
                    validation.decision_valid
                    if validation is not None and hasattr(validation, "decision_valid")
                    else False
                )
            )
            if pipeline.get("fallback_used"):
                (call_dir / "global_region_fallback.json").write_text(
                    json.dumps(
                        {
                            "configured_strategy": pipeline.get("configured_strategy"),
                            "effective_strategy": pipeline.get("effective_strategy"),
                            "fallback_reason": pipeline.get("fallback_reason"),
                            "global_proposal_failures": pipeline.get("global_proposal_failures", []),
                        },
                        indent=2,
                    ),
                    encoding="utf-8",
                )
        else:
            validation = validate_qwen_decision(
                api_result.raw_response,
                inp.snapshot_id,
                labels_from_input(inp),
            )
            decision = decision_from_validation(validation, inp.snapshot_id)
            fusion = {}
            if validation.decision_valid and validation.parsed:
                fusion = fuse_geometric_and_qwen_ranking(inp.regions, validation.parsed, self.cfg)
                rev_errors = revalidate_decision(
                    self._snapshot_at_request or snap,
                    self._latest_snapshot or snap,
                    fusion.get("algorithm_final_region"),
                    {r.label: {"stable": r.stable, "snapshot_eligible": r.snapshot_eligible} for r in inp.regions},
                    self.cfg,
                )
                if rev_errors:
                    validation.decision_valid = False
                    validation.errors.extend(rev_errors)
                    fusion["algorithm_final_region"] = None
            decision_dict = decision_to_dict(
                decision,
                decision_valid=validation.decision_valid,
                validation_errors=validation.errors,
                unsupported_visual_claims=validation.unsupported_visual_claims,
            )
            decision_dict.update(fusion)
            decision_dict["configured_strategy"] = STRATEGY_CANDIDATE_RANKING
            decision_dict["effective_strategy"] = STRATEGY_CANDIDATE_RANKING
            if fusion.get("algorithm_final_region"):
                decision_dict["selected_region"] = fusion["algorithm_final_region"]
                decision_dict["fallback_regions"] = [
                    x["label"] for x in fusion.get("fusion_scores", [])[1:3]
                ]
            decision_valid = validation.decision_valid

        parsed_response = None
        validation_errors: list = []
        unsupported_claims: list = []
        if hasattr(validation, "parsed"):
            parsed_response = validation.parsed
            validation_errors = list(getattr(validation, "errors", []))
            unsupported_claims = list(getattr(validation, "unsupported_visual_claims", []))
        elif validation is not None and hasattr(validation, "parsed"):
            parsed_response = validation.parsed
            validation_errors = list(getattr(validation, "errors", []))

        debug_payload = {
            "call_id": call_id,
            "snapshot_id": snapshot_id,
            "target_instruction": self._latest_instruction,
            "configured_strategy": decision_dict.get("configured_strategy", strategy),
            "effective_strategy": decision_dict.get("effective_strategy", strategy),
            "raw_response": api_result.raw_response,
            "parsed_response": parsed_response,
            "validation": {
                "decision_valid": decision_valid,
                "errors": validation_errors,
                "unsupported_visual_claims": unsupported_claims,
            },
            "timing": {
                "request_latency_ms": api_result.request_latency_ms,
                "total_latency_ms": (time.perf_counter() - t0) * 1000.0,
                "retry_count": api_result.retry_count,
            },
            "model": api_result.model,
        }

        (call_dir / "decision.json").write_text(
            json.dumps(decision_dict, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        (self.run_dir / "latest_decision.json").write_text(
            json.dumps(decision_dict, ensure_ascii=False, indent=2), encoding="utf-8"
        )

        msg_dec = String()
        msg_dec.data = json.dumps(decision_dict, ensure_ascii=False)
        self.pub_decision.publish(msg_dec)

        msg_dbg = String()
        msg_dbg.data = json.dumps(debug_payload, ensure_ascii=False)
        self.pub_debug.publish(msg_dbg)

        self._publish_status(
            {
                "status": "SUCCESS" if decision_valid else "INVALID",
                "snapshot_id": snapshot_id,
                "selected_region": decision_dict.get("algorithm_final_region"),
                "decision_valid": decision_valid,
                "configured_strategy": decision_dict.get("configured_strategy", strategy),
                "effective_strategy": decision_dict.get("effective_strategy", strategy),
                "timestamp": _utc_now_iso(),
            }
        )

        self.get_logger().info(
            f"[SELECTOR] snapshot={snapshot_id} strategy={strategy} "
            f"valid={decision_valid} selected={decision_dict.get('algorithm_final_region')} "
            f"motion_executed=false"
        )

        if not decision_valid:
            resp.success = False
            resp.message = "SELECTOR_RESPONSE_INVALID"
            return resp

        resp.success = True
        resp.message = (
            f"snapshot_id={snapshot_id} selected={decision_dict.get('algorithm_final_region')} "
            f"strategy={decision_dict.get('effective_strategy', strategy)}"
        )
        return resp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = QwenRegionSelectorNode(cfg, run_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
