#!/usr/bin/env python3
"""ROS 2 dry-run node: Qwen region selection from frozen snapshots only."""

from __future__ import annotations

import argparse
import json
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
            inp, _ = load_region_snapshot(tmp_snap, self._latest_instruction)
        except ValueError as exc:
            resp.success = False
            resp.message = str(exc)
            return resp

        snap_errors = validate_region_snapshot(inp)
        if snap_errors:
            resp.success = False
            resp.message = snap_errors[0]
            return resp

        map_path = Path(annotated or inp.annotated_map_file)
        if not map_path.is_file():
            resp.success = False
            resp.message = "SELECTOR_SNAPSHOT_FILE_MISSING"
            return resp

        call_id = f"ros_{datetime.now().strftime('%H%M%S')}"
        call_dir = self.run_dir / "calls" / call_id
        call_dir.mkdir(parents=True, exist_ok=True)
        prompt = build_region_selection_prompt(inp)
        (call_dir / "prompt.txt").write_text(prompt, encoding="utf-8")
        (call_dir / "target_instruction.txt").write_text(self._latest_instruction, encoding="utf-8")
        shutil.copy2(map_path, call_dir / "annotated_map.png")

        t0 = time.perf_counter()
        api_result = call_qwen_region_selection(prompt, map_path, self.cfg)
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
        validation = validate_qwen_decision(
            api_result.raw_response,
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

        debug_payload = {
            "call_id": call_id,
            "snapshot_id": snapshot_id,
            "target_instruction": self._latest_instruction,
            "raw_response": api_result.raw_response,
            "parsed_response": validation.parsed,
            "validation": {
                "decision_valid": validation.decision_valid,
                "errors": validation.errors,
                "unsupported_visual_claims": validation.unsupported_visual_claims,
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
                "status": "SUCCESS" if validation.decision_valid else "INVALID",
                "snapshot_id": snapshot_id,
                "selected_region": decision.selected_region,
                "decision_valid": validation.decision_valid,
                "timestamp": _utc_now_iso(),
            }
        )

        self.get_logger().info(
            f"[REGION_SELECTION] snapshot_id={snapshot_id} "
            f"selected={decision.selected_region} valid={validation.decision_valid} "
            f"motion_executed=false"
        )

        if not validation.decision_valid:
            resp.success = False
            resp.message = "SELECTOR_RESPONSE_INVALID"
            return resp

        resp.success = True
        resp.message = (
            f"snapshot_id={snapshot_id} selected={decision.selected_region} "
            f"fallback={decision.fallback_regions}"
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
