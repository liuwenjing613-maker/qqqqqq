#!/usr/bin/env python3
"""Real online backend: live OccupancyGrid -> frontier candidates -> Qwen -> Nav2.

This is the missing teammate-side process for the V1 fusion protocol.  It never
publishes /cmd_vel directly. Nav2 is routed to /map_qwen_plan/cmd_vel_raw; this
node relays it only for the active request to /map_qwen_plan/cmd_vel, and the
existing bridge/mux remains the sole route to the chassis.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import requests
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = PROJECT_ROOT.parent
import sys
sys.path.insert(0, str(PROJECT_ROOT / "src"))

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from fusion.live_frontier_backend_core_v2 import (
    FrontierCandidate,
    FrontierConfig,
    GridMeta,
    RobotPose2D,
    candidate_from_payload,
    candidate_is_currently_safe,
    candidate_summary_payload,
    candidate_table,
    choose_geometric_candidate,
    extract_frontier_candidates,
    parse_qwen_candidate_id,
    quaternion_to_yaw,
    render_candidate_map,
    wrap_angle,
)
from fusion.map_qwen_plan_markers import (
    build_candidate_markers,
    build_clear_markers,
    build_selected_goal_markers,
)


def load_config(path: str) -> Dict[str, Any]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    cfg = raw.get("online_map_plan_fullflow_v2", raw) or {}
    if not isinstance(cfg, dict):
        raise ValueError("online_map_plan_fullflow_v2 config must be a mapping")
    return cfg


def json_msg(payload: Dict[str, Any]) -> String:
    return String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class OnlineMapQwenNavBackendV2(Node):
    def __init__(self, config_path: str, task: str):
        super().__init__("online_map_qwen_nav_backend_v2")
        cfg = load_config(config_path)
        topics = cfg.get("topics", {}) or {}
        candidate_cfg = cfg.get("candidate", {}) or {}
        qwen_cfg = cfg.get("qwen", {}) or {}
        nav_cfg = cfg.get("navigation", {}) or {}
        safety_cfg = cfg.get("safety", {}) or {}

        self.task = str(task or cfg.get("default_task", "find the bottle")).strip()
        self.map_topic = str(topics.get("map", "/map"))
        self.request_topic = str(topics.get("backend_request", "/map_qwen_plan/request"))
        self.cancel_topic = str(topics.get("backend_cancel", "/map_qwen_plan/cancel"))
        self.probe_topic = str(topics.get("backend_candidate_probe", "/map_qwen_plan/candidate_probe"))
        self.status_topic = str(topics.get("backend_status", "/map_qwen_plan/status"))
        self.summary_topic = str(topics.get("backend_candidate_summary", "/map_qwen_plan/candidate_summary"))
        self.raw_cmd_topic = str(topics.get("nav2_cmd_raw", "/map_qwen_plan/cmd_vel_raw"))
        self.cmd_topic = str(topics.get("backend_cmd", "/map_qwen_plan/cmd_vel"))
        self.debug_topic = str(topics.get("backend_debug", "/map_qwen_plan/backend_debug"))
        self.candidate_markers_topic = str(
            topics.get("candidate_markers", "/map_qwen_plan/candidate_markers")
        )
        self.selected_goal_markers_topic = str(
            topics.get("selected_goal_markers", "/map_qwen_plan/selected_goal_markers")
        )
        self.nav_action_name = str(topics.get("navigate_action", "/navigate_to_pose"))
        self.map_frame = str(topics.get("map_frame", "map"))
        self.base_frame = str(topics.get("base_frame", "base_link"))

        self.frontier_cfg = FrontierConfig(
            free_max_value=int(candidate_cfg.get("free_max_value", 20)),
            occupied_min_value=int(candidate_cfg.get("occupied_min_value", 65)),
            obstacle_inflation_m=float(candidate_cfg.get("obstacle_inflation_m", 0.30)),
            min_frontier_cells=int(candidate_cfg.get("min_frontier_cells", 8)),
            information_radius_m=float(candidate_cfg.get("information_radius_m", 0.85)),
            min_goal_distance_m=float(candidate_cfg.get("min_goal_distance_m", 0.50)),
            max_goal_distance_m=float(candidate_cfg.get("max_goal_distance_m", 2.60)),
            preferred_min_distance_m=float(candidate_cfg.get("preferred_min_distance_m", 0.75)),
            preferred_max_distance_m=float(candidate_cfg.get("preferred_max_distance_m", 1.80)),
            max_abs_relative_heading_deg=float(candidate_cfg.get("max_abs_relative_heading_deg", 150.0)),
            min_heading_separation_deg=float(candidate_cfg.get("min_heading_separation_deg", 32.0)),
            max_candidates=int(candidate_cfg.get("max_candidates", 8)),
            stable_id_quantization_m=float(candidate_cfg.get("stable_id_quantization_m", 0.15)),
        )
        self.map_max_age_sec = float(candidate_cfg.get("map_max_age_sec", 2.0))
        self.tf_max_age_sec = float(candidate_cfg.get("tf_max_age_sec", 1.0))
        self.candidate_cache_sec = float(candidate_cfg.get("cache_sec", 1.2))

        self.qwen_enabled = bool(qwen_cfg.get("enabled", True))
        self.qwen_model = str(qwen_cfg.get("model", os.getenv("QWEN_MODEL", "qwen3-vl-flash")))
        self.qwen_base_url = str(
            qwen_cfg.get("base_url", os.getenv("QWEN_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"))
        ).rstrip("/")
        self.qwen_timeout_sec = float(qwen_cfg.get("timeout_sec", 15.0))
        self.qwen_retries = int(qwen_cfg.get("retries", 1))
        self.qwen_image_side = int(qwen_cfg.get("image_side", 1280))
        self.qwen_fallback_enabled = bool(qwen_cfg.get("fallback_to_geometry", True))
        self.qwen_dry_run = bool(qwen_cfg.get("dry_run", False)) or os.getenv("MAP_QWEN_DRY_RUN", "0") == "1"

        self.nav_server_wait_sec = float(nav_cfg.get("server_wait_sec", 5.0))
        self.nav_timeout_sec = float(nav_cfg.get("navigation_timeout_sec", 150.0))
        self.raw_cmd_timeout_sec = float(nav_cfg.get("raw_cmd_timeout_sec", 0.35))
        self.align_enabled = bool(nav_cfg.get("final_alignment_enabled", True))
        self.align_tolerance_rad = math.radians(float(nav_cfg.get("final_alignment_tolerance_deg", 8.0)))
        self.align_hold_sec = float(nav_cfg.get("final_alignment_hold_sec", 0.40))
        self.align_timeout_sec = float(nav_cfg.get("final_alignment_timeout_sec", 12.0))
        self.align_kp = float(nav_cfg.get("final_alignment_kp", 0.65))
        self.align_min_wz = float(nav_cfg.get("final_alignment_min_wz", 0.025))
        self.align_max_wz = float(nav_cfg.get("final_alignment_max_wz", 0.05))
        self.max_linear_x = float(safety_cfg.get("max_linear_x", 0.055))
        self.max_angular_z = float(safety_cfg.get("max_angular_z", 0.060))

        debug_dir = Path(str(cfg.get("debug_dir", PROJECT_ROOT / "logs" / "fullflow_v2"))).expanduser()
        if not debug_dir.is_absolute():
            debug_dir = (PROJECT_ROOT / debug_dir).resolve()
        self.debug_dir = debug_dir
        self.debug_dir.mkdir(parents=True, exist_ok=True)

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, reliable)
        self.summary_pub = self.create_publisher(String, self.summary_topic, reliable)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, reliable)
        self.debug_pub = self.create_publisher(String, self.debug_topic, reliable)
        self.candidate_markers_pub = self.create_publisher(
            MarkerArray, self.candidate_markers_topic, reliable
        )
        self.selected_goal_markers_pub = self.create_publisher(
            MarkerArray, self.selected_goal_markers_topic, reliable
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(String, self.probe_topic, self._on_probe, reliable)
        self.create_subscription(String, self.request_topic, self._on_request, reliable)
        self.create_subscription(String, self.cancel_topic, self._on_cancel, reliable)
        self.create_subscription(Twist, self.raw_cmd_topic, self._on_raw_cmd, sensor)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client: ActionClient = ActionClient(self, NavigateToPose, self.nav_action_name)
        # Do not name this self.executor — that collides with rclpy.Node.executor.
        self.api_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="map_qwen_v2")

        self.map_array: Optional[np.ndarray] = None
        self.map_meta: Optional[GridMeta] = None
        self.map_version = "none"
        self.map_received_mono = float("-inf")
        self.latest_candidates: List[FrontierCandidate] = []
        self.latest_diagnostics: Dict[str, Any] = {}
        self.candidates_built_mono = float("-inf")
        self.last_robot_pose: Optional[RobotPose2D] = None
        self.last_pose_mono = float("-inf")

        self.active_request_id: Optional[str] = None
        self.active_operation: Optional[str] = None
        self.active_candidate: Optional[FrontierCandidate] = None
        self.pending_candidates: Dict[str, FrontierCandidate] = {}
        self.phase = "IDLE"
        self.phase_started = time.monotonic()
        self.request_started = float("-inf")
        self.selection_future: Optional[Future] = None
        self.selection_token = 0
        self.goal_handle = None
        self.nav_result_future = None
        self.raw_cmd = Twist()
        self.raw_cmd_mono = float("-inf")
        self.align_stable_since: Optional[float] = None
        self.last_status = "STARTUP"
        self.last_status_publish_mono = float("-inf")
        self.lock = threading.RLock()
        self.viz_candidates: List[FrontierCandidate] = []
        self.viz_selected_id: Optional[str] = None
        self.viz_selected_candidate: Optional[FrontierCandidate] = None
        self.viz_active = False

        self.timer = self.create_timer(0.05, self._tick)
        self.status_timer = self.create_timer(0.5, self._publish_debug)
        self.viz_timer = self.create_timer(0.5, self._republish_viz_markers)
        self.get_logger().info(
            "fullflow V2 backend ready: "
            f"map={self.map_topic}, request={self.request_topic}, nav={self.nav_action_name}, "
            f"raw_cmd={self.raw_cmd_topic}, output={self.cmd_topic}"
        )

    def _parse(self, msg: String, label: str) -> Optional[Dict[str, Any]]:
        try:
            value = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"invalid {label} JSON: {exc}")
            return None
        if not isinstance(value, dict):
            self.get_logger().warning(f"invalid {label}: root must be object")
            return None
        return value

    def _on_map(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            self.get_logger().warning("ignore malformed OccupancyGrid")
            return
        array = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        meta = GridMeta(
            width=width,
            height=height,
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            frame_id=str(msg.header.frame_id or self.map_frame),
        )
        stamp = msg.header.stamp
        version = f"{width}x{height}:{stamp.sec}.{stamp.nanosec:09d}"
        with self.lock:
            self.map_array = array
            self.map_meta = meta
            self.map_version = version
            self.map_received_mono = time.monotonic()

    def _lookup_robot_pose(self) -> Optional[RobotPose2D]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.15),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        pose = RobotPose2D(
            x=float(t.x),
            y=float(t.y),
            yaw=quaternion_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.last_robot_pose = pose
        self.last_pose_mono = time.monotonic()
        return pose

    def _extract_now(self, force: bool = False) -> Tuple[List[FrontierCandidate], Dict[str, Any]]:
        now = time.monotonic()
        with self.lock:
            if (
                not force
                and self.latest_candidates
                and now - self.candidates_built_mono <= self.candidate_cache_sec
            ):
                return list(self.latest_candidates), dict(self.latest_diagnostics)
            if self.map_array is None or self.map_meta is None:
                return [], {"failure": "map_missing"}
            if now - self.map_received_mono > self.map_max_age_sec:
                return [], {"failure": "map_stale", "age_sec": now - self.map_received_mono}
            occupancy = self.map_array.copy()
            meta = self.map_meta
        pose = self._lookup_robot_pose()
        if pose is None:
            return [], {"failure": "map_to_base_tf_missing"}
        candidates, diagnostics = extract_frontier_candidates(
            occupancy, meta, pose, self.frontier_cfg
        )
        with self.lock:
            self.latest_candidates = list(candidates)
            self.latest_diagnostics = dict(diagnostics)
            self.candidates_built_mono = now
        return candidates, diagnostics

    def _on_probe(self, msg: String) -> None:
        payload = self._parse(msg, "candidate probe")
        if payload is None:
            return
        if str(payload.get("operation", "")).upper() != "EXTRACT_CANDIDATES_ONLY":
            return
        candidates, diagnostics = self._extract_now(force=True)
        summary = candidate_summary_payload(
            candidates,
            map_version=self.map_version,
            probe_id=str(payload.get("probe_id", "")) or None,
            diagnostics=diagnostics,
        )
        self.summary_pub.publish(json_msg(summary))

    def _on_request(self, msg: String) -> None:
        payload = self._parse(msg, "backend request")
        if payload is None:
            return
        request_id = str(payload.get("request_id", "")).strip()
        operation = str(payload.get("operation", "")).strip().upper()
        if not request_id:
            return
        with self.lock:
            if self.active_request_id:
                if request_id == self.active_request_id:
                    return
                self._status(request_id, "REJECTED", reason=f"backend_busy:{self.active_request_id}")
                return
            self.active_request_id = request_id
            self.active_operation = operation
            self.phase = "EXTRACTING"
            self.phase_started = time.monotonic()
            self.request_started = self.phase_started
            self.active_candidate = None
            self.pending_candidates = {}
            self.goal_handle = None
            self.raw_cmd_mono = float("-inf")
            self.selection_token += 1
            token = self.selection_token
        self._status(request_id, "ACCEPTED", operation=operation)
        self._publish_zero()

        # Geometry is deterministic and should finish quickly; Qwen runs in the
        # worker thread so callbacks, cancellation and zero-command heartbeat
        # remain responsive.
        try:
            candidates, diagnostics = self._extract_now(force=True)
            resolved = self._resolve_request_candidates(payload, candidates)
            self.pending_candidates = {c.candidate_id: c for c in resolved}
            if not resolved:
                raise RuntimeError(f"no_valid_candidate:{diagnostics.get('failure', 'empty')}")
            self._set_viz_candidates(resolved, selected_id=None)
        except Exception as exc:  # noqa: BLE001
            self._fail_active(f"candidate_extraction_failed:{exc}")
            return

        direct = operation == "NAVIGATE_CANDIDATE" or len(resolved) == 1
        if direct:
            chosen = choose_geometric_candidate(resolved)
            self._set_viz_selected(chosen)
            self._begin_navigation(request_id, chosen, selected_by="direct_or_single")
            return

        self.phase = "QWEN_SELECTING"
        self.phase_started = time.monotonic()
        self._status(request_id, "QWEN_SELECTING", candidate_ids=[c.candidate_id for c in resolved])
        self.selection_future = self.api_pool.submit(
            self._select_candidate_worker,
            token,
            request_id,
            self.task or str(payload.get("instruction", "")),
            resolved,
        )

    def _resolve_request_candidates(
        self,
        request: Dict[str, Any],
        current: Sequence[FrontierCandidate],
    ) -> List[FrontierCandidate]:
        current_by_id = {c.candidate_id: c for c in current}
        requested_ids = [str(v) for v in (request.get("candidate_ids") or [])]
        snapshots = request.get("candidate_snapshot") or []
        snapshot_by_id: Dict[str, FrontierCandidate] = {}
        if self.map_meta is not None:
            for raw in snapshots:
                if not isinstance(raw, dict):
                    continue
                candidate = candidate_from_payload(raw, self.map_meta)
                if candidate is not None:
                    snapshot_by_id[candidate.candidate_id] = candidate

        ids = requested_ids or list(current_by_id)
        out: List[FrontierCandidate] = []
        seen = set()
        for cid in ids:
            candidate = current_by_id.get(cid)
            if candidate is None:
                candidate = snapshot_by_id.get(cid)
                if (
                    candidate is not None
                    and self.map_array is not None
                    and self.map_meta is not None
                    and not candidate_is_currently_safe(
                        candidate, self.map_array, self.map_meta, self.frontier_cfg
                    )
                ):
                    candidate = None
            if candidate is not None and candidate.candidate_id not in seen:
                out.append(candidate)
                seen.add(candidate.candidate_id)
        if not requested_ids:
            for candidate in current:
                if candidate.candidate_id not in seen:
                    out.append(candidate)
                    seen.add(candidate.candidate_id)
        return out

    def _select_candidate_worker(
        self,
        token: int,
        request_id: str,
        instruction: str,
        candidates: Sequence[FrontierCandidate],
    ) -> Dict[str, Any]:
        geometric = choose_geometric_candidate(candidates)
        if self.qwen_dry_run or not self.qwen_enabled:
            return {
                "token": token,
                "request_id": request_id,
                "candidate_id": geometric.candidate_id,
                "selected_by": "geometry_dry_run",
                "reason": "Qwen disabled/dry-run",
            }
        api_key = os.getenv("DASHSCOPE_API_KEY") or os.getenv("QWEN_API_KEY")
        if not api_key:
            if self.qwen_fallback_enabled:
                return {
                    "token": token,
                    "request_id": request_id,
                    "candidate_id": geometric.candidate_id,
                    "selected_by": "geometry_missing_api_key",
                    "reason": "DASHSCOPE_API_KEY missing",
                }
            raise RuntimeError("DASHSCOPE_API_KEY missing")
        if self.map_array is None or self.map_meta is None or self.last_robot_pose is None:
            raise RuntimeError("map/pose snapshot missing for Qwen")

        image = render_candidate_map(
            self.map_array,
            self.map_meta,
            self.last_robot_pose,
            candidates,
            max_side=self.qwen_image_side,
        )
        image_path = self.debug_dir / f"{request_id}_candidates.jpg"
        cv2.imwrite(str(image_path), image)
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        if not ok:
            raise RuntimeError("candidate image JPEG encoding failed")
        data_url = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
        allowed = [c.candidate_id for c in candidates]
        nearest = min(candidates, key=lambda c: (c.distance_m, c.candidate_id))
        prompt = (
            "你是移动机器人全局探索候选选择器。程序已经保证候选位于已知自由区、"
            "与机器人处于同一可通行连通区域，并会由 Nav2 再次检查路径。\n"
            f"用户任务：{instruction}\n"
            "地图颜色：白色=已知自由，灰色=未知，黑色=障碍；蓝点和红箭头是机器人；"
            "黄色编号点是候选。当前阶段没有长期轨迹记忆，因此不得声称某区域已经探索。\n"
            "选择原则：优先更可能继续发现目标的方向；语义线索相近时，兼顾信息增益、"
            "安全余量和适中距离。只能从 allowed_candidate_ids 中选择，不得生成坐标或新编号。\n"
            "兜底规则（仅当前面语义原则无法明确选出唯一候选时启用，且不得跳过 allowed_candidate_ids）："
            f"选择 distance_m 最小的候选，即 {nearest.candidate_id}（distance_m="
            f"{nearest.distance_m:.3f}）。若任务语义已能明确区分，仍按语义优先，不要机械选最近。\n"
            f"allowed_candidate_ids={json.dumps(allowed, ensure_ascii=False)}\n"
            f"候选表：\n{candidate_table(candidates)}\n"
            "只输出一行合法 JSON："
            '{"candidate_id":"F_x","confidence":0.0,"reason":"简短原因"}'
        )
        (self.debug_dir / f"{request_id}_prompt.txt").write_text(prompt, encoding="utf-8")
        payload = {
            "model": self.qwen_model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": data_url}},
                        {"type": "text", "text": prompt},
                    ],
                }
            ],
            "temperature": 0.0,
            "max_tokens": 220,
        }
        last_error: Optional[Exception] = None
        for attempt in range(max(1, self.qwen_retries + 1)):
            try:
                response = requests.post(
                    f"{self.qwen_base_url}/chat/completions",
                    headers={
                        "Authorization": f"Bearer {api_key}",
                        "Content-Type": "application/json",
                    },
                    json=payload,
                    timeout=self.qwen_timeout_sec,
                )
                response.raise_for_status()
                body = response.json()
                content = body["choices"][0]["message"]["content"]
                if isinstance(content, list):
                    content = "".join(
                        str(item.get("text", "")) if isinstance(item, dict) else str(item)
                        for item in content
                    )
                raw_text = str(content)
                (self.debug_dir / f"{request_id}_qwen_raw.txt").write_text(raw_text, encoding="utf-8")
                selected_id, parsed = parse_qwen_candidate_id(raw_text, allowed)
                return {
                    "token": token,
                    "request_id": request_id,
                    "candidate_id": selected_id,
                    "selected_by": "qwen",
                    "reason": str(parsed.get("reason", "")),
                    "confidence": parsed.get("confidence"),
                }
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                if attempt < self.qwen_retries:
                    time.sleep(0.25)
        if self.qwen_fallback_enabled:
            return {
                "token": token,
                "request_id": request_id,
                "candidate_id": geometric.candidate_id,
                "selected_by": "geometry_after_qwen_error",
                "reason": str(last_error),
            }
        raise RuntimeError(f"Qwen selection failed: {last_error}")

    def _begin_navigation(
        self,
        request_id: str,
        candidate: FrontierCandidate,
        *,
        selected_by: str,
        selection_meta: Optional[Dict[str, Any]] = None,
    ) -> None:
        if request_id != self.active_request_id:
            return
        if not self.nav_client.wait_for_server(timeout_sec=self.nav_server_wait_sec):
            self._fail_active("navigate_to_pose_action_unavailable")
            return
        self.active_candidate = candidate
        self._set_viz_selected(candidate)
        self.phase = "PLANNING"
        self.phase_started = time.monotonic()
        self._status(
            request_id,
            "PLANNING",
            selected_candidate_id=candidate.candidate_id,
            selected_by=selected_by,
            goal_pose=candidate.to_payload()["goal_pose"],
            selection_meta=selection_meta or {},
        )
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(candidate.x)
        goal.pose.pose.position.y = float(candidate.y)
        qx, qy, qz, qw = yaw_to_quaternion(float(candidate.yaw))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._fail_active(f"nav_goal_send_exception:{exc}")
            return
        if not goal_handle.accepted:
            self._fail_active("nav_goal_rejected")
            return
        self.goal_handle = goal_handle
        self.phase = "NAVIGATING"
        self.phase_started = time.monotonic()
        self._status(
            self.active_request_id,
            "NAVIGATING",
            selected_candidate_id=self.active_candidate.candidate_id if self.active_candidate else None,
        )
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self._on_nav_result)

    def _on_nav_feedback(self, feedback_msg) -> None:
        # Feedback is deliberately not republished at high frequency. The debug
        # heartbeat contains phase and selected goal without flooding String topics.
        _ = feedback_msg

    def _on_nav_result(self, future) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:  # noqa: BLE001
            self._fail_active(f"nav_result_exception:{exc}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail_active(f"nav_result_status_{status}")
            return
        self._publish_zero()
        if not self.align_enabled or self.active_candidate is None:
            self._complete_active("nav2_reached", final_orientation_done=not self.align_enabled)
            return
        self.phase = "ALIGNING"
        self.phase_started = time.monotonic()
        self.align_stable_since = None
        self._status(
            self.active_request_id,
            "NAVIGATING",
            substate="ALIGNING",
            selected_candidate_id=self.active_candidate.candidate_id,
        )

    def _on_raw_cmd(self, msg: Twist) -> None:
        with self.lock:
            self.raw_cmd = msg
            self.raw_cmd_mono = time.monotonic()

    def _on_cancel(self, msg: String) -> None:
        payload = self._parse(msg, "backend cancel") or {}
        request_id = str(payload.get("request_id", "")).strip()
        if not self.active_request_id:
            return
        if request_id and request_id != self.active_request_id:
            return
        reason = str(payload.get("reason", "cancelled"))
        self.selection_token += 1
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        active = self.active_request_id
        self._publish_zero()
        self._status(active, "CANCELLED", reason=reason)
        self._reset_active("cancelled")

    def _tick(self) -> None:
        now = time.monotonic()
        if self.selection_future is not None and self.selection_future.done():
            future = self.selection_future
            self.selection_future = None
            try:
                result = future.result()
                if (
                    int(result.get("token", -1)) != self.selection_token
                    or str(result.get("request_id", "")) != self.active_request_id
                ):
                    return
                by_id = dict(self.pending_candidates)
                candidate = by_id.get(str(result.get("candidate_id", "")))
                if candidate is None:
                    raise RuntimeError("selected candidate disappeared from current cache")
                self._begin_navigation(
                    self.active_request_id,
                    candidate,
                    selected_by=str(result.get("selected_by", "unknown")),
                    selection_meta=result,
                )
            except Exception as exc:  # noqa: BLE001
                self._fail_active(f"candidate_selection_failed:{exc}")
                return

        if self.phase in {"EXTRACTING", "QWEN_SELECTING", "PLANNING", "NAVIGATING", "ALIGNING"}:
            if now - self.request_started > self.nav_timeout_sec:
                self._fail_active("backend_navigation_timeout")
                return

        if self.phase == "NAVIGATING":
            if now - self.raw_cmd_mono <= self.raw_cmd_timeout_sec:
                self._publish_clamped(self.raw_cmd)
            else:
                self._publish_zero()
        elif self.phase == "ALIGNING":
            self._tick_alignment(now)
        else:
            self._publish_zero()

    def _tick_alignment(self, now: float) -> None:
        if self.active_candidate is None:
            self._fail_active("alignment_candidate_missing")
            return
        if now - self.phase_started > self.align_timeout_sec:
            self._fail_active("final_alignment_timeout")
            return
        pose = self._lookup_robot_pose()
        if pose is None:
            self._publish_zero()
            return
        error = wrap_angle(self.active_candidate.yaw - pose.yaw)
        if abs(error) <= self.align_tolerance_rad:
            self._publish_zero()
            if self.align_stable_since is None:
                self.align_stable_since = now
            elif now - self.align_stable_since >= self.align_hold_sec:
                self._complete_active("arrived_and_aligned", final_orientation_done=True)
            return
        self.align_stable_since = None
        magnitude = min(self.align_max_wz, max(self.align_min_wz, abs(error) * self.align_kp))
        cmd = Twist()
        cmd.angular.z = math.copysign(magnitude, error)
        self._publish_clamped(cmd)

    def _publish_clamped(self, source: Twist) -> None:
        cmd = Twist()
        cmd.linear.x = max(-self.max_linear_x, min(self.max_linear_x, float(source.linear.x)))
        cmd.angular.z = max(-self.max_angular_z, min(self.max_angular_z, float(source.angular.z)))
        self.cmd_pub.publish(cmd)

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def _status(self, request_id: Optional[str], status: str, **extra: Any) -> None:
        payload: Dict[str, Any] = {
            "schema_version": "online_map_qwen_nav_status_v2",
            "request_id": request_id,
            "status": str(status).upper(),
            "phase": self.phase,
            "source": "online_map_qwen_nav_backend_v2",
            "stamp": time.time(),
        }
        payload.update({k: v for k, v in extra.items() if v is not None})
        self.last_status = str(status).upper()
        self.last_status_publish_mono = time.monotonic()
        self.status_pub.publish(json_msg(payload))

    def _complete_active(self, reason: str, *, final_orientation_done: bool) -> None:
        request_id = self.active_request_id
        candidate = self.active_candidate
        self._publish_zero()
        self._status(
            request_id,
            "COMPLETED",
            reason=reason,
            selected_candidate_id=None if candidate is None else candidate.candidate_id,
            final_orientation_done=bool(final_orientation_done),
        )
        self._reset_active("completed")

    def _fail_active(self, reason: str) -> None:
        request_id = self.active_request_id
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self._publish_zero()
        if request_id:
            self._status(request_id, "FAILED", reason=reason)
        self._reset_active(reason)

    def _set_viz_candidates(
        self,
        candidates: Sequence[FrontierCandidate],
        *,
        selected_id: Optional[str],
    ) -> None:
        self.viz_candidates = list(candidates)
        self.viz_selected_id = selected_id
        self.viz_selected_candidate = None
        self.viz_active = bool(candidates)
        self._publish_viz_markers()

    def _set_viz_selected(self, candidate: FrontierCandidate) -> None:
        self.viz_selected_id = candidate.candidate_id
        self.viz_selected_candidate = candidate
        if not any(c.candidate_id == candidate.candidate_id for c in self.viz_candidates):
            self.viz_candidates = list(self.viz_candidates) + [candidate]
        self.viz_active = True
        self._publish_viz_markers()

    def _clear_viz_markers(self) -> None:
        self.viz_candidates = []
        self.viz_selected_id = None
        self.viz_selected_candidate = None
        self.viz_active = False
        stamp = self.get_clock().now().to_msg()
        cleared = build_clear_markers(self.map_frame, stamp)
        self.candidate_markers_pub.publish(cleared)
        self.selected_goal_markers_pub.publish(cleared)

    def _publish_viz_markers(self) -> None:
        if not self.viz_active or not self.viz_candidates:
            return
        stamp = self.get_clock().now().to_msg()
        self.candidate_markers_pub.publish(
            build_candidate_markers(
                self.viz_candidates,
                frame_id=self.map_frame,
                stamp=stamp,
                selected_id=self.viz_selected_id,
            )
        )
        if self.viz_selected_candidate is not None:
            self.selected_goal_markers_pub.publish(
                build_selected_goal_markers(
                    self.viz_selected_candidate,
                    frame_id=self.map_frame,
                    stamp=stamp,
                )
            )

    def _republish_viz_markers(self) -> None:
        if self.viz_active:
            self._publish_viz_markers()

    def _reset_active(self, reason: str) -> None:
        self._clear_viz_markers()
        self.phase = "IDLE"
        self.phase_started = time.monotonic()
        self.active_request_id = None
        self.active_operation = None
        self.active_candidate = None
        self.pending_candidates = {}
        self.goal_handle = None
        self.nav_result_future = None
        self.selection_future = None
        self.raw_cmd_mono = float("-inf")
        self.align_stable_since = None
        self.get_logger().info(f"backend reset: {reason}")

    def _publish_debug(self) -> None:
        now = time.monotonic()
        if self.active_request_id and now - self.last_status_publish_mono >= 0.75:
            heartbeat_status = self.phase
            if self.phase == "ALIGNING":
                heartbeat_status = "NAVIGATING"
            self._status(
                self.active_request_id,
                heartbeat_status,
                heartbeat=True,
                substate=self.phase,
                selected_candidate_id=(
                    None if self.active_candidate is None else self.active_candidate.candidate_id
                ),
            )
        payload = {
            "phase": self.phase,
            "active_request_id": self.active_request_id,
            "active_candidate_id": None if self.active_candidate is None else self.active_candidate.candidate_id,
            "map_version": self.map_version,
            "map_age_sec": None if self.map_array is None else round(time.monotonic() - self.map_received_mono, 3),
            "candidate_count": len(self.latest_candidates),
            "candidate_age_sec": None if not self.latest_candidates else round(time.monotonic() - self.candidates_built_mono, 3),
            "last_status": self.last_status,
            "qwen_dry_run": self.qwen_dry_run,
            "task": self.task,
        }
        self.debug_pub.publish(json_msg(payload))

    def shutdown(self) -> None:
        self.selection_token += 1
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        for _ in range(4):
            self._publish_zero()
            time.sleep(0.03)
        self.api_pool.shutdown(wait=False, cancel_futures=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "configs" / "online_map_plan_fullflow_v2.yaml"),
    )
    parser.add_argument("--task", default="")
    args = parser.parse_args()
    rclpy.init()
    node: Optional[OnlineMapQwenNavBackendV2] = None
    try:
        node = OnlineMapQwenNavBackendV2(args.config, args.task)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
