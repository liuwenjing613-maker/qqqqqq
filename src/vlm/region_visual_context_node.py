#!/usr/bin/env python3
"""ROS 2 passive 360° visual context capture — no motion control."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.vlm.region_visual_context_core import (  # noqa: E402
    DirectionalFrameCandidate,
    VisualCaptureSession,
    accumulate_rotation_deg,
    associate_regions_with_views,
    build_visual_context_manifest,
    generate_visual_context_id,
    manifest_to_dict,
    region_view_mapping_to_dict,
    relative_angle_from_initial,
    save_visual_context_artifacts,
    select_directional_frames,
    validate_visual_context_config,
)

try:
    import cv2  # type: ignore
    from cv_bridge import CvBridge  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    CvBridge = None  # type: ignore


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _quat_to_yaw_deg(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class RegionVisualContextNode(Node):
    NODE_NAME = "region_visual_context"

    STATES = frozenset(
        {
            "IDLE",
            "WAITING_FOR_SNAPSHOT",
            "WAITING_FOR_IMAGE",
            "WAITING_FOR_TF",
            "CAPTURING",
            "COMPLETE",
            "INCOMPLETE",
            "TIMEOUT",
            "FAILED",
            "CANCELED",
        }
    )

    def __init__(self, cfg: Dict[str, Any], run_dir: Path) -> None:
        super().__init__(self.NODE_NAME)
        self.cfg = cfg
        self.run_dir = run_dir
        self.state = "IDLE"
        self._latest_snapshot: Optional[Dict[str, Any]] = None
        self._session: Optional[VisualCaptureSession] = None
        self._visual_context_id = ""
        self._session_dir: Optional[Path] = None
        self._capture_start_s = 0.0
        self._bridge = CvBridge() if CvBridge is not None else None

        vcfg = cfg.get("visual_context", {})
        topics = cfg.get("topics", {})
        services = cfg.get("services", {})
        self.map_frame = str(vcfg.get("map_frame", "map"))
        self.robot_frame = str(vcfg.get("robot_frame", "base_link"))
        self.image_topic = str(vcfg.get("image_topic", "")).strip()
        self.image_transport = str(vcfg.get("image_transport", "raw")).lower()
        self.max_capture_duration_s = float(vcfg.get("max_capture_duration_s", 90.0))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        snap_topic = topics.get("snapshot", "/qwen_explore_debug/region_snapshot_json")
        self.create_subscription(String, snap_topic, self._snapshot_cb, 10)

        if self.image_topic:
            if self.image_transport == "compressed":
                self.create_subscription(
                    CompressedImage, self.image_topic, self._compressed_image_cb, qos_profile_sensor_data
                )
            else:
                self.create_subscription(
                    Image, self.image_topic, self._image_cb, qos_profile_sensor_data
                )
        else:
            self.get_logger().warn(
                "[VISUAL_CONTEXT] image_topic empty — configure before runtime capture"
            )

        self.pub_status = self.create_publisher(
            String, topics.get("capture_status", "/qwen_explore/visual_capture_status"), 10
        )
        self.pub_manifest = self.create_publisher(
            String, topics.get("manifest_json", "/qwen_explore/visual_context_manifest_json"), 10
        )
        self.pub_mapping = self.create_publisher(
            String, topics.get("region_view_mapping_json", "/qwen_explore/region_view_mapping_json"), 10
        )
        self.pub_contact = self.create_publisher(
            CompressedImage,
            topics.get("contact_sheet", "/qwen_explore/panorama_contact_sheet/compressed"),
            10,
        )
        self.pub_board = self.create_publisher(
            CompressedImage,
            topics.get("decision_board", "/qwen_explore/decision_board/compressed"),
            10,
        )

        start_svc = services.get("start_capture", "/qwen_explore/start_visual_context_capture")
        cancel_svc = services.get("cancel_capture", "/qwen_explore/cancel_visual_context_capture")
        self.create_service(Trigger, start_svc, self._start_capture_cb)
        self.create_service(Trigger, cancel_svc, self._cancel_capture_cb)

        self.create_timer(0.5, self._capture_timer_cb)
        self.get_logger().info(
            f"Region visual context node started (passive capture, image_topic={self.image_topic or 'UNCONFIGURED'})"
        )

    def _snapshot_cb(self, msg: String) -> None:
        try:
            self._latest_snapshot = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("[VISUAL_CONTEXT] invalid snapshot JSON ignored")

    def _lookup_yaw(self) -> Tuple[Optional[float], float, float, List[str]]:
        reasons: List[str] = []
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            yaw_deg = _quat_to_yaw_deg(tf.transform.rotation)
            now_ns = self.get_clock().now().nanoseconds
            tf_ns = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
            tf_age_s = max(0.0, (now_ns - tf_ns) * 1e-9)
            tf_stamp = float(tf.header.stamp.sec) + float(tf.header.stamp.nanosec) * 1e-9
            max_age = float(self.cfg.get("visual_context", {}).get("max_tf_age_s", 0.30))
            if tf_age_s > max_age:
                reasons.append("FRAME_TF_STALE")
                return None, tf_stamp, tf_age_s, reasons
            return yaw_deg, tf_stamp, tf_age_s, reasons
        except TransformException as exc:
            reasons.append("FRAME_TF_MISSING")
            self.get_logger().debug(f"[VISUAL_CONTEXT] TF missing: {exc}")
            return None, 0.0, 0.0, reasons

    def _save_frame_image(self, msg: Any, frame_path: Path) -> Tuple[int, int, str]:
        if self._bridge is None or cv2 is None:
            return 0, 0, ""
        try:
            if isinstance(msg, CompressedImage):
                img = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                encoding = "jpeg"
            else:
                img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                encoding = str(getattr(msg, "encoding", "bgr8")).lower()
            h, w = img.shape[:2]
            frame_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(frame_path), img)
            return w, h, encoding
        except Exception as exc:  # pragma: no cover
            self.get_logger().debug(f"[VISUAL_CONTEXT] image save failed: {exc}")
            return 0, 0, ""

    def _ingest_image(self, msg: Any, *, is_compressed: bool) -> None:
        if self.state not in ("CAPTURING", "WAITING_FOR_IMAGE", "WAITING_FOR_TF"):
            return
        if self._session is None:
            return
        if not self.image_topic:
            return

        yaw_deg, tf_stamp, tf_age_s, tf_reasons = self._lookup_yaw()
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if yaw_deg is None:
            cand = DirectionalFrameCandidate(
                image_stamp_sec=now_s,
                tf_stamp_sec=tf_stamp,
                tf_age_s=tf_age_s,
                absolute_yaw_deg=0.0,
                relative_yaw_deg=0.0,
                image_reference="",
                valid=False,
                rejection_reasons=tf_reasons or ["FRAME_TF_MISSING"],
            )
            self._session.candidates.append(cand)
            self.state = "WAITING_FOR_TF"
            return

        rel_yaw = relative_angle_from_initial(self._session.initial_robot_yaw_deg, yaw_deg)
        self._session.yaw_samples_deg.append(yaw_deg)
        self._session.final_robot_yaw_deg = yaw_deg
        self._session.accumulated_rotation_deg = accumulate_rotation_deg(
            self._session.yaw_samples_deg
        )

        stamp = float(getattr(msg.header, "stamp", msg.header.stamp).sec) + float(
            getattr(msg.header.stamp, "nanosec", 0)
        ) * 1e-9
        ref = f"frame_{len(self._session.candidates):05d}"
        image_file = ""
        width = height = 0
        encoding = "bgr8"
        if bool(self.cfg.get("visual_context", {}).get("save_frames", True)) and self._session_dir:
            view_guess = f"cand_{len(self._session.candidates):05d}.jpg"
            image_file = str(self._session_dir / "frames" / view_guess)
            width, height, encoding = self._save_frame_image(msg, Path(image_file))

        valid = width > 0 and height > 0
        reasons: List[str] = []
        if not valid:
            reasons.append("FRAME_IMAGE_INVALID")

        cand = DirectionalFrameCandidate(
            image_stamp_sec=stamp,
            tf_stamp_sec=tf_stamp,
            tf_age_s=tf_age_s,
            absolute_yaw_deg=yaw_deg,
            relative_yaw_deg=rel_yaw,
            image_reference=image_file or ref,
            width=width,
            height=height,
            encoding=encoding,
            valid=valid,
            rejection_reasons=reasons,
        )
        self._session.candidates.append(cand)
        self.state = "CAPTURING"

    def _image_cb(self, msg: Image) -> None:
        self._ingest_image(msg, is_compressed=False)

    def _compressed_image_cb(self, msg: CompressedImage) -> None:
        self._ingest_image(msg, is_compressed=True)

    def _publish_status(self, extra: Optional[Dict[str, Any]] = None) -> None:
        payload: Dict[str, Any] = {
            "state": self.state,
            "visual_context_id": self._visual_context_id,
            "snapshot_id": self._session.snapshot_id if self._session else "",
            "accumulated_rotation_deg": (
                self._session.accumulated_rotation_deg if self._session else 0.0
            ),
            "candidate_count": len(self._session.candidates) if self._session else 0,
            "timestamp": _utc_now_iso(),
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_status.publish(msg)

    def _start_capture_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self._latest_snapshot is None:
            self.state = "WAITING_FOR_SNAPSHOT"
            response.success = False
            response.message = "WAITING_FOR_SNAPSHOT"
            self._publish_status()
            return response

        snapshot_id = str(self._latest_snapshot.get("snapshot_id", ""))
        if not snapshot_id:
            self.state = "FAILED"
            response.success = False
            response.message = "SNAPSHOT_ID_MISSING"
            return response

        yaw_deg, _, _, tf_reasons = self._lookup_yaw()
        if yaw_deg is None:
            self.state = "WAITING_FOR_TF"
            response.success = False
            response.message = f"WAITING_FOR_TF reasons={tf_reasons}"
            self._publish_status()
            return response

        if not self.image_topic:
            self.state = "WAITING_FOR_IMAGE"
            response.success = False
            response.message = "IMAGE_TOPIC_UNCONFIGURED"
            self._publish_status()
            return response

        self._visual_context_id = generate_visual_context_id()
        run_id = self.run_dir.name
        self._session_dir = self.run_dir / "sessions" / self._visual_context_id
        self._session_dir.mkdir(parents=True, exist_ok=True)
        (self._session_dir / "frames").mkdir(exist_ok=True)

        self._session = VisualCaptureSession(
            capture_session_id=f"CS_{self._visual_context_id}",
            snapshot_id=snapshot_id,
            state="CAPTURING",
            initial_robot_yaw_deg=yaw_deg,
            final_robot_yaw_deg=yaw_deg,
            capture_start_time=_utc_now_iso(),
            yaw_samples_deg=[yaw_deg],
        )
        self._capture_start_s = self.get_clock().now().nanoseconds * 1e-9
        self.state = "CAPTURING"
        self._publish_status({"snapshot_id": snapshot_id})
        response.success = True
        response.message = (
            f"visual_context_id={self._visual_context_id} snapshot_id={snapshot_id} state=CAPTURING"
        )
        self.get_logger().info(f"[VISUAL_CAPTURE] started {response.message}")
        return response

    def _cancel_capture_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self.state = "CANCELED"
        if self._session:
            self._session.capture_end_time = _utc_now_iso()
        self._publish_status()
        response.success = True
        response.message = "capture_canceled"
        return response

    def _finalize_capture(self) -> None:
        if self._session is None or self._latest_snapshot is None:
            self.state = "FAILED"
            return

        self._session.capture_end_time = _utc_now_iso()
        frames, capture_complete, frame_errors = select_directional_frames(
            self._session.candidates,
            self.cfg,
            initial_yaw_deg=self._session.initial_robot_yaw_deg,
        )
        associations, map_errors = associate_regions_with_views(
            self._latest_snapshot,
            frames,
            self._session.initial_robot_yaw_deg,
            self.cfg,
        )
        all_errors = frame_errors + map_errors
        manifest = build_visual_context_manifest(
            self._session,
            frames,
            associations,
            self.cfg,
            visual_context_id=self._visual_context_id,
            validation_errors=all_errors,
        )

        annotated = str(self._latest_snapshot.get("annotated_map_file", ""))
        paths: Dict[str, str] = {}
        if self._session_dir:
            paths = save_visual_context_artifacts(
                self._session_dir,
                manifest,
                associations,
                annotated_map_path=Path(annotated) if annotated else None,
                cfg=self.cfg,
            )

        manifest_payload = manifest_to_dict(manifest)
        mapping_payload = region_view_mapping_to_dict(
            associations, manifest.snapshot_id, manifest.visual_context_id
        )

        self.pub_manifest.publish(String(data=json.dumps(manifest_payload, ensure_ascii=False)))
        self.pub_mapping.publish(String(data=json.dumps(mapping_payload, ensure_ascii=False)))

        if cv2 is not None and paths.get("contact_sheet_file"):
            img = cv2.imread(paths["contact_sheet_file"])
            if img is not None:
                ok, buf = cv2.imencode(".jpg", img)
                if ok:
                    cmsg = CompressedImage()
                    cmsg.format = "jpeg"
                    cmsg.data = buf.tobytes()
                    self.pub_contact.publish(cmsg)
        if cv2 is not None and paths.get("decision_board_file"):
            img = cv2.imread(paths["decision_board_file"])
            if img is not None:
                ok, buf = cv2.imencode(".jpg", img)
                if ok:
                    bmsg = CompressedImage()
                    bmsg.format = "jpeg"
                    bmsg.data = buf.tobytes()
                    self.pub_board.publish(bmsg)

        if capture_complete:
            self.state = "COMPLETE"
        else:
            self.state = "INCOMPLETE"
        self._publish_status(
            {
                "capture_complete": capture_complete,
                "validation_errors": all_errors,
                "artifact_paths": paths,
            }
        )

    def _capture_timer_cb(self) -> None:
        if self.state not in ("CAPTURING", "WAITING_FOR_TF", "WAITING_FOR_IMAGE"):
            return
        if self._session is None:
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._capture_start_s > self.max_capture_duration_s:
            self.state = "TIMEOUT"
            self._finalize_capture()
            return

        threshold = float(
            self.cfg.get("visual_context", {}).get("complete_rotation_threshold_deg", 350.0)
        )
        if self._session.accumulated_rotation_deg >= threshold:
            frames, complete, _ = select_directional_frames(
                self._session.candidates,
                self.cfg,
                initial_yaw_deg=self._session.initial_robot_yaw_deg,
            )
            if complete or self._session.accumulated_rotation_deg >= threshold:
                self._finalize_capture()


def main() -> None:
    parser = argparse.ArgumentParser(description="Region visual context capture node")
    parser.add_argument(
        "--config",
        default=str(ROOT / "configs/qwen_region_visual_context.yaml"),
    )
    parser.add_argument("--run-dir", default="")
    args = parser.parse_args()

    cfg_path = Path(args.config).resolve()
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    cfg_errors = validate_visual_context_config(cfg)
    if cfg_errors:
        print(f"[FATAL] config errors: {cfg_errors}")
        sys.exit(1)

    log_root = Path(cfg.get("logging", {}).get("root_dir", "logs/qwen_visual_context"))
    run_dir = Path(args.run_dir) if args.run_dir else log_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = RegionVisualContextNode(cfg, run_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
