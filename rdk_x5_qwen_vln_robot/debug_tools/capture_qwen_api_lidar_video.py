#!/usr/bin/env python3
"""
Qwen API + LiDAR first-person navigation recorder.

Subscribes to /image_raw, /qwen_api_state, /qwen_api_json, and cmd_vel.
Overlays phase/action, Qwen u/v or path waypoint, and velocity.
Starts recording when navigation begins; saves on SUCCESS, ARRIVED, or Ctrl+C.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

PROJECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RDK_ORIGINAL_ROOT = os.environ.get("RDK_ORIGINAL_ROOT", "/root/rdk_x5_vln_robot")
sys.path.insert(0, RDK_ORIGINAL_ROOT)

from src.nav.nav_video_overlay import NavOverlayContext, annotate_nav_frame, safe_json_load

DEFAULT_SAVE_DIR = os.path.join(PROJECT_DIR, "capture_video")
IDLE_PHASES = {"", "BOOT", "WAIT_SENSORS", "INIT"}


def _normalize_qwen_state(data: dict) -> dict:
    out = dict(data)
    phase = str(out.get("phase", "") or "")
    if phase and not out.get("fsm_mode"):
        out["fsm_mode"] = phase
    action = str(out.get("action", "") or "")
    if action and not out.get("fsm_reason"):
        out["fsm_reason"] = action
    return out


class CaptureQwenApiLidarVideo(Node):
    def __init__(
        self,
        image_topic: str = "/image_raw",
        cmd_topic: str = "/cmd_vel",
        state_topic: str = "/qwen_api_state",
        json_topic: str = "/qwen_api_json",
        save_dir: str = DEFAULT_SAVE_DIR,
        record_fps: float = 20.0,
        post_success_sec: float = 1.5,
        record_on_nav_start: bool = True,
    ):
        super().__init__("capture_qwen_api_lidar_video")

        self.image_topic = image_topic
        self.cmd_topic = cmd_topic
        self.state_topic = state_topic
        self.json_topic = json_topic
        self.save_dir = os.path.expanduser(save_dir)
        self.record_fps = float(record_fps)
        self.post_success_sec = float(post_success_sec)
        self.record_on_nav_start = bool(record_on_nav_start)

        self.bridge = CvBridge()
        self.overlay_ctx = NavOverlayContext()
        self.phase = "WAITING"
        self.video_writer = None
        self.output_path = ""
        self.frame_count = 0
        self.recorded_frames = 0
        self.wait_start = time.time()
        self.record_start: Optional[float] = None
        self.success_time: Optional[float] = None
        self.shutdown_requested = False
        self.finished = False
        self.cmd_received = False

        os.makedirs(self.save_dir, exist_ok=True)

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.create_subscription(String, self.state_topic, self.state_callback, 10)
        self.create_subscription(String, self.json_topic, self.json_callback, 10)

        self.get_logger().info("===== capture_qwen_api_lidar_video =====")
        self.get_logger().info("phase: WAITING (navigation start triggers recording)")
        self.get_logger().info(
            f"topics image={self.image_topic} state={self.state_topic} "
            f"json={self.json_topic} cmd={self.cmd_topic}"
        )
        self.get_logger().info(f"save dir: {self.save_dir}")
        self.get_logger().info("Press Ctrl+C to stop and save recorded video.")

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd_received = True
        self.overlay_ctx.update_cmd(float(msg.linear.x), float(msg.angular.z))

    def state_callback(self, msg: String) -> None:
        data = safe_json_load(msg.data)
        if data:
            self.overlay_ctx.update_nav_state(_normalize_qwen_state(data))

    def json_callback(self, msg: String) -> None:
        data = safe_json_load(msg.data)
        if not data:
            return
        mode = str(data.get("mode", data.get("qwen_mode", "")) or "").upper()
        u = data.get("u")
        v = data.get("v")
        wu = data.get("waypoint_u")
        wv = data.get("waypoint_v")
        point_kind = str(data.get("point_kind", mode) or mode)
        if mode == "TARGET" and u is not None and v is not None:
            self.overlay_ctx.target_point = {
                "u": u,
                "v": v,
                "visible": True,
                "class_name": "qwen_target",
                "score": data.get("confidence"),
                "reason": data.get("reason", ""),
            }
        elif (wu is not None and wv is not None) or (mode == "PATH" and u is not None and v is not None):
            pu = wu if wu is not None else u
            pv = wv if wv is not None else v
            self.overlay_ctx.target_point = {
                "u": pu,
                "v": pv,
                "visible": True,
                "class_name": point_kind or "path_waypoint",
                "score": data.get("confidence"),
                "reason": data.get("reason", ""),
            }

    def navigation_started(self) -> bool:
        if not self.record_on_nav_start:
            return True
        if abs(self.overlay_ctx.cmd_vx) > 1e-3 or abs(self.overlay_ctx.cmd_wz) > 1e-3:
            return True
        phase = str(self.overlay_ctx.nav_state.get("phase", "") or "")
        if phase and phase not in IDLE_PHASES:
            return True
        return self.overlay_ctx.fsm_mode() not in IDLE_PHASES

    def is_success(self) -> bool:
        phase = str(self.overlay_ctx.nav_state.get("phase", "") or "")
        action = str(self.overlay_ctx.nav_state.get("action", "") or "")
        if phase == "SUCCESS":
            return True
        return action == "ARRIVED"

    def _header_lines(self) -> list[str]:
        elapsed_wait = time.time() - self.wait_start
        elapsed_rec = 0.0 if self.record_start is None else time.time() - self.record_start
        return [
            f"[REC {self.phase}] frame={self.frame_count} rec={self.recorded_frames} "
            f"wait={elapsed_wait:.1f}s rec={elapsed_rec:.1f}s"
        ]

    def _annotate_frame(self, frame):
        banner = self.overlay_ctx.fsm_mode() or self.phase
        if self.phase == "WAITING" and not self.overlay_ctx.fsm_mode():
            banner = self.phase
        return annotate_nav_frame(
            frame,
            self.overlay_ctx,
            header_lines=self._header_lines(),
            status_banner=banner,
        )

    def _make_output_path(self) -> str:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.save_dir, f"qwen_nav_{stamp}.mp4")

    def _open_writer(self, frame) -> None:
        h, w = frame.shape[:2]
        self.output_path = self._make_output_path()
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(self.output_path, fourcc, self.record_fps, (w, h))
        if not writer.isOpened():
            self.output_path = self.output_path.replace(".mp4", ".avi")
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            writer = cv2.VideoWriter(self.output_path, fourcc, self.record_fps, (w, h))
        self.video_writer = writer
        self.record_start = time.time()
        self.phase = "RECORDING"
        self.get_logger().info(f"recording started: {self.output_path}")

    def _close_writer(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.phase = "FINISHED"

        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            elapsed = time.time() - self.record_start if self.record_start else 0.0
            self.get_logger().info(
                f"video saved ({reason}): {self.output_path} "
                f"frames={self.recorded_frames} duration={elapsed:.1f}s"
            )
            print(f"[VIDEO_SAVED] {self.output_path}", flush=True)
        else:
            self.get_logger().warn(f"no video recorded ({reason})")

    def request_shutdown(self, reason: str = "interrupted") -> None:
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.get_logger().info(f"shutdown requested: {reason}")
        self._close_writer(reason)

    def image_callback(self, msg: Image) -> None:
        if self.shutdown_requested or self.finished:
            return

        self.frame_count += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge failed: {repr(exc)}")
            return

        vis = self._annotate_frame(frame)

        if self.phase == "WAITING":
            if self.navigation_started():
                self._open_writer(vis)
            else:
                if self.frame_count % 60 == 0:
                    self.get_logger().info(
                        "waiting for navigation... "
                        f"(frames={self.frame_count}, cmd_received={self.cmd_received})"
                    )
                return

        if self.phase == "RECORDING" and self.video_writer is not None:
            self.video_writer.write(vis)
            self.recorded_frames += 1

            if self.is_success() and self.success_time is None:
                self.success_time = time.time()
                self.get_logger().info("SUCCESS/ARRIVED detected, finishing recording soon...")

            if self.success_time is not None and time.time() - self.success_time >= self.post_success_sec:
                self.request_shutdown("task_success")


def main():
    parser = argparse.ArgumentParser(description="Record Qwen API LiDAR navigation video.")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--state-topic", default="/qwen_api_state")
    parser.add_argument("--json-topic", default="/qwen_api_json")
    parser.add_argument("--save-dir", default=DEFAULT_SAVE_DIR)
    parser.add_argument("--record-fps", type=float, default=20.0)
    parser.add_argument("--post-success-sec", type=float, default=1.5)
    parser.add_argument(
        "--record-immediately",
        action="store_true",
        help="start recording on first image frame (skip wait-for-nav)",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CaptureQwenApiLidarVideo(
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        state_topic=args.state_topic,
        json_topic=args.json_topic,
        save_dir=args.save_dir,
        record_fps=args.record_fps,
        post_success_sec=args.post_success_sec,
        record_on_nav_start=not args.record_immediately,
    )

    def handle_signal(signum, _frame):
        node.request_shutdown(f"signal_{signum}")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.request_shutdown("keyboard_interrupt")
    finally:
        if not node.finished:
            node.request_shutdown("cleanup")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[DONE] capture finished, save dir: {node.save_dir}", flush=True)


if __name__ == "__main__":
    main()
