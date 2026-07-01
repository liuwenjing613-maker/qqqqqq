#!/usr/bin/env python3
"""
V2 导航第一视角录制：订阅 /image_raw + /nav_state + /target_bbox_json + /cmd_vel，
画面上叠加 target、状态切换、u/v、当前速度；导航开始后自动录制，SUCCESS 或 Ctrl+C 后保存。

用法：
  bash ~/rdk_x5_vln_robot/scripts/debug/capture_navigation_video.sh
  # 另开终端
  bash ~/rdk_x5_vln_robot/scripts/nav/start_yolo_lidar_nav.sh
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

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.nav.nav_video_overlay import NavOverlayContext, annotate_nav_frame, safe_json_load

DEFAULT_SAVE_DIR = os.path.join(PROJECT_ROOT, "capture_video")
IDLE_FSM_MODES = {"", "BOOT", "WAIT_SENSORS", "INIT"}


class CaptureNavigationVideo(Node):
    def __init__(
        self,
        image_topic: str = "/image_raw",
        cmd_topic: str = "/cmd_vel",
        nav_state_topic: str = "/nav_state",
        bbox_topic: str = "/target_bbox_json",
        point_topic: str = "/nav_target_point",
        save_dir: str = DEFAULT_SAVE_DIR,
        record_fps: float = 20.0,
        post_success_sec: float = 1.0,
        record_on_nav_start: bool = True,
    ):
        super().__init__("capture_navigation_video")

        self.image_topic = image_topic
        self.cmd_topic = cmd_topic
        self.nav_state_topic = nav_state_topic
        self.bbox_topic = bbox_topic
        self.point_topic = point_topic
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
        self.create_subscription(String, self.nav_state_topic, self.nav_state_callback, 10)
        self.create_subscription(String, self.bbox_topic, self.bbox_callback, 10)
        self.create_subscription(String, self.point_topic, self.point_callback, 10)

        self.get_logger().info("===== capture_navigation_video (V2) =====")
        self.get_logger().info(f"phase: WAITING (start navigation to begin recording)")
        self.get_logger().info(
            f"topics image={self.image_topic} nav_state={self.nav_state_topic} "
            f"bbox={self.bbox_topic} cmd={self.cmd_topic}"
        )
        self.get_logger().info(f"save dir: {self.save_dir}")
        self.get_logger().info("overlay: target | state | u/v | velocity")
        self.get_logger().info("Press Ctrl+C anytime to stop and save recorded video.")

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd_received = True
        self.overlay_ctx.update_cmd(float(msg.linear.x), float(msg.angular.z))

    def nav_state_callback(self, msg: String) -> None:
        self.overlay_ctx.update_nav_state(safe_json_load(msg.data))

    def bbox_callback(self, msg: String) -> None:
        data = safe_json_load(msg.data)
        if data:
            self.overlay_ctx.target_bbox = data

    def point_callback(self, msg: String) -> None:
        data = safe_json_load(msg.data)
        if data:
            self.overlay_ctx.target_point = data

    def navigation_started(self) -> bool:
        if not self.record_on_nav_start:
            return True
        if abs(self.overlay_ctx.cmd_vx) > 1e-3 or abs(self.overlay_ctx.cmd_wz) > 1e-3:
            return True
        return self.overlay_ctx.fsm_mode() not in IDLE_FSM_MODES

    def is_success(self) -> bool:
        return self.overlay_ctx.fsm_mode() == "SUCCESS"

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
        return os.path.join(self.save_dir, f"nav_{stamp}.mp4")

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
                self.get_logger().info("SUCCESS detected, finishing recording soon...")

            if self.success_time is not None and time.time() - self.success_time >= self.post_success_sec:
                self.request_shutdown("task_success")


def main():
    parser = argparse.ArgumentParser(description="Record V2 navigation video with overlay.")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--nav-state-topic", default="/nav_state")
    parser.add_argument("--bbox-topic", default="/target_bbox_json")
    parser.add_argument("--point-topic", default="/nav_target_point")
    parser.add_argument("--save-dir", default=DEFAULT_SAVE_DIR)
    parser.add_argument("--record-fps", type=float, default=20.0)
    parser.add_argument("--post-success-sec", type=float, default=1.0)
    parser.add_argument(
        "--record-immediately",
        action="store_true",
        help="start recording on first image frame (skip wait-for-nav)",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CaptureNavigationVideo(
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        nav_state_topic=args.nav_state_topic,
        bbox_topic=args.bbox_topic,
        point_topic=args.point_topic,
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
        print(f"[DONE] capture finished, saved to: {node.save_dir}")


if __name__ == "__main__":
    main()
