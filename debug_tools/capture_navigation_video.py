#!/usr/bin/env python3
"""
导航录制脚本：先启动并等待，检测到导航开始后再录制；任务成功或 Ctrl+C 后保存视频。

用法（先开录制，再启动导航）：
  source /opt/tros/humble/setup.bash
  python3 ~/rdk_x5_vln_robot/debug_tools/capture_navigation_video.py

  bash ~/rdk_x5_vln_robot/scripts/capture_navigation_video.sh
"""

import argparse
import os
import signal
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.control.mvp_visual_servo import MVPVisualServo
from src.fsm.mvp_state_machine import MVPState, MVPStateMachine
from src.perception.target_backend_red import find_red_target

DEFAULT_SAVE_DIR = os.path.join(PROJECT_ROOT, "capture_video")


def draw_text_block(img, lines, origin=(12, 12), line_height=22, font_scale=0.55):
    """左上角绘制半透明信息面板。"""
    if not lines:
        return img

    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 1
    max_width = 0
    for line in lines:
        (tw, _), _ = cv2.getTextSize(line, font, font_scale, thickness)
        max_width = max(max_width, tw)

    panel_w = max_width + 24
    panel_h = line_height * len(lines) + 16
    x0, y0 = origin
    overlay = img.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

    y = y0 + 20
    for line in lines:
        cv2.putText(img, line, (x0 + 10, y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
        y += line_height


class CaptureNavigationVideo(Node):
    def __init__(
        self,
        image_topic="/image_raw",
        cmd_topic="/cmd_vel",
        save_dir=DEFAULT_SAVE_DIR,
        image_width=1280,
        image_height=720,
        record_fps=20.0,
        post_success_sec=1.0,
    ):
        super().__init__("capture_navigation_video")

        self.image_topic = image_topic
        self.cmd_topic = cmd_topic
        self.save_dir = os.path.expanduser(save_dir)
        self.image_width = image_width
        self.image_height = image_height
        self.record_fps = float(record_fps)
        self.post_success_sec = float(post_success_sec)

        self.bridge = CvBridge()
        self.servo = MVPVisualServo(image_width=self.image_width)
        self.fsm = MVPStateMachine(stable_frames_required=5, lost_frames_limit=8)

        self.phase = "WAITING"  # WAITING -> RECORDING -> FINISHED
        self.video_writer = None
        self.output_path = ""
        self.frame_count = 0
        self.recorded_frames = 0
        self.wait_start = time.time()
        self.record_start = None
        self.success_time = None
        self.shutdown_requested = False
        self.finished = False

        self.last_cmd_vx = 0.0
        self.last_cmd_wz = 0.0
        self.cmd_received = False
        self.last_target = {"visible": False}
        self.last_fsm = MVPState.INIT
        self.last_servo = "LOST_STOP"
        self.last_action = "-"

        os.makedirs(self.save_dir, exist_ok=True)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self.cmd_callback, 10
        )

        self.get_logger().info("===== capture_navigation_video =====")
        self.get_logger().info(f"phase: WAITING (start navigation to begin recording)")
        self.get_logger().info(f"image topic: {self.image_topic}")
        self.get_logger().info(f"cmd topic: {self.cmd_topic}")
        self.get_logger().info(f"save dir: {self.save_dir}")
        self.get_logger().info("Press Ctrl+C anytime to stop and save recorded video.")

    def cmd_callback(self, msg: Twist):
        self.cmd_received = True
        self.last_cmd_vx = float(msg.linear.x)
        self.last_cmd_wz = float(msg.angular.z)

    def navigation_started(self, fsm_state, servo_state):
        if abs(self.last_cmd_vx) > 1e-3 or abs(self.last_cmd_wz) > 1e-3:
            return True
        if fsm_state in (
            MVPState.VISUAL_SERVO,
            MVPState.RECOVERY_SCAN,
            MVPState.TARGET_LOCKED,
        ):
            return True
        if fsm_state == MVPState.OBSERVE and servo_state in ("TURN_ONLY", "FORWARD"):
            return True
        return False

    def _make_output_path(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.save_dir, f"nav_{stamp}.mp4")

    def _open_writer(self, frame):
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

    def _close_writer(self, reason):
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

    def request_shutdown(self, reason="interrupted"):
        if not self.shutdown_requested:
            self.shutdown_requested = True
            self.get_logger().info(f"shutdown requested: {reason}")
        self._close_writer(reason)

    def _build_overlay_lines(self, target, fsm_state, servo_state, action):
        h, w = self.image_width, self.image_height
        elapsed_wait = time.time() - self.wait_start
        elapsed_rec = 0.0 if self.record_start is None else time.time() - self.record_start

        lines = [
            f"[{self.phase}] frame={self.frame_count} rec={self.recorded_frames}",
            f"time wait={elapsed_wait:.1f}s rec={elapsed_rec:.1f}s fps={self.record_fps:.0f}",
            f"fsm={fsm_state} servo={servo_state} action={action}",
            f"cmd_vx={self.last_cmd_vx:+.3f} cmd_wz={self.last_cmd_wz:+.3f}",
        ]

        if target.get("visible", False):
            x, y, bw, bh = target["bbox"]
            cx = target.get("cx", 0.0)
            cy = target.get("cy", 0.0)
            area = target.get("area_ratio", 0.0)
            ex = (cx - w / 2.0) / w
            lines.extend([
                "target=FOUND class=red_backpack",
                f"bbox=[{x},{y},{bw},{bh}]",
                f"cx={cx:.1f} cy={cy:.1f} ex={ex:+.3f}",
                f"area_ratio={area:.4f}",
            ])
        else:
            reason = target.get("reason", "not_visible")
            lines.append(f"target=LOST reason={reason}")

        lines.append(f"image={w}x{h} topic={self.image_topic}")
        return lines

    def _annotate_frame(self, frame, target, fsm_state, servo_state, action):
        vis = frame.copy()

        if target.get("visible", False):
            x, y, bw, bh = target["bbox"]
            cx = int(target.get("cx", x + bw / 2.0))
            cy = int(target.get("cy", y + bh / 2.0))
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)
            cv2.line(vis, (self.image_width // 2, 0), (self.image_width // 2, vis.shape[0]), (255, 255, 0), 1)

        status_color = (0, 0, 255) if self.phase == "WAITING" else (0, 255, 0)
        cv2.putText(
            vis,
            self.phase,
            (vis.shape[1] - 180, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            status_color,
            2,
            cv2.LINE_AA,
        )

        lines = self._build_overlay_lines(target, fsm_state, servo_state, action)
        draw_text_block(vis, lines, origin=(12, 12))
        return vis

    def image_callback(self, msg: Image):
        if self.shutdown_requested or self.finished:
            return

        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            return

        target = find_red_target(frame)
        servo_state, _ = self.servo.compute_cmd(target)
        target_visible = bool(target.get("visible", False))
        fsm_state = self.fsm.update(target_visible, servo_state)

        if fsm_state == MVPState.RECOVERY_SCAN:
            action = "RECOVERY_SCAN"
        elif fsm_state == MVPState.SUCCESS:
            action = "SUCCESS_STOP"
        else:
            action = "SERVO_CMD"

        self.last_target = target
        self.last_fsm = fsm_state
        self.last_servo = servo_state
        self.last_action = action

        vis = self._annotate_frame(frame, target, fsm_state, servo_state, action)

        if self.phase == "WAITING":
            if self.navigation_started(fsm_state, servo_state):
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

            if fsm_state == MVPState.SUCCESS and self.success_time is None:
                self.success_time = time.time()
                self.get_logger().info("task SUCCESS detected, finishing recording soon...")

            if self.success_time is not None:
                if time.time() - self.success_time >= self.post_success_sec:
                    self.request_shutdown("task_success")


def main():
    parser = argparse.ArgumentParser(description="Record navigation video with bbox overlay.")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--save-dir", default=DEFAULT_SAVE_DIR)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--record-fps", type=float, default=20.0)
    parser.add_argument("--post-success-sec", type=float, default=1.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CaptureNavigationVideo(
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        save_dir=args.save_dir,
        image_width=args.image_width,
        image_height=args.image_height,
        record_fps=args.record_fps,
        post_success_sec=args.post_success_sec,
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
