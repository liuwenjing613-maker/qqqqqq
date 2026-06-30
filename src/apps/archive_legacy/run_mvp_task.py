#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge

# 允许直接从项目根目录运行
PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.vlm.mock_qwen import mock_qwen_parse
from src.perception.target_backend_red import find_red_target
from src.perception.multi_frame_voter import MultiFrameTargetVoter
from src.perception.target_backend_yolo import extract_yolo_target, parse_target_classes
from src.perception.stamp_sync import StampSyncBuffer
from src.config.mvp_tune import DEFAULT_TUNE_PATH, load_mvp_tune
from src.control.mvp_visual_servo import MVPVisualServo
from src.fsm.mvp_state_machine import MVPStateMachine, MVPState


class RunMVPTask(Node):
    def __init__(
        self,
        instruction="find the red backpack",
        image_topic="/image_raw",
        cmd_topic="/cmd_vel",
        backend="red",
        image_width=1280,
        image_height=720,
        det_topic="/hobot_yolo_world",
        target_words_topic="/target_words",
        min_score=0.002,
        det_stale_sec=1.0,
        save_debug=False,
        save_bbox_dir=None,
        save_bbox_interval=15,
        min_red_ratio=0.06,
        max_area_ratio=0.15,
        require_red_verify=True,
        sync_max_delta_sec=0.12,
        sync_buffer_len=60,
        max_vx=0.04,
        max_wz=0.16,
        kp_turn=0.08,
        center_threshold=0.28,
        arrive_area_ratio=0.20,
        slowdown_area_ratio=0.07,
        turn_threshold=0.30,
        forward_threshold=0.18,
        wz_deadzone=0.05,
        cmd_wz_deadzone=0.01,
        forward_turn_scale=0.35,
        recovery_scan_wz=0.006,
        min_cruise_wz=0.16,
        lost_hold_frames=3,
        lost_observe_frames=7,
        recovery_scan_max_frames=25,
        recovery_replan_sec=2.0,
        lost_hold_wz_scale=0.4,
        lost_hold_max_wz=0.035,
        recovery_pulse_frames=2,
        recovery_observe_frames=6,
        stable_frames_required=3,
        lost_frames_limit=15,
        target_classes_override=None,
        yolo_prompts_override=None,
        publish_target_words=True,
        use_multi_frame_voter=False,
        vote_window_size=6,
        vote_min_votes=2,
        vote_lost_hold_frames=1,
    ):
        super().__init__("run_mvp_task")

        self.instruction = instruction
        self.image_topic = image_topic
        self.cmd_topic = cmd_topic
        self.backend = backend
        self.image_width = image_width
        self.image_height = image_height
        self.det_topic = det_topic
        self.target_words_topic = target_words_topic
        self.min_score = float(min_score)
        self.det_stale_sec = float(det_stale_sec)
        self.save_debug = save_debug
        self.save_bbox_interval = max(1, int(save_bbox_interval))
        self.save_bbox_dir = (
            os.path.expanduser(save_bbox_dir)
            if save_bbox_dir
            else os.path.join(PROJECT_ROOT, "check_bbox")
        )
        self.save_bbox = bool(save_bbox_dir) or save_debug
        self.found_bbox_count = 0
        self.min_red_ratio = float(min_red_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.require_red_verify = bool(require_red_verify)
        self.sync_max_delta_sec = float(sync_max_delta_sec)
        self.recovery_scan_wz = float(recovery_scan_wz)
        self.min_cruise_wz = float(min_cruise_wz)
        self.lost_hold_frames = max(0, int(lost_hold_frames))
        self.lost_observe_frames = max(self.lost_hold_frames, int(lost_observe_frames))
        self.recovery_scan_max_frames = max(self.lost_observe_frames + 1, int(recovery_scan_max_frames))
        self.recovery_replan_sec = float(recovery_replan_sec)
        self.lost_hold_wz_scale = float(lost_hold_wz_scale)
        self.lost_hold_max_wz = float(lost_hold_max_wz)
        self.recovery_pulse_frames = max(1, int(recovery_pulse_frames))
        self.recovery_observe_frames = max(1, int(recovery_observe_frames))
        self.last_cmd = Twist()
        self.has_seen_target = False
        self.last_seen_target_time = 0.0
        self.last_target_wz_dir = 1.0
        self.last_target_wz = 0.0

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.qwen_result = mock_qwen_parse(self.instruction)
        if yolo_prompts_override is not None:
            self.yolo_prompts = parse_target_classes(yolo_prompts_override)
        else:
            self.yolo_prompts = list(self.qwen_result.get("possible_yolo_world_prompts", []))
        if target_classes_override is not None:
            self.target_classes = parse_target_classes(target_classes_override)
        else:
            self.target_classes = list(self.qwen_result.get("target_classes", []))
        self.enable_target_words_pub = bool(publish_target_words)
        self.target_voter = None
        if use_multi_frame_voter:
            self.target_voter = MultiFrameTargetVoter(
                window_size=int(vote_window_size),
                min_votes=int(vote_min_votes),
                lost_hold_frames=int(vote_lost_hold_frames),
                iou_threshold=0.05,
                center_dist_threshold=0.35,
                smooth_alpha=0.20,
                image_width=self.image_width,
                image_height=self.image_height,
            )
        self.det_buffer = StampSyncBuffer(
            max_len=sync_buffer_len,
            max_delta_sec=self.sync_max_delta_sec,
        )
        self.frame_buffer = StampSyncBuffer(
            max_len=sync_buffer_len,
            max_delta_sec=self.sync_max_delta_sec,
        )
        self.last_det_time = 0.0
        self.last_sync_warn_time = 0.0

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        if self.backend == "yolo_world":
            self.det_sub = self.create_subscription(
                PerceptionTargets,
                self.det_topic,
                self.det_callback,
                10,
            )
            if self.enable_target_words_pub:
                self.target_words_pub = self.create_publisher(
                    String,
                    self.target_words_topic,
                    10,
                )
                self.create_timer(1.0, self.publish_target_words)
                self.publish_target_words()
            else:
                self.target_words_pub = None
                self.get_logger().info(
                    "target_words publish disabled (external publisher expected)"
                )

        self.servo = MVPVisualServo(
            image_width=self.image_width,
            kp_turn=float(kp_turn),
            max_vx=float(max_vx),
            max_wz=float(max_wz),
            center_threshold=float(center_threshold),
            arrive_area_ratio=float(arrive_area_ratio),
            slowdown_area_ratio=float(slowdown_area_ratio),
            turn_threshold=float(turn_threshold),
            forward_threshold=float(forward_threshold),
            wz_deadzone=float(wz_deadzone),
            cmd_wz_deadzone=float(cmd_wz_deadzone),
            forward_turn_scale=float(forward_turn_scale),
            min_cruise_wz=float(min_cruise_wz),
        )

        self.fsm = MVPStateMachine(
            stable_frames_required=int(stable_frames_required),
            lost_frames_limit=int(lost_frames_limit),
        )

        self.start_time = time.time()
        self.frame_count = 0
        self.success = False

        self.debug_dir = os.path.join(PROJECT_ROOT, "data/images/mvp_debug")
        if self.save_debug:
            os.makedirs(self.debug_dir, exist_ok=True)
        if self.save_bbox:
            os.makedirs(self.save_bbox_dir, exist_ok=True)

        self.get_logger().info("===== MVP TASK START =====")
        self.get_logger().info(f"instruction: {self.instruction}")
        self.get_logger().info(f"backend: {self.backend}")
        self.get_logger().info(
            f"servo: max_vx={max_vx} max_wz={max_wz} kp_turn={kp_turn} "
            f"turn_th={turn_threshold} fwd_th={forward_threshold} "
            f"arrive_area={arrive_area_ratio} slowdown_area={slowdown_area_ratio} "
            f"wz_deadzone={wz_deadzone} cmd_wz_deadzone={cmd_wz_deadzone} "
            f"fwd_turn_scale={forward_turn_scale}"
        )
        self.get_logger().info(
            f"fsm: stable_frames={stable_frames_required} lost_limit={lost_frames_limit} "
            f"recovery_scan_wz={recovery_scan_wz}"
        )
        self.get_logger().info(
            "lost recovery: "
            f"hold<= {self.lost_hold_frames}f, observe<= {self.lost_observe_frames}f, "
            f"scan<= {self.recovery_scan_max_frames}f, replan={self.recovery_replan_sec:.2f}s, "
            f"hold_scale={self.lost_hold_wz_scale:.2f}, hold_max_wz={self.lost_hold_max_wz:.3f}, "
            f"pulse={self.recovery_pulse_frames}f/{self.recovery_observe_frames}f"
        )
        if self.backend == "yolo_world":
            self.get_logger().info(f"det_topic: {self.det_topic}")
            self.get_logger().info(f"target_words_topic: {self.target_words_topic}")
            self.get_logger().info(f"yolo_prompts: {self.yolo_prompts}")
            self.get_logger().info(f"target_classes: {self.target_classes}")
            self.get_logger().info(f"min_score: {self.min_score}")
            self.get_logger().info(f"min_red_ratio: {self.min_red_ratio}")
            self.get_logger().info(f"max_area_ratio: {self.max_area_ratio}")
            self.get_logger().info(f"require_red_verify: {self.require_red_verify}")
            self.get_logger().info(
                f"stamp_sync max_delta={self.sync_max_delta_sec}s buffer={sync_buffer_len}"
            )
            if self.target_voter is not None:
                self.get_logger().info(
                    "multi_frame_voter: enabled "
                    f"window={self.target_voter.window_size} "
                    f"min_votes={self.target_voter.min_votes} "
                    f"lost_hold={self.target_voter.lost_hold_frames}"
                )
        if self.save_bbox:
            self.get_logger().info(
                f"save_bbox_dir={self.save_bbox_dir} interval={self.save_bbox_interval}"
            )
        self.get_logger().info("Mock Qwen output:")
        self.get_logger().info(json.dumps(self.qwen_result, ensure_ascii=False))

    def publish_target_words(self):
        if (
            self.backend != "yolo_world"
            or not self.enable_target_words_pub
            or not self.yolo_prompts
        ):
            return
        msg = String()
        msg.data = ",".join(self.yolo_prompts)
        self.target_words_pub.publish(msg)

    def det_callback(self, msg: PerceptionTargets):
        self.last_det_time = time.time()
        stamp = msg.header.stamp
        if not (stamp.sec or stamp.nanosec):
            fallback = self.get_clock().now().to_msg()
            self.get_logger().warn(
                "det header.stamp empty, using node clock fallback "
                f"({fallback.sec}.{fallback.nanosec:09d})"
            )
            msg.header.stamp = fallback
            stamp = msg.header.stamp
        self.det_buffer.push(stamp, msg)

    def _log_mvp_target_diagnostic(self, target):
        if target.get("visible", False):
            self.get_logger().info(
                "[MVP_TARGET] "
                f"class={target.get('class_name', '')} "
                f"score={target.get('score', 0.0):.4f} "
                f"bbox={target.get('bbox')} "
                f"area_ratio={target.get('area_ratio', 0.0):.4f} "
                f"red_ratio={target.get('red_ratio', 0.0):.4f}"
            )
        else:
            self.get_logger().info(
                "[MVP_REJECT] "
                f"reason={target.get('reason', 'unknown')} "
                f"class={target.get('class_name', '')} "
                f"score={target.get('score', 0.0):.4f} "
                f"bbox={target.get('bbox')} "
                f"area_ratio={target.get('area_ratio', 0.0):.4f} "
                f"red_ratio={target.get('red_ratio', 0.0):.4f}"
            )

    def _is_stale_control_target(self, target):
        if not target or not target.get("visible", False):
            return False
        return bool(target.get("stale", False)) or target.get("vote_reason") == "hold_last_target"

    def resolve_target(self, frame, image_stamp=None):
        if self.backend == "red":
            return find_red_target(frame), frame
        det_msg = self.det_buffer.peek_latest()
        if det_msg is None:
            return {"visible": False, "reason": "no_det_msg"}, frame
        if time.time() - self.last_det_time > self.det_stale_sec:
            return {"visible": False, "reason": "det_stale"}, frame
        matched_frame, delta = self.frame_buffer.find_closest(det_msg.header.stamp)
        if matched_frame is None:
            now = time.time()
            if now - self.last_sync_warn_time >= 1.0:
                self.last_sync_warn_time = now
                delta_str = f"{delta:.3f}s" if delta is not None else "no_frame"
                self.get_logger().warn(
                    f"stamp sync skip: no frame for det stamp, delta {delta_str} "
                    f"> {self.sync_max_delta_sec}s (frame_buf={len(self.frame_buffer)})"
                )
            return {"visible": False, "reason": "stamp_sync_failed"}, frame
        target = extract_yolo_target(
            det_msg,
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.min_score,
            max_area_ratio=self.max_area_ratio,
            frame=matched_frame,
            min_red_ratio=self.min_red_ratio,
            require_red_verify=self.require_red_verify,
        )
        if self.target_voter is not None:
            target = self.target_voter.update(target)
        self._log_mvp_target_diagnostic(target)
        return target, matched_frame

    def _draw_bbox_overlay(self, img, target, fsm_state, servo_state, trigger):
        h, w = img.shape[:2]
        status = "FOUND" if target.get("visible", False) else "NO_MVP"
        reject_hint = ""
        if status == "NO_MVP" and target.get("reason"):
            reject_hint = f" reject={target['reason']}"

        if target.get("visible", False):
            x, y, bw, bh = target["bbox"]
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
            red_ratio = target.get("red_ratio", 0.0)
            label = (
                f"MVP {target.get('class_name', '')} {target.get('score', 0.0):.3f} "
                f"r={red_ratio:.2f}"
            )
            cv2.putText(
                img,
                label,
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

        lines = [
            f"MVP task frame={self.frame_count} found={self.found_bbox_count} fsm={fsm_state}",
            (
                f"backend={self.backend} min_score={self.min_score} "
                f"red_min={self.min_red_ratio} max_area={self.max_area_ratio}"
            ),
            f"status={status}{reject_hint} trigger={trigger} servo={servo_state}",
            f"{w}x{h} topic={self.image_topic}",
        ]
        if target.get("visible", False):
            lines.append(
                f"bbox={target.get('bbox')} area={target.get('area_ratio', 0.0):.3f}"
            )
        elif target.get("class_name"):
            lines.append(
                f"best={target.get('class_name')} score={target.get('score', 0.0):.4f} "
                f"bbox={target.get('bbox')}"
            )

        y0 = 12
        for line in lines:
            cv2.rectangle(img, (8, y0), (8 + len(line) * 9 + 10, y0 + 22), (0, 0, 0), -1)
            cv2.putText(
                img, line, (12, y0 + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )
            y0 += 24
        return status

    def save_bbox_snapshot(self, vis_frame, target, fsm_state, servo_state, trigger):
        if vis_frame is None:
            return
        vis = vis_frame.copy()
        status = self._draw_bbox_overlay(vis, target, fsm_state, servo_state, trigger)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(
            self.save_bbox_dir,
            f"mvp_{stamp}_f{self.frame_count}_{status}.jpg",
        )
        cv2.imwrite(save_path, vis)
        self.get_logger().info(f"saved bbox snapshot: {save_path}")

    def publish_stop(self):
        self.last_cmd = Twist()
        self.cmd_pub.publish(Twist())

    def publish_twist(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)
        self.last_cmd = msg
        return msg

    def publish_lost_recovery_cmd(self):
        """
        目标丢失后分阶段恢复：
          A. 短时漏检：停住，只保留一点上一帧转向趋势。
          B. 观察刹车：完全停住等图像稳定。
          C. 脉冲扫描：小角速度命令触发底盘桥 kick，随后停下观察。
          D. 扫描失败：停止等待重规划。
        """
        lost_frames = int(self.fsm.lost_frames)
        lost_sec = time.time() - self.last_seen_target_time if self.last_seen_target_time else 0.0

        if lost_frames <= self.lost_hold_frames:
            wz = self.last_target_wz * self.lost_hold_wz_scale
            wz = max(-self.lost_hold_max_wz, min(self.lost_hold_max_wz, wz))
            self.publish_twist(0.0, wz)
            return "LOST_HOLD_TURN"

        if lost_frames <= self.lost_observe_frames:
            self.publish_stop()
            return "LOST_OBSERVE_STOP"

        if lost_frames > self.recovery_scan_max_frames or lost_sec >= self.recovery_replan_sec:
            self.publish_stop()
            return "RECOVERY_REPLAN_WAIT"

        scan_frame = lost_frames - self.lost_observe_frames - 1
        cycle = self.recovery_pulse_frames + self.recovery_observe_frames
        if scan_frame % cycle < self.recovery_pulse_frames:
            wz = self.last_target_wz_dir * abs(self.recovery_scan_wz)
            self.publish_twist(0.0, wz)
            return "RECOVERY_PULSE_SCAN"

        self.publish_stop()
        return "RECOVERY_OBSERVE_STOP"

    def image_callback(self, msg):
        if self.success:
            self.publish_stop()
            return

        self.frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            self.publish_stop()
            return

        self.frame_buffer.push(msg.header.stamp, frame)

        target, vis_frame = self.resolve_target(frame, msg.header.stamp)

        if self._is_stale_control_target(target):
            self.get_logger().warn(
                "[SERVO_STALE_TARGET] stale target, stop servo to avoid drifting. "
                f"reason={target.get('reason')} "
                f"vote_reason={target.get('vote_reason')} "
                f"source={target.get('source')} "
                f"bbox={target.get('bbox')}"
            )
            self.publish_stop()
            self.get_logger().info(
                f"frame={self.frame_count} fsm={self.fsm.state} action=STALE_TARGET_STOP "
                f"bbox={target.get('bbox')} cx={target.get('cx', 0.0):.0f} "
                f"area={target.get('area_ratio', 0.0):.3f} "
                f"vote_reason={target.get('vote_reason')}"
            )
            return

        servo_state, cmd = self.servo.compute_cmd(target)

        target_visible = bool(target.get("visible", False))
        if target_visible:
            self.found_bbox_count += 1
            self.has_seen_target = True
            self.last_seen_target_time = time.time()
            cx = target.get("cx", self.image_width / 2.0)
            ex = (cx - self.image_width / 2.0) / self.image_width
            if abs(cmd.angular.z) > 1e-4:
                self.last_target_wz = float(cmd.angular.z)
                self.last_target_wz_dir = 1.0 if cmd.angular.z > 0.0 else -1.0
            elif abs(ex) > 0.01:
                self.last_target_wz_dir = 1.0 if ex < 0.0 else -1.0
        fsm_state = self.fsm.update(target_visible, servo_state)

        if not self.has_seen_target:
            # 目标第一次出现前忽略所有导航/恢复动作，避免启动阶段乱转。
            self.publish_stop()
            action = "WAIT_TARGET_STOP"
        elif fsm_state == MVPState.SUCCESS:
            self.publish_stop()
            self.success = True
            action = "SUCCESS_STOP"
        elif servo_state == "LOST_STOP":
            action = self.publish_lost_recovery_cmd()
        else:
            self.cmd_pub.publish(cmd)
            self.last_cmd = cmd
            action = "SERVO_CMD"

        if target_visible:
            bbox = target.get("bbox", None)
            area_ratio = target.get("area_ratio", 0.0)
            cx = target.get("cx", 0.0)
            ex = (cx - self.image_width / 2.0) / self.image_width
            side = "左" if ex < -0.01 else ("右" if ex > 0.01 else "中")
            extra = ""
            if self.backend == "yolo_world":
                extra = (
                    f" class={target.get('class_name', '')}"
                    f" score={target.get('score', 0.0):.3f}"
                )
            self.get_logger().info(
                f"frame={self.frame_count} fsm={fsm_state} servo={servo_state} action={action} "
                f"bbox={bbox} cx={cx:.0f} ex={ex:+.3f} side={side} area={area_ratio:.3f}{extra} "
                f"cmd_vx={cmd.linear.x:.3f} cmd_wz={cmd.angular.z:+.3f}"
            )
        else:
            reason = target.get("reason", "not_visible")
            self.get_logger().info(
                f"frame={self.frame_count} fsm={fsm_state} servo={servo_state} action={action} "
                f"target=LOST reason={reason}"
            )

        if self.save_bbox:
            if target_visible:
                self.save_bbox_snapshot(vis_frame, target, fsm_state, servo_state, "found")
            elif self.frame_count % self.save_bbox_interval == 0:
                self.save_bbox_snapshot(vis_frame, target, fsm_state, servo_state, "interval")

        if self.save_debug and self.frame_count % 15 == 0:
            vis = frame.copy()
            if target_visible:
                x, y, w, h = target["bbox"]
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                label = f"{fsm_state} {servo_state}"
                if self.backend == "yolo_world":
                    label = (
                        f"{label} {target.get('class_name', '')}"
                        f" {target.get('score', 0.0):.2f}"
                    )
                cv2.putText(
                    vis,
                    label,
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
            save_path = os.path.join(self.debug_dir, f"frame_{self.frame_count:05d}.jpg")
            cv2.imwrite(save_path, vis)

        if self.success:
            elapsed = time.time() - self.start_time
            self.get_logger().info(f"===== TASK SUCCESS, elapsed={elapsed:.2f}s =====")
            self.publish_stop()

    def destroy_node(self):
        self.publish_stop()
        time.sleep(0.2)
        super().destroy_node()


def main():
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--mvp-tune-config", default=DEFAULT_TUNE_PATH)
    pre_args, _ = pre_parser.parse_known_args()
    tune = load_mvp_tune(pre_args.mvp_tune_config)

    parser = argparse.ArgumentParser()
    parser.add_argument("--mvp-tune-config", default=pre_args.mvp_tune_config)
    parser.add_argument("--instruction", default="find the red backpack")
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--backend", default="red", choices=["red", "yolo_world"])
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--target-words-topic", default="/target_words")
    parser.add_argument("--min-score", type=float, default=tune["min_score"])
    parser.add_argument("--min-red-ratio", type=float, default=tune["min_red_ratio"])
    parser.add_argument("--max-area-ratio", type=float, default=tune["max_area_ratio"])
    parser.add_argument("--no-red-verify", action="store_true")
    parser.add_argument("--det-stale-sec", type=float, default=tune["det_stale_sec"])
    parser.add_argument("--sync-max-delta-sec", type=float, default=tune["sync_max_delta_sec"])
    parser.add_argument("--max-vx", type=float, default=tune["max_vx"])
    parser.add_argument("--max-wz", type=float, default=tune["max_wz"])
    parser.add_argument("--kp-turn", type=float, default=tune["kp_turn"])
    parser.add_argument("--center-threshold", type=float, default=tune["turn_threshold"])
    parser.add_argument("--turn-threshold", type=float, default=tune["turn_threshold"])
    parser.add_argument("--forward-threshold", type=float, default=tune["forward_threshold"])
    parser.add_argument("--wz-deadzone", type=float, default=tune["wz_deadzone"])
    parser.add_argument("--cmd-wz-deadzone", type=float, default=tune["cmd_wz_deadzone"])
    parser.add_argument("--forward-turn-scale", type=float, default=tune["forward_turn_scale"])
    parser.add_argument("--recovery-scan-wz", type=float, default=tune["recovery_scan_wz"])
    parser.add_argument("--min-cruise-wz", type=float, default=tune["min_cruise_wz"])
    parser.add_argument("--lost-hold-frames", type=int, default=tune["lost_hold_frames"])
    parser.add_argument("--lost-observe-frames", type=int, default=tune["lost_observe_frames"])
    parser.add_argument("--recovery-scan-max-frames", type=int, default=tune["recovery_scan_max_frames"])
    parser.add_argument("--recovery-replan-sec", type=float, default=tune["recovery_replan_sec"])
    parser.add_argument("--lost-hold-wz-scale", type=float, default=tune["lost_hold_wz_scale"])
    parser.add_argument("--lost-hold-max-wz", type=float, default=tune["lost_hold_max_wz"])
    parser.add_argument("--recovery-pulse-frames", type=int, default=tune["recovery_pulse_frames"])
    parser.add_argument("--recovery-observe-frames", type=int, default=tune["recovery_observe_frames"])
    parser.add_argument("--arrive-area-ratio", type=float, default=tune["arrive_area_ratio"])
    parser.add_argument("--slowdown-area-ratio", type=float, default=tune["slowdown_area_ratio"])
    parser.add_argument("--stable-frames-required", type=int, default=tune["stable_frames_required"])
    parser.add_argument("--lost-frames-limit", type=int, default=tune["lost_frames_limit"])
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--save-debug", action="store_true")
    parser.add_argument(
        "--save-bbox-dir",
        default=None,
        help="Save annotated snapshots to this dir (e.g. check_bbox); set to enable",
    )
    parser.add_argument("--save-bbox-interval", type=int, default=15)
    parser.add_argument(
        "--target-classes",
        default=None,
        help="Override mock_qwen target_classes, e.g. bottle",
    )
    parser.add_argument(
        "--yolo-prompts",
        default=None,
        help="Override YOLO /target_words prompts, e.g. bottle",
    )
    parser.add_argument(
        "--no-publish-target-words",
        action="store_true",
        help="Do not publish /target_words (use external ros2 topic pub)",
    )
    parser.add_argument(
        "--multi-frame-voter",
        action="store_true",
        help="Use MultiFrameTargetVoter (same as live preview post-process)",
    )
    parser.add_argument("--vote-window-size", type=int, default=6)
    parser.add_argument("--vote-min-votes", type=int, default=2)
    parser.add_argument("--vote-lost-hold-frames", type=int, default=1)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = RunMVPTask(
        instruction=args.instruction,
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        backend=args.backend,
        image_width=args.image_width,
        image_height=args.image_height,
        det_topic=args.det_topic,
        target_words_topic=args.target_words_topic,
        min_score=args.min_score,
        det_stale_sec=args.det_stale_sec,
        save_debug=args.save_debug,
        save_bbox_dir=args.save_bbox_dir,
        save_bbox_interval=args.save_bbox_interval,
        min_red_ratio=args.min_red_ratio,
        max_area_ratio=args.max_area_ratio,
        require_red_verify=not args.no_red_verify,
        sync_max_delta_sec=args.sync_max_delta_sec,
        max_vx=args.max_vx,
        max_wz=args.max_wz,
        kp_turn=args.kp_turn,
        center_threshold=args.center_threshold,
        arrive_area_ratio=args.arrive_area_ratio,
        slowdown_area_ratio=args.slowdown_area_ratio,
        turn_threshold=args.turn_threshold,
        forward_threshold=args.forward_threshold,
        wz_deadzone=args.wz_deadzone,
        cmd_wz_deadzone=args.cmd_wz_deadzone,
        forward_turn_scale=args.forward_turn_scale,
        recovery_scan_wz=args.recovery_scan_wz,
        min_cruise_wz=args.min_cruise_wz,
        lost_hold_frames=args.lost_hold_frames,
        lost_observe_frames=args.lost_observe_frames,
        recovery_scan_max_frames=args.recovery_scan_max_frames,
        recovery_replan_sec=args.recovery_replan_sec,
        lost_hold_wz_scale=args.lost_hold_wz_scale,
        lost_hold_max_wz=args.lost_hold_max_wz,
        recovery_pulse_frames=args.recovery_pulse_frames,
        recovery_observe_frames=args.recovery_observe_frames,
        stable_frames_required=args.stable_frames_required,
        lost_frames_limit=args.lost_frames_limit,
        target_classes_override=args.target_classes,
        yolo_prompts_override=args.yolo_prompts,
        publish_target_words=not args.no_publish_target_words,
        use_multi_frame_voter=args.multi_frame_voter,
        vote_window_size=args.vote_window_size,
        vote_min_votes=args.vote_min_votes,
        vote_lost_hold_frames=args.vote_lost_hold_frames,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
