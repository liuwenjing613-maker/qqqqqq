#!/usr/bin/env python3
"""
YOLO-World 诊断预览：订阅 /image_raw + /hobot_yolo_world，绘制全部检测框并保存快照。

不发布 /cmd_vel，不驱动底盘。用于排查模型原始输出与 MVP 过滤差异。

用法（通常由 scripts/yolo/start_yolo_diag_raw.sh 启动）：
  source /opt/tros/humble/setup.bash
  python3 ~/rdk_x5_vln_robot/debug_tools/yolo_world_diag_preview.py
"""

import argparse
import os
import sys
import time

import cv2
import rclpy
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.config.mvp_tune import DEFAULT_TUNE_PATH, load_mvp_tune
from src.perception.target_backend_yolo import (
    extract_yolo_target,
    list_yolo_candidates,
    parse_target_classes,
    _pick_best_rejected,
)
from src.perception.stamp_sync import StampSyncBuffer


class YoloWorldDiagPreview(Node):
    def __init__(
        self,
        image_topic="/image_raw",
        det_topic="/hobot_yolo_world",
        save_dir=None,
        target_classes="backpack,handbag,suitcase",
        image_width=1280,
        image_height=720,
        min_score=0.002,
        raw_min_score=0.0,
        min_red_ratio=0.06,
        max_area_ratio=0.15,
        require_red_verify=True,
        save_interval=15,
        show_all_boxes=False,
        sync_max_delta_sec=0.12,
        sync_buffer_len=60,
    ):
        super().__init__("yolo_world_diag_preview")

        self.image_topic = image_topic
        self.det_topic = det_topic
        self.save_dir = os.path.expanduser(
            save_dir or os.path.join(PROJECT_ROOT, "check_bbox")
        )
        self.target_classes = parse_target_classes(target_classes)
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.min_score = float(min_score)
        self.raw_min_score = float(raw_min_score)
        self.min_red_ratio = float(min_red_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.require_red_verify = bool(require_red_verify)
        self.save_interval = max(1, int(save_interval))
        self.show_all_boxes = bool(show_all_boxes)
        self.sync_max_delta_sec = float(sync_max_delta_sec)

        self.bridge = CvBridge()
        self.frame_buffer = StampSyncBuffer(
            max_len=sync_buffer_len,
            max_delta_sec=self.sync_max_delta_sec,
        )
        self.det_buffer = StampSyncBuffer(
            max_len=sync_buffer_len,
            max_delta_sec=self.sync_max_delta_sec,
        )
        self.synced_frame = None
        self.synced_det_msg = None
        self.last_sync_warn_time = 0.0
        self.sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.frame_count = 0
        self.det_count = 0
        self.found_count = 0
        self.last_log_time = 0.0

        os.makedirs(self.save_dir, exist_ok=True)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.sensor_qos,
        )
        self.det_sub = self.create_subscription(
            PerceptionTargets,
            self.det_topic,
            self.det_callback,
            self.sensor_qos,
        )

        self.get_logger().info("===== yolo_world_diag_preview =====")
        self.get_logger().info(f"image_topic={self.image_topic}")
        self.get_logger().info(f"det_topic={self.det_topic}")
        self.get_logger().info(f"target_classes={self.target_classes}")
        self.get_logger().info(
            f"mvp_min={self.min_score} raw_min={self.raw_min_score} "
            f"red_min={self.min_red_ratio} max_area={self.max_area_ratio}"
        )
        self.get_logger().info(f"require_red_verify={self.require_red_verify}")
        self.get_logger().info(
            f"stamp_sync max_delta={self.sync_max_delta_sec}s buffer={sync_buffer_len}"
        )
        self.get_logger().info(f"save_dir={self.save_dir} save_interval={self.save_interval}")

    def _extract_kwargs(self, frame):
        return dict(
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.min_score,
            max_area_ratio=self.max_area_ratio,
            frame=frame,
            min_red_ratio=self.min_red_ratio,
            require_red_verify=self.require_red_verify,
            min_red_iou=0.10,
        )

    def _sync_pair_from_det(self, det_msg):
        """YOLO 输出 stamp 等于其处理的源图 stamp；用 det stamp 回查 frame 缓存。"""
        det_stamp = det_msg.header.stamp
        frame, delta = self.frame_buffer.find_closest(det_stamp)
        if frame is None:
            now = time.time()
            if now - self.last_sync_warn_time >= 1.0:
                self.last_sync_warn_time = now
                delta_str = f"{delta:.3f}s" if delta is not None else "no_frame"
                self.get_logger().warn(
                    f"stamp sync skip: no frame for det stamp, delta {delta_str} "
                    f"> {self.sync_max_delta_sec}s (frame_buf={len(self.frame_buffer)})"
                )
            return None, None
        return frame, det_msg

    def _process_synced(self, frame, det_msg):
        if frame is None or det_msg is None:
            return {"visible": False, "reason": "no_sync"}, []

        raw_dets = list_yolo_candidates(
            det_msg,
            target_classes=self.target_classes,
            image_width=self.image_width,
            image_height=self.image_height,
            min_score=self.raw_min_score,
            frame=frame,
            require_red_verify=False,
            max_area_ratio=self.max_area_ratio,
            min_area_ratio=0.0,
        )
        mvp_target = extract_yolo_target(det_msg, **self._extract_kwargs(frame))
        return mvp_target, raw_dets

    def _process_on_det(self, det_msg):
        synced_frame, synced_det = self._sync_pair_from_det(det_msg)
        if synced_frame is None or synced_det is None:
            return

        self.synced_frame = synced_frame
        self.synced_det_msg = synced_det
        self.frame_count += 1
        mvp_target, raw_dets = self._process_synced(synced_frame, synced_det)

        now = time.time()
        if now - self.last_log_time >= 1.0 and raw_dets is not None:
            self.last_log_time = now
            if raw_dets:
                summary = ", ".join(
                    f"{d['class_name']}:{d['score']:.3f}(red={d.get('red_ratio', 0):.2f})"
                    for d in raw_dets[:5]
                )
                self.get_logger().info(
                    f"frame#{self.frame_count} raw={len(raw_dets)} [{summary}]"
                )
            else:
                self.get_logger().info(f"frame#{self.frame_count} raw=0")

        if mvp_target and mvp_target.get("visible", False):
            self.found_count += 1
            self.get_logger().info(
                f"MVP_FOUND class={mvp_target['class_name']} "
                f"score={mvp_target['score']:.3f} red={mvp_target.get('red_ratio', 0):.2f} "
                f"bbox={mvp_target['bbox']} area={mvp_target['area_ratio']:.3f}"
            )
            self.render_and_save(
                trigger="found",
                mvp_target=mvp_target,
                raw_dets=raw_dets,
            )
        elif self.frame_count % self.save_interval == 0:
            self.render_and_save(
                trigger="interval",
                mvp_target=mvp_target,
                raw_dets=raw_dets,
            )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {repr(e)}")
            return

        self.frame_buffer.push(msg.header.stamp, frame)

    def det_callback(self, msg: PerceptionTargets):
        stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else None
        if stamp is None:
            return
        self.det_buffer.push(stamp, msg)
        self.det_count += 1
        self._process_on_det(msg)

    def render_and_save(self, trigger="interval", mvp_target=None, raw_dets=None):
        if self.synced_frame is None:
            return

        frame = self.synced_frame.copy()
        h, w = frame.shape[:2]

        if raw_dets is None or mvp_target is None:
            mvp_target, raw_dets = self._process_synced(
                self.synced_frame, self.synced_det_msg
            )

        class_dets = raw_dets or []

        mvp_bbox = None
        if mvp_target and mvp_target.get("visible", False):
            mvp_bbox = tuple(mvp_target["bbox"])

        def _draw_box(det, color, thickness, tag):
            x, y, bw, bh = det["bbox"]
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, thickness)
            red_ratio = det.get("red_ratio", 0.0)
            label = f"{tag} {det['class_name']} {det['score']:.3f} r={red_ratio:.2f}"
            cv2.putText(
                frame,
                label,
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )

        if mvp_target and mvp_target.get("visible", False):
            _draw_box(mvp_target, (0, 255, 0), 3, "MVP")
        elif self.show_all_boxes:
            for det in class_dets:
                if det["area_ratio"] > self.max_area_ratio:
                    continue
                is_mvp = mvp_bbox is not None and det["bbox"] == list(mvp_bbox)
                if is_mvp:
                    _draw_box(det, (0, 255, 0), 3, "MVP")
                elif not det["visible"]:
                    _draw_box(det, (0, 0, 255), 1, f"REJ:{det.get('reject_reason', '?')}")
                elif det["score"] >= self.min_score:
                    _draw_box(det, (0, 255, 255), 1, "CLS")

        status = "FOUND" if mvp_target and mvp_target.get("visible", False) else "NO_MVP"
        reject_hint = ""
        if status == "NO_MVP" and mvp_target and mvp_target.get("reason"):
            reject_hint = f" reject={mvp_target['reason']}"
        best_raw = "none"
        if raw_dets:
            best = _pick_best_rejected(raw_dets, self.max_area_ratio) or raw_dets[0]
            det = best
            best_raw = (
                f"{det['class_name']} score={det['score']:.4f} "
                f"bbox={det['bbox']} red={det.get('red_ratio', 0.0):.2f} "
                f"rej={det.get('reject_reason') or 'ok'}"
            )

        lines = [
            f"YOLO diag frame={self.frame_count} det={self.det_count} found={self.found_count}",
            (
                f"raw={len(raw_dets or [])} mvp_min={self.min_score} "
                f"red_min={self.min_red_ratio} max_area={self.max_area_ratio}"
            ),
            f"status={status}{reject_hint} trigger={trigger}",
            f"best_raw={best_raw}",
            f"{w}x{h} topic={self.image_topic}",
        ]
        self._draw_text_block(frame, lines)

        stamp = time.strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(
            self.save_dir,
            f"yolo_diag_{stamp}_f{self.frame_count}_{status}.jpg",
        )
        cv2.imwrite(save_path, frame)
        self.get_logger().info(f"saved: {save_path}")

    @staticmethod
    def _draw_text_block(img, lines, origin=(12, 12), line_height=22, font_scale=0.55):
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
            cv2.putText(
                img,
                line,
                (x0 + 10, y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
            y += line_height


def main():
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--mvp-tune-config", default=DEFAULT_TUNE_PATH)
    pre_args, _ = pre_parser.parse_known_args()
    tune = load_mvp_tune(pre_args.mvp_tune_config)

    parser = argparse.ArgumentParser(description="YOLO-World diagnostic preview (no chassis).")
    parser.add_argument("--mvp-tune-config", default=pre_args.mvp_tune_config)
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--det-topic", default="/hobot_yolo_world")
    parser.add_argument("--save-dir", default=None)
    parser.add_argument("--target-classes", default="backpack,handbag,suitcase")
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--min-score", type=float, default=tune["min_score"], help="MVP filter threshold")
    parser.add_argument("--raw-min-score", type=float, default=0.0, help="Draw all boxes above this")
    parser.add_argument("--min-red-ratio", type=float, default=tune["min_red_ratio"], help="HSV red ratio in bbox")
    parser.add_argument("--max-area-ratio", type=float, default=tune["max_area_ratio"], help="Reject oversized boxes")
    parser.add_argument("--no-red-verify", action="store_true")
    parser.add_argument("--show-all-boxes", action="store_true", help="Draw every raw/filtered box")
    parser.add_argument("--save-interval", type=int, default=15)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = YoloWorldDiagPreview(
        image_topic=args.image_topic,
        det_topic=args.det_topic,
        save_dir=args.save_dir,
        target_classes=args.target_classes,
        image_width=args.image_width,
        image_height=args.image_height,
        min_score=args.min_score,
        raw_min_score=args.raw_min_score,
        min_red_ratio=args.min_red_ratio,
        max_area_ratio=args.max_area_ratio,
        require_red_verify=not args.no_red_verify,
        save_interval=args.save_interval,
        show_all_boxes=args.show_all_boxes,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"summary: frames={node.frame_count} dets={node.det_count} mvp_found={node.found_count}"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
