#!/usr/bin/env python3
import argparse
import json
import time
from collections import deque, Counter
from typing import List, Dict, Any, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String, Float32


COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
]


def letterbox(image: np.ndarray, new_shape=(640, 640), color=(114, 114, 114)):
    h, w = image.shape[:2]
    new_h, new_w = new_shape

    r = min(new_w / w, new_h / h)
    resized_w = int(round(w * r))
    resized_h = int(round(h * r))

    dw = new_w - resized_w
    dh = new_h - resized_h
    dw /= 2
    dh /= 2

    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))

    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right,
        cv2.BORDER_CONSTANT, value=color
    )
    return padded, r, (dw, dh)


def clip_box(x1, y1, x2, y2, w, h):
    x1 = max(0, min(int(round(x1)), w - 1))
    y1 = max(0, min(int(round(y1)), h - 1))
    x2 = max(0, min(int(round(x2)), w - 1))
    y2 = max(0, min(int(round(y2)), h - 1))
    return x1, y1, x2, y2


def decode_yolov5_output(
    out,
    conf_thres: float,
    iou_thres: float,
    wanted_classes: set,
    orig_w: int,
    orig_h: int,
    ratio: float,
    dw: float,
    dh: float,
    imgsz: int,
) -> List[Dict[str, Any]]:
    if isinstance(out, (list, tuple)):
        out = out[0]

    if out.ndim == 3:
        pred = out[0]
    elif out.ndim == 2:
        pred = out
    else:
        raise RuntimeError(f"Unsupported output shape: {out.shape}")

    boxes = []
    scores = []
    class_ids = []

    for row in pred:
        row = row.astype(float)

        # 标准 YOLOv5: [cx, cy, w, h, obj, cls0...cls79]
        if row.shape[0] >= 85:
            obj = row[4]
            cls_scores = row[5:]
            cls_id = int(np.argmax(cls_scores))
            cls_score = cls_scores[cls_id]
            conf = float(obj * cls_score)

            if conf < conf_thres:
                continue

            class_name = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else str(cls_id)
            if wanted_classes and class_name not in wanted_classes:
                continue

            cx, cy, bw, bh = row[:4]

            # 兼容归一化输出
            if max(cx, cy, bw, bh) <= 2.0:
                cx *= imgsz
                cy *= imgsz
                bw *= imgsz
                bh *= imgsz

            x1 = cx - bw / 2
            y1 = cy - bh / 2
            x2 = cx + bw / 2
            y2 = cy + bh / 2

        # 部分带 NMS 的 ONNX: [x1, y1, x2, y2, score, cls]
        elif row.shape[0] == 6:
            x1, y1, x2, y2, conf, cls_id = row
            cls_id = int(cls_id)
            conf = float(conf)

            if conf < conf_thres:
                continue

            class_name = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else str(cls_id)
            if wanted_classes and class_name not in wanted_classes:
                continue

            if max(x1, y1, x2, y2) <= 2.0:
                x1 *= imgsz
                y1 *= imgsz
                x2 *= imgsz
                y2 *= imgsz
        else:
            continue

        # letterbox 还原到原始图像坐标
        x1 = (x1 - dw) / ratio
        y1 = (y1 - dh) / ratio
        x2 = (x2 - dw) / ratio
        y2 = (y2 - dh) / ratio

        x1, y1, x2, y2 = clip_box(x1, y1, x2, y2, orig_w, orig_h)

        if x2 <= x1 or y2 <= y1:
            continue

        boxes.append([x1, y1, x2 - x1, y2 - y1])
        scores.append(float(conf))
        class_ids.append(cls_id)

    keep = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)

    detections = []
    if len(keep) > 0:
        keep = np.array(keep).reshape(-1).tolist()
        for idx in keep:
            x, y, w, h = boxes[idx]
            cls_id = class_ids[idx]
            class_name = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else str(cls_id)
            score = float(scores[idx])

            detections.append({
                "class_name": class_name,
                "score": score,
                "bbox": [int(x), int(y), int(x + w), int(y + h)],  # xyxy
                "u": float(x + w / 2),
                "v": float(y + h / 2),
                "cx": float(x + w / 2),
                "cy": float(y + h / 2),
                "area_ratio": float((w * h) / max(1.0, orig_w * orig_h)),
                "image_width": int(orig_w),
                "image_height": int(orig_h),
            })

    detections.sort(key=lambda d: d["score"], reverse=True)
    return detections


class Yolov5sOnnxRos(Node):
    def __init__(self, args):
        super().__init__("yolov5s_onnx_ros")

        self.model_path = args.model
        self.image_topic = args.image_topic
        self.image_type = args.image_type
        self.imgsz = int(args.imgsz)
        self.conf = float(args.conf)
        self.iou = float(args.iou)
        self.max_fps = float(args.max_fps)
        self.debug_scale = float(args.debug_scale)
        self.jpeg_quality = int(args.jpeg_quality)
        self.publish_raw_debug = bool(args.publish_raw_debug)
        self.wanted_classes = {x.strip() for x in args.classes.split(",") if x.strip()}

        if self.imgsz != 640:
            self.get_logger().warn(
                f"imgsz={self.imgsz} may fail: bundled yolov5s.onnx expects fixed 640 input"
            )

        self.bridge = CvBridge()
        if int(args.opencv_threads) > 0:
            cv2.setNumThreads(int(args.opencv_threads))
        self.net = cv2.dnn.readNetFromONNX(self.model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._warmup_model()

        # foxglove_bridge subscribes with RELIABLE; BEST_EFFORT publishers get no data.
        foxglove_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.bbox_pub = self.create_publisher(String, "/target_bbox_json", foxglove_qos)
        self.status_pub = self.create_publisher(String, "/yolov5s/stability", foxglove_qos)
        self.debug_pub = None
        if self.publish_raw_debug:
            self.debug_pub = self.create_publisher(Image, "/yolov5s/debug_image", foxglove_qos)
        self.debug_compressed_pub = self.create_publisher(
            CompressedImage,
            "/yolov5s/debug_image/compressed",
            foxglove_qos,
        )

        self.fps_pub = self.create_publisher(Float32, "/yolov5s/fps", foxglove_qos)
        self.infer_pub = self.create_publisher(Float32, "/yolov5s/infer_ms", foxglove_qos)
        self.visible_rate_pub = self.create_publisher(Float32, "/yolov5s/visible_rate", foxglove_qos)
        self.score_pub = self.create_publisher(Float32, "/yolov5s/score", foxglove_qos)
        self.jitter_pub = self.create_publisher(Float32, "/yolov5s/jitter_px", foxglove_qos)

        if self.image_type == "compressed":
            self.create_subscription(
                CompressedImage,
                self.image_topic,
                self.compressed_cb,
                qos_profile_sensor_data,
            )
        else:
            self.create_subscription(
                Image,
                self.image_topic,
                self.raw_cb,
                qos_profile_sensor_data,
            )

        self.last_process_time = 0.0
        self.last_frame_time = time.time()
        self.frame_id = 0

        self.window = deque(maxlen=int(args.window))
        self.prev_center: Optional[Tuple[float, float]] = None
        self.lost_streak = 0
        self.max_lost_streak = 0
        self.class_counter = Counter()

        self.get_logger().info("===== YOLOv5s ONNX ROS realtime test =====")
        self.get_logger().info(f"model={self.model_path}")
        self.get_logger().info(f"image_topic={self.image_topic} image_type={self.image_type}")
        self.get_logger().info(f"classes={self.wanted_classes}")
        self.get_logger().info(
            f"conf={self.conf} iou={self.iou} imgsz={self.imgsz} max_fps={self.max_fps} "
            f"debug_scale={self.debug_scale} jpeg_quality={self.jpeg_quality} "
            f"opencv_threads={cv2.getNumThreads()}"
        )
        self.get_logger().info("Foxglove Image topic: /yolov5s/debug_image/compressed")
        if self.publish_raw_debug:
            self.get_logger().info("Raw debug Image topic: /yolov5s/debug_image")
        self.get_logger().info("BBox JSON topic: /target_bbox_json")

    def _warmup_model(self) -> None:
        warmup = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        blob = cv2.dnn.blobFromImage(
            warmup,
            scalefactor=1.0 / 255.0,
            size=(self.imgsz, self.imgsz),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False,
        )
        t0 = time.time()
        self.net.setInput(blob)
        self.net.forward()
        warmup_ms = (time.time() - t0) * 1000.0
        self.get_logger().info(f"Model warmup done in {warmup_ms:.0f}ms")

    def compressed_cb(self, msg: CompressedImage):
        now = time.time()
        if self.max_fps > 0 and now - self.last_process_time < 1.0 / self.max_fps:
            return
        self.last_process_time = now

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("cv2.imdecode failed")
            return
        self.process_frame(frame, msg.header)

    def raw_cb(self, msg: Image):
        now = time.time()
        if self.max_fps > 0 and now - self.last_process_time < 1.0 / self.max_fps:
            return
        self.last_process_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return
        self.process_frame(frame, msg.header)

    def process_frame(self, frame: np.ndarray, header):
        self.frame_id += 1
        now = time.time()
        dt = max(1e-6, now - self.last_frame_time)
        fps_inst = 1.0 / dt
        self.last_frame_time = now

        orig_h, orig_w = frame.shape[:2]

        inp, ratio, (dw, dh) = letterbox(frame, (self.imgsz, self.imgsz))
        blob = cv2.dnn.blobFromImage(
            inp,
            scalefactor=1.0 / 255.0,
            size=(self.imgsz, self.imgsz),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False,
        )

        infer_start = time.time()
        self.net.setInput(blob)
        out = self.net.forward()
        infer_ms = (time.time() - infer_start) * 1000.0

        try:
            detections = decode_yolov5_output(
                out=out,
                conf_thres=self.conf,
                iou_thres=self.iou,
                wanted_classes=self.wanted_classes,
                orig_w=orig_w,
                orig_h=orig_h,
                ratio=ratio,
                dw=dw,
                dh=dh,
                imgsz=self.imgsz,
            )
        except Exception as exc:
            self.get_logger().warn(f"decode failed: {exc}")
            detections = []

        best = detections[0] if detections else None
        visible = best is not None
        jitter = 0.0

        if visible:
            self.lost_streak = 0
            cx, cy = float(best["cx"]), float(best["cy"])
            if self.prev_center is not None:
                jitter = float(np.hypot(cx - self.prev_center[0], cy - self.prev_center[1]))
            self.prev_center = (cx, cy)
            self.class_counter[best["class_name"]] += 1
        else:
            self.lost_streak += 1
            self.max_lost_streak = max(self.max_lost_streak, self.lost_streak)
            self.prev_center = None

        self.window.append({
            "visible": visible,
            "score": float(best["score"]) if best else 0.0,
            "jitter": float(jitter),
            "infer_ms": float(infer_ms),
            "fps": float(fps_inst),
        })

        visible_rate = sum(1 for x in self.window if x["visible"]) / max(1, len(self.window))
        mean_score = np.mean([x["score"] for x in self.window if x["visible"]]) if any(x["visible"] for x in self.window) else 0.0
        mean_jitter = np.mean([x["jitter"] for x in self.window if x["visible"]]) if any(x["visible"] for x in self.window) else 0.0
        mean_fps = np.mean([x["fps"] for x in self.window])
        mean_infer = np.mean([x["infer_ms"] for x in self.window])

        bbox_msg = {
            "visible": bool(visible),
            "source": "yolov5s_onnx",
            "frame_id": int(self.frame_id),
            "timestamp": float(now),
            "image_width": int(orig_w),
            "image_height": int(orig_h),
        }

        if best:
            bbox_msg.update(best)
        else:
            bbox_msg.update({
                "class_name": "",
                "score": 0.0,
                "bbox": [],
                "u": None,
                "v": None,
                "cx": None,
                "cy": None,
                "area_ratio": 0.0,
                "reason": "no_detection",
            })

        status = {
            "frame_id": int(self.frame_id),
            "visible": bool(visible),
            "best": best,
            "num_detections": len(detections),
            "fps_inst": float(fps_inst),
            "fps_mean_window": float(mean_fps),
            "infer_ms": float(infer_ms),
            "infer_ms_mean_window": float(mean_infer),
            "visible_rate_window": float(visible_rate),
            "mean_score_window": float(mean_score),
            "jitter_px": float(jitter),
            "mean_jitter_px_window": float(mean_jitter),
            "lost_streak": int(self.lost_streak),
            "max_lost_streak": int(self.max_lost_streak),
            "class_counter": dict(self.class_counter),
            "classes": sorted(list(self.wanted_classes)),
            "conf": self.conf,
        }

        self.bbox_pub.publish(String(data=json.dumps(bbox_msg, ensure_ascii=False)))
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

        self.fps_pub.publish(Float32(data=float(mean_fps)))
        self.infer_pub.publish(Float32(data=float(infer_ms)))
        self.visible_rate_pub.publish(Float32(data=float(visible_rate)))
        self.score_pub.publish(Float32(data=float(best["score"]) if best else 0.0))
        self.jitter_pub.publish(Float32(data=float(jitter)))

        debug = self.build_debug_image(frame, detections, status)

        ok, encoded = cv2.imencode(
            ".jpg",
            debug,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if ok:
            jpg_msg = CompressedImage()
            jpg_msg.header = header
            jpg_msg.format = "jpeg"
            jpg_msg.data = encoded.tobytes()
            self.debug_compressed_pub.publish(jpg_msg)
        else:
            self.get_logger().warn("cv2.imencode debug jpeg failed")

        if self.debug_pub is not None:
            img_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            img_msg.header = header
            self.debug_pub.publish(img_msg)

        if self.frame_id % 10 == 0:
            if best:
                self.get_logger().info(
                    f"visible cls={best['class_name']} score={best['score']:.3f} "
                    f"bbox={best['bbox']} vr={visible_rate:.2f} fps={mean_fps:.2f} infer={infer_ms:.1f}ms"
                )
            else:
                self.get_logger().warn(
                    f"NO target vr={visible_rate:.2f} lost={self.lost_streak} "
                    f"fps={mean_fps:.2f} infer={infer_ms:.1f}ms"
                )

    def build_debug_image(self, frame, detections, status):
        scale = self.debug_scale if 0.0 < self.debug_scale < 1.0 else 1.0
        if scale != 1.0:
            vis = cv2.resize(
                frame,
                (max(1, int(frame.shape[1] * scale)), max(1, int(frame.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )
        else:
            vis = frame

        h, w = vis.shape[:2]

        for i, det in enumerate(detections[:8]):
            x1 = int(det["bbox"][0] * scale)
            y1 = int(det["bbox"][1] * scale)
            x2 = int(det["bbox"][2] * scale)
            y2 = int(det["bbox"][3] * scale)
            cx = int(det["cx"] * scale)
            cy = int(det["cy"] * scale)
            color = (0, 255, 0) if i == 0 else (0, 255, 255)
            thickness = max(1, int((3 if i == 0 else 2) * scale))
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
            cv2.circle(vis, (cx, cy), max(2, int(4 * scale)), color, -1)
            label = f"{det['class_name']} {det['score']:.2f} a={det['area_ratio']:.3f}"
            cv2.putText(
                vis,
                label,
                (x1, max(16, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                max(0.45, 0.65 * scale),
                color,
                max(1, int(2 * scale)),
                cv2.LINE_AA,
            )

        cv2.line(vis, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)

        lines = [
            f"YOLOv5s ONNX | visible={status['visible']} dets={status['num_detections']}",
            f"fps={status['fps_mean_window']:.2f} infer={status['infer_ms']:.1f}ms conf={self.conf}",
            f"visible_rate={status['visible_rate_window']:.2f} score_mean={status['mean_score_window']:.3f}",
            f"jitter={status['jitter_px']:.1f}px lost={status['lost_streak']} max_lost={status['max_lost_streak']}",
            f"classes={','.join(sorted(list(self.wanted_classes)))}",
        ]

        x0, y0 = 8, 10
        panel_w = min(w - 16, 420)
        panel_h = int(18 * len(lines) + 10)
        overlay = vis.copy()
        cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, vis, 0.45, 0, vis)

        for idx, line in enumerate(lines):
            cv2.putText(
                vis,
                line,
                (x0 + 6, y0 + 18 + idx * 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                max(0.4, 0.55 * scale),
                (255, 255, 255),
                max(1, int(2 * scale)),
                cv2.LINE_AA,
            )

        return vis


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="/root/rdk_x5_vln_robot/models/yolov5s.onnx")
    parser.add_argument("--image-topic", default="/image")
    parser.add_argument("--image-type", choices=["compressed", "raw"], default="compressed")
    parser.add_argument("--classes", default="cup,bottle,backpack")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-fps", type=float, default=4.0)
    parser.add_argument("--debug-scale", type=float, default=0.45)
    parser.add_argument("--jpeg-quality", type=int, default=55)
    parser.add_argument("--opencv-threads", type=int, default=4)
    parser.add_argument("--publish-raw-debug", action="store_true")
    parser.add_argument("--window", type=int, default=60)
    args = parser.parse_args()

    rclpy.init()
    node = Yolov5sOnnxRos(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
