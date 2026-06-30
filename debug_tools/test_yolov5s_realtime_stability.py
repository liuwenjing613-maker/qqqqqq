#!/usr/bin/env python3
import argparse
import csv
import os
import time
from collections import Counter

import cv2
import numpy as np


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


def letterbox(image, new_shape=(640, 640), color=(114, 114, 114)):
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
        resized,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=color
    )
    return padded, r, (dw, dh)


def clip_box(x1, y1, x2, y2, w, h):
    x1 = max(0, min(int(round(x1)), w - 1))
    y1 = max(0, min(int(round(y1)), h - 1))
    x2 = max(0, min(int(round(x2)), w - 1))
    y2 = max(0, min(int(round(y2)), h - 1))
    return x1, y1, x2, y2


def decode_yolov5_output(out, conf_thres, iou_thres, wanted_classes,
                         orig_w, orig_h, ratio, dw, dh, imgsz):
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

        # 标准 YOLOv5: cx, cy, w, h, obj, cls0...cls79
        if row.shape[0] >= 85:
            obj = row[4]
            cls_scores = row[5:]
            cls_id = int(np.argmax(cls_scores))
            cls_score = cls_scores[cls_id]
            conf = obj * cls_score

            if conf < conf_thres:
                continue

            class_name = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else str(cls_id)
            if wanted_classes and class_name not in wanted_classes:
                continue

            cx, cy, bw, bh = row[:4]

            # 如果有些模型输出是 0~1 归一化坐标，做一次兼容
            if max(cx, cy, bw, bh) <= 2.0:
                cx *= imgsz
                cy *= imgsz
                bw *= imgsz
                bh *= imgsz

            x1 = cx - bw / 2
            y1 = cy - bh / 2
            x2 = cx + bw / 2
            y2 = cy + bh / 2

        # 部分带 NMS 的模型: x1, y1, x2, y2, score, cls
        elif row.shape[0] == 6:
            x1, y1, x2, y2, conf, cls_id = row
            cls_id = int(cls_id)

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

        # 从 letterbox 输入坐标还原到原图坐标
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
            score = scores[idx]

            detections.append({
                "class_name": class_name,
                "score": float(score),
                "bbox": [int(x), int(y), int(x + w), int(y + h)],
                "center": [float(x + w / 2), float(y + h / 2)],
                "area_ratio": float((w * h) / max(1.0, orig_w * orig_h)),
            })

    detections.sort(key=lambda d: d["score"], reverse=True)
    return detections


def open_source(source, width, height):
    if str(source).isdigit():
        cap = cv2.VideoCapture(int(source))
    else:
        cap = cv2.VideoCapture(source)

    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    return cap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Path to yolov5s ONNX model")
    parser.add_argument("--source", default="0", help="0 for webcam, or video path")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--classes", default="cup,bottle,backpack",
                        help="comma-separated target COCO classes")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--cam-height", type=int, default=480)
    parser.add_argument("--no-view", action="store_true")
    parser.add_argument("--save-video", default="")
    parser.add_argument("--csv", default="yolov5s_stability_log.csv")
    args = parser.parse_args()

    wanted_classes = {x.strip() for x in args.classes.split(",") if x.strip()}

    print("Loading ONNX:", args.model)
    net = cv2.dnn.readNetFromONNX(args.model)

    cap = open_source(args.source, args.cam_width, args.cam_height)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open source: {args.source}")

    writer = None
    csv_file = open(args.csv, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "frame_id", "time_sec", "fps_inst", "visible", "class_name", "score",
        "x1", "y1", "x2", "y2", "cx", "cy", "area_ratio", "center_jitter_px"
    ])

    total_frames = 0
    visible_frames = 0
    lost_frames = 0
    max_lost_streak = 0
    current_lost_streak = 0
    scores = []
    jitters = []
    class_counter = Counter()

    prev_center = None
    start_time = time.time()
    last_time = start_time

    print("Start realtime test")
    print("Target classes:", wanted_classes)
    print("Press q to quit if view window is enabled.")

    while True:
        now = time.time()
        if args.duration > 0 and now - start_time > args.duration:
            break

        ret, frame = cap.read()
        if not ret:
            print("Frame read failed")
            break

        total_frames += 1
        orig_h, orig_w = frame.shape[:2]

        inp, ratio, (dw, dh) = letterbox(frame, (args.imgsz, args.imgsz))

        blob = cv2.dnn.blobFromImage(
            inp,
            scalefactor=1.0 / 255.0,
            size=(args.imgsz, args.imgsz),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False
        )

        infer_start = time.time()
        net.setInput(blob)
        out = net.forward()
        infer_end = time.time()

        detections = decode_yolov5_output(
            out=out,
            conf_thres=args.conf,
            iou_thres=args.iou,
            wanted_classes=wanted_classes,
            orig_w=orig_w,
            orig_h=orig_h,
            ratio=ratio,
            dw=dw,
            dh=dh,
            imgsz=args.imgsz
        )

        curr_time = time.time()
        fps_inst = 1.0 / max(1e-6, curr_time - last_time)
        last_time = curr_time

        best = detections[0] if detections else None
        center_jitter = 0.0

        vis = frame.copy()

        if best is not None:
            visible_frames += 1
            current_lost_streak = 0

            x1, y1, x2, y2 = best["bbox"]
            cx, cy = best["center"]
            score = best["score"]
            class_name = best["class_name"]
            area_ratio = best["area_ratio"]

            if prev_center is not None:
                center_jitter = float(np.hypot(cx - prev_center[0], cy - prev_center[1]))
                jitters.append(center_jitter)
            prev_center = (cx, cy)

            scores.append(score)
            class_counter[class_name] += 1

            label = f"{class_name} {score:.2f} area={area_ratio:.3f}"
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(vis, (int(cx), int(cy)), 4, (0, 255, 0), -1)
            cv2.putText(vis, label, (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            csv_writer.writerow([
                total_frames, curr_time - start_time, fps_inst, 1, class_name, score,
                x1, y1, x2, y2, cx, cy, area_ratio, center_jitter
            ])
        else:
            lost_frames += 1
            current_lost_streak += 1
            max_lost_streak = max(max_lost_streak, current_lost_streak)
            prev_center = None

            cv2.putText(vis, "NO TARGET", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            csv_writer.writerow([
                total_frames, curr_time - start_time, fps_inst, 0, "", 0,
                0, 0, 0, 0, 0, 0, 0, 0
            ])

        cv2.putText(vis, f"FPS {fps_inst:.2f} infer {(infer_end - infer_start) * 1000:.1f} ms",
                    (20, orig_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if args.save_video:
            if writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(args.save_video, fourcc, 15.0, (orig_w, orig_h))
            writer.write(vis)

        if not args.no_view:
            cv2.imshow("YOLOv5s realtime stability", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    elapsed = time.time() - start_time
    avg_fps = total_frames / max(elapsed, 1e-6)
    visible_rate = visible_frames / max(total_frames, 1)
    mean_score = float(np.mean(scores)) if scores else 0.0
    std_score = float(np.std(scores)) if scores else 0.0
    mean_jitter = float(np.mean(jitters)) if jitters else 0.0
    max_lost_sec = max_lost_streak / max(avg_fps, 1e-6)

    print("\n===== STABILITY SUMMARY =====")
    print(f"elapsed_sec        : {elapsed:.2f}")
    print(f"total_frames       : {total_frames}")
    print(f"avg_fps            : {avg_fps:.2f}")
    print(f"visible_frames     : {visible_frames}")
    print(f"visible_rate       : {visible_rate * 100:.1f}%")
    print(f"mean_score         : {mean_score:.3f}")
    print(f"std_score          : {std_score:.3f}")
    print(f"mean_center_jitter : {mean_jitter:.2f} px")
    print(f"max_lost_streak    : {max_lost_streak} frames")
    print(f"max_lost_seconds   : {max_lost_sec:.2f} sec")
    print(f"class_counter      : {dict(class_counter)}")
    print(f"csv_saved          : {args.csv}")
    if args.save_video:
        print(f"video_saved        : {args.save_video}")

    csv_file.close()
    cap.release()
    if writer is not None:
        writer.release()
    if not args.no_view:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
