#!/usr/bin/env bash
# 只停止相机和 YOLO 检测相关进程；不停止 nav / lidar / chassis / foxglove。
set -euo pipefail

echo "[stop_yolo_camera] stopping camera + yolo only..."

# YOLO detectors / bridges
pkill -f yolov5s_bpu_web_node.py || true
pkill -f yolov5s_onnx_ros.py || true
pkill -f yolo_world_to_bbox_json.py || true
pkill -f hobot_yolo_world || true
pkill -f yolo_live_browser_preview.py || true

# Camera + image bridge
pkill -f compressed_to_raw_image.py || true
pkill -f hobot_usb_cam || true
pkill -f "perception/launch/usb_cam.launch.py" || true

echo "[stop_yolo_camera] done (nav/lidar/chassis/foxglove untouched)."
