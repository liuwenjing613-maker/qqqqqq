#!/usr/bin/env bash

echo "[STOP] stopping robot-related processes..."

pkill -f "run_mvp_task.py" 2>/dev/null || true
pkill -f "red_target_servo_ros.py" 2>/dev/null || true
pkill -f "yolo_world_servo_ros.py" 2>/dev/null || true
pkill -f "yolo_world_bbox_preview.py" 2>/dev/null || true
pkill -f "keyboard_cmd_vel.py" 2>/dev/null || true
pkill -f "cmd_vel_to_rosmaster.py" 2>/dev/null || true
pkill -f "compressed_to_raw_image.py" 2>/dev/null || true
pkill -f "save_raw_image_once.py" 2>/dev/null || true
pkill -f "hobot_yolo_world" 2>/dev/null || true
pkill -f "ros2 topic pub -r 1 /target_words" 2>/dev/null || true
pkill -f "target_words std_msgs/msg/String" 2>/dev/null || true
pkill -f "hobot_usb_cam" 2>/dev/null || true
pkill -f "websocket" 2>/dev/null || true

sleep 0.5

source /opt/tros/humble/setup.bash

echo "[STOP] publish zero /cmd_vel once..."
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true

echo "[STOP] done."
