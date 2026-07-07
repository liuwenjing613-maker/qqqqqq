#!/usr/bin/env bash
set -euo pipefail

pkill -TERM -f run_shared_nav_semantic_explore 2>/dev/null || true
pkill -TERM -f semantic_mapper_node 2>/dev/null || true
pkill -TERM -f explore_goal_selector 2>/dev/null || true
sleep 1
pkill -KILL -f run_shared_nav_semantic_explore 2>/dev/null || true
pkill -KILL -f semantic_mapper_node 2>/dev/null || true
pkill -KILL -f explore_goal_selector 2>/dev/null || true
pkill -f run_shared_nav.py || true
pkill -f yolov5s_bpu_web_node.py || true
pkill -f yolo_world_to_bbox_json.py || true
pkill -f hobot_yolo_world || true
pkill -f compressed_to_raw_image.py || true
pkill -f hobot_usb_cam || true
pkill -TERM -f "run_slam_calibrated.sh" 2>/dev/null || true
pkill -TERM -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
pkill -TERM -f "async_slam_toolbox_node" 2>/dev/null || true
sleep 1
pkill -KILL -f "run_corridor_mapping_live_foxglove.sh" 2>/dev/null || true
pkill -KILL -f "run_slam_calibrated.sh" 2>/dev/null || true
pkill -KILL -f "async_slam_toolbox_node" 2>/dev/null || true
pkill -f ydlidar_ros2_driver_node || true
pkill -f m1_pwm_cmd_vel_bridge.py || true
pkill -f cmd_vel_to_rosmaster.py || true
pkill -f foxglove_bridge || true

timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  >/dev/null 2>&1 || true

echo "[stop_nav] stopped."
