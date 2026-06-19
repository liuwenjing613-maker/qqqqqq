#!/usr/bin/env bash
set -e

cd /root/rdk_x5_vln_robot
source /opt/tros/humble/setup.bash

echo "[STOP] publish zero cmd_vel..."
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
-r 10 || true

echo "[STOP] kill qwen lidar related processes..."
pkill -f run_qwen_lidar_nav.py || true
pkill -f run_qwen_pixel_task.py || true
pkill -f compressed_to_raw_image.py || true
pkill -f chassis_cmdvel_bridge.py || true

echo "[STOP] done."