#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash

echo "[STOP] publish zero cmd_vel..."
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 >/dev/null 2>&1 || true

echo "[STOP] kill qwen lidar related processes..."
pkill -f run_qwen_lidar_nav.py || true
pkill -f run_qwen_pixel_task.py || true
pkill -f compressed_to_raw_image.py || true
pkill -f m1_pwm_cmd_vel_bridge.py || true
pkill -f cmd_vel_to_rosmaster.py || true

echo "[STOP] done."
