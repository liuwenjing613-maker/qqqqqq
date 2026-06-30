#!/usr/bin/env bash
set +u
source /opt/tros/humble/setup.bash
set -u

echo "===== /cmd_vel publishers ====="
ros2 topic info /cmd_vel -v || true

echo "===== /odom hz ====="
timeout 5s ros2 topic hz /odom || true

echo "===== /scan_filtered hz ====="
timeout 5s ros2 topic hz /scan_filtered || true

echo "===== chassis bridge state once ====="
timeout 3s ros2 topic echo /chassis_bridge_state --once || true

echo "===== TF odom -> base_link ====="
timeout 3s ros2 run tf2_ros tf2_echo odom base_link || true

echo "===== scan frame ====="
timeout 3s ros2 topic echo /scan_filtered --once | head -n 20 || true
