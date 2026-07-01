#!/usr/bin/env bash
set -euo pipefail
cd /root/rdk_x5_vln_robot

echo "===== ENV CONFIG ====="
grep -n "CHASSIS_MOTOR_SIGNS\|CHASSIS_MOTOR_TRIMS\|CHASSIS_ODOM_VX_SCALE\|CHASSIS_ODOM_VY_SCALE\|CHASSIS_ODOM_WZ_SCALE\|CHASSIS_ODOM_XY_YAW_OFFSET\|CHASSIS_BASE_YAW_OFFSET\|LASER_YAW\|JOY_SCALE" scripts/lib/slam_calibrated_env.sh scripts/lib/lidar_frame_config.sh || true

echo
echo "===== RUN CHASSIS BRIDGE ARGS ====="
grep -n "odom-vx-scale\|odom-vy-scale\|odom-wz-scale\|odom-xy-yaw-offset\|base-yaw-offset\|motor-signs\|motor-trims" scripts/lib/run_chassis_bridge.sh || true

echo
echo "===== M1 BRIDGE YAW LOGIC ====="
grep -n "odom_xy_yaw_offset\|base_yaw_offset\|xy_yaw_before\|published_yaw_before\|published_yaw" ros2_bridge/m1_pwm_cmd_vel_bridge.py || true

echo
echo "===== LASER STATIC TF SOURCE ====="
grep -n "static_transform_publisher\|LASER_YAW\|frame-id base_link\|child-frame-id" scripts/slam/run_corridor_mapping_live_foxglove.sh scripts/lib/lidar_frame_config.sh || true
