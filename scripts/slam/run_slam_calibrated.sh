#!/usr/bin/env bash
set -e

cd /root/rdk_x5_vln_robot
set +u
source /opt/tros/humble/setup.bash
set -u

# shellcheck source=scripts/lib/slam_calibrated_env.sh
source "${PWD}/scripts/lib/slam_calibrated_env.sh"
export SLAM_USE_CALIBRATION=1

echo "[INFO] Starting calibrated SLAM stack..."
echo "[INFO] CHASSIS_MOTOR_TRIMS=${CHASSIS_MOTOR_TRIMS}"
echo "[INFO] CHASSIS_ODOM_VX_SCALE=${CHASSIS_ODOM_VX_SCALE}"
echo "[INFO] CHASSIS_ODOM_WZ_SCALE=${CHASSIS_ODOM_WZ_SCALE}"
echo "[INFO] CHASSIS_ODOM_USE_VY=${CHASSIS_ODOM_USE_VY}"
echo "[INFO] CHASSIS_MAX_VX=${CHASSIS_MAX_VX} CHASSIS_MAX_WZ=${CHASSIS_MAX_WZ}"

bash scripts/slam/run_corridor_mapping_live_foxglove.sh
