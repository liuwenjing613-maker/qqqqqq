#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

LIDAR_DEV="${LIDAR_DEV:-/dev/ydlidar}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"

echo "============================================================"
echo " RDK X5 VLN Robot - Lidar Path Test"
echo "  lidar + chassis bridge + keyboard teleop"
echo "============================================================"
echo "PROJECT_DIR  = $PROJECT_DIR"
echo "Tune file    = $MVP_TUNE_FILE"
echo "LIDAR_DEV    = $LIDAR_DEV"
echo "CHASSIS_PORT = $CHASSIS_PORT"
echo "CMD_TOPIC    = $CMD_TOPIC"
echo "============================================================"

cd "$PROJECT_DIR"
mkdir -p logs

echo "[0/4] stop old processes..."
bash scripts/system/stop_all_safe.sh || true
sleep 1

echo "[1/4] start lidar driver -> /scan"
bash scripts/lidar/start_lidar_only.sh
sleep 2

echo "[2/4] start chassis bridge (PWM): $CMD_TOPIC -> M1"
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/lidar_test_chassis_bridge.log"
sleep 2

echo "[3/4] check /scan topic..."
source "$PROJECT_DIR/scripts/lidar/source_ydlidar.sh"
ros2 topic info /scan || true

echo "[4/4] start keyboard teleop (w/s/a/d/x/q)"
echo "Use another terminal to monitor:"
echo "  ros2 topic hz /scan"
echo "  ros2 topic echo /scan --once"
cd "$PROJECT_DIR/ros2_bridge"
python3 keyboard_cmd_vel.py --cmd-topic "$CMD_TOPIC"
