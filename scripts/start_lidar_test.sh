#!/usr/bin/env bash
set -e

PROJECT_DIR="${HOME}/rdk_x5_vln_robot"
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
bash scripts/stop_all_safe.sh || true
sleep 1

echo "[1/4] start lidar driver -> /scan"
bash scripts/start_lidar_only.sh
sleep 2

echo "[2/4] start chassis bridge: $CMD_TOPIC -> M1"
cd "$PROJECT_DIR/ros2_bridge"
source "$PROJECT_DIR/scripts/source_ydlidar.sh"
python3 cmd_vel_to_rosmaster.py \
  --mvp-tune-config "$MVP_TUNE_FILE" \
  --port "$CHASSIS_PORT" \
  --max-vx 0.08 \
  --max-wz 0.35 \
  --cmd-smooth-alpha "$CMD_SMOOTH_ALPHA" \
  --max-vx-delta "$MAX_VX_DELTA" \
  --max-wz-delta "$MAX_WZ_DELTA" \
  --control-rate-hz "$CONTROL_RATE_HZ" \
  --debug \
  > "$PROJECT_DIR/logs/lidar_test_chassis_bridge.log" 2>&1 &
sleep 2

echo "[3/4] check /scan topic..."
source "$PROJECT_DIR/scripts/source_ydlidar.sh"
ros2 topic info /scan || true

echo "[4/4] start keyboard teleop (w/s/a/d/x/q)"
echo "Use another terminal to monitor:"
echo "  ros2 topic hz /scan"
echo "  ros2 topic echo /scan --once"
cd "$PROJECT_DIR/ros2_bridge"
python3 keyboard_cmd_vel.py --cmd-topic "$CMD_TOPIC"
