#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
PARAMS_FILE="${PROJECT_DIR}/lidar/config/tmini_plus.yaml"
LOG_FILE="${PROJECT_DIR}/logs/lidar_driver.log"
DRIVER_PID_FILE="${PROJECT_DIR}/runtime/ydlidar_driver.pid"
FOREGROUND=0

for arg in "$@"; do
  case "$arg" in
    --foreground) FOREGROUND=1 ;;
  esac
done

mkdir -p "${PROJECT_DIR}/logs" "${PROJECT_DIR}/runtime"
cd "$PROJECT_DIR"
source "$PROJECT_DIR/scripts/lidar/source_ydlidar.sh"
[ -f "$PROJECT_DIR/scripts/lib/ros_dds_env.sh" ] && source "$PROJECT_DIR/scripts/lib/ros_dds_env.sh"

echo "============================================================"
echo " Start YDLidar T-MINI PLUS driver -> /scan"
echo "============================================================"
echo "PARAMS_FILE = $PARAMS_FILE"
echo "LOG_FILE    = $LOG_FILE"
echo "FOREGROUND  = $FOREGROUND"
echo "============================================================"

pkill -f ydlidar_ros2_driver_node 2>/dev/null || true
sleep 0.5
rm -f "$DRIVER_PID_FILE"

if [[ "$FOREGROUND" -eq 1 ]]; then
  echo "[LIDAR] launcher pid=$$ (foreground exec)"
  exec ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
    --ros-args --params-file "$PARAMS_FILE" \
    >> "$LOG_FILE" 2>&1
fi

ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
  --ros-args --params-file "$PARAMS_FILE" \
  >> "$LOG_FILE" 2>&1 &
DRIVER_PID=$!
echo "$DRIVER_PID" > "$DRIVER_PID_FILE"

echo "[LIDAR] launcher pid=$$"
echo "[LIDAR] driver pid=$DRIVER_PID"

sleep 3
if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
  echo "ERROR: ydlidar_ros2_driver_node exited during startup (pid=$DRIVER_PID)" >&2
  tail -20 "$LOG_FILE" 2>/dev/null || true
  rm -f "$DRIVER_PID_FILE"
  exit 1
fi

echo "Driver started. Check:"
echo "  ros2 topic hz /scan"
echo "  tail -f $LOG_FILE"

wait "$DRIVER_PID"
rm -f "$DRIVER_PID_FILE"
