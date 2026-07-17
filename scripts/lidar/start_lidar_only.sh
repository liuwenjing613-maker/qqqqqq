#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

PARAMS_FILE="${PROJECT_DIR}/lidar/config/tmini_plus.yaml"
LOG_FILE="${PROJECT_DIR}/logs/lidar_driver.log"
DRIVER_PID_FILE="${PROJECT_DIR}/runtime/ydlidar_driver.pid"
FOREGROUND=0

usage() {
  echo "usage: $0 [--foreground]" >&2
}

for arg in "$@"; do
  case "$arg" in
    --foreground) FOREGROUND=1 ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $arg" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ ! -f "$PARAMS_FILE" ]]; then
  echo "ERROR: params file not found: $PARAMS_FILE" >&2
  exit 1
fi

mkdir -p "${PROJECT_DIR}/logs" "${PROJECT_DIR}/runtime"
cd "$PROJECT_DIR"
source "$PROJECT_DIR/scripts/lidar/source_ydlidar.sh"
[ -f "$PROJECT_DIR/scripts/lib/ros_dds_env.sh" ] && source "$PROJECT_DIR/scripts/lib/ros_dds_env.sh"

PACKAGE_PREFIX="$(ros2 pkg prefix ydlidar_ros2_driver)"
DRIVER_EXE="${PACKAGE_PREFIX}/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node"

if [[ ! -x "$DRIVER_EXE" ]]; then
  echo "ERROR: driver executable not found or not executable: $DRIVER_EXE" >&2
  exit 1
fi

echo "============================================================"
echo " Start YDLidar T-MINI PLUS driver -> /scan"
echo "============================================================"
echo "EXECUTABLE  = $DRIVER_EXE"
echo "PARAMS_FILE = $PARAMS_FILE"
echo "LOG_FILE    = $LOG_FILE"
echo "FOREGROUND  = $FOREGROUND"
echo "============================================================"

pkill -f '/ydlidar_ros2_driver/ydlidar_ros2_driver_node' 2>/dev/null || true
pkill -f 'ydlidar_ros2_driver_node' 2>/dev/null || true
sleep 0.5
rm -f "$DRIVER_PID_FILE"

if [[ "$FOREGROUND" -eq 1 ]]; then
  echo "[LIDAR] foreground exec driver pid=$$"
  echo "$$" > "$DRIVER_PID_FILE"
  exec "$DRIVER_EXE" \
    --ros-args \
    --params-file "$PARAMS_FILE" \
    >> "$LOG_FILE" 2>&1
fi

"$DRIVER_EXE" \
  --ros-args \
  --params-file "$PARAMS_FILE" \
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
