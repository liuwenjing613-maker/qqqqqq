#!/usr/bin/env bash
set -e

PROJECT_DIR="${HOME}/rdk_x5_vln_robot"
PARAMS_FILE="${PROJECT_DIR}/lidar/config/tmini_plus.yaml"
LOG_FILE="${PROJECT_DIR}/logs/lidar_driver.log"

mkdir -p "${PROJECT_DIR}/logs"
cd "$PROJECT_DIR"
source "$PROJECT_DIR/scripts/source_ydlidar.sh"

echo "============================================================"
echo " Start YDLidar T-MINI PLUS driver -> /scan"
echo "============================================================"
echo "PARAMS_FILE = $PARAMS_FILE"
echo "LOG_FILE    = $LOG_FILE"
echo "============================================================"

pkill -f ydlidar_ros2_driver_node 2>/dev/null || true
sleep 0.5

ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
  --ros-args --params-file "$PARAMS_FILE" \
  > "$LOG_FILE" 2>&1 &

sleep 3
echo "Driver started. Check:"
echo "  ros2 topic hz /scan"
echo "  tail -f $LOG_FILE"
