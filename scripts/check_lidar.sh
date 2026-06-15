#!/usr/bin/env bash
set -e

PROJECT_DIR="${HOME}/rdk_x5_vln_robot"
LIDAR_DEV="${LIDAR_DEV:-/dev/ydlidar}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"

echo "============================================================"
echo " RDK X5 VLN Robot - Lidar Check (T-MINI PLUS)"
echo "============================================================"

cd "$PROJECT_DIR"
source "$PROJECT_DIR/scripts/source_ydlidar.sh"

echo "[1/5] Check lidar serial device..."
if [ -e "$LIDAR_DEV" ]; then
  ls -l "$LIDAR_DEV"
else
  echo "FAIL: $LIDAR_DEV not found"
  echo "Hint: sudo cp lidar/udev/99-ydlidar-tmini.rules /etc/udev/rules.d/"
  echo "      sudo udevadm control --reload-rules && sudo udevadm trigger"
  exit 1
fi

echo
echo "[2/5] Check ydlidar_ros2_driver package..."
if ros2 pkg prefix ydlidar_ros2_driver >/dev/null 2>&1; then
  echo "OK   ydlidar_ros2_driver: $(ros2 pkg prefix ydlidar_ros2_driver)"
else
  echo "FAIL: ydlidar_ros2_driver not found. Build ~/ydlidar_ws first."
  exit 1
fi

echo
echo "[3/5] Check if /scan is publishing..."
if timeout 3 ros2 topic info "$SCAN_TOPIC" >/dev/null 2>&1; then
  echo "OK   topic $SCAN_TOPIC exists"
  timeout 5 ros2 topic hz "$SCAN_TOPIC" 2>/dev/null || echo "WARN: no hz data yet (driver may not be running)"
else
  echo "WARN: $SCAN_TOPIC not found. Start driver first:"
  echo "  bash scripts/start_lidar_only.sh"
fi

echo
echo "[4/5] Sample one scan message (if available)..."
timeout 5 ros2 topic echo "$SCAN_TOPIC" --once 2>/dev/null | head -20 || echo "WARN: no scan message received"

echo
echo "[5/5] Done."
