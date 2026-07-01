#!/usr/bin/env bash
# Start visualization-only calibrated arrows after your normal SLAM stack is already running.
# It does not move the robot and does not change SLAM TF.
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
CONFIG_FILE="${ARROW_CONFIG_FILE:-${PROJECT_DIR}/configs/arrow_calibration.env}"
LOG_DIR="${PROJECT_DIR}/logs/arrow_calibration"
mkdir -p "${LOG_DIR}"
cd "${PROJECT_DIR}"

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

if [ ! -f "${CONFIG_FILE}" ]; then
  echo "[WARN] Missing ${CONFIG_FILE}; starting arrows with zero offset."
  echo "Run scripts/slam/run_arrow_calibrated_joy_mapping.sh once to auto-generate it."
fi

python3 "${PROJECT_DIR}/ros2_bridge/arrow_display_markers.py" \
  --config "${CONFIG_FILE}" \
  "$@" \
  2>&1 | tee "${LOG_DIR}/arrow_display_only.log"
