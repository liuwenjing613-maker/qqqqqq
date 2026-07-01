#!/usr/bin/env bash
# Install/check arrow calibration tools. Run from the repository root after unzipping.
set -euo pipefail
PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"

if [ "$(pwd)" != "${PROJECT_DIR}" ]; then
  echo "[INFO] Current dir: $(pwd)"
  echo "[INFO] Expected repo root: ${PROJECT_DIR}"
  echo "[INFO] This script is safe to run from repo root after copying files."
fi

chmod +x ros2_bridge/arrow_calibration_probe.py
chmod +x ros2_bridge/arrow_display_markers.py
chmod +x scripts/slam/run_arrow_calibrated_joy_mapping.sh
chmod +x scripts/slam/run_arrow_display_only.sh

python3 -m py_compile \
  ros2_bridge/arrow_calibration_probe.py \
  ros2_bridge/arrow_display_markers.py

mkdir -p configs logs/arrow_calibration maps

echo "[OK] Arrow calibration tools installed/check passed."
echo "Run: bash scripts/slam/run_arrow_calibrated_joy_mapping.sh"
echo "Or, after normal SLAM is already running: bash scripts/slam/run_arrow_display_only.sh"
