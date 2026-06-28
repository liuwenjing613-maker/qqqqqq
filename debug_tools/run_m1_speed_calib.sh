#!/usr/bin/env bash
set -e

cd /root/rdk_x5_vln_robot/debug_tools

PORT="${1:-/dev/myserial}"

echo "===== M1 speed calibration ====="
echo "PORT=$PORT"

source /opt/tros/humble/setup.bash 2>/dev/null || true
source /opt/ros/humble/setup.bash 2>/dev/null || true

python3 emergency_stop.py --port "$PORT" 2>/dev/null || true

python3 m1_speed_calibration.py \
  --port "$PORT" \
  --mode all \
  --directions pos \
  --ros \
  --duration 2.0 \
  --rate 25 \
  --rest-sec 0.8 \
  --kick-vx 0.055 \
  --kick-wz 0.240 \
  --kick-duration 0.055 \
  --yes-i-have-space
