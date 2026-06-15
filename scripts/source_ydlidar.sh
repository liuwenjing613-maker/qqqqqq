#!/usr/bin/env bash
# Load TROS + YDLidar driver workspace.
source /opt/tros/humble/setup.bash
if [ -f "${HOME}/ydlidar_ws/install/setup.bash" ]; then
  source "${HOME}/ydlidar_ws/install/setup.bash"
fi
