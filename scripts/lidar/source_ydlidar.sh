#!/usr/bin/env bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
# Load TROS + YDLidar driver workspace.
source /opt/tros/humble/setup.bash
if [ -f "${HOME}/ydlidar_ws/install/setup.bash" ]; then
  source "${HOME}/ydlidar_ws/install/setup.bash"
fi
