#!/usr/bin/env bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
# Load TROS + YDLidar driver workspace.
# ROS setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES，须在 set -u 下临时关闭
set +u
source /opt/tros/humble/setup.bash
if [ -f "${HOME}/ydlidar_ws/install/setup.bash" ]; then
  source "${HOME}/ydlidar_ws/install/setup.bash"
fi
set -u
