#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="${1:-/root/rdk_x5_vln_robot}"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$PROJECT_DIR/scripts/slam" "$PROJECT_DIR/docs"
cp "$SRC_DIR/scripts/slam/nav2_oneclick_goal.sh" "$PROJECT_DIR/scripts/slam/nav2_oneclick_goal.sh"
cp "$SRC_DIR/scripts/slam/cmd_vel_burst.py" "$PROJECT_DIR/scripts/slam/cmd_vel_burst.py"
cp "$SRC_DIR/docs/NAV2_ONECLICK_README.md" "$PROJECT_DIR/docs/NAV2_ONECLICK_README.md"
chmod +x "$PROJECT_DIR/scripts/slam/nav2_oneclick_goal.sh" "$PROJECT_DIR/scripts/slam/cmd_vel_burst.py"
echo "[OK] installed into $PROJECT_DIR"
echo "Run: cd $PROJECT_DIR && bash scripts/slam/nav2_oneclick_goal.sh 0.30 0.00 0.00"
