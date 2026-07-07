#!/usr/bin/env bash
# Stop Qwen API + LiDAR stack without starting anything.
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

CONFIG="${1:-$PROJECT_DIR/configs/qwen_api_lidar_nav.yaml}"
# shellcheck source=scripts/lib/load_chassis_from_config.sh
source "$PROJECT_DIR/scripts/lib/load_chassis_from_config.sh"
load_chassis_from_config "$CONFIG" 2>/dev/null || true

# shellcheck source=scripts/lib/stop_qwen_stack.sh
source "$PROJECT_DIR/scripts/lib/stop_qwen_stack.sh"
_QWEN_STACK_CLEANUP_DONE=0
stop_qwen_stack
