#!/usr/bin/env bash
# Convenience wrapper: stop fullflow V2 and all related robot processes.
set -u
# This file lives at <repo>/scripts/system/stop_all_robot.sh → repo is ../..
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STOP_SCRIPT="$PROJECT_DIR/rdk_x5_qwen3_vln_debug_v1/scripts/fusion/stop_v1_map_qwen_fullflow_v2.sh"
if [[ ! -f "$STOP_SCRIPT" ]]; then
  echo "ERROR: missing stop script: $STOP_SCRIPT" >&2
  exit 1
fi
exec bash "$STOP_SCRIPT" "$@"
