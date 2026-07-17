#!/usr/bin/env bash
# Convenience wrapper: stop fullflow V2 and all related robot processes.
set -u
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$PROJECT_DIR/rdk_x5_qwen3_vln_debug_v1/scripts/fusion/stop_v1_map_qwen_fullflow_v2.sh" "$@"
