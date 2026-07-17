#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT/rdk_x5_qwen3_vln_debug_v1"
exec bash scripts/fusion/start_v1_map_qwen_simple_voice_v2.sh "$@"
