#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$ROOT/.env.local" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/.env.local"
fi
export PYTHONPATH="$ROOT/src:${PYTHONPATH:-}"
IMAGE="${1:?usage: test_single_image.sh IMAGE INSTRUCTION [observe|track|search|verify]}"
INSTRUCTION="${2:?usage: test_single_image.sh IMAGE INSTRUCTION [observe|track|search|verify]}"
MODE="${3:-observe}"
exec python3 "$ROOT/src/apps/test_single_image.py" --config "$ROOT/configs/qwen3_vln_debug.yaml" --image "$IMAGE" --instruction "$INSTRUCTION" --mode "$MODE"
