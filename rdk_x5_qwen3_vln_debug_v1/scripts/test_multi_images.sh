#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$ROOT/.env.local" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/.env.local"
fi
export PYTHONPATH="$ROOT/src:${PYTHONPATH:-}"

usage() {
  cat <<'EOF'
usage:
  test_multi_images.sh INSTRUCTION MODE IMAGE [IMAGE ...]
  test_multi_images.sh INSTRUCTION MODE --dir IMAGE_DIR

examples:
  bash scripts/test_multi_images.sh "find the bottle" search \
    /root/rdk_x5_vln_robot/data/images/test1.png \
    /root/rdk_x5_vln_robot/data/images/test6.png

  bash scripts/test_multi_images.sh "find the bottle" search \
    --dir /root/rdk_x5_vln_robot/data/images
EOF
}

INSTRUCTION="${1:?$(usage)}"
MODE="${2:?$(usage)}"
shift 2

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

IMAGES=()
if [[ "$1" == "--dir" ]]; then
  DIR="${2:?usage: --dir IMAGE_DIR}"
  IMAGES+=("$DIR")
else
  IMAGES+=("$@")
fi

OUTPUT_DIR="${OUTPUT_DIR:-$ROOT/outputs/batch}"
exec python3 "$ROOT/src/apps/test_multi_images.py" \
  --config "$ROOT/configs/qwen3_vln_debug.yaml" \
  --instruction "$INSTRUCTION" \
  --mode "$MODE" \
  --output-dir "$OUTPUT_DIR" \
  --images "${IMAGES[@]}"
