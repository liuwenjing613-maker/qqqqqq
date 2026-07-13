#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/lib/ros_env.sh"
mkdir -p "$ROOT/logs"
IN_TOPIC="${CAMERA_COMPRESSED_TOPIC:-/image}"
OUT_TOPIC="${IMAGE_RAW_TOPIC:-/image_raw}"
MAX_FPS="${IMAGE_RAW_MAX_FPS:-8}"
echo "[bridge] subscribe $IN_TOPIC (CompressedImage/BEST_EFFORT)"
echo "[bridge] publish   $OUT_TOPIC (Image bgr8/RELIABLE)"
exec python3 -u "$ROOT/src/perception/compressed_to_raw_image.py" \
  --in-topic "$IN_TOPIC" --out-topic "$OUT_TOPIC" --max-fps "$MAX_FPS"
