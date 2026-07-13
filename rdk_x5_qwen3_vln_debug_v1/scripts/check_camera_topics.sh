#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/lib/ros_env.sh"
for topic in "${CAMERA_COMPRESSED_TOPIC:-/image}" "${IMAGE_RAW_TOPIC:-/image_raw}" \
             "/qwen_vln/annotated_image/compressed"; do
  echo "===== $topic ====="
  ros2 topic type "$topic" 2>/dev/null || true
  ros2 topic info -v "$topic" 2>/dev/null || true
  echo
done
echo "NOTE: /image usually uses BEST_EFFORT QoS; plain 'ros2 topic hz /image' can be misleading."
echo "Use: ros2 topic echo --qos-reliability best_effort /image --once"
