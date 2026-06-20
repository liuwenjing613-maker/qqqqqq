#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
cd $PROJECT_DIR

mkdir -p data/images

source /opt/tros/humble/setup.bash

echo "[SAVE] save one frame from /image_raw..."
python3 src/perception/save_raw_image_once.py \
  --image-topic /image_raw \
  --save-path data/images/latest_image_raw.jpg

ls -lh data/images/latest_image_raw.jpg
echo "[SAVE] done: data/images/latest_image_raw.jpg"
