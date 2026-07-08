#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

python "$PROJECT_DIR/gimbal/gimbal_tune.py" \
    --mvp-tune-config "$MVP_TUNE_FILE" \
    --yaw-id 2 \
    --yaw 65 \
    --pitch-id 3 \
    --pitch 20 \
    --port /dev/ttyUSB1
