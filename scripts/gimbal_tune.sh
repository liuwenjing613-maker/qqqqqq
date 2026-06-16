#!/usr/bin/env bash
set -e

PROJECT_DIR=~/rdk_x5_vln_robot
source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

python /root/rdk_x5_vln_robot/gimbal/gimbal_tune.py \
    --mvp-tune-config "$MVP_TUNE_FILE" \
    --yaw-id 2 \
    --yaw 65 \
    --pitch-id 3 \
    --pitch 30 \
    --port "$CHASSIS_PORT"
