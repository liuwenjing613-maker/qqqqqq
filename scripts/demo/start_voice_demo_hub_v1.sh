#!/usr/bin/env bash
set -Eeuo pipefail
REPO_ROOT="${ROBOT_PROJECT_DIR:-/root/rdk_x5_vln_robot}"
CONFIG="${VOICE_DEMO_CONFIG:-$REPO_ROOT/configs/voice_demo_hub_v1.yaml}"
RECORD_SECONDS="${VOICE_FUNCTION_RECORD_SECONDS:-5}"

usage() {
  cat <<EOF
Usage: bash scripts/demo/start_voice_demo_hub_v1.sh [--config PATH] [--record-seconds N]

一次启动后持续等待唤醒，固定命令切换：
  联网探索 / 断网探索 / 开始建图 / 完成建图 / 点击导航 / 停止
EOF
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) CONFIG="${2:?missing config}"; shift 2 ;;
    --record-seconds) RECORD_SECONDS="${2:?missing seconds}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "$REPO_ROOT"
exec python3 -u "$REPO_ROOT/scripts/demo/voice_demo_hub_v1.py" \
  --config "$CONFIG" --record-seconds "$RECORD_SECONDS"
