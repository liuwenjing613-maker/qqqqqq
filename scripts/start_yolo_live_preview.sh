#!/usr/bin/env bash
set -euo pipefail

# YOLO-World live browser preview launcher.
# 默认会启动现有 scripts/start_yolo_diag_raw.sh，再启动浏览器预览。
# 不控制底盘，不发布 /cmd_vel。

PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$PROJECT_DIR"

if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
fi

TARGET_WORDS="${TARGET_WORDS:-bottle,water bottle,cup}"
TARGET_CLASSES="${TARGET_CLASSES:-bottle,cup}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
DET_TOPIC="${DET_TOPIC:-/hobot_yolo_world}"
WEB_PORT="${WEB_PORT:-8088}"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
MIN_SCORE="${MIN_SCORE:-0.002}"
RAW_MIN_SCORE="${RAW_MIN_SCORE:-0.0}"
MAX_AREA_RATIO="${MAX_AREA_RATIO:-0.24}"
SYNC_MAX_DELTA_SEC="${SYNC_MAX_DELTA_SEC:-0.5}"
SHOW_ALL_BOXES="${SHOW_ALL_BOXES:-1}"
RUN_YOLO_CHAIN="${RUN_YOLO_CHAIN:-1}"

mkdir -p logs

echo "===== YOLO live preview ====="
echo "PROJECT_DIR=$PROJECT_DIR"
echo "TARGET_WORDS=$TARGET_WORDS"
echo "TARGET_CLASSES=$TARGET_CLASSES"
echo "IMAGE_TOPIC=$IMAGE_TOPIC"
echo "DET_TOPIC=$DET_TOPIC"
echo "WEB_PORT=$WEB_PORT"

CHAIN_PID=""

cleanup() {
  echo "[live_preview] cleanup..."
  if [ -n "${CHAIN_PID:-}" ]; then
    kill "$CHAIN_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "$RUN_YOLO_CHAIN" = "1" ]; then
  echo "[live_preview] starting existing diagnostic YOLO chain..."
  TARGET_WORDS="$TARGET_WORDS" \
  TARGET_CLASSES="$TARGET_CLASSES" \
  SHOW_ALL_BOXES="$SHOW_ALL_BOXES" \
  RAW_MIN_SCORE="$RAW_MIN_SCORE" \
  SAVE_INTERVAL="${SAVE_INTERVAL:-30}" \
  bash "$PROJECT_DIR/scripts/start_yolo_diag_raw.sh" \
    > "$PROJECT_DIR/logs/live_preview_yolo_chain.log" 2>&1 &
  CHAIN_PID="$!"
  echo "[live_preview] yolo chain pid=$CHAIN_PID"
  echo "[live_preview] waiting 8 seconds for camera/YOLO startup..."
  sleep 8
else
  echo "[live_preview] RUN_YOLO_CHAIN=0, only starting browser preview node."
fi

BOARD_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [ -n "$BOARD_IP" ]; then
  echo "[live_preview] open browser: http://$BOARD_IP:$WEB_PORT"
else
  echo "[live_preview] open browser: http://<board_ip>:$WEB_PORT"
fi

ARGS=(
  --image-topic "$IMAGE_TOPIC"
  --det-topic "$DET_TOPIC"
  --target-classes "$TARGET_CLASSES"
  --min-score "$MIN_SCORE"
  --raw-min-score "$RAW_MIN_SCORE"
  --max-area-ratio "$MAX_AREA_RATIO"
  --sync-max-delta-sec "$SYNC_MAX_DELTA_SEC"
  --host "$WEB_HOST"
  --port "$WEB_PORT"
  --no-red-verify
)

if [ "$SHOW_ALL_BOXES" = "1" ]; then
  ARGS+=(--show-all-boxes)
fi

python3 "$PROJECT_DIR/debug_tools/yolo_live_browser_preview.py" "${ARGS[@]}"
