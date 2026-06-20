#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
# MVP 全流程（相机 + 底盘 + 视觉伺服），YOLO 检测/后处理与 start_yolo_live_preview.sh 对齐：
#   - TARGET_WORDS / TARGET_CLASSES 默认 bottle
#   - SCORE_THRESHOLD / MIN_SCORE 默认 0.001
#   - MAX_AREA_RATIO / SYNC_MAX_DELTA_SEC 与 preview 一致
#   - ros2 topic pub 持续发布 /target_words（同 diag 链）
#   - run_mvp_task.py 启用 --multi-frame-voter
#   - 后台启动 yolo_live_browser_preview.py，浏览器实时看框选

PROJECT_DIR=~/rdk_x5_vln_robot
PROJECT_DIR="$(eval echo "$PROJECT_DIR")"
CAMERA_DEV=/dev/video0

# 命令行传入的 YOLO 阈值优先；否则 load_mvp_tune 后再回落到 preview 默认 0.001
_cli_min_score="${MIN_SCORE-__unset__}"
_cli_score_threshold="${SCORE_THRESHOLD-__unset__}"

source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

if [ "$_cli_min_score" = "__unset__" ]; then
  MIN_SCORE="0.001"
else
  MIN_SCORE="$_cli_min_score"
fi
if [ "$_cli_score_threshold" = "__unset__" ]; then
  SCORE_THRESHOLD="0.001"
else
  SCORE_THRESHOLD="$_cli_score_threshold"
fi

COMPRESSED_IMAGE_TOPIC=/image
RAW_IMAGE_TOPIC=/image_raw
YOLO_IMAGE_TOPIC=/image_raw
CMD_TOPIC=/cmd_vel
TARGET_WORDS_TOPIC=/target_words
DET_TOPIC=/hobot_yolo_world

INSTRUCTION="${INSTRUCTION:-find the bottle}"

# preview 默认（可在命令行前 export 覆盖）
TARGET_WORDS="${TARGET_WORDS:-bottle}"
TARGET_CLASSES="${TARGET_CLASSES:-bottle}"
MAX_AREA_RATIO="${MAX_AREA_RATIO:-0.24}"
SYNC_MAX_DELTA_SEC="${SYNC_MAX_DELTA_SEC:-0.5}"
VOTE_WINDOW_SIZE="${VOTE_WINDOW_SIZE:-10}"
VOTE_MIN_VOTES="${VOTE_MIN_VOTES:-3}"
VOTE_LOST_HOLD_FRAMES="${VOTE_LOST_HOLD_FRAMES:-3}"

# 浏览器实时预览（与 start_yolo_live_preview.sh 相同组件，不重复拉 YOLO 链）
ENABLE_LIVE_PREVIEW="${ENABLE_LIVE_PREVIEW:-1}"
WEB_PORT="${WEB_PORT:-8088}"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
RAW_MIN_SCORE="${RAW_MIN_SCORE:-0.0}"
SHOW_ALL_BOXES="${SHOW_ALL_BOXES:-1}"

LIVE_PREVIEW_PID=""

cleanup() {
  echo "[mvp_preview] cleanup..."
  if [ -n "${LIVE_PREVIEW_PID:-}" ]; then
    kill "$LIVE_PREVIEW_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

start_live_browser_preview() {
  if [ "$ENABLE_LIVE_PREVIEW" != "1" ]; then
    echo "[live_preview] ENABLE_LIVE_PREVIEW=0, skip browser preview."
    return
  fi

  echo "[4.5/7] start browser live preview (background, same topics as MVP)..."
  cd "$PROJECT_DIR"
  source /opt/tros/humble/setup.bash
  if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
    source "$PROJECT_DIR/source_stage10.sh"
  fi

  PREVIEW_ARGS=(
    --image-topic "$RAW_IMAGE_TOPIC"
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
    PREVIEW_ARGS+=(--show-all-boxes)
  fi

  python3 "$PROJECT_DIR/debug_tools/yolo_live_browser_preview.py" "${PREVIEW_ARGS[@]}" \
    > "$PROJECT_DIR/logs/yolo_mvp_preview_browser.log" 2>&1 &
  LIVE_PREVIEW_PID="$!"
  echo "[live_preview] pid=$LIVE_PREVIEW_PID log=logs/yolo_mvp_preview_browser.log"
  sleep 2

  BOARD_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
  if [ -n "$BOARD_IP" ]; then
    echo "[live_preview] open browser: http://$BOARD_IP:$WEB_PORT"
  else
    echo "[live_preview] open browser: http://<board_ip>:$WEB_PORT"
  fi
}

echo "============================================================"
echo " RDK X5 VLN Robot MVP - YOLO Preview Pipeline + Chassis"
echo " Tune file: $MVP_TUNE_FILE"
echo " (YOLO params aligned with start_yolo_live_preview.sh)"
echo "============================================================"
echo "PROJECT_DIR              = $PROJECT_DIR"
echo "CAMERA_DEV               = $CAMERA_DEV"
echo "CHASSIS_PORT             = $CHASSIS_PORT"
echo "COMPRESSED_IMAGE_TOPIC   = $COMPRESSED_IMAGE_TOPIC"
echo "RAW_IMAGE_TOPIC          = $RAW_IMAGE_TOPIC"
echo "YOLO_IMAGE_TOPIC         = $YOLO_IMAGE_TOPIC"
echo "CMD_TOPIC                = $CMD_TOPIC"
echo "TARGET_WORDS_TOPIC       = $TARGET_WORDS_TOPIC"
echo "DET_TOPIC                = $DET_TOPIC"
echo "INSTRUCTION              = $INSTRUCTION"
echo "TARGET_WORDS             = $TARGET_WORDS"
echo "TARGET_CLASSES           = $TARGET_CLASSES"
echo "SCORE_THRESHOLD (node)   = $SCORE_THRESHOLD"
echo "MIN_SCORE (MVP filter)   = $MIN_SCORE"
echo "MAX_AREA_RATIO           = $MAX_AREA_RATIO"
echo "SYNC_MAX_DELTA_SEC       = $SYNC_MAX_DELTA_SEC"
echo "multi_frame_voter        = window=$VOTE_WINDOW_SIZE min_votes=$VOTE_MIN_VOTES hold=$VOTE_LOST_HOLD_FRAMES"
echo "ENABLE_LIVE_PREVIEW      = $ENABLE_LIVE_PREVIEW"
echo "WEB_PORT                 = $WEB_PORT"
echo "SHOW_ALL_BOXES           = $SHOW_ALL_BOXES"
echo "max_vx                   = $MAX_VX"
echo "lost_frames_limit        = $LOST_FRAMES_LIMIT"
echo "TURN_THRESHOLD           = $TURN_THRESHOLD"
echo "FORWARD_THRESHOLD        = $FORWARD_THRESHOLD"
echo "============================================================"

cd "$PROJECT_DIR"
mkdir -p logs data/images/mvp_debug

echo "[0/7] stop old processes..."
bash scripts/system/stop_all_safe.sh || true
sleep 1

echo "[1/7] start camera: hobot_usb_cam -> $COMPRESSED_IMAGE_TOPIC"
cd "$PROJECT_DIR/perception"
source /opt/tros/humble/setup.bash
ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" usb_video_device:="$CAMERA_DEV" \
  > "$PROJECT_DIR/logs/yolo_mvp_preview_camera_compressed.log" 2>&1 &
sleep 3

echo "[2/7] start image bridge: $COMPRESSED_IMAGE_TOPIC -> $RAW_IMAGE_TOPIC"
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
python3 src/perception/compressed_to_raw_image.py \
  --in-topic "$COMPRESSED_IMAGE_TOPIC" \
  --out-topic "$RAW_IMAGE_TOPIC" \
  > "$PROJECT_DIR/logs/yolo_mvp_preview_image_raw_bridge.log" 2>&1 &
sleep 2

echo "[3/7] publish target words (preview/diag style) -> $TARGET_WORDS_TOPIC"
source /opt/tros/humble/setup.bash
ros2 topic pub -r 1 "$TARGET_WORDS_TOPIC" std_msgs/msg/String "{data: '$TARGET_WORDS'}" \
  > "$PROJECT_DIR/logs/yolo_mvp_preview_target_words.log" 2>&1 &
sleep 2

echo "[4/7] start YOLO-World (preview thresholds): $YOLO_IMAGE_TOPIC -> $DET_TOPIC"
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh"
fi

ros2 run hobot_yolo_world hobot_yolo_world \
  --ros-args \
  -p feed_type:=1 \
  -p ros_img_sub_topic_name:="$YOLO_IMAGE_TOPIC" \
  -p ros_string_sub_topic_name:="$TARGET_WORDS_TOPIC" \
  -p ai_msg_pub_topic_name:="$DET_TOPIC" \
  -p texts:="$TARGET_WORDS" \
  -p score_threshold:="$SCORE_THRESHOLD" \
  -p iou_threshold:=0.45 \
  --ros-args --log-level warn \
  > "$PROJECT_DIR/logs/yolo_mvp_preview_yolo_world.log" 2>&1 &
sleep 4

start_live_browser_preview

echo "[5/7] start chassis bridge: $CMD_TOPIC -> M1"
source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
run_chassis_bridge "$PROJECT_DIR/logs/yolo_mvp_preview_chassis_bridge.log"
sleep 2

echo "[5.5/7] check topics..."
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh"
fi

ros2 topic info "$COMPRESSED_IMAGE_TOPIC" || true
ros2 topic info "$RAW_IMAGE_TOPIC" || true
ros2 topic info "$TARGET_WORDS_TOPIC" || true
ros2 topic info "$DET_TOPIC" || true
ros2 topic info "$CMD_TOPIC" || true

echo "[6/7] start MVP task (preview YOLO post-process + multi-frame voter)"
cd "$PROJECT_DIR"
source /opt/tros/humble/setup.bash
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh"
fi

python3 src/apps/run_mvp_task.py \
  --mvp-tune-config "$MVP_TUNE_FILE" \
  --instruction "$INSTRUCTION" \
  --backend yolo_world \
  --image-topic "$RAW_IMAGE_TOPIC" \
  --det-topic "$DET_TOPIC" \
  --cmd-topic "$CMD_TOPIC" \
  --target-words-topic "$TARGET_WORDS_TOPIC" \
  --min-score "$MIN_SCORE" \
  --max-area-ratio "$MAX_AREA_RATIO" \
  --sync-max-delta-sec "$SYNC_MAX_DELTA_SEC" \
  --target-classes "$TARGET_CLASSES" \
  --no-publish-target-words \
  --multi-frame-voter \
  --vote-window-size "$VOTE_WINDOW_SIZE" \
  --vote-min-votes "$VOTE_MIN_VOTES" \
  --vote-lost-hold-frames "$VOTE_LOST_HOLD_FRAMES" \
  --no-red-verify \
  2>&1 | tee "$PROJECT_DIR/logs/yolo_mvp_preview_task.log"
