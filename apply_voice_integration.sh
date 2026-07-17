#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_ROOT="${1:-/root/rdk_x5_vln_robot}"

[[ -d "$TARGET_ROOT" ]] || {
  echo "[INSTALL][ERROR] 目标工程目录不存在：$TARGET_ROOT" >&2
  exit 1
}

install -Dm755 \
  "$PACKAGE_DIR/voice_interaction/src/kws_instruction_once_voice.py" \
  "$TARGET_ROOT/voice_interaction/src/kws_instruction_once_voice.py"

install -Dm755 \
  "$PACKAGE_DIR/voice_interaction/scripts/run_voice_instruction_once_voice.sh" \
  "$TARGET_ROOT/voice_interaction/scripts/run_voice_instruction_once_voice.sh"

install -Dm755 \
  "$PACKAGE_DIR/rdk_x5_qwen3_vln_debug_v1/scripts/qwen_servo/start_live_servo_voice.sh" \
  "$TARGET_ROOT/rdk_x5_qwen3_vln_debug_v1/scripts/qwen_servo/start_live_servo_voice.sh"

echo "[INSTALL] 已添加 3 个 voice 后缀文件，未覆盖任何原文件。"
echo "[INSTALL] 运行检查：bash $PACKAGE_DIR/verify_voice_integration.sh $TARGET_ROOT"
echo "[INSTALL] 正式启动：bash $TARGET_ROOT/rdk_x5_qwen3_vln_debug_v1/scripts/qwen_servo/start_live_servo_voice.sh"
