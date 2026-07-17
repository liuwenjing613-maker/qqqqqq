#!/usr/bin/env bash
set -euo pipefail

TARGET_ROOT="${1:-/root/rdk_x5_vln_robot}"
VOICE_ROOT="$TARGET_ROOT/voice_interaction"
NAV_ROOT="$TARGET_ROOT/rdk_x5_qwen3_vln_debug_v1"
FAILED=0

check_file() {
  local path="$1"
  if [[ -f "$path" ]]; then
    echo "[OK] $path"
  else
    echo "[MISSING] $path" >&2
    FAILED=1
  fi
}

check_file "$VOICE_ROOT/src/kws_instruction_once_voice.py"
check_file "$VOICE_ROOT/scripts/run_voice_instruction_once_voice.sh"
check_file "$NAV_ROOT/scripts/qwen_servo/start_live_servo_voice.sh"
check_file "$VOICE_ROOT/.env"
check_file "$VOICE_ROOT/assets/i_am_here.wav"
check_file "$NAV_ROOT/scripts/qwen_servo/start_live_servo.sh"
check_file "$NAV_ROOT/scripts/lib/qwen_ready.sh"
check_file "$NAV_ROOT/scripts/lib/nav_api_env.sh"
check_file "$NAV_ROOT/src/apps/qwen_visual_servo_node.py"
check_file "$TARGET_ROOT/scripts/control/cmd_vel_priority_mux.py"

if [[ -f "$VOICE_ROOT/scripts/run_voice_instruction_once_voice.sh" ]]; then
  bash -n "$VOICE_ROOT/scripts/run_voice_instruction_once_voice.sh"
  echo "[OK] voice runner bash syntax"
fi
if [[ -f "$NAV_ROOT/scripts/qwen_servo/start_live_servo_voice.sh" ]]; then
  bash -n "$NAV_ROOT/scripts/qwen_servo/start_live_servo_voice.sh"
  echo "[OK] voice navigation bash syntax"
fi
if [[ -f "$VOICE_ROOT/src/kws_instruction_once_voice.py" ]]; then
  python3 -m py_compile "$VOICE_ROOT/src/kws_instruction_once_voice.py"
  echo "[OK] one-shot Python syntax"
fi

for command in arecord aplay python3; do
  if command -v "$command" >/dev/null 2>&1; then
    echo "[OK] command: $command"
  else
    echo "[MISSING] command: $command" >&2
    FAILED=1
  fi
done

if [[ -f "$VOICE_ROOT/.env" ]]; then
  if grep -Eq '^DASHSCOPE_API_KEY=.+|^QWEN_API_KEY=.+' "$VOICE_ROOT/.env"; then
    echo "[OK] cloud key entry exists in voice .env"
  else
    echo "[WARN] voice .env 中未发现 DASHSCOPE_API_KEY/QWEN_API_KEY"
  fi
fi

if [[ -f "$TARGET_ROOT/.env" ]]; then
  if grep -Eq '^QWEN_MODEL=your_vision_model_name_here' "$TARGET_ROOT/.env"; then
    echo "[WARN] $TARGET_ROOT/.env 中 QWEN_MODEL 仍是占位符，导航阶段会回退到 qwen3-vl-flash"
  else
    echo "[OK] QWEN_MODEL configured in project .env"
  fi
fi

if [[ -f "$NAV_ROOT/scripts/lib/qwen_ready.sh" ]]; then
  bash -n "$NAV_ROOT/scripts/lib/qwen_ready.sh"
  echo "[OK] qwen_ready bash syntax"
fi
if [[ -f "$NAV_ROOT/scripts/lib/nav_api_env.sh" ]]; then
  bash -n "$NAV_ROOT/scripts/lib/nav_api_env.sh"
  echo "[OK] nav_api_env bash syntax"
fi

if [[ "$FAILED" == "1" ]]; then
  echo "[VERIFY] 存在缺失项，请先修复。" >&2
  exit 1
fi

echo "[VERIFY] 静态检查通过。硬件麦克风、云端 ASR 与真车运动仍需在 RDK X5 上实测。"
