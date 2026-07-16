#!/usr/bin/env bash
# Stop only qwen_session_foxglove_viz_node by pid file.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PID_FILE="$PROJECT_DIR/runtime/qwen_session/foxglove_viz.pid"

if [[ ! -f "$PID_FILE" ]]; then
  echo "[STOP][PASS] pid file missing"
  exit 0
fi

pid="$(cat "$PID_FILE" 2>/dev/null || true)"
if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
  rm -f "$PID_FILE"
  echo "[STOP][PASS] viz node not running"
  exit 0
fi

cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
if [[ "$cmdline" != *"qwen_session_foxglove_viz_node.py"* ]]; then
  echo "[STOP][FAIL] pid identity mismatch pid=$pid"
  exit 2
fi

kill -INT "$pid" 2>/dev/null || true
sleep 0.5
if kill -0 "$pid" 2>/dev/null; then
  kill -TERM "$pid" 2>/dev/null || true
fi
rm -f "$PID_FILE"
echo "[STOP][PASS] qwen_session_foxglove_viz stopped pid=$pid"
