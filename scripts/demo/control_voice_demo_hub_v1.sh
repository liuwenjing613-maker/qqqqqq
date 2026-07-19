#!/usr/bin/env bash
# Keyboard fallback for the live demo. It injects the same wake+command events
# as the persistent voice server, so process switching follows one code path.
set -Eeuo pipefail

RUNTIME="${VOICE_DEMO_RUNTIME:-/tmp/rdk_x5_voice_demo_hub_v1_${USER:-robot}}"
EVENT_FILE="$RUNTIME/voice_events.jsonl"
ACTION="${1:-status}"

case "$ACTION" in
  online)  TEXT="开始联网探索" ;;
  offline) TEXT="开始断网探索" ;;
  mapping) TEXT="开始建图" ;;
  finish)  TEXT="完成建图" ;;
  click)   TEXT="点击导航" ;;
  stop)    TEXT="停止机器人" ;;
  reset)   TEXT="重置地图" ;;
  status)  TEXT="系统状态" ;;
  raw)
    shift
    TEXT="$*"
    [[ -n "$TEXT" ]] || { echo "raw requires command text" >&2; exit 2; }
    ;;
  -h|--help)
    echo "Usage: bash scripts/demo/control_voice_demo_hub_v1.sh {online|offline|mapping|finish|click|stop|reset|status|raw TEXT}"
    exit 0
    ;;
  *) echo "unknown action: $ACTION" >&2; exit 2 ;;
esac

[[ -e "$EVENT_FILE" ]] || {
  echo "[control][ERROR] hub event file not found: $EVENT_FILE" >&2
  echo "Start scripts/demo/start_voice_demo_hub_v1.sh first." >&2
  exit 1
}

python3 - "$EVENT_FILE" "$TEXT" <<'PY'
import fcntl
import json
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
text = sys.argv[2]
seq = time.time_ns() // 1000
now = time.time()
with path.open("a", encoding="utf-8", buffering=1) as f:
    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
    f.write(json.dumps({"seq": seq, "event": "wake", "time": now, "source": "keyboard"}, ensure_ascii=False) + "\n")
    f.write(json.dumps({"seq": seq, "event": "command", "time": time.time(), "text": text, "valid": True, "source": "keyboard"}, ensure_ascii=False) + "\n")
    f.flush()
    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
print(f"[control] injected: {text}")
PY
