#!/usr/bin/env bash
# 独立启动：已有会话地图 + 位姿 → 快速/冷启动 Nav2（不调用 Qwen、不发目标）。
# 用于实车验证快速交接与 AMCL settle。
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

MAP_YAML="${1:-}"
POSE_JSON="${2:-}"
LOG_DIR="${LOG_DIR:-$PROJECT_DIR/logs/nav2_start_only_$(date +%Y%m%d_%H%M%S)}"
FAST_NAV="${FAST_NAV:-1}"

if [[ -z "$MAP_YAML" || -z "$POSE_JSON" ]]; then
  echo "Usage: $0 <session_map.yaml> <session_pose.json>"
  echo "  env: LOG_DIR, FAST_NAV=1|0, NAV2_REQUIRE_AMCL_SETTLED=1"
  exit 1
fi

MAP_YAML="$(readlink -f "$MAP_YAML")"
POSE_JSON="$(readlink -f "$POSE_JSON")"
mkdir -p "$LOG_DIR"

if [[ ! -f "$MAP_YAML" ]]; then
  echo "[FAIL] map not found: $MAP_YAML"
  exit 1
fi
if [[ ! -f "$POSE_JSON" ]]; then
  echo "[FAIL] pose not found: $POSE_JSON"
  exit 1
fi

PLACEHOLDER="$LOG_DIR/nav2_start_only_placeholder.json"
python3 - "$PLACEHOLDER" "$MAP_YAML" <<'PY'
import json, sys, time
from pathlib import Path
path, my = Path(sys.argv[1]), sys.argv[2]
path.write_text(json.dumps({
    "schema_version": "qwen_live_session_nav_goal_v1",
    "session_id": f"nav2_only_{int(time.time())}",
    "map_yaml": my,
    "selection_status": "PENDING",
    "note": "start_session_nav2_only placeholder",
}, indent=2) + "\n", encoding="utf-8")
PY

export POSE_STATE_FILE="$POSE_JSON"
export LOG_DIR
export NAV2_START_ONLY=1
export NAV2_REQUIRE_AMCL_SETTLED="${NAV2_REQUIRE_AMCL_SETTLED:-1}"
if [[ "$FAST_NAV" -eq 1 ]]; then
  export NAV2_STOP_CONFLICTS=0
  export NAV2_REUSE_EXISTING=1
  export NAV2_SKIP_DAEMON_REFRESH=1
else
  export NAV2_STOP_CONFLICTS=1
  export NAV2_REUSE_EXISTING=0
fi

echo "[NAV2_ONLY] MAP_YAML=$MAP_YAML"
echo "[NAV2_ONLY] POSE=$POSE_JSON"
echo "[NAV2_ONLY] LOG_DIR=$LOG_DIR FAST_NAV=$FAST_NAV"
exec bash "$PROJECT_DIR/scripts/nav/run_qwen_session_nav2_goal.sh" "$MAP_YAML" "$PLACEHOLDER"
