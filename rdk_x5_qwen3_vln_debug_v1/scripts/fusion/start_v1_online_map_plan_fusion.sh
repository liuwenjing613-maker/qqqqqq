#!/usr/bin/env bash
# Start current V1 first-person navigation plus online map/Qwen/A* fusion.
# The fusion switch defaults off.  Disabled means the original V1 start script
# is executed directly with no bridge, no manager, and no velocity mux inserted.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BASE_START="$ROOT/scripts/qwen_servo/start_live_servo.sh"
BASE_SERVO_CONFIG="${SERVO_CONFIG:-$ROOT/configs/qwen3_vln_servo.yaml}"
FUSION_CONFIG="${FUSION_CONFIG:-$ROOT/configs/online_map_plan_fusion.yaml}"
RUNTIME_CONFIG="${FUSION_RUNTIME_CONFIG:-/tmp/qwen3_vln_online_fusion_${USER:-robot}.yaml}"
TASK="${1:-${QWEN_TASK:-}}"

[[ -f "$BASE_START" ]] || { echo "[FUSION][FATAL] missing $BASE_START" >&2; exit 1; }
[[ -f "$BASE_SERVO_CONFIG" ]] || { echo "[FUSION][FATAL] missing $BASE_SERVO_CONFIG" >&2; exit 1; }
[[ -f "$FUSION_CONFIG" ]] || { echo "[FUSION][FATAL] missing $FUSION_CONFIG" >&2; exit 1; }

read_cfg() {
  python3 - "$FUSION_CONFIG" "$1" "$2" <<'PY'
import sys, yaml
p, section, key = sys.argv[1:]
raw = yaml.safe_load(open(p, encoding='utf-8')) or {}
root = raw.get('online_map_plan_fusion', raw) or {}
value = (root.get(section, {}) or {}).get(key, '') if section else root.get(key, '')
if isinstance(value, bool): print('1' if value else '0')
else: print(value)
PY
}

ENABLED="$(read_cfg '' enabled)"
if [[ "$ENABLED" != "1" ]]; then
  echo "[FUSION] disabled: starting the original V1 path unchanged"
  THIRD_VIEW_WRAPPER_ACTIVE=1 SERVO_CONFIG="$BASE_SERVO_CONFIG" \
    exec bash "$BASE_START" "$@"
fi

for required in \
  "$ROOT/src/intervention/core.py" \
  "$ROOT/src/intervention/mux_logic.py" \
  "$ROOT/src/apps/third_view_intervention_node.py" \
  "$ROOT/src/apps/cmd_vel_intervention_mux.py" \
  "$ROOT/scripts/intervention/start_live_servo_with_intervention.sh" \
  "$ROOT/src/apps/online_map_plan_bridge_node.py"; do
  [[ -f "$required" ]] || { echo "[FUSION][FATAL] missing prerequisite: $required" >&2; exit 1; }
done

python3 "$ROOT/scripts/fusion/make_online_fusion_runtime_config.py" \
  --servo-config "$BASE_SERVO_CONFIG" \
  --fusion-config "$FUSION_CONFIG" \
  --output "$RUNTIME_CONFIG" \
  --enable true >/dev/null

mkdir -p "$ROOT/logs"
BRIDGE_PID=""
STACK_PID=""
MAP_PID=""
BACKEND_PID=""
CLEANED=0

kill_tree() {
  local pid="${1:-}"
  [[ -z "$pid" ]] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  local child
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do kill_tree "$child"; done
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 15); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  [[ "$CLEANED" == "1" ]] && return 0
  CLEANED=1
  echo "[FUSION] cleanup"
  kill_tree "$STACK_PID"
  kill_tree "$BRIDGE_PID"
  kill_tree "$BACKEND_PID"
  kill_tree "$MAP_PID"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

START_MAP="$(read_cfg startup start_map_stack)"
MAP_CMD="$(read_cfg startup map_stack_command)"
START_BACKEND="$(read_cfg startup start_backend)"
BACKEND_CMD="$(read_cfg startup backend_command)"
WAIT_SEC="$(read_cfg startup startup_wait_sec)"
[[ -n "$WAIT_SEC" ]] || WAIT_SEC=1.0

if [[ "$START_MAP" == "1" ]]; then
  [[ -n "$MAP_CMD" ]] || { echo "[FUSION][FATAL] start_map_stack=true but map_stack_command empty" >&2; exit 1; }
  echo "[FUSION] starting live map stack: $MAP_CMD"
  setsid bash -lc "$MAP_CMD" >"$ROOT/logs/fusion_map_stack.log" 2>&1 &
  MAP_PID=$!
fi

if [[ "$START_BACKEND" == "1" ]]; then
  [[ -n "$BACKEND_CMD" ]] || { echo "[FUSION][FATAL] start_backend=true but backend_command empty" >&2; exit 1; }
  echo "[FUSION] starting teammate backend: $BACKEND_CMD"
  setsid bash -lc "$BACKEND_CMD" >"$ROOT/logs/fusion_backend.log" 2>&1 &
  BACKEND_PID=$!
fi

sleep "$WAIT_SEC"

python3 -u "$ROOT/src/apps/online_map_plan_bridge_node.py" \
  --config "$RUNTIME_CONFIG" --task "$TASK" \
  >"$ROOT/logs/online_map_plan_bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 0.8
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
  echo "[FUSION][FATAL] online bridge failed during startup" >&2
  tail -n 160 "$ROOT/logs/online_map_plan_bridge.log" || true
  exit 1
fi

echo "[FUSION] enabled"
echo "[FUSION] V1 cmd: /cmd_vel_ego -> intervention mux -> /cmd_vel_autonomy"
echo "[FUSION] map backend cmd: /map_qwen_plan/cmd_vel -> bridge -> /cmd_vel_map"
echo "[FUSION] task: ${TASK:-<backend default>}"
FLOW_LOG="${THIRD_VIEW_FLOW_LOG:-$ROOT/logs/third_view_flow.log}"
export THIRD_VIEW_FLOW_LOG="$FLOW_LOG"
echo "[FUSION] 第三视角流程日志: $FLOW_LOG"
echo "[FUSION] 实时查看: tail -f $FLOW_LOG"

SERVO_CONFIG="$RUNTIME_CONFIG" \
  bash "$ROOT/scripts/intervention/start_live_servo_with_intervention.sh" "$@" &
STACK_PID=$!

while true; do
  if ! kill -0 "$STACK_PID" 2>/dev/null; then
    set +e; wait "$STACK_PID"; status=$?; set -e
    STACK_PID=""
    exit "$status"
  fi
  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "[FUSION][FATAL] online map-plan bridge exited unexpectedly" >&2
    tail -n 160 "$ROOT/logs/online_map_plan_bridge.log" || true
    exit 1
  fi
  if [[ -n "$BACKEND_PID" ]] && ! kill -0 "$BACKEND_PID" 2>/dev/null; then
    echo "[FUSION][FATAL] configured backend process exited" >&2
    tail -n 160 "$ROOT/logs/fusion_backend.log" || true
    exit 1
  fi
  if [[ -n "$MAP_PID" ]] && ! kill -0 "$MAP_PID" 2>/dev/null; then
    echo "[FUSION][FATAL] configured live map process exited" >&2
    tail -n 160 "$ROOT/logs/fusion_map_stack.log" || true
    exit 1
  fi
  sleep 0.5
done
