#!/usr/bin/env bash
# Optional wrapper around the proven 85739b0 start_live_servo.sh.
# enabled=false -> execute the original path unchanged.
# enabled=true  -> insert EGO/MAP/HOLD mux + intervention manager.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BASE_START="$ROOT/scripts/qwen_servo/start_live_servo.sh"
BASE_CONFIG="${SERVO_CONFIG:-$ROOT/configs/qwen3_vln_servo.yaml}"
RUNTIME_CONFIG="${INTERVENTION_RUNTIME_CONFIG:-/tmp/qwen3_vln_servo_intervention_${USER:-robot}.yaml}"
mkdir -p "$ROOT/logs"

[[ -f "$BASE_START" ]] || { echo "ERROR: missing $BASE_START" >&2; exit 1; }
[[ -f "$BASE_CONFIG" ]] || { echo "ERROR: missing $BASE_CONFIG" >&2; exit 1; }

ENABLED="$(python3 - "$BASE_CONFIG" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as handle:
    cfg = yaml.safe_load(handle) or {}
print("1" if bool((cfg.get("third_view_intervention") or {}).get("enabled", False)) else "0")
PY
)"

if [[ "$ENABLED" != "1" ]]; then
  echo "[intervention] disabled; preserve original 85739b0 startup/control path"
  THIRD_VIEW_WRAPPER_ACTIVE=1 exec bash "$BASE_START" "$@"
fi

python3 "$ROOT/scripts/qwen_servo/make_intervention_servo_config.py" \
  --input "$BASE_CONFIG" --output "$RUNTIME_CONFIG"

FLOW_LOG="${THIRD_VIEW_FLOW_LOG:-$ROOT/logs/third_view_flow.log}"
export THIRD_VIEW_FLOW_LOG="$FLOW_LOG"
mkdir -p "$(dirname "$FLOW_LOG")" "$ROOT/logs"
# Do not truncate here: bridge may already be appending to the same flow log.
touch "$FLOW_LOG"
echo "[intervention] 第三视角流程日志: $FLOW_LOG"
echo "[intervention] 实时查看: tail -f $FLOW_LOG"

MUX_PID=""
MANAGER_PID=""
BASE_PID=""
CLEANUP_DONE=0

kill_tree() {
  local pid="${1:-}"
  [[ -z "$pid" ]] && return 0
  kill -0 "$pid" 2>/dev/null || return 0
  local child
  for child in $(pgrep -P "$pid" 2>/dev/null || true); do
    kill_tree "$child"
  done
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 12); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  [[ "$CLEANUP_DONE" == "1" ]] && return 0
  CLEANUP_DONE=1
  echo "[intervention] cleanup"
  kill_tree "$BASE_PID"
  kill_tree "$MANAGER_PID"
  kill_tree "$MUX_PID"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

python3 -u "$ROOT/src/apps/cmd_vel_intervention_mux.py" \
  --config "$RUNTIME_CONFIG" \
  >"$ROOT/logs/cmd_vel_intervention_mux.log" 2>&1 &
MUX_PID=$!

python3 -u "$ROOT/src/apps/third_view_intervention_node.py" \
  --config "$RUNTIME_CONFIG" \
  --flow-log "$FLOW_LOG" \
  >"$ROOT/logs/third_view_intervention.log" 2>&1 &
MANAGER_PID=$!

sleep 0.8
if ! kill -0 "$MUX_PID" 2>/dev/null; then
  echo "ERROR: intervention cmd mux exited during startup" >&2
  tail -n 120 "$ROOT/logs/cmd_vel_intervention_mux.log" || true
  exit 1
fi
if ! kill -0 "$MANAGER_PID" 2>/dev/null; then
  echo "ERROR: intervention manager exited during startup" >&2
  tail -n 120 "$ROOT/logs/third_view_intervention.log" || true
  exit 1
fi

echo "[intervention] enabled"
echo "[intervention] ego cmd : /cmd_vel_ego"
echo "[intervention] map cmd : /cmd_vel_map"
echo "[intervention] mux out : /cmd_vel_autonomy"
echo "[intervention] request : /third_view/intervention/request"
echo "[intervention] flow log: $FLOW_LOG"

THIRD_VIEW_WRAPPER_ACTIVE=1 \
SERVO_CONFIG="$RUNTIME_CONFIG" \
  bash "$BASE_START" "$@" &
BASE_PID=$!

# Monitor every process. If either safety-critical supervisor process dies, stop
# the original stack instead of silently falling back to an undefined command path.
while true; do
  if ! kill -0 "$BASE_PID" 2>/dev/null; then
    set +e
    wait "$BASE_PID"
    status=$?
    set -e
    BASE_PID=""
    exit "$status"
  fi
  if ! kill -0 "$MANAGER_PID" 2>/dev/null; then
    echo "ERROR: intervention manager exited unexpectedly" >&2
    tail -n 120 "$ROOT/logs/third_view_intervention.log" || true
    exit 1
  fi
  if ! kill -0 "$MUX_PID" 2>/dev/null; then
    echo "ERROR: intervention cmd mux exited unexpectedly" >&2
    tail -n 120 "$ROOT/logs/cmd_vel_intervention_mux.log" || true
    exit 1
  fi
  sleep 0.5
done
