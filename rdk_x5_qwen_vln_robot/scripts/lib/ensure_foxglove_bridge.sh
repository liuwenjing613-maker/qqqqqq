#!/usr/bin/env bash
# Start foxglove_bridge WebSocket if not already listening (shared by nav + viz scripts).

ensure_foxglove_bridge() {
  local port="${FOXGLOVE_PORT:-8765}"
  local log_file="${1:-}"

  if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo "[foxglove] ERROR: foxglove_bridge not installed"
    echo "  Install ROS package foxglove_bridge or use Foxglove Studio native ROS connection."
    return 1
  fi

  if command -v ss >/dev/null 2>&1 && ss -tln 2>/dev/null | grep -q ":${port} "; then
    echo "[foxglove] OK: already listening on ws port ${port}"
    return 0
  fi

  echo "[foxglove] starting bridge on port ${port}..."
  if [ -n "$log_file" ]; then
    bash "${RDK_ORIGINAL_ROOT}/scripts/lidar/start_foxglove.sh" >"$log_file" 2>&1 &
  else
    bash "${RDK_ORIGINAL_ROOT}/scripts/lidar/start_foxglove.sh" &
  fi

  local waited=0
  local max_wait="${FOXGLOVE_BRIDGE_WAIT_SEC:-30}"
  while [ "$waited" -lt "$max_wait" ]; do
    if ss -tln 2>/dev/null | grep -q ":${port} "; then
      echo "[foxglove] OK: listening on ${port} (${waited}s)"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done

  echo "[foxglove] ERROR: bridge failed after ${max_wait}s"
  if [ -n "$log_file" ] && [ -f "$log_file" ]; then
    tail -20 "$log_file" 2>/dev/null || true
  fi
  return 1
}

start_qwen_api_foxglove_viz_node() {
  local config_file="${1:-$PROJECT_DIR/configs/qwen_api_lidar_nav.yaml}"
  local log_file="${2:-$PROJECT_DIR/logs/qwen_api_lidar_foxglove_viz.log}"

  echo "[viz] starting qwen_api_lidar_foxglove_viz..."
  python3 "$PROJECT_DIR/src/apps/qwen_api_lidar_foxglove_viz.py" \
    --config "$config_file" >"$log_file" 2>&1 &
  local pid=$!
  sleep 0.5
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[viz] ERROR: viz node exited; see $log_file"
    tail -20 "$log_file" 2>/dev/null || true
    return 1
  fi
  echo "[viz] OK: qwen_api_lidar_foxglove_viz pid=$pid"
  FOXGLOVE_VIZ_PID=$pid
  return 0
}
