#!/usr/bin/env bash
# Reliable readiness checks for qwen3_vln_debug_node.
#
# `ros2 topic info` alone is unreliable on a busy Fast DDS graph (it can hang or
# keep reporting "Unknown topic" even after the node is running). Prefer the node
# log line emitted at the end of __init__, with node-list / topic-info fallbacks.

_qwen_ros2_topic_info() {
  timeout 4 ros2 topic info "$1" 2>/dev/null || true
}

_qwen_debug_log_ready() {
  local log_file="$1"
  [[ -f "$log_file" ]] && grep -q 'started model=' "$log_file"
}

_qwen_debug_node_visible() {
  timeout 4 ros2 node list 2>/dev/null | grep -q '/qwen3_vln_debug_node'
}

_qwen_debug_topic_ready() {
  local topic="$1"
  _qwen_ros2_topic_info "$topic" | grep -Eq 'Publisher count: [1-9][0-9]*'
}

# Args: pid log_file [max_attempts] [topic] [on_tick_fn]
wait_qwen_debug_ready() {
  local pid="$1"
  local log_file="$2"
  local max_attempts="${3:-60}"
  local topic="${4:-/qwen_vln/result_json}"
  local on_tick="${5:-}"

  local attempt reason=""
  for attempt in $(seq 1 "$max_attempts"); do
    if _qwen_debug_log_ready "$log_file"; then
      reason="log:started model="
      echo "[wait] qwen ready (${attempt}s, ${reason})"
      return 0
    fi
    if _qwen_debug_topic_ready "$topic"; then
      reason="topic:${topic}"
      echo "[wait] qwen ready (${attempt}s, ${reason})"
      return 0
    fi
    if _qwen_debug_node_visible && [[ -f "$log_file" ]] \
        && grep -qE 'Qwen warmup (completed|failed)' "$log_file"; then
      reason="node:warmup-done"
      echo "[wait] qwen ready (${attempt}s, ${reason})"
      return 0
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "[wait] qwen process exited during startup (${attempt}s)" >&2
      return 1
    fi
    if [[ -n "$on_tick" ]]; then
      "$on_tick" "$attempt" "$max_attempts"
    fi
    sleep 1
  done

  echo "[wait] qwen not ready after ${max_attempts}s" >&2
  return 1
}
