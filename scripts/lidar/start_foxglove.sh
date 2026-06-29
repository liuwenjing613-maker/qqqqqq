#!/usr/bin/env bash
# Foxglove WebSocket bridge for RDK X5.
set -euo pipefail

source /opt/tros/humble/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

PORT="${FOXGLOVE_PORT:-8765}"
WHITELIST="${FOXGLOVE_TOPIC_WHITELIST:-['.*']}"

echo "[foxglove] starting bridge on port ${PORT} whitelist=${WHITELIST}"

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:="${PORT}" \
  topic_whitelist:="${WHITELIST}" \
  send_buffer_limit:=10000000 \
  max_qos_depth:=10
