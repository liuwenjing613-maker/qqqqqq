#!/usr/bin/env bash
# Foxglove WebSocket bridge for RDK X5.
set -euo pipefail

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

PORT="${FOXGLOVE_PORT:-8765}"
WHITELIST="${FOXGLOVE_TOPIC_WHITELIST:-['.*']}"

echo "[foxglove] starting bridge on port ${PORT} whitelist=${WHITELIST}"

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:="${PORT}" \
  topic_whitelist:="${WHITELIST}" \
  send_buffer_limit:=10000000 \
  max_qos_depth:=10
