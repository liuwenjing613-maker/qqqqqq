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

if command -v ss >/dev/null 2>&1 && ss -tln 2>/dev/null | grep -q ":${PORT} "; then
  echo "[foxglove] ERROR: port ${PORT} already in use; cannot start bridge"
  echo "[foxglove] HINT: run ensure_foxglove_port_free or stop the other Foxglove stack first"
  ss -tlnp 2>/dev/null | grep ":${PORT} " || true
  exit 1
fi

echo "[foxglove] starting bridge on port ${PORT} whitelist=${WHITELIST}"

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:="${PORT}" \
  topic_whitelist:="${WHITELIST}" \
  send_buffer_limit:=10000000 \
  max_qos_depth:=10
