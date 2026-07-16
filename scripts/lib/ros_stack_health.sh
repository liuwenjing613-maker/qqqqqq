#!/usr/bin/env bash
# Shared SLAM / Foxglove / Nav2 readiness checks (TF + topic streaming).

wait_tf_frames_python() {
  local parent_frame="$1"
  local child_frame="$2"
  local timeout_sec="${3:-60}"
  python3 - "$parent_frame" "$child_frame" "$timeout_sec" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

parent, child, timeout = sys.argv[1], sys.argv[2], float(sys.argv[3])
rclpy.init()
node = Node("ros_stack_health_tf_wait")
buf = Buffer(cache_time=Duration(seconds=30.0))
TransformListener(buf, node, spin_thread=False)
start = time.time()
last_report = start
while time.time() - start < timeout:
    rclpy.spin_once(node, timeout_sec=0.1)
    now = time.time()
    if now - last_report >= 10.0:
        print(
            f"[TF_WAIT] ... 仍在等待 {parent} -> {child} ({int(now - start)}s)",
            flush=True,
        )
        last_report = now
    try:
        tf = buf.lookup_transform(
            parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.3)
        )
        t = tf.transform.translation
        print(
            f"[TF_WAIT] OK {parent} -> {child} x={t.x:.3f} y={t.y:.3f}",
            flush=True,
        )
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    except Exception:
        pass
node.destroy_node()
rclpy.shutdown()
print(f"[TF_WAIT] FAIL {parent} -> {child} not available after {timeout:.0f}s")
raise SystemExit(1)
PY
}

topic_is_streaming() {
  local topic="$1"
  # LiDAR 多为 BEST_EFFORT；默认 RELIABLE 的 hz/echo 会永远收不到数据
  if timeout 2 ros2 topic echo "$topic" --once \
    --qos-reliability best_effort --qos-durability volatile \
    >/dev/null 2>&1; then
    return 0
  fi
  local out
  out="$(timeout 3 ros2 topic hz "$topic" --window 3 \
    --qos-reliability best_effort 2>/dev/null || true)"
  if echo "$out" | grep -qE "average rate|rate: [1-9]"; then
    return 0
  fi
  return 1
}

wait_topic_streaming() {
  local topic="$1"
  local timeout_sec="${2:-15}"
  local label="${3:-等待 topic 数据流}"
  local start now elapsed
  start="$(date +%s)"
  while true; do
    if topic_is_streaming "$topic"; then
      now="$(date +%s)"
      elapsed=$((now - start))
      echo "[STREAM] OK ${topic} (${elapsed}s)"
      return 0
    fi
    now="$(date +%s)"
    elapsed=$((now - start))
    if (( elapsed >= timeout_sec )); then
      echo "[STREAM] FAIL ${topic} 无数据 (${timeout_sec}s)"
      return 1
    fi
    if (( elapsed > 0 && elapsed % 5 == 0 )); then
      echo "[STREAM] ... ${label} ${topic} (${elapsed}s)"
    fi
    sleep 1
  done
}

wait_slam_foxglove_ready() {
  local timeout_sec="${1:-30}"
  local label="${2:-SLAM/Foxglove 可视化就绪}"
  local start now elapsed
  start="$(date +%s)"
  echo "[HEALTH] ${label} (最多 ${timeout_sec}s，非阻塞预检) ..."

  if ! wait_topic_streaming /scan_filtered 8 "雷达"; then
    echo "[HEALTH] WARN: /scan_filtered 暂无数据，Foxglove 可能无激光点"
  fi

  if ! wait_tf_frames_python odom base_link 10; then
    echo "[HEALTH] WARN: 暂无 odom -> base_link TF（请轻推摇杆）"
  fi

  while true; do
    if wait_tf_frames_python map base_link 3; then
      echo "[HEALTH] SLAM TF map -> base_link 就绪"
      return 0
    fi
    now="$(date +%s)"
    elapsed=$((now - start))
    if (( elapsed >= timeout_sec )); then
      echo "[HEALTH] WARN: ${timeout_sec}s 内无 map -> base_link TF，继续启动（请轻推摇杆）"
      echo "[HEALTH] HINT: 用手柄缓慢移动 1–2m 让 SLAM 发布 map 坐标系"
      return 0
    fi
    if (( elapsed > 0 && elapsed % 10 == 0 )); then
      echo "[HEALTH] ... 等待 map TF，请轻推摇杆 (${elapsed}s)"
    fi
    sleep 2
  done
}
