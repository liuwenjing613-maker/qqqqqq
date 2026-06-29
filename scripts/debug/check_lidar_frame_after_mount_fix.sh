#!/usr/bin/env bash
# Verify LiDAR frame / TF / scan topics after physical forward mount fix.

set -u

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

source "${PROJECT_DIR}/scripts/lib/lidar_frame_config.sh"

PASS=0
FAIL=0
WARN=0

check_pass() {
  echo "[PASS] $*"
  PASS=$((PASS + 1))
}

check_fail() {
  echo "[FAIL] $*"
  FAIL=$((FAIL + 1))
}

check_warn() {
  echo "[WARN] $*"
  WARN=$((WARN + 1))
}

echo "===== LiDAR frame check (post mount fix) ====="

if ros2 topic list 2>/dev/null | grep -qx "/scan"; then
  scan_frame="$(timeout 5 ros2 topic echo /scan --once 2>/dev/null | awk '/frame_id:/{print $2; exit}')"
  echo "[CHECK] /scan frame_id: ${scan_frame:-<none>}"
  if [ "${scan_frame:-}" = "${LASER_FRAME}" ]; then
    check_pass "/scan frame_id is ${LASER_FRAME}"
  else
    check_fail "/scan frame_id='${scan_frame:-}' expected '${LASER_FRAME}'"
  fi
else
  check_fail "/scan topic not found"
fi

if ros2 topic list 2>/dev/null | grep -qx "/scan_filtered"; then
  filt_frame="$(timeout 5 ros2 topic echo /scan_filtered --once 2>/dev/null | awk '/frame_id:/{print $2; exit}')"
  echo "[CHECK] /scan_filtered frame_id: ${filt_frame:-<none>}"
  if [ "${filt_frame:-}" = "${LASER_FRAME}" ]; then
    check_pass "/scan_filtered frame_id is ${LASER_FRAME}"
  else
    check_fail "/scan_filtered frame_id='${filt_frame:-}' expected '${LASER_FRAME}'"
  fi
else
  check_warn "/scan_filtered topic not found (start scan filter for SLAM)"
fi

echo "[CHECK] tf2_echo base_link ${LASER_FRAME}"
tf_bl="$(timeout 6 ros2 run tf2_ros tf2_echo base_link "${LASER_FRAME}" 2>&1 | tail -20 || true)"
if echo "$tf_bl" | grep -q "Translation"; then
  echo "$tf_bl" | grep -E "Translation|Rotation: in RPY \(degree\)" | head -4
  check_pass "TF base_link -> ${LASER_FRAME} available"
else
  check_fail "TF base_link -> ${LASER_FRAME} not available"
fi

echo "[CHECK] tf2_echo odom base_link"
tf_ob="$(timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | tail -20 || true)"
if echo "$tf_ob" | grep -q "Translation"; then
  echo "$tf_ob" | grep -E "Translation|Rotation: in RPY \(degree\)" | head -4
  check_pass "TF odom -> base_link available"
else
  check_fail "TF odom -> base_link not available"
fi

echo "[CHECK] topic info /tf_static -v"
tf_static_info="$(ros2 topic info /tf_static -v 2>/dev/null || true)"
echo "$tf_static_info" | head -40
pub_count="$(echo "$tf_static_info" | awk '/Publisher count:/{print $3; exit}')"
if [ "${pub_count:-0}" -le 1 ]; then
  check_pass "/tf_static publisher count=${pub_count:-0} (no duplicate base_link->laser expected)"
else
  check_warn "/tf_static publisher count=${pub_count:-0} (check for duplicate static TF)"
fi

echo "[CHECK] topic info /map -v"
map_info="$(ros2 topic info /map -v 2>/dev/null || true)"
if [ -n "$map_info" ]; then
  echo "$map_info" | head -40
  map_pub="$(echo "$map_info" | awk '/Publisher count:/{print $3; exit}')"
  if [ "${map_pub:-0}" -le 1 ]; then
    check_pass "/map publisher count=${map_pub:-0}"
  else
    check_warn "/map publisher count=${map_pub:-0} (multiple SLAM nodes?)"
  fi
else
  check_warn "/map topic not present (OK if SLAM not running)"
fi

echo
echo "===== Manual tests (required) ====="
echo "A) Box-in-front: robot still, box 0.5m ahead, Foxglove Fixed Frame=base_link, /scan_filtered -> box on +x"
echo "B) Odom yaw: left CCW turn -> yaw increases; right CW -> yaw decreases; stationary -> stable"
echo "C) SLAM: fresh stack, slow 0.5m straight + small turn, walls should not stack misaligned"
echo
echo "===== Summary ====="
echo "PASS=${PASS} FAIL=${FAIL} WARN=${WARN}"
if [ "$FAIL" -gt 0 ]; then
  exit 1
fi
exit 0
