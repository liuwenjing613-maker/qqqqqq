#!/usr/bin/env bash
set -u

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
MAP_YAML="${MAP_YAML:-$PROJECT_DIR/maps/joy_calibrated_corridor_map.yaml}"
POSE_STATE_FILE="${POSE_STATE_FILE:-$PROJECT_DIR/state/last_pose_map.json}"
fail=0

source_ros() {
  set +u
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  [ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
  set -u
}

ok() { echo "[OK] $*"; }
warn() { echo "[WARN] $*"; }
bad() { echo "[FAIL] $*"; fail=1; }

source_ros

echo "========== File check =========="
[ -f "$MAP_YAML" ] && ok "map yaml: $MAP_YAML" || bad "missing map yaml: $MAP_YAML"
[ -f "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh" ] && ok "run_nav2_saved_map.sh exists" || bad "missing run_nav2_saved_map.sh"
[ -f "$PROJECT_DIR/scripts/slam/foxglove_click_goal_bridge.py" ] && ok "foxglove_click_goal_bridge.py exists" || bad "missing foxglove_click_goal_bridge.py"
[ -f "$PROJECT_DIR/scripts/slam/run_nav2_foxglove_click_goal.sh" ] && ok "run_nav2_foxglove_click_goal.sh exists" || bad "missing run_nav2_foxglove_click_goal.sh"
[ -f "$PROJECT_DIR/scripts/slam/pose_memory_node.py" ] && ok "pose_memory_node.py exists" || bad "missing pose_memory_node.py"
[ -f "$POSE_STATE_FILE" ] && ok "pose state file: $POSE_STATE_FILE" || warn "pose state file not found yet: $POSE_STATE_FILE (expected before first mapping run)"

echo
echo "========== Syntax check =========="
bash -n "$PROJECT_DIR/scripts/slam/run_nav2_foxglove_click_goal.sh" && ok "bash syntax: run_nav2_foxglove_click_goal.sh" || bad "bash syntax error"
python3 -m py_compile "$PROJECT_DIR/scripts/slam/foxglove_click_goal_bridge.py" && ok "python syntax: foxglove_click_goal_bridge.py" || bad "python syntax error"
python3 -m py_compile "$PROJECT_DIR/scripts/slam/pose_memory_node.py" && ok "python syntax: pose_memory_node.py" || bad "python syntax error"

echo
echo "========== ROS package check =========="
for p in rclpy action_msgs nav2_msgs geometry_msgs nav_msgs visualization_msgs; do
  python3 - <<PY >/dev/null 2>&1
import importlib
importlib.import_module('$p')
PY
  if [ $? -eq 0 ]; then ok "python import: $p"; else bad "python import failed: $p"; fi
done

for p in nav2_bringup nav2_planner nav2_bt_navigator foxglove_bridge; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then ok "ROS2 package: $p"; else warn "missing or not sourced ROS2 package: $p"; fi
done

echo
echo "========== Running ROS graph check =========="
if ros2 node list >/dev/null 2>&1; then
  ros2 topic list | egrep '^/(map|odom|tf)$' || warn "Nav2 may not be running yet; this is OK before startup."
  ros2 action list | egrep '^/(navigate_to_pose|compute_path_to_pose)$' || warn "Nav2 actions not visible yet; run the click-navigation script first."
else
  warn "ROS graph not available yet. Start Nav2 before live topic/action checks."
fi

echo
echo "========== Result =========="
if [ "$fail" -eq 0 ]; then
  echo "[PASS] Basic file/syntax checks passed."
else
  echo "[FAIL] Fix the FAIL items above first."
fi
exit "$fail"
