#!/usr/bin/env bash
# Read-only ROS graph checks.  This script never publishes velocity.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT_ROOT="$(cd "$ROOT/.." && pwd)"

if [[ -f /opt/tros/humble/setup.bash ]]; then source /opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash
fi
[[ -f "$ROBOT_ROOT/install/setup.bash" ]] && source "$ROBOT_ROOT/install/setup.bash"

command -v ros2 >/dev/null || { echo "[FAIL] ros2 not found"; exit 1; }
TOPICS="$(ros2 topic list 2>/dev/null || true)"

required=(/map /odom /qwen_vln/servo/status)
for topic in "${required[@]}"; do
  if grep -qx "$topic" <<<"$TOPICS"; then echo "[PASS] $topic"
  else echo "[FAIL] missing $topic"; exit 1
  fi
done

if grep -qx /scan_filtered <<<"$TOPICS"; then echo "[PASS] /scan_filtered"
elif grep -qx /scan <<<"$TOPICS"; then echo "[WARN] /scan_filtered missing; /scan exists"
else echo "[FAIL] no scan topic"; exit 1
fi

optional=(
  /map_qwen_plan/candidate_summary
  /map_qwen_plan/status
  /map_qwen_plan/cmd_vel
  /third_view/intervention/status
  /map_qwen_plan/bridge_status
)
for topic in "${optional[@]}"; do
  if grep -qx "$topic" <<<"$TOPICS"; then echo "[PASS] $topic"
  else echo "[WARN] not visible yet: $topic"
  fi
done

echo "--- /cmd_vel_autonomy graph ---"
ros2 topic info -v /cmd_vel_autonomy 2>/dev/null || true
echo "--- /cmd_vel_map graph ---"
ros2 topic info -v /cmd_vel_map 2>/dev/null || true

echo "[NOTE] In fusion mode /cmd_vel_autonomy should have one automatic publisher: cmd_vel_intervention_mux."
echo "[NOTE] The teammate backend must publish /map_qwen_plan/cmd_vel, not /cmd_vel or /cmd_vel_autonomy."
