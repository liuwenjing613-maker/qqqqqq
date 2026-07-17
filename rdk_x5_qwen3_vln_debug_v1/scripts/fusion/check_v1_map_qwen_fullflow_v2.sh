#!/usr/bin/env bash
set -u
set +e
source /opt/tros/humble/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
printf '%-44s %s\n' ITEM STATUS
for topic in /map /odom /scan_filtered /qwen_vln/servo/status /third_view/candidate_summary /map_qwen_plan/backend_debug /cmd_vel_ego /cmd_vel_map /cmd_vel_autonomy /cmd_vel; do
  count="$(ros2 topic info "$topic" 2>/dev/null | awk -F': ' '/Publisher count:/ {print $2}' | tail -n1)"
  printf '%-44s publishers=%s\n' "$topic" "${count:-0}"
done
printf '%-44s ' /navigate_to_pose
ros2 action info /navigate_to_pose 2>/dev/null | grep 'Action servers:' || echo 'Action servers: 0'
echo
for topic in /map_qwen_plan/backend_debug /map_qwen_plan/bridge_status /third_view/intervention/status; do
  echo "--- $topic (one sample) ---"
  timeout 3 ros2 topic echo --once "$topic" 2>/dev/null || echo "no sample"
done
