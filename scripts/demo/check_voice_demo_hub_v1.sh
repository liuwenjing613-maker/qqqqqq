#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="${ROBOT_PROJECT_DIR:-/root/rdk_x5_vln_robot}"
cd "$ROOT"
python3 scripts/demo/patch_exp2_reuse_base_v1.py \
  scripts/nav/start_yolo_lidar_semantic_explore_nav_exp2.sh --check
bash -n scripts/demo/start_voice_demo_hub_v1.sh
bash -n scripts/demo/control_voice_demo_hub_v1.sh
bash -n scripts/demo/prewarm_qwen_ego_v1.sh
bash -n voice_interaction/scripts/run_voice_function_event_server_v1.sh
python3 -m py_compile \
  scripts/demo/voice_demo_hub_v1.py \
  scripts/demo/patch_exp2_reuse_base_v1.py \
  voice_interaction/src/kws_function_event_server_v1.py
python3 tests/test_voice_demo_router_v1.py
for f in \
  scripts/slam/run_slam_calibrated.sh \
  scripts/slam/run_joy_mapping_calibrated.sh \
  scripts/slam/run_nav2_foxglove_click_goal.sh \
  scripts/nav/start_yolo_lidar_semantic_explore_nav_exp2.sh \
  scripts/demo/prewarm_qwen_ego_v1.sh \
  rdk_x5_qwen3_vln_debug_v1/scripts/fusion/start_v1_map_qwen_simple_voice_v2.sh; do
  [[ -f "$f" ]] || { echo "[FAIL] missing $f" >&2; exit 1; }
done
echo "[OK] voice demo hub static checks passed"
