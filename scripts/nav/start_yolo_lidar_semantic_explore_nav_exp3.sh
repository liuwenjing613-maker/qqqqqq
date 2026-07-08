#!/usr/bin/env bash
set -euo pipefail

# EXP3 launcher: exp2 full stack + merged exp3 config + EXP3 ROS selector node.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${RDK_X5_VLN_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
EXP2_SCRIPT="$ROOT/scripts/nav/start_yolo_lidar_semantic_explore_nav_exp2.sh"
EXP2_CONFIG="$ROOT/configs/nav_yolo_lidar_semantic_explore_exp2.yaml"
EXP3_CONFIG_DEFAULT="$ROOT/configs/nav_yolo_lidar_semantic_explore_exp3.yaml"
EXP3_SELECTOR="$ROOT/src/planning/explore_goal_selector_exp3_ros.py"
MERGE_TOOL="$ROOT/scripts/lib/merge_nav_config.py"
MERGED_CONFIG="/tmp/nav_yolo_lidar_semantic_explore_exp3.merged.yaml"
TMP_SCRIPT="/tmp/start_yolo_lidar_semantic_explore_nav_exp3.runtime.sh"

if [[ ! -f "$EXP2_SCRIPT" ]]; then
  echo "[EXP3][ERROR] missing exp2 launcher: $EXP2_SCRIPT" >&2
  exit 3
fi
if [[ ! -f "$EXP3_CONFIG_DEFAULT" ]]; then
  echo "[EXP3][ERROR] missing exp3 config: $EXP3_CONFIG_DEFAULT" >&2
  exit 4
fi
if [[ ! -f "$EXP3_SELECTOR" ]]; then
  echo "[EXP3][ERROR] missing exp3 ROS selector: $EXP3_SELECTOR" >&2
  exit 5
fi
if [[ ! -f "$EXP2_CONFIG" ]]; then
  echo "[EXP3][ERROR] missing exp2 base config: $EXP2_CONFIG" >&2
  exit 6
fi
if [[ ! -f "$MERGE_TOOL" ]]; then
  echo "[EXP3][ERROR] missing merge tool: $MERGE_TOOL" >&2
  exit 7
fi

if [[ $# -eq 0 ]]; then
  CONFIG_ARG="$EXP3_CONFIG_DEFAULT"
  INSTRUCTION_ARG="find the bottle"
elif [[ $# -eq 1 ]]; then
  if [[ "$1" == *.yaml ]] || [[ -f "$1" ]]; then
    CONFIG_ARG="$1"
    INSTRUCTION_ARG="find the bottle"
  else
    CONFIG_ARG="$EXP3_CONFIG_DEFAULT"
    INSTRUCTION_ARG="$1"
  fi
else
  CONFIG_ARG="$1"
  INSTRUCTION_ARG="$2"
fi

export PYTHONPATH="$ROOT:$ROOT/src:${PYTHONPATH:-}"
export SEMANTIC_EXPLORE_PROFILE="exp3"
export EXPLORE_GOAL_SELECTOR="exp3_strict_v3"
export EXP3_SELECTOR_MODULE="explore_goal_selector_exp3_ros"
export EXP3_CONFIG="$CONFIG_ARG"
export EXP3_FORCE_GOAL_IN_ACTIVE_SECTOR="${EXP3_FORCE_GOAL_IN_ACTIVE_SECTOR:-1}"
export EXP3_GOAL_LATCH="${EXP3_GOAL_LATCH:-1}"
export EXP3_SCAN_CORRIDOR_CHECK="${EXP3_SCAN_CORRIDOR_CHECK:-1}"

python3 -m py_compile "$EXP3_SELECTOR" "$ROOT/src/planning/explore_goal_selector_exp3.py"
python3 "$MERGE_TOOL" "$EXP2_CONFIG" "$CONFIG_ARG" "$MERGED_CONFIG" >/dev/null

sed \
  -e 's/start_yolo_lidar_semantic_explore_nav_exp2/start_yolo_lidar_semantic_explore_nav_exp3/g' \
  -e 's/nav_yolo_lidar_semantic_explore_exp2\.yaml/nav_yolo_lidar_semantic_explore_exp3.yaml/g' \
  -e 's/nav_yolo_lidar_semantic_explore_exp2/nav_yolo_lidar_semantic_explore_exp3/g' \
  -e 's/explore_goal_selector_exp2/explore_goal_selector_exp3_ros/g' \
  -e 's/explore_goal_selector_exp3_ros_ros/explore_goal_selector_exp3_ros/g' \
  -e 's/semantic_explore_exp2/semantic_explore_exp3/g' \
  -e 's/run_shared_nav_semantic_explore_exp3/run_shared_nav_semantic_explore_exp2/g' \
  "$EXP2_SCRIPT" > "$TMP_SCRIPT"
# Generated script runs from /tmp; fix project_dir relative path.
sed -i "4s|^source .*|source \"$ROOT/scripts/lib/project_dir.sh\"|" "$TMP_SCRIPT"
chmod +x "$TMP_SCRIPT"

echo "===== semantic_explore_nav EXP3 STRICT v3 ====="
echo "ROOT=$ROOT"
echo "CONFIG=$MERGED_CONFIG"
echo "PATCH=$CONFIG_ARG"
echo "SELECTOR=$EXP3_SELECTOR"
echo "INSTRUCTION=$INSTRUCTION_ARG"
echo "=============================================="

exec bash "$TMP_SCRIPT" "$MERGED_CONFIG" "$INSTRUCTION_ARG"
