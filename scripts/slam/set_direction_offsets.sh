#!/usr/bin/env bash
set -euo pipefail

cd /root/rdk_x5_vln_robot

BASE_OFFSET="${1:-}"
ODOM_XY_OFFSET="${2:-}"
LASER_OFFSET="${3:-__KEEP__}"

if [ -z "$BASE_OFFSET" ] || [ -z "$ODOM_XY_OFFSET" ]; then
  cat <<'EOF'
Usage:
  bash scripts/slam/set_direction_offsets.sh <base_yaw_offset> <odom_xy_yaw_offset> [laser_yaw]

Examples:
  # 只翻转 base_link 箭头，不改 odom 轨迹
  bash scripts/slam/set_direction_offsets.sh 3.141592653589793 0.0

  # 同时翻转 base_link 箭头和 /odom 轨迹方向
  bash scripts/slam/set_direction_offsets.sh 3.141592653589793 3.141592653589793

  # base_link 与 odom 翻转，同时 laser 相对 base_link 也翻转
  bash scripts/slam/set_direction_offsets.sh 3.141592653589793 3.141592653589793 3.141592653589793

  # 恢复默认
  bash scripts/slam/set_direction_offsets.sh 0.0 0.0
EOF
  exit 1
fi

ENV_FILE="scripts/lib/slam_calibrated_env.sh"
LIDAR_FILE="scripts/lib/lidar_frame_config.sh"

upsert_export() {
  local file="$1"
  local name="$2"
  local value="$3"

  if grep -qE "^export[[:space:]]+${name}=" "$file"; then
    python3 - "$file" "$name" "$value" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
name = sys.argv[2]
value = sys.argv[3]

s = path.read_text()
s = re.sub(
    rf"^export\s+{re.escape(name)}=.*$",
    f"export {name}={value}",
    s,
    flags=re.M,
)
path.write_text(s)
PY
  else
    printf "\nexport %s=%s\n" "$name" "$value" >> "$file"
  fi
}

upsert_export "$ENV_FILE" "CHASSIS_BASE_YAW_OFFSET" "$BASE_OFFSET"
upsert_export "$ENV_FILE" "CHASSIS_ODOM_XY_YAW_OFFSET" "$ODOM_XY_OFFSET"

if [ "$LASER_OFFSET" != "__KEEP__" ]; then
  upsert_export "$LIDAR_FILE" "LASER_YAW" "$LASER_OFFSET"
fi

echo "===== Direction offsets updated ====="
grep -n "CHASSIS_BASE_YAW_OFFSET\|CHASSIS_ODOM_XY_YAW_OFFSET" "$ENV_FILE" || true
grep -n "LASER_YAW" "$LIDAR_FILE" || true
