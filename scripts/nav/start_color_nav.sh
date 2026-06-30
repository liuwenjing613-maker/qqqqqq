#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

source_ros_env() {
  set +u
  if [ -f /opt/tros/humble/setup.bash ]; then
    source /opt/tros/humble/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  fi
  set -u
}

cfg_value() {
  local config="$1"
  local expr="$2"
  python3 - "$config" "$expr" <<'PY'
import sys, yaml
path, expr = sys.argv[1], sys.argv[2]
with open(path, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}
cur = cfg
for part in expr.split("."):
    cur = cur.get(part, {}) if isinstance(cur, dict) else {}
if isinstance(cur, bool):
    print("1" if cur else "0")
elif cur in ({}, None):
    print("")
else:
    print(cur)
PY
}

source_ros_env

CONFIG="${1:-configs/nav_color.yaml}"
USER_INSTRUCTION="${2:-}"
NAV_ONLY="${NAV_ONLY:-0}"
_CAMERA_OVERRIDE="${CAMERA_DEV:-}"
_CHASSIS_OVERRIDE="${CHASSIS_PORT:-}"

eval "$(python3 "$PROJECT_DIR/src/config/failsafe_nav_launch.py" --config "$CONFIG" --shell-export)"
if [ -n "$USER_INSTRUCTION" ]; then
  export INSTRUCTION="$USER_INSTRUCTION"
fi
if [ -n "$_CAMERA_OVERRIDE" ]; then
  export CAMERA_DEV="$_CAMERA_OVERRIDE"
fi
if [ -n "$_CHASSIS_OVERRIDE" ]; then
  export CHASSIS_PORT="$_CHASSIS_OVERRIDE"
fi

REQUIRE_LIDAR="$(cfg_value "$CONFIG" "safety.require_lidar")"
mkdir -p logs

echo "===== V1 color_nav ====="
echo "CONFIG=$CONFIG INSTRUCTION=$INSTRUCTION NAV_ONLY=$NAV_ONLY"
echo "CAMERA_DEV=$CAMERA_DEV CHASSIS_PORT=$CHASSIS_PORT REQUIRE_LIDAR=$REQUIRE_LIDAR"

pkill -f run_shared_nav.py || true
pkill -f yolo_world_to_bbox_json.py || true
pkill -f hobot_yolo_world || true
pkill -f compressed_to_raw_image.py || true
pkill -f m1_pwm_cmd_vel_bridge.py || true
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  >/dev/null 2>&1 || true

if [ "$NAV_ONLY" != "1" ]; then
  echo "[1/4] start camera + image bridge..."
  ros2 launch "$PROJECT_DIR/perception/launch/usb_cam.launch.py" \
    usb_video_device:="$CAMERA_DEV" \
    > logs/color_nav_camera.log 2>&1 &
  sleep 5

  python3 "$PROJECT_DIR/src/perception/compressed_to_raw_image.py" \
    --in-topic "$CAMERA_COMPRESSED_TOPIC" \
    --out-topic "$IMAGE_RAW_TOPIC" \
    --max-fps "$IMAGE_RAW_MAX_FPS" \
    > logs/color_nav_image_raw.log 2>&1 &
  sleep 2

  if [ "$REQUIRE_LIDAR" = "1" ]; then
    echo "[2/4] start lidar..."
    pkill -f ydlidar_ros2_driver_node 2>/dev/null || true
    source "$PROJECT_DIR/scripts/lidar/source_ydlidar.sh"
    ros2 launch "$PROJECT_DIR/lidar/launch/tmini_plus.launch.py" > logs/color_nav_scan.log 2>&1 &
    sleep 5
  else
    echo "[2/4] lidar disabled by config."
  fi

  echo "[3/4] start chassis bridge..."
  source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
  run_chassis_bridge "$PROJECT_DIR/logs/color_nav_chassis.log"
  sleep 1
fi

echo "[4/4] start shared nav..."
python3 "$PROJECT_DIR/src/apps/run_shared_nav.py" \
  --config "$CONFIG" \
  --instruction "$INSTRUCTION" \
  > "$PROJECT_DIR/logs/color_nav.log" 2>&1 &

echo "[color_nav] started pid=$!"
echo "  tail -f logs/color_nav.log"
echo "  ros2 topic echo /nav_state"
echo "  ros2 topic echo /cmd_vel"
