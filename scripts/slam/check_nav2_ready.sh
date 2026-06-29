#!/usr/bin/env bash
set -u

# ROS setup.bash may read unset variables; disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
[ -f /opt/tros/humble/setup.bash ] && source /opt/tros/humble/setup.bash
set -u

PROJECT_DIR="${PROJECT_DIR:-/root/rdk_x5_vln_robot}"
MAP_YAML="${MAP_YAML:-$PROJECT_DIR/maps/joy_corridor_map.yaml}"
NAV2_PARAMS="${NAV2_PARAMS:-$PROJECT_DIR/configs/nav2_params.yaml}"

fail=0

ok() {
  echo "[OK] $*"
}

warn() {
  echo "[WARN] $*"
}

bad() {
  echo "[FAIL] $*"
  fail=1
}

check_file() {
  local f="$1"
  if [ -f "$f" ]; then
    ok "file exists: $f"
  else
    bad "file missing: $f"
  fi
}

check_exec() {
  local f="$1"
  if [ -x "$f" ]; then
    ok "executable: $f"
  elif [ -f "$f" ]; then
    warn "file exists but not executable: $f"
  else
    bad "missing executable: $f"
  fi
}

check_pkg() {
  local p="$1"
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then
    ok "ROS2 package: $p"
  else
    bad "missing ROS2 package: $p"
  fi
}

echo "========== Nav2 package check =========="
for p in \
  nav2_bringup \
  nav2_map_server \
  nav2_amcl \
  nav2_controller \
  nav2_planner \
  nav2_bt_navigator \
  nav2_lifecycle_manager \
  tf2_ros
do
  check_pkg "$p"
done

echo
echo "========== Project file check =========="
check_file "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh"
check_exec "$PROJECT_DIR/scripts/lidar/start_lidar_only.sh"
check_file "$PROJECT_DIR/ros2_bridge/m1_pwm_cmd_vel_bridge.py"
check_file "$PROJECT_DIR/ros2_bridge/cmd_vel_to_rosmaster.py"
check_file "$PROJECT_DIR/configs/mvp_tune.yaml"
check_file "$NAV2_PARAMS"
check_file "$MAP_YAML"

echo
echo "========== Syntax check =========="
if bash -n "$PROJECT_DIR/scripts/slam/run_nav2_saved_map.sh"; then
  ok "bash syntax: run_nav2_saved_map.sh"
else
  bad "bash syntax error: run_nav2_saved_map.sh"
fi

if python3 -m py_compile "$PROJECT_DIR/ros2_bridge/m1_pwm_cmd_vel_bridge.py"; then
  ok "python syntax: m1_pwm_cmd_vel_bridge.py"
else
  bad "python syntax error: m1_pwm_cmd_vel_bridge.py"
fi

if python3 -m py_compile "$PROJECT_DIR/ros2_bridge/cmd_vel_to_rosmaster.py"; then
  ok "python syntax: cmd_vel_to_rosmaster.py (backup)"
else
  bad "python syntax error: cmd_vel_to_rosmaster.py"
fi

echo
echo "========== YAML / map check =========="
MAP_YAML="$MAP_YAML" NAV2_PARAMS="$NAV2_PARAMS" python3 - <<'PY'
import os
from pathlib import Path

try:
    import yaml
except Exception as e:
    raise SystemExit(f"[FAIL] python yaml import failed: {e}")

fail = False

def check_yaml(path_str, name):
    global fail
    p = Path(path_str)
    if not p.exists():
        print(f"[FAIL] {name} missing: {p}")
        fail = True
        return None
    try:
        data = yaml.safe_load(p.read_text())
        print(f"[OK] YAML parse: {p}")
        return data
    except Exception as e:
        print(f"[FAIL] YAML parse failed: {p}: {e}")
        fail = True
        return None

map_yaml = Path(os.environ["MAP_YAML"])
nav2_params = Path(os.environ["NAV2_PARAMS"])

map_data = check_yaml(str(map_yaml), "map yaml")
_ = check_yaml(str(nav2_params), "nav2 params")

if map_data:
    required = ["image", "resolution", "origin", "occupied_thresh", "free_thresh"]
    for k in required:
        if k not in map_data:
            print(f"[FAIL] map yaml missing key: {k}")
            fail = True

    img = map_data.get("image")
    if img:
        img_path = Path(img)
        if not img_path.is_absolute():
            img_path = map_yaml.parent / img_path
        if img_path.exists():
            print(f"[OK] map image exists: {img_path}")
        else:
            print(f"[FAIL] map image missing: {img_path}")
            fail = True

raise SystemExit(1 if fail else 0)
PY

if [ $? -eq 0 ]; then
  ok "YAML / map validation"
else
  bad "YAML / map validation"
fi

echo
echo "========== Device check =========="
if [ -e /dev/ttyACM0 ]; then
  ok "possible chassis device: /dev/ttyACM0"
elif [ -e /dev/ttyUSB0 ]; then
  ok "possible chassis device: /dev/ttyUSB0"
else
  warn "no /dev/ttyACM0 or /dev/ttyUSB0 found. If chassis is disconnected, this is expected."
fi

echo
echo "========== Result =========="
if [ "$fail" -eq 0 ]; then
  echo "[PASS] Nav2 environment basic check passed."
else
  echo "[FAIL] Nav2 environment has problems. Fix FAIL items first."
fi

exit "$fail"
