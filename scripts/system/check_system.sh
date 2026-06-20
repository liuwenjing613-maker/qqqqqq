#!/usr/bin/env bash

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"

echo "============================================================"
echo " RDK X5 VLN Robot - System Check"
echo "============================================================"

cd $PROJECT_DIR

echo "[1/9] Check project directory..."
pwd
ls -d perception ros2_bridge src scripts logs 2>/dev/null || true

echo
echo "[2/9] Check ROS2 environment..."
source /opt/tros/humble/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
which ros2 || true

echo
echo "[3/9] Check optional YOLO-World environment..."
if [ -f "$PROJECT_DIR/source_stage10.sh" ]; then
  source "$PROJECT_DIR/source_stage10.sh"
  echo "source_stage10.sh found"
else
  echo "source_stage10.sh not found, only base TROS used"
fi

ros2 pkg list | grep hobot_yolo_world || echo "WARN: hobot_yolo_world not found"

echo
echo "[4/9] Check camera devices..."
ls -l /dev/video* 2>/dev/null || echo "WARN: no /dev/video* found"

echo
echo "[5/9] Check chassis serial..."
ls -l /dev/myserial 2>/dev/null || echo "WARN: /dev/myserial not found"
ls -l /dev/ttyUSB* 2>/dev/null || true
ls -l /dev/ttyACM* 2>/dev/null || true

echo
echo "[6/9] Check important files..."
FILES=(
  "src/perception/compressed_to_raw_image.py"
  "src/perception/target_backend_red.py"
  "src/perception/target_backend_yolo.py"
  "src/vlm/mock_qwen.py"
  "src/control/mvp_visual_servo.py"
  "src/fsm/mvp_state_machine.py"
  "src/apps/run_mvp_task.py"
  "ros2_bridge/cmd_vel_to_rosmaster.py"
  "scripts/system/stop_all_safe.sh"
  "scripts/mvp/start_red_mvp_raw.sh"
  "scripts/yolo/start_yolo_mvp_raw.sh"
)

for f in "${FILES[@]}"; do
  if [ -f "$f" ]; then
    echo "OK   $f"
  else
    echo "MISS $f"
  fi
done

echo
echo "[7/9] Test Python imports..."
python3 - <<'PY'
import sys
sys.path.append("/root/rdk_x5_vln_robot")
sys.path.append("/home/sunrise/rdk_x5_vln_robot")

mods = [
    "src.vlm.mock_qwen",
    "src.perception.target_backend_red",
    "src.perception.target_backend_yolo",
    "src.control.mvp_visual_servo",
    "src.fsm.mvp_state_machine",
]
for m in mods:
    try:
        __import__(m)
        print("OK  ", m)
    except Exception as e:
        print("FAIL", m, repr(e))
PY

echo
echo "[8/9] Check Rosmaster_Lib..."
python3 - <<'PY'
try:
    from Rosmaster_Lib import Rosmaster
    print("OK Rosmaster_Lib")
except Exception as e:
    print("FAIL Rosmaster_Lib", repr(e))
PY

echo
echo "[9/9] Current ROS topics, if nodes are running..."
ros2 topic list 2>/dev/null || true

echo
echo "============================================================"
echo " System check finished."
echo "============================================================"
