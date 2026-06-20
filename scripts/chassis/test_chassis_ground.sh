#!/usr/bin/env bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
# 落地底盘测试入口
#   bash scripts/chassis/test_chassis_ground.sh direct          # 直连 M1，不经过 ROS（推荐先测）
#   bash scripts/chassis/test_chassis_ground.sh direct forward  # 只测前进
#   bash scripts/chassis/test_chassis_ground.sh ros             # 经 /cmd_vel 桥接测试（与 MVP 同路径）
#   bash scripts/chassis/test_chassis_ground.sh ros-bridge      # 仅启动底盘桥，供另一终端跑 ros 测试
#   bash scripts/chassis/test_chassis_ground.sh sweep           # 多档速度扫描（交互式）

set -e

PROJECT_DIR="${PROJECT_DIR:-$HOME/rdk_x5_vln_robot}"
PROJECT_DIR="$(eval echo "$PROJECT_DIR")"
cd "$PROJECT_DIR"

source "$PROJECT_DIR/scripts/lib/load_mvp_tune.sh"

MODE="${1:-direct}"
ACTION="${2:-all}"

stop_others() {
  echo "[1/3] 停止 MVP / 相机 / YOLO 等进程..."
  bash "$PROJECT_DIR/scripts/system/stop_all_safe.sh" || true
  sleep 1
}

run_direct() {
  stop_others
  echo "[2/3] 直连底盘测试 port=$CHASSIS_PORT"
  echo "      action=$ACTION  (forward | left | right | all)"
  python3 "$PROJECT_DIR/debug_tools/test_chassis_ground_basic.py" \
    --port "$CHASSIS_PORT" \
    --max-vx "$CHASSIS_MAX_VX" \
    --max-wz "$CHASSIS_MAX_WZ" \
    --action "$ACTION"
}

run_ros_bridge() {
  stop_others
  echo "[2/3] 启动底盘桥 -> $CHASSIS_PORT"
  source "$PROJECT_DIR/scripts/lib/run_chassis_bridge.sh"
  mkdir -p logs
  run_chassis_bridge "$PROJECT_DIR/logs/chassis_ground_test_bridge.log"
  sleep 2
  echo "[OK] 底盘桥已后台启动，日志: logs/chassis_ground_test_bridge.log"
  echo "     另开终端执行:"
  echo "     bash scripts/chassis/test_chassis_ground.sh ros $ACTION"
  tail -f "$PROJECT_DIR/logs/chassis_ground_test_bridge.log"
}

run_ros() {
  echo "[3/3] 经 /cmd_vel 测试 action=$ACTION"
  source /opt/tros/humble/setup.bash 2>/dev/null || true
  python3 "$PROJECT_DIR/debug_tools/test_chassis_ground_ros.py" \
    --vx "$MAX_VX" \
    --wz "$(python3 -c "print(${MAX_WZ}*0.8)")" \
    --action "$ACTION"
}

run_sweep() {
  stop_others
  echo "[2/3] 速度扫描（交互式，每档需按 Enter）"
  python3 "$PROJECT_DIR/v0/test_ground_speed_sweep.py" --port "$CHASSIS_PORT"
}

case "$MODE" in
  direct)
    run_direct
    ;;
  ros)
    run_ros
    ;;
  ros-bridge|bridge)
    run_ros_bridge
    ;;
  sweep)
    run_sweep
    ;;
  rotate-sweep)
    stop_others
    echo "[2/3] 转向速度扫描 port=$CHASSIS_PORT"
    python3 "$PROJECT_DIR/debug_tools/test_chassis_ground_basic.py" \
      --port "$CHASSIS_PORT" \
      --max-wz 0.60 \
      --sweep-rotate
    ;;
  *)
    echo "用法:"
    echo "  bash scripts/chassis/test_chassis_ground.sh direct [forward|left|right|all]"
    echo "  bash scripts/chassis/test_chassis_ground.sh ros [forward|left|right|all]"
    echo "  bash scripts/chassis/test_chassis_ground.sh ros-bridge"
    echo "  bash scripts/chassis/test_chassis_ground.sh sweep"
    echo "  bash scripts/chassis/test_chassis_ground.sh rotate-sweep"
    exit 1
    ;;
esac
