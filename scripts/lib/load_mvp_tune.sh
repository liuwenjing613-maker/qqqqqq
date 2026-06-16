#!/usr/bin/env bash
# 从 configs/mvp_tune.yaml 加载统一调参，导出为环境变量供启动脚本使用。
# 用法: source scripts/lib/load_mvp_tune.sh

PROJECT_DIR="${PROJECT_DIR:-$HOME/rdk_x5_vln_robot}"
PROJECT_DIR="$(eval echo "$PROJECT_DIR")"
MVP_TUNE_FILE="${MVP_TUNE_FILE:-$PROJECT_DIR/configs/mvp_tune.yaml}"
MVP_TUNE_FILE="$(eval echo "$MVP_TUNE_FILE")"

# 启动前已设置 CHASSIS_PORT 时保留，便于临时覆盖配置文件
_CHASSIS_PORT_OVERRIDE="${CHASSIS_PORT:-}"

eval "$(python3 "$PROJECT_DIR/src/config/mvp_tune.py" --config "$MVP_TUNE_FILE" --shell-export)"

if [ -n "$_CHASSIS_PORT_OVERRIDE" ]; then
  export CHASSIS_PORT="$_CHASSIS_PORT_OVERRIDE"
fi
