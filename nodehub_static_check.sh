#!/usr/bin/env bash
set +e

ROOT="/root/rdk_x5_vln_robot"
REPORT="$ROOT/docs/nodehub_static_check_report.md"

cd "$ROOT" || exit 1
mkdir -p docs

MISSING=0
BASH_ERROR=0
PY_ERROR=0
YAML_ERROR=0
BIG_FILE=0
BAD_FILE=0
REAL_SECRET=0

echo "# NodeHub Static Check Report" > "$REPORT"
echo "" >> "$REPORT"
echo "- Root: $ROOT" >> "$REPORT"
echo "- Time: $(date '+%Y-%m-%d %H:%M:%S')" >> "$REPORT"
echo "" >> "$REPORT"

check_path() {
  local p="$1"
  if [ -e "$p" ]; then
    echo "OK      $p"
    echo "- OK: \`$p\`" >> "$REPORT"
  else
    echo "MISSING $p"
    echo "- MISSING: \`$p\`" >> "$REPORT"
    MISSING=$((MISSING + 1))
  fi
}

echo "========== 1. 核心文件检查 =========="

for p in \
README.md \
.env.example \
configs \
scripts \
src \
ros2_bridge \
scripts/lib/project_dir.sh \
scripts/lib/run_chassis_bridge.sh \
scripts/system/stop_all_safe.sh \
ros2_bridge/m1_pwm_cmd_vel_bridge.py \
ros2_bridge/simple_scan_filter.py; do
  check_path "$p"
done

echo "========== 2. 主入口脚本检查 =========="

for p in \
scripts/nav/start_yolo_lidar_semantic_explore_nav.sh \
scripts/nav/start_yolo_lidar_failsafe_nav.sh \
scripts/nav/start_yolo_lidar_stable_nav.sh \
scripts/nav/start_qwen_lidar_nav.sh \
scripts/slam/run_joy_mapping_calibrated.sh \
scripts/slam/run_joy_mapping_all.sh \
scripts/slam/run_nav2_saved_map.sh \
scripts/slam/run_nav2_foxglove_click_goal.sh \
rdk_x5_qwen_vln_robot/scripts/nav/start_qwen_api_lidar_nav.sh; do
  if [ -f "$p" ]; then
    echo "ENTRY_OK      $p"
    echo "- ENTRY_OK: \`$p\`" >> "$REPORT"
  else
    echo "ENTRY_MISSING $p"
    echo "- ENTRY_MISSING: \`$p\`" >> "$REPORT"
  fi
done

echo "========== 3. Shell 语法检查 =========="

rm -f /tmp/nodehub_bash_errors.txt

find scripts rdk_x5_qwen_vln_robot -type f -name "*.sh" 2>/dev/null | sort | while read -r f; do
  bash -n "$f" >/tmp/bash_check_err.txt 2>&1
  if [ $? -eq 0 ]; then
    echo "BASH_OK    $f"
  else
    echo "BASH_ERROR $f"
    echo "$f" >> /tmp/nodehub_bash_errors.txt
    echo "- BASH_ERROR: \`$f\`" >> "$REPORT"
    cat /tmp/bash_check_err.txt >> "$REPORT"
  fi
done

if [ -f /tmp/nodehub_bash_errors.txt ]; then
  BASH_ERROR=$(wc -l < /tmp/nodehub_bash_errors.txt)
fi

echo "========== 4. Python 语法检查 =========="

rm -f /tmp/nodehub_py_errors.txt

find src ros2_bridge scripts rdk_x5_qwen_vln_robot -type f -name "*.py" 2>/dev/null | sort | while read -r f; do
  python3 -m py_compile "$f" >/tmp/py_check_err.txt 2>&1
  if [ $? -eq 0 ]; then
    echo "PY_OK      $f"
  else
    echo "PY_ERROR   $f"
    echo "$f" >> /tmp/nodehub_py_errors.txt
    echo "- PY_ERROR: \`$f\`" >> "$REPORT"
    cat /tmp/py_check_err.txt >> "$REPORT"
  fi
done

if [ -f /tmp/nodehub_py_errors.txt ]; then
  PY_ERROR=$(wc -l < /tmp/nodehub_py_errors.txt)
fi

echo "========== 5. YAML 解析检查 =========="

python3 - <<'PY' > /tmp/nodehub_yaml_result.txt
from pathlib import Path
import yaml

files = []
for folder in ["configs", "maps"]:
    p = Path(folder)
    if p.exists():
        files.extend(sorted(p.glob("*.yaml")))

for f in files:
    try:
        yaml.safe_load(f.read_text())
        print("YAML_OK", f)
    except Exception as e:
        print("YAML_ERROR", f, repr(e))
PY

cat /tmp/nodehub_yaml_result.txt

if grep -q "YAML_ERROR" /tmp/nodehub_yaml_result.txt; then
  YAML_ERROR=$(grep -c "YAML_ERROR" /tmp/nodehub_yaml_result.txt)
fi

echo "========== 6. 大文件检查 =========="

BIG="$(find . -type f -size +10M -exec ls -lh {} \; | sort)"
if [ -n "$BIG" ]; then
  echo "$BIG"
  BIG_FILE=$(echo "$BIG" | wc -l)
else
  echo "OK no file larger than 10MB"
fi

echo "========== 7. 不应提交文件类型检查 =========="

BAD="$(find . -type f \( \
-name "*.pt" -o \
-name "*.pth" -o \
-name "*.onnx" -o \
-name "*.engine" -o \
-name "*.mp4" -o \
-name "*.avi" -o \
-name "*.bag" -o \
-name "*.db3" -o \
-name "*.zip" -o \
-name "*.rar" -o \
-name "*.bundle" \
\) -exec ls -lh {} \; | sort)"

if [ -n "$BAD" ]; then
  echo "$BAD"
  BAD_FILE=$(echo "$BAD" | wc -l)
else
  echo "OK no forbidden artifact type"
fi

echo "========== 8. 真实密钥检查 =========="

SECRET="$(grep -RInE 'sk-[A-Za-z0-9_-]{20,}|AKIA[0-9A-Z]{16}|AIza[0-9A-Za-z_-]{20,}|dashscope_[A-Za-z0-9_-]{20,}|Bearer [A-Za-z0-9._-]{20,}' . \
  --exclude=".env.example" \
  --exclude="nodehub_static_check_report.md" \
  --exclude="nodehub_static_check.sh" 2>/dev/null)"

if [ -n "$SECRET" ]; then
  echo "$SECRET"
  REAL_SECRET=$(echo "$SECRET" | wc -l)
else
  echo "OK no obvious real secret"
fi

echo "========== 9. 总结 =========="

SIZE="$(du -sh . | awk '{print $1}')"

echo "MISSING=$MISSING"
echo "BASH_ERROR=$BASH_ERROR"
echo "PY_ERROR=$PY_ERROR"
echo "YAML_ERROR=$YAML_ERROR"
echo "BIG_FILE=$BIG_FILE"
echo "BAD_FILE=$BAD_FILE"
echo "REAL_SECRET=$REAL_SECRET"
echo "SIZE=$SIZE"

{
  echo ""
  echo "## Summary"
  echo ""
  echo "- MISSING=$MISSING"
  echo "- BASH_ERROR=$BASH_ERROR"
  echo "- PY_ERROR=$PY_ERROR"
  echo "- YAML_ERROR=$YAML_ERROR"
  echo "- BIG_FILE=$BIG_FILE"
  echo "- BAD_FILE=$BAD_FILE"
  echo "- REAL_SECRET=$REAL_SECRET"
  echo "- SIZE=$SIZE"
} >> "$REPORT"

if [ "$MISSING" -eq 0 ] && [ "$BASH_ERROR" -eq 0 ] && [ "$PY_ERROR" -eq 0 ] && [ "$YAML_ERROR" -eq 0 ] && [ "$REAL_SECRET" -eq 0 ]; then
  echo "NODEHUB_STATIC_RESULT=PASS"
  echo "NODEHUB_STATIC_RESULT=PASS" >> "$REPORT"
else
  echo "NODEHUB_STATIC_RESULT=NEED_FIX"
  echo "NODEHUB_STATIC_RESULT=NEED_FIX" >> "$REPORT"
fi

echo "Report saved to: $REPORT"
