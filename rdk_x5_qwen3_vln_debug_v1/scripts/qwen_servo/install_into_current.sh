#!/usr/bin/env bash
set -euo pipefail
PATCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="${1:-/root/rdk_x5_vln_robot/rdk_x5_qwen3_vln_debug_v1}"

if [[ ! -f "$TARGET/src/apps/qwen_vln_debug_node.py" ]]; then
  echo "ERROR: target is not the current qwen debug folder: $TARGET" >&2
  exit 1
fi

mkdir -p "$TARGET/src/control" "$TARGET/src/apps" "$TARGET/configs" \
  "$TARGET/scripts" "$TARGET/tests" "$TARGET/docs"

# Additive copy only. None of the tuned Qwen files below are touched:
# qwen_vln_debug_node.py, qwen_client.py, state_machine.py, prompts/*,
# configs/qwen3_vln_debug.yaml.
cp "$PATCH_ROOT/src/control/qwen_visual_servo.py" "$TARGET/src/control/"
cp "$PATCH_ROOT/src/apps/qwen_visual_servo_node.py" "$TARGET/src/apps/"
cp "$PATCH_ROOT/configs/qwen3_vln_servo.yaml" "$TARGET/configs/"
cp "$PATCH_ROOT/scripts/qwen_servo/make_fast_qwen_config.py" "$TARGET/scripts/"
cp "$PATCH_ROOT/scripts/qwen_servo/start_servo_node.sh" "$TARGET/scripts/"
cp "$PATCH_ROOT/scripts/qwen_servo/start_live_servo.sh" "$TARGET/scripts/"
cp "$PATCH_ROOT/scripts/qwen_servo/check_servo_ready.sh" "$TARGET/scripts/"
cp "$PATCH_ROOT/tests/test_visual_servo.py" "$TARGET/tests/"
cp "$PATCH_ROOT/docs/SERVO_V2_ANY_POINT_GUIDE.md" "$TARGET/docs/"

chmod +x \
  "$TARGET/src/apps/qwen_visual_servo_node.py" \
  "$TARGET/scripts/make_fast_qwen_config.py" \
  "$TARGET/scripts/start_servo_node.sh" \
  "$TARGET/scripts/start_live_servo.sh" \
  "$TARGET/scripts/check_servo_ready.sh"

echo "Installed additive servo V2 (any valid pixel) into: $TARGET"
echo "Qwen source/prompts/original config were not overwritten."
echo "Policy: any fresh valid model pixel may drive."
echo "Run tests: cd $TARGET && python3 -m unittest tests/test_visual_servo.py -v"
