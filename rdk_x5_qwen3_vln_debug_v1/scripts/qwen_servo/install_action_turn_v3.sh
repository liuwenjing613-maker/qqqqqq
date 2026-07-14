#!/usr/bin/env bash
set -euo pipefail
PATCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="${1:-/root/rdk_x5_vln_robot/rdk_x5_qwen3_vln_debug_v1}"

if [[ ! -f "$TARGET/src/qwen_vln/qwen_client.py" ]] || \
   [[ ! -f "$TARGET/scripts/qwen_servo/start_live_servo.sh" ]]; then
  echo "ERROR: target is not the expected qwen3 debug folder: $TARGET" >&2
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$TARGET/backups/action_turn_v3_$STAMP"
mkdir -p "$BACKUP"

FILES=(
  src/qwen_vln/types.py
  src/qwen_vln/prompt_manager.py
  src/qwen_vln/qwen_client.py
  src/qwen_vln/visualizer.py
  src/control/qwen_visual_servo.py
  src/apps/qwen_visual_servo_node.py
  prompts/common.txt
  prompts/observe.txt
  prompts/search.txt
  prompts/track.txt
  prompts/verify.txt
  configs/qwen3_vln_servo.yaml
  tests/test_action_protocol.py
  tests/test_visual_servo.py
)

for rel in "${FILES[@]}"; do
  src="$PATCH_ROOT/$rel"
  dst="$TARGET/$rel"
  [[ -f "$src" ]] || { echo "ERROR: patch file missing: $src" >&2; exit 1; }
  if [[ -f "$dst" ]]; then
    mkdir -p "$BACKUP/$(dirname "$rel")"
    cp -a "$dst" "$BACKUP/$rel"
  fi
  mkdir -p "$(dirname "$dst")"
  cp -a "$src" "$dst"
done

chmod +x "$TARGET/src/apps/qwen_visual_servo_node.py"
echo "Installed action-turn V3 into: $TARGET"
echo "Backup of replaced files: $BACKUP"
echo "Unchanged on purpose: scripts/qwen_servo/start_live_servo.sh, state_machine.py, qwen_vln_debug_node.py"
echo "Run: cd $TARGET && python3 -m unittest discover -s tests -p 'test_*servo.py' -v"
echo "Then: python3 -m unittest tests/test_action_protocol.py -v"
