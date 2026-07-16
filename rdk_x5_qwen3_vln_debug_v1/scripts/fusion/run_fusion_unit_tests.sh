#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest -v "$ROOT/tests/test_online_map_protocol.py"
python3 -m py_compile \
  "$ROOT/src/fusion/online_map_protocol.py" \
  "$ROOT/src/apps/online_map_plan_bridge_node.py" \
  "$ROOT/src/apps/mock_online_map_plan_backend.py" \
  "$ROOT/scripts/fusion/make_online_fusion_runtime_config.py" \
  "$ROOT/scripts/qwen_servo/make_intervention_servo_config.py"
while IFS= read -r script; do bash -n "$script"; done < <(find "$ROOT/scripts/fusion" -maxdepth 1 -type f -name '*.sh' | sort)

TMP_DIR="$(mktemp -d /tmp/qwen_fusion_config_test.XXXXXX)"
trap 'rm -rf "$TMP_DIR"' EXIT
cat > "$TMP_DIR/base.yaml" <<'YAML'
topics:
  cmd_output: /cmd_vel_autonomy
third_view_intervention:
  enabled: false
  topics:
    ego_cmd: /cmd_vel_ego
YAML
python3 "$ROOT/scripts/fusion/make_online_fusion_runtime_config.py" \
  --servo-config "$TMP_DIR/base.yaml" \
  --fusion-config "$ROOT/configs/online_map_plan_fusion.yaml" \
  --output "$TMP_DIR/fusion.yaml" --enable true >/dev/null
python3 "$ROOT/scripts/qwen_servo/make_intervention_servo_config.py" \
  --input "$TMP_DIR/fusion.yaml" --output "$TMP_DIR/intervention.yaml" >/dev/null
python3 - "$TMP_DIR/fusion.yaml" "$TMP_DIR/intervention.yaml" <<'PY'
import sys, yaml
fusion = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
intervention = yaml.safe_load(open(sys.argv[2], encoding='utf-8'))
assert fusion['online_map_plan_fusion']['enabled'] is True
assert fusion['third_view_intervention']['enabled'] is True
assert fusion['topics']['cmd_output'] == '/cmd_vel_autonomy'
assert intervention['topics']['cmd_output'] == '/cmd_vel_ego'
assert intervention['third_view_intervention']['enabled'] is True
print('[PASS] runtime config overlay/remap')
PY

echo "[PASS] online map fusion unit/static tests"
