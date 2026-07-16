#!/usr/bin/env bash
# Offline checks. They do not require ROS2, a camera, lidar, or a real chassis.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

echo "[1/5] pure intervention policy unit tests"
python3 -m unittest -v \
  "$ROOT/tests/test_intervention_core.py" \
  "$ROOT/tests/test_mux_logic.py"

echo "[2/5] readable trigger simulation"
python3 "$ROOT/tests/simulate_intervention_flow.py"

echo "[2b/5] control-transfer handshake simulation"
python3 "$ROOT/tests/simulate_transfer_handshake.py"

echo "[3/5] Python syntax checks"
python3 -m py_compile \
  "$ROOT/src/intervention/core.py" \
  "$ROOT/src/intervention/mux_logic.py" \
  "$ROOT/src/apps/third_view_intervention_node.py" \
  "$ROOT/src/apps/cmd_vel_intervention_mux.py" \
  "$ROOT/scripts/qwen_servo/make_intervention_servo_config.py" \
  "$ROOT/tests/simulate_transfer_handshake.py"

echo "[4/5] shell syntax checks"
bash -n "$ROOT/scripts/qwen_servo/start_live_servo_with_intervention.sh"
bash -n "$ROOT/scripts/qwen_servo/install_intervention_v1.sh"

echo "[4b/5] disabled-wrapper compatibility test"
compat="$(mktemp -d)"
mkdir -p "$compat/scripts/qwen_servo" "$compat/configs" "$compat/logs"
cp "$ROOT/scripts/qwen_servo/start_live_servo_with_intervention.sh" \
  "$compat/scripts/qwen_servo/"
cp "$ROOT/scripts/qwen_servo/make_intervention_servo_config.py" \
  "$compat/scripts/qwen_servo/"
cat > "$compat/configs/qwen3_vln_servo.yaml" <<'YAML'
third_view_intervention:
  enabled: false
YAML
cat > "$compat/scripts/qwen_servo/start_live_servo.sh" <<'BASE'
#!/usr/bin/env bash
set -euo pipefail
if [[ "${THIRD_VIEW_WRAPPER_ACTIVE:-0}" != "1" ]]; then
  exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_live_servo_with_intervention.sh" "$@"
fi
printf 'ORIGINAL_PATH:%s\n' "$*"
BASE
chmod +x "$compat/scripts/qwen_servo/"*.sh
compat_output="$(bash "$compat/scripts/qwen_servo/start_live_servo.sh" alpha beta)"
grep -q 'ORIGINAL_PATH:alpha beta' <<<"$compat_output"
rm -rf "$compat"
echo "disabled wrapper returns to original path: PASS"

echo "[5/5] runtime config remap test"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT
python3 - "$ROOT/configs/qwen3_vln_servo.yaml" "$tmpdir/enabled.yaml" <<'PY'
from pathlib import Path
import sys
import yaml

src = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
src.setdefault("third_view_intervention", {})["enabled"] = True
Path(sys.argv[2]).write_text(
    yaml.safe_dump(src, allow_unicode=True, sort_keys=False),
    encoding="utf-8",
)
PY
python3 "$ROOT/scripts/qwen_servo/make_intervention_servo_config.py" \
  --input "$tmpdir/enabled.yaml" --output "$tmpdir/runtime.yaml"
python3 - "$tmpdir/runtime.yaml" <<'PY'
from pathlib import Path
import sys
import yaml
cfg = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
assert cfg["topics"]["cmd_output"] == "/cmd_vel_ego", cfg["topics"]
assert cfg["third_view_intervention"]["enabled"] is True
print("runtime config remap: PASS")
PY

echo "ALL OFFLINE INTERVENTION TESTS PASSED"
