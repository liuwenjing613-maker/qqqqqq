#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPO_ROOT="$(cd "$ROOT/.." && pwd)"
python3 -m py_compile \
  "$ROOT/src/fusion/live_frontier_backend_core_v2.py" \
  "$ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  "$ROOT/scripts/fusion/make_nav2_online_params_v2.py" \
  "$ROOT/scripts/fusion/make_fullflow_servo_config_v2.py" \
  "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py"
for file in \
  "$ROOT/scripts/fusion/start_v1_map_qwen_fullflow_v2.sh" \
  "$ROOT/scripts/fusion/stop_v1_map_qwen_fullflow_v2.sh" \
  "$ROOT/scripts/fusion/check_v1_map_qwen_fullflow_v2.sh"; do
  bash -n "$file"
done
python3 "$ROOT/tests/test_live_frontier_backend_core_v2.py"
python3 "$ROOT/tests/simulate_fullflow_v2_protocol.py"
python3 - "$ROOT/configs/online_map_plan_fullflow_v2.yaml" "$ROOT/configs/online_map_plan_fusion_fullflow_v2.yaml" <<'PY'
import sys, yaml
for path in sys.argv[1:]:
    data = yaml.safe_load(open(path, encoding='utf-8'))
    assert isinstance(data, dict) and data
print('yaml configs: PASS')
PY
echo "fullflow_v2 static tests: PASS"
