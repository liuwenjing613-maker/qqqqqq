#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REPO_ROOT="$(cd "$ROOT/.." && pwd)"
CFG="${1:-$ROOT/configs/simple_handoff_v2.yaml}"

echo "[CHECK] Python syntax"
python3 -m py_compile \
  "$ROOT/src/intervention/simple_handoff_core_v2.py" \
  "$ROOT/src/apps/simple_handoff_supervisor_v2.py" \
  "$ROOT/scripts/fusion/make_simple_handoff_runtime_config_v2.py" \
  "$ROOT/scripts/fusion/log_fan_in_v2.py" \
  "$ROOT/scripts/fusion/simple_handoff_event_console_v2.py"

echo "[CHECK] Shell syntax"
bash -n "$ROOT/scripts/fusion/start_v1_map_qwen_simple_voice_v2.sh"
bash -n "$ROOT/scripts/fusion/check_simple_handoff_v2.sh"

echo "[CHECK] YAML structure and parameter invariants"
python3 - "$CFG" <<'PY'
from pathlib import Path
import sys, yaml
p=Path(sys.argv[1]); root=yaml.safe_load(p.read_text(encoding='utf-8')) or {}
cfg=root.get('simple_handoff_v2', root)
assert cfg.get('enabled') is True
s=cfg['stuck']; r=cfg['revisit']; h=cfg['handoff']; run=cfg['runtime']; mux=cfg['cmd_mux']
assert s['window_s'] > 0
assert 0 < s['pose_sample_period_s'] < s['window_s']
assert 0 < s['max_actual_path_m'] < s['min_commanded_linear_m']
assert 0 < s['max_actual_yaw_rad'] < s['min_commanded_yaw_rad']
assert r['corridor_radius_m'] > 0
assert r['recent_path_length_m'] > 0
assert r['history_gap_length_m'] >= 2 * r['corridor_radius_m'] - 1e-9
assert r['max_pose_jump_m'] > r['path_sample_step_m']
assert 0 <= r['max_new_area_ratio'] <= 1
assert r['required_consecutive_checks'] >= 1
assert h['decision'] in {'MAP_QWEN', 'MAP_DIRECT'}
assert h['request_ack_timeout_s'] > h['stop_hold_s']
assert h['navigation_timeout_s'] > h['request_ack_timeout_s']
assert 1.0 / run['status_hz'] < mux['mode_timeout_sec']
print('[CHECK] YAML PASS')
PY

echo "[CHECK] Pure trigger unit tests"
python3 "$ROOT/tests/test_simple_handoff_core_v2.py"

echo "[CHECK] Required current-branch integration files"
for f in \
  "$ROOT/scripts/qwen_servo/start_live_servo_voice.sh" \
  "$ROOT/src/apps/online_map_plan_bridge_node.py" \
  "$ROOT/src/apps/online_map_qwen_nav_backend_v2.py" \
  "$ROOT/src/apps/cmd_vel_intervention_mux.py" \
  "$ROOT/configs/qwen3_vln_servo.yaml" \
  "$ROOT/configs/online_map_plan_fusion_fullflow_v2.yaml" \
  "$ROOT/configs/online_map_plan_fullflow_v2.yaml" \
  "$REPO_ROOT/scripts/slam/run_slam_calibrated.sh" \
  "$REPO_ROOT/configs/nav2_online_slam_fusion_navigation_launch_v2.py"; do
  [[ -f "$f" ]] || { echo "[CHECK][FAIL] missing $f" >&2; exit 2; }
done

echo "[CHECK] Online bridge protocol compatibility"
PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
from fusion.online_map_protocol import build_backend_request
request = build_backend_request(
    {
        'request_id': 'simple-check-0001',
        'decision': 'MAP_QWEN',
        'reason_code': 'LOW_NOVELTY_REVISIT',
        'candidate_ids': [],
        'robot_pose': {'frame_id': 'map', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'evidence': {'new_area_ratio': 0.12},
    },
    instruction='find the bottle',
    map_topic='/map',
    odom_topic='/odom',
    scan_topic='/scan_filtered',
    use_memory=False,
)
assert request['operation'] == 'EXTRACT_SELECT_AND_NAVIGATE'
assert request['request_id'] == 'simple-check-0001'
assert request['options']['keep_mapping_alive'] is True
print('[CHECK] bridge protocol PASS')
PY

echo "[CHECK] Runtime config compatibility"
TMP_DIR="$(mktemp -d /tmp/simple_handoff_check.XXXXXX)"
trap 'rm -rf "$TMP_DIR"' EXIT
python3 "$ROOT/scripts/fusion/make_simple_handoff_runtime_config_v2.py" \
  --servo-config "$ROOT/configs/qwen3_vln_servo.yaml" \
  --fusion-config "$ROOT/configs/online_map_plan_fusion_fullflow_v2.yaml" \
  --simple-config "$CFG" \
  --backend-config "$ROOT/configs/online_map_plan_fullflow_v2.yaml" \
  --servo-output "$TMP_DIR/servo.yaml" \
  --backend-output "$TMP_DIR/backend.yaml" \
  --backend-debug-dir "$TMP_DIR/backend_debug" >/dev/null
python3 - "$TMP_DIR/servo.yaml" "$TMP_DIR/backend.yaml" <<'PY'
import sys, yaml
servo=yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
backend=yaml.safe_load(open(sys.argv[2], encoding='utf-8'))
assert servo['topics']['cmd_output']=='/cmd_vel_ego'
assert servo['third_view_intervention']['enabled'] is True
assert servo['third_view_intervention']['topics']['map_cmd']=='/cmd_vel_map'
assert servo['online_map_plan_fusion']['enabled'] is True
assert servo['online_map_plan_fusion']['candidate_probe']['enabled'] is False
assert backend['online_map_plan_fullflow_v2']['debug_dir']
print('[CHECK] runtime config PASS')
PY

echo "[CHECK] PASS: static checks, runtime compatibility and deterministic trigger tests completed"
