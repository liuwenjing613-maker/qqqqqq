#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

echo "===== existing intervention tests ====="
if [[ -x "$ROOT/scripts/intervention/run_intervention_tests.sh" ]]; then
  bash "$ROOT/scripts/intervention/run_intervention_tests.sh"
else
  echo "[WARN] existing intervention test entry not found; run its repository test command separately"
fi

echo "===== online bridge unit/static tests ====="
bash "$ROOT/scripts/fusion/run_fusion_unit_tests.sh"

echo "===== deterministic full-flow simulation ====="
python3 "$ROOT/tests/simulate_online_fusion_flow.py"
