#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENARIO="${1:-multi_success}"
shift || true
exec python3 -u "$ROOT/src/apps/mock_online_map_plan_backend.py" --scenario "$SCENARIO" "$@"
