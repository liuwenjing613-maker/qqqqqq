#!/usr/bin/env bash
# Idempotently hook the wrapper into the existing start_live_servo.sh while
# preserving every original line below the hook. A timestamped backup is made.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="$ROOT/scripts/qwen_servo/start_live_servo.sh"
MARKER="# THIRD_VIEW_INTERVENTION_V1_HOOK"

[[ -f "$TARGET" ]] || { echo "missing $TARGET" >&2; exit 1; }
if grep -qF "$MARKER" "$TARGET"; then
  echo "[install] hook already present: $TARGET"
  exit 0
fi

BACKUP="$TARGET.before_intervention_v1.$(date +%Y%m%d_%H%M%S).bak"
cp -a "$TARGET" "$BACKUP"

python3 - "$TARGET" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
needle = "set -euo pipefail\n"
hook = '''set -euo pipefail

# THIRD_VIEW_INTERVENTION_V1_HOOK
# The wrapper returns here with THIRD_VIEW_WRAPPER_ACTIVE=1, so enabled=false
# follows the exact original startup code and enabled=true only changes the
# automatic velocity source through a runtime config.
if [[ "${THIRD_VIEW_WRAPPER_ACTIVE:-0}" != "1" ]]; then
  exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_live_servo_with_intervention.sh" "$@"
fi
'''
if needle not in text:
    raise SystemExit("cannot find 'set -euo pipefail' anchor")
text = text.replace(needle, hook, 1)
path.write_text(text, encoding="utf-8")
PY
chmod +x "$TARGET"
echo "[install] patched: $TARGET"
echo "[install] backup : $BACKUP"
echo "[install] same launch command remains valid"
