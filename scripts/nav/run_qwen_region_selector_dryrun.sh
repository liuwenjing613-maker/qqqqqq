#!/usr/bin/env bash
# Offline dry-run: Qwen region selection from frozen snapshot (no robot motion).
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_DIR"

SNAPSHOT_JSON=""
INSTRUCTION=""
REPEAT=1
CONFIG="$PROJECT_DIR/configs/qwen_region_selector.yaml"
OUTPUT_DIR=""
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --snapshot-json)
      SNAPSHOT_JSON="$2"
      shift 2
      ;;
    --instruction)
      INSTRUCTION="$2"
      shift 2
      ;;
    --repeat)
      REPEAT="$2"
      shift 2
      ;;
    --config)
      CONFIG="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --mock-response)
      EXTRA_ARGS+=(--mock-response "$2")
      shift 2
      ;;
    --no-api)
      EXTRA_ARGS+=(--no-api)
      shift
      ;;
    *)
      echo "[FATAL] unknown arg: $1"
      exit 1
      ;;
  esac
done

if [[ -z "$SNAPSHOT_JSON" ]] || [[ -z "$INSTRUCTION" ]]; then
  echo "Usage: $0 --snapshot-json <path> --instruction \"...\" [--repeat N]"
  exit 1
fi

SNAPSHOT_JSON="$(readlink -f "$SNAPSHOT_JSON")"
CONFIG="$(readlink -f "$CONFIG")"

# Load API credentials if present (do not print secrets)
for envfile in "$PROJECT_DIR/.env" "$PROJECT_DIR/voice_interaction/.env"; do
  if [[ -f "$envfile" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$envfile"
    set +a
    break
  fi
done

CMD=(python3 -u "$PROJECT_DIR/src/vlm/qwen_region_selector_cli.py"
  --snapshot-json "$SNAPSHOT_JSON"
  --instruction "$INSTRUCTION"
  --config "$CONFIG"
  --repeat "$REPEAT")

if [[ -n "$OUTPUT_DIR" ]]; then
  CMD+=(--output-dir "$(readlink -f "$OUTPUT_DIR")")
fi

CMD+=("${EXTRA_ARGS[@]}")

echo "[DRYRUN] snapshot=$SNAPSHOT_JSON instruction=$INSTRUCTION repeat=$REPEAT"
echo "[DRYRUN] api_key_set=$([[ -n "${DASHSCOPE_API_KEY:-}" ]] && echo true || echo false)"
echo "[DRYRUN] model=${QWEN_MODEL:-qwen3-vl-flash(default)}"

"${CMD[@]}"
