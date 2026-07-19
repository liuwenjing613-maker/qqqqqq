#!/usr/bin/env bash
# Persistent form of the tested KWS -> post-wake recording -> Qwen ASR path.
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
EVENT_FILE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --event-file) EVENT_FILE="${2:?missing event file}"; shift 2 ;;
    -h|--help)
      echo "Usage: bash scripts/run_voice_function_event_server_v1.sh --event-file PATH"
      exit 0 ;;
    *) echo "[VOICE-MENU][ERROR] unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$EVENT_FILE" ]] || { echo "[VOICE-MENU][ERROR] --event-file is required" >&2; exit 2; }

ENV_FILE="${VOICE_ENV_FILE:-$ROOT_DIR/.env}"
[[ -f "$ENV_FILE" ]] || { echo "[VOICE-MENU][ERROR] missing $ENV_FILE" >&2; exit 1; }
# Preserve hub/CLI overrides across .env sourcing.
_PRESET_RECORD_SECONDS="${VOICE_RECORD_SECONDS-}"
_PRESET_FUNCTION_RECORD_SECONDS="${VOICE_FUNCTION_RECORD_SECONDS-}"
set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a
: "${DASHSCOPE_API_KEY:=${QWEN_API_KEY:-}}"
[[ -n "${DASHSCOPE_API_KEY:-}" ]] || { echo "[VOICE-MENU][ERROR] API key missing" >&2; exit 1; }

# Priority: hub VOICE_FUNCTION_RECORD_SECONDS > caller VOICE_RECORD_SECONDS > .env > 5
if [[ -n "${_PRESET_FUNCTION_RECORD_SECONDS}" ]]; then
  export VOICE_RECORD_SECONDS="$_PRESET_FUNCTION_RECORD_SECONDS"
elif [[ -n "${_PRESET_RECORD_SECONDS}" ]]; then
  export VOICE_RECORD_SECONDS="$_PRESET_RECORD_SECONDS"
fi
export VOICE_RECORD_SECONDS="${VOICE_RECORD_SECONDS:-5}"
export VOICE_FUNCTION_RECORD_SECONDS="${VOICE_FUNCTION_RECORD_SECONDS:-$VOICE_RECORD_SECONDS}"
export VOICE_DEVICE_INDEX="${VOICE_DEVICE_INDEX:-0}"
export VOICE_ALSA_DEVICE="${VOICE_ALSA_DEVICE:-plughw:0,0}"
VENV_DIR="${VOICE_KWS_VENV:-$ROOT_DIR/.venv-kws}"
MODEL_DIR="${VOICE_KWS_MODEL_DIR:-$ROOT_DIR/models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20}"
KEYWORDS_FILE="${VOICE_KEYWORDS_FILE:-$ROOT_DIR/config/keywords.txt}"

if [[ ! -d "$VENV_DIR" || ! -f "$KEYWORDS_FILE" ]]; then
  echo "[VOICE-MENU] KWS 环境缺失，执行现有 setup_kws.sh"
  bash "$ROOT_DIR/scripts/setup_kws.sh"
fi
[[ -f "$ROOT_DIR/assets/i_am_here.wav" ]] || { echo "[VOICE-MENU][ERROR] missing prompt wav" >&2; exit 1; }
# shellcheck source=scripts/lib/setup_usb_mic.sh
source "$ROOT_DIR/scripts/lib/setup_usb_mic.sh"
setup_usb_mic
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

exec python3 -u "$ROOT_DIR/src/kws_function_event_server_v1.py" \
  --device "$VOICE_ALSA_DEVICE" \
  --encoder "$MODEL_DIR/encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx" \
  --decoder "$MODEL_DIR/decoder-epoch-13-avg-2-chunk-16-left-64.onnx" \
  --joiner "$MODEL_DIR/joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx" \
  --tokens "$MODEL_DIR/tokens.txt" \
  --keywords-file "$KEYWORDS_FILE" \
  --num-threads "${VOICE_KWS_THREADS:-1}" \
  --event-file "$EVENT_FILE"
