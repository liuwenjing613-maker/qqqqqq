#!/usr/bin/env bash
# Run local wake-word detection test (sherpa-onnx, no cloud ASR).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

VENV_DIR="${ROOT_DIR}/.venv-kws"
MODEL_DIR="${ROOT_DIR}/models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"
DEVICE="${VOICE_ALSA_DEVICE:-plughw:0,0}"

if [[ ! -d "${VENV_DIR}" ]] || [[ ! -f "${ROOT_DIR}/config/keywords.txt" ]]; then
    echo "[INFO] First run: executing setup_kws.sh"
    bash "${ROOT_DIR}/scripts/setup_kws.sh"
fi

if pgrep -f "voice_asr_node.py" >/dev/null 2>&1; then
    echo "[WARN] voice_asr_node.py is running and may hold the microphone."
    echo "[WARN] Stop it with Ctrl+C in that terminal before testing KWS."
fi

# shellcheck source=scripts/lib/setup_usb_mic.sh
source "${ROOT_DIR}/scripts/lib/setup_usb_mic.sh"
setup_usb_mic

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

exec python3 "${ROOT_DIR}/src/kws_test.py" \
    --device "${DEVICE}" \
    --encoder "${MODEL_DIR}/encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx" \
    --decoder "${MODEL_DIR}/decoder-epoch-13-avg-2-chunk-16-left-64.onnx" \
    --joiner "${MODEL_DIR}/joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx" \
    --tokens "${MODEL_DIR}/tokens.txt" \
    --keywords-file "${ROOT_DIR}/config/keywords.txt"
