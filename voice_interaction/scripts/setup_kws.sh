#!/usr/bin/env bash
# One-time setup: venv, sherpa-onnx, KWS model, keywords file.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

VENV_DIR="${ROOT_DIR}/.venv-kws"
MODEL_ARCHIVE="sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2"
MODEL_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/${MODEL_ARCHIVE}"
MODEL_DIR="${ROOT_DIR}/models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"

if ! python3 -m venv --help >/dev/null 2>&1; then
    echo "[SETUP] Installing python3-venv..."
    apt-get update -qq
    apt-get install -y python3-venv wget
fi

if [[ ! -d "${VENV_DIR}" ]]; then
    echo "[SETUP] Creating virtual environment: ${VENV_DIR}"
    python3 -m venv --system-site-packages "${VENV_DIR}"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

python3 -m pip install --upgrade pip
python3 -m pip install -r "${ROOT_DIR}/requirements-kws.txt"

python3 -c "import sherpa_onnx; print('[SETUP] sherpa-onnx import OK')"

mkdir -p "${ROOT_DIR}/models/kws" "${ROOT_DIR}/config"

if [[ ! -d "${MODEL_DIR}" ]]; then
    echo "[SETUP] Downloading KWS model..."
    cd "${ROOT_DIR}/models/kws"
    wget -q --show-progress "${MODEL_URL}"
    tar xf "${MODEL_ARCHIVE}"
    rm -f "${MODEL_ARCHIVE}"
    cd "${ROOT_DIR}"
else
    echo "[SETUP] Model already present: ${MODEL_DIR}"
fi

for required in tokens.txt en.phone \
    encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx \
    decoder-epoch-13-avg-2-chunk-16-left-64.onnx \
    joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx; do
    if [[ ! -f "${MODEL_DIR}/${required}" ]]; then
        echo "[FATAL] Missing model file: ${MODEL_DIR}/${required}"
        exit 1
    fi
done

echo "[SETUP] Generating keywords.txt from keywords_raw.txt"
sherpa-onnx-cli text2token \
    --tokens "${MODEL_DIR}/tokens.txt" \
    --tokens-type phone+ppinyin \
    --lexicon "${MODEL_DIR}/en.phone" \
    "${ROOT_DIR}/config/keywords_raw.txt" \
    "${ROOT_DIR}/config/keywords.txt"

echo "[SETUP] keywords.txt:"
cat "${ROOT_DIR}/config/keywords.txt"

echo "[SETUP] Done."
