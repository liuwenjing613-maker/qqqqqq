#!/usr/bin/env bash
# Load .env and start the KWS + ASR voice pipeline.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

if [[ ! -f "${ROOT_DIR}/.env" ]]; then
    echo "[ERROR] 没有找到 .env，请先从 .env.example 复制并填写 DASHSCOPE_API_KEY"
    exit 1
fi

set -a
# shellcheck disable=SC1091
source "${ROOT_DIR}/.env"
set +a

if [[ -z "${DASHSCOPE_API_KEY:-}" || "${DASHSCOPE_API_KEY}" == "sk-your-key-here" ]]; then
    echo "[WARN] DASHSCOPE_API_KEY 未设置或仍是占位符；唤醒后 ASR 会失败。"
fi

if [[ ! -f "${ROOT_DIR}/assets/i_am_here.wav" ]]; then
    echo "[ERROR] 缺少提示音：assets/i_am_here.wav"
    exit 1
fi

echo "[SYSTEM] 已加载语音配置"
echo "[SYSTEM] 启动 KWS + ASR 语音闭环"
echo "[SYSTEM] 麦克风设备: ${VOICE_ALSA_DEVICE:-plughw:0,0}"

exec bash "${ROOT_DIR}/scripts/run_kws_test.sh"
