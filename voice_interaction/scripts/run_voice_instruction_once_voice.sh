#!/usr/bin/env bash
# One-shot form of the tested KWS + ASR + translation pipeline.
# It keeps all voice progress visible in the current terminal and exits only
# after a valid English navigation instruction has been written.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

OUTPUT_FILE="${VOICE_INSTRUCTION_FILE:-}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-file)
      [[ $# -ge 2 ]] || { echo "[VOICE][ERROR] --output-file 缺少路径" >&2; exit 2; }
      OUTPUT_FILE="$2"
      shift 2
      ;;
    -h|--help)
      echo "Usage: bash scripts/run_voice_instruction_once_voice.sh --output-file PATH"
      exit 0
      ;;
    *)
      echo "[VOICE][ERROR] 未知参数：$1" >&2
      exit 2
      ;;
  esac
done

[[ -n "$OUTPUT_FILE" ]] || {
  echo "[VOICE][ERROR] 必须提供 --output-file PATH" >&2
  exit 2
}

ENV_FILE="${VOICE_ENV_FILE:-$ROOT_DIR/.env}"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "[VOICE][ERROR] 没有找到配置文件：$ENV_FILE" >&2
  echo "[VOICE][ERROR] 请从 $ROOT_DIR/.env.example 复制并填写 DASHSCOPE_API_KEY。" >&2
  exit 1
fi

set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a

if [[ -z "${DASHSCOPE_API_KEY:-${QWEN_API_KEY:-}}" ]]; then
  echo "[VOICE][ERROR] DASHSCOPE_API_KEY / QWEN_API_KEY 未配置。" >&2
  exit 1
fi

export VOICE_RECORD_SECONDS="${VOICE_RECORD_SECONDS:-10}"
export VOICE_DEVICE_INDEX="${VOICE_DEVICE_INDEX:-0}"
export VOICE_ALSA_DEVICE="${VOICE_ALSA_DEVICE:-plughw:0,0}"

VENV_DIR="${VOICE_KWS_VENV:-$ROOT_DIR/.venv-kws}"
MODEL_DIR="${VOICE_KWS_MODEL_DIR:-$ROOT_DIR/models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20}"
KEYWORDS_FILE="${VOICE_KEYWORDS_FILE:-$ROOT_DIR/config/keywords.txt}"

if [[ ! -d "$VENV_DIR" || ! -f "$KEYWORDS_FILE" ]]; then
  echo "[VOICE] 首次运行或 KWS 文件缺失，执行 setup_kws.sh"
  bash "$ROOT_DIR/scripts/setup_kws.sh"
fi

if [[ ! -f "$ROOT_DIR/assets/i_am_here.wav" ]]; then
  echo "[VOICE][ERROR] 缺少提示音：$ROOT_DIR/assets/i_am_here.wav" >&2
  exit 1
fi

# Use the same USB microphone initialization as the original voice pipeline.
# shellcheck source=scripts/lib/setup_usb_mic.sh
source "$ROOT_DIR/scripts/lib/setup_usb_mic.sh"
setup_usb_mic

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

python3 - <<'PY'
missing = []
for module in ("sherpa_onnx", "numpy", "openai", "dotenv", "pyaudio"):
    try:
        __import__(module)
    except Exception:
        missing.append(module)
if missing:
    raise SystemExit(
        "[VOICE][ERROR] Python 依赖缺失: " + ", ".join(missing)
        + "。请在 voice_interaction 下执行: "
        + "python3 -m pip install -r requirements.txt -r requirements-kws.txt"
    )
PY

echo "[SYSTEM] 已加载语音配置"
echo "[SYSTEM] 启动一次性 KWS + 10s ASR + 英译流程"
echo "[SYSTEM] ALSA 麦克风：$VOICE_ALSA_DEVICE"
echo "[SYSTEM] PyAudio 设备索引：$VOICE_DEVICE_INDEX"

exec python3 -u "$ROOT_DIR/src/kws_instruction_once_voice.py" \
  --device "$VOICE_ALSA_DEVICE" \
  --encoder "$MODEL_DIR/encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx" \
  --decoder "$MODEL_DIR/decoder-epoch-13-avg-2-chunk-16-left-64.onnx" \
  --joiner "$MODEL_DIR/joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx" \
  --tokens "$MODEL_DIR/tokens.txt" \
  --keywords-file "$KEYWORDS_FILE" \
  --num-threads "${VOICE_KWS_THREADS:-2}" \
  --output-file "$OUTPUT_FILE"
