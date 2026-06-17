#!/usr/bin/env bash
# Free memory before a VLM inference on memory-constrained boards.
set -euo pipefail

MODEL="${1:-qwen2.5vl:3b}"
HOST="${OLLAMA_HOST:-127.0.0.1:11434}"

echo "[prep] memory before:"
free -h

# Ensure swap exists (survives reboot if setup_ollama_memory.sh was run).
if ! swapon --show 2>/dev/null | grep -q '/swapfile'; then
  if [[ -f /swapfile ]]; then
    swapon /swapfile || true
  else
    echo "[prep] WARN: no swap. Run: sudo bash scripts/setup_ollama_memory.sh"
  fi
fi

# Unload all loaded models to avoid two llama-server copies (common OOM cause).
if curl -sf --max-time 3 "http://${HOST}/api/tags" >/dev/null; then
  echo "[prep] unloading loaded models..."
  ollama ps 2>/dev/null | awk 'NR>1 {print $1}' | while read -r name; do
    [[ -z "${name}" || "${name}" == "NAME" ]] && continue
    echo "[prep] ollama stop ${name}"
    ollama stop "${name}" 2>/dev/null || true
  done
fi

# Kill orphan llama-server from prior OOM/crash.
pids=$(ps -eo pid=,cmd= | grep 'llama-server' | grep -v grep | awk '{print $1}' || true)
if [[ -n "${pids}" ]]; then
  echo "[prep] killing orphan llama-server: ${pids}"
  kill -9 ${pids} 2>/dev/null || true
  sleep 1
fi

systemctl restart ollama
sleep 2

if ! curl -sf --max-time 5 "http://${HOST}/api/tags" >/dev/null; then
  echo "[prep] ERROR: Ollama not reachable after restart"
  exit 1
fi

echo "[prep] target model: ${MODEL}"
if ! ollama list 2>/dev/null | grep -q "${MODEL%%:*}"; then
  ollama pull "${MODEL}"
fi

echo "[prep] memory after:"
free -h

echo "[prep] warming up model (text + vision)..."
cd /root/rdk_x5_vln_robot
python3 src/apps/ollama_warmup.py --model "${MODEL}" || {
  echo "[prep] WARN: warmup failed; first infer will be slower"
}

echo "[prep] ready for inference with model=${MODEL}"
