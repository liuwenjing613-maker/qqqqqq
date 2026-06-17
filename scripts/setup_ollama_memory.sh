#!/usr/bin/env bash
# One-time setup: swap + Ollama memory limits for RDK X5 (7GB RAM).
set -euo pipefail

SWAPFILE="${SWAPFILE:-/swapfile}"
SWAP_GB="${SWAP_GB:-2}"
SERVICE_FILE="/etc/systemd/system/ollama.service"

echo "[setup] configuring ${SWAP_GB}GB swap at ${SWAPFILE}..."
if ! swapon --show | rg -q "${SWAPFILE}"; then
  if [[ ! -f "${SWAPFILE}" ]]; then
    fallocate -l "${SWAP_GB}G" "${SWAPFILE}"
    chmod 600 "${SWAPFILE}"
    mkswap "${SWAPFILE}"
  fi
  swapon "${SWAPFILE}"
fi

if ! rg -q "^${SWAPFILE} " /etc/fstab 2>/dev/null; then
  echo "${SWAPFILE} none swap sw 0 0" >> /etc/fstab
  echo "[setup] added ${SWAPFILE} to /etc/fstab"
fi

echo "[setup] updating ${SERVICE_FILE} memory env..."
python3 << 'PY'
from pathlib import Path

path = Path("/etc/systemd/system/ollama.service")
text = path.read_text()
env_lines = {
    'OLLAMA_NUM_PARALLEL': '1',
    'OLLAMA_MAX_LOADED_MODELS': '1',
    'OLLAMA_KEEP_ALIVE': '2m',
    'OLLAMA_HOST': 'http://127.0.0.1:11434',
}
for key, val in env_lines.items():
    line = f'Environment="{key}={val}"'
    if f'{key}=' in text:
        import re
        text = re.sub(rf'Environment="{key}=[^"]*"', line, text)
    else:
        text = text.replace('[Service]\n', f'[Service]\n{line}\n')

path.write_text(text)
print('[setup] ollama.service env:', ', '.join(f'{k}={v}' for k, v in env_lines.items()))
PY

systemctl daemon-reload
systemctl restart ollama
sleep 2

echo "[setup] done."
free -h
swapon --show
systemctl is-active ollama
