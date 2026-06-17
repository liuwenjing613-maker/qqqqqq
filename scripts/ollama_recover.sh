#!/usr/bin/env bash
# Recover Ollama when llama-server is stuck on vision encoding.
set -euo pipefail

HOST="${OLLAMA_HOST:-127.0.0.1:11434}"
CPU_THRESHOLD="${OLLAMA_STUCK_CPU_SEC:-600}"

echo "[ollama_recover] checking Ollama at ${HOST}..."

if ! curl -sf --max-time 3 "http://${HOST}/api/tags" >/dev/null; then
  echo "[ollama_recover] Ollama API unreachable, restarting service..."
  systemctl restart ollama || true
  sleep 3
fi

stuck_pids=""
while read -r pid etime pcpu cmd; do
  [[ -z "${pid}" ]] && continue
  # etime like MM:SS or HH:MM:SS
  secs=0
  if [[ "${etime}" =~ ^([0-9]+):([0-9]+)$ ]]; then
    secs=$((10#${BASH_REMATCH[1]} * 60 + 10#${BASH_REMATCH[2]}))
  elif [[ "${etime}" =~ ^([0-9]+):([0-9]+):([0-9]+)$ ]]; then
    secs=$((10#${BASH_REMATCH[1]} * 3600 + 10#${BASH_REMATCH[2]} * 60 + 10#${BASH_REMATCH[3]}))
  fi
  cpu_int=${pcpu%.*}
  cpu_int=${cpu_int:-0}
  if (( secs >= CPU_THRESHOLD && cpu_int >= 200 )); then
    stuck_pids+="${pid} "
    echo "[ollama_recover] stuck llama-server pid=${pid} etime=${etime} cpu=${pcpu}%"
  fi
done < <(ps -eo pid=,etime=,pcpu=,cmd= | rg 'llama-server-bin' || true)

if [[ -n "${stuck_pids}" ]]; then
  echo "[ollama_recover] killing stuck llama-server: ${stuck_pids}"
  kill -9 ${stuck_pids} || true
  sleep 2
fi

if curl -sf --max-time 5 "http://${HOST}/api/tags" >/dev/null; then
  echo "[ollama_recover] Ollama OK"
  curl -s "http://${HOST}/api/tags" | python3 -c "import sys,json; print('models:', [m['name'] for m in json.load(sys.stdin).get('models',[])])"
else
  echo "[ollama_recover] still unreachable; try: systemctl restart ollama"
  exit 1
fi
