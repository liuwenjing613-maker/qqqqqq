#!/usr/bin/env bash
set -e

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/project_dir.sh"
cd "$PROJECT_DIR"
echo "[ollama_recover] killing stuck llama/ollama runners..."
pkill -9 -f llama-server || true
pkill -9 -f ollama_llama_server || true
pkill -9 -f "ollama runner" || true
pkill -9 -f "qwen2.5vl" || true

echo "[ollama_recover] restarting ollama service..."
systemctl restart ollama || true

sleep 8

echo "[ollama_recover] checking Ollama at 127.0.0.1:11434..."
curl -s http://127.0.0.1:11434/api/tags || true
echo

echo "[ollama_recover] ollama ps:"
ollama ps || true

echo "[ollama_recover] memory:"
free -h || true
