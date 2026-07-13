#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ -f ".env" ]]; then
    set -a
    source ".env"
    set +a
fi

python3 src/voice_asr_node.py
