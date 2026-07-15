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

# USB microphone (C-Media, ALSA card 0 / PyAudio index 0).
export VOICE_DEVICE_INDEX="${VOICE_DEVICE_INDEX:-0}"
export VOICE_ALSA_DEVICE="${VOICE_ALSA_DEVICE:-plughw:0,0}"
# shellcheck source=scripts/lib/setup_usb_mic.sh
source "${ROOT_DIR}/scripts/lib/setup_usb_mic.sh"
setup_usb_mic

python3 src/voice_asr_node.py
