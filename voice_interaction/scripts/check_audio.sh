#!/usr/bin/env bash
set -euo pipefail

echo "========== USB devices =========="
lsusb || true

echo
echo "========== ALSA capture devices =========="
arecord -l || true

echo
echo "========== ALSA capture PCMs =========="
arecord -L || true

echo
echo "========== ALSA playback devices =========="
aplay -l || true
