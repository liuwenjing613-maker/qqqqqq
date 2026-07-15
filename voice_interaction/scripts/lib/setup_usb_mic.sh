#!/usr/bin/env bash
# Configure USB microphone gain (C-Media, ALSA card 0).
# Important: set capture only; never enable Mic playback (monitor/loopback).
setup_usb_mic() {
    local capture_level="${VOICE_USB_MIC_LEVEL:-30}"
    amixer -q -c 0 set 'Mic' playback off
    amixer -q -c 0 set 'Mic' capture "${capture_level}" cap on
    # AGC can cause sudden loud spikes; keep off unless you need it.
    amixer -q -c 0 set 'Auto Gain Control' off
}
