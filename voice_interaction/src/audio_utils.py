from __future__ import annotations

import math
import os
import shutil
import subprocess
import wave
from pathlib import Path
from typing import Optional

import pyaudio

# USB microphone (C-Media, ALSA card 0 / PyAudio index 0).
DEFAULT_DEVICE_INDEX = int(os.getenv("VOICE_DEVICE_INDEX", "0") or "0")
DEFAULT_ALSA_CARD = int(os.getenv("VOICE_ALSA_CARD", "0") or "0")

SAMPLE_RATE = 16000
CAPTURE_CHANNELS = 2
OUTPUT_CHANNELS = 1
CHUNK_FRAMES = 3200
SAMPLE_FORMAT = pyaudio.paInt16


class AudioCaptureError(RuntimeError):
    """Raised when the microphone cannot be opened or audio cannot be recorded."""


def list_input_devices() -> list[dict]:
    audio = pyaudio.PyAudio()
    devices: list[dict] = []
    try:
        for index in range(audio.get_device_count()):
            info = audio.get_device_info_by_index(index)
            if int(info.get("maxInputChannels", 0)) <= 0:
                continue
            devices.append(
                {
                    "index": index,
                    "name": str(info.get("name", "unknown")),
                    "max_input_channels": int(info.get("maxInputChannels", 0)),
                    "default_sample_rate": int(float(info.get("defaultSampleRate", 0))),
                }
            )
    finally:
        audio.terminate()
    return devices


def resolve_device_index(device_index: Optional[int]) -> int:
    if device_index is None:
        return DEFAULT_DEVICE_INDEX
    return int(device_index)


def _alsa_device_name(device_index: Optional[int]) -> str:
    if device_index is None:
        return f"plughw:{DEFAULT_ALSA_CARD},0"
    return f"plughw:{int(device_index)},0"


def _trim_wav(wav_path: Path, seconds: float, sample_rate: int) -> None:
    target_frames = int(sample_rate * seconds)
    with wave.open(str(wav_path), "rb") as wav_file:
        params = wav_file.getparams()
        frames = wav_file.readframes(min(wav_file.getnframes(), target_frames))
    with wave.open(str(wav_path), "wb") as wav_file:
        wav_file.setparams(params)
        wav_file.writeframes(frames)


def _record_with_arecord(
    output_path: Path,
    seconds: float,
    device_index: Optional[int],
    sample_rate: int,
) -> Path:
    if shutil.which("arecord") is None:
        raise AudioCaptureError("arecord is not installed.")

    duration_sec = max(1, math.ceil(seconds))
    command = [
        "arecord",
        "-q",
        "-D",
        _alsa_device_name(device_index),
        "-f",
        "S16_LE",
        "-r",
        str(sample_rate),
        "-c",
        "1",
        "-d",
        str(duration_sec),
        str(output_path),
    ]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        stderr = (result.stderr or "").strip() or "unknown error"
        raise AudioCaptureError(f"arecord failed: {stderr}")
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise AudioCaptureError("arecord did not produce an audio file.")
    _trim_wav(output_path, seconds, sample_rate)
    return output_path


def _stereo_to_mono(stereo_samples: list[int]) -> list[int]:
    if not stereo_samples:
        return []
    if len(stereo_samples) < 2:
        return stereo_samples
    mono: list[int] = []
    for i in range(0, len(stereo_samples) - 1, 2):
        mono.append(int((stereo_samples[i] + stereo_samples[i + 1]) / 2))
    return mono


def _record_with_pyaudio(
    output_path: Path,
    seconds: float,
    device_index: Optional[int],
    sample_rate: int,
) -> Path:
    resolved_index = resolve_device_index(device_index)
    audio = pyaudio.PyAudio()
    stream = None
    frames: list[bytes] = []
    channels = 1
    sample_width = audio.get_sample_size(SAMPLE_FORMAT)
    try:
        info = audio.get_device_info_by_index(resolved_index)
        channels = max(1, min(2, int(info.get("maxInputChannels", 1))))
        try:
            stream = audio.open(
                format=SAMPLE_FORMAT,
                channels=channels,
                rate=sample_rate,
                input=True,
                input_device_index=resolved_index,
                frames_per_buffer=CHUNK_FRAMES,
            )
            total_chunks = max(1, int(sample_rate / CHUNK_FRAMES * seconds))
            for _ in range(total_chunks):
                frames.append(stream.read(CHUNK_FRAMES, exception_on_overflow=False))
        except OSError as exc:
            raise AudioCaptureError(
                f"Cannot open or read the microphone: {exc}. "
                "Run list_audio_devices.py and choose a valid input device index."
            ) from exc
    finally:
        if stream is not None:
            stream.stop_stream()
            stream.close()
        audio.terminate()

    stereo_samples = [
        int.from_bytes(raw[i : i + 2], "little", signed=True)
        for raw in frames
        for i in range(0, len(raw), 2)
    ]
    if channels >= 2:
        mono_samples = _stereo_to_mono(stereo_samples)
    else:
        mono_samples = stereo_samples
    mono_frames = b"".join(
        sample.to_bytes(2, "little", signed=True) for sample in mono_samples
    )

    with wave.open(str(output_path), "wb") as wav_file:
        wav_file.setnchannels(OUTPUT_CHANNELS)
        wav_file.setsampwidth(sample_width)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(mono_frames)
    return output_path


def record_wav(
    output_path: str | Path,
    seconds: float = 5.0,
    device_index: Optional[int] = None,
    sample_rate: int = SAMPLE_RATE,
) -> Path:
    if seconds <= 0:
        raise ValueError("seconds must be greater than 0")

    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    try:
        return _record_with_arecord(output, seconds, device_index, sample_rate)
    except AudioCaptureError:
        return _record_with_pyaudio(output, seconds, device_index, sample_rate)
