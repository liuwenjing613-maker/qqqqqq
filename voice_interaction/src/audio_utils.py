from __future__ import annotations

import math
import shutil
import subprocess
import wave
from pathlib import Path
from typing import Optional

import pyaudio


SAMPLE_RATE = 16000
OUTPUT_CHANNELS = 1
# ES8326 on RDK X5 needs stereo capture when using PyAudio directly.
CAPTURE_CHANNELS = 2
SAMPLE_FORMAT = pyaudio.paInt16
CHUNK_FRAMES = 3200


class AudioCaptureError(RuntimeError):
    """Raised when the microphone cannot be opened or audio cannot be recorded."""


def list_input_devices() -> list[dict]:
    audio = pyaudio.PyAudio()
    devices: list[dict] = []
    try:
        for index in range(audio.get_device_count()):
            info = audio.get_device_info_by_index(index)
            if int(info.get("maxInputChannels", 0)) > 0:
                devices.append(
                    {
                        "index": index,
                        "name": str(info.get("name", "unknown")),
                        "max_input_channels": int(info.get("maxInputChannels", 0)),
                        "default_sample_rate": int(
                            float(info.get("defaultSampleRate", 0))
                        ),
                    }
                )
    finally:
        audio.terminate()
    return devices


def _alsa_device_name(device_index: Optional[int]) -> str:
    card = 0 if device_index is None else device_index
    return f"plughw:{card},0"


def _trim_wav(
    wav_path: Path,
    seconds: float,
    sample_rate: int,
) -> None:
    target_frames = int(sample_rate * seconds)
    with wave.open(str(wav_path), "rb") as wav_file:
        params = wav_file.getparams()
        frames = wav_file.readframes(
            min(wav_file.getnframes(), target_frames)
        )

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

    record_seconds = max(1, math.ceil(seconds))
    command = [
        "arecord",
        "-D",
        _alsa_device_name(device_index),
        "-f",
        "S16_LE",
        "-r",
        str(sample_rate),
        "-c",
        "1",
        "-d",
        str(record_seconds),
        str(output_path),
    ]
    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise AudioCaptureError(
            f"arecord failed: {detail or 'unknown error'}"
        )

    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise AudioCaptureError("arecord did not produce an audio file.")

    if record_seconds > seconds:
        _trim_wav(output_path, seconds, sample_rate)

    return output_path


def _stereo_to_mono(stereo_samples: list[int]) -> list[int]:
    mono: list[int] = []
    for index in range(0, len(stereo_samples) - 1, 2):
        left = stereo_samples[index]
        right = stereo_samples[index + 1]
        mono.append(int((left + right) / 2))
    return mono


def _record_with_pyaudio(
    output_path: Path,
    seconds: float,
    device_index: Optional[int],
    sample_rate: int,
) -> Path:
    audio = pyaudio.PyAudio()
    stream = None
    frames: list[bytes] = []
    sample_width = audio.get_sample_size(SAMPLE_FORMAT)

    try:
        stream = audio.open(
            format=SAMPLE_FORMAT,
            channels=CAPTURE_CHANNELS,
            rate=sample_rate,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=CHUNK_FRAMES,
        )
        total_chunks = max(1, int(sample_rate / CHUNK_FRAMES * seconds))
        for _ in range(total_chunks):
            frames.append(
                stream.read(CHUNK_FRAMES, exception_on_overflow=False)
            )
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

    stereo_samples = list(
        int.from_bytes(raw[i : i + 2], "little", signed=True)
        for raw in frames
        for i in range(0, len(raw), 2)
    )
    mono_samples = _stereo_to_mono(stereo_samples)
    mono_frames = b"".join(
        sample.to_bytes(2, "little", signed=True)
        for sample in mono_samples
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
        return _record_with_arecord(
            output_path=output,
            seconds=seconds,
            device_index=device_index,
            sample_rate=sample_rate,
        )
    except AudioCaptureError:
        return _record_with_pyaudio(
            output_path=output,
            seconds=seconds,
            device_index=device_index,
            sample_rate=sample_rate,
        )
