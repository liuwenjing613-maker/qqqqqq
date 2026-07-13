from __future__ import annotations

import wave
from pathlib import Path
from typing import Optional

import pyaudio


SAMPLE_RATE = 16000
CHANNELS = 1
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

    audio = pyaudio.PyAudio()
    stream = None
    frames: list[bytes] = []

    try:
        stream = audio.open(
            format=SAMPLE_FORMAT,
            channels=CHANNELS,
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
        sample_width = audio.get_sample_size(SAMPLE_FORMAT)
        audio.terminate()

    with wave.open(str(output), "wb") as wav_file:
        wav_file.setnchannels(CHANNELS)
        wav_file.setsampwidth(sample_width)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(b"".join(frames))

    return output
