from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

from audio_utils import record_wav
from qwen_asr_client import transcribe_file


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record one utterance and transcribe it with Qwen ASR."
    )
    parser.add_argument("--device", type=int, default=None)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--output-dir", default="recordings")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = Path(args.output_dir) / f"utterance_{timestamp}.wav"

    print(f"[VOICE] Recording for {args.seconds:.1f} seconds...")
    wav_path = record_wav(
        output_path=output_path,
        seconds=args.seconds,
        device_index=args.device,
    )
    print(f"[VOICE] Audio saved: {wav_path}")
    print("[ASR] Sending audio to Qwen ASR...")
    text = transcribe_file(wav_path)
    print(f"[ASR] Recognized text: {text}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
