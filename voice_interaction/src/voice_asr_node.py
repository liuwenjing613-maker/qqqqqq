from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from audio_utils import record_wav
from qwen_asr_client import transcribe_file


class VoiceAsrNode(Node):
    def __init__(self) -> None:
        super().__init__("voice_asr_node")
        self.publisher = self.create_publisher(String, "/voice/text", 10)
        self.device_index = self._optional_int(
            os.getenv("VOICE_DEVICE_INDEX")
        )
        self.record_seconds = float(
            os.getenv("VOICE_RECORD_SECONDS", "5")
        )
        self.output_dir = Path(
            os.getenv("VOICE_RECORD_DIR", "recordings")
        ).expanduser()

        self.get_logger().info(
            "Ready. Press Enter to record, or type q then Enter to quit."
        )

    @staticmethod
    def _optional_int(value: str | None) -> int | None:
        if value is None or not value.strip():
            return None
        return int(value)

    def capture_transcribe_publish(self) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = self.output_dir / f"utterance_{timestamp}.wav"

        self.get_logger().info(
            f"Recording {self.record_seconds:.1f} seconds..."
        )
        wav_path = record_wav(
            output_path=output_path,
            seconds=self.record_seconds,
            device_index=self.device_index,
        )
        self.get_logger().info(f"Audio saved: {wav_path}")

        text = transcribe_file(wav_path)
        message = String()
        message.data = text
        self.publisher.publish(message)
        self.get_logger().info(f"Recognized and published: {text}")


def main() -> int:
    rclpy.init()
    node = VoiceAsrNode()
    try:
        while rclpy.ok():
            command = input(
                "\n[Enter=record | q=quit] > "
            ).strip().lower()
            if command == "q":
                break
            try:
                node.capture_transcribe_publish()
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception as exc:
                node.get_logger().error(str(exc))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
