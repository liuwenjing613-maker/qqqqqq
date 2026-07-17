#!/usr/bin/env python3
"""Print only high-value transfer events to the launch terminal."""
from __future__ import annotations

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EventConsole(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("simple_handoff_event_console_v2")
        self.create_subscription(String, topic, self.on_event, 20)

    def on_event(self, msg: String) -> None:
        try:
            p = json.loads(msg.data)
            event = str(p.get("event", "EVENT"))
            phase = str(p.get("phase", "?"))
            text = str(p.get("message", msg.data))
            print(f"[HANDOFF][{event}][{phase}] {text}", flush=True)
        except Exception:
            print(f"[HANDOFF] {msg.data}", flush=True)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/third_view/simple_handoff/event")
    args = parser.parse_args()
    rclpy.init()
    node = EventConsole(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
