#!/usr/bin/env python3
"""Append transient component logs into one stable, tagged main log."""
from __future__ import annotations

import argparse
from pathlib import Path
import signal
import time
from typing import Dict, Tuple

running = True


def stop(*_: object) -> None:
    global running
    running = False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--source", action="append", default=[], help="TAG=PATH")
    parser.add_argument("--poll", type=float, default=0.10)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    sources: Dict[str, Path] = {}
    for item in args.source:
        if "=" not in item:
            raise SystemExit(f"invalid --source {item!r}; expected TAG=PATH")
        tag, path = item.split("=", 1)
        sources[tag.strip()] = Path(path)
    states: Dict[str, Tuple[int, int]] = {tag: (0, 0) for tag in sources}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    while running:
        with output.open("a", encoding="utf-8", errors="replace") as sink:
            for tag, path in sources.items():
                if not path.exists() or not path.is_file():
                    continue
                try:
                    stat = path.stat()
                    inode, offset = states[tag]
                    if inode != stat.st_ino or stat.st_size < offset:
                        inode, offset = stat.st_ino, 0
                    if stat.st_size <= offset:
                        states[tag] = (inode, offset)
                        continue
                    with path.open("r", encoding="utf-8", errors="replace") as src:
                        src.seek(offset)
                        for line in src:
                            stamp = time.strftime("%H:%M:%S")
                            sink.write(f"[{stamp}][{tag}] {line}")
                        offset = src.tell()
                    states[tag] = (inode, offset)
                except (OSError, ValueError):
                    continue
            sink.flush()
        time.sleep(max(0.03, args.poll))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
