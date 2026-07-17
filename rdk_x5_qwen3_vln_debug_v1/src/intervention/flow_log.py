"""Dedicated, low-noise flow log for third-view intervention.

Writes a single human-readable stream of:
  CHECK   → condition progress (what is met / not met)
  TRIGGER → decision that starts MAP transfer
  PHASE   → handshake state machine transitions
  HANDSHAKE / NAV / MUX → companion nodes

Default file: <project>/logs/third_view_flow.log
Override with env THIRD_VIEW_FLOW_LOG.
"""

from __future__ import annotations

import os
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional, TextIO, Union


_LOCK = threading.Lock()
_DEFAULT_NAME = "third_view_flow.log"


def default_flow_log_path(project_root: Union[str, Path]) -> Path:
    override = os.environ.get("THIRD_VIEW_FLOW_LOG", "").strip()
    if override:
        return Path(override).expanduser()
    return Path(project_root) / "logs" / _DEFAULT_NAME


class FlowLogger:
    """Append-only flow logger shared by intervention / bridge / mux."""

    def __init__(
        self,
        path: Union[str, Path],
        *,
        also_stdout: bool = True,
        source: str = "intervention",
    ):
        self.path = Path(path)
        self.also_stdout = bool(also_stdout)
        self.source = str(source)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh: Optional[TextIO] = None
        with _LOCK:
            if not self.path.exists() or self.path.stat().st_size == 0:
                self._write_unlocked(
                    "====",
                    f"第三视角流程日志开始 | source={self.source} | file={self.path}",
                )

    def close(self) -> None:
        with _LOCK:
            if self._fh is not None:
                try:
                    self._fh.close()
                except Exception:  # noqa: BLE001
                    pass
                self._fh = None

    def event(self, tag: str, message: str) -> None:
        with _LOCK:
            self._write_unlocked(tag, message)

    def section(self, title: str) -> None:
        self.event("====", title)

    def _write_unlocked(self, tag: str, message: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"{ts} | {tag:<10} | {message}"
        try:
            if self._fh is None or self._fh.closed:
                self._fh = open(self.path, "a", encoding="utf-8")
            self._fh.write(line + "\n")
            self._fh.flush()
        except Exception:  # noqa: BLE001
            pass
        if self.also_stdout:
            try:
                print(f"[第三视角] {line}", flush=True)
            except Exception:  # noqa: BLE001
                pass


def mark(ok: bool) -> str:
    return "✓" if ok else "✗"


def fmt_check(name: str, ok: bool, detail: str) -> str:
    return f"{name}{mark(ok)}({detail})"
