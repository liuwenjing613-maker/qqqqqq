#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "scripts/demo/voice_demo_hub_v1.py"
spec = importlib.util.spec_from_file_location("voice_demo_hub_v1", MODULE)
assert spec and spec.loader
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)

raw = yaml.safe_load((ROOT / "configs/voice_demo_hub_v1.yaml").read_text(encoding="utf-8"))
hub = mod.DemoHub.__new__(mod.DemoHub)
hub.cfg = raw["voice_demo_hub_v1"]
hub.log = lambda _message: None

cases = {
    "开始联网探索": "online",
    "开始联网导航。诶。": "online",
    "请开始断网探索": "offline",
    "完成建图": "finish_mapping",
    "停止机器人": "stop",
    "地图导航": "click_nav",
    "导航": "click_nav",
    "系统状态": "status",
}
for text, expected in cases.items():
    actual = hub.route(text)
    assert actual == expected, (text, actual, expected)
assert hub.route("随便说点什么") is None
assert hub.route("") is None

# Event file restart/truncation recovery: a voice process restart must not strand
# the reader beyond EOF.
with tempfile.TemporaryDirectory() as td:
    event_file = Path(td) / "events.jsonl"
    event_file.write_text(json.dumps({"seq": 1, "event": "wake", "padding": "x" * 80}) + "\n", encoding="utf-8")
    hub.event_file = event_file
    hub.event_offset = 0
    first = hub.read_events()
    assert first[0]["seq"] == 1
    event_file.write_text(json.dumps({"seq": 2, "event": "wake"}) + "\n", encoding="utf-8")
    second = hub.read_events()
    assert second[0]["seq"] == 2

print("[OK] router and event-reader tests passed")
