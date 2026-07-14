from __future__ import annotations

import json
import os
from pathlib import Path
from string import Template
from typing import Dict, Optional

from .types import PromptMode


_JSON_TEMPLATE = '{"s":"V|I|S|F","a":"POINT|TURN_LEFT|TURN_RIGHT","p":[0,0]|null}'
_DEFAULT_PROMPT_DIR = Path(__file__).resolve().parents[2] / "prompts"


class PromptManager:
    """Load short state-specific prompts from editable text files."""

    def __init__(self, prompt_dir: Optional[str] = None):
        selected = (
            prompt_dir
            or os.getenv("QWEN_PROMPT_DIR", "")
            or str(_DEFAULT_PROMPT_DIR)
        )
        self.prompt_dir = Path(selected).expanduser().resolve()
        filenames: Dict[PromptMode, str] = {
            PromptMode.OBSERVE: "observe.txt",
            PromptMode.TRACK: "track.txt",
            PromptMode.SEARCH: "search.txt",
            PromptMode.VERIFY: "verify.txt",
        }
        self.common = self._read("common.txt")
        self.mode_templates = {
            mode: self._read(name) for mode, name in filenames.items()
        }

    def _read(self, filename: str) -> Template:
        path = self.prompt_dir / filename
        if not path.is_file():
            raise FileNotFoundError(f"Prompt file not found: {path}")
        return Template(path.read_text(encoding="utf-8").strip())

    def build(
        self,
        mode: PromptMode,
        instruction: str,
        image_width: int,
        image_height: int,
        previous_point: str = "none",
    ) -> str:
        common = self.common.safe_substitute(
            instruction_json=json.dumps(instruction, ensure_ascii=False),
            width=image_width,
            height=image_height,
            # Pixel extremes are only negative examples. Model output remains 0..1000.
            max_x=image_width - 1,
            max_y=image_height - 1,
            json_template=_JSON_TEMPLATE,
        )
        mode_text = self.mode_templates[mode].safe_substitute(
            previous_point=previous_point
        )
        return common + "\n\n" + mode_text
