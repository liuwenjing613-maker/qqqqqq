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
            # Keep the existing filename spelling used by the user.
            PromptMode.SPAWN_SCAN: "spawn_sacn.txt",
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

    @staticmethod
    def _strip_hash_comment_lines(text: str) -> str:
        """Drop full-line '#' comments used to archive prior prompt revisions.

        Keeps recovery text in the .txt file without sending it to the model.
        """
        kept = [
            line
            for line in text.splitlines()
            if not line.lstrip().startswith("#")
        ]
        return "\n".join(kept).strip()

    def build(
        self,
        mode: PromptMode,
        instruction: str,
        image_width: int,
        image_height: int,
        previous_point: str = "none",
    ) -> str:
        instruction_json = json.dumps(instruction, ensure_ascii=False)
        # Spawn scan uses its own standalone prompt (no common.txt mix-in).
        if mode == PromptMode.SPAWN_SCAN:
            # Archived (commented) prior SPAWN_SCAN revisions must not be sent.
            active = self._strip_hash_comment_lines(
                self.mode_templates[mode].template
            )
            return Template(active).safe_substitute(
                instruction_json=instruction_json,
            )

        # image_width/height stay in the signature for callers, but are not
        # injected into the prompt (relative [0,1000] only).
        _ = (image_width, image_height)
        common = self.common.safe_substitute(
            instruction_json=instruction_json,
            json_template=_JSON_TEMPLATE,
        )
        mode_text = self.mode_templates[mode].safe_substitute(
            previous_point=previous_point
        )
        return common + "\n\n" + mode_text
