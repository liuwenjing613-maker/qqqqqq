#!/usr/bin/env python3
"""Optional Qwen text reasoner via Ollama (default disabled)."""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional

from src.vlm.prompt_templates import CANDIDATE_RERANK_PROMPT, INSTRUCTION_PARSE_PROMPT


class QwenTextReasoner:
    def __init__(self, cfg: Dict[str, Any]):
        self.enabled = bool(cfg.get("enabled", False))
        self.host = str(cfg.get("host", "http://127.0.0.1:11434")).rstrip("/")
        self.model = str(cfg.get("model", "qwen2.5:latest"))
        self.timeout_sec = float(cfg.get("timeout_sec", 4.0))
        self.num_ctx = int(cfg.get("num_ctx", 512))
        self.num_predict = int(cfg.get("num_predict", 96))
        self.temperature = float(cfg.get("temperature", 0.0))
        self.min_confidence = float(cfg.get("min_confidence", 0.55))
        self.use_for_instruction_parse = bool(cfg.get("use_for_instruction_parse", True))
        self.use_for_candidate_rerank = bool(cfg.get("use_for_candidate_rerank", True))
        self.hard_override = bool(cfg.get("hard_override", False))
        self.weight = float(cfg.get("weight", 0.05))

    def _unavailable(self, reason: str) -> Dict[str, Any]:
        return {"enabled": False, "score": 0.0, "reason": reason}

    def _generate(self, prompt: str) -> Dict[str, Any]:
        if not self.enabled:
            return self._unavailable("qwen disabled")
        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": self.temperature,
                "num_ctx": self.num_ctx,
                "num_predict": self.num_predict,
            },
        }
        req = urllib.request.Request(
            f"{self.host}/api/generate",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                body = json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            return self._unavailable(f"qwen unavailable: {exc}")

        text = str(body.get("response", "")).strip()
        try:
            start = text.find("{")
            end = text.rfind("}")
            if start < 0 or end < 0:
                return self._unavailable("non-json")
            parsed = json.loads(text[start : end + 1])
            return {"enabled": True, **parsed}
        except json.JSONDecodeError:
            return self._unavailable("non-json")

    def parse_instruction(self, instruction: str) -> Dict[str, Any]:
        if not self.enabled or not self.use_for_instruction_parse:
            return self._unavailable("qwen disabled")
        result = self._generate(INSTRUCTION_PARSE_PROMPT.format(instruction=instruction))
        conf = float(result.get("confidence", 0.0))
        if not result.get("enabled") or conf < self.min_confidence:
            return self._unavailable("low confidence")
        return result

    def rerank_candidates(
        self,
        instruction: str,
        semantic_summary: Dict[str, Any],
        candidates: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        if not self.enabled or not self.use_for_candidate_rerank:
            return self._unavailable("qwen disabled")
        result = self._generate(
            CANDIDATE_RERANK_PROMPT.format(
                instruction=instruction,
                target_aliases=json.dumps(semantic_summary.get("target_aliases", []), ensure_ascii=False),
                semantic_summary=json.dumps(semantic_summary, ensure_ascii=False),
                candidates=json.dumps(candidates, ensure_ascii=False),
            )
        )
        conf = float(result.get("confidence", 0.0))
        if not result.get("enabled") or conf < self.min_confidence:
            return self._unavailable("low confidence")
        result["score"] = conf * self.weight
        return result
