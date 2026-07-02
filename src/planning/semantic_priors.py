#!/usr/bin/env python3
"""Rule-based semantic priors for explore goal selection."""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class ParsedInstruction:
    raw: str
    target_category: str
    target_aliases: List[str]
    context_objects: Dict[str, float]
    confidence: float = 1.0
    source: str = "rules"


_RULE_PATTERNS: List[tuple] = [
    (r"cup|mug|drink|drinking|喝水|水杯", "cup", ["cup", "bottle", "mug"]),
    (r"bottle|瓶", "bottle", ["bottle", "cup"]),
    (r"bag|backpack|包|背包", "backpack", ["backpack", "bag"]),
    (r"book|书", "book", ["book"]),
    (r"chair|椅|坐", "chair", ["chair", "couch"]),
    (r"plant|植物|绿植", "plant", ["potted plant", "plant"]),
    (r"phone|手机", "phone", ["cell phone"]),
]


def _section(raw: Dict[str, Any], key: str) -> Dict[str, Any]:
    block = raw.get(key, {})
    return block if isinstance(block, dict) else {}


def _normalize_class(name: str) -> str:
    return str(name).strip().lower().replace("_", " ")


def _match_aliases(class_name: str, aliases: List[str]) -> bool:
    norm = _normalize_class(class_name)
    for alias in aliases:
        a = _normalize_class(alias)
        if norm == a or a in norm or norm in a:
            return True
    return False


def load_semantic_priors_config(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return _section(cfg, "semantic_priors")


def parse_instruction_by_rules(
    instruction: str,
    priors_cfg: Optional[Dict[str, Any]] = None,
    fallback_classes: Optional[List[str]] = None,
    fallback_words: Optional[List[str]] = None,
) -> ParsedInstruction:
    text = (instruction or "").strip().lower()
    priors_cfg = priors_cfg or {}
    fallback_classes = fallback_classes or []
    fallback_words = fallback_words or []

    for pattern, category, aliases in _RULE_PATTERNS:
        if re.search(pattern, text, re.IGNORECASE):
            block = priors_cfg.get(category, {})
            if isinstance(block, dict) and block.get("target_aliases"):
                aliases = [str(x) for x in block["target_aliases"]]
            ctx = {}
            if isinstance(block, dict):
                ctx = {str(k): float(v) for k, v in (block.get("context_objects") or {}).items()}
            return ParsedInstruction(
                raw=instruction,
                target_category=category,
                target_aliases=aliases,
                context_objects=ctx,
                confidence=0.9,
                source="rules",
            )

    # Extract quoted or last noun-like token
    words = re.findall(r"[a-zA-Z\u4e00-\u9fff]+", text)
    target_word = words[-1] if words else "target"
    aliases = list(fallback_classes or fallback_words or [target_word])
    block = priors_cfg.get(target_word, priors_cfg.get("default", {}))
    if isinstance(block, dict) and block.get("target_aliases"):
        aliases = [str(x) for x in block["target_aliases"]]
    ctx = {}
    if isinstance(block, dict):
        ctx = {str(k): float(v) for k, v in (block.get("context_objects") or {}).items()}
    return ParsedInstruction(
        raw=instruction,
        target_category=target_word,
        target_aliases=aliases,
        context_objects=ctx,
        confidence=0.6,
        source="fallback",
    )


def get_target_aliases(target: str, priors_cfg: Optional[Dict[str, Any]] = None) -> List[str]:
    priors_cfg = priors_cfg or {}
    block = priors_cfg.get(target, {})
    if isinstance(block, dict) and block.get("target_aliases"):
        return [str(x) for x in block["target_aliases"]]
    return [target]


def context_score(target: str, object_class: str, priors_cfg: Optional[Dict[str, Any]] = None) -> float:
    priors_cfg = priors_cfg or {}
    parsed = parse_instruction_by_rules(f"find {target}", priors_cfg)
    if parsed.context_objects:
        norm = _normalize_class(object_class)
        best = 0.0
        for ctx_name, weight in parsed.context_objects.items():
            cn = _normalize_class(ctx_name)
            if norm == cn or cn in norm or norm in cn:
                best = max(best, float(weight))
        return best
    block = priors_cfg.get(parsed.target_category, priors_cfg.get(target, {}))
    if isinstance(block, dict):
        for ctx_name, weight in (block.get("context_objects") or {}).items():
            cn = _normalize_class(ctx_name)
            norm = _normalize_class(object_class)
            if norm == cn or cn in norm or norm in cn:
                return float(weight)
    return 0.0


def is_target_match(target_aliases: List[str], class_name: str) -> float:
    if _match_aliases(class_name, target_aliases):
        return 1.0
    return 0.0
