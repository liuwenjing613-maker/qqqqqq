#!/usr/bin/env python3
"""Prompt templates for Qwen text reasoning."""

INSTRUCTION_PARSE_PROMPT = """你是机器人语义导航指令解析器。你只能返回 JSON。
根据用户指令解析目标类别和同义词别名。
输出格式:
{{"target_category":"...", "target_aliases":["..."], "context_objects":["..."], "success_description":"...", "confidence":0.0}}

用户指令：{instruction}
"""

CANDIDATE_RERANK_PROMPT = """你是机器人语义导航候选选择器。你只能返回 JSON。
你不能输出速度，不能判断任务成功，不能选择 unsafe 候选。
请根据用户指令、语义地图摘要、候选探索点，选择最推荐候选。
如果候选都不安全，返回 best_candidate_id=null。

输出格式:
{{"best_candidate_id":"cand_001", "confidence":0.0, "reason":"..."}}

用户指令：{instruction}
目标别名：{target_aliases}
语义摘要：{semantic_summary}
候选列表：{candidates}
"""
