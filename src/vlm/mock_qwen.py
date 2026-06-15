#!/usr/bin/env python3
import time
import json
import sys


def mock_qwen_parse(instruction: str):
    """
    Mock Qwen:
    将自然语言任务转换为结构化语义任务。
    当前主目标：red backpack。
    后续真实 Qwen 接入时，保持这个 JSON 输出格式不变。
    """
    text = instruction.lower().strip()

    if "red backpack" in text or "红色背包" in text:
        # YOLO 节点 texts 只用 offline_vocabulary 内词，避免 embedding 异常。
        prompts = ["backpack", "handbag", "suitcase"]
        target_classes = ["backpack", "handbag", "suitcase"]
        target_category = "backpack"
        target_description = "red backpack"

    elif "backpack" in text or "背包" in text:
        prompts = ["backpack", "handbag", "suitcase"]
        target_classes = ["backpack", "handbag", "suitcase"]
        target_category = "backpack"
        target_description = "backpack"

    elif "bag" in text or "包" in text:
        prompts = ["handbag", "backpack", "suitcase"]
        target_classes = ["handbag", "backpack", "suitcase"]
        target_category = "bag"
        target_description = "bag"

    elif "green cup" in text or "绿色水杯" in text or "绿色杯子" in text:
        prompts = ["bottle", "cup", "wine glass"]
        target_classes = ["bottle", "cup", "wine glass"]
        target_category = "cup"
        target_description = "green cup"

    elif "red cup" in text or "红色杯子" in text:
        prompts = ["cup", "bottle", "wine glass"]
        target_classes = ["cup", "bottle", "wine glass"]
        target_category = "cup"
        target_description = "red cup"

    elif "cup" in text or "杯子" in text or "水杯" in text:
        prompts = ["cup", "bottle", "wine glass"]
        target_classes = ["cup", "bottle", "wine glass"]
        target_category = "cup"
        target_description = "cup"

    elif "bottle" in text or "瓶" in text:
        prompts = ["bottle", "cup", "wine glass"]
        target_classes = ["bottle", "cup", "wine glass"]
        target_category = "bottle"
        target_description = "bottle"

    elif "book" in text or "书" in text:
        prompts = ["book"]
        target_classes = ["book"]
        target_category = "book"
        target_description = "book"

    elif "box" in text or "盒" in text or "箱" in text:
        prompts = ["box", "carton"]
        target_classes = ["box", "carton"]
        target_category = "box"
        target_description = "box"

    else:
        prompts = ["backpack", "handbag", "suitcase"]
        target_classes = ["backpack", "handbag", "suitcase"]
        target_category = "backpack"
        target_description = "red backpack"

    return {
        "timestamp": time.time(),
        "instruction": instruction,
        "target_category": target_category,
        "target_description": target_description,
        "possible_yolo_world_prompts": prompts,
        "target_classes": target_classes,
        "target_visible": None,
        "semantic_region": None,
        "search_direction": None,
        "confidence": 0.90,
        "need_replan": False,
        "action_hint": "approach_slowly",
        "reason": "Mock Qwen output for red backpack MVP semantic-to-vision bridge."
    }


if __name__ == "__main__":
    instruction = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "find the red backpack"
    print(json.dumps(mock_qwen_parse(instruction), indent=2, ensure_ascii=False))
