import math
import sys
import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.apps.run_shared_nav_semantic_explore import SharedNavSemanticExplore
from src.vlm.qwen_ollama_client import QwenOllamaClient


def test_is_wall_like_no_path_reason():
    node = object.__new__(SharedNavSemanticExplore)
    node.wall_recovery_reason_keywords = [
        "wall_like",
        "no_floor",
        "no_traversable_path",
        "blocked_view",
    ]
    assert node._is_wall_like_no_path_reason("wall_like_no_floor")
    assert node._is_wall_like_no_path_reason("blocked_view_close")
    assert not node._is_wall_like_no_path_reason("safe corridor visible")


def test_wall_recovery_cmd_finishes_by_yaw():
    node = object.__new__(SharedNavSemanticExplore)
    node.wall_recovery_active = True
    node.wall_recovery_start_time = 0.0
    node.wall_recovery_start_yaw = 0.0
    node.wall_recovery_target_rad = math.radians(90.0)
    node.wall_recovery_turn_dir = 1.0
    node.wall_recovery_wz = 0.08
    node.wall_recovery_max_sec = 15.0
    node.latest_odom_yaw = math.pi / 2.0
    node.explore_phase = "WALL_ROTATE_90"
    node.explore_phase_start = 0.0
    node.get_logger = lambda: type("L", (), {"info": lambda *a, **k: None})()

    cmd = node._wall_recovery_cmd(1.0)
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0
    assert node.wall_recovery_active is False
    assert node.explore_phase == "EXPLORE_SELECT"


def test_ollama_path_prompt_wall_rules():
    client = object.__new__(QwenOllamaClient)
    client.coord_mode = "pixel"
    prompt = client._build_mode_prompt(
        "bottle",
        640,
        480,
        640,
        480,
        task_mode="path",
    )
    assert "wall_like_no_floor" in prompt
    assert "GATE_STATE_SAFE" in prompt
    assert "GATE_FLOOR" in prompt
    assert "visual gates" in prompt.lower()
    assert "mode=NONE" in prompt
