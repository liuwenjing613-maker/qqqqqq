import sys
import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.apps.run_shared_nav_semantic_explore import SharedNavSemanticExplore


def _make_node(**overrides):
    node = object.__new__(SharedNavSemanticExplore)
    node.explore_lock_goal_while_moving = True
    node.explore_allow_goal_switch_only_when_near = True
    node.explore_goal_switch_max_distance_m = 0.45
    node.explore_goal_switch_min_dist_m = 0.45
    node.explore_map_frame = "map"
    node.explore_phase = "EXPLORE_STEP"
    node.active_explore_goal = {"candidate_id": "cand_a", "score": 0.5}
    node.active_planned_path = []
    node.active_planned_path_frame = None
    node.active_path_waypoint_idx = -1
    node.get_logger = lambda: type("L", (), {"info": lambda *a, **k: None})()
    for key, val in overrides.items():
        setattr(node, key, val)
    return node


def test_explore_moving_phase_includes_step():
    node = _make_node()
    assert node._is_explore_moving_phase() is True
    node.explore_phase = "EXPLORE_SELECT"
    assert node._is_explore_moving_phase() is False


def test_goal_switch_only_when_near():
    node = _make_node()
    prev = {"candidate_id": "a", "goal_pose": [1.0, 2.0]}
    fresh = {"candidate_id": "b", "goal_pose": [3.0, 4.0]}
    assert node._explore_goal_switch_allowed(prev, fresh, 0.3) is True
    assert node._explore_goal_switch_allowed(prev, fresh, 1.5) is False


def test_merge_same_candidate_updates_path():
    node = _make_node()
    prev = {"candidate_id": "cand_a", "score": 0.4, "planned_path": [[0, 0], [1, 0]]}
    fresh = {
        "candidate_id": "cand_a",
        "score": 0.9,
        "planned_path": [[0, 0], [1, 0], [2, 0]],
    }
    node._merge_same_candidate_goal(prev, fresh)
    assert node.active_explore_goal["score"] == 0.9
    assert len(node.active_planned_path) == 3
