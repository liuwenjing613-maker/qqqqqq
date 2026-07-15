from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from .types import ModelResult, PromptMode, VlnState


@dataclass
class StateMachineConfig:
    observe_interval_sec: float = 2.5
    track_interval_sec: float = 1.6
    search_interval_sec: float = 3.0
    verify_interval_sec: float = 9999.0
    error_cooldown_sec: float = 3.0
    auto_enter_search: bool = True


class NavigationStateMachine:
    """Big-project-style Locked / Inferred / Searching state machine.

    V1 only switches visual prompts. It intentionally has no chassis control.
    """

    def __init__(self, config: StateMachineConfig):
        self.config = config
        self.state = VlnState.WAIT_IMAGE
        self.instruction = ""
        self.has_image = False
        self.last_transition_reason = "initialized"
        self.last_request_time = 0.0
        self.error_since: Optional[float] = None
        self.generation = 0

    def set_instruction(self, instruction: str) -> None:
        cleaned = (instruction or "").strip()
        if not cleaned:
            return
        self.instruction = cleaned
        self.generation += 1
        self._transition(
            VlnState.OBSERVE if self.has_image else VlnState.WAIT_IMAGE,
            "new_instruction" if self.has_image else "waiting_for_image",
        )

    def mark_image_ready(self) -> None:
        self.has_image = True
        if self.state == VlnState.WAIT_IMAGE and self.instruction:
            self._transition(VlnState.OBSERVE, "first_image_ready")

    def command(self, command: str) -> None:
        cmd = (command or "").strip().lower()
        mapping = {
            "observe": VlnState.OBSERVE,
            "search": VlnState.SEARCHING,
            "inferred": VlnState.TARGET_INFERRED,
            "track": VlnState.TARGET_LOCKED,
            "verify": VlnState.VERIFY,
            "pause": VlnState.PAUSED,
            "success": VlnState.SUCCESS,
        }
        if cmd in mapping:
            self._transition(mapping[cmd], f"manual_{cmd}")
        elif cmd in {"resume", "reset"}:
            target = VlnState.OBSERVE if self.has_image and self.instruction else VlnState.WAIT_IMAGE
            self._transition(target, f"manual_{cmd}")
        else:
            raise ValueError(
                "Unknown command. Use observe/search/inferred/track/verify/"
                "pause/success/resume/reset"
            )

    def prompt_mode(self) -> Optional[PromptMode]:
        return {
            VlnState.OBSERVE: PromptMode.OBSERVE,
            VlnState.TARGET_LOCKED: PromptMode.TRACK,
            VlnState.TARGET_INFERRED: PromptMode.SEARCH,
            VlnState.SEARCHING: PromptMode.SEARCH,
            VlnState.VERIFY: PromptMode.VERIFY,
        }.get(self.state)

    def request_interval(self) -> float:
        return {
            VlnState.OBSERVE: self.config.observe_interval_sec,
            VlnState.TARGET_LOCKED: self.config.track_interval_sec,
            VlnState.TARGET_INFERRED: self.config.search_interval_sec,
            VlnState.SEARCHING: self.config.search_interval_sec,
            VlnState.VERIFY: self.config.verify_interval_sec,
        }.get(self.state, 9999.0)

    def should_request(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        if not self.instruction or not self.has_image:
            return False
        if self.state == VlnState.ERROR:
            if self.error_since is not None and now - self.error_since >= self.config.error_cooldown_sec:
                self._transition(VlnState.OBSERVE, "error_cooldown_finished")
            else:
                return False
        if self.prompt_mode() is None:
            return False
        return now - self.last_request_time >= self.request_interval()

    def mark_request_started(self, now: Optional[float] = None) -> None:
        self.last_request_time = time.monotonic() if now is None else now

    def apply_result(
        self,
        result: ModelResult,
        request_mode: PromptMode,
    ) -> None:
        visible = result.result == "TARGET_VISIBLE"

        inferred = result.result in {"TARGET_INFERRED", "VERIFY_FAILED"}

        if request_mode in {
            PromptMode.OBSERVE,
            PromptMode.TRACK,
            PromptMode.SEARCH,
        }:
            if visible:
                self._transition(
                    VlnState.TARGET_LOCKED,
                    "target_visible",
                )
            elif inferred:
                self._transition(
                    VlnState.TARGET_INFERRED,
                    "search_waypoint_available",
                )
            else:
                self._transition(
                    VlnState.SEARCHING,
                    "no_visible_or_inferred_target",
                )
            return

        if request_mode == PromptMode.VERIFY:
            success = result.result == "VERIFY_SUCCESS"

            if success:
                self._transition(
                    VlnState.SUCCESS,
                    "verify_success",
                )
            elif inferred:
                self._transition(
                    VlnState.TARGET_INFERRED,
                    "verify_failed_new_search_waypoint",
                )
            else:
                self._transition(
                    VlnState.SEARCHING,
                    "verify_failed",
                )

    def apply_error(self, reason: str) -> None:
        self.error_since = time.monotonic()
        self._transition(VlnState.ERROR, reason or "api_error")

    def _transition(self, state: VlnState, reason: str) -> None:
        previous_state = self.state
        changed = previous_state != state
        invalidates_inflight = changed or reason.startswith("manual_")
        if invalidates_inflight:
            self.generation += 1
        self.state = state
        self.last_transition_reason = reason
        if state != VlnState.ERROR:
            self.error_since = None

        # A new active state should request immediately. Re-applying a result
        # while remaining in the same state must preserve the interval, or the
        # node would hammer the API in a very expensive little loop.
        same_search_prompt_family = {previous_state, state}.issubset(
            {VlnState.SEARCHING, VlnState.TARGET_INFERRED}
        )
        force_immediate = (
            (changed and not same_search_prompt_family)
            or reason.startswith("manual_")
            or reason in {"new_instruction", "first_image_ready", "error_cooldown_finished"}
        )
        if force_immediate and state in {
            VlnState.OBSERVE,
            VlnState.SEARCHING,
            VlnState.TARGET_INFERRED,
            VlnState.TARGET_LOCKED,
            VlnState.VERIFY,
        }:
            self.last_request_time = 0.0
