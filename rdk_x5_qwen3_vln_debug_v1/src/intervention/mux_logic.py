"""Pure velocity-source selection for the EGO/MAP/HOLD mux."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MuxDecision:
    source: str  # ZERO / EGO / MAP
    effective_mode: str
    reason: str


def choose_source(
    *,
    requested_mode: str,
    mode_age_sec: float,
    ego_age_sec: Optional[float],
    map_age_sec: Optional[float],
    safety_age_sec: Optional[float],
    emergency_reverse_active: bool,
    mode_timeout_sec: float,
    ego_timeout_sec: float,
    map_timeout_sec: float,
    safety_status_timeout_sec: float,
    safety_override_enabled: bool,
    require_fresh_safety_status_in_map: bool,
) -> MuxDecision:
    mode = requested_mode.strip().upper()
    if mode_age_sec > mode_timeout_sec:
        return MuxDecision("ZERO", "HOLD", "mode_stale")

    if mode == "MAP":
        if require_fresh_safety_status_in_map and (
            safety_age_sec is None
            or safety_age_sec > safety_status_timeout_sec
        ):
            return MuxDecision("ZERO", "HOLD", "map_safety_status_stale")
        if safety_override_enabled and emergency_reverse_active:
            if ego_age_sec is not None and ego_age_sec <= ego_timeout_sec:
                return MuxDecision(
                    "EGO", "EGO_SAFETY", "emergency_reverse_override"
                )
            return MuxDecision(
                "ZERO", "HOLD", "emergency_reverse_ego_stale"
            )
        if map_age_sec is not None and map_age_sec <= map_timeout_sec:
            return MuxDecision("MAP", "MAP", "selected")
        return MuxDecision("ZERO", "HOLD", "map_cmd_stale")

    if mode == "EGO":
        if ego_age_sec is not None and ego_age_sec <= ego_timeout_sec:
            return MuxDecision("EGO", "EGO", "selected")
        return MuxDecision("ZERO", "HOLD", "ego_cmd_stale")

    if mode == "HOLD":
        return MuxDecision("ZERO", "HOLD", "explicit_hold")

    return MuxDecision("ZERO", "HOLD", "invalid_mode")
