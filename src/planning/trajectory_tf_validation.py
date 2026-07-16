#!/usr/bin/env python3
"""Pure trajectory TF validation — no ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple


TF_OK = "TF_OK"
TF_AGE_WARNING = "TF_AGE_WARNING"
TRAJECTORY_TF_WARMING_UP = "TRAJECTORY_TF_WARMING_UP"
TRAJECTORY_TF_LOOKUP_FAILED = "TRAJECTORY_TF_LOOKUP_FAILED"
TRAJECTORY_TF_STALE = "TRAJECTORY_TF_STALE"
TRAJECTORY_TF_CLOCK_SKEW = "TRAJECTORY_TF_CLOCK_SKEW"
TRAJECTORY_TF_NONFINITE = "TRAJECTORY_TF_NONFINITE"
TRAJECTORY_TF_MISSING = "TRAJECTORY_TF_MISSING"


def trajectory_tf_cfg(cfg: Dict[str, Any]) -> Dict[str, float]:
    tcfg = cfg.get("trajectory", {})
    return {
        "tf_lookup_timeout_s": float(tcfg.get("tf_lookup_timeout_s", 0.30)),
        "tf_warmup_s": float(tcfg.get("tf_warmup_s", 2.0)),
        "tf_warn_age_s": float(tcfg.get("tf_warn_age_s", 0.50)),
        "max_tf_age_s": float(tcfg.get("max_tf_age_s", 1.50)),
        "max_clock_future_skew_s": float(tcfg.get("max_clock_future_skew_s", 0.10)),
        "require_consecutive_valid_tf": int(tcfg.get("require_consecutive_valid_tf", 2)),
    }


def validate_trajectory_tf_config(cfg: Dict[str, Any]) -> list[str]:
    errors: list[str] = []
    tcfg = trajectory_tf_cfg(cfg)
    if tcfg["tf_lookup_timeout_s"] <= 0:
        errors.append("trajectory.tf_lookup_timeout_s must be > 0")
    if tcfg["tf_warmup_s"] < 0:
        errors.append("trajectory.tf_warmup_s must be >= 0")
    if tcfg["tf_warn_age_s"] < 0:
        errors.append("trajectory.tf_warn_age_s must be >= 0")
    if tcfg["max_tf_age_s"] <= 0:
        errors.append("trajectory.max_tf_age_s must be > 0")
    if tcfg["tf_warn_age_s"] > tcfg["max_tf_age_s"]:
        errors.append("trajectory.tf_warn_age_s must be <= trajectory.max_tf_age_s")
    if tcfg["max_clock_future_skew_s"] < 0:
        errors.append("trajectory.max_clock_future_skew_s must be >= 0")
    if tcfg["require_consecutive_valid_tf"] < 1:
        errors.append("trajectory.require_consecutive_valid_tf must be >= 1")
    return errors


def stamp_to_ns(stamp_sec: int, stamp_nanosec: int) -> int:
    return int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)


def compute_tf_age_s(now_ns: int, stamp_sec: int, stamp_nanosec: int) -> float:
    """Compute TF age using ROS clock nanoseconds (now - tf_stamp)."""
    tf_ns = stamp_to_ns(stamp_sec, stamp_nanosec)
    return (int(now_ns) - tf_ns) / 1e9


def compute_tf_age_s_from_stamp_sec(now_ns: int, tf_stamp_sec: float) -> float:
    tf_ns = int(round(float(tf_stamp_sec) * 1e9))
    return (int(now_ns) - tf_ns) / 1e9


def classify_lookup_exception(exception_text: str) -> str:
    msg = exception_text.lower()
    if "extrapolation into the future" in msg:
        return "extrapolation into the future"
    if "lookup would require extrapolation" in msg:
        return "lookup would require extrapolation"
    if "frame does not exist" in msg:
        return "frame does not exist"
    if "timeout" in msg:
        return "lookup timeout"
    return exception_text


@dataclass
class TrajectoryTfEvaluation:
    status: str
    valid: bool
    rejection_reason: str
    tf_stamp_sec: float
    tf_age_s: float
    exception_text: str = ""
    warning: bool = False


@dataclass
class TrajectoryTfTracker:
    cfg: Dict[str, Any]
    consecutive_valid_count: int = 0
    consecutive_invalid_count: int = 0
    latest_tf_stamp_sec: float = 0.0
    latest_tf_age_s: float = 0.0
    latest_tf_status: str = "INIT"
    latest_tf_exception: str = ""
    odom_tf_available_for_diagnostics: bool = False

    @property
    def tf_params(self) -> Dict[str, float]:
        return trajectory_tf_cfg(self.cfg)

    @property
    def trajectory_tf_ready(self) -> bool:
        req = int(self.tf_params["require_consecutive_valid_tf"])
        return self.consecutive_valid_count >= req

    def warmup_remaining_s(self, node_uptime_s: float) -> float:
        return max(0.0, self.tf_params["tf_warmup_s"] - float(node_uptime_s))

    def evaluate_lookup_failure(
        self,
        *,
        node_uptime_s: float,
        exception_text: str,
    ) -> TrajectoryTfEvaluation:
        params = self.tf_params
        self.latest_tf_exception = exception_text

        if node_uptime_s < params["tf_warmup_s"]:
            self.latest_tf_status = TRAJECTORY_TF_WARMING_UP
            return TrajectoryTfEvaluation(
                status=TRAJECTORY_TF_WARMING_UP,
                valid=False,
                rejection_reason=TRAJECTORY_TF_WARMING_UP,
                tf_stamp_sec=0.0,
                tf_age_s=0.0,
                exception_text=exception_text,
            )

        self.consecutive_valid_count = 0
        self.consecutive_invalid_count += 1
        self.latest_tf_status = TRAJECTORY_TF_LOOKUP_FAILED
        return TrajectoryTfEvaluation(
            status=TRAJECTORY_TF_LOOKUP_FAILED,
            valid=False,
            rejection_reason=TRAJECTORY_TF_LOOKUP_FAILED,
            tf_stamp_sec=0.0,
            tf_age_s=0.0,
            exception_text=exception_text,
        )

    def evaluate_transform(
        self,
        *,
        now_ns: int,
        node_uptime_s: float,
        stamp_sec: int,
        stamp_nanosec: int,
        x: float,
        y: float,
        yaw_rad: float,
    ) -> TrajectoryTfEvaluation:
        params = self.tf_params
        tf_stamp_sec = float(stamp_sec) + float(stamp_nanosec) * 1e-9
        raw_age_s = compute_tf_age_s(now_ns, stamp_sec, stamp_nanosec)

        self.latest_tf_stamp_sec = tf_stamp_sec
        self.latest_tf_exception = ""

        if node_uptime_s < params["tf_warmup_s"]:
            self.latest_tf_age_s = max(0.0, raw_age_s)
            self.latest_tf_status = TRAJECTORY_TF_WARMING_UP
            return TrajectoryTfEvaluation(
                status=TRAJECTORY_TF_WARMING_UP,
                valid=False,
                rejection_reason=TRAJECTORY_TF_WARMING_UP,
                tf_stamp_sec=tf_stamp_sec,
                tf_age_s=max(0.0, raw_age_s),
            )

        if raw_age_s < -params["max_clock_future_skew_s"]:
            self.consecutive_valid_count = 0
            self.consecutive_invalid_count += 1
            self.latest_tf_age_s = raw_age_s
            self.latest_tf_status = TRAJECTORY_TF_CLOCK_SKEW
            return TrajectoryTfEvaluation(
                status=TRAJECTORY_TF_CLOCK_SKEW,
                valid=False,
                rejection_reason=TRAJECTORY_TF_CLOCK_SKEW,
                tf_stamp_sec=tf_stamp_sec,
                tf_age_s=raw_age_s,
            )

        tf_age_s = max(0.0, raw_age_s)
        self.latest_tf_age_s = tf_age_s

        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(yaw_rad):
            self.consecutive_valid_count = 0
            self.consecutive_invalid_count += 1
            self.latest_tf_status = TRAJECTORY_TF_NONFINITE
            return TrajectoryTfEvaluation(
                status=TRAJECTORY_TF_NONFINITE,
                valid=False,
                rejection_reason=TRAJECTORY_TF_NONFINITE,
                tf_stamp_sec=tf_stamp_sec,
                tf_age_s=tf_age_s,
            )

        if tf_age_s > params["max_tf_age_s"]:
            self.consecutive_valid_count = 0
            self.consecutive_invalid_count += 1
            self.latest_tf_status = TRAJECTORY_TF_STALE
            return TrajectoryTfEvaluation(
                status=TRAJECTORY_TF_STALE,
                valid=False,
                rejection_reason=TRAJECTORY_TF_STALE,
                tf_stamp_sec=tf_stamp_sec,
                tf_age_s=tf_age_s,
            )

        warning = tf_age_s > params["tf_warn_age_s"]
        status = TF_AGE_WARNING if warning else TF_OK
        self.consecutive_valid_count += 1
        self.consecutive_invalid_count = 0
        self.latest_tf_status = status
        return TrajectoryTfEvaluation(
            status=status,
            valid=True,
            rejection_reason="",
            tf_stamp_sec=tf_stamp_sec,
            tf_age_s=tf_age_s,
            warning=warning,
        )

    def evaluate_missing(self, *, node_uptime_s: float) -> TrajectoryTfEvaluation:
        if node_uptime_s < self.tf_params["tf_warmup_s"]:
            self.latest_tf_status = TRAJECTORY_TF_WARMING_UP
            return TrajectoryTfEvaluation(
                status=TRAJECTORY_TF_WARMING_UP,
                valid=False,
                rejection_reason=TRAJECTORY_TF_WARMING_UP,
                tf_stamp_sec=0.0,
                tf_age_s=0.0,
            )
        self.consecutive_valid_count = 0
        self.consecutive_invalid_count += 1
        self.latest_tf_status = TRAJECTORY_TF_MISSING
        self.latest_tf_exception = ""
        return TrajectoryTfEvaluation(
            status=TRAJECTORY_TF_MISSING,
            valid=False,
            rejection_reason=TRAJECTORY_TF_MISSING,
            tf_stamp_sec=0.0,
            tf_age_s=0.0,
        )

    def diagnostics_payload(self, node_uptime_s: float) -> Dict[str, Any]:
        return {
            "trajectory_tf_ready": self.trajectory_tf_ready,
            "trajectory_tf_warmup_remaining_s": round(self.warmup_remaining_s(node_uptime_s), 3),
            "trajectory_consecutive_valid_count": self.consecutive_valid_count,
            "trajectory_consecutive_invalid_count": self.consecutive_invalid_count,
            "latest_tf_stamp_sec": round(self.latest_tf_stamp_sec, 6),
            "latest_tf_age_s": round(self.latest_tf_age_s, 6),
            "latest_tf_status": self.latest_tf_status,
            "latest_tf_exception": self.latest_tf_exception,
            "odom_tf_available_for_diagnostics": self.odom_tf_available_for_diagnostics,
        }
