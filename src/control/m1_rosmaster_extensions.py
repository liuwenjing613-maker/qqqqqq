#!/usr/bin/env python3
"""M1 Rosmaster runtime extensions and startup sanitize.

Rosmaster_Lib 3.3.9 does not ship set_speed_limit / set_imu_adjust. This module
adds those helpers via the expansion-board serial protocol and applies the same
sanitize sequence used by debug_tools/m1_runtime_sanitize.py.
"""

from __future__ import annotations

import struct
import time
from typing import Any, Callable, Dict, Optional

_HEAD = 0xFF
_DEVICE_ID = 0xFC
_COMPLEMENT = 257 - _DEVICE_ID
_DELAY_SEC = 0.002

# Inferred from Yahboom FUNC sequence after 0x15 (SET_CAR_TYPE).
_FUNC_SET_SPEED_LIMIT = 0x16
_FUNC_SET_IMU_ADJUST = 0x17
_FUNC_SET_YAW_PID = 0x14


def _write_cmd(bot: Any, func: int, payload: list[int], forever: bool = False) -> None:
    state = 0x5F if forever else 0
    cmd = [_HEAD, _DEVICE_ID, 0x00, int(func) & 0xFF, *payload, state]
    cmd[2] = len(cmd) - 1
    checksum = sum(cmd, _COMPLEMENT) & 0xFF
    cmd.append(checksum)
    bot.ser.write(cmd)
    time.sleep(_DELAY_SEC)


def _set_yaw_pid_param(bot: Any, kp: float, ki: float, kd: float, forever: bool = False) -> None:
    if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
        raise ValueError("yaw PID must be in [0, 10]")
    kp_params = bytearray(struct.pack("h", int(kp * 1000)))
    ki_params = bytearray(struct.pack("h", int(ki * 1000)))
    kd_params = bytearray(struct.pack("h", int(kd * 1000)))
    _write_cmd(
        bot,
        _FUNC_SET_YAW_PID,
        [
            kp_params[0],
            kp_params[1],
            ki_params[0],
            ki_params[1],
            kd_params[0],
            kd_params[1],
        ],
        forever=forever,
    )


def set_speed_limit(bot: Any, vx_limit: float, wz_limit: float, forever: bool = False) -> None:
    """Clear/set MCU minimum speed thresholds (m/s, rad/s)."""
    vx_params = bytearray(struct.pack("h", int(float(vx_limit) * 1000)))
    wz_params = bytearray(struct.pack("h", int(float(wz_limit) * 1000)))
    _write_cmd(
        bot,
        _FUNC_SET_SPEED_LIMIT,
        [vx_params[0], vx_params[1], wz_params[0], wz_params[1]],
        forever=forever,
    )


def set_imu_adjust(bot: Any, enable: bool, forever: bool = False) -> None:
    """Enable/disable MCU IMU-assisted motion correction."""
    _write_cmd(bot, _FUNC_SET_IMU_ADJUST, [1 if enable else 0], forever=forever)


def ensure_m1_extensions(bot: Any) -> None:
    """Attach missing helpers to a Rosmaster instance if needed."""
    if not hasattr(bot, "set_speed_limit"):
        bot.set_speed_limit = lambda vx, wz, forever=False: set_speed_limit(bot, vx, wz, forever)
    if not hasattr(bot, "set_imu_adjust"):
        bot.set_imu_adjust = lambda enable, forever=False: set_imu_adjust(bot, enable, forever)
    if not hasattr(bot, "set_yaw_pid_param"):
        bot.set_yaw_pid_param = lambda kp, ki, kd, forever=False: _set_yaw_pid_param(
            bot, kp, ki, kd, forever
        )


def _call_step(
    name: str,
    fn: Callable[[], Any],
    log: Optional[Callable[[str], None]] = None,
) -> Dict[str, Any]:
    try:
        ret = fn()
        msg = f"[OK] {name} -> {ret!r}"
        if log:
            log(msg)
        return {"ok": True, "result": ret, "error": ""}
    except Exception as exc:
        msg = f"[ERR] {name}: {exc!r}"
        if log:
            log(msg)
        return {"ok": False, "result": None, "error": repr(exc)}


def apply_m1_runtime_sanitize(
    bot: Any,
    *,
    pid_kp: float = 1.2,
    pid_ki: float = 0.05,
    pid_kd: float = 0.02,
    speed_limit_vx: float = 0.0,
    speed_limit_wz: float = 0.0,
    imu_adjust: bool = False,
    yaw_pid_zero: bool = True,
    write_flash: bool = False,
    settle_sec: float = 0.8,
    log: Optional[Callable[[str], None]] = None,
) -> Dict[str, Any]:
    """Apply M1 MCU runtime sanitize; safe to call at chassis-bridge startup."""
    ensure_m1_extensions(bot)
    forever = bool(write_flash)

    if log:
        log("===== M1 runtime sanitize (chassis) =====")

    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(0.4)

    steps: Dict[str, Any] = {}
    steps["version"] = _call_step("get_version", bot.get_version, log)
    steps["pid_before"] = _call_step("get_motion_pid", bot.get_motion_pid, log)
    steps["car_type"] = _call_step("get_car_type_from_machine", bot.get_car_type_from_machine, log)
    steps["auto_report"] = _call_step(
        "set_auto_report_state",
        lambda: bot.set_auto_report_state(True, forever),
        log,
    )
    steps["clear_auto_report"] = _call_step("clear_auto_report_data", bot.clear_auto_report_data, log)
    steps["speed_limit"] = _call_step(
        "set_speed_limit",
        lambda: bot.set_speed_limit(speed_limit_vx, speed_limit_wz, forever),
        log,
    )
    steps["imu_adjust"] = _call_step(
        "set_imu_adjust",
        lambda: bot.set_imu_adjust(imu_adjust, forever),
        log,
    )
    if yaw_pid_zero and not imu_adjust:
        steps["yaw_pid_zero"] = _call_step(
            "set_yaw_pid_param",
            lambda: bot.set_yaw_pid_param(0.0, 0.0, 0.0, forever),
            log,
        )
    steps["pid_set"] = _call_step(
        "set_pid_param",
        lambda: bot.set_pid_param(pid_kp, pid_ki, pid_kd, forever),
        log,
    )

    time.sleep(max(0.0, float(settle_sec)))
    steps["pid_after"] = _call_step("get_motion_pid", bot.get_motion_pid, log)

    bot.set_car_motion(0.0, 0.0, 0.0)
    steps["all_ok"] = all(v.get("ok", False) for k, v in steps.items() if k != "all_ok")

    if log:
        log(f"===== M1 runtime sanitize done (all_ok={steps['all_ok']}) =====")

    return steps
