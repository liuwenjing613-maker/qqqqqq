#!/usr/bin/env python3
"""
M1 minimum controllable speed calibration.

This script directly controls Yahboom Rosmaster M1 via Rosmaster_Lib.
It measures:
  - vx_breakaway: minimum forward speed that starts from rest
  - vx_hold_min: minimum forward speed that can be maintained after a short kick
  - wz_breakaway: minimum angular speed that starts from rest
  - wz_hold_min: minimum angular speed that can be maintained after a short kick

It also optionally publishes:
  - /cmd_vel_sent
  - /m1_motion_feedback
  - /m1_calib_state

Recommended:
  source /opt/tros/humble/setup.bash
  python3 m1_speed_calibration.py --port /dev/myserial --mode all --ros
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from Rosmaster_Lib import Rosmaster


@dataclass
class Sample:
    t: float
    elapsed: float
    test_type: str
    phase: str
    axis: str
    direction: int
    target_speed: float
    cmd_vx: float
    cmd_wz: float
    fb_vx: float
    fb_vy: float
    fb_wz: float


@dataclass
class TrialResult:
    test_type: str
    axis: str
    direction: int
    target_speed: float
    moved: bool
    mean_signed: float
    mean_abs: float
    peak_abs: float
    active_ratio: float
    integral_abs: float
    sample_count: int


class OptionalRosPublisher:
    def __init__(self, enabled: bool):
        self.enabled = False
        self.rclpy = None
        self.node = None
        self.twist_cls = None
        self.string_cls = None
        self.cmd_pub = None
        self.fb_pub = None
        self.state_pub = None

        if not enabled:
            return

        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from std_msgs.msg import String

            rclpy.init(args=None)
            self.node = rclpy.create_node("m1_speed_calibration_debug")
            self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel_sent", 10)
            self.fb_pub = self.node.create_publisher(Twist, "/m1_motion_feedback", 10)
            self.state_pub = self.node.create_publisher(String, "/m1_calib_state", 10)
            self.rclpy = rclpy
            self.twist_cls = Twist
            self.string_cls = String
            self.enabled = True
            print("[ROS] publishing /cmd_vel_sent, /m1_motion_feedback, /m1_calib_state")
        except Exception as exc:
            print(f"[WARN] ROS publish disabled: {exc!r}")

    def publish(self, cmd_vx: float, cmd_wz: float, fb_vx: float, fb_vy: float, fb_wz: float, state: str):
        if not self.enabled:
            return
        cmd = self.twist_cls()
        cmd.linear.x = float(cmd_vx)
        cmd.angular.z = float(cmd_wz)
        self.cmd_pub.publish(cmd)

        fb = self.twist_cls()
        fb.linear.x = float(fb_vx)
        fb.linear.y = float(fb_vy)
        fb.angular.z = float(fb_wz)
        self.fb_pub.publish(fb)

        msg = self.string_cls()
        msg.data = state
        self.state_pub.publish(msg)

        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        if self.enabled and self.rclpy is not None:
            try:
                self.node.destroy_node()
                self.rclpy.shutdown()
            except Exception:
                pass


def parse_float_list(text: str) -> List[float]:
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def safe_get_motion(bot: Rosmaster) -> Tuple[float, float, float]:
    try:
        data = bot.get_motion_data()
        if data is None:
            return 0.0, 0.0, 0.0
        if len(data) < 3:
            return 0.0, 0.0, 0.0
        return float(data[0]), float(data[1]), float(data[2])
    except Exception:
        return 0.0, 0.0, 0.0


def send_motion(bot: Rosmaster, vx: float, wz: float):
    bot.set_car_motion(float(vx), 0.0, float(wz))


def stop_robot(bot: Rosmaster, repeat: int = 5, interval: float = 0.06):
    for _ in range(repeat):
        try:
            send_motion(bot, 0.0, 0.0)
        except Exception:
            pass
        time.sleep(interval)


def collect_phase(
    bot: Rosmaster,
    ros: OptionalRosPublisher,
    samples: List[Sample],
    duration: float,
    rate_hz: float,
    test_type: str,
    phase: str,
    axis: str,
    direction: int,
    target_speed: float,
    cmd_vx: float,
    cmd_wz: float,
):
    dt = 1.0 / rate_hz
    start = time.time()
    next_t = start

    while True:
        now = time.time()
        elapsed = now - start
        if elapsed >= duration:
            break

        send_motion(bot, cmd_vx, cmd_wz)
        fb_vx, fb_vy, fb_wz = safe_get_motion(bot)

        state = json.dumps(
            {
                "test_type": test_type,
                "phase": phase,
                "axis": axis,
                "direction": direction,
                "target_speed": target_speed,
                "cmd_vx": cmd_vx,
                "cmd_wz": cmd_wz,
                "fb_vx": fb_vx,
                "fb_wz": fb_wz,
            },
            ensure_ascii=False,
        )
        ros.publish(cmd_vx, cmd_wz, fb_vx, fb_vy, fb_wz, state)

        samples.append(
            Sample(
                t=now,
                elapsed=elapsed,
                test_type=test_type,
                phase=phase,
                axis=axis,
                direction=direction,
                target_speed=target_speed,
                cmd_vx=cmd_vx,
                cmd_wz=cmd_wz,
                fb_vx=fb_vx,
                fb_vy=fb_vy,
                fb_wz=fb_wz,
            )
        )

        next_t += dt
        sleep_time = next_t - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)


def analyze_trial(
    phase_samples: List[Sample],
    axis: str,
    direction: int,
    detect_thresh: float,
    peak_thresh: float,
    integral_thresh: float,
    min_active_ratio: float,
) -> Tuple[bool, float, float, float, float, float]:
    if not phase_samples:
        return False, 0.0, 0.0, 0.0, 0.0, 0.0

    # Only use the second half, because motor startup is messy. Nature is rude like that.
    half = max(1, len(phase_samples) // 2)
    use_samples = phase_samples[half:]

    if axis == "vx":
        values = [s.fb_vx for s in use_samples]
    else:
        values = [s.fb_wz for s in use_samples]

    signed_values = [direction * v for v in values]
    abs_values = [abs(v) for v in values]

    mean_signed = statistics.mean(signed_values) if signed_values else 0.0
    mean_abs = statistics.mean(abs_values) if abs_values else 0.0
    peak_abs = max(abs_values) if abs_values else 0.0
    active_count = sum(1 for v in signed_values if v >= detect_thresh)
    active_ratio = active_count / max(1, len(signed_values))

    if len(use_samples) >= 2:
        t0 = use_samples[0].t
        integral_abs = 0.0
        prev_t = use_samples[0].t
        for s, value_abs in zip(use_samples[1:], abs_values[1:]):
            dt = max(0.0, s.t - prev_t)
            integral_abs += value_abs * dt
            prev_t = s.t
    else:
        integral_abs = 0.0

    moved = (
        mean_signed >= detect_thresh
        or peak_abs >= peak_thresh
        or integral_abs >= integral_thresh
        or active_ratio >= min_active_ratio
    )

    return moved, mean_signed, mean_abs, peak_abs, active_ratio, integral_abs


def run_breakaway_trial(
    bot: Rosmaster,
    ros: OptionalRosPublisher,
    all_samples: List[Sample],
    axis: str,
    direction: int,
    speed: float,
    duration: float,
    rate_hz: float,
    rest_sec: float,
    vx_detect_thresh: float,
    wz_detect_thresh: float,
) -> TrialResult:
    stop_robot(bot)
    time.sleep(rest_sec)

    cmd_vx = direction * speed if axis == "vx" else 0.0
    cmd_wz = direction * speed if axis == "wz" else 0.0

    before_len = len(all_samples)
    collect_phase(
        bot=bot,
        ros=ros,
        samples=all_samples,
        duration=duration,
        rate_hz=rate_hz,
        test_type="breakaway",
        phase="target_from_rest",
        axis=axis,
        direction=direction,
        target_speed=speed,
        cmd_vx=cmd_vx,
        cmd_wz=cmd_wz,
    )
    phase_samples = all_samples[before_len:]

    stop_robot(bot)
    time.sleep(rest_sec)

    if axis == "vx":
        detect = vx_detect_thresh
        peak = max(vx_detect_thresh * 1.8, 0.010)
        integral = 0.010
    else:
        detect = wz_detect_thresh
        peak = max(wz_detect_thresh * 1.8, 0.040)
        integral = 0.040

    moved, mean_signed, mean_abs, peak_abs, active_ratio, integral_abs = analyze_trial(
        phase_samples,
        axis=axis,
        direction=direction,
        detect_thresh=detect,
        peak_thresh=peak,
        integral_thresh=integral,
        min_active_ratio=0.35,
    )

    return TrialResult(
        test_type="breakaway",
        axis=axis,
        direction=direction,
        target_speed=speed,
        moved=moved,
        mean_signed=mean_signed,
        mean_abs=mean_abs,
        peak_abs=peak_abs,
        active_ratio=active_ratio,
        integral_abs=integral_abs,
        sample_count=len(phase_samples),
    )


def run_hold_trial(
    bot: Rosmaster,
    ros: OptionalRosPublisher,
    all_samples: List[Sample],
    axis: str,
    direction: int,
    speed: float,
    duration: float,
    rate_hz: float,
    rest_sec: float,
    kick_vx: float,
    kick_wz: float,
    kick_duration: float,
    vx_detect_thresh: float,
    wz_detect_thresh: float,
) -> TrialResult:
    stop_robot(bot)
    time.sleep(rest_sec)

    if axis == "vx":
        kick_cmd_vx = direction * max(abs(kick_vx), speed)
        kick_cmd_wz = 0.0
        hold_cmd_vx = direction * speed
        hold_cmd_wz = 0.0
    else:
        kick_cmd_vx = 0.0
        kick_cmd_wz = direction * max(abs(kick_wz), speed)
        hold_cmd_vx = 0.0
        hold_cmd_wz = direction * speed

    collect_phase(
        bot=bot,
        ros=ros,
        samples=all_samples,
        duration=kick_duration,
        rate_hz=rate_hz,
        test_type="hold",
        phase="kick",
        axis=axis,
        direction=direction,
        target_speed=speed,
        cmd_vx=kick_cmd_vx,
        cmd_wz=kick_cmd_wz,
    )

    before_len = len(all_samples)
    collect_phase(
        bot=bot,
        ros=ros,
        samples=all_samples,
        duration=duration,
        rate_hz=rate_hz,
        test_type="hold",
        phase="hold_after_kick",
        axis=axis,
        direction=direction,
        target_speed=speed,
        cmd_vx=hold_cmd_vx,
        cmd_wz=hold_cmd_wz,
    )
    phase_samples = all_samples[before_len:]

    stop_robot(bot)
    time.sleep(rest_sec)

    if axis == "vx":
        detect = vx_detect_thresh
        peak = max(vx_detect_thresh * 1.5, 0.008)
        integral = 0.008
    else:
        detect = wz_detect_thresh
        peak = max(wz_detect_thresh * 1.5, 0.030)
        integral = 0.030

    moved, mean_signed, mean_abs, peak_abs, active_ratio, integral_abs = analyze_trial(
        phase_samples,
        axis=axis,
        direction=direction,
        detect_thresh=detect,
        peak_thresh=peak,
        integral_thresh=integral,
        min_active_ratio=0.50,
    )

    return TrialResult(
        test_type="hold",
        axis=axis,
        direction=direction,
        target_speed=speed,
        moved=moved,
        mean_signed=mean_signed,
        mean_abs=mean_abs,
        peak_abs=peak_abs,
        active_ratio=active_ratio,
        integral_abs=integral_abs,
        sample_count=len(phase_samples),
    )


def write_samples_csv(path: str, samples: List[Sample]):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(samples[0]).keys()) if samples else [])
        if samples:
            writer.writeheader()
            for s in samples:
                writer.writerow(asdict(s))


def write_trials_csv(path: str, trials: List[TrialResult]):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(trials[0]).keys()) if trials else [])
        if trials:
            writer.writeheader()
            for r in trials:
                writer.writerow(asdict(r))


def first_moved(trials: List[TrialResult], test_type: str, axis: str, direction: int) -> Optional[float]:
    candidates = [
        r.target_speed
        for r in trials
        if r.test_type == test_type and r.axis == axis and r.direction == direction and r.moved
    ]
    return min(candidates) if candidates else None


def make_report(summary: Dict, trials: List[TrialResult]) -> str:
    lines = []
    lines.append("# M1 最小可控速度标定报告")
    lines.append("")
    lines.append("## 最终建议值")
    lines.append("")
    lines.append(f"- vx_breakaway: `{summary.get('vx_breakaway')}` m/s")
    lines.append(f"- vx_hold_min: `{summary.get('vx_hold_min')}` m/s")
    lines.append(f"- wz_breakaway: `{summary.get('wz_breakaway')}` rad/s")
    lines.append(f"- wz_hold_min: `{summary.get('wz_hold_min')}` rad/s")
    lines.append("")
    lines.append("## 建议解释")
    lines.append("")
    lines.append("- `breakaway` 是从完全静止直接给目标速度，看它能不能自己动。")
    lines.append("- `hold` 是先给一个极短 kick，再降到目标速度，看它能不能维持。")
    lines.append("- 导航里不要低于 `hold_min` 连续控制；如果想更慢，用短脉冲寸进。")
    lines.append("- 如果 `breakaway` 明显大于 `hold_min`，说明静摩擦启动门槛很高，kick 必须保留但要缩短。")
    lines.append("")
    lines.append("## 每项测试结果")
    lines.append("")
    lines.append("| test | axis | dir | target | moved | mean_signed | peak_abs | active_ratio | integral_abs |")
    lines.append("|---|---|---:|---:|---|---:|---:|---:|---:|")
    for r in trials:
        lines.append(
            f"| {r.test_type} | {r.axis} | {r.direction} | {r.target_speed:.4f} | "
            f"{'YES' if r.moved else 'NO'} | {r.mean_signed:.4f} | {r.peak_abs:.4f} | "
            f"{r.active_ratio:.2f} | {r.integral_abs:.4f} |"
        )
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Measure M1 minimum controllable vx/wz.")
    parser.add_argument("--port", default="/dev/myserial", help="Serial port")
    parser.add_argument("--mode", choices=["linear", "angular", "all"], default="all")
    parser.add_argument("--directions", choices=["pos", "both"], default="pos")
    parser.add_argument("--vx-speeds", default="0.005,0.008,0.010,0.015,0.020,0.030,0.040,0.050,0.060")
    parser.add_argument("--wz-speeds", default="0.030,0.050,0.080,0.100,0.120,0.160,0.200,0.240")
    parser.add_argument("--duration", type=float, default=2.0, help="Duration for each target-speed phase")
    parser.add_argument("--rate", type=float, default=25.0, help="Sampling/control rate")
    parser.add_argument("--rest-sec", type=float, default=0.8, help="Rest time between trials")
    parser.add_argument("--kick-vx", type=float, default=0.055)
    parser.add_argument("--kick-wz", type=float, default=0.240)
    parser.add_argument("--kick-duration", type=float, default=0.055)
    parser.add_argument("--vx-detect-thresh", type=float, default=0.006)
    parser.add_argument("--wz-detect-thresh", type=float, default=0.025)
    parser.add_argument("--ros", action="store_true", help="Publish ROS2 debug topics if rclpy is available")
    parser.add_argument("--out-dir", default="/root/rdk_x5_vln_robot/debug_tools/logs")
    parser.add_argument("--yes-i-have-space", action="store_true", help="Skip safety confirmation")
    args = parser.parse_args()

    if not args.yes_i_have_space:
        print("")
        print("安全确认：")
        print("1. 小车放在平整地面，前方至少 2 米无障碍。")
        print("2. 手能立刻拔电或 Ctrl-C。")
        print("3. 没有其他 ROS 节点正在发底盘命令。")
        print("")
        reply = input("确认后输入 YES 开始：").strip()
        if reply != "YES":
            print("已取消。")
            return

    os.makedirs(args.out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    sample_csv = os.path.join(args.out_dir, f"m1_speed_calib_{ts}_samples.csv")
    directions = [1] if args.directions == "pos" else [1, -1]

    ros = OptionalRosPublisher(args.ros)
    samples: List[Sample] = []
    trials: List[TrialResult] = []

    print(f"[INFO] Connecting Rosmaster on {args.port}")
    bot = Rosmaster(com=args.port)
    bot.create_receive_threading()
    time.sleep(0.5)

    try:
        try:
            bot.set_auto_report_state(True, forever=False)
            time.sleep(0.2)
        except Exception:
            pass

        try:
            pid = bot.get_motion_pid()
            print(f"[INFO] current motion PID: {pid}")
        except Exception as exc:
            pid = None
            print(f"[WARN] cannot read PID: {exc!r}")

        try:
            voltage = bot.get_battery_voltage()
            print(f"[INFO] battery voltage: {voltage}")
        except Exception:
            voltage = None

        stop_robot(bot)

        if args.mode in ("linear", "all"):
            for direction in directions:
                print(f"\n===== LINEAR vx direction={direction} breakaway =====")
                for speed in vx_speeds:
                    r = run_breakaway_trial(
                        bot, ros, samples, "vx", direction, speed,
                        args.duration, args.rate, args.rest_sec,
                        args.vx_detect_thresh, args.wz_detect_thresh
                    )
                    trials.append(r)
                    print(f"breakaway vx={direction*speed:+.4f}: {'YES' if r.moved else 'NO'} "
                          f"mean={r.mean_signed:.4f} peak={r.peak_abs:.4f} active={r.active_ratio:.2f}")

                print(f"\n===== LINEAR vx direction={direction} hold-after-kick =====")
                for speed in vx_speeds:
                    r = run_hold_trial(
                        bot, ros, samples, "vx", direction, speed,
                        args.duration, args.rate, args.rest_sec,
                        args.kick_vx, args.kick_wz, args.kick_duration,
                        args.vx_detect_thresh, args.wz_detect_thresh
                    )
                    trials.append(r)
                    print(f"hold vx={direction*speed:+.4f}: {'YES' if r.moved else 'NO'} "
                          f"mean={r.mean_signed:.4f} peak={r.peak_abs:.4f} active={r.active_ratio:.2f}")

        if args.mode in ("angular", "all"):
            for direction in directions:
                print(f"\n===== ANGULAR wz direction={direction} breakaway =====")
                for speed in wz_speeds:
                    r = run_breakaway_trial(
                        bot, ros, samples, "wz", direction, speed,
                        args.duration, args.rate, args.rest_sec,
                        args.vx_detect_thresh, args.wz_detect_thresh
                    )
                    trials.append(r)
                    print(f"breakaway wz={direction*speed:+.4f}: {'YES' if r.moved else 'NO'} "
                          f"mean={r.mean_signed:.4f} peak={r.peak_abs:.4f} active={r.active_ratio:.2f}")

                print(f"\n===== ANGULAR wz direction={direction} hold-after-kick =====")
                for speed in wz_speeds:
                    r = run_hold_trial(
                        bot, ros, samples, "wz", direction, speed,
                        args.duration, args.rate, args.rest_sec,
                        args.kick_vx, args.kick_wz, args.kick_duration,
                        args.vx_detect_thresh, args.wz_detect_thresh
                    )
                    trials.append(r)
                    print(f"hold wz={direction*speed:+.4f}: {'YES' if r.moved else 'NO'} "
                          f"mean={r.mean_signed:.4f} peak={r.peak_abs:.4f} active={r.active_ratio:.2f}")

    except KeyboardInterrupt:
        print("\n[WARN] KeyboardInterrupt, stopping robot.")
    finally:
        stop_robot(bot, repeat=10, interval=0.05)
        ros.shutdown()

    if samples:
        write_samples_csv(sample_csv, samples)
    if trials:
        write_trials_csv(trial_csv, trials)

    summary = {
        "timestamp": ts,
        "port": args.port,
        "mode": args.mode,
        "directions": args.directions,
        "pid": str(pid),
        "battery_voltage": voltage,
        "vx_breakaway": first_moved(trials, "breakaway", "vx", 1),
        "vx_hold_min": first_moved(trials, "hold", "vx", 1),
        "wz_breakaway": first_moved(trials, "breakaway", "wz", 1),
        "wz_hold_min": first_moved(trials, "hold", "wz", 1),
        "sample_csv": sample_csv,
        "trial_csv": trial_csv,
        "report_md": report_md,
        "thresholds": {
            "vx_detect_thresh": args.vx_detect_thresh,
            "wz_detect_thresh": args.wz_detect_thresh,
        },
        "kick": {
            "kick_vx": args.kick_vx,
            "kick_wz": args.kick_wz,
            "kick_duration": args.kick_duration,
        },
    }

    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    report = make_report(summary, trials)
    with open(report_md, "w", encoding="utf-8") as f:
        f.write(report)

    print("\n===== SUMMARY =====")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("")
    print(f"[OK] samples: {sample_csv}")
    print(f"[OK] trials : {trial_csv}")
    print(f"[OK] summary: {summary_json}")
    print(f"[OK] report : {report_md}")


if __name__ == "__main__":
    main()
