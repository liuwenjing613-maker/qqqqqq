#!/usr/bin/env python3
import argparse
import csv
import json
import os
import statistics
import time
from datetime import datetime
from Rosmaster_Lib import Rosmaster


def parse_float_list(s):
    return [float(x.strip()) for x in s.split(",") if x.strip()]


def stop_robot(bot, repeat=8, interval=0.06):
    for _ in range(repeat):
        try:
            bot.set_car_motion(0.0, 0.0, 0.0)
        except Exception:
            pass
        time.sleep(interval)


def get_motion(bot):
    try:
        data = bot.get_motion_data()
        if data is None or len(data) < 3:
            return 0.0, 0.0, 0.0
        return float(data[0]), float(data[1]), float(data[2])
    except Exception:
        return 0.0, 0.0, 0.0


class RosDebug:
    def __init__(self, enabled):
        self.enabled = False
        self.rclpy = None
        self.node = None
        self.cmd_pub = None
        self.fb_pub = None
        self.state_pub = None
        self.Twist = None
        self.String = None

        if not enabled:
            return

        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from std_msgs.msg import String

            rclpy.init(args=None)
            self.rclpy = rclpy
            self.Twist = Twist
            self.String = String
            self.node = rclpy.create_node("m1_speed_calibration")
            self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel_sent", 10)
            self.fb_pub = self.node.create_publisher(Twist, "/m1_motion_feedback", 10)
            self.state_pub = self.node.create_publisher(String, "/m1_calib_state", 10)
            self.enabled = True
            print("[ROS] publish: /cmd_vel_sent /m1_motion_feedback /m1_calib_state")
        except Exception as e:
            print("[WARN] ROS debug disabled:", repr(e))

    def publish(self, cmd_vx, cmd_wz, fb_vx, fb_vy, fb_wz, state_dict):
        if not self.enabled:
            return

        cmd = self.Twist()
        cmd.linear.x = float(cmd_vx)
        cmd.angular.z = float(cmd_wz)
        self.cmd_pub.publish(cmd)

        fb = self.Twist()
        fb.linear.x = float(fb_vx)
        fb.linear.y = float(fb_vy)
        fb.angular.z = float(fb_wz)
        self.fb_pub.publish(fb)

        msg = self.String()
        msg.data = json.dumps(state_dict, ensure_ascii=False)
        self.state_pub.publish(msg)

        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self):
        if self.enabled:
            try:
                self.node.destroy_node()
                self.rclpy.shutdown()
            except Exception:
                pass


def run_phase(bot, ros, samples, phase_info, cmd_vx, cmd_wz, duration, rate_hz):
    dt = 1.0 / rate_hz
    start = time.time()
    next_t = start

    while True:
        now = time.time()
        elapsed = now - start
        if elapsed >= duration:
            break

        bot.set_car_motion(float(cmd_vx), 0.0, float(cmd_wz))
        fb_vx, fb_vy, fb_wz = get_motion(bot)

        row = {
            "time": now,
            "elapsed": elapsed,
            "test_type": phase_info["test_type"],
            "phase": phase_info["phase"],
            "axis": phase_info["axis"],
            "direction": phase_info["direction"],
            "target_speed": phase_info["target_speed"],
            "cmd_vx": cmd_vx,
            "cmd_wz": cmd_wz,
            "fb_vx": fb_vx,
            "fb_vy": fb_vy,
            "fb_wz": fb_wz,
        }
        samples.append(row)

        ros.publish(cmd_vx, cmd_wz, fb_vx, fb_vy, fb_wz, row)

        next_t += dt
        sleep_time = next_t - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)


def analyze_samples(samples, axis, direction, detect_thresh, peak_thresh, active_ratio_thresh):
    if not samples:
        return {
            "moved": False,
            "mean_signed": 0.0,
            "mean_abs": 0.0,
            "peak_abs": 0.0,
            "active_ratio": 0.0,
        }

    half = max(1, len(samples) // 2)
    use = samples[half:]

    if axis == "vx":
        vals = [float(x["fb_vx"]) for x in use]
    else:
        vals = [float(x["fb_wz"]) for x in use]

    signed = [direction * v for v in vals]
    abs_vals = [abs(v) for v in vals]

    mean_signed = statistics.mean(signed) if signed else 0.0
    mean_abs = statistics.mean(abs_vals) if abs_vals else 0.0
    peak_abs = max(abs_vals) if abs_vals else 0.0
    active_ratio = sum(1 for v in signed if v >= detect_thresh) / max(1, len(signed))

    moved = (
        mean_signed >= detect_thresh
        or peak_abs >= peak_thresh
        or active_ratio >= active_ratio_thresh
    )

    return {
        "moved": bool(moved),
        "mean_signed": float(mean_signed),
        "mean_abs": float(mean_abs),
        "peak_abs": float(peak_abs),
        "active_ratio": float(active_ratio),
    }


def run_breakaway(bot, ros, all_samples, axis, direction, speed, args):
    stop_robot(bot)
    time.sleep(args.rest_sec)

    cmd_vx = direction * speed if axis == "vx" else 0.0
    cmd_wz = direction * speed if axis == "wz" else 0.0

    before = len(all_samples)

    phase_info = {
        "test_type": "breakaway",
        "phase": "target_from_rest",
        "axis": axis,
        "direction": direction,
        "target_speed": speed,
    }

    run_phase(
        bot=bot,
        ros=ros,
        samples=all_samples,
        phase_info=phase_info,
        cmd_vx=cmd_vx,
        cmd_wz=cmd_wz,
        duration=args.duration,
        rate_hz=args.rate,
    )

    phase_samples = all_samples[before:]
    stop_robot(bot)
    time.sleep(args.rest_sec)

    if axis == "vx":
        detect = args.vx_detect_thresh
        peak = max(0.010, detect * 1.8)
    else:
        detect = args.wz_detect_thresh
        peak = max(0.040, detect * 1.8)

    a = analyze_samples(
        samples=phase_samples,
        axis=axis,
        direction=direction,
        detect_thresh=detect,
        peak_thresh=peak,
        active_ratio_thresh=0.35,
    )

    result = {
        "test_type": "breakaway",
        "axis": axis,
        "direction": direction,
        "target_speed": speed,
        "sample_count": len(phase_samples),
    }
    result.update(a)
    return result


def run_hold(bot, ros, all_samples, axis, direction, speed, args):
    stop_robot(bot)
    time.sleep(args.rest_sec)

    if axis == "vx":
        kick_vx = direction * max(args.kick_vx, speed)
        kick_wz = 0.0
        hold_vx = direction * speed
        hold_wz = 0.0
    else:
        kick_vx = 0.0
        kick_wz = direction * max(args.kick_wz, speed)
        hold_vx = 0.0
        hold_wz = direction * speed

    kick_info = {
        "test_type": "hold",
        "phase": "kick",
        "axis": axis,
        "direction": direction,
        "target_speed": speed,
    }

    run_phase(
        bot=bot,
        ros=ros,
        samples=all_samples,
        phase_info=kick_info,
        cmd_vx=kick_vx,
        cmd_wz=kick_wz,
        duration=args.kick_duration,
        rate_hz=args.rate,
    )

    before = len(all_samples)

    hold_info = {
        "test_type": "hold",
        "phase": "hold_after_kick",
        "axis": axis,
        "direction": direction,
        "target_speed": speed,
    }

    run_phase(
        bot=bot,
        ros=ros,
        samples=all_samples,
        phase_info=hold_info,
        cmd_vx=hold_vx,
        cmd_wz=hold_wz,
        duration=args.duration,
        rate_hz=args.rate,
    )

    phase_samples = all_samples[before:]
    stop_robot(bot)
    time.sleep(args.rest_sec)

    if axis == "vx":
        detect = args.vx_detect_thresh
        peak = max(0.008, detect * 1.5)
    else:
        detect = args.wz_detect_thresh
        peak = max(0.030, detect * 1.5)

    a = analyze_samples(
        samples=phase_samples,
        axis=axis,
        direction=direction,
        detect_thresh=detect,
        peak_thresh=peak,
        active_ratio_thresh=0.50,
    )

    result = {
        "test_type": "hold",
        "axis": axis,
        "direction": direction,
        "target_speed": speed,
        "sample_count": len(phase_samples),
    }
    result.update(a)
    return result


def write_csv(path, rows):
    if not rows:
        return
    keys = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def first_moved(results, test_type, axis, direction=1):
    vals = [
        r["target_speed"]
        for r in results
        if r["test_type"] == test_type
        and r["axis"] == axis
        and r["direction"] == direction
        and r["moved"]
    ]
    return min(vals) if vals else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--mode", choices=["linear", "angular", "all"], default="all")
    parser.add_argument("--directions", choices=["pos", "both"], default="pos")
    parser.add_argument("--vx-speeds", default="0.005,0.008,0.010,0.015,0.020,0.030,0.040,0.050,0.060")
    parser.add_argument("--wz-speeds", default="0.030,0.050,0.080,0.100,0.120,0.160,0.200,0.240")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=25.0)
    parser.add_argument("--rest-sec", type=float, default=0.8)
    parser.add_argument("--kick-vx", type=float, default=0.055)
    parser.add_argument("--kick-wz", type=float, default=0.240)
    parser.add_argument("--kick-duration", type=float, default=0.055)
    parser.add_argument("--vx-detect-thresh", type=float, default=0.006)
    parser.add_argument("--wz-detect-thresh", type=float, default=0.025)
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--yes-i-have-space", action="store_true")
    parser.add_argument("--out-dir", default="/root/rdk_x5_vln_robot/debug_tools/logs")
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

    vx_speeds = parse_float_list(args.vx_speeds)
    wz_speeds = parse_float_list(args.wz_speeds)
    directions = [1] if args.directions == "pos" else [1, -1]

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    sample_csv = os.path.join(args.out_dir, f"m1_speed_calib_{ts}_samples.csv")
    result_csv = os.path.join(args.out_dir, f"m1_speed_calib_{ts}_results.csv")
    summary_json = os.path.join(args.out_dir, f"m1_speed_calib_{ts}_summary.json")

    print("[INFO] port:", args.port)
    print("[INFO] vx_speeds:", vx_speeds)
    print("[INFO] wz_speeds:", wz_speeds)
    print("[INFO] output:", args.out_dir)

    samples = []
    results = []
    ros = RosDebug(args.ros)

    bot = Rosmaster(com=args.port)
    bot.create_receive_threading()
    time.sleep(0.5)

    try:
        try:
            bot.set_auto_report_state(True, forever=False)
            time.sleep(0.2)
        except Exception as e:
            print("[WARN] set_auto_report_state failed:", repr(e))

        try:
            print("[INFO] battery:", bot.get_battery_voltage())
        except Exception:
            pass

        try:
            print("[INFO] motion_pid:", bot.get_motion_pid())
        except Exception:
                    pass

        stop_robot(bot)

        if args.mode in ("linear", "all"):
            for d in directions:
                print(f"\n===== LINEAR vx direction={d} breakaway =====")
                for speed in vx_speeds:
                    r = run_breakaway(bot, ros, samples, "vx", d, speed, args)
                    results.append(r)
                    print(
                        f"breakaway vx={d*speed:+.4f}: "
                        f"{'YES' if r['moved'] else 'NO'} "
                        f"mean={r['mean_signed']:.4f} "
                        f"peak={r['peak_abs']:.4f} "
                        f"active={r['active_ratio']:.2f}"
                    )

                print(f"\n===== LINEAR vx direction={d} hold =====")
                for speed in vx_speeds:
                    r = run_hold(bot, ros, samples, "vx", d, speed, args)
                    results.append(r)
                    print(
                        f"hold vx={d*speed:+.4f}: "
                        f"{'YES' if r['moved'] else 'NO'} "
                        f"mean={r['mean_signed']:.4f} "
                        f"peak={r['peak_abs']:.4f} "
                        f"active={r['active_ratio']:.2f}"
                    )

        if args.mode in ("angular", "all"):
            for d in directions:
                print(f"\n===== ANGULAR wz direction={d} breakaway =====")
                for speed in wz_speeds:
                    r = run_breakaway(bot, ros, samples, "wz", d, speed, args)
                    results.append(r)
                    print(
                        f"breakaway wz={d*speed:+.4f}: "
                        f"{'YES' if r['moved'] else 'NO'} "
                        f"mean={r['mean_signed']:.4f} "
                        f"peak={r['peak_abs']:.4f} "
                        f"active={r['active_ratio']:.2f}"
                    )

                print(f"\n===== ANGULAR wz direction={d} hold =====")
                for speed in wz_speeds:
                    r = run_hold(bot, ros, samples, "wz", d, speed, args)
                    results.append(r)
                    print(
                        f"hold wz={d*speed:+.4f}: "
                        f"{'YES' if r['moved'] else 'NO'} "
                        f"mean={r['mean_signed']:.4f} "
                        f"peak={r['peak_abs']:.4f} "
                        f"active={r['active_ratio']:.2f}"
                    )

    except KeyboardInterrupt:
        print("\n[WARN] interrupted, stopping robot")
    finally:
        stop_robot(bot)
        ros.close()

    write_csv(sample_csv, samples)
    write_csv(result_csv, results)

    summary = {
        "timestamp": ts,
        "port": args.port,
        "vx_breakaway": first_moved(results, "breakaway", "vx", 1),
        "vx_hold_min": first_moved(results, "hold", "vx", 1),
        "wz_breakaway": first_moved(results, "breakaway", "wz", 1),
        "wz_hold_min": first_moved(results, "hold", "wz", 1),
        "sample_csv": sample_csv,
        "result_csv": result_csv,
    }

    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("\n===== SUMMARY =====")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("[OK] sample_csv:", sample_csv)
    print("[OK] result_csv:", result_csv)
    print("[OK] summary_json:", summary_json)


if __name__ == "__main__":
    main()
