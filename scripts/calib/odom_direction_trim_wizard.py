#!/usr/bin/env python3
"""
odom_direction_trim_wizard.py

Interactive ROS2 calibration assistant for Yahboom/Rosmaster M1 odom direction, scale,
and straight-line trim diagnosis.

It publishes /cmd_vel only when you explicitly start an auto drive command.
It records /odom, /cmd_vel, /cmd_vel_sent, and /chassis_bridge_state.

Typical commands:
  static 20
  rot ccw 360 0.08
  done
  line forward 1.0 0.05
  done
  bias left medium
  save
  q

Direction convention:
  ccw/cw are judged from TOP-DOWN view, looking at the robot from above.
  positive ROS angular.z should be CCW top-down.
"""

import argparse
import csv
import json
import math
import os
import queue
import select
import statistics
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from typing import Dict, List, Optional


class HotKeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None
        self.enabled = False

    def start(self):
        if self.enabled:
            return
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.enabled = True

    def stop(self):
        if self.enabled and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        self.enabled = False
        self.old_settings = None

    def read_key(self):
        if not self.enabled:
            return None
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
        return None

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def p95_abs(values: List[float]) -> float:
    if not values:
        return 0.0
    arr = sorted(abs(v) for v in values)
    return arr[int(0.95 * (len(arr) - 1))]


def mean(values: List[float]) -> float:
    return statistics.fmean(values) if values else 0.0


class OdomDirectionTrimWizard(Node):
    def __init__(self, args):
        super().__init__("odom_direction_trim_wizard")
        self.args = args

        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 50)
        self.create_subscription(Twist, args.cmd_topic, self.cmd_cb, 20)
        self.create_subscription(Twist, args.cmd_sent_topic, self.cmd_sent_cb, 20)
        self.create_subscription(String, args.state_topic, self.state_cb, 20)

        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_sent: Optional[Twist] = None
        self.latest_state: Dict = {}

        self.last_yaw_raw: Optional[float] = None
        self.yaw_unwrapped = 0.0

        self.input_q = queue.Queue()
        self.line_buffer = ""
        self.hotkey_reader = HotKeyReader()
        self.results = []
        self.samples_current = []
        self.active = None
        self.baseline = None
        self.auto_cmd: Optional[Twist] = None
        self.auto_name = ""
        self.static_until = None

        os.makedirs(args.out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(args.out_dir, f"direction_trim_records_{stamp}.csv")
        self.json_path = os.path.join(args.out_dir, f"direction_trim_results_{stamp}.json")

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=[
            "time", "active", "auto_name",
            "odom_x", "odom_y", "yaw_unwrapped", "odom_vx", "odom_vy", "odom_wz",
            "cmd_vx", "cmd_wz", "cmd_sent_vx", "cmd_sent_wz",
            "last_sent_vx", "last_sent_wz", "vx_pwm", "wz_pwm",
            "pwm_1", "pwm_2", "pwm_3", "pwm_4", "wheel_layout", "motor_signs", "motor_trims",
        ])
        self.writer.writeheader()

        self.hotkey_reader.start()
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.print_help()
        self.print_prompt()
        self.get_logger().info(f"CSV: {self.csv_path}")

    def print_prompt(self):
        sys.stdout.write("diag> ")
        sys.stdout.flush()

    def shutdown(self):
        self.publish_stop()
        self.hotkey_reader.stop()
        self.save()
        try:
            self.csv_file.close()
        except Exception:
            pass
        rclpy.shutdown()

    def poll_stdin(self):
        while True:
            ch = self.hotkey_reader.read_key()
            if ch is None:
                break
            self.handle_stdin_char(ch)

    def handle_stdin_char(self, ch: str):
        # Hotkeys only when the line buffer is empty (so "done" etc. still work).
        if not self.line_buffer:
            if ch == "d":
                if self.active:
                    self.finish_active_test(settle=True)
                else:
                    print("\n[WARN] no active test.")
                    self.print_prompt()
                return
            if ch == " ":
                self.publish_stop()
                print("\n[STOP] /cmd_vel set to zero.")
                self.print_prompt()
                return
            if ch == "q":
                print("\n[QUIT] save and exit.")
                self.shutdown()
                return
            if ch == "h":
                print()
                self.print_help()
                self.print_prompt()
                return

        if ch in ("\n", "\r"):
            line = self.line_buffer.strip()
            self.line_buffer = ""
            print()
            if line:
                self.input_q.put(line)
            self.print_prompt()
            return

        if ch in ("\x7f", "\b"):
            if self.line_buffer:
                self.line_buffer = self.line_buffer[:-1]
                sys.stdout.write("\b \b")
                sys.stdout.flush()
            return

        if ch == "\x03":
            raise KeyboardInterrupt

        if ch.isprintable():
            self.line_buffer += ch
            sys.stdout.write(ch)
            sys.stdout.flush()

    def print_help(self):
        print("""
================ ODOM DIRECTION + TRIM WIZARD ================
方向判断：ccw/cw 一律按【从小车正上方向下看】判断。
ROS约定：angular.z 为正时，小车应从上往下看逆时针(ccw)旋转。

核心命令：
  static 20
      静止采样20秒，推荐 deadzone。

  rot ccw 360 0.08
  rot cw 360 -0.08
      脚本自动发布旋转 /cmd_vel。真实转到指定角度后输入 done。
      ccw/cw 是你肉眼看到的真实方向；角度是实际角度；最后一个数是命令 wz。

  line forward 1.0 0.05
  line backward 1.0 -0.05
      脚本自动发布直线 /cmd_vel。真实走到指定距离后输入 done。
      forward/backward 是你肉眼看到的真实方向；距离是实际距离；最后一个数是命令 vx。

  done
      结束当前 rot/line/static，并计算结果。输入 done 前脚本会自动先停小车。

  stop
      立刻停止小车，但不结束当前测试。

  bias left small|medium|large
  bias right small|medium|large
      记录真实直线偏向，并给出 motor trim 调整建议。
      left = 车头向左偏；right = 车头向右偏。

  status
      打印当前 odom、pwm、cmd 状态。

  save
      保存 JSON。

  q
      停车、保存、退出。

Hotkeys during test (no Enter needed):
  d = done immediately (stop + settle + compute result)
  SPACE = stop only
  q = save and quit
  h = print help
================================================================
""")

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.last_yaw_raw is None:
            self.last_yaw_raw = yaw
            self.yaw_unwrapped = yaw
        else:
            self.yaw_unwrapped += angle_diff(yaw, self.last_yaw_raw)
            self.last_yaw_raw = yaw

    def cmd_cb(self, msg: Twist):
        self.latest_cmd = msg

    def cmd_sent_cb(self, msg: Twist):
        self.latest_cmd_sent = msg

    def state_cb(self, msg: String):
        try:
            self.latest_state = json.loads(msg.data)
        except Exception:
            self.latest_state = {}

    def snapshot(self):
        if self.latest_odom is None:
            return None
        o = self.latest_odom
        return {
            "time": time.time(),
            "x": o.pose.pose.position.x,
            "y": o.pose.pose.position.y,
            "yaw": self.yaw_unwrapped,
            "vx": o.twist.twist.linear.x,
            "vy": o.twist.twist.linear.y,
            "wz": o.twist.twist.angular.z,
            "state": dict(self.latest_state or {}),
        }

    def publish_stop(self):
        m = Twist()
        self.cmd_pub.publish(m)
        self.auto_cmd = None
        self.auto_name = ""

    def set_auto_cmd(self, vx=0.0, wz=0.0, name=""):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.auto_cmd = m
        self.auto_name = name

    def process_command(self, s: str):
        if not s:
            return
        parts = s.split()
        cmd = parts[0].lower()

        if cmd in ("help", "h"):
            self.print_help(); return
        if cmd in ("q", "quit", "exit"):
            self.shutdown(); return
        if cmd == "save":
            self.save(); return
        if cmd == "stop":
            self.publish_stop(); print("[STOP] /cmd_vel set to zero."); return
        if cmd == "status":
            self.print_status(); return

        if cmd == "static":
            seconds = float(parts[1]) if len(parts) >= 2 else 20.0
            self.publish_stop()
            self.active = {"type": "static", "seconds": seconds}
            self.baseline = self.snapshot()
            self.samples_current = []
            self.static_until = time.time() + seconds
            print(f"[STATIC] sampling {seconds:.1f}s. Do not touch robot.")
            return

        if cmd == "rot":
            if len(parts) < 4:
                print("Usage: rot ccw 360 0.08  OR  rot cw 360 -0.08")
                return
            direction = parts[1].lower()
            if direction not in ("ccw", "cw"):
                print("direction must be ccw or cw, top-down view."); return
            actual_deg = abs(float(parts[2]))
            wz_cmd = float(parts[3])
            snap = self.snapshot()
            if snap is None:
                print("[ERROR] no /odom yet."); return
            self.active = {"type": "rot", "direction": direction, "actual_deg": actual_deg, "wz_cmd": wz_cmd}
            self.baseline = snap
            self.samples_current = []
            self.set_auto_cmd(vx=0.0, wz=wz_cmd, name=f"rot_{direction}_{actual_deg}_{wz_cmd}")
            print(f"[ROT START] actual={direction} {actual_deg}deg, cmd_wz={wz_cmd}. When real angle reached, press d or type done.")
            return

        if cmd == "line":
            if len(parts) < 4:
                print("Usage: line forward 1.0 0.05  OR  line backward 1.0 -0.05")
                return
            direction = parts[1].lower()
            if direction not in ("forward", "backward"):
                print("direction must be forward or backward."); return
            actual_m = abs(float(parts[2]))
            vx_cmd = float(parts[3])
            snap = self.snapshot()
            if snap is None:
                print("[ERROR] no /odom yet."); return
            self.active = {"type": "line", "direction": direction, "actual_m": actual_m, "vx_cmd": vx_cmd}
            self.baseline = snap
            self.samples_current = []
            self.set_auto_cmd(vx=vx_cmd, wz=0.0, name=f"line_{direction}_{actual_m}_{vx_cmd}")
            print(f"[LINE START] actual={direction} {actual_m}m, cmd_vx={vx_cmd}. When real distance reached, press d or type done.")
            return

        if cmd == "done":
            self.finish_active_test(settle=False)
            return

        if cmd == "bias":
            if len(parts) < 3:
                print("Usage: bias left small|medium|large OR bias right small|medium|large")
                return
            side = parts[1].lower(); sev = parts[2].lower()
            self.bias_suggestion(side, sev)
            return

        print("Unknown command. Type help.")

    def finish_active_test(self, settle: bool = False):
        self.publish_stop()
        if settle:
            time.sleep(0.05)
        if not self.active:
            print("[WARN] no active test.")
            return
        if self.active["type"] == "static":
            self.finish_static()
        elif self.active["type"] == "rot":
            self.finish_rot()
        elif self.active["type"] == "line":
            self.finish_line()
        self.active = None
        self.baseline = None
        self.samples_current = []
        self.static_until = None

    def print_status(self):
        snap = self.snapshot()
        st = self.latest_state or {}
        if not snap:
            print("No /odom yet."); return
        print("\n------ STATUS ------")
        print(f"odom: x={snap['x']:.4f}, y={snap['y']:.4f}, yaw={snap['yaw']:.4f}, vx={snap['vx']:.4f}, vy={snap['vy']:.4f}, wz={snap['wz']:.4f}")
        print(f"state: last_sent_vx={st.get('last_sent_vx','')}, last_sent_wz={st.get('last_sent_wz','')}, vx_pwm={st.get('vx_pwm','')}, wz_pwm={st.get('wz_pwm','')}")
        print(f"pwm: {st.get('pwm_1','')}, {st.get('pwm_2','')}, {st.get('pwm_3','')}, {st.get('pwm_4','')}")
        print(f"layout={st.get('wheel_layout','')}, signs={st.get('motor_signs','')}, trims={st.get('motor_trims','')}")
        print("--------------------\n")

    def finish_static(self):
        if not self.samples_current:
            print("[STATIC] no samples."); return
        vx = [s["vx"] for s in self.samples_current]
        vy = [s["vy"] for s in self.samples_current]
        wz = [s["wz"] for s in self.samples_current]
        rec_vxy = max(0.003, 1.5 * max(p95_abs(vx), p95_abs(vy)))
        rec_wz = max(0.015, 1.5 * p95_abs(wz))
        res = {
            "type": "static",
            "samples": len(self.samples_current),
            "p95_abs_vx": p95_abs(vx), "p95_abs_vy": p95_abs(vy), "p95_abs_wz": p95_abs(wz),
            "recommended_CHASSIS_ODOM_VXY_DEADZONE": rec_vxy,
            "recommended_CHASSIS_ODOM_WZ_DEADZONE": rec_wz,
        }
        self.results.append(res)
        print("\n========== STATIC RESULT ==========")
        print(f"samples={res['samples']}")
        print(f"p95 |vx|={res['p95_abs_vx']:.6f}, |vy|={res['p95_abs_vy']:.6f}, |wz|={res['p95_abs_wz']:.6f}")
        print(f"recommended CHASSIS_ODOM_VXY_DEADZONE={rec_vxy:.6f}")
        print(f"recommended CHASSIS_ODOM_WZ_DEADZONE={rec_wz:.6f}")
        print("===================================\n")

    def finish_rot(self):
        b = self.baseline; e = self.snapshot(); cfg = self.active
        if not b or not e:
            print("[ROT] missing baseline/current."); return
        actual_rad = math.radians(abs(cfg["actual_deg"]))
        actual_signed = actual_rad if cfg["direction"] == "ccw" else -actual_rad
        dyaw = e["yaw"] - b["yaw"]
        dx = e["x"] - b["x"]; dy = e["y"] - b["y"]
        scale_signed = actual_signed / dyaw if abs(dyaw) > 1e-9 else None
        scale_abs = abs(actual_signed) / abs(dyaw) if abs(dyaw) > 1e-9 else None
        sign_ok = (actual_signed * dyaw) > 0
        res = {
            "type": "rot",
            "actual_direction": cfg["direction"], "actual_deg": cfg["actual_deg"], "cmd_wz": cfg["wz_cmd"],
            "actual_signed_rad": actual_signed, "odom_yaw_delta_rad": dyaw,
            "translation_drift_by_odom_m": math.hypot(dx, dy), "dx_by_odom_m": dx, "dy_by_odom_m": dy,
            "recommended_CHASSIS_ODOM_WZ_SCALE_SIGNED": scale_signed,
            "recommended_CHASSIS_ODOM_WZ_SCALE_ABS_ONLY": scale_abs,
            "sign_ok": sign_ok,
            "note": "translation drift is odom-estimated, not external ground truth",
        }
        self.results.append(res)
        print("\n========== ROTATION RESULT ==========")
        print(f"actual: {cfg['direction']} {cfg['actual_deg']:.2f} deg = {actual_signed:.4f} rad signed")
        print(f"cmd_wz: {cfg['wz_cmd']:.4f}")
        print(f"odom yaw delta: {dyaw:.4f} rad")
        print(f"odom-estimated translation drift: {math.hypot(dx, dy):.4f} m")
        if scale_signed is not None:
            print(f"recommended CHASSIS_ODOM_WZ_SCALE_SIGNED={scale_signed:.4f}")
            print(f"abs-only scale={scale_abs:.4f}")
        if sign_ok:
            print("sign check: OK")
        else:
            print("sign check: WRONG. If your direction input is correct, use the SIGNED scale or invert odom wz sign.")
        print("=====================================\n")

    def finish_line(self):
        b = self.baseline; e = self.snapshot(); cfg = self.active
        if not b or not e:
            print("[LINE] missing baseline/current."); return
        actual_signed = abs(cfg["actual_m"]) if cfg["direction"] == "forward" else -abs(cfg["actual_m"])
        dx = e["x"] - b["x"]; dy = e["y"] - b["y"]
        yaw0 = b["yaw"]
        forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        dyaw = e["yaw"] - b["yaw"]
        scale_signed = actual_signed / forward if abs(forward) > 1e-9 else None
        sign_ok = (actual_signed * forward) > 0
        res = {
            "type": "line",
            "actual_direction": cfg["direction"], "actual_m": cfg["actual_m"], "cmd_vx": cfg["vx_cmd"],
            "actual_signed_m": actual_signed,
            "odom_forward_m": forward, "odom_lateral_m": lateral,
            "odom_total_m": math.hypot(dx, dy), "yaw_drift_rad_by_odom": dyaw,
            "recommended_CHASSIS_ODOM_VX_SCALE_SIGNED": scale_signed,
            "sign_ok": sign_ok,
            "note": "lateral/yaw drift are odom-estimated; use visual observation for real straightness too",
        }
        self.results.append(res)
        print("\n========== LINE RESULT ==========")
        print(f"actual: {cfg['direction']} {cfg['actual_m']:.4f} m = {actual_signed:.4f} m signed")
        print(f"cmd_vx: {cfg['vx_cmd']:.4f}")
        print(f"odom forward: {forward:.4f} m")
        print(f"odom lateral: {lateral:.4f} m")
        print(f"odom total: {math.hypot(dx, dy):.4f} m")
        print(f"odom yaw drift: {dyaw:.4f} rad ({math.degrees(dyaw):.2f} deg)")
        if scale_signed is not None:
            print(f"recommended CHASSIS_ODOM_VX_SCALE_SIGNED={scale_signed:.4f}")
        if sign_ok:
            print("vx sign check: OK")
        else:
            print("vx sign check: WRONG. If actual direction input is correct, use SIGNED scale or invert odom vx sign.")
        if abs(lateral) > 0.15 or abs(dyaw) > 0.25:
            print("[WARNING] straight-line quality is poor. Do not trust VX_SCALE until physical straightness is improved.")
            print("Use: bias left/right small|medium|large according to real front-heading drift.")
        print("=================================\n")

    def bias_suggestion(self, side: str, severity: str):
        if side not in ("left", "right") or severity not in ("small", "medium", "large"):
            print("Usage: bias left small|medium|large OR bias right small|medium|large")
            return
        delta = {"small": 0.03, "medium": 0.06, "large": 0.10}[severity]
        # Default layout from your bridge: M1=FL, M2=RL, M3=FR, M4=RR.
        if side == "left":
            # Robot veers left => right side effectively stronger or left side weaker.
            trims = [1.0 + delta, 1.0 + delta, 1.0, 1.0]
            explanation = "车头向左偏：优先增强左侧 M1/M2，或降低右侧 M3/M4。"
        else:
            trims = [1.0, 1.0, 1.0 + delta, 1.0 + delta]
            explanation = "车头向右偏：优先增强右侧 M3/M4，或降低左侧 M1/M2。"
        trim_str = ",".join(f"{x:.2f}" for x in trims)
        res = {"type": "bias", "real_bias": side, "severity": severity, "suggested_CHASSIS_MOTOR_TRIMS": trim_str, "note": explanation}
        self.results.append(res)
        print("\n========== BIAS SUGGESTION ==========")
        print(explanation)
        print(f"suggested CHASSIS_MOTOR_TRIMS={trim_str}")
        print("前提：你的 bridge 已经支持 --motor-trims；否则先让 Cursor 加这个参数。")
        print("=====================================\n")

    def timer_cb(self):
        self.poll_stdin()
        while not self.input_q.empty():
            self.process_command(self.input_q.get())

        if self.auto_cmd is not None:
            self.cmd_pub.publish(self.auto_cmd)

        snap = self.snapshot()
        if snap is not None and self.active is not None:
            self.samples_current.append(snap)
            if self.active["type"] == "static" and self.static_until is not None and time.time() >= self.static_until:
                self.finish_static()
                self.active = None; self.baseline = None; self.samples_current = []; self.static_until = None

        self.write_csv_row()

    def write_csv_row(self):
        snap = self.snapshot()
        if snap is None:
            return
        st = self.latest_state or {}
        cmd = self.latest_cmd; sent = self.latest_cmd_sent
        row = {
            "time": time.time(), "active": self.active["type"] if self.active else "", "auto_name": self.auto_name,
            "odom_x": snap["x"], "odom_y": snap["y"], "yaw_unwrapped": snap["yaw"],
            "odom_vx": snap["vx"], "odom_vy": snap["vy"], "odom_wz": snap["wz"],
            "cmd_vx": cmd.linear.x if cmd else "", "cmd_wz": cmd.angular.z if cmd else "",
            "cmd_sent_vx": sent.linear.x if sent else "", "cmd_sent_wz": sent.angular.z if sent else "",
            "last_sent_vx": st.get("last_sent_vx", ""), "last_sent_wz": st.get("last_sent_wz", ""),
            "vx_pwm": st.get("vx_pwm", ""), "wz_pwm": st.get("wz_pwm", ""),
            "pwm_1": st.get("pwm_1", ""), "pwm_2": st.get("pwm_2", ""), "pwm_3": st.get("pwm_3", ""), "pwm_4": st.get("pwm_4", ""),
            "wheel_layout": st.get("wheel_layout", ""), "motor_signs": st.get("motor_signs", ""), "motor_trims": st.get("motor_trims", ""),
        }
        self.writer.writerow(row)
        self.csv_file.flush()

    def save(self):
        summary = {"created_at": datetime.now().isoformat(), "csv_path": self.csv_path, "results": self.results}
        with open(self.json_path, "w") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        print(f"[SAVE] {self.json_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--cmd-sent-topic", default="/cmd_vel_sent")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--state-topic", default="/chassis_bridge_state")
    parser.add_argument("--out-dir", default="logs/calib")
    args = parser.parse_args()

    rclpy.init()
    node = OdomDirectionTrimWizard(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.publish_stop()
            node.save()
        except Exception:
            pass
    finally:
        try:
            node.hotkey_reader.stop()
        except Exception:
            pass
        try:
            node.csv_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
