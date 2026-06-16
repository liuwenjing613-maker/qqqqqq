#!/usr/bin/env python3
import time
import argparse
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from Rosmaster_Lib import Rosmaster

import os
import sys

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.control.cmd_smoother import CmdSmoother


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class CmdVelToRosmaster(Node):
    """
    ROS2 /cmd_vel -> Yahboom Rosmaster_Lib -> M1 chassis

    简单两段式启动：
      1. 从停止收到任何非零底盘动作时，先输出短启动脉冲。
      2. 脉冲结束后，严格回到 /cmd_vel 给的实际 vx/wz。
    """

    def __init__(
        self,
        port="/dev/myserial",
        max_vx=0.10,
        max_wz=0.50,
        watchdog_timeout=0.5,
        enable_kick_start=True,
        kick_vx=0.09,
        kick_wz=0.24,
        kick_duration=0.18,
        kick_cooldown=1.0,
        cmd_wz_deadzone=0.01,
        cmd_smooth_alpha=0.4,
        max_vx_delta=0.015,
        max_wz_delta=0.02,
        control_rate_hz=20.0,
        debug=False,
    ):
        super().__init__("cmd_vel_to_rosmaster")

        self.port = port
        self.max_vx = float(max_vx)
        self.max_wz = float(max_wz)
        self.watchdog_timeout = float(watchdog_timeout)
        self.debug = debug

        self.lock = threading.Lock()
        self.last_cmd_time = time.time()
        self.last_vx = 0.0
        self.last_wz = 0.0

        self.enable_kick_start = bool(enable_kick_start)
        self.kick_vx = float(kick_vx)
        self.kick_wz = float(kick_wz)
        self.kick_duration = float(kick_duration)
        self.kick_cooldown = float(kick_cooldown)
        self.cmd_wz_deadzone = float(cmd_wz_deadzone)
        self.last_kick_time = 0.0
        self.kick_active_until = 0.0
        self.kick_vx_target = 0.0
        self.kick_wz_target = 0.0

        self.target_vx = 0.0
        self.target_wz = 0.0
        self.smoother = CmdSmoother(
            alpha=cmd_smooth_alpha,
            max_vx_delta=max_vx_delta,
            max_wz_delta=max_wz_delta,
        )

        self.get_logger().info("Initializing Rosmaster...")
        self.get_logger().info(f"Serial port: {self.port}")

        self.bot = Rosmaster(com=self.port)
        self.bot.create_receive_threading()
        time.sleep(0.5)

        self.safe_stop()

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        period = 1.0 / max(5.0, float(control_rate_hz))
        self.control_timer = self.create_timer(period, self.control_timer_callback)

        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)
        self.status_timer = self.create_timer(2.0, self.status_callback)

        self.get_logger().info("cmd_vel_to_rosmaster started.")
        self.get_logger().info("Subscribed topic: /cmd_vel")
        self.get_logger().info(
            f"Safety limits: max_vx={self.max_vx:.3f} m/s, max_wz={self.max_wz:.3f} rad/s"
        )
        self.get_logger().info(
            f"Smooth: alpha={cmd_smooth_alpha:.2f} dvx={max_vx_delta:.3f} dwz={max_wz_delta:.3f} "
            f"rate={control_rate_hz:.1f}Hz"
        )
        self.get_logger().info(
            "Kick start: "
            f"enable={self.enable_kick_start}, kick_vx={self.kick_vx:.3f}, "
            f"kick_wz={self.kick_wz:.3f}, duration={self.kick_duration:.3f}s, "
            f"cooldown={self.kick_cooldown:.3f}s, cmd_wz_deadzone={self.cmd_wz_deadzone:.3f}"
        )

    def _is_stopped(self):
        return abs(self.last_vx) < 1e-4 and abs(self.last_wz) < 1e-4

    def cmd_vel_callback(self, msg: Twist):
        raw_vx = float(msg.linear.x)
        raw_wz = float(msg.angular.z)

        vx = clamp(raw_vx, -self.max_vx, self.max_vx)
        wz = clamp(raw_wz, -self.max_wz, self.max_wz)
        if abs(wz) < self.cmd_wz_deadzone:
            wz = 0.0
        send_vx = vx
        send_wz = wz

        now = time.time()
        nonzero_cmd = abs(send_vx) > 1e-4 or abs(send_wz) > 1e-4
        need_kick = (
            self.enable_kick_start
            and nonzero_cmd
            and self._is_stopped()
            and now - self.last_kick_time > self.kick_cooldown
        )

        with self.lock:
            self.target_vx = send_vx
            self.target_wz = send_wz
            self.last_cmd_time = now
            if not nonzero_cmd:
                self.kick_active_until = 0.0
                self.kick_vx_target = 0.0
                self.kick_wz_target = 0.0
                self.smoother.reset()
            elif need_kick:
                self.kick_active_until = now + self.kick_duration
                self.kick_vx_target = (
                    (1.0 if send_vx > 0 else -1.0) * abs(self.kick_vx)
                    if abs(send_vx) > 1e-4
                    else 0.0
                )
                self.kick_wz_target = (
                    (1.0 if send_wz > 0 else -1.0) * abs(self.kick_wz)
                    if abs(send_wz) > 1e-4
                    else 0.0
                )
                self.last_kick_time = now

        if self.debug:
            self.get_logger().info(
                f"raw cmd: linear.x={raw_vx:.3f}, angular.z={raw_wz:.3f} "
                f"=> target: vx={send_vx:.3f}, wz={send_wz:.3f}, "
                f"kick={need_kick}"
            )

    def control_timer_callback(self):
        now = time.time()
        with self.lock:
            raw_vx = self.target_vx
            raw_wz = self.target_wz
            in_kick = now < self.kick_active_until
            if in_kick:
                raw_vx = self.kick_vx_target
                raw_wz = self.kick_wz_target

        if in_kick:
            # 启动脉冲必须直达底盘，不能被平滑器削弱到死区以下。
            out_vx, out_wz = raw_vx, raw_wz
        else:
            # 脉冲后严格执行 /cmd_vel 的实际设定值，不再做最小速度或巡航改写。
            out_vx, out_wz = raw_vx, raw_wz
        out_vx = clamp(out_vx, -self.max_vx, self.max_vx)
        out_wz = clamp(out_wz, -self.max_wz, self.max_wz)

        with self.lock:
            self.bot.set_car_motion(out_vx, 0.0, out_wz)
            self.last_vx = out_vx
            self.last_wz = out_wz

    def watchdog_callback(self):
        now = time.time()
        if now - self.last_cmd_time > self.watchdog_timeout:
            with self.lock:
                if abs(self.last_vx) > 1e-6 or abs(self.last_wz) > 1e-6:
                    self.target_vx = 0.0
                    self.target_wz = 0.0
                    self.kick_active_until = 0.0
                    self.smoother.reset()
                    self.bot.set_car_motion(0.0, 0.0, 0.0)
                    self.last_vx = 0.0
                    self.last_wz = 0.0
                    self.last_cmd_time = now
                    self.get_logger().warn("Watchdog timeout, auto stop.")

    def status_callback(self):
        try:
            battery = self.bot.get_battery_voltage()
        except Exception:
            battery = None

        try:
            motion = self.bot.get_motion_data()
        except Exception:
            motion = None

        self.get_logger().info(
            f"status: battery={battery}, motion={motion}, last_cmd=(vx={self.last_vx:.3f}, wz={self.last_wz:.3f})"
        )

    def safe_stop(self):
        try:
            self.bot.set_car_motion(0.0, 0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f"safe_stop failed: {repr(e)}")

    def destroy_node(self):
        self.get_logger().info("Stopping car before node shutdown...")
        self.safe_stop()
        time.sleep(0.2)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/myserial")
    parser.add_argument("--max-vx", type=float, default=0.10)
    parser.add_argument("--max-wz", type=float, default=0.50)
    parser.add_argument("--watchdog-timeout", type=float, default=0.5)
    parser.add_argument("--enable-kick-start", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--kick-vx", type=float, default=0.09)
    parser.add_argument("--kick-wz", type=float, default=0.24)
    parser.add_argument("--kick-duration", type=float, default=0.18)
    parser.add_argument("--kick-cooldown", type=float, default=1.0)
    parser.add_argument("--cmd-wz-deadzone", type=float, default=0.01)
    parser.add_argument("--cmd-smooth-alpha", type=float, default=0.4)
    parser.add_argument("--max-vx-delta", type=float, default=0.015)
    parser.add_argument("--max-wz-delta", type=float, default=0.02)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--debug", action="store_true")
    args, _ = parser.parse_known_args()

    rclpy.init()

    node = CmdVelToRosmaster(
        port=args.port,
        max_vx=args.max_vx,
        max_wz=args.max_wz,
        watchdog_timeout=args.watchdog_timeout,
        enable_kick_start=args.enable_kick_start,
        kick_vx=args.kick_vx,
        kick_wz=args.kick_wz,
        kick_duration=args.kick_duration,
        kick_cooldown=args.kick_cooldown,
        cmd_wz_deadzone=args.cmd_wz_deadzone,
        cmd_smooth_alpha=args.cmd_smooth_alpha,
        max_vx_delta=args.max_vx_delta,
        max_wz_delta=args.max_wz_delta,
        control_rate_hz=args.control_rate_hz,
        debug=args.debug,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        node.safe_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
