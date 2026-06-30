#!/usr/bin/env python3
"""ROS2 /cmd_vel -> Rosmaster set_motor() PWM bridge (open-loop, no MCU speed loop)."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from Rosmaster_Lib import Rosmaster
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.insert(0, PROJECT_ROOT)

from src.control.m1_mecanum_pwm import (
    YAHBOOM_M1_LAYOUT,
    describe_layout,
    pwm_from_twist,
)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def parse_motor_signs(text: str) -> Tuple[int, int, int, int]:
    parts = [p.strip() for p in str(text).split(",") if p.strip()]
    if len(parts) != 4:
        raise ValueError(f"motor-signs must have 4 comma-separated values, got {text!r}")
    signs: List[int] = []
    for part in parts:
        val = int(float(part))
        if val not in (-1, 0, 1):
            raise ValueError(f"motor sign must be -1, 0, or 1, got {part!r}")
        signs.append(val)
    return signs[0], signs[1], signs[2], signs[3]


def parse_motor_trims(text: str) -> Tuple[float, float, float, float]:
    parts = [p.strip() for p in str(text).split(",") if p.strip()]
    if len(parts) != 4:
        raise ValueError(f"motor-trims must have 4 comma-separated values, got {text!r}")
    trims = [float(p) for p in parts]
    for i, t in enumerate(trims):
        if t < 0.5 or t > 1.5:
            raise ValueError(f"motor trim[{i}] must be in [0.5, 1.5], got {t}")
    return trims[0], trims[1], trims[2], trims[3]


class PwmSmoother:
    def __init__(self, alpha: float, max_delta: float, pwm_max: float):
        self.alpha = float(alpha)
        self.max_delta = float(max_delta)
        self.pwm_max = float(pwm_max)
        self.state = [0.0, 0.0, 0.0, 0.0]

    def reset(self) -> None:
        self.state = [0.0, 0.0, 0.0, 0.0]

    def update(self, targets: Sequence[float]) -> List[int]:
        if all(abs(t) < 1e-6 for t in targets):
            self.reset()
            return [0, 0, 0, 0]

        out: List[int] = []
        for i, target in enumerate(targets):
            value = float(target)
            if self.alpha > 0.0:
                value = self.alpha * self.state[i] + (1.0 - self.alpha) * value
            delta = clamp(value - self.state[i], -self.max_delta, self.max_delta)
            self.state[i] += delta
            out.append(int(round(clamp(self.state[i], -self.pwm_max, self.pwm_max))))
        return out


class M1PwmCmdVelBridge(Node):
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        max_vx: float = 0.06,
        max_wz: float = 0.06,
        cmd_vx_deadzone: float = 0.0,
        cmd_wz_deadzone: float = 0.0,
        watchdog_timeout: float = 0.5,
        control_rate_hz: float = 20.0,
        vx_pwm_deadband: float = 6.0,
        wz_pwm_deadband: float = 8.0,
        pwm_max: float = 30.0,
        vx_pwm_gain: float = 180.0,
        wz_pwm_gain: float = 120.0,
        smooth_alpha: float = 0.35,
        max_pwm_delta: float = 3.0,
        motor_signs: str = "1,1,1,1",
        motor_trims: str = "1.0,1.0,1.0,1.0",
        wheel_layout: str = YAHBOOM_M1_LAYOUT,
        debug: bool = False,
        publish_odom: bool = False,
        odom_topic: str = "/odom",
        odom_frame: str = "odom",
        base_frame: str = "base_link",
        odom_rate_hz: float = 30.0,
        odom_vxy_deadzone: float = 0.003,
        odom_wz_deadzone: float = 0.015,
        odom_vx_scale: float = 1.0,
        odom_vy_scale: float = 1.0,
        odom_wz_scale: float = 1.0,
        odom_use_vy: bool = False,
        base_yaw_offset: float = 0.0,
    ):
        super().__init__("m1_pwm_cmd_vel_bridge")

        self.publish_odom = bool(publish_odom)
        self.odom_topic = str(odom_topic)
        self.odom_frame = str(odom_frame)
        self.base_frame = str(base_frame)
        self.odom_rate_hz = float(odom_rate_hz)
        self.odom_vxy_deadzone = float(odom_vxy_deadzone)
        self.odom_wz_deadzone = float(odom_wz_deadzone)
        self.odom_vx_scale = float(odom_vx_scale)
        self.odom_vy_scale = float(odom_vy_scale)
        self.odom_wz_scale = float(odom_wz_scale)
        self.odom_use_vy = bool(odom_use_vy)
        self.base_yaw_offset = float(base_yaw_offset)

        self.port = str(port)
        self.max_vx = float(max_vx)
        self.max_wz = float(max_wz)
        self.cmd_vx_deadzone = float(cmd_vx_deadzone)
        self.cmd_wz_deadzone = float(cmd_wz_deadzone)
        self.watchdog_timeout = float(watchdog_timeout)
        self.vx_pwm_deadband = float(vx_pwm_deadband)
        self.wz_pwm_deadband = float(wz_pwm_deadband)
        self.pwm_max = float(pwm_max)
        self.vx_pwm_gain = float(vx_pwm_gain)
        self.wz_pwm_gain = float(wz_pwm_gain)
        self.debug = bool(debug)
        self.motor_signs = parse_motor_signs(motor_signs)
        self.motor_trims = parse_motor_trims(motor_trims)
        self.wheel_layout = str(wheel_layout)

        self.lock = threading.Lock()
        self.last_cmd_time = time.time()
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.last_raw_vx = 0.0
        self.last_raw_wz = 0.0
        self.last_sent_vx = 0.0
        self.last_sent_wz = 0.0
        self.vx_pwm = 0.0
        self.wz_pwm = 0.0
        self.pwm_1 = 0
        self.pwm_2 = 0
        self.pwm_3 = 0
        self.pwm_4 = 0

        self.pwm_smoother = PwmSmoother(smooth_alpha, max_pwm_delta, pwm_max)

        self.odom_pub = None
        self.tf_broadcaster = None
        self.odom_timer = None
        self.odom_lock = threading.Lock()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = None

        self.sent_cmd_pub = self.create_publisher(Twist, "/cmd_vel_sent", 10)
        self.state_pub = self.create_publisher(String, "/chassis_bridge_state", 10)

        self.get_logger().info(f"Initializing Rosmaster PWM bridge on {self.port}")
        try:
            self.bot = Rosmaster(car_type=1, com=self.port)
            self.bot.create_receive_threading()
            time.sleep(0.5)
            self.connected = True
        except Exception as exc:
            self.get_logger().error(f"Rosmaster init failed on port={self.port!r}: {exc!r}")
            raise

        self.safe_stop()

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        period = 1.0 / max(5.0, float(control_rate_hz))
        self.control_timer = self.create_timer(period, self.control_timer_callback)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)
        self.state_timer = self.create_timer(0.5, self.publish_bridge_state)

        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
            self.tf_broadcaster = TransformBroadcaster(self)
            odom_period = 1.0 / max(1.0, self.odom_rate_hz)
            self.odom_timer = self.create_timer(odom_period, self.odom_timer_callback)

        if self.publish_odom:
            self.get_logger().info("m1_pwm_cmd_vel_bridge started (drive_mode=pwm)")
            self.get_logger().info(
                f"Publishing odom topic={self.odom_topic}, "
                f"TF {self.odom_frame}->{self.base_frame}, source=get_motion_data()"
            )
            self.get_logger().info(f"odom_wz_scale={self.odom_wz_scale}")
            self.get_logger().info(
                f"base_link yaw offset={self.base_yaw_offset:.4f} rad "
                f"({math.degrees(self.base_yaw_offset):.1f} deg)"
            )
        else:
            self.get_logger().info("m1_pwm_cmd_vel_bridge started (drive_mode=pwm, no /odom)")
        self.get_logger().info(describe_layout(self.wheel_layout))
        self.get_logger().info(
            f"motor_signs={list(self.motor_signs)}, motor_trims={list(self.motor_trims)}"
        )
        self.get_logger().info(
            f"limits: max_vx={self.max_vx:.3f}, max_wz={self.max_wz:.3f}, "
            f"vx_db={self.vx_pwm_deadband:.1f}, wz_db={self.wz_pwm_deadband:.1f}, "
            f"pwm_max={self.pwm_max:.1f}, vx_gain={self.vx_pwm_gain:.1f}, wz_gain={self.wz_pwm_gain:.1f}"
        )
        self.get_logger().info(
            "Tip: use angular.z in rad/s (e.g. 0.12~0.16), NOT 3.0. "
            "M1 wz breakaway from calibration is about 0.16 rad/s."
        )

    def _filter_cmd(self, vx: float, wz: float) -> Tuple[float, float]:
        if abs(vx) < self.cmd_vx_deadzone:
            vx = 0.0
        if abs(wz) < self.cmd_wz_deadzone:
            wz = 0.0
        vx = clamp(vx, -self.max_vx, self.max_vx)
        wz = clamp(wz, -self.max_wz, self.max_wz)
        return vx, wz

    def cmd_vel_callback(self, msg: Twist) -> None:
        raw_vx = float(msg.linear.x)
        raw_wz = float(msg.angular.z)
        vx, wz = self._filter_cmd(raw_vx, raw_wz)

        with self.lock:
            self.last_raw_vx = raw_vx
            self.last_raw_wz = raw_wz
            self.target_vx = vx
            self.target_wz = wz
            self.last_sent_vx = vx
            self.last_sent_wz = wz
            self.last_cmd_time = time.time()

        if self.debug:
            self.get_logger().info(
                f"raw cmd vx={raw_vx:.3f} wz={raw_wz:.3f} => target vx={vx:.3f} wz={wz:.3f}"
            )

    def _apply_pwm(self, vx: float, wz: float) -> List[int]:
        if abs(vx) < 1e-6 and abs(wz) < 1e-6:
            self.pwm_smoother.reset()
            self.vx_pwm = 0.0
            self.wz_pwm = 0.0
            self.pwm_1 = self.pwm_2 = self.pwm_3 = self.pwm_4 = 0
            self.bot.set_motor(0, 0, 0, 0)
            return [0, 0, 0, 0]

        vx_pwm, wz_pwm, m1, m2, m3, m4 = pwm_from_twist(
            vx,
            wz,
            vx_pwm_deadband=self.vx_pwm_deadband,
            wz_pwm_deadband=self.wz_pwm_deadband,
            vx_pwm_gain=self.vx_pwm_gain,
            wz_pwm_gain=self.wz_pwm_gain,
            pwm_max=self.pwm_max,
            wheel_layout=self.wheel_layout,
        )
        s1, s2, s3, s4 = self.motor_signs
        t1, t2, t3, t4 = self.motor_trims
        targets = [
            m1 * s1 * t1,
            m2 * s2 * t2,
            m3 * s3 * t3,
            m4 * s4 * t4,
        ]
        pwms = self.pwm_smoother.update(targets)

        self.vx_pwm = vx_pwm
        self.wz_pwm = wz_pwm
        self.pwm_1, self.pwm_2, self.pwm_3, self.pwm_4 = pwms
        self.bot.set_motor(pwms[0], pwms[1], pwms[2], pwms[3])
        return pwms

    def control_timer_callback(self) -> None:
        with self.lock:
            vx = self.target_vx
            wz = self.target_wz

        sent = Twist()
        sent.linear.x = vx
        sent.angular.z = wz
        self.sent_cmd_pub.publish(sent)

        pwms = self._apply_pwm(vx, wz)
        if self.debug and any(p != 0 for p in pwms):
            self.get_logger().info(
                f"sent pwm=({pwms[0]}, {pwms[1]}, {pwms[2]}, {pwms[3]}) "
                f"from vx={vx:.3f} wz={wz:.3f} vx_pwm={self.vx_pwm:.1f} wz_pwm={self.wz_pwm:.1f}"
            )

    def odom_timer_callback(self) -> None:
        if not self.publish_odom or self.odom_pub is None or self.tf_broadcaster is None:
            return

        now = time.time()
        try:
            motion = self.bot.get_motion_data()
            vx = float(motion[0]) * self.odom_vx_scale
            vy = float(motion[1]) * self.odom_vy_scale
            wz = float(motion[2]) * self.odom_wz_scale

            if not self.odom_use_vy:
                vy = 0.0

            if abs(vx) < self.odom_vxy_deadzone:
                vx = 0.0
            if abs(vy) < self.odom_vxy_deadzone:
                vy = 0.0
            if abs(wz) < self.odom_wz_deadzone:
                wz = 0.0
        except Exception as exc:
            self.get_logger().warning(
                f"get_motion_data failed, skip odom publish: {exc!r}"
            )
            return

        with self.odom_lock:
            if self.last_odom_time is not None:
                dt = now - self.last_odom_time
                if dt > 0.2:
                    dt = 0.0
                if dt > 0.0:
                    dx_body = vx * dt
                    dy_body = vy * dt
                    dyaw = wz * dt

                    published_yaw_before = math.atan2(
                        math.sin(self.odom_yaw + self.base_yaw_offset),
                        math.cos(self.odom_yaw + self.base_yaw_offset),
                    )

                    yaw_mid = published_yaw_before + 0.5 * dyaw

                    self.odom_x += dx_body * math.cos(yaw_mid) - dy_body * math.sin(yaw_mid)
                    self.odom_y += dx_body * math.sin(yaw_mid) + dy_body * math.cos(yaw_mid)

                    self.odom_yaw += dyaw
                    self.odom_yaw = math.atan2(
                        math.sin(self.odom_yaw), math.cos(self.odom_yaw)
                    )
            self.last_odom_time = now
            x = self.odom_x
            y = self.odom_y
            raw_yaw = self.odom_yaw

        published_yaw = math.atan2(
            math.sin(raw_yaw + self.base_yaw_offset),
            math.cos(raw_yaw + self.base_yaw_offset),
        )

        stamp = self.get_clock().now().to_msg()
        orientation = yaw_to_quaternion(published_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(transform)

    def watchdog_callback(self) -> None:
        now = time.time()
        if now - self.last_cmd_time <= self.watchdog_timeout:
            return

        with self.lock:
            if abs(self.target_vx) < 1e-6 and abs(self.target_wz) < 1e-6:
                return
            self.target_vx = 0.0
            self.target_wz = 0.0
            self.last_sent_vx = 0.0
            self.last_sent_wz = 0.0
            self.last_cmd_time = now

        self.pwm_smoother.reset()
        self.bot.set_motor(0, 0, 0, 0)
        self.pwm_1 = self.pwm_2 = self.pwm_3 = self.pwm_4 = 0
        self.get_logger().warn("Watchdog timeout, PWM stop.")

    def publish_bridge_state(self) -> None:
        with self.lock:
            state = {
                "port": self.port,
                "connected": getattr(self, "connected", False),
                "drive_mode": "pwm",
                "wheel_layout": self.wheel_layout,
                "wheel_layout_desc": describe_layout(self.wheel_layout),
                "last_raw_vx": self.last_raw_vx,
                "last_raw_wz": self.last_raw_wz,
                "last_sent_vx": self.last_sent_vx,
                "last_sent_wz": self.last_sent_wz,
                "vx_pwm": self.vx_pwm,
                "wz_pwm": self.wz_pwm,
                "pwm_1": self.pwm_1,
                "pwm_2": self.pwm_2,
                "pwm_3": self.pwm_3,
                "pwm_4": self.pwm_4,
                "watchdog_timeout": self.watchdog_timeout,
                "vx_pwm_deadband": self.vx_pwm_deadband,
                "wz_pwm_deadband": self.wz_pwm_deadband,
                "pwm_max": self.pwm_max,
                "vx_pwm_gain": self.vx_pwm_gain,
                "wz_pwm_gain": self.wz_pwm_gain,
                "motor_signs": list(self.motor_signs),
                "motor_trims": list(self.motor_trims),
                "odom_vx_scale": self.odom_vx_scale,
                "odom_vy_scale": self.odom_vy_scale,
                "odom_wz_scale": self.odom_wz_scale,
                "odom_use_vy": self.odom_use_vy,
                "odom_vxy_deadzone": self.odom_vxy_deadzone,
                "odom_wz_deadzone": self.odom_wz_deadzone,
                "base_yaw_offset": self.base_yaw_offset,
                "base_yaw_offset_deg": math.degrees(self.base_yaw_offset),
                "time": time.time(),
            }
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def safe_stop(self) -> None:
        try:
            self.pwm_smoother.reset()
            self.bot.set_motor(0, 0, 0, 0)
            self.pwm_1 = self.pwm_2 = self.pwm_3 = self.pwm_4 = 0
        except Exception as exc:
            self.get_logger().error(f"safe_stop failed: {exc!r}")

    def destroy_node(self) -> None:
        self.get_logger().info("Stopping PWM motors before node shutdown...")
        self.safe_stop()
        time.sleep(0.2)
        super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="M1 PWM /cmd_vel bridge via set_motor()")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--max-vx", type=float, default=0.06)
    parser.add_argument("--max-wz", type=float, default=0.06)
    parser.add_argument("--cmd-vx-deadzone", type=float, default=0.0)
    parser.add_argument("--cmd-wz-deadzone", type=float, default=0.0)
    parser.add_argument("--watchdog-timeout", type=float, default=0.5)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--vx-pwm-deadband", type=float, default=6.0)
    parser.add_argument("--wz-pwm-deadband", type=float, default=8.0)
    parser.add_argument("--pwm-max", type=float, default=30.0)
    parser.add_argument("--vx-pwm-gain", type=float, default=180.0)
    parser.add_argument("--wz-pwm-gain", type=float, default=120.0)
    parser.add_argument("--smooth-alpha", type=float, default=0.35)
    parser.add_argument("--max-pwm-delta", type=float, default=3.0)
    parser.add_argument("--motor-signs", default="1,1,1,1")
    parser.add_argument("--motor-trims", default="1.0,1.0,1.0,1.0")
    parser.add_argument(
        "--wheel-layout",
        default=YAHBOOM_M1_LAYOUT,
        choices=["fl-rl-fr-rr", "yahboom", "m1", "fl-fr-rl-rr", "fl-fr-rr-rl"],
        help="Default fl-rl-fr-rr matches Yahboom board: M1=FL M2=RL M3=FR M4=RR",
    )
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--publish-odom", action="store_true", default=False)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--odom-rate-hz", type=float, default=30.0)
    parser.add_argument("--odom-vxy-deadzone", type=float, default=0.003)
    parser.add_argument("--odom-wz-deadzone", type=float, default=0.015)
    parser.add_argument("--odom-vx-scale", type=float, default=1.0)
    parser.add_argument("--odom-vy-scale", type=float, default=1.0)
    parser.add_argument("--odom-wz-scale", type=float, default=1.0)
    parser.add_argument("--odom-use-vy", action="store_true", default=False)
    parser.add_argument("--base-yaw-offset", type=float, default=0.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = M1PwmCmdVelBridge(
        port=args.port,
        max_vx=args.max_vx,
        max_wz=args.max_wz,
        cmd_vx_deadzone=args.cmd_vx_deadzone,
        cmd_wz_deadzone=args.cmd_wz_deadzone,
        watchdog_timeout=args.watchdog_timeout,
        control_rate_hz=args.control_rate_hz,
        vx_pwm_deadband=args.vx_pwm_deadband,
        wz_pwm_deadband=args.wz_pwm_deadband,
        pwm_max=args.pwm_max,
        vx_pwm_gain=args.vx_pwm_gain,
        wz_pwm_gain=args.wz_pwm_gain,
        smooth_alpha=args.smooth_alpha,
        max_pwm_delta=args.max_pwm_delta,
        motor_signs=args.motor_signs,
        motor_trims=args.motor_trims,
        wheel_layout=args.wheel_layout,
        debug=args.debug,
        publish_odom=args.publish_odom,
        odom_topic=args.odom_topic,
        odom_frame=args.odom_frame,
        base_frame=args.base_frame,
        odom_rate_hz=args.odom_rate_hz,
        odom_vxy_deadzone=args.odom_vxy_deadzone,
        odom_wz_deadzone=args.odom_wz_deadzone,
        odom_vx_scale=args.odom_vx_scale,
        odom_vy_scale=args.odom_vy_scale,
        odom_wz_scale=args.odom_wz_scale,
        odom_use_vy=args.odom_use_vy,
        base_yaw_offset=args.base_yaw_offset,
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
