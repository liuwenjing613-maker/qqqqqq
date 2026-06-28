#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from Rosmaster_Lib import Rosmaster

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")
sys.path.append(PROJECT_ROOT)

from src.config.mvp_tune import load_mvp_tune
from src.control.cmd_smoother import CmdSmoother


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


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
        reset_on_zero=False,
        zero_reset_hold_sec=0.4,
        debug=False,
        publish_odom=True,
        odom_topic="/odom",
        odom_frame="odom",
        base_frame="base_link",
        odom_rate_hz=30.0,
    ):
        super().__init__("cmd_vel_to_rosmaster")

        self.port = port
        self.publish_odom = bool(publish_odom)
        self.odom_topic = str(odom_topic)
        self.odom_frame = str(odom_frame)
        self.base_frame = str(base_frame)
        self.odom_rate_hz = float(odom_rate_hz)
        self.max_vx = float(max_vx)
        self.max_wz = float(max_wz)
        self.watchdog_timeout = float(watchdog_timeout)
        self.debug = debug
        self.connected = False

        self.lock = threading.Lock()
        self.last_cmd_time = time.time()
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.last_raw_vx = 0.0
        self.last_raw_wz = 0.0
        self.last_sent_vx = 0.0
        self.last_sent_wz = 0.0

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
        self.in_kick = False

        self.target_vx = 0.0
        self.target_wz = 0.0
        self.smoother = CmdSmoother(
            alpha=cmd_smooth_alpha,
            max_vx_delta=max_vx_delta,
            max_wz_delta=max_wz_delta,
            reset_on_zero=reset_on_zero,
            zero_reset_hold_sec=zero_reset_hold_sec,
        )

        self.sent_cmd_pub = self.create_publisher(Twist, "/cmd_vel_sent", 10)
        self.state_pub = self.create_publisher(String, "/chassis_bridge_state", 10)

        self.odom_pub = None
        self.tf_broadcaster = None
        self.odom_timer = None
        self.odom_lock = threading.Lock()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = None

        self.get_logger().info("Initializing Rosmaster...")
        self.get_logger().info(f"Serial port: {self.port}")

        try:
            self.bot = Rosmaster(com=self.port)
            self.bot.create_receive_threading()
            time.sleep(0.5)
            self.connected = True
        except Exception as exc:
            self.get_logger().error(
                f"Rosmaster init failed on port={self.port!r}: {exc!r}. "
                "Check CHASSIS_PORT (/dev/ttyUSB1 or /dev/myserial), cable, power, "
                "and that no other process holds the serial port."
            )
            raise

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
        self.bridge_state_timer = self.create_timer(0.5, self.publish_bridge_state)

        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
            self.tf_broadcaster = TransformBroadcaster(self)
            odom_period = 1.0 / max(1.0, self.odom_rate_hz)
            self.odom_timer = self.create_timer(odom_period, self.odom_timer_callback)

        self.get_logger().info("cmd_vel_to_rosmaster started.")
        self.get_logger().info("Subscribed topic: /cmd_vel")
        publish_topics = "/cmd_vel_sent, /chassis_bridge_state"
        if self.publish_odom:
            publish_topics += f", {self.odom_topic} (TF {self.odom_frame}->{self.base_frame})"
        self.get_logger().info(f"Publishing: {publish_topics}")
        self.get_logger().info(
            f"Safety limits: max_vx={self.max_vx:.3f} m/s, max_wz={self.max_wz:.3f} rad/s"
        )
        self.get_logger().info(
            f"Smooth: alpha={cmd_smooth_alpha:.2f} dvx={max_vx_delta:.3f} dwz={max_wz_delta:.3f} "
            f"rate={control_rate_hz:.1f}Hz reset_on_zero={reset_on_zero}"
        )
        self.get_logger().info(
            "Kick start: "
            f"enable={self.enable_kick_start}, kick_vx={self.kick_vx:.3f}, "
            f"kick_wz={self.kick_wz:.3f}, duration={self.kick_duration:.3f}s, "
            f"cooldown={self.kick_cooldown:.3f}s, cmd_wz_deadzone={self.cmd_wz_deadzone:.3f}"
        )
        if self.publish_odom:
            self.get_logger().info(
                f"Odom: topic={self.odom_topic}, frames={self.odom_frame}->{self.base_frame}, "
                f"rate={self.odom_rate_hz:.1f}Hz, source=get_motion_data()"
            )

    def odom_timer_callback(self):
        if not self.publish_odom or self.odom_pub is None or self.tf_broadcaster is None:
            return

        now = time.time()
        try:
            motion = self.bot.get_motion_data()
            vx = float(motion[0])
            vy = float(motion[1])
            wz = float(motion[2])
        except Exception as exc:
            self.get_logger().warning(
                f"get_motion_data failed, skip odom publish: {exc!r}"
            )
            return

        with self.odom_lock:
            if self.last_odom_time is not None:
                dt = now - self.last_odom_time
                if dt > 0.0:
                    cos_yaw = math.cos(self.odom_yaw)
                    sin_yaw = math.sin(self.odom_yaw)
                    self.odom_x += (vx * cos_yaw - vy * sin_yaw) * dt
                    self.odom_y += (vx * sin_yaw + vy * cos_yaw) * dt
                    self.odom_yaw += wz * dt
                    self.odom_yaw = math.atan2(
                        math.sin(self.odom_yaw), math.cos(self.odom_yaw)
                    )
            self.last_odom_time = now
            x = self.odom_x
            y = self.odom_y
            yaw = self.odom_yaw

        stamp = self.get_clock().now().to_msg()
        orientation = yaw_to_quaternion(yaw)

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

    def _is_stopped(self):
        return abs(self.last_vx) < 1e-4 and abs(self.last_wz) < 1e-4

    def cmd_vel_callback(self, msg: Twist):
        raw_vx = float(msg.linear.x)
        raw_wz = float(msg.angular.z)

        vx = clamp(raw_vx, -self.max_vx, self.max_vx)
        wz = clamp(raw_wz, -self.max_wz, self.max_wz)
        if abs(wz) < self.cmd_wz_deadzone:
            wz = 0.0

        now = time.time()
        nonzero_cmd = abs(vx) > 1e-4 or abs(wz) > 1e-4
        need_kick = (
            self.enable_kick_start
            and nonzero_cmd
            and self._is_stopped()
            and now - self.last_kick_time > self.kick_cooldown
        )

        with self.lock:
            self.last_raw_vx = raw_vx
            self.last_raw_wz = raw_wz
            self.target_vx = vx
            self.target_wz = wz
            self.last_cmd_time = now
            if not nonzero_cmd:
                self.kick_active_until = 0.0
                self.kick_vx_target = 0.0
                self.kick_wz_target = 0.0
            elif need_kick:
                self.kick_active_until = now + self.kick_duration
                self.kick_vx_target = (
                    (1.0 if vx > 0 else -1.0) * abs(self.kick_vx) if abs(vx) > 1e-4 else 0.0
                )
                self.kick_wz_target = (
                    (1.0 if wz > 0 else -1.0) * abs(self.kick_wz) if abs(wz) > 1e-4 else 0.0
                )
                self.last_kick_time = now

        if self.debug:
            self.get_logger().info(
                f"raw cmd: linear.x={raw_vx:.3f}, angular.z={raw_wz:.3f} "
                f"=> target: vx={vx:.3f}, wz={wz:.3f}, kick={need_kick}"
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

        self.in_kick = in_kick

        if in_kick:
            out_vx, out_wz = raw_vx, raw_wz
        else:
            out_vx, out_wz = self.smoother.update(raw_vx, raw_wz)

        out_vx = clamp(out_vx, -self.max_vx, self.max_vx)
        out_wz = clamp(out_wz, -self.max_wz, self.max_wz)

        sent = Twist()
        sent.linear.x = out_vx
        sent.linear.y = 0.0
        sent.angular.z = out_wz
        self.sent_cmd_pub.publish(sent)

        with self.lock:
            self.last_sent_vx = out_vx
            self.last_sent_wz = out_wz
            self.bot.set_car_motion(out_vx, 0.0, out_wz)
            self.last_vx = out_vx
            self.last_wz = out_wz

        if self.debug and (abs(out_vx) > 1e-4 or abs(out_wz) > 1e-4):
            self.get_logger().info(
                f"sent cmd: vx={out_vx:.3f}, wz={out_wz:.3f}, kick={in_kick}"
            )

    def publish_bridge_state(self):
        with self.lock:
            state = {
                "port": self.port,
                "connected": self.connected,
                "last_raw_vx": self.last_raw_vx,
                "last_raw_wz": self.last_raw_wz,
                "last_sent_vx": self.last_sent_vx,
                "last_sent_wz": self.last_sent_wz,
                "kick_enabled": self.enable_kick_start,
                "kick_active": self.in_kick,
                "watchdog_timeout": self.watchdog_timeout,
                "time": time.time(),
            }
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

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
                    self.last_sent_vx = 0.0
                    self.last_sent_wz = 0.0
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
            f"status: battery={battery}, motion={motion}, "
            f"last_sent=(vx={self.last_sent_vx:.3f}, wz={self.last_sent_wz:.3f})"
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
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--mvp-tune-config", default=None)
    pre_args, _ = pre_parser.parse_known_args()
    tune = load_mvp_tune(pre_args.mvp_tune_config)

    parser = argparse.ArgumentParser()
    parser.add_argument("--mvp-tune-config", default=tune["config_path"])
    parser.add_argument("--port", default=tune["chassis_port"])
    parser.add_argument("--max-vx", type=float, default=tune["chassis_max_vx"])
    parser.add_argument("--max-wz", type=float, default=tune["chassis_max_wz"])
    parser.add_argument("--watchdog-timeout", type=float, default=0.5)
    parser.add_argument(
        "--enable-kick-start",
        action=argparse.BooleanOptionalAction,
        default=tune["enable_kick_start"],
    )
    parser.add_argument("--kick-vx", type=float, default=tune["kick_vx"])
    parser.add_argument("--kick-wz", type=float, default=tune["kick_wz"])
    parser.add_argument("--kick-duration", type=float, default=tune["kick_duration"])
    parser.add_argument("--kick-cooldown", type=float, default=tune["kick_cooldown"])
    parser.add_argument("--cmd-wz-deadzone", type=float, default=tune["cmd_wz_deadzone"])
    parser.add_argument("--cmd-smooth-alpha", type=float, default=tune["cmd_smooth_alpha"])
    parser.add_argument("--max-vx-delta", type=float, default=tune["max_vx_delta"])
    parser.add_argument("--max-wz-delta", type=float, default=tune["max_wz_delta"])
    parser.add_argument("--control-rate-hz", type=float, default=tune["control_rate_hz"])
    parser.add_argument(
        "--reset-on-zero",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--zero-reset-hold-sec", type=float, default=0.4)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument(
        "--publish-odom",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--odom-rate-hz", type=float, default=30.0)
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
        reset_on_zero=args.reset_on_zero,
        zero_reset_hold_sec=args.zero_reset_hold_sec,
        debug=args.debug,
        publish_odom=args.publish_odom,
        odom_topic=args.odom_topic,
        odom_frame=args.odom_frame,
        base_frame=args.base_frame,
        odom_rate_hz=args.odom_rate_hz,
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
