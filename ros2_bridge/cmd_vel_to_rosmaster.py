#!/usr/bin/env python3
import time
import argparse
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from Rosmaster_Lib import Rosmaster


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class CmdVelToRosmaster(Node):
    """
    ROS2 /cmd_vel -> Yahboom Rosmaster_Lib -> M1 chassis

    当前阶段禁用横移：
    - 只使用 msg.linear.x 控制前进/后退
    - 忽略 msg.linear.y
    - 只使用 msg.angular.z 控制左转/右转

    坐标约定：
    linear.x > 0 : 前进
    linear.x < 0 : 后退
    angular.z > 0 : 左转
    angular.z < 0 : 右转
    """

    def __init__(
        self,
        port="/dev/myserial",
        max_vx=0.10,
        max_wz=0.50,
        watchdog_timeout=0.5,
        enable_kick_start=True,
        kick_vx=0.09,
        kick_duration=0.18,
        min_drive_vx=0.045,
        kick_max_wz=0.08,
        kick_cooldown=1.0,
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

        # 启动死区补偿：M1 落地低速启动困难时使用
        self.enable_kick_start = bool(enable_kick_start)
        self.kick_vx = float(kick_vx)
        self.kick_duration = float(kick_duration)
        self.min_drive_vx = float(min_drive_vx)
        self.kick_max_wz = float(kick_max_wz)
        self.kick_cooldown = float(kick_cooldown)
        self.last_kick_time = 0.0

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
            10
        )

        self.watchdog_timer = self.create_timer(
            0.05,
            self.watchdog_callback
        )

        self.status_timer = self.create_timer(
            2.0,
            self.status_callback
        )

        self.get_logger().info("cmd_vel_to_rosmaster started.")
        self.get_logger().info("Subscribed topic: /cmd_vel")
        self.get_logger().info("linear.y is ignored in this stage.")
        self.get_logger().info(
            f"Safety limits: max_vx={self.max_vx:.3f} m/s, max_wz={self.max_wz:.3f} rad/s"
        )
        self.get_logger().info(
            "Kick start: "
            f"enable={self.enable_kick_start}, kick_vx={self.kick_vx:.3f}, "
            f"kick_duration={self.kick_duration:.3f}s, min_drive_vx={self.min_drive_vx:.3f}, "
            f"kick_max_wz={self.kick_max_wz:.3f}, kick_cooldown={self.kick_cooldown:.3f}s"
        )

    def cmd_vel_callback(self, msg: Twist):
        raw_vx = float(msg.linear.x)
        raw_vy = float(msg.linear.y)
        raw_wz = float(msg.angular.z)

        # 阶段内仍禁用横移
        vx = clamp(raw_vx, -self.max_vx, self.max_vx)
        vy = 0.0
        wz = clamp(raw_wz, -self.max_wz, self.max_wz)

        now = time.time()

        # 如果上层给了一个很小但非零的前进速度，可能低于静摩擦死区
        # 处理策略：
        # 1. abs(vx) 太小则直接归零，防止电机嗡嗡响
        # 2. 从停止状态开始前进时，给一个很短的启动脉冲
        send_vx = vx

        if abs(vx) < 1e-4:
            send_vx = 0.0
        elif abs(vx) < self.min_drive_vx:
            send_vx = self.min_drive_vx if vx > 0 else -self.min_drive_vx

        need_kick = (
            self.enable_kick_start
            and abs(self.last_vx) < 1e-4
            and abs(vx) > 1e-4
            and abs(wz) < self.kick_max_wz
            and now - self.last_kick_time > self.kick_cooldown
        )

        with self.lock:
            if need_kick:
                kick = self.kick_vx if vx > 0 else -self.kick_vx
                kick = clamp(kick, -self.max_vx, self.max_vx)

                self.bot.set_car_motion(kick, 0.0, wz)
                time.sleep(self.kick_duration)
                self.last_kick_time = time.time()

            self.bot.set_car_motion(send_vx, vy, wz)
            self.last_cmd_time = time.time()
            self.last_vx = send_vx
            self.last_wz = wz

        if self.debug:
            self.get_logger().info(
                f"raw cmd: linear.x={raw_vx:.3f}, linear.y={raw_vy:.3f}, angular.z={raw_wz:.3f} "
                f"=> send: vx={send_vx:.3f}, vy=0.000, wz={wz:.3f}, kick={need_kick}"
            )
    def watchdog_callback(self):
        """
        如果超过 watchdog_timeout 没有收到新的 /cmd_vel，自动停车。
        防止上层程序卡死后小车继续跑。
        """
        now = time.time()
        if now - self.last_cmd_time > self.watchdog_timeout:
            with self.lock:
                if abs(self.last_vx) > 1e-6 or abs(self.last_wz) > 1e-6:
                    self.bot.set_car_motion(0.0, 0.0, 0.0)
                    self.last_vx = 0.0
                    self.last_wz = 0.0
                    self.last_cmd_time = now
                    self.get_logger().warn("Watchdog timeout, auto stop.")

    def status_callback(self):
        """
        定期打印状态，方便确认底盘仍在线。
        """
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
    parser.add_argument("--kick-vx", type=float, default=0.09,
                        help="startup pulse linear speed (m/s)")
    parser.add_argument("--kick-duration", type=float, default=0.18,
                        help="startup pulse duration (s)")
    parser.add_argument("--min-drive-vx", type=float, default=0.045,
                        help="minimum sustained drive speed to overcome deadzone")
    parser.add_argument("--kick-max-wz", type=float, default=0.08,
                        help="allow kick only when |wz| is below this value")
    parser.add_argument("--kick-cooldown", type=float, default=1.0,
                        help="minimum interval between kick pulses (s)")
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
        kick_duration=args.kick_duration,
        min_drive_vx=args.min_drive_vx,
        kick_max_wz=args.kick_max_wz,
        kick_cooldown=args.kick_cooldown,
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
