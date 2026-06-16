#!/usr/bin/env python3
import os
import time
import threading
from Rosmaster_Lib import Rosmaster

PROJECT_ROOT = os.path.expanduser("~/rdk_x5_vln_robot")


def _default_chassis_port() -> str:
    try:
        import sys

        if PROJECT_ROOT not in sys.path:
            sys.path.insert(0, PROJECT_ROOT)
        from src.config.mvp_tune import load_mvp_tune

        return load_mvp_tune()["chassis_port"]
    except Exception:
        return "/dev/ttyUSB1"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ChassisController:
    """
    ROSMASTER M1 底盘控制封装。

    坐标约定：
    vx > 0：前进
    vx < 0：后退
    vy > 0：左平移
    vy < 0：右平移
    wz > 0：左旋
    wz < 0：右旋

    第一阶段默认低速，保证演示安全。
    """

    def __init__(
        self,
        port=None,
        max_vx=0.12,
        max_vy=0.08,
        max_wz=0.60,
        watchdog_timeout=0.5,
    ):
        self.port = port or _default_chassis_port()
        self.bot = Rosmaster(com=self.port)
        self.bot.create_receive_threading()
        time.sleep(0.5)

        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_wz = max_wz

        self.watchdog_timeout = watchdog_timeout
        self.last_cmd_time = time.time()
        self._lock = threading.Lock()
        self._running = True

        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    def set_velocity(self, vx, vy=0.0, wz=0.0):
        """
        设置底盘速度。
        所有速度都会被限幅，避免上层算法误输出导致撞车。
        """
        vx = clamp(float(vx), -self.max_vx, self.max_vx)
        vy = clamp(float(vy), -self.max_vy, self.max_vy)
        wz = clamp(float(wz), -self.max_wz, self.max_wz)

        with self._lock:
            self.bot.set_car_motion(vx, vy, wz)
            self.last_cmd_time = time.time()

    def stop(self):
        """
        普通停车。
        """
        with self._lock:
            self.bot.set_car_motion(0.0, 0.0, 0.0)
            self.last_cmd_time = time.time()

    def emergency_stop(self):
        """
        急停。后续可以扩展蜂鸣器、日志、状态锁死等。
        """
        with self._lock:
            self.bot.set_car_motion(0.0, 0.0, 0.0)
            self.last_cmd_time = time.time()

    def get_status(self):
        """
        返回底盘基础状态。读取失败时返回 None。
        """
        status = {
            "port": self.port,
            "version": None,
            "battery_voltage": None,
            "motion": None,
            "acc": None,
            "gyro": None,
        }

        try:
            status["version"] = self.bot.get_version()
        except Exception:
            pass

        try:
            status["battery_voltage"] = self.bot.get_battery_voltage()
        except Exception:
            pass

        try:
            status["motion"] = self.bot.get_motion_data()
        except Exception:
            pass

        try:
            status["acc"] = self.bot.get_accelerometer_data()
        except Exception:
            pass

        try:
            status["gyro"] = self.bot.get_gyroscope_data()
        except Exception:
            pass

        return status

    def close(self):
        self._running = False
        self.emergency_stop()
        time.sleep(0.1)

    def _watchdog_loop(self):
        """
        如果上层超过 watchdog_timeout 没有继续发命令，自动停车。
        这是防止程序卡死后小车继续跑。
        """
        while self._running:
            time.sleep(0.05)
            if time.time() - self.last_cmd_time > self.watchdog_timeout:
                with self._lock:
                    self.bot.set_car_motion(0.0, 0.0, 0.0)
                    self.last_cmd_time = time.time()
