#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read-only Nav2 S-shape diagnostic node.
It subscribes to topics and reads TF only. It never publishes /cmd_vel,
never calls services, and never modifies params.
"""

import argparse
import csv
import math
import os
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import tf2_ros


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_q(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


def is_ok(x):
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def mean_abs(xs):
    xs = [abs(x) for x in xs if is_ok(x)]
    return sum(xs) / len(xs) if xs else None


def fmt(x, n=3):
    if x is None or not is_ok(x):
        return "nan"
    return f"{x:.{n}f}"


class Nav2SDiag(Node):
    def __init__(self, args):
        super().__init__("nav2_s_diagnose")
        self.args = args
        self.t0 = self.now_s()

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.join(args.out, "nav2_s_diag_" + stamp)
        os.makedirs(self.out_dir, exist_ok=True)

        self.log_path = os.path.join(self.out_dir, "report.log")
        self.csv_path = os.path.join(self.out_dir, "summary.csv")
        self.log_file = open(self.log_path, "w", encoding="utf-8")
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv = csv.DictWriter(self.csv_file, fieldnames=[
            "t", "cmd_pubs", "cmd_vx", "cmd_wz", "smooth_vx", "smooth_wz",
            "odom_vx", "odom_wz", "cmd_flip_rate", "smooth_flip_rate", "odom_flip_rate",
            "straight_mean_abs_odom_wz", "map_odom_xy_jump_max", "map_odom_yaw_jump_max_deg",
            "path_dist", "path_heading_err_deg", "path_curv_changes", "scan_front_min",
            "scan_left_min", "scan_right_min", "scan_invalid_ratio", "scan_speckle", "diagnosis"
        ])
        self.csv.writeheader()

        self.rate_hist = {k: deque(maxlen=2000) for k in [
            "cmd", "smooth", "odom", "amcl", "scan", "plan", "local_plan"]}
        self.cmd_hist = deque(maxlen=2000)
        self.smooth_hist = deque(maxlen=2000)
        self.odom_hist = deque(maxlen=3000)
        self.straight_hist = deque(maxlen=1500)
        self.map_odom_hist = deque(maxlen=1000)

        self.last_cmd = None
        self.last_smooth = None
        self.last_odom = None
        self.last_scan = None
        self.last_plan = None
        self.last_local_plan = None

        normal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Twist, args.cmd_topic, self.on_cmd, normal_qos)
        self.create_subscription(Twist, args.smooth_topic, self.on_smooth, normal_qos)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, normal_qos)
        self.create_subscription(PoseWithCovarianceStamped, args.amcl_topic, self.on_amcl, normal_qos)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, sensor_qos)
        self.create_subscription(Path, args.plan_topic, self.on_plan, normal_qos)
        self.create_subscription(Path, args.local_plan_topic, self.on_local_plan, normal_qos)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(args.period, self.report)

        self.log("[INFO] Nav2 S diagnose started.")
        self.log("[INFO] READ ONLY: no cmd_vel publish, no service call, no param modification.")
        self.log("[INFO] Output dir: " + self.out_dir)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, s):
        print(s)
        self.log_file.write(s + "\n")
        self.log_file.flush()

    def mark(self, name):
        self.rate_hist[name].append(self.now_s())

    def rate(self, name, window=5.0):
        now = self.now_s()
        xs = [t for t in self.rate_hist[name] if now - t <= window]
        if len(xs) < 2:
            return 0.0
        return (len(xs) - 1) / max(xs[-1] - xs[0], 1e-6)

    def on_cmd(self, msg):
        t = self.now_s()
        self.mark("cmd")
        self.last_cmd = (t, msg.linear.x, msg.angular.z)
        self.cmd_hist.append(self.last_cmd)

    def on_smooth(self, msg):
        t = self.now_s()
        self.mark("smooth")
        self.last_smooth = (t, msg.linear.x, msg.angular.z)
        self.smooth_hist.append(self.last_smooth)

    def on_odom(self, msg):
        t = self.now_s()
        self.mark("odom")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_q(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.last_odom = (t, x, y, yaw, vx, wz)
        self.odom_hist.append(self.last_odom)

        cmd = self.active_cmd()
        if cmd is not None:
            ct, cvx, cwz, src = cmd
            if t - ct < 0.8 and abs(cvx) > self.args.min_move_vx and abs(cwz) < self.args.straight_cmd_wz:
                self.straight_hist.append((t, cvx, vx, wz, src))

    def on_amcl(self, msg):
        self.mark("amcl")

    def on_scan(self, msg):
        self.mark("scan")
        self.last_scan = self.scan_metrics(msg)

    def on_plan(self, msg):
        self.mark("plan")
        self.last_plan = msg

    def on_local_plan(self, msg):
        self.mark("local_plan")
        self.last_local_plan = msg

    def active_cmd(self):
        now = self.now_s()
        if self.last_smooth is not None and now - self.last_smooth[0] < 1.0:
            return self.last_smooth + ("smooth",)
        if self.last_cmd is not None and now - self.last_cmd[0] < 1.0:
            return self.last_cmd + ("cmd",)
        return None

    def flip_rate(self, hist, idx=2, window=5.0):
        now = self.now_s()
        vals = [(x[0], x[idx]) for x in hist if now - x[0] <= window and abs(x[idx]) >= self.args.flip_wz]
        if len(vals) < 3:
            return 0.0
        signs = [1 if v > 0 else -1 for _, v in vals]
        flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        span = max(vals[-1][0] - vals[0][0], 1e-6)
        return flips / span

    def recent(self, hist, window):
        now = self.now_s()
        return [x for x in hist if now - x[0] <= window]

    def scan_metrics(self, msg):
        front = []
        left = []
        right = []
        invalid = 0
        valid = 0
        speckle = 0
        rs = list(msg.ranges)
        n = len(rs)

        def good(r):
            return is_ok(r) and msg.range_min <= r <= msg.range_max

        for i, r in enumerate(rs):
            angle = wrap(msg.angle_min + i * msg.angle_increment)
            if not good(r):
                invalid += 1
                continue
            valid += 1

            if abs(angle) <= math.radians(20):
                front.append(r)
            if math.radians(60) <= angle <= math.radians(120):
                left.append(r)
            if -math.radians(120) <= angle <= -math.radians(60):
                right.append(r)

            if 2 <= i < n - 2 and r < 1.0:
                neigh = [rs[i - 2], rs[i - 1], rs[i + 1], rs[i + 2]]
                bad = 0
                for nr in neigh:
                    if (not good(nr)) or abs(nr - r) > 0.45:
                        bad += 1
                if bad >= 3:
                    speckle += 1

        total = max(valid + invalid, 1)
        return {
            "front_min": min(front) if front else None,
            "left_min": min(left) if left else None,
            "right_min": min(right) if right else None,
            "invalid_ratio": invalid / total,
            "speckle": speckle,
        }

    def tf_xy_yaw(self, target, source):
        try:
            tr = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            p = tr.transform.translation
            yaw = yaw_from_q(tr.transform.rotation)
            return p.x, p.y, yaw
        except Exception:
            return None

    def map_odom_jump(self):
        mo = self.tf_xy_yaw(self.args.map_frame, self.args.odom_frame)
        now = self.now_s()
        if mo is not None:
            self.map_odom_hist.append((now, mo[0], mo[1], mo[2]))

        vals = self.recent(self.map_odom_hist, 5.0)
        if len(vals) < 3:
            return None, None

        xy_rates = []
        yaw_rates = []
        for a, b in zip(vals, vals[1:]):
            dt = max(b[0] - a[0], 1e-6)
            xy_rates.append(math.hypot(b[1] - a[1], b[2] - a[2]) / dt)
            yaw_rates.append(abs(wrap(b[3] - a[3])) / dt)

        return max(xy_rates), max(yaw_rates)

    def path_metrics(self, path):
        if path is None or len(path.poses) < 2:
            return None

        frame = path.header.frame_id or self.args.map_frame
        base = self.tf_xy_yaw(frame, self.args.base_frame)
        if base is None:
            return None

        bx, by, byaw = base
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        ds = [math.hypot(px - bx, py - by) for px, py in pts]
        idx = min(range(len(ds)), key=lambda i: ds[i])
        dist = ds[idx]

        fut = min(idx + 6, len(pts) - 1)
        pyaw = math.atan2(pts[fut][1] - pts[idx][1], pts[fut][0] - pts[idx][0])
        heading_err = wrap(pyaw - byaw)

        headings = []
        a = max(0, idx - 6)
        b = min(len(pts) - 1, idx + 10)
        for j in range(a, b):
            dx = pts[j + 1][0] - pts[j][0]
            dy = pts[j + 1][1] - pts[j][1]
            if math.hypot(dx, dy) > 1e-4:
                headings.append(math.atan2(dy, dx))

        dyaws = [wrap(y2 - y1) for y1, y2 in zip(headings, headings[1:])]
        signs = [1 if d > 0 else -1 for d in dyaws if abs(d) > 0.03]
        changes = sum(1 for x, y in zip(signs, signs[1:]) if x != y)

        return dist, heading_err, changes

    def straight_metrics(self):
        vals = self.recent(self.straight_hist, 8.0)
        if len(vals) < 5:
            return None
        odom_wz = [v[3] for v in vals]
        return mean_abs(odom_wz)

    def report(self):
        elapsed = self.now_s() - self.t0

        pubs = self.get_publishers_info_by_topic(self.args.cmd_topic)
        pub_names = []
        for p in pubs:
            ns = p.node_namespace or ""
            name = p.node_name or "?"
            pub_names.append((ns.rstrip("/") + "/" + name).replace("//", "/"))

        cmd = self.last_cmd
        sm = self.last_smooth
        odom = self.last_odom

        cmd_vx = cmd[1] if cmd else None
        cmd_wz = cmd[2] if cmd else None
        sm_vx = sm[1] if sm else None
        sm_wz = sm[2] if sm else None
        odom_vx = odom[4] if odom else None
        odom_wz = odom[5] if odom else None

        cmd_flip = self.flip_rate(self.cmd_hist, idx=2)
        sm_flip = self.flip_rate(self.smooth_hist, idx=2)
        odom_flip = self.flip_rate(self.odom_hist, idx=5)

        straight_wz = self.straight_metrics()
        jump_xy, jump_yaw = self.map_odom_jump()

        path = self.last_local_plan if self.last_local_plan is not None else self.last_plan
        pm = self.path_metrics(path)
        if pm is None:
            path_dist = None
            path_heading = None
            path_changes = None
        else:
            path_dist, path_heading, path_changes = pm

        scan = self.last_scan or {}
        front = scan.get("front_min")
        left = scan.get("left_min")
        right = scan.get("right_min")
        invalid_ratio = scan.get("invalid_ratio")
        speckle = scan.get("speckle")

        diagnosis = []

        if len(pubs) > 1:
            diagnosis.append("HIGH: 多个 /cmd_vel 发布者抢控制权")
        if cmd_flip > self.args.flip_rate_bad:
            diagnosis.append("HIGH: controller 输出 angular.z 正负频繁跳，控制器/路径/costmap 振荡")
        if sm_flip > self.args.flip_rate_bad:
            diagnosis.append("MED: 平滑后的 angular.z 仍频繁跳，smoother 没压住振荡")
        if odom_flip > self.args.flip_rate_bad:
            diagnosis.append("MED: odom 角速度也左右跳，车体真实在摇或里程计在抖")
        if straight_wz is not None and straight_wz > self.args.straight_bad_wz:
            diagnosis.append("HIGH: 近似直行命令下 odom 仍有明显角速度，优先查底盘/PWM/轮子/打滑")
        if jump_xy is not None and jump_xy > self.args.map_odom_bad_xy:
            diagnosis.append("HIGH: map->odom 平移跳变偏大，AMCL/定位反馈可能在抖")
        if jump_yaw is not None and jump_yaw > math.radians(self.args.map_odom_bad_yaw_deg):
            diagnosis.append("HIGH: map->odom yaw 跳变偏大，AMCL/雷达匹配/TF 可能不稳")
        if path_changes is not None and path_changes >= 2:
            diagnosis.append("MED: 路径局部曲率正负变化，路径本身可能在弯曲/绕障")
        if path_dist is not None and path_dist > self.args.path_bad_dist:
            diagnosis.append("MED: 小车距离路径较远，控制器可能追不回路径")
        if speckle is not None and speckle > self.args.speckle_bad:
            diagnosis.append("MED: 雷达孤立噪点较多，可能生成幽灵障碍")
        if front is not None and front < self.args.front_bad:
            diagnosis.append("MED: 前方近距离有障碍/噪点，局部控制可能在躲避")
        if self.rate("odom") < self.args.odom_min_hz:
            diagnosis.append("MED: odom 频率偏低，闭环反馈延迟可能导致 S")
        if self.rate("scan") < self.args.scan_min_hz:
            diagnosis.append("MED: scan 频率偏低，局部避障反馈延迟可能导致 S")
        if not diagnosis:
            diagnosis.append("当前窗口未发现强异常，继续观察导航中段")

        text_diag = "；".join(diagnosis)

        lines = [
            "",
            f"=== NAV2 S-DIAG t={elapsed:.1f}s ===",
            f"Output: {self.out_dir}",
            f"/cmd_vel publishers={len(pubs)} [{', '.join(pub_names) if pub_names else 'none'}]",
            "Rates Hz: "
            f"cmd={self.rate('cmd'):.1f}, smooth={self.rate('smooth'):.1f}, "
            f"odom={self.rate('odom'):.1f}, amcl={self.rate('amcl'):.1f}, "
            f"scan={self.rate('scan'):.1f}, plan={self.rate('plan'):.1f}, local_plan={self.rate('local_plan'):.1f}",
            f"cmd: vx={fmt(cmd_vx)} wz={fmt(cmd_wz)} | smooth: vx={fmt(sm_vx)} wz={fmt(sm_wz)} | odom: vx={fmt(odom_vx)} wz={fmt(odom_wz)}",
            f"flip_rate: cmd={cmd_flip:.2f}/s smooth={sm_flip:.2f}/s odom={odom_flip:.2f}/s",
            f"straight_mean_abs_odom_wz={fmt(straight_wz)}",
            f"map_odom_jump: xy_max={fmt(jump_xy)} m/s yaw_max={fmt(math.degrees(jump_yaw) if jump_yaw is not None else None)} deg/s",
            f"path: dist={fmt(path_dist)} heading_err={fmt(math.degrees(path_heading) if path_heading is not None else None)} deg curv_changes={path_changes}",
            f"scan: front={fmt(front)} left={fmt(left)} right={fmt(right)} invalid={fmt(invalid_ratio)} speckle={speckle}",
            "Diagnosis: " + text_diag,
        ]

        for line in lines:
            self.log(line)

        self.csv.writerow({
            "t": f"{elapsed:.3f}",
            "cmd_pubs": len(pubs),
            "cmd_vx": fmt(cmd_vx),
            "cmd_wz": fmt(cmd_wz),
            "smooth_vx": fmt(sm_vx),
            "smooth_wz": fmt(sm_wz),
            "odom_vx": fmt(odom_vx),
            "odom_wz": fmt(odom_wz),
            "cmd_flip_rate": f"{cmd_flip:.3f}",
            "smooth_flip_rate": f"{sm_flip:.3f}",
            "odom_flip_rate": f"{odom_flip:.3f}",
            "straight_mean_abs_odom_wz": fmt(straight_wz),
            "map_odom_xy_jump_max": fmt(jump_xy),
            "map_odom_yaw_jump_max_deg": fmt(math.degrees(jump_yaw) if jump_yaw is not None else None),
            "path_dist": fmt(path_dist),
            "path_heading_err_deg": fmt(math.degrees(path_heading) if path_heading is not None else None),
            "path_curv_changes": path_changes,
            "scan_front_min": fmt(front),
            "scan_left_min": fmt(left),
            "scan_right_min": fmt(right),
            "scan_invalid_ratio": fmt(invalid_ratio),
            "scan_speckle": speckle,
            "diagnosis": text_diag,
        })
        self.csv_file.flush()

    def close(self):
        try:
            self.log_file.close()
        except Exception:
            pass
        try:
            self.csv_file.close()
        except Exception:
            pass


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="/tmp/nav2_s_diag")
    p.add_argument("--period", type=float, default=1.0)
    p.add_argument("--cmd-topic", default="/cmd_vel")
    p.add_argument("--smooth-topic", default="/cmd_vel_smoothed")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--amcl-topic", default="/amcl_pose")
    p.add_argument("--scan-topic", default="/scan_filtered")
    p.add_argument("--plan-topic", default="/plan")
    p.add_argument("--local-plan-topic", default="/local_plan")
    p.add_argument("--map-frame", default="map")
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--min-move-vx", type=float, default=0.03)
    p.add_argument("--straight-cmd-wz", type=float, default=0.03)
    p.add_argument("--flip-wz", type=float, default=0.025)
    p.add_argument("--flip-rate-bad", type=float, default=0.8)
    p.add_argument("--straight-bad-wz", type=float, default=0.045)
    p.add_argument("--map-odom-bad-xy", type=float, default=0.05)
    p.add_argument("--map-odom-bad-yaw-deg", type=float, default=8.0)
    p.add_argument("--path-bad-dist", type=float, default=0.18)
    p.add_argument("--speckle-bad", type=int, default=12)
    p.add_argument("--front-bad", type=float, default=0.35)
    p.add_argument("--odom-min-hz", type=float, default=10.0)
    p.add_argument("--scan-min-hz", type=float, default=4.0)
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Nav2SDiag(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log("[INFO] Ctrl+C received. Exit.")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
