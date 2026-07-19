#!/usr/bin/env python3
"""Voice-selectable multi-function demo hub for the RDK X5 robot.

Shared base: calibrated live SLAM + lidar + odom + chassis + Foxglove.
Exclusive modes: online Qwen exploration, offline semantic exploration,
joystick mapping, and saved-map Foxglove click navigation.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class ManagedProcess:
    name: str
    proc: subprocess.Popen
    log_handle: Any
    pgid: int
    pump_thread: threading.Thread | None = field(default=None)

    def group_alive(self) -> bool:
        # Prefer the tracked Popen: orphan children can keep the PGID "alive"
        # after the launcher exits (e.g. stop_all wiped DDS mid-prewarm).
        if self.proc.poll() is not None:
            return False
        try:
            os.killpg(self.pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True


class DemoHub:
    def __init__(self, config_path: Path, record_seconds: int | None = None) -> None:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        self.cfg = raw.get("voice_demo_hub_v1", raw)
        self.repo = Path(self.cfg["repo_root"]).expanduser().resolve()
        self.v1 = Path(self.cfg["v1_root"]).expanduser().resolve()
        if record_seconds is not None:
            self.cfg["voice"]["record_seconds"] = record_seconds
        run_id = time.strftime("%Y%m%d_%H%M%S")
        self.log_dir = self.v1 / "logs" / "voice_demo_hub_v1" / run_id
        self.log_dir.mkdir(parents=True, exist_ok=True)
        latest = self.log_dir.parent / "latest"
        try:
            latest.unlink(missing_ok=True)
            latest.symlink_to(self.log_dir, target_is_directory=True)
        except OSError:
            pass
        self.runtime = Path(f"/tmp/rdk_x5_voice_demo_hub_v1_{os.getenv('USER', 'robot')}")
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.event_file = self.runtime / "voice_events.jsonl"
        self.event_file.write_text("", encoding="utf-8")
        self.event_offset = 0
        self.main_log = (self.log_dir / "main.log").open("a", encoding="utf-8", buffering=1)
        self.base: ManagedProcess | None = None
        self.voice: ManagedProcess | None = None
        self.qwen_prewarm: ManagedProcess | None = None
        self.active: ManagedProcess | None = None
        self.active_mode = "idle"
        self.base_expected = False
        self.qwen_prewarm_expected = False
        self.interrupted_by_seq: dict[int, str] = {}
        self.last_map_valid = False
        self.mapping_wake_pending = False
        self.shutting_down = False
        self.prewarm_runtime = Path(
            f"/tmp/rdk_x5_voice_demo_qwen_prewarm_{os.getenv('USER', 'robot')}"
        )

    def log(self, message: str) -> None:
        line = f"[{time.strftime('%H:%M:%S')}] {message}"
        print(line, flush=True)
        self.main_log.write(line + "\n")

    def path(self, value: str, root: Path | None = None) -> Path:
        p = Path(value)
        return p if p.is_absolute() else (root or self.repo) / p

    def validate(self) -> None:
        required = [
            self.path(self.cfg["base"]["command"]),
            self.path(self.cfg["online"]["script"]),
            self.path(self.cfg["offline"]["script"]),
            self.path(self.cfg["offline"]["config"]),
            self.path(self.cfg["mapping"]["script"]),
            self.path(self.cfg["click_nav"]["script"]),
            self.repo / "voice_interaction/scripts/run_voice_function_event_server_v1.sh",
        ]
        prewarm_cfg = self.cfg.get("qwen_prewarm") or {}
        if prewarm_cfg.get("enabled", True):
            required.append(self.path(prewarm_cfg.get("script", "scripts/demo/prewarm_qwen_ego_v1.sh")))
        missing = [str(p) for p in required if not p.is_file()]
        if missing:
            raise RuntimeError("missing required files:\n  " + "\n  ".join(missing))
        exp2 = self.path(self.cfg["offline"]["script"])
        if "VOICE_DEMO_HUB_V1_REUSE_BASE_PATCH" not in exp2.read_text(encoding="utf-8"):
            raise RuntimeError("exp2 reuse-base patch is not installed; run apply_voice_demo_hub_v1.sh")

    def _pump_child_log(self, name: str, stream: Any, handle: Any) -> None:
        """Mirror child stdout/stderr to both the log file and the hub terminal."""
        prefix = f"[{name}] "
        try:
            for line in iter(stream.readline, ""):
                handle.write(line)
                handle.flush()
                print(f"{prefix}{line.rstrip()}", flush=True)
        except Exception:
            pass
        finally:
            try:
                stream.close()
            except Exception:
                pass

    def start_process(
        self,
        name: str,
        argv: list[str],
        *,
        env_extra: dict[str, str] | None = None,
        holder: bool = False,
    ) -> ManagedProcess:
        env = os.environ.copy()
        env.update(env_extra or {})
        log_path = self.log_dir / f"{name}.log"
        handle = log_path.open("w", encoding="utf-8", buffering=1)
        if holder:
            command = shlex.join(argv)
            argv = [
                "bash",
                "-lc",
                f"{command}; rc=$?; if [ $rc -ne 0 ]; then exit $rc; fi; "
                f"echo '[hub-holder] {name} launcher returned; keeping process group alive'; "
                "while true; do sleep 3600; done",
            ]
        proc = subprocess.Popen(
            argv,
            cwd=self.repo,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
            bufsize=1,
        )
        pump = threading.Thread(
            target=self._pump_child_log,
            args=(name, proc.stdout, handle),
            name=f"hub-log-{name}",
            daemon=True,
        )
        pump.start()
        managed = ManagedProcess(
            name=name, proc=proc, log_handle=handle, pgid=proc.pid, pump_thread=pump
        )
        self.log(f"START {name} pid={proc.pid} log={log_path}")
        return managed

    def ros_setup_prefix(self) -> str:
        # Must match the SLAM stack DDS profile (UDP-only). Default FastDDS SHM
        # participants cannot reliably discover publishers started with fastdds_no_shm.xml.
        dds = self.repo / "scripts/lib/ros_dds_env.sh"
        return (
            "set +u; "
            "if [ -f /opt/tros/humble/setup.bash ]; then source /opt/tros/humble/setup.bash; "
            "elif [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi; "
            "[ -f $HOME/ydlidar_ws/install/setup.bash ] && source $HOME/ydlidar_ws/install/setup.bash; "
            f"PROJECT_DIR={shlex.quote(str(self.repo))}; "
            f"source {shlex.quote(str(dds))}; "
            "attach_ros_dds_env; "
            "set -u; "
        )

    def ros_shell(self, command: str, timeout: float = 6.0) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["bash", "-lc", self.ros_setup_prefix() + command],
            cwd=self.repo,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )

    def publisher_count(self, topic: str) -> int:
        try:
            # Keep probes short: long CLI discovery under UDP FastDDS was blocking
            # start_online for ~50s and falsely reporting 0 publishers on a healthy stack.
            out = self.ros_shell(
                f"timeout 2 ros2 topic info {shlex.quote(topic)} 2>/dev/null || true",
                4,
            ).stdout
        except subprocess.TimeoutExpired:
            return -1  # unknown / flake — do not treat as missing
        match = re.search(r"Publisher count:\s*(\d+)", out)
        return int(match.group(1)) if match else -1

    def base_log_reports_started(self) -> bool:
        log_path = self.log_dir / "base_slam.log"
        try:
            text = log_path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            return False
        return "===== SLAM live stack started =====" in text

    def base_log_tail(self, n: int = 25) -> str:
        log_path = self.log_dir / "base_slam.log"
        try:
            lines = log_path.read_text(encoding="utf-8", errors="ignore").splitlines()
        except OSError:
            return f"(missing {log_path})"
        return "\n".join(lines[-n:]) if lines else f"(empty {log_path})"

    def wait_base_tf_chain(self) -> None:
        """Use the proven rclpy wait_tf_chain helper.

        Direct `tf2_echo map base_link` often fails on cold start / extrapolation even
        when map<-odom and odom<-base_link are healthy (see wait_tf_chain.py).
        """
        timeout = float(self.cfg["base"]["tf_timeout_sec"])
        script = self.repo / "scripts/nav/wait_tf_chain.py"
        if not script.is_file():
            raise RuntimeError(f"missing TF waiter: {script}")
        cmd = (
            f"python3 {shlex.quote(str(script))} "
            f"--map-frame map --odom-frame odom --base-frame base_link "
            f"--timeout {timeout} --need-ok 3 --poll 0.5"
        )
        self.log("WAITING TF chain map<-odom<-base_link via wait_tf_chain.py")
        proc = subprocess.Popen(
            ["bash", "-lc", self.ros_setup_prefix() + cmd],
            cwd=self.repo,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert proc.stdout is not None
        try:
            for line in proc.stdout:
                text = line.rstrip()
                if text:
                    self.log(text)
                if self.shutting_down:
                    proc.terminate()
                    break
                if self.base and not self.base.group_alive():
                    proc.terminate()
                    raise RuntimeError(
                        "base_slam exited before TF ready; last log:\n" + self.base_log_tail()
                    )
        finally:
            try:
                proc.stdout.close()
            except Exception:
                pass
        try:
            rc = proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)
            rc = 1
        if self.shutting_down:
            raise RuntimeError("hub shutdown during TF wait")
        if rc != 0:
            raise RuntimeError(
                "TF chain timeout (map<-odom<-base_link)\n--- base_slam.log ---\n"
                + self.base_log_tail()
            )
        self.log("READY base topics + TF map<-odom<-base_link")

    def wait_base_ready(self) -> None:
        deadline = time.monotonic() + float(self.cfg["base"]["startup_timeout_sec"])
        pending = set(self.cfg["base"]["ready_topics"])
        last_report = 0.0
        self.log(f"WAITING base topics: {sorted(pending)}")
        while pending and time.monotonic() < deadline:
            if self.shutting_down:
                raise RuntimeError("hub shutdown during base topic wait")
            if self.base and not self.base.group_alive():
                raise RuntimeError(
                    "base_slam exited before topics ready; last log:\n" + self.base_log_tail()
                )
            still_missing = set()
            for topic in pending:
                count = self.publisher_count(topic)
                if count > 0:
                    continue
                still_missing.add(topic)
            pending = still_missing
            # Launcher now waits for /map publisher_count>0 before printing the
            # started banner. Hub-side ros2 CLI often cannot discover those same
            # publishers (UDP FastDDS flake) and used to timeout then tear down
            # a healthy stack. Trust the banner and gate on TF instead.
            if pending and self.base_log_reports_started():
                self.log(
                    "launcher confirmed SLAM started (/map publisher in-stack); "
                    f"hub CLI still missing={sorted(pending)} — skipping to TF wait"
                )
                pending.clear()
                break
            now = time.monotonic()
            if pending and now - last_report >= 5.0:
                self.log(
                    f"WAITING base topics still missing={sorted(pending)} "
                    f"({int(deadline - now)}s left)"
                )
                last_report = now
            if pending:
                time.sleep(1)
        if pending:
            raise RuntimeError(
                f"base topics timeout: {sorted(pending)}\n--- base_slam.log ---\n"
                + self.base_log_tail()
            )
        self.wait_base_tf_chain()

    def start_voice(self) -> None:
        if self.voice and self.voice.group_alive():
            return
        runner = self.repo / "voice_interaction/scripts/run_voice_function_event_server_v1.sh"
        kws_threads = int((self.cfg.get("voice") or {}).get("kws_threads", 1))
        self.voice = self.start_process(
            "voice",
            ["bash", str(runner), "--event-file", str(self.event_file)],
            env_extra={
                "VOICE_RECORD_SECONDS": str(self.cfg["voice"]["record_seconds"]),
                "VOICE_FUNCTION_RECORD_SECONDS": str(self.cfg["voice"]["record_seconds"]),
                "VOICE_ENV_FILE": str(self.repo / "voice_interaction/.env"),
                "VOICE_KWS_THREADS": str(kws_threads),
            },
        )

    def start_base(self, reset: bool = False) -> None:
        if self.base and self.base.group_alive() and not reset:
            self.base_expected = True
            return
        if reset:
            self.stop_base()
        script = self.path(self.cfg["base"]["command"])
        # If ego prewarm is still up, never wipe FastDDS shm under it.
        attach_only = "1" if (
            self.qwen_prewarm and self.qwen_prewarm.group_alive()
        ) else "0"
        self.base = self.start_process(
            "base_slam",
            ["bash", str(script)],
            env_extra={
                "ATTACH_ROS_DDS_ONLY": attach_only,
                "VOICE_DEMO_ATTACH_DDS": attach_only,
            },
        )
        self.base_expected = True
        self.wait_base_ready()

    def signal_process(
        self, managed: ManagedProcess | None, sig: int, *, process_only: bool = False
    ) -> None:
        if not managed or not managed.group_alive():
            return
        try:
            if process_only and managed.proc.poll() is None:
                # Let launchers with traps save maps and clean their own children first.
                os.kill(managed.proc.pid, sig)
            else:
                os.killpg(managed.pgid, sig)
        except ProcessLookupError:
            pass

    def finish_termination(self, managed: ManagedProcess | None, timeout: float) -> None:
        if not managed:
            return
        deadline = time.monotonic() + timeout
        while managed.group_alive() and time.monotonic() < deadline:
            time.sleep(0.2)
        if managed.group_alive():
            try:
                os.killpg(managed.pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        try:
            managed.proc.wait(timeout=2)
        except (subprocess.TimeoutExpired, ChildProcessError):
            pass
        try:
            if managed.proc.stdout is not None:
                managed.proc.stdout.close()
        except Exception:
            pass
        if managed.pump_thread is not None:
            managed.pump_thread.join(timeout=1.5)
        try:
            managed.log_handle.close()
        except Exception:
            pass

    def terminate_process(
        self,
        managed: ManagedProcess | None,
        sig: int,
        timeout: float,
        *,
        process_only: bool = False,
    ) -> None:
        self.signal_process(managed, sig, process_only=process_only)
        self.finish_termination(managed, timeout)

    def stop_base(self) -> None:
        self.base_expected = False
        if self.base:
            self.log("STOP shared base stack")
            self.terminate_process(self.base, signal.SIGTERM, 12)
            self.base = None
        # Serial (/dev/rosmaster) + lidar need a beat before relaunch.
        time.sleep(2.5)

    def qwen_prewarm_enabled(self) -> bool:
        return bool((self.cfg.get("qwen_prewarm") or {}).get("enabled", True))

    def qwen_prewarm_marker(self) -> Path:
        return self.prewarm_runtime / "ready.marker"

    def qwen_prewarm_is_ready(self) -> bool:
        if not self.qwen_prewarm or not self.qwen_prewarm.group_alive():
            return False
        marker = self.qwen_prewarm_marker()
        if marker.is_file() and "ready" in marker.read_text(encoding="utf-8", errors="ignore"):
            return True
        return self.publisher_count("/qwen_vln/result_json") > 0

    def pause_qwen_prewarm(self) -> None:
        if not self.qwen_prewarm_is_ready():
            return
        try:
            self.ros_shell(
                "timeout 3 ros2 topic pub --once /qwen_vln/command std_msgs/msg/String "
                "\"{data: 'pause'}\" >/dev/null 2>&1 || true",
                5,
            )
        except subprocess.TimeoutExpired:
            pass

    def stop_qwen_prewarm(self) -> None:
        self.qwen_prewarm_expected = False
        if self.qwen_prewarm:
            self.log("STOP qwen ego prewarm")
            self.terminate_process(self.qwen_prewarm, signal.SIGTERM, 10)
            self.qwen_prewarm = None
        try:
            self.qwen_prewarm_marker().unlink(missing_ok=True)
        except OSError:
            pass
        time.sleep(0.5)

    def start_qwen_prewarm(self, *, force: bool = False) -> None:
        if not self.qwen_prewarm_enabled():
            return
        if self.qwen_prewarm and self.qwen_prewarm.group_alive() and self.qwen_prewarm_is_ready() and not force:
            self.qwen_prewarm_expected = True
            return
        if force or (self.qwen_prewarm and not self.qwen_prewarm.group_alive()):
            self.stop_qwen_prewarm()
        cfg = self.cfg.get("qwen_prewarm") or {}
        script = self.path(cfg.get("script", "scripts/demo/prewarm_qwen_ego_v1.sh"))
        instruction = str(cfg.get("instruction") or self.cfg["online"].get("task") or "find the bottle")
        timeout = float(cfg.get("ready_timeout_sec", 90))
        self.prewarm_runtime.mkdir(parents=True, exist_ok=True)
        self.log(f"Boot/idle: prewarming ego Qwen (instruction={instruction!r})...")
        self.qwen_prewarm = self.start_process(
            "qwen_prewarm",
            ["bash", str(script)],
            env_extra={
                "ROBOT_PROJECT_DIR": str(self.repo),
                "V1_ROOT": str(self.v1),
                "VOICE_ENV_FILE": str(self.repo / "voice_interaction/.env"),
                "QWEN_PREWARM_INSTRUCTION": instruction,
                "QWEN_PREWARM_RUNTIME_DIR": str(self.prewarm_runtime),
                "QWEN_PREWARM_WAIT_SEC": str(int(timeout)),
                "CAMERA_BACKEND": str(cfg.get("camera_backend", "opencv")),
                "CAMERA_WIDTH": str(cfg.get("camera_width", 960)),
                "CAMERA_HEIGHT": str(cfg.get("camera_height", 540)),
                "CAMERA_FPS": str(cfg.get("camera_fps", 10)),
                "CAMERA_DEV": str(cfg.get("camera_dev", "/dev/video0")),
            },
        )
        self.qwen_prewarm_expected = True
        deadline = time.monotonic() + timeout
        last_report = 0.0
        while time.monotonic() < deadline:
            if self.shutting_down:
                raise RuntimeError("hub shutdown during qwen prewarm")
            if self.qwen_prewarm and not self.qwen_prewarm.group_alive():
                raise RuntimeError(
                    "qwen_prewarm exited early; inspect qwen_prewarm.log"
                )
            if self.qwen_prewarm_is_ready():
                self.log("READY ego Qwen prewarm (camera + qwen paused)")
                return
            now = time.monotonic()
            if now - last_report >= 5.0:
                self.log(f"WAITING qwen prewarm ({int(deadline - now)}s left)")
                last_report = now
            time.sleep(1)
        raise RuntimeError("qwen prewarm timeout; inspect qwen_prewarm.log")

    def ensure_qwen_prewarm(self) -> None:
        if not self.qwen_prewarm_enabled():
            return
        try:
            self.start_qwen_prewarm()
        except Exception as exc:
            self.log(f"WARN qwen prewarm unavailable: {exc}")
            self.qwen_prewarm_expected = False

    def publish_zero(self) -> None:
        # Publish all stop commands concurrently. Sequential ros2 CLI startup can waste
        # several seconds, which is precisely when a demo robot should stop, not ponder.
        jobs = []
        for topic in self.cfg["safety"]["publish_zero_topics"]:
            jobs.append(
                "(timeout 1.5 ros2 topic pub --once "
                f"{shlex.quote(topic)} geometry_msgs/msg/Twist '{{}}' "
                ">/dev/null 2>&1 || true) &"
            )
        try:
            self.ros_shell(" ".join(jobs) + " wait", 4)
        except subprocess.TimeoutExpired:
            pass

    def kill_patterns(self, patterns: list[str]) -> None:
        for pattern in patterns:
            subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)
        time.sleep(1)
        for pattern in patterns:
            subprocess.run(["pkill", "-KILL", "-f", pattern], check=False)

    def map_paths(self) -> tuple[Path, Path, Path]:
        cfg = self.cfg["mapping"]
        map_dir = self.path(cfg["map_dir"])
        stem = map_dir / cfg["map_name"]
        return stem.with_suffix(".yaml"), stem.with_suffix(".pgm"), stem.with_suffix(".png")

    def clear_demo_map(self) -> None:
        for p in self.map_paths():
            p.unlink(missing_ok=True)
        self.last_map_valid = False

    def save_live_demo_map(self) -> bool:
        """Save current live /map into the demo map paths (hub-owned, DDS-attached)."""
        yaml_path, pgm_path, _ = self.map_paths()
        map_dir = yaml_path.parent
        map_dir.mkdir(parents=True, exist_ok=True)
        stem = self.cfg["mapping"]["map_name"]
        tmp = map_dir / f"{stem}.hub_tmp_{int(time.time())}"
        self.log(f"SAVE live /map -> {yaml_path}")
        if self.publisher_count("/map") < 1:
            self.log("ERROR /map has no publisher; cannot save")
            return False
        try:
            result = self.ros_shell(
                f"timeout 35 ros2 run nav2_map_server map_saver_cli "
                f"-t /map -f {shlex.quote(str(tmp))} --ros-args "
                f"-p save_map_timeout:=25.0",
                45,
            )
        except subprocess.TimeoutExpired:
            self.log("ERROR map_saver_cli timed out")
            for ext in (".pgm", ".yaml"):
                (Path(str(tmp) + ext)).unlink(missing_ok=True)
            return False
        tmp_pgm = Path(str(tmp) + ".pgm")
        tmp_yaml = Path(str(tmp) + ".yaml")
        if result.returncode != 0 or not tmp_pgm.is_file() or not tmp_yaml.is_file():
            self.log(
                f"ERROR map_saver_cli failed rc={result.returncode}; "
                f"out={(result.stdout or '')[-400:]}"
            )
            tmp_pgm.unlink(missing_ok=True)
            tmp_yaml.unlink(missing_ok=True)
            return False
        tmp_pgm.replace(pgm_path)
        # Rewrite image basename inside yaml to the final .pgm name.
        try:
            text = tmp_yaml.read_text(encoding="utf-8")
            text = re.sub(
                r"(?m)^image:\s*.*$",
                f"image: {pgm_path.name}",
                text,
                count=1,
            )
            yaml_path.write_text(text, encoding="utf-8")
            tmp_yaml.unlink(missing_ok=True)
        except OSError as exc:
            self.log(f"ERROR writing map yaml: {exc}")
            return False
        self.last_map_valid = yaml_path.is_file() and pgm_path.is_file()
        self.log(f"OK map saved map_saved={self.last_map_valid}")
        return self.last_map_valid

    def stop_mapping_session(self, reason: str, *, save: bool) -> bool:
        """Stop joystick mapping addon; optionally save live map via hub (keep shared SLAM)."""
        self.mapping_wake_pending = False
        saved = False
        if save:
            saved = self.save_live_demo_map()
            self.last_map_valid = saved
        proc = self.active if self.active_mode == "mapping" else None
        if proc:
            self.log(f"STOP mapping launcher ({reason})")
            self.signal_process(proc, signal.SIGINT, process_only=True)
            self.publish_zero()
            self.finish_termination(proc, 12.0)
        # Ensure joy nodes are gone even if launcher trap was slow/killed.
        for pattern in ("teleop_twist_joy", "joy_node --ros-args", "pose_memory_node.py"):
            subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)
        time.sleep(0.5)
        self.active = None
        self.active_mode = "idle"
        self.publish_zero()
        if not save:
            self.log(f"mapping stopped without save ({reason})")
        else:
            self.log(f"mapping stopped; map_saved={saved}")
        return saved

    def reset_live_slam_map(self, *, reason: str, allow_full_restart: bool = True) -> bool:
        """Clear slam_toolbox occupancy in-place; keep lidar/chassis/Foxglove running.

        Full base restart is only a fallback when Clear fails and allow_full_restart.
        """
        self.ensure_base_for_mode()
        self.clear_demo_map()
        self.log(f"RESET live slam map in-place ({reason})")
        try:
            result = self.ros_shell(
                "timeout 8 ros2 service call /slam_toolbox/clear slam_toolbox/srv/Clear '{}' "
                ">/dev/null 2>&1",
                12,
            )
        except subprocess.TimeoutExpired:
            result = None
        if result is not None and result.returncode == 0:
            # Brief settle so callers waiting on /map see a fresh grid.
            time.sleep(1.0)
            if self.base_tf_healthy(12.0):
                self.log("OK live slam map cleared (stack kept)")
                return True
            self.log("WARN map Clear returned OK but TF unhealthy")
        else:
            self.log("WARN /slam_toolbox/clear failed or unavailable")
        if not allow_full_restart:
            return False
        self.log("FALLBACK: restart shared base to obtain a fresh live map")
        self.start_base(reset=True)
        return True

    def ensure_base_for_mode(self) -> None:
        if not self.base or not self.base.group_alive():
            self.start_base()

    def base_tf_healthy(self, timeout: float = 18.0) -> bool:
        """Return True when map<-odom<-base_link is usable (not just /map topic present)."""
        script = self.repo / "scripts/nav/wait_tf_chain.py"
        if not script.is_file():
            return self.publisher_count("/map") > 0 and self.publisher_count("/odom") > 0
        try:
            result = self.ros_shell(
                f"python3 {shlex.quote(str(script))} "
                f"--map-frame map --odom-frame odom --base-frame base_link "
                f"--timeout {timeout} --need-ok 2 --poll 0.5",
                timeout + 12,
            )
        except subprocess.TimeoutExpired:
            return False
        return result.returncode == 0

    def _missing_base_topics(self) -> list[str]:
        """Return topics with confirmed zero publishers. CLI flakes (-1) are ignored."""
        missing: list[str] = []
        for topic in self.cfg["base"]["ready_topics"]:
            count = self.publisher_count(topic)
            if count == 0:
                missing.append(topic)
        return missing

    def repair_base_tf_if_needed(self, reason: str) -> None:
        """Verify shared SLAM; never tear it down on flaky probes.

        Auto restart of the full lidar/chassis/SLAM stack was the main demo
        breaker: ros2 CLI / short TF probes often fail while the stack is fine,
        then STOP+START cascades and blocks voice commands for minutes.
        Only start_base when the process is actually dead.
        """
        if not self.base or not self.base.group_alive():
            self.log(f"base process dead before {reason}; starting shared SLAM")
            self.start_base()
            return
        self.log(f"CHECK base TF before {reason}")
        if self.base_tf_healthy(6.0):
            self.log("OK base TF map<-odom<-base_link")
            return
        self.log(f"WARN base TF probe failed before {reason}; settle and recheck")
        time.sleep(2.0)
        if self.base_tf_healthy(10.0):
            self.log("OK base TF map<-odom<-base_link (after settle)")
            return
        self.log(
            f"WARN base TF unhealthy before {reason}; keeping shared SLAM alive "
            "(no auto-restart)"
        )

    def start_online(self) -> None:
        self.ensure_base_for_mode()
        self.repair_base_tf_if_needed("pre-online")
        if not self.base_tf_healthy(12.0):
            raise RuntimeError(
                "refuse online: shared SLAM TF not ready (stack kept; retry command)"
            )
        self.ensure_qwen_prewarm()
        cfg = self.cfg["online"]
        script = self.path(cfg["script"])
        args = ["bash", str(script), "--no-slam", "--no-foxglove", "--task", cfg["task"]]
        if cfg.get("motion", True):
            args.insert(2, "--motion")
        else:
            args.insert(2, "--dry-run")
        reuse = "1" if self.qwen_prewarm_is_ready() else "0"
        self.active = self.start_process(
            "online",
            args,
            env_extra={
                "REUSE_QWEN_PREWARM": reuse,
                "QWEN_PREWARM_RUNTIME_DIR": str(self.prewarm_runtime),
                # Online --no-slam must not die on flaky ros2 topic info; hub just
                # verified TF on the shared base stack.
                "VOICE_DEMO_HUB_BASE_READY": "1",
            },
        )
        self.active_mode = "online"
        self.log(f"ACTIVE online task={cfg['task']} reuse_qwen_prewarm={reuse}")

    def start_offline(self) -> None:
        # Offline YOLO stack needs exclusive camera ownership.
        self.stop_qwen_prewarm()
        self.ensure_base_for_mode()
        # After online, slam_toolbox may still publish /map while map TF is gone.
        self.repair_base_tf_if_needed("offline")
        cfg = self.cfg["offline"]
        self.active = self.start_process(
            "offline",
            ["bash", str(self.path(cfg["script"])), str(self.path(cfg["config"])), cfg["task"]],
            env_extra={
                "REUSE_BASE_STACK": "1",
                "NAV_ONLY": "0",
                "JOY_ENABLED": "1" if cfg.get("joy_enabled", True) else "0",
                "MAP_NAME": "voice_demo_offline",
            },
            holder=True,
        )
        self.active_mode = "offline"
        time.sleep(float(cfg.get("startup_grace_sec", 6)))
        if not self.active.group_alive():
            raise RuntimeError("offline launcher exited during startup; inspect offline.log")
        self.log(f"ACTIVE offline task={cfg['task']}")

    def start_mapping(self) -> None:
        # Reuse boot-time calibrated SLAM; mapping only adds joy/teleop + map save.
        self.ensure_base_for_mode()
        # Fresh occupancy for this mapping session without tearing down sensors.
        self.reset_live_slam_map(reason="start_mapping", allow_full_restart=False)
        cfg = self.cfg["mapping"]
        self.active = self.start_process(
            "mapping",
            ["bash", str(self.path(cfg["script"]))],
            env_extra={
                "MAP_NAME": cfg["map_name"],
                "REUSE_BASE_STACK": "1",
            },
        )
        self.active_mode = "mapping"
        self.log("ACTIVE mapping: reuse shared SLAM; joystick drive; say ‘完成建图’ to save")

    def start_click_nav(self) -> None:
        yaml_path, pgm_path, _ = self.map_paths()
        if not yaml_path.is_file() or not pgm_path.is_file():
            raise RuntimeError(f"saved map missing: {yaml_path} / {pgm_path}")
        # Saved-map Nav2 localization cannot coexist with live slam_toolbox on /map.
        self.stop_qwen_prewarm()
        self.stop_base()
        cfg = self.cfg["click_nav"]
        self.active = self.start_process(
            "click_nav",
            ["bash", str(self.path(cfg["script"]))],
            env_extra={"MAP_YAML": str(yaml_path)},
        )
        self.active_mode = "click_nav"
        self.log("ACTIVE click_nav: connect Foxglove and publish the goal point/pose")

    def stop_active(self, reason: str) -> str:
        previous = self.active_mode
        if previous == "idle":
            self.publish_zero()
            return previous
        self.log(f"INTERRUPT mode={previous} reason={reason}")
        proc = self.active
        if previous == "mapping":
            # Hub saves /map itself (joy-script trap is unreliable under REUSE + SIGKILL).
            self.stop_mapping_session(reason, save=True)
            return previous
        elif previous == "click_nav":
            self.signal_process(proc, signal.SIGINT, process_only=True)
            self.publish_zero()
            self.finish_termination(proc, float(self.cfg["click_nav"]["stop_timeout_sec"]))
        elif previous == "online":
            self.signal_process(proc, signal.SIGINT, process_only=True)
            self.publish_zero()
            self.finish_termination(proc, float(self.cfg["online"]["stop_timeout_sec"]))
            self.kill_patterns(self.cfg["safety"]["online_fallback_patterns"])
            self.pause_qwen_prewarm()
            # Do not auto-restart shared SLAM after online — probes are flaky and
            # tear-down cascades block the next voice command.
            try:
                self.repair_base_tf_if_needed("post-online")
            except Exception as exc:
                self.log(f"WARN post-online base check failed: {exc}")
        elif previous == "offline":
            # exp2 returns after spawning children, so its holder owns the process group.
            self.signal_process(proc, signal.SIGTERM, process_only=False)
            self.publish_zero()
            self.finish_termination(proc, float(self.cfg["offline"]["stop_timeout_sec"]))
            self.kill_patterns(self.cfg["safety"]["offline_fallback_patterns"])
        self.active = None
        self.active_mode = "idle"
        self.publish_zero()
        if previous == "offline":
            self.ensure_qwen_prewarm()
        return previous

    @staticmethod
    def normalize_command(text: str) -> str:
        text = text.lower().strip()
        text = re.sub(r"[，。！？、,.!?;；:\s]+", "", text)
        return text

    def route(self, text: str) -> str | None:
        normalized = self.normalize_command(text)
        if not normalized:
            return None
        # Priority matters: “断网探索” contains “网”, so offline is checked before online.
        order = ["stop", "finish_mapping", "offline", "online", "mapping", "click_nav", "reset", "status"]
        commands = self.cfg["voice"]["commands"]
        for action in order:
            for phrase in commands.get(action, []):
                p = self.normalize_command(str(phrase))
                if p and (normalized == p or p in normalized):
                    return action
        return None

    def execute_action(self, action: str | None, text: str, interrupted_mode: str) -> None:
        if action is None:
            # Mapping soft-wake: keep joystick mapping alive on ASR garbage.
            if interrupted_mode == "mapping" and self.active_mode == "mapping":
                self.mapping_wake_pending = False
                self.log(f"UNRECOGNIZED command={text!r}; continue mapping")
                return
            self.log(f"UNRECOGNIZED command={text!r}; stay idle")
            if interrupted_mode == "click_nav":
                self.start_base()
                self.ensure_qwen_prewarm()
            return
        self.log(f"COMMAND action={action} text={text!r} interrupted={interrupted_mode}")
        if action == "stop":
            if self.active_mode == "mapping":
                self.stop_mapping_session("voice_stop", save=True)
            self.start_base()
            self.ensure_qwen_prewarm()
            self.log("READY idle")
        elif action == "reset":
            if self.active_mode == "mapping":
                self.stop_mapping_session("voice_reset", save=False)
            # Prefer in-place slam_toolbox Clear; full restart only if Clear/TF fails.
            self.reset_live_slam_map(reason="voice_reset", allow_full_restart=True)
            self.ensure_qwen_prewarm()
            self.log("READY idle with a fresh live map")
        elif action == "online":
            if self.active_mode == "mapping":
                self.stop_mapping_session("switch_online", save=True)
            self.start_online()
        elif action == "offline":
            if self.active_mode == "mapping":
                self.stop_mapping_session("switch_offline", save=True)
            self.start_offline()
        elif action == "mapping":
            if self.active_mode == "mapping":
                self.mapping_wake_pending = False
                self.reset_live_slam_map(reason="remap_while_mapping", allow_full_restart=False)
                self.log("ACTIVE mapping: live map cleared; continue joystick mapping")
                return
            self.start_mapping()
        elif action in {"finish_mapping", "click_nav"}:
            if self.active_mode == "mapping" or interrupted_mode == "mapping":
                if self.active_mode == "mapping":
                    self.stop_mapping_session("finish_mapping", save=True)
                if not self.last_map_valid:
                    # One more attempt from live /map (session may already be idle).
                    self.save_live_demo_map()
                if not self.last_map_valid:
                    self.log("ERROR mapping save failed; keep shared base and stay idle")
                    self.ensure_base_for_mode()
                    self.ensure_qwen_prewarm()
                    return
                self.start_click_nav()
                return
            yaml_path, pgm_path, _ = self.map_paths()
            if not yaml_path.is_file() or not pgm_path.is_file():
                # Idle "完成建图/导航": still try saving whatever live map exists.
                if self.publisher_count("/map") > 0 and self.save_live_demo_map():
                    self.start_click_nav()
                    return
                self.log(
                    "ERROR no saved map yet; say ‘开始建图’ then ‘完成建图’ first "
                    f"(missing {yaml_path.name})"
                )
                self.ensure_base_for_mode()
                self.ensure_qwen_prewarm()
            else:
                self.start_click_nav()
        elif action == "status":
            self.log(
                f"STATUS base_alive={bool(self.base and self.base.group_alive())} "
                f"qwen_prewarm={self.qwen_prewarm_is_ready()} mode={self.active_mode}"
            )
            if interrupted_mode == "mapping" and self.active_mode == "mapping":
                self.mapping_wake_pending = False
                return
            if interrupted_mode == "click_nav":
                self.start_base()
                self.ensure_qwen_prewarm()

    def read_events(self) -> list[dict]:
        events: list[dict] = []
        if not self.event_file.exists():
            return events
        # Defensive recovery if an operator or an old voice process truncated the file.
        if self.event_file.stat().st_size < self.event_offset:
            self.log("WARN voice event file was truncated; reset read offset")
            self.event_offset = 0
        with self.event_file.open("r", encoding="utf-8") as f:
            f.seek(self.event_offset)
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    self.log(f"WARN bad voice event line: {line[:120]!r}")
            self.event_offset = f.tell()
        return events

    def handle_event(self, event: dict) -> None:
        seq = int(event.get("seq", 0))
        kind = event.get("event")
        if kind == "wake":
            # Mapping soft-wake: stop motors only; keep SLAM+joy until the command
            # arrives so ASR garbage ("心电图") cannot tear down an unfinished map.
            if self.active_mode == "mapping":
                self.publish_zero()
                self.mapping_wake_pending = True
                self.interrupted_by_seq[seq] = "mapping"
                self.log(
                    "VOICE wake during mapping: motion paused, map session kept; "
                    "say 完成建图 / 停止 / 其他功能"
                )
                return
            previous = self.stop_active(f"wake_seq_{seq}")
            self.interrupted_by_seq[seq] = previous
            self.log("VOICE wake accepted; current function has been stopped, recording command...")
        elif kind == "command":
            text = str(event.get("text", ""))
            previous = self.interrupted_by_seq.pop(seq, "idle")
            action = self.route(text)
            try:
                self.execute_action(action, text, previous)
            except Exception as exc:
                self.log(f"ACTION ERROR: {exc}")
                self.publish_zero()
                try:
                    self.start_base()
                except Exception as base_exc:
                    self.log(f"BASE RECOVERY ERROR: {base_exc}")
        elif kind in {"error", "fatal"}:
            self.log(f"VOICE {kind}: {event.get('error', 'unknown')}")

    def check_health(self) -> None:
        if self.voice and not self.voice.group_alive() and not self.shutting_down:
            self.log("VOICE process died; restart persistent KWS")
            try:
                self.voice.log_handle.close()
            except Exception:
                pass
            self.voice = None
            self.start_voice()
        if self.base_expected and self.base and not self.base.group_alive() and self.active_mode != "click_nav":
            self.log("BASE process died; stop active mode and recover")
            self.stop_active("base_failure")
            self.base = None
            self.start_base()
            if self.active_mode == "idle":
                self.ensure_qwen_prewarm()
        if (
            self.qwen_prewarm_expected
            and self.active_mode == "idle"
            and self.qwen_prewarm
            and not self.qwen_prewarm.group_alive()
            and not self.shutting_down
        ):
            self.log("QWEN prewarm died; restarting in idle")
            self.qwen_prewarm = None
            self.ensure_qwen_prewarm()
        if self.active_mode in {"online", "offline", "mapping", "click_nav"} and self.active and not self.active.group_alive():
            self.log(f"MODE {self.active_mode} exited unexpectedly; returning to idle")
            self.active.log_handle.close()
            old = self.active_mode
            self.active = None
            self.active_mode = "idle"
            self.publish_zero()
            if old == "online":
                self.pause_qwen_prewarm()
                try:
                    self.repair_base_tf_if_needed("post-online-exit")
                except Exception as exc:
                    self.log(f"WARN post-online-exit base check failed: {exc}")
            if old == "offline":
                self.ensure_qwen_prewarm()
            if old in {"mapping", "click_nav"}:
                self.start_base()
                self.ensure_qwen_prewarm()

    def run(self) -> int:
        self.validate()
        self.log("Boot: starting persistent voice KWS...")
        self.start_voice()
        self.log("Boot: starting shared calibrated base SLAM...")
        self.start_base()
        try:
            self.start_qwen_prewarm()
        except Exception as exc:
            self.log(f"WARN qwen prewarm failed at boot: {exc}; online will cold-start if needed")
            self.qwen_prewarm_expected = False
        # Discard any accidental wake command spoken before hardware readiness.
        self.event_offset = self.event_file.stat().st_size
        self.log("=" * 68)
        self.log("READY：说唤醒词后，在 5 秒内说固定功能命令")
        self.log("命令：联网探索 / 断网探索 / 开始建图 / 完成建图 / 点击导航 / 停止")
        self.log("任何模式运行时再次唤醒，会先立即停车并停止该模式，再识别新命令")
        if self.qwen_prewarm_is_ready():
            self.log("ego Qwen 已预热，联网探索将复用相机/Qwen")
        self.log("=" * 68)
        poll = float(self.cfg["voice"].get("event_poll_sec", 0.1))
        while not self.shutting_down:
            for event in self.read_events():
                self.handle_event(event)
            self.check_health()
            time.sleep(poll)
        return 0

    def cleanup(self) -> None:
        if self.shutting_down:
            return
        self.shutting_down = True
        self.log("Hub shutdown")
        try:
            self.stop_active("hub_shutdown")
        except Exception as exc:
            self.log(f"active cleanup error: {exc}")
        self.publish_zero()
        self.stop_qwen_prewarm()
        self.stop_base()
        if self.voice:
            self.terminate_process(self.voice, signal.SIGTERM, 5)
            self.voice = None
        self.main_log.close()


def acquire_lock(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError(f"another demo hub is running: {path}") from exc
    return handle


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("/root/rdk_x5_vln_robot/configs/voice_demo_hub_v1.yaml"),
    )
    parser.add_argument("--record-seconds", type=int, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    lock = acquire_lock(Path("/tmp/rdk_x5_voice_demo_hub_v1.lock"))
    hub = DemoHub(args.config.resolve(), args.record_seconds)

    def request_stop(_signum, _frame):
        hub.shutting_down = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        return hub.run()
    finally:
        # cleanup() checks the flag, so clear it once to perform actual cleanup.
        hub.shutting_down = False
        hub.cleanup()
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
