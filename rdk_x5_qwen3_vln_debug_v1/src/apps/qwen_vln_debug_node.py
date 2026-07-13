#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import sys
import threading
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, String

from qwen_vln.prompt_manager import PromptManager
from qwen_vln.qwen_client import ClientConfig, QwenVisionClient, pixel_to_norm1000
from qwen_vln.state_machine import NavigationStateMachine, StateMachineConfig
from qwen_vln.types import ModelResult, PromptMode
from qwen_vln.visualizer import ResultVisualizer


def resize_for_api(image, max_width: int):
    height, width = image.shape[:2]
    if width <= max_width:
        return image
    scale = max_width / float(width)
    return cv2.resize(
        image,
        (max_width, max(1, int(round(height * scale)))),
        interpolation=cv2.INTER_AREA,
    )


def _rows_with_step(msg: Image, row_count: int) -> np.ndarray:
    expected = int(row_count) * int(msg.step)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if data.size < expected:
        raise ValueError(f"image buffer too short: {data.size} < {expected}")
    return data[:expected].reshape(row_count, msg.step)


def raw_ros_image_to_bgr(msg: Image):
    encoding = (msg.encoding or "").strip().lower()

    if encoding in {"bgr8", "rgb8"}:
        row = _rows_with_step(msg, msg.height)
        array = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else array.copy()

    if encoding in {"bgra8", "rgba8"}:
        row = _rows_with_step(msg, msg.height)
        array = row[:, : msg.width * 4].reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(array, code)

    if encoding in {"mono8", "8uc1"}:
        row = _rows_with_step(msg, msg.height)
        return cv2.cvtColor(row[:, : msg.width], cv2.COLOR_GRAY2BGR)

    if encoding in {"yuyv", "yuy2", "yuv422_yuy2"}:
        row = _rows_with_step(msg, msg.height)
        array = row[:, : msg.width * 2].reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(array, cv2.COLOR_YUV2BGR_YUY2)

    if encoding in {"uyvy", "yuv422"}:
        row = _rows_with_step(msg, msg.height)
        array = row[:, : msg.width * 2].reshape(msg.height, msg.width, 2)
        return cv2.cvtColor(array, cv2.COLOR_YUV2BGR_UYVY)

    if encoding in {"nv12", "nv21"}:
        row_count = msg.height * 3 // 2
        row = _rows_with_step(msg, row_count)
        array = row[:, : msg.width]
        code = cv2.COLOR_YUV2BGR_NV12 if encoding == "nv12" else cv2.COLOR_YUV2BGR_NV21
        return cv2.cvtColor(array, code)

    raise ValueError(
        f"Unsupported raw image encoding: {msg.encoding!r}. "
        "Use compressed transport or publish bgr8/rgb8/mono8/YUYV/UYVY/NV12/NV21."
    )


@dataclass
class PendingRequest:
    generation: int
    mode: PromptMode
    request_id: int
    frame: Any
    header: Any


class QwenVlnDebugNode(Node):
    def __init__(
        self,
        config_path: str,
        instruction: str,
        image_topic: Optional[str],
        image_transport: Optional[str],
    ):
        super().__init__("qwen3_vln_debug_node")
        self.config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))

        api_cfg = self.config["api"]
        self.client = QwenVisionClient(
            ClientConfig(
                model=api_cfg["model"],
                api_key_env=api_cfg["api_key_env"],
                base_url_env=api_cfg["base_url_env"],
                default_base_url=api_cfg["default_base_url"],
                timeout_sec=float(api_cfg["timeout_sec"]),
                max_retries=int(api_cfg["max_retries"]),
                temperature=float(api_cfg["temperature"]),
                max_tokens=int(api_cfg["max_tokens"]),
                enable_thinking=bool(api_cfg["enable_thinking"]),
                jpeg_quality=int(self.config["image"]["jpeg_quality"]),
            )
        )

        sm_cfg = self.config["state_machine"]
        self.fsm = NavigationStateMachine(
            StateMachineConfig(
                observe_interval_sec=float(sm_cfg["observe_interval_sec"]),
                track_interval_sec=float(sm_cfg["track_interval_sec"]),
                search_interval_sec=float(sm_cfg["search_interval_sec"]),
                verify_interval_sec=float(sm_cfg["verify_interval_sec"]),
                error_cooldown_sec=float(sm_cfg["error_cooldown_sec"]),
                min_confidence_visible=float(sm_cfg["min_confidence_visible"]),
                min_confidence_search_hint=float(sm_cfg["min_confidence_search_hint"]),
                min_confidence_verify=float(sm_cfg["min_confidence_verify"]),
                auto_enter_search=bool(sm_cfg["auto_enter_search"]),
            )
        )

        prompt_dir = PROJECT_ROOT / str(self.config.get("prompts", {}).get("directory", "prompts"))
        self.prompt_manager = PromptManager(str(prompt_dir))
        vis_cfg = self.config["visualization"]
        self.visualizer = ResultVisualizer(
            int(vis_cfg["point_radius"]),
            int(vis_cfg["history_length"]),
            bool(vis_cfg["draw_center_line"]),
        )
        self.api_max_width = int(self.config["image"]["api_max_width"])
        self.output_jpeg_quality = int(vis_cfg["jpeg_quality"])

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_header = None
        self.result_frame = None
        self.result_header = None
        self.latest_result: Optional[ModelResult] = None
        self.latest_error = ""
        self.latest_prompt = ""
        self.latest_request_mode: Optional[PromptMode] = None

        self.future: Optional[Future] = None
        self.future_meta: Optional[PendingRequest] = None
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="qwen_api")
        self.request_id = 0

        topics = self.config["topics"]
        self.state_pub = self.create_publisher(String, topics["state"], 10)
        self.result_pub = self.create_publisher(String, topics["result_json"], 10)
        self.latency_pub = self.create_publisher(Float32, topics["latency_ms"], 10)
        self.point_pub = self.create_publisher(PointStamped, topics["pixel_point"], 10)
        self.prompt_pub = self.create_publisher(String, topics["prompt_text"], 10)
        self.annotated_pub = self.create_publisher(CompressedImage, topics["annotated_image"], 2)
        self.create_subscription(String, topics["instruction"], self._on_instruction, 10)
        self.create_subscription(String, topics["command"], self._on_command, 10)

        image_topic = image_topic or self.config["image"]["topic"]
        image_transport = (image_transport or self.config["image"]["transport"]).strip().lower()
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        if image_transport == "compressed":
            self.create_subscription(
                CompressedImage,
                image_topic,
                self._on_compressed_image,
                sensor_qos,
            )
        elif image_transport == "raw":
            self.create_subscription(Image, image_topic, self._on_raw_image, sensor_qos)
        else:
            raise ValueError("image transport must be raw or compressed")

        initial = instruction.strip() if instruction else str(sm_cfg["initial_instruction"]).strip()
        self.fsm.set_instruction(initial)
        self.timer = self.create_timer(
            1.0 / max(1.0, float(vis_cfg["publish_hz"])),
            self._on_timer,
        )
        self.get_logger().info(
            f"started model={self.client.config.model} image_topic={image_topic} "
            f"transport={image_transport} task={initial!r}"
        )
        self.get_logger().warning("V1 publishes no chassis velocity commands")

    def destroy_node(self):
        self.executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _on_raw_image(self, msg: Image) -> None:
        try:
            self._store_frame(raw_ros_image_to_bgr(msg), msg.header)
        except Exception as exc:
            self.latest_error = f"raw_image_decode: {exc}"

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("cv2.imdecode returned None")
            self._store_frame(frame, msg.header)
        except Exception as exc:
            self.latest_error = f"compressed_image_decode: {exc}"

    def _store_frame(self, frame, header) -> None:
        frame = resize_for_api(frame, self.api_max_width)
        with self.frame_lock:
            self.latest_frame = frame
            self.latest_header = copy.deepcopy(header)
        self.fsm.mark_image_ready()

    def _on_instruction(self, msg: String) -> None:
        self.fsm.set_instruction(msg.data)
        self.latest_result = None
        self.result_frame = None
        self.result_header = None
        self.latest_error = ""
        self.visualizer.clear()

    def _on_command(self, msg: String) -> None:
        try:
            self.fsm.command(msg.data)
            self.latest_error = ""
        except ValueError as exc:
            self.latest_error = str(exc)
            self.get_logger().error(str(exc))

    def _on_timer(self) -> None:
        self._consume_future()
        self._maybe_submit_request()
        self._publish_visualization()
        self._publish_state()

    def _consume_future(self) -> None:
        if self.future is None or not self.future.done():
            return
        future, meta = self.future, self.future_meta
        self.future, self.future_meta = None, None
        if meta is None:
            return
        try:
            result = future.result()
        except Exception as exc:
            self.latest_error = f"api_or_parse_error: {exc}"
            self.fsm.apply_error(self.latest_error)
            self.get_logger().error(self.latest_error)
            return

        if meta.generation != self.fsm.generation:
            self.get_logger().warning(f"discard stale result request_id={meta.request_id}")
            return

        # Crucial for pixel debugging: draw the point on the exact image sent to
        # Qwen, never on a newer live frame that arrived during API latency.
        self.result_frame = meta.frame
        self.result_header = meta.header
        self.latest_result = result
        self.latest_error = ""
        self.visualizer.add_result(result)
        self.fsm.apply_result(result, meta.mode)

        self.result_pub.publish(String(data=json.dumps(result.to_dict(), ensure_ascii=False)))
        self.latency_pub.publish(Float32(data=float(result.latency_ms)))
        if result.point is not None:
            point_msg = PointStamped()
            point_msg.header.frame_id = f"camera_pixels_{result.point_role}"
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.point.x = float(result.point.x)
            point_msg.point.y = float(result.point.y)
            point_msg.point.z = float(result.confidence)
            self.point_pub.publish(point_msg)

        self.get_logger().info(
            f"id={meta.request_id} result={result.result} point={result.point} "
            f"conf={result.confidence:.2f} latency={result.latency_ms:.0f}ms "
            f"next={self.fsm.state.value}"
        )

    def _maybe_submit_request(self) -> None:
        if self.future is not None or not self.fsm.should_request():
            return
        mode = self.fsm.prompt_mode()
        if mode is None:
            return

        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
            header = copy.deepcopy(self.latest_header)

        height, width = frame.shape[:2]
        if self.latest_result is None or self.latest_result.point is None:
            previous = "none"
        else:
            # Feed previous point back in the same 0-1000 protocol the model outputs.
            prev_w = self.latest_result.image_width or width
            prev_h = self.latest_result.image_height or height
            previous = (
                f"({pixel_to_norm1000(self.latest_result.point.x, prev_w)}, "
                f"{pixel_to_norm1000(self.latest_result.point.y, prev_h)})"
            )
        prompt = self.prompt_manager.build(
            mode,
            self.fsm.instruction,
            width,
            height,
            previous,
        )

        self.request_id += 1
        self.fsm.mark_request_started()
        self.latest_prompt = prompt
        self.latest_request_mode = mode
        self.prompt_pub.publish(String(data=prompt))
        self.future_meta = PendingRequest(
            generation=self.fsm.generation,
            mode=mode,
            request_id=self.request_id,
            frame=frame,
            header=header,
        )
        self.future = self.executor.submit(
            self.client.infer,
            frame,
            prompt,
            mode,
            self.request_id,
        )

    def _publish_visualization(self) -> None:
        with self.frame_lock:
            live_frame = None if self.latest_frame is None else self.latest_frame.copy()
            live_header = copy.deepcopy(self.latest_header)

        if self.result_frame is not None and self.latest_result is not None:
            frame = self.result_frame.copy()
            header = copy.deepcopy(self.result_header)
            result = self.latest_result
            frame_note = f"exact API input image for request {result.request_id}"
        else:
            if live_frame is None:
                return
            frame = live_frame
            header = live_header
            result = None
            frame_note = "live camera image; waiting for first API result"

        annotated = self.visualizer.draw(
            frame,
            self.fsm.state,
            self.fsm.instruction,
            result,
            self.future is not None,
            self.latest_error,
            frame_note,
        )
        ok, encoded = cv2.imencode(
            ".jpg",
            annotated,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.output_jpeg_quality],
        )
        if not ok:
            return

        msg = CompressedImage()
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.annotated_pub.publish(msg)

    def _publish_state(self) -> None:
        point = (
            None
            if self.latest_result is None or self.latest_result.point is None
            else {"x": self.latest_result.point.x, "y": self.latest_result.point.y}
        )
        payload = {
            "state": self.fsm.state.value,
            "instruction": self.fsm.instruction,
            "transition_reason": self.fsm.last_transition_reason,
            "generation": self.fsm.generation,
            "request_in_flight": self.future is not None,
            "last_request_mode": None if self.latest_request_mode is None else self.latest_request_mode.value,
            "last_result": None if self.latest_result is None else self.latest_result.result,
            "last_request_id": None if self.latest_result is None else self.latest_result.request_id,
            "last_point": point,
            "last_confidence": None if self.latest_result is None else self.latest_result.confidence,
            "last_latency_ms": None if self.latest_result is None else self.latest_result.latency_ms,
            "visualized_frame": "exact_api_input" if self.result_frame is not None else "live_camera",
            "error": self.latest_error,
        }
        self.state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "configs/qwen3_vln_debug.yaml"),
    )
    parser.add_argument("--instruction", default="")
    parser.add_argument("--image-topic", default=None)
    parser.add_argument("--image-transport", choices=["raw", "compressed"], default=None)
    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        node = QwenVlnDebugNode(
            args.config,
            args.instruction,
            args.image_topic,
            args.image_transport,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
