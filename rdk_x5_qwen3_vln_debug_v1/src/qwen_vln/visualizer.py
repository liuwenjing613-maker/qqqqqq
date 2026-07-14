from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple

import cv2

from .types import ModelResult, PixelPoint, VlnState


_ROLE_COLORS = {
    "target": (70, 220, 70),
    "search": (0, 210, 255),
    "verify": (255, 220, 80),
    "none": (180, 180, 180),
}
_ACTION_COLORS = {
    "TURN_LEFT": (255, 180, 60),
    "TURN_RIGHT": (255, 180, 60),
    "STOP": (80, 80, 255),
}


class ResultVisualizer:
    def __init__(
        self,
        point_radius: int = 11,
        history_length: int = 1,
        draw_center_line: bool = True,
    ):
        self.point_radius = max(4, int(point_radius))
        self.draw_center_line = bool(draw_center_line)
        self.history: Deque[Tuple[PixelPoint, str]] = deque(
            maxlen=max(1, int(history_length))
        )
        self._last_request_id = -1

    def clear(self) -> None:
        self.history.clear()
        self._last_request_id = -1

    def add_result(self, result: Optional[ModelResult]) -> None:
        if (
            result is None
            or result.point is None
            or result.request_id == self._last_request_id
        ):
            return
        self._last_request_id = result.request_id
        self.history.append((result.point, result.point_role))

    def draw(
        self,
        frame_bgr,
        state: VlnState,
        instruction: str,
        result: Optional[ModelResult],
        request_in_flight: bool,
        error_text: str = "",
        frame_note: str = "live camera image",
    ):
        canvas = frame_bgr.copy()
        height, width = canvas.shape[:2]
        if self.draw_center_line:
            cv2.line(
                canvas,
                (width // 2, 0),
                (width // 2, height),
                (90, 90, 90),
                1,
            )

        historical = list(self.history)[:-1]
        for index, (point, role) in enumerate(historical):
            age = len(historical) - index
            color = _ROLE_COLORS.get(role, (180, 180, 180))
            alpha = max(0.18, 0.65 - age * 0.10)
            overlay = canvas.copy()
            cv2.circle(
                overlay,
                point.as_tuple(),
                max(3, self.point_radius // 3),
                color,
                -1,
            )
            canvas = cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0)

        if result is not None and result.point is not None:
            point = result.point.as_tuple()
            color = _ROLE_COLORS.get(result.point_role, (255, 255, 255))
            r = self.point_radius
            cv2.circle(canvas, point, r, color, 3)
            cv2.line(
                canvas,
                (point[0] - r - 7, point[1]),
                (point[0] + r + 7, point[1]),
                color,
                2,
            )
            cv2.line(
                canvas,
                (point[0], point[1] - r - 7),
                (point[0], point[1] + r + 7),
                color,
                2,
            )
            self._label(
                canvas,
                f"{result.action} {result.point_role.upper()} ({point[0]}, {point[1]})",
                (point[0] + 14, max(24, point[1] - 14)),
                color,
            )
        elif result is not None and result.action in _ACTION_COLORS:
            color = _ACTION_COLORS[result.action]
            if result.action == "TURN_LEFT":
                start, end = (width // 2 + 80, height // 2), (width // 2 - 80, height // 2)
            elif result.action == "TURN_RIGHT":
                start, end = (width // 2 - 80, height // 2), (width // 2 + 80, height // 2)
            else:
                start = end = (width // 2, height // 2)
            if result.action.startswith("TURN"):
                cv2.arrowedLine(canvas, start, end, color, 8, tipLength=0.25)
            else:
                cv2.putText(
                    canvas,
                    "STOP",
                    (max(20, width // 2 - 70), height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    color,
                    4,
                    cv2.LINE_AA,
                )

        panel_height = 178 if error_text else 156
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 0), (width, panel_height), (18, 18, 18), -1)
        canvas = cv2.addWeighted(overlay, 0.72, canvas, 0.28, 0)
        status = f"STATE: {state.value}" + (
            " | API REQUESTING" if request_in_flight else ""
        )
        cv2.putText(canvas, status, (16, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"TASK: {self._shorten(instruction, 90)}", (16, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (235, 235, 235), 1, cv2.LINE_AA)
        summary = (
            "RESULT: waiting"
            if result is None
            else f"RESULT: {result.result} action={result.action} c={result.confidence:.0f} latency={result.latency_ms:.0f}ms req={result.request_id}"
        )
        cv2.putText(canvas, summary, (16, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.51, (235, 235, 235), 1, cv2.LINE_AA)
        reason = "" if result is None else result.reason_code
        cv2.putText(canvas, f"REASON: {self._shorten(reason or '-', 90)}", (16, 101), cv2.FONT_HERSHEY_SIMPLEX, 0.49, (205, 205, 205), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"FRAME: {self._shorten(frame_note, 90)}", (16, 126), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (180, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "POINT=servo | TURN=one pulse then fresh observation | STOP=zero", (16, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 255, 210), 1, cv2.LINE_AA)
        if error_text:
            cv2.putText(canvas, f"ERROR: {self._shorten(error_text, 100)}", (16, 174), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (80, 80, 255), 1, cv2.LINE_AA)
        return canvas

    @staticmethod
    def _shorten(text: str, limit: int) -> str:
        cleaned = " ".join((text or "").split())
        return cleaned if len(cleaned) <= limit else cleaned[: max(0, limit - 3)] + "..."

    @staticmethod
    def _label(image, text: str, origin: Tuple[int, int], color) -> None:
        x, y = origin
        (tw, th), baseline = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1
        )
        x = min(max(0, x), max(0, image.shape[1] - tw - 8))
        y = min(max(th + 6, y), image.shape[0] - baseline - 4)
        cv2.rectangle(
            image,
            (x - 3, y - th - 5),
            (x + tw + 5, y + baseline + 3),
            (20, 20, 20),
            -1,
        )
        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            1,
            cv2.LINE_AA,
        )
