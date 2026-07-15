from __future__ import annotations

from collections import deque
from dataclasses import dataclass
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


@dataclass(frozen=True)
class ServoZoneOverlay:
    """POINT visual-servo horizontal zones drawn on the Foxglove image.

    Pixel mapping matches QwenVisualServo:
      error = (point_x - half_width) / half_width in [-1, 1]
    """

    max_vx: float = 0.07
    max_wz: float = 0.05
    kp_wz: float = 0.05
    angular_sign: float = -1.0
    center_deadband: float = 0.06
    turn_only_threshold: float = 0.40
    cmd_wz_deadband: float = 0.006
    enabled: bool = True

    def validate(self) -> None:
        if not 0.0 <= self.center_deadband < self.turn_only_threshold <= 1.0:
            raise ValueError(
                "require 0 <= center_deadband < turn_only_threshold <= 1"
            )


class ResultVisualizer:
    def __init__(
        self,
        point_radius: int = 11,
        history_length: int = 1,
        draw_center_line: bool = True,
        servo_zones: Optional[ServoZoneOverlay] = None,
    ):
        self.point_radius = max(4, int(point_radius))
        self.draw_center_line = bool(draw_center_line)
        if servo_zones is not None:
            servo_zones.validate()
        self.servo_zones = servo_zones
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
        panel_height = 178 if error_text else 156
        zone_top = panel_height + 2

        if (
            self.servo_zones is not None
            and self.servo_zones.enabled
            and width > 2
        ):
            canvas = self._draw_servo_zones(
                canvas,
                zone_top=zone_top,
                result=result,
            )

        if self.draw_center_line:
            cv2.line(
                canvas,
                (width // 2, zone_top),
                (width // 2, height - 1),
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
            zone_name = self._zone_name_for_point(point[0], width)
            self._label(
                canvas,
                f"{result.action} {result.point_role.upper()} "
                f"({point[0]}, {point[1]}) {zone_name}",
                (point[0] + 14, max(zone_top + 24, point[1] - 14)),
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
        cv2.putText(
            canvas,
            "zones: green=deadband | cyan=steer+drive | orange=rotate-only",
            (16, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (180, 255, 210),
            1,
            cv2.LINE_AA,
        )
        if error_text:
            cv2.putText(canvas, f"ERROR: {self._shorten(error_text, 100)}", (16, 174), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (80, 80, 255), 1, cv2.LINE_AA)
        return canvas

    def _zone_name_for_point(self, point_x: int, width: int) -> str:
        zones = self.servo_zones
        if zones is None or not zones.enabled or width <= 1:
            return ""
        half = 0.5 * float(width - 1)
        error = (float(point_x) - half) / max(1.0, half)
        abs_error = abs(error)
        if abs_error <= zones.center_deadband:
            return "[deadband]"
        if abs_error >= zones.turn_only_threshold:
            return "[rotate-only]"
        return "[steer+drive]"

    def _draw_servo_zones(self, canvas, zone_top: int, result: Optional[ModelResult]):
        zones = self.servo_zones
        assert zones is not None
        height, width = canvas.shape[:2]
        half = 0.5 * float(width - 1)
        cx = int(round(half))

        def error_to_x(error: float) -> int:
            return int(round(half + float(error) * half))

        dead_l = max(0, error_to_x(-zones.center_deadband))
        dead_r = min(width - 1, error_to_x(zones.center_deadband))
        turn_l = max(0, error_to_x(-zones.turn_only_threshold))
        turn_r = min(width - 1, error_to_x(zones.turn_only_threshold))

        overlay = canvas.copy()
        # Outer rotate-only bands
        cv2.rectangle(
            overlay, (0, zone_top), (turn_l, height - 1), (0, 110, 255), -1
        )
        cv2.rectangle(
            overlay, (turn_r, zone_top), (width - 1, height - 1), (0, 110, 255), -1
        )
        # Middle steer+drive bands
        cv2.rectangle(
            overlay, (turn_l, zone_top), (dead_l, height - 1), (210, 180, 40), -1
        )
        cv2.rectangle(
            overlay, (dead_r, zone_top), (turn_r, height - 1), (210, 180, 40), -1
        )
        # Center deadband
        cv2.rectangle(
            overlay, (dead_l, zone_top), (dead_r, height - 1), (40, 180, 70), -1
        )
        canvas = cv2.addWeighted(overlay, 0.18, canvas, 0.82, 0)

        line_bottom = height - 1
        for x, color in (
            (dead_l, (80, 220, 100)),
            (dead_r, (80, 220, 100)),
            (turn_l, (40, 160, 255)),
            (turn_r, (40, 160, 255)),
        ):
            cv2.line(canvas, (x, zone_top), (x, line_bottom), color, 1, cv2.LINE_AA)

        # Zone labels near top of image body
        label_y = min(height - 8, zone_top + 22)
        self._put_shadow(
            canvas,
            f"deadband |e|<={zones.center_deadband:.2f}",
            (max(4, dead_l + 4), label_y),
            (160, 255, 180),
            0.42,
        )
        mid_left = (turn_l + dead_l) // 2
        if dead_l - turn_l > 48:
            self._put_shadow(
                canvas,
                "steer+drive",
                (max(4, mid_left - 36), label_y + 18),
                (200, 230, 120),
                0.40,
            )
        if turn_l > 40:
            self._put_shadow(
                canvas,
                "rotate-only",
                (max(4, turn_l // 2 - 34), label_y + 18),
                (120, 190, 255),
                0.40,
            )
        if width - turn_r > 40:
            self._put_shadow(
                canvas,
                "rotate-only",
                (min(width - 92, turn_r + 6), label_y + 18),
                (120, 190, 255),
                0.40,
            )
        self._put_shadow(
            canvas,
            f"|e|>={zones.turn_only_threshold:.2f} => vx=0",
            (max(4, turn_r + 4), label_y),
            (140, 200, 255),
            0.40,
        )

        # Bottom parameter legend (control gains + thresholds)
        legend_h = 54
        legend_top = max(zone_top + 2, height - legend_h)
        legend = canvas.copy()
        cv2.rectangle(
            legend, (0, legend_top), (width, height), (16, 16, 16), -1
        )
        canvas = cv2.addWeighted(legend, 0.62, canvas, 0.38, 0)
        sign = "L" if zones.angular_sign < 0 else "R"
        line1 = (
            f"servo: max_vx={zones.max_vx:.3f}  max_wz={zones.max_wz:.3f}  "
            f"kp_wz={zones.kp_wz:.3f}  sign={zones.angular_sign:+.1f}({sign} if point right)"
        )
        line2 = (
            f"center_deadband={zones.center_deadband:.2f}  "
            f"turn_only={zones.turn_only_threshold:.2f}  "
            f"cmd_wz_db={zones.cmd_wz_deadband:.3f}  "
            f"cx={cx}px"
        )
        cv2.putText(
            canvas,
            line1,
            (10, legend_top + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            line2,
            (10, legend_top + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (200, 220, 255),
            1,
            cv2.LINE_AA,
        )

        # Current point error marker
        if result is not None and result.point is not None and width > 2:
            px = int(result.point.x)
            half_w = max(1.0, half)
            err = (float(px) - half) / half_w
            err = max(-1.0, min(1.0, err))
            marker_y = legend_top - 6
            if marker_y > zone_top + 8:
                cv2.line(
                    canvas,
                    (px, zone_top),
                    (px, marker_y),
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                self._put_shadow(
                    canvas,
                    f"e={err:+.3f}",
                    (min(width - 70, max(4, px + 4)), max(zone_top + 16, marker_y - 4)),
                    (255, 255, 255),
                    0.45,
                )

        return canvas

    @staticmethod
    def _put_shadow(
        image,
        text: str,
        origin: Tuple[int, int],
        color,
        scale: float,
    ) -> None:
        x, y = origin
        cv2.putText(
            image,
            text,
            (x + 1, y + 1),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            1,
            cv2.LINE_AA,
        )

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
