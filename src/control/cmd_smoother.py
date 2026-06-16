#!/usr/bin/env python3
"""对 vx/wz 做限速与低通平滑，减轻突变和突然转圈。"""


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class CmdSmoother:
    def __init__(
        self,
        alpha=0.4,
        max_vx_delta=0.015,
        max_wz_delta=0.02,
    ):
        """
        alpha: 低通系数，越大越平滑（0~1）。
        max_*_delta: 单次更新允许的最大速度跳变。
        """
        self.alpha = float(alpha)
        self.max_vx_delta = float(max_vx_delta)
        self.max_wz_delta = float(max_wz_delta)
        self.vx = 0.0
        self.wz = 0.0

    def reset(self):
        self.vx = 0.0
        self.wz = 0.0

    def update(self, target_vx, target_wz):
        tvx = float(target_vx)
        twz = float(target_wz)

        # 停车/零速指令立即生效，避免平滑器拖尾滑行
        if abs(tvx) < 1e-4 and abs(twz) < 1e-4:
            self.vx = 0.0
            self.wz = 0.0
            return 0.0, 0.0

        if self.alpha > 0.0:
            tvx = self.alpha * self.vx + (1.0 - self.alpha) * tvx
            twz = self.alpha * self.wz + (1.0 - self.alpha) * twz

        dvx = clamp(tvx - self.vx, -self.max_vx_delta, self.max_vx_delta)
        dwz = clamp(twz - self.wz, -self.max_wz_delta, self.max_wz_delta)

        self.vx += dvx
        self.wz += dwz
        return self.vx, self.wz
