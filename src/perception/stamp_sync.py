#!/usr/bin/env python3
"""按 header.stamp 缓存并配对 image / PerceptionTargets，避免 YOLO 推理滞后导致错帧。"""

from collections import deque


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def stamp_diff_sec(stamp_a, stamp_b):
    return abs(stamp_to_ns(stamp_a) - stamp_to_ns(stamp_b)) / 1e9


class StampSyncBuffer:
    def __init__(self, max_len=60, max_delta_sec=0.12):
        self.max_len = max(1, int(max_len))
        self.max_delta_sec = float(max_delta_sec)
        self._items = deque(maxlen=self.max_len)

    def push(self, stamp, data):
        self._items.append((stamp_to_ns(stamp), data))

    def find_closest(self, query_stamp):
        if not self._items or query_stamp is None:
            return None, None
        q = stamp_to_ns(query_stamp)
        best_ns, best_data = min(self._items, key=lambda item: abs(item[0] - q))
        delta = abs(best_ns - q) / 1e9
        if delta > self.max_delta_sec:
            return None, delta
        return best_data, delta

    def peek_latest(self):
        if not self._items:
            return None
        return self._items[-1][1]

    def __len__(self):
        return len(self._items)
