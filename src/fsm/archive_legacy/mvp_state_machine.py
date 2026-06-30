#!/usr/bin/env python3
from enum import Enum


class MVPState(str, Enum):
    INIT = "INIT"
    PARSE_TASK = "PARSE_TASK"
    OBSERVE = "OBSERVE"
    TARGET_LOCKED = "TARGET_LOCKED"
    VISUAL_SERVO = "VISUAL_SERVO"
    RECOVERY_SCAN = "RECOVERY_SCAN"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


class MVPStateMachine:
    def __init__(self, stable_frames_required=5, lost_frames_limit=8):
        self.state = MVPState.INIT
        self.stable_frames = 0
        self.lost_frames = 0
        self.stable_frames_required = stable_frames_required
        self.lost_frames_limit = lost_frames_limit

    def reset(self):
        self.state = MVPState.INIT
        self.stable_frames = 0
        self.lost_frames = 0

    def update(self, target_visible, servo_state):
        """
        输入:
          target_visible: 当前是否看到目标
          servo_state: LOST_STOP / TURN_ONLY / FORWARD / ARRIVED_STOP

        输出:
          当前状态
        """
        if self.state == MVPState.INIT:
            self.state = MVPState.PARSE_TASK
            return self.state

        if self.state == MVPState.PARSE_TASK:
            self.state = MVPState.OBSERVE
            return self.state

        if target_visible:
            self.stable_frames += 1
            self.lost_frames = 0
        else:
            self.lost_frames += 1
            self.stable_frames = 0

        if servo_state == "ARRIVED_STOP" and self.stable_frames >= self.stable_frames_required:
            self.state = MVPState.SUCCESS
            return self.state

        if self.lost_frames >= self.lost_frames_limit:
            self.state = MVPState.RECOVERY_SCAN
            return self.state

        if self.stable_frames >= self.stable_frames_required:
            self.state = MVPState.TARGET_LOCKED
        else:
            self.state = MVPState.OBSERVE

        if self.state == MVPState.TARGET_LOCKED:
            self.state = MVPState.VISUAL_SERVO

        return self.state
