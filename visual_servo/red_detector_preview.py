#!/usr/bin/env python3
import argparse
import time
import os

import cv2
import numpy as np


def find_red_bbox(frame):
    """
    输入 BGR 图像，返回最大红色区域：
    bbox = (x, y, w, h, area, area_ratio, cx, cy)
    如果找不到，返回 None。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 红色 HSV 跨 0 度，所以分两段
    lower_red1 = np.array([0, 80, 60])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 80, 60])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    # 形态学去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, mask

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)

    H, W = frame.shape[:2]
    area_ratio = area / float(W * H)

    if area < 500:
        return None, mask

    x, y, w, h = cv2.boundingRect(c)
    cx = x + w / 2.0
    cy = y + h / 2.0

    return (x, y, w, h, area, area_ratio, cx, cy), mask


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--save-dir", default="../data/images/red_preview")
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[ERROR] cannot open camera: {args.camera}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print("=== red detector preview ===")
    print(f"camera: {args.camera}")
    print("小车不会运动，只检测红色目标。")
    print("把红色目标放到摄像头前方，观察终端输出。")

    saved_count = 0

    for i in range(args.frames):
        ok, frame = cap.read()
        if not ok:
            print("[WARN] camera read failed")
            time.sleep(0.05)
            continue

        H, W = frame.shape[:2]
        result, mask = find_red_bbox(frame)

        if result is None:
            print(f"[{i:04d}] target: LOST")
        else:
            x, y, w, h, area, area_ratio, cx, cy = result
            ex = (cx - W / 2.0) / W

            print(
                f"[{i:04d}] target: FOUND "
                f"bbox=({x},{y},{w},{h}) "
                f"cx={cx:.1f} ex={ex:+.3f} area_ratio={area_ratio:.3f}"
            )

            # 每隔 30 帧保存一次画框图，方便你电脑上看效果
            if saved_count < 10 and i % 30 == 0:
                vis = frame.copy()
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.line(vis, (W // 2, 0), (W // 2, H), (255, 255, 255), 1)
                cv2.circle(vis, (int(cx), int(cy)), 5, (255, 0, 0), -1)

                save_path = os.path.join(args.save_dir, f"red_preview_{saved_count:02d}.jpg")
                cv2.imwrite(save_path, vis)
                print(f"saved: {save_path}")
                saved_count += 1

        time.sleep(0.05)

    cap.release()
    print("preview done.")


if __name__ == "__main__":
    main()
