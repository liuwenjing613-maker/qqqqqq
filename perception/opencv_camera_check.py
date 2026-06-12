#!/usr/bin/env python3
import cv2
import argparse
import time
import os


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="/dev/video0", help="camera device, e.g. /dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--save", default="../data/images/stage6_opencv_test.jpg")
    args = parser.parse_args()

    print("=== OpenCV camera check ===")
    print(f"camera: {args.camera}")
    print(f"resolution: {args.width}x{args.height}")

    cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera: {args.camera}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    ok_count = 0
    last_frame = None
    start = time.time()

    for i in range(args.frames):
        ok, frame = cap.read()
        if not ok or frame is None:
            print(f"[WARN] frame {i}: read failed")
            time.sleep(0.05)
            continue

        ok_count += 1
        last_frame = frame
        h, w = frame.shape[:2]
        print(f"[OK] frame {i}: shape={w}x{h}")

        time.sleep(0.03)

    elapsed = time.time() - start
    fps = ok_count / elapsed if elapsed > 0 else 0.0

    print("================================")
    print(f"read ok frames: {ok_count}/{args.frames}")
    print(f"approx fps: {fps:.2f}")

    if last_frame is not None:
        os.makedirs(os.path.dirname(args.save), exist_ok=True)
        cv2.imwrite(args.save, last_frame)
        print(f"saved image: {args.save}")
    else:
        print("[ERROR] no valid frame saved")

    cap.release()


if __name__ == "__main__":
    main()
