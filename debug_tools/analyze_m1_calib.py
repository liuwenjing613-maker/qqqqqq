#!/usr/bin/env python3
import argparse
import csv
import json
import os
from collections import defaultdict


def read_trials(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            r["direction"] = int(r["direction"])
            r["target_speed"] = float(r["target_speed"])
            r["moved"] = str(r["moved"]).lower() in ("true", "1", "yes")
            r["mean_signed"] = float(r["mean_signed"])
            r["peak_abs"] = float(r["peak_abs"])
            r["active_ratio"] = float(r["active_ratio"])
            r["integral_abs"] = float(r["integral_abs"])
            rows.append(r)
    return rows


def first_moved(rows, test_type, axis, direction=1):
    vals = [
        r["target_speed"]
        for r in rows
        if r["test_type"] == test_type
        and r["axis"] == axis
        and r["direction"] == direction
        and r["moved"]
    ]
    return min(vals) if vals else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trial_csv", help="*_trials.csv")
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    rows = read_trials(args.trial_csv)

    summary = {
        "vx_breakaway": first_moved(rows, "breakaway", "vx", 1),
        "vx_hold_min": first_moved(rows, "hold", "vx", 1),
        "wz_breakaway": first_moved(rows, "breakaway", "wz", 1),
        "wz_hold_min": first_moved(rows, "hold", "wz", 1),
    }

    print(json.dumps(summary, ensure_ascii=False, indent=2))

    print("\n建议导航参数：")
    vx_hold = summary["vx_hold_min"]
    wz_hold = summary["wz_hold_min"]
    vx_break = summary["vx_breakaway"]= summary["wz_breakaway"]

    if vx_hold is not None:
        print(f"  target_slow_vx 建议 >= {vx_hold:.3f}")
        print(f"  pulse_vx 建议 {max(vx_hold, 0.03):.3f} ~ {max(vx_hold, 0.035):.3f}")
    if wz_hold is not None:
        print(f"  scan_wz / blocked_rotate_wz 建议 >= {wz_hold:.3f}")
    if vx_break is not None:
        print(f"  kick_vx 建议略高于 vx_breakaway: {max(vx_break, 0.055):.3f}")
    if wz_break is not None:
        print(f"  kick_wz 建议略高于 wz_breakaway: {max(wz_break, 0.24):.3f}")

    print("\n注意：如果 breakaway 明显大于 hold_min，就不要追求连续超低速，后续用短脉冲寸进。")

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
