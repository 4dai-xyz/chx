#!/usr/bin/env python3
"""快速打印视频元信息。

用法:
    python src/people_bev_tracker/scripts/inspect_video.py --video resources/input_video.mp4
"""

from __future__ import annotations

import argparse

import cv2


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--video", required=True)
    args = p.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {args.video}")
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()

    duration = n / fps if fps > 0 else 0.0
    print(f"video         : {args.video}")
    print(f"width         : {w}")
    print(f"height        : {h}")
    print(f"fps           : {fps:.3f}")
    print(f"frame_count   : {n}")
    print(f"duration      : {duration:.2f} s")


if __name__ == "__main__":
    main()
