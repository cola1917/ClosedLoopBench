#!/usr/bin/env python3
"""Extract and verify one frame from a video."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--frame", type=int, required=True)
    args = parser.parse_args()
    capture = cv2.VideoCapture(str(args.video.resolve()))
    try:
        if not capture.isOpened():
            raise RuntimeError("could not open input video")
        capture.set(cv2.CAP_PROP_POS_FRAMES, args.frame)
        ok, image = capture.read()
    finally:
        capture.release()
    if not ok or image is None:
        raise RuntimeError(f"could not decode frame {args.frame}")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), image):
        raise RuntimeError("could not write output image")
    print(f"created {output}: frame={args.frame} resolution={image.shape[1]}x{image.shape[0]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
