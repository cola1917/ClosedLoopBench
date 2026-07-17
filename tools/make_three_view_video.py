#!/usr/bin/env python3
"""Build a verified side-by-side MP4 from three NuRec JPEG sequences."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def frame_paths(directory: Path) -> list[Path]:
    paths = sorted(directory.glob("*.jpg"))
    if not paths:
        raise RuntimeError(f"no JPEG frames found in {directory}")
    return paths


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("left", type=Path)
    parser.add_argument("center", type=Path)
    parser.add_argument("right", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    sequences = [frame_paths(path.resolve()) for path in (args.left, args.center, args.right)]
    names = [[path.name for path in sequence] for sequence in sequences]
    if names[1:] != names[:-1]:
        raise RuntimeError("camera sequences do not have identical frame names")

    first_frames = [cv2.imread(str(sequence[0]), cv2.IMREAD_COLOR) for sequence in sequences]
    if any(frame is None for frame in first_frames):
        raise RuntimeError("failed to decode the first frame")
    heights = {frame.shape[0] for frame in first_frames}
    if len(heights) != 1:
        raise RuntimeError("camera frame heights differ")
    height = first_frames[0].shape[0]
    width = sum(frame.shape[1] for frame in first_frames)

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open the MP4 writer")
    try:
        for paths in zip(*sequences):
            frames = [cv2.imread(str(path), cv2.IMREAD_COLOR) for path in paths]
            if any(frame is None for frame in frames):
                raise RuntimeError(f"failed to decode frame set {paths[0].name}")
            joined = np.hstack(frames)
            if joined.shape[:2] != (height, width):
                raise RuntimeError(f"unexpected frame dimensions at {paths[0].name}")
            writer.write(joined)
    finally:
        writer.release()

    capture = cv2.VideoCapture(str(output))
    try:
        if not capture.isOpened():
            raise RuntimeError("OpenCV could not reopen the generated video")
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    finally:
        capture.release()
    expected_count = len(sequences[0])
    if frame_count != expected_count:
        raise RuntimeError(f"video has {frame_count} frames, expected {expected_count}")
    print(
        f"created {output}: frames={frame_count} fps={actual_fps:g} "
        f"resolution={actual_width}x{actual_height} duration={frame_count / actual_fps:.3f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
