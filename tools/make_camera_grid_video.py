#!/usr/bin/env python3
"""Build and verify a labeled MP4 grid from synchronized camera JPEG sequences."""

from __future__ import annotations

import argparse
import math
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
    parser.add_argument("camera_dirs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--columns", type=int, default=3)
    args = parser.parse_args()
    if args.columns <= 0:
        raise ValueError("columns must be positive")

    directories = [path.resolve() for path in args.camera_dirs]
    sequences = [frame_paths(path) for path in directories]
    names = [[path.name for path in sequence] for sequence in sequences]
    if any(sequence_names != names[0] for sequence_names in names[1:]):
        raise RuntimeError("camera sequences do not have identical frame names")

    first_frames = [cv2.imread(str(sequence[0]), cv2.IMREAD_COLOR) for sequence in sequences]
    if any(frame is None for frame in first_frames):
        raise RuntimeError("failed to decode the first frame")
    shapes = {frame.shape[:2] for frame in first_frames}
    if len(shapes) != 1:
        raise RuntimeError("camera frame dimensions differ")
    cell_height, cell_width = first_frames[0].shape[:2]
    columns = min(args.columns, len(sequences))
    rows = math.ceil(len(sequences) / columns)
    output_width = cell_width * columns
    output_height = cell_height * rows

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (output_width, output_height),
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open the MP4 writer")
    try:
        for frame_set in zip(*sequences):
            canvas = np.zeros((output_height, output_width, 3), dtype=np.uint8)
            for index, (directory, frame_path) in enumerate(zip(directories, frame_set)):
                frame = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
                if frame is None or frame.shape[:2] != (cell_height, cell_width):
                    raise RuntimeError(f"invalid frame {frame_path}")
                row, column = divmod(index, columns)
                cv2.putText(
                    frame,
                    directory.name,
                    (18, 34),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                y0, x0 = row * cell_height, column * cell_width
                canvas[y0 : y0 + cell_height, x0 : x0 + cell_width] = frame
            writer.write(canvas)
    finally:
        writer.release()

    capture = cv2.VideoCapture(str(output))
    try:
        if not capture.isOpened():
            raise RuntimeError("OpenCV could not reopen the generated video")
        actual = {
            "frames": int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
            "fps": capture.get(cv2.CAP_PROP_FPS),
            "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        }
    finally:
        capture.release()
    expected_frames = len(sequences[0])
    if actual != {
        "frames": expected_frames,
        "fps": args.fps,
        "width": output_width,
        "height": output_height,
    }:
        raise RuntimeError(f"unexpected generated video metadata: {actual}")
    print(
        f"created {output}: cameras={len(sequences)} frames={actual['frames']} "
        f"fps={actual['fps']:g} resolution={actual['width']}x{actual['height']} "
        f"duration={actual['frames'] / actual['fps']:.3f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
