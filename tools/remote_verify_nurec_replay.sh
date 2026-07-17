#!/usr/bin/env bash
set -euo pipefail

IMAGE_ROOT="${IMAGE_ROOT:?IMAGE_ROOT is required}"
LOG_FILE="${LOG_FILE:?LOG_FILE is required}"
PYTHON_BIN="${PYTHON_BIN:-/home/cwadmin/sim-env/miniconda3/envs/autodrive/bin/python}"
EXPECTED_FRAMES="${EXPECTED_FRAMES:-577}"
EXPECTED_WIDTH="${EXPECTED_WIDTH:-800}"
EXPECTED_HEIGHT="${EXPECTED_HEIGHT:-450}"
EXPECTED_CAMERAS="${EXPECTED_CAMERAS:-3}"

"$PYTHON_BIN" - "$IMAGE_ROOT" "$LOG_FILE" "$EXPECTED_FRAMES" \
    "$EXPECTED_WIDTH" "$EXPECTED_HEIGHT" "$EXPECTED_CAMERAS" <<'PY'
from __future__ import annotations

import re
import sys
from pathlib import Path

import cv2

root = Path(sys.argv[1])
log_path = Path(sys.argv[2])
expected_frames = int(sys.argv[3])
expected_width = int(sys.argv[4])
expected_height = int(sys.argv[5])
expected_cameras = int(sys.argv[6])

camera_dirs = sorted(path for path in root.iterdir() if path.is_dir())
assert len(camera_dirs) == expected_cameras, (len(camera_dirs), camera_dirs)
total_frames = 0
for camera_dir in camera_dirs:
    frames = sorted(camera_dir.glob("*.jpg"))
    assert len(frames) == expected_frames, (camera_dir.name, len(frames))
    total_frames += len(frames)
    dimensions = set()
    for frame in (frames[0], frames[len(frames) // 2], frames[-1]):
        image = cv2.imread(str(frame))
        assert image is not None, frame
        dimensions.add((image.shape[1], image.shape[0]))
    assert dimensions == {(expected_width, expected_height)}, (camera_dir.name, dimensions)
    print(f"CAMERA {camera_dir.name} frames={len(frames)} dimensions={sorted(dimensions)}")

log = log_path.read_text(encoding="utf-8", errors="replace")
bad = re.findall(
    r"Traceback|terminate called|segmentation fault|core dumped|REPLAY_EXIT_CODE=[^0]",
    log,
    flags=re.IGNORECASE,
)
assert not bad, bad
assert "Starting replay" in log
print(f"TOTAL_FRAMES={total_frames}")
print(f"SIMULATED_DURATION_SECONDS={expected_frames / 30:.6f}")
print("LOG_CLEAN=1")
PY

echo "PORTS"
ss -ltn '( sport = :2000 or sport = :46435 )'
echo "NUREC_CONTAINERS"
docker ps --filter name=NuRec_scene-0061 \
    --format '{{.Names}} {{.Image}} {{.Status}}'
