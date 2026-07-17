#!/usr/bin/env python3
"""Make CARLA 0.9.16's NuRec Scenario parser accept pinhole cameras.

The bundled parser documents OpenCV pinhole support but unconditionally reads
f-theta-only polynomial fields. NeuralSceneBridge's nuScenes exports correctly
store ``opencv-pinhole`` intrinsics, so those f-theta fields are optional. The
gRPC server remains authoritative for rendering intrinsics; this parser only
needs the logical camera names and rig transforms.
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


OLD_BLOCK = '''                reference_poly=calibration["camera_model"]["parameters"]["reference_poly"],
                pixeldist_to_angle_poly=calibration["camera_model"]["parameters"]["pixeldist_to_angle_poly"],
                angle_to_pixeldist_poly=calibration["camera_model"]["parameters"]["angle_to_pixeldist_poly"],
                max_angle=calibration["camera_model"]["parameters"]["max_angle"],
                linear_cde=calibration["camera_model"]["parameters"]["linear_cde"],
'''

NEW_BLOCK = '''                # These fields are specific to f-theta cameras. NuScenes
                # exports use opencv-pinhole intrinsics and omit them.
                reference_poly=calibration["camera_model"]["parameters"].get("reference_poly", 1),
                pixeldist_to_angle_poly=calibration["camera_model"]["parameters"].get("pixeldist_to_angle_poly", []),
                angle_to_pixeldist_poly=calibration["camera_model"]["parameters"].get("angle_to_pixeldist_poly", []),
                max_angle=calibration["camera_model"]["parameters"].get("max_angle", np.pi),
                linear_cde=calibration["camera_model"]["parameters"].get("linear_cde", []),
'''

OLD_ZERO_TIME_BLOCK = '''        self.tracks = Tracks(
            track_data, self.metadata["pose-range"]["start-timestamp_us"]
        )
'''

PARTIAL_V4_BLOCK = '''        if "pose-range" in self.metadata:
            zero_time = self.metadata["pose-range"]["start-timestamp_us"]
        else:
            # NuRec v4 metadata uses the sequence timestamp interval.
            zero_time = self.metadata["sequence_timestamp_interval_us"]["start"]
        self.tracks = Tracks(track_data, zero_time)
'''

NORMALIZE_V4_PREFIX = '''        if "pose-range" not in self.metadata:
            # Normalize NuRec v4 metadata once so all legacy CARLA call sites
            # can use the established pose-range keys.
            interval = self.metadata["sequence_timestamp_interval_us"]
            end_timestamp = interval.get("stop", interval.get("end"))
            if end_timestamp is None:
                raise KeyError("sequence_timestamp_interval_us has no stop/end value")
            self.metadata["pose-range"] = {
                "start-timestamp_us": interval["start"],
                "end-timestamp_us": end_timestamp,
            }
'''

NEW_V4_BLOCK = NORMALIZE_V4_PREFIX + OLD_ZERO_TIME_BLOCK


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=Path)
    args = parser.parse_args()
    target = args.target.resolve()
    text = target.read_text(encoding="utf-8")
    patched = text
    changes = []
    while NORMALIZE_V4_PREFIX + NORMALIZE_V4_PREFIX in patched:
        patched = patched.replace(
            NORMALIZE_V4_PREFIX + NORMALIZE_V4_PREFIX,
            NORMALIZE_V4_PREFIX,
            1,
        )
        if "v4-metadata-deduplicate" not in changes:
            changes.append("v4-metadata-deduplicate")
    if OLD_BLOCK in patched:
        patched = patched.replace(OLD_BLOCK, NEW_BLOCK, 1)
        changes.append("pinhole")
    elif NEW_BLOCK not in patched:
        raise RuntimeError("expected f-theta parameter block was not found")
    if NEW_V4_BLOCK in patched:
        pass
    elif PARTIAL_V4_BLOCK in patched:
        patched = patched.replace(PARTIAL_V4_BLOCK, NEW_V4_BLOCK, 1)
        changes.append("v4-metadata-normalize")
    elif OLD_ZERO_TIME_BLOCK in patched:
        patched = patched.replace(OLD_ZERO_TIME_BLOCK, NEW_V4_BLOCK, 1)
        changes.append("v4-metadata")
    else:
        raise RuntimeError("expected metadata zero-time block was not found")
    if not changes:
        print("already_patched")
        return 0
    backup = target.with_suffix(target.suffix + ".pre-closedloopbench")
    if not backup.exists():
        shutil.copy2(target, backup)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(patched, encoding="utf-8")
    os.replace(temporary, target)
    print("patched:" + ",".join(changes))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
