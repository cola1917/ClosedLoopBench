"""Multi-frame LiDAR axis-provenance regression for NuRec closed-loop runs.

The coordinate-fixed M8 diagnosis established that the raw NRE response is
already in the calibrated sensor-local basis. Multi-tick runs do not carry a
same-frame physical probe,
so this regression re-verifies, for EVERY frame of a run:

1. the frame's ``axis_normalization`` declares the exact transform matrix
   that passed the r22 physical gate (matrix sha256 must match);
2. the materialized raw NRE payload re-normalizes byte-for-byte into the
   materialized normalized payload (the conversion is replayed, not
   trusted); and
3. both payloads re-hash to the sha256 values recorded in the trace.

This is the "hardening moved onto the already-working loop" pass of the
re-sequenced plan: same provenance guarantees, amortized across a whole
closed-loop drive instead of gating each tick before the loop may run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from runtime.scene0061_lidar_axis_normalization import (  # noqa: E402
    normalize_lidar_xyzi,
)

# The identity raw-response-to-sensor transform selected by the coordinate-fixed
# M8 occupancy diagnosis. A later physical gate may replace this only with new
# payload-bound evidence and a corresponding explicit CLI argument.
RAW_RESPONSE_TO_SENSOR_SHA256 = (
    "bec390d7d89d2fd82783a1022dedab9c79736c27fe490d2a0462e8c3443843eb"
)


def validate_run(
    run_dir: Path, *, expected_matrix_sha256: str
) -> dict:
    trace_path = run_dir / "nurec_multimodal_trace.jsonl"
    if not trace_path.is_file():
        raise FileNotFoundError(f"missing nurec trace: {trace_path}")

    frames_checked = 0
    frames_passed = 0
    frames_unrendered = 0
    problems: list[dict] = []

    for line_no, line in enumerate(
        trace_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        frame = json.loads(line)
        lidar_records = [
            record
            for record in frame.get("records") or []
            if record.get("modality") == "lidar"
        ]
        for record in lidar_records:
            if record.get("status") != "passed":
                # No rendered payload exists for this frame (e.g. the drive
                # outlived the scene time range) - nothing to re-verify.
                frames_unrendered += 1
                continue
            frames_checked += 1
            frame_problems: list[str] = []
            metadata = record.get("response_metadata") or {}
            normalization = metadata.get("axis_normalization") or {}

            declared_sha = str(normalization.get("response_to_sensor_sha256") or "")
            if declared_sha != expected_matrix_sha256:
                frame_problems.append(
                    "axis matrix sha256 mismatch: "
                    f"declared={declared_sha or '<absent>'}"
                )

            raw_ref = metadata.get("raw_response_payload") or {}
            normalized_ref = metadata.get("materialized_payload") or {}
            matrix = normalization.get("response_to_sensor")
            if not raw_ref or not normalized_ref or matrix is None:
                frame_problems.append(
                    "frame lacks raw/normalized payload refs or matrix"
                )
            else:
                raw_path = _resolve_payload(run_dir, raw_ref)
                normalized_path = _resolve_payload(run_dir, normalized_ref)
                if raw_path is None or normalized_path is None:
                    frame_problems.append("payload file missing on disk")
                else:
                    raw_bytes = raw_path.read_bytes()
                    normalized_bytes = normalized_path.read_bytes()
                    if hashlib.sha256(raw_bytes).hexdigest() != raw_ref.get("sha256"):
                        frame_problems.append("raw payload sha256 mismatch on disk")
                    if (
                        hashlib.sha256(normalized_bytes).hexdigest()
                        != normalized_ref.get("sha256")
                    ):
                        frame_problems.append(
                            "normalized payload sha256 mismatch on disk"
                        )
                    replayed = normalize_lidar_xyzi(raw_bytes, matrix)
                    if replayed != normalized_bytes:
                        frame_problems.append(
                            "replayed normalization differs from materialized payload"
                        )

            if frame_problems:
                problems.append(
                    {
                        "trace_line": line_no,
                        "frame_id": frame.get("frame_id"),
                        "request_id": record.get("request_id"),
                        "problems": frame_problems,
                    }
                )
            else:
                frames_passed += 1

    status = "passed" if frames_checked > 0 and not problems else "failed"
    return {
        "schema_version": "nurec_lidar_axis_regression.v1",
        "status": status,
        "run_dir": str(run_dir),
        "expected_matrix_sha256": expected_matrix_sha256,
        "matrix_provenance": "coordinate-fixed M8 raw-response occupancy diagnosis",
        "lidar_frames_checked": frames_checked,
        "lidar_frames_passed": frames_passed,
        "lidar_frames_unrendered": frames_unrendered,
        "problems": problems[:50],
        "problem_count": len(problems),
    }


def _resolve_payload(run_dir: Path, ref: dict) -> Path | None:
    path = Path(str(ref.get("path") or ""))
    if path.is_file():
        return path
    relative = str(ref.get("relative_path") or "")
    if relative:
        candidate = run_dir.parent / relative
        if candidate.is_file():
            return candidate
        candidate = run_dir / relative
        if candidate.is_file():
            return candidate
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Re-verify the r22-frozen LiDAR axis transform and replay the "
            "raw->normalized conversion for every frame of a NuRec run."
        )
    )
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--expected-matrix-sha256", default=RAW_RESPONSE_TO_SENSOR_SHA256
    )
    args = parser.parse_args(argv)

    result = validate_run(
        args.run_dir.expanduser().resolve(),
        expected_matrix_sha256=str(args.expected_matrix_sha256),
    )
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
