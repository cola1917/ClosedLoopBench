"""Freeze M5 KPI comparison and a portable evidence/video archive.

M5 summarizes exactly three M4-passing, short-horizon TF++ attempts.  The
comparison is a repeatability characterization of one pinned runtime identity,
not a cross-algorithm ranking or a route-completion claim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import tarfile
from pathlib import Path
from typing import Any, Iterable


REQUIRED_RUN_FILES = (
    "closed_loop_report.json",
    "frame_trace.jsonl",
    "metrics_trace.jsonl",
    "nurec_multimodal_trace.jsonl",
    "cleanup_audit.json",
    "nurec_run_summary.json",
)

KPI_FIELDS = (
    "collision_count",
    "min_ttc",
    "route_progress",
    "average_speed_mps",
    "hard_brake_count",
    "max_jerk",
    "control_timeout_count",
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _file_ref(path: Path, *, root: Path | None = None) -> dict[str, Any]:
    resolved = path.resolve()
    relative_path = None
    if root is not None:
        try:
            relative_path = str(resolved.relative_to(root.resolve()))
        except ValueError:
            pass
    return {
        "path": str(resolved),
        "relative_to_evidence_root": relative_path,
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
    }


def _numbers(values: Iterable[Any]) -> list[float]:
    return [float(value) for value in values if isinstance(value, (int, float))]


def _stats(values: Iterable[Any]) -> dict[str, Any]:
    numbers = _numbers(values)
    if not numbers:
        return {"count": 0, "mean": None, "min": None, "max": None, "stddev": None}
    mean = sum(numbers) / len(numbers)
    return {
        "count": len(numbers),
        "mean": mean,
        "min": min(numbers),
        "max": max(numbers),
        "stddev": math.sqrt(sum((value - mean) ** 2 for value in numbers) / len(numbers)),
    }


def _require_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"required evidence file missing: {path}")
    return path.resolve()


def _validate_video(summary: dict[str, Any], video_path: Path, run_dir: Path) -> list[str]:
    problems: list[str] = []
    if summary.get("status") != "rendered":
        problems.append("video_summary_not_rendered")
    if Path(str(summary.get("video") or "")).resolve() != video_path.resolve():
        problems.append("video_summary_path_mismatch")
    if int(summary.get("frames") or 0) < 60:
        problems.append("video_frame_count_below_60")
    if int(summary.get("nurec_frames_present") or 0) != int(summary.get("frames") or 0):
        problems.append("video_nurec_frame_coverage_incomplete")
    samples = [Path(str(value)) for value in summary.get("sample_pngs") or []]
    if len(samples) < 3 or any(not sample.is_file() or sample.stat().st_size == 0 for sample in samples):
        problems.append("video_samples_missing_or_empty")
    if run_dir.resolve() not in video_path.resolve().parents:
        problems.append("video_not_archived_below_evidence_root")
    try:
        import cv2

        capture = cv2.VideoCapture(str(video_path))
        decoded_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        decoded_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        decoded_frames = 0
        while True:
            available, _ = capture.read()
            if not available:
                break
            decoded_frames += 1
        capture.release()
        if [decoded_width, decoded_height] != list(summary.get("canvas") or []):
            problems.append("video_decoded_canvas_mismatch")
        if decoded_frames != int(summary.get("frames") or 0):
            problems.append("video_decoded_frame_count_mismatch")
    except ImportError:
        problems.append("video_decode_validation_unavailable")
    return problems


def build_m5_archive(
    *, evidence_root: Path, m4_report_path: Path, run_dirs: list[Path],
    video_path: Path, video_summary_path: Path, output_dir: Path,
) -> dict[str, Any]:
    evidence_root = evidence_root.resolve()
    m4_report_path = _require_file(m4_report_path)
    video_path = _require_file(video_path)
    video_summary_path = _require_file(video_summary_path)
    if len(run_dirs) != 3:
        raise ValueError("M5 requires exactly three M4 attempts")
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite archive directory: {output_dir}")

    m4 = _load(m4_report_path)
    if m4.get("schema_version") != "scene0061_m4_triplicate.v1" or m4.get("status") != "passed":
        raise ValueError("M4 strict report must be a passing scene0061_m4_triplicate.v1")
    attempts = m4.get("attempts") or []
    m4_attempts = {str(attempt.get("run_id")): attempt for attempt in attempts}
    if len(m4_attempts) != 3 or any(attempt.get("status") != "passed" for attempt in attempts):
        raise ValueError("M4 report does not contain three passing unique attempts")

    output_dir.mkdir(parents=True)
    records: list[dict[str, Any]] = []
    provenance_scripts = {
        "m5_archive_builder": _require_file(Path(__file__)),
        "m5_video_renderer": _require_file(
            PROJECT_ROOT / "runners" / "render_nurec_closed_loop_video.py"
        ),
        "m4_triplicate_validator": _require_file(
            PROJECT_ROOT / "runners" / "validate_scene0061_m4_triplicate.py"
        ),
    }
    source_files: list[Path] = [m4_report_path, *provenance_scripts.values()]
    for configured_path in (
        Path(str((m4.get("run_config") or {}).get("path") or "")),
        Path(str((m4.get("runtime_config") or {}).get("path") or "")),
        Path(str((m4.get("cuda_preflight") or {}).get("path") or "")),
    ):
        source_files.append(_require_file(configured_path))

    for supplied_dir in run_dirs:
        run_dir = supplied_dir.resolve()
        report_path = _require_file(run_dir / "closed_loop_report.json")
        report = _load(report_path)
        runtime = report.get("runtime") or {}
        summary = report.get("summary") or {}
        diagnostics = runtime.get("ego_driver_diagnostics") or {}
        binding = diagnostics.get("algorithm_sensor_binding") or {}
        run_id = str(report.get("run_id") or "")
        m4_attempt = m4_attempts.get(run_id)
        if m4_attempt is None or m4_attempt.get("status") != "passed":
            raise ValueError(f"run is not a passed M4 attempt: {run_dir}")
        if int(runtime.get("frame_trace_count") or 0) < 60:
            raise ValueError(f"run has fewer than 60 frames: {run_dir}")
        if report.get("status") != "ego_closed_loop" or runtime.get("cleanup_succeeded") is not True:
            raise ValueError(f"run did not finish a clean ego closed loop: {run_dir}")
        if int(summary.get("collision_count") or 0) != 0:
            raise ValueError(f"run contains a collision: {run_dir}")
        if int(diagnostics.get("mismatched_control_count") or 0) != 0:
            raise ValueError(f"run contains mismatched controls: {run_dir}")
        if int(diagnostics.get("fallback_count") or 0) != int(binding.get("initialization_safe_stop_count") or 0):
            raise ValueError(f"run has a non-initialization fallback: {run_dir}")

        for filename in REQUIRED_RUN_FILES:
            source_files.append(_require_file(run_dir / filename))
        axis_path = run_dir / "axis_regression.json"
        if axis_path.is_file():
            source_files.append(axis_path)
        latency = diagnostics.get("latency_ms") or {}
        records.append({
            "run_id": run_id,
            "run_dir": str(run_dir),
            "report_sha256": _sha256(report_path),
            "kpi": {field: summary.get(field) for field in KPI_FIELDS},
            "control": {
                "frame_count": runtime.get("frame_trace_count"),
                "control_count": diagnostics.get("control_count"),
                "initialization_safe_stop_count": binding.get("initialization_safe_stop_count"),
                "non_initialization_fallback_count": max(0, int(diagnostics.get("fallback_count") or 0) - int(binding.get("initialization_safe_stop_count") or 0)),
                "mismatched_control_count": diagnostics.get("mismatched_control_count"),
                "matched_frame_ratio": diagnostics.get("matched_frame_ratio"),
                "inference_latency_mean_ms": latency.get("mean"),
                "inference_latency_max_ms": latency.get("max"),
            },
            "m4_attempt": {
                "axis_status": (m4_attempt.get("lidar_axis_regression") or {}).get("status"),
                "axis_frames_checked": (m4_attempt.get("lidar_axis_regression") or {}).get("lidar_frames_checked"),
                "intermediate_record_count": m4_attempt.get("intermediate_record_count"),
            },
        })

    if len({record["run_id"] for record in records}) != 3:
        raise ValueError("M5 attempts must have unique run IDs")
    video_summary = _load(video_summary_path)
    video_problems = _validate_video(video_summary, video_path, run_dirs[-1].resolve())
    if video_problems:
        raise ValueError("; ".join(video_problems))
    source_files.extend([video_path, video_summary_path])
    for sample in video_summary.get("sample_pngs") or []:
        source_files.append(_require_file(Path(str(sample))))

    aggregate = {
        field: _stats(record["kpi"].get(field) for record in records)
        for field in KPI_FIELDS
    }
    aggregate["inference_latency_mean_ms"] = _stats(
        record["control"].get("inference_latency_mean_ms") for record in records
    )
    aggregate["inference_latency_max_ms"] = _stats(
        record["control"].get("inference_latency_max_ms") for record in records
    )
    comparison = {
        "schema_version": "scene0061_m5_kpi_comparison.v1",
        "status": "passed",
        "scope": "M5_pinned_tfpp_short_horizon_repeatability_not_cross_algorithm_ranking",
        "provenance_scripts": {
            name: _file_ref(path, root=evidence_root)
            for name, path in provenance_scripts.items()
        },
        "m4_report": _file_ref(m4_report_path, root=evidence_root),
        "attempt_count": len(records),
        "attempts": records,
        "aggregate": aggregate,
        "video": _file_ref(video_path, root=evidence_root),
    }
    comparison_path = output_dir / "m5_kpi_comparison.json"
    comparison_path.write_text(json.dumps(comparison, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    # De-duplicate by resolved path so shared config references occur once.
    unique_sources = sorted({path.resolve() for path in source_files}, key=lambda path: str(path))
    manifest = {
        "schema_version": "scene0061_m5_evidence_manifest.v1",
        "status": "passed",
        "scope": comparison["scope"],
        "comparison": _file_ref(comparison_path, root=evidence_root),
        "files": [_file_ref(path, root=evidence_root) for path in unique_sources],
    }
    manifest_path = output_dir / "m5_evidence_manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    archive_path = output_dir / "m5_evidence_archive.tar.gz"
    with tarfile.open(archive_path, "w:gz") as archive:
        for path in [comparison_path, manifest_path, *unique_sources]:
            try:
                relative = path.resolve().relative_to(evidence_root)
                arcname = Path("evidence") / relative
            except ValueError:
                arcname = Path("external") / path.name
            archive.add(path, arcname=str(arcname), recursive=False)
    return {
        "status": "passed",
        "comparison": _file_ref(comparison_path, root=evidence_root),
        "manifest": _file_ref(manifest_path, root=evidence_root),
        "archive": _file_ref(archive_path, root=evidence_root),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build the M5 KPI comparison and evidence archive.")
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--m4-report", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, action="append", required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--video-summary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = build_m5_archive(
            evidence_root=args.evidence_root,
            m4_report_path=args.m4_report,
            run_dirs=args.run_dir,
            video_path=args.video,
            video_summary_path=args.video_summary,
            output_dir=args.output_dir,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        result = {"status": "failed", "detail": str(exc)}
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if result.get("status") == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
