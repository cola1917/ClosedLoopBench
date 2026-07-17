from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

from adapters.shared_protocol_validation import validate_document
from runners.validate_multimodal_closed_loop import validate_multimodal_closed_loop_result


class CosmosTransferError(ValueError):
    """Raised when an offline Cosmos job would blur the closed-loop boundary."""


SUPPORTED_CONTROLS = {"edge", "depth", "vis", "seg"}


def build_cosmos_transfer_job(
    accepted_run_path: str | Path,
    rgb_video_path: str | Path,
    control_videos: Mapping[str, str | Path],
    *,
    prompt: str,
    frame_count: int,
    frames_per_sec: float,
    width: int,
    height: int,
    resolution: str = "480",
    seed: int = 0,
    negative_prompt: str | None = None,
    control_weights: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    """Build an offline Transfer2.5 job from an already accepted sensor run.

    The function intentionally accepts videos, never NuRec USDZ artifacts. Cosmos
    output is presentation/augmentation material and cannot become run evidence.
    """

    run_path = _file(accepted_run_path, "accepted run", suffix=".json")
    try:
        run_result = json.loads(run_path.read_text(encoding="utf-8"))
        acceptance = validate_multimodal_closed_loop_result(run_result)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise CosmosTransferError(f"accepted run is invalid: {exc}") from exc
    except RuntimeError as exc:
        raise CosmosTransferError(f"accepted run is invalid: {exc}") from exc

    if not isinstance(prompt, str) or not prompt.strip():
        raise CosmosTransferError("Cosmos prompt is required")
    unknown = sorted(set(control_videos) - SUPPORTED_CONTROLS)
    if unknown:
        raise CosmosTransferError("unsupported Cosmos controls: " + ", ".join(unknown))
    if not control_videos:
        raise CosmosTransferError("Cosmos Transfer2.5 requires at least one control video")
    weights = dict(control_weights or {})
    unknown_weights = sorted(set(weights) - set(control_videos))
    if unknown_weights:
        raise CosmosTransferError(
            "control weights reference missing controls: " + ", ".join(unknown_weights)
        )

    metadata = {
        "frame_count": frame_count,
        "frames_per_sec": frames_per_sec,
        "width": width,
        "height": height,
    }
    rgb = _video_ref(rgb_video_path, metadata, "RGB input")
    controls = []
    for control_type, path in sorted(control_videos.items()):
        weight = float(weights.get(control_type, 1.0))
        if not 0.0 <= weight <= 1.0:
            raise CosmosTransferError(f"{control_type} control weight must be in [0, 1]")
        controls.append(
            {
                "type": control_type,
                "video": _video_ref(path, metadata, f"{control_type} control"),
                "control_weight": weight,
            }
        )

    job = {
        "schema_version": "cosmos_transfer_job.v1",
        "scene_id": acceptance["scene_id"],
        "source": {
            "accepted_run": _file_ref(run_path),
            "rgb_video": rgb,
        },
        "controls": controls,
        "request": {
            "model": "cosmos-transfer2.5-2b",
            "transport": "http",
            "endpoint": "/v1/infer",
            "prompt": prompt.strip(),
            "negative_prompt": negative_prompt,
            "resolution": str(resolution),
            "seed": seed,
        },
        "execution": {
            "mode": "offline_batch",
            "realtime": False,
            "consumes_usdz": False,
            "expected_latency_class": "minutes",
        },
        "boundary": {
            "part_of_control_loop": False,
            "part_of_sensor_acceptance": False,
            "allowed_uses": ["portfolio_presentation", "offline_data_augmentation"],
            "forbidden_uses": [
                "closed_loop_metrics",
                "safety_evidence",
                "rgb_lidar_consistency_evidence",
            ],
        },
    }
    job["job_id"] = _canonical_sha256(job)
    try:
        validate_document(job)
    except ValueError as exc:
        raise CosmosTransferError(str(exc)) from exc
    return job


def verify_cosmos_transfer_job_files(job: Mapping[str, Any]) -> dict[str, Any]:
    """Re-hash every local input immediately before a remote Cosmos submission."""

    try:
        validate_document(dict(job))
    except ValueError as exc:
        raise CosmosTransferError(str(exc)) from exc
    references = [
        ("accepted_run", job["source"]["accepted_run"]),
        ("rgb_video", job["source"]["rgb_video"]),
        *(
            (f"control:{item['type']}", item["video"])
            for item in job["controls"]
        ),
    ]
    records = []
    for label, reference in references:
        path = Path(reference["path"])
        if not path.is_file():
            raise CosmosTransferError(f"Cosmos input disappeared: {label}: {path}")
        size = path.stat().st_size
        digest = _file_sha256(path)
        if size != reference["size_bytes"] or digest != reference["sha256"]:
            raise CosmosTransferError(f"Cosmos input changed after packaging: {label}")
        records.append({"name": label, "sha256": digest, "size_bytes": size})
    return {
        "schema_version": "cosmos_transfer_input_verification.v1",
        "job_id": job["job_id"],
        "status": "passed",
        "records": records,
    }


def _file(value: str | Path, label: str, *, suffix: str) -> Path:
    path = Path(value).resolve()
    if not path.is_file() or path.stat().st_size < 1:
        raise CosmosTransferError(f"{label} does not exist or is empty: {path}")
    if path.suffix.lower() != suffix:
        raise CosmosTransferError(f"{label} must be {suffix}, not {path.suffix or 'extensionless'}")
    return path


def _video_ref(value: str | Path, metadata: Mapping[str, Any], label: str) -> dict[str, Any]:
    path = _file(value, label, suffix=".mp4")
    return {**_file_ref(path), "media_type": "video/mp4", **dict(metadata)}


def _file_ref(path: Path) -> dict[str, Any]:
    return {
        "path": str(path),
        "sha256": _file_sha256(path),
        "size_bytes": path.stat().st_size,
    }


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()
