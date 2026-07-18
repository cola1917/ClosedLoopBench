from __future__ import annotations

import math
from copy import deepcopy
from typing import Any

from adapters.shared_protocol_validation import validate_document


class RuntimeAlignmentError(ValueError):
    """Raised when visual/runtime registration evidence is incomplete or invalid."""


def validate_runtime_alignment(
    scene_package: dict[str, Any],
    observations: dict[str, Any],
    *,
    horizontal_threshold_m: float = 0.25,
    vertical_threshold_m: float = 0.25,
    yaw_threshold_deg: float = 2.0,
) -> dict[str, Any]:
    validate_document(scene_package)
    if scene_package.get("schema_version") != "closed_loop_scene_package.v1":
        raise RuntimeAlignmentError("scene package must use closed_loop_scene_package.v1")
    if observations.get("schema_version") != "runtime_alignment_observations.v1":
        raise RuntimeAlignmentError("observations must use runtime_alignment_observations.v1")
    scene_id = str(scene_package["scene_id"])
    if observations.get("scene_id") != scene_id:
        raise RuntimeAlignmentError("observation scene_id does not match Scene Package")
    runtime = observations.get("runtime")
    if not isinstance(runtime, dict):
        raise RuntimeAlignmentError("runtime provenance is required")
    required_runtime = ("simulator", "renderer", "capture_method", "nurec_artifact_sha256")
    if any(not runtime.get(name) for name in required_runtime):
        raise RuntimeAlignmentError("runtime provenance is incomplete")
    digest = str(runtime["nurec_artifact_sha256"])
    if len(digest) != 64 or any(char not in "0123456789abcdef" for char in digest):
        raise RuntimeAlignmentError("nurec_artifact_sha256 must be lowercase SHA-256")
    captured_at = observations.get("captured_at")
    if not isinstance(captured_at, str) or not captured_at:
        raise RuntimeAlignmentError("captured_at is required")

    matrix = (scene_package.get("alignment") or {}).get("sim_from_log_transform")
    if not isinstance(matrix, list) or len(matrix) != 16:
        raise RuntimeAlignmentError("Scene Package has no defined 4x4 sim_from_log_transform")
    landmarks = observations.get("landmarks")
    if not isinstance(landmarks, list) or len(landmarks) < 3:
        raise RuntimeAlignmentError("at least three measured landmarks are required")
    _require_non_collinear(landmarks)

    capture = observations.get("capture") or {}
    reference_mode = str(
        capture.get("runtime_reference") or "scene_package_rigid_transform"
    )
    supported_reference_modes = {
        "scene_package_rigid_transform",
        "nurec_usdz_ego_trajectory",
    }
    if reference_mode not in supported_reference_modes:
        raise RuntimeAlignmentError(
            f"unsupported runtime alignment reference {reference_mode!r}"
        )
    if reference_mode == "nurec_usdz_ego_trajectory":
        if capture.get("vertical_reference") != "nurec_usdz_ego_trajectory":
            raise RuntimeAlignmentError(
                "NuRec runtime alignment requires an explicit USDZ vertical reference"
            )
        if capture.get("coordinate_boundary") != "canonical_y_left_to_carla_y_right":
            raise RuntimeAlignmentError(
                "NuRec runtime alignment requires the canonical CARLA coordinate boundary"
            )

    residuals = []
    for landmark in landmarks:
        landmark_id = str(landmark.get("landmark_id") or "")
        log_point = _point(landmark.get("log_global"), f"{landmark_id}.log_global")
        measured = _point(landmark.get("sim_measured"), f"{landmark_id}.sim_measured")
        scene_package_expected = _apply(matrix, log_point)
        runtime_reference = landmark.get("nurec_runtime")
        if reference_mode == "nurec_usdz_ego_trajectory":
            expected = _point(runtime_reference, f"{landmark_id}.nurec_runtime")
            if runtime_reference.get("pose_source") != (
                "nurec_usdz_rig_trajectory_interpolated"
            ):
                raise RuntimeAlignmentError(
                    f"landmark {landmark_id!r} has an invalid NuRec pose source"
                )
            measurement = landmark.get("measurement") or {}
            if int(runtime_reference.get("timestamp_us", -1)) != int(
                measurement.get("timestamp_us", -2)
            ):
                raise RuntimeAlignmentError(
                    f"landmark {landmark_id!r} NuRec and LiDAR timestamps differ"
                )
        elif runtime_reference is not None:
            raise RuntimeAlignmentError(
                "NuRec runtime poses require nurec_usdz_ego_trajectory capture mode"
            )
        else:
            expected = scene_package_expected
        horizontal = math.hypot(expected[0] - measured[0], expected[1] - measured[1])
        vertical = abs(expected[2] - measured[2])
        item: dict[str, Any] = {
            "landmark_id": landmark_id,
            "expected_sim": {"x": expected[0], "y": expected[1], "z": expected[2]},
            "scene_package_expected_sim": {
                "x": scene_package_expected[0],
                "y": scene_package_expected[1],
                "z": scene_package_expected[2],
            },
            "measured_sim": deepcopy(landmark["sim_measured"]),
            "horizontal_error_m": horizontal,
            "vertical_error_m": vertical,
        }
        if reference_mode == "nurec_usdz_ego_trajectory":
            item["log_to_runtime_horizontal_error_m"] = math.hypot(
                scene_package_expected[0] - expected[0],
                scene_package_expected[1] - expected[1],
            )
            # nuScenes mini does not expose reconstructed road elevation.  Keep
            # the discrepancy visible, but do not call zero-filled log z a 3D
            # ground-truth measurement.
            item["log_to_runtime_vertical_delta_m_not_gated"] = abs(
                scene_package_expected[2] - expected[2]
            )
        log_yaw = landmark["log_global"].get("yaw_deg")
        measured_yaw = landmark["sim_measured"].get("yaw_deg")
        if (log_yaw is None) != (measured_yaw is None):
            raise RuntimeAlignmentError(
                f"landmark {landmark_id!r} must provide yaw_deg in both frames or neither"
            )
        if log_yaw is not None:
            rotation_deg = math.degrees(math.atan2(float(matrix[4]), float(matrix[0])))
            scene_package_expected_yaw = _normalize_degrees(
                float(log_yaw) + rotation_deg
            )
            if reference_mode == "nurec_usdz_ego_trajectory":
                runtime_yaw = runtime_reference.get("yaw_deg")
                if runtime_yaw is None:
                    raise RuntimeAlignmentError(
                        f"landmark {landmark_id!r} NuRec pose lacks yaw_deg"
                    )
                expected_yaw = _normalize_degrees(float(runtime_yaw))
                item["scene_package_expected_yaw_deg"] = scene_package_expected_yaw
                item["log_to_runtime_yaw_error_deg"] = abs(
                    _normalize_degrees(expected_yaw - scene_package_expected_yaw)
                )
            else:
                expected_yaw = scene_package_expected_yaw
            yaw_error = abs(_normalize_degrees(float(measured_yaw) - expected_yaw))
            item["expected_yaw_deg"] = expected_yaw
            item["yaw_error_deg"] = yaw_error
        residuals.append(item)

    max_horizontal = max(item["horizontal_error_m"] for item in residuals)
    max_vertical = max(item["vertical_error_m"] for item in residuals)
    yaw_errors = [item["yaw_error_deg"] for item in residuals if "yaw_error_deg" in item]
    max_yaw = max(yaw_errors) if yaw_errors else None
    log_to_runtime_horizontal = [
        item["log_to_runtime_horizontal_error_m"]
        for item in residuals
        if "log_to_runtime_horizontal_error_m" in item
    ]
    log_to_runtime_yaw = [
        item["log_to_runtime_yaw_error_deg"]
        for item in residuals
        if "log_to_runtime_yaw_error_deg" in item
    ]
    max_log_to_runtime_horizontal = (
        max(log_to_runtime_horizontal) if log_to_runtime_horizontal else None
    )
    max_log_to_runtime_yaw = max(log_to_runtime_yaw) if log_to_runtime_yaw else None
    passed = (
        max_horizontal <= horizontal_threshold_m
        and max_vertical <= vertical_threshold_m
        and (max_yaw is None or max_yaw <= yaw_threshold_deg)
        and (
            max_log_to_runtime_horizontal is None
            or max_log_to_runtime_horizontal <= horizontal_threshold_m
        )
        and (
            max_log_to_runtime_yaw is None
            or max_log_to_runtime_yaw <= yaw_threshold_deg
        )
    )
    evidence = {
        "schema_version": "runtime_alignment_evidence.v1",
        "scene_id": scene_id,
        "captured_at": captured_at,
        "runtime": {name: runtime[name] for name in required_runtime},
        "thresholds": {
            "horizontal_error_m": float(horizontal_threshold_m),
            "vertical_error_m": float(vertical_threshold_m),
            "yaw_error_deg": float(yaw_threshold_deg),
        },
        "landmarks": residuals,
        "summary": {
            "reference_model": reference_mode,
            "vertical_source": (
                "nurec_usdz_ego_trajectory"
                if reference_mode == "nurec_usdz_ego_trajectory"
                else "nuscenes_global_z"
            ),
            "landmark_count": len(residuals),
            "max_horizontal_error_m": max_horizontal,
            "rms_horizontal_error_m": math.sqrt(
                sum(item["horizontal_error_m"] ** 2 for item in residuals) / len(residuals)
            ),
            "max_vertical_error_m": max_vertical,
            "max_yaw_error_deg": max_yaw,
            "max_log_to_runtime_horizontal_error_m": max_log_to_runtime_horizontal,
            "max_log_to_runtime_yaw_error_deg": max_log_to_runtime_yaw,
        },
        "status": "passed" if passed else "failed",
    }
    validate_document(evidence)
    return evidence


def promote_runtime_validated_package(
    scene_package: dict[str, Any],
    evidence: dict[str, Any],
    *,
    evidence_path: str,
) -> dict[str, Any]:
    validate_document(evidence)
    if evidence.get("status") != "passed":
        raise RuntimeAlignmentError("failed alignment evidence cannot validate a Scene Package")
    if evidence.get("scene_id") != scene_package.get("scene_id"):
        raise RuntimeAlignmentError("alignment evidence scene identity does not match")
    if not evidence_path or evidence_path.startswith(("/", "\\")) or ".." in evidence_path.split("/"):
        raise RuntimeAlignmentError("alignment evidence path must be safe and relative")
    promoted = deepcopy(scene_package)
    promoted["alignment"]["status"] = "runtime_validated"
    promoted["alignment"]["validation_evidence"] = evidence_path
    reference_model = (evidence.get("summary") or {}).get("reference_model")
    if reference_model == "nurec_usdz_ego_trajectory":
        promoted["alignment"]["runtime_reference"] = {
            "horizontal_and_yaw": "nuscenes_global_rigid_transform_cross_checked_to_nurec_usdz",
            "vertical": "nurec_usdz_ego_trajectory",
            "simulator_surface": "carla_opendrive_driving_waypoint",
            "coordinate_boundary": "canonical_y_left_to_carla_y_right",
        }
    validate_document(promoted)
    return promoted


def _point(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, dict) or any(name not in value for name in ("x", "y", "z")):
        raise RuntimeAlignmentError(f"{label} must contain x, y, z")
    return float(value["x"]), float(value["y"]), float(value["z"])


def _apply(matrix: list[float], point: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = point
    return (
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    )


def _require_non_collinear(landmarks: list[dict[str, Any]]) -> None:
    points = [_point(item.get("log_global"), "log_global") for item in landmarks]
    x0, y0, _ = points[0]
    for first in range(1, len(points) - 1):
        ax, ay = points[first][0] - x0, points[first][1] - y0
        for second in range(first + 1, len(points)):
            bx, by = points[second][0] - x0, points[second][1] - y0
            if abs(ax * by - ay * bx) > 1e-6:
                return
    raise RuntimeAlignmentError("alignment landmarks must include three non-collinear XY points")


def _normalize_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0
