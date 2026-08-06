"""Compare the three M8 TransFuser++ input routes fail-closed."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.plugin_contract import strict_json_loads
from metrics.open_loop import OPEN_LOOP_REPORT_SCHEMA, validate_open_loop_report
from metrics.transfuserpp_m8 import M8_REPORT_SCHEMA


ROUTE_ORDER = ("raw_original", "reconstructed", "harmonized")
EXPECTED_SOURCES = {
    "raw_original": "carla_stage_a_native_rgb_lidar",
    "reconstructed": "reconstructed_rgb_lidar",
    "harmonized": "harmonized_rgb_reconstructed_lidar",
}


class TriplicateComparisonError(ValueError):
    """Raised when the three route reports are not directly comparable."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = strict_json_loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise TriplicateComparisonError(f"cannot load {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise TriplicateComparisonError(f"{label} must be a JSON object: {path}")
    return value


def _report_route(report: Mapping[str, Any], path: Path) -> str:
    route = report.get("input_route")
    if not isinstance(route, Mapping):
        raise TriplicateComparisonError(f"report has no input_route: {path}")
    route_id = route.get("route_id")
    if route_id not in ROUTE_ORDER:
        raise TriplicateComparisonError(f"report route is invalid: {path}")
    if report.get("sensor_source") != EXPECTED_SOURCES[route_id]:
        raise TriplicateComparisonError(f"report sensor source does not match route: {path}")
    if route.get("harmonizer_rgb_only") is not (route_id == "harmonized"):
        raise TriplicateComparisonError(f"report Harmonizer declaration is invalid: {path}")
    return str(route_id)


def _validate_report(report: Mapping[str, Any], path: Path, expected_frame_count: int) -> str:
    try:
        validate_open_loop_report(report)
    except ValueError as exc:
        raise TriplicateComparisonError(f"invalid open-loop report {path}: {exc}") from exc
    route_id = _report_route(report, path)
    if report.get("execution_status") != "completed":
        raise TriplicateComparisonError(f"route report is not completed: {path}")
    sync = report.get("frame_sync") or {}
    if sync.get("matched_frame_count") != expected_frame_count or sync.get("dropped_frame_count") != 0:
        raise TriplicateComparisonError(f"route frame gate failed: {path}")
    tfpp = report.get("tfpp") or {}
    if (
        tfpp.get("intermediate_count") != expected_frame_count
        or tfpp.get("fallback_count") != 0
    ):
        raise TriplicateComparisonError(f"route TF++ intermediate gate failed: {path}")
    frames = report.get("frames")
    if not isinstance(frames, list) or len(frames) != expected_frame_count:
        raise TriplicateComparisonError(f"route frame rows are incomplete: {path}")
    for index, row in enumerate(frames):
        if not isinstance(row, Mapping) or row.get("frame_id") != index:
            raise TriplicateComparisonError(f"route frame identity failed at {index}: {path}")
        payloads = row.get("input_payloads") or {}
        camera = payloads.get("camera_front")
        lidar = payloads.get("lidar_top")
        if not isinstance(camera, Mapping) or not isinstance(lidar, Mapping):
            raise TriplicateComparisonError(f"route sensor payloads are incomplete at {index}: {path}")
        camera_parent = Path(str(camera.get("path") or "")).parent
        lidar_parent = Path(str(lidar.get("path") or "")).parent
        if camera_parent != lidar_parent or camera_parent.name != f"frame_{index:08d}":
            raise TriplicateComparisonError(f"route RGB/LiDAR pairing failed at {index}: {path}")
        if row.get("input_provenance", {}).get("input_source") != EXPECTED_SOURCES[route_id]:
            raise TriplicateComparisonError(f"route provenance failed at {index}: {path}")
        materialization = lidar.get("materialization")
        if route_id == "raw_original":
            if materialization is not None:
                raise TriplicateComparisonError(
                    f"raw route unexpectedly has reconstructed LiDAR materialization at {index}: {path}"
                )
        else:
            if not isinstance(materialization, Mapping):
                raise TriplicateComparisonError(
                    f"reconstructed LiDAR materialization is missing at {index}: {path}"
                )
            source_path = str(materialization.get("source_path") or "")
            if (
                "multimodal_20fps/lidar/" not in source_path
                or not source_path.endswith("_original.xyzi.bin")
                or materialization.get("source_coordinate_frame") != "nre_26_04_lidar_sensor"
                or materialization.get("source_axis_convention") != "nre_26_04_render_axes"
            ):
                raise TriplicateComparisonError(
                    f"route LiDAR is not bound to the NuRec reconstructed source at {index}: {path}"
                )
    return route_id


def _validate_intermediate(
    evaluation: Mapping[str, Any],
    path: Path,
    *,
    route_id: str,
    expected_frame_count: int,
    scenario_ir_sha256: str,
) -> None:
    if evaluation.get("schema_version") != M8_REPORT_SCHEMA:
        raise TriplicateComparisonError(f"invalid M8 evaluation schema: {path}")
    if evaluation.get("status") != "evaluated" or evaluation.get("frame_count") != expected_frame_count:
        raise TriplicateComparisonError(f"M8 intermediate evaluation gate failed: {path}")
    if evaluation.get("fail_closed_reasons"):
        raise TriplicateComparisonError(f"M8 evaluation has fail-closed reasons: {path}")
    binding = evaluation.get("input_binding") or {}
    if binding.get("observed_source") != EXPECTED_SOURCES[route_id]:
        raise TriplicateComparisonError(f"M8 evaluation source does not match route: {path}")
    truth = evaluation.get("ground_truth_binding") or {}
    if truth.get("scenario_ir_sha256") != scenario_ir_sha256:
        raise TriplicateComparisonError(f"M8 evaluation GT hash does not match routes: {path}")
    if evaluation.get("formal_bbox_evaluation") is not True:
        raise TriplicateComparisonError(
            f"M8 evaluation is not a formal actor-aware bbox evaluation: {path}"
        )
    actor_binding = evaluation.get("actor_binding") or {}
    if (
        actor_binding.get("required") is not True
        or actor_binding.get("status") != "bound"
        or actor_binding.get("problems")
    ):
        raise TriplicateComparisonError(f"M8 actor manifest gate failed: {path}")
    dynamic_bev = evaluation.get("dynamic_bev") or {}
    if (
        dynamic_bev.get("status") != "evaluated"
        or dynamic_bev.get("metric_scope") != "dynamic_actor_oriented_bev_bbox"
    ):
        raise TriplicateComparisonError(
            f"M8 evaluation does not contain real oriented bbox metrics: {path}"
        )
    detection = dynamic_bev.get("box_detection") or {}
    required_detection_fields = (
        "tp",
        "fp",
        "fn",
        "precision",
        "recall",
        "mAP25",
        "mAP50",
        "per_class",
    )
    if any(field not in detection for field in required_detection_fields):
        raise TriplicateComparisonError(f"M8 bbox detection fields are incomplete: {path}")


def compare_routes(
    report_paths: list[Path],
    intermediate_paths: list[Path],
    *,
    expected_frame_count: int = 39,
) -> dict[str, Any]:
    if len(report_paths) != len(ROUTE_ORDER):
        raise TriplicateComparisonError("exactly three route reports are required")
    if len(intermediate_paths) != len(ROUTE_ORDER):
        raise TriplicateComparisonError("exactly three M8 intermediate evaluations are required")
    reports: dict[str, dict[str, Any]] = {}
    report_files: dict[str, Path] = {}
    for path in report_paths:
        report = _load_object(path, "route report")
        route_id = _validate_report(report, path, expected_frame_count)
        if route_id in reports:
            raise TriplicateComparisonError(f"duplicate route report: {route_id}")
        reports[route_id] = report
        report_files[route_id] = path
    if set(reports) != set(ROUTE_ORDER):
        raise TriplicateComparisonError("the route set must be raw, reconstructed, and harmonized")

    scenario_ids = {str(report.get("scenario_id")) for report in reports.values()}
    scene_ids = {str(report.get("scene_id")) for report in reports.values()}
    gt_hashes = {
        str((report.get("artifacts") or {}).get("scenario_ir_sha256"))
        for report in reports.values()
    }
    opendrive_hashes = {
        str((report.get("artifacts") or {}).get("opendrive_sha256"))
        for report in reports.values()
    }
    runtime_hashes = {str(report.get("runtime_config_sha256")) for report in reports.values()}
    actor_manifest_refs = {
        (
            str((report.get("actor_manifest") or {}).get("sha256")),
            str((report.get("actor_manifest") or {}).get("file_sha256")),
        )
        for report in reports.values()
    }
    algorithm_identity = {
        route_id: {
            key: (reports[route_id].get("plugin_identity") or {}).get(key)
            for key in ("algorithm_id", "repo_revision", "repo_sha256", "checkpoint_sha256", "config_sha256")
        }
        for route_id in ROUTE_ORDER
    }
    if len(scenario_ids) != 1 or len(scene_ids) != 1 or len(gt_hashes) != 1 or len(opendrive_hashes) != 1:
        raise TriplicateComparisonError("route scene/GT/OpenDRIVE identities differ")
    if len(runtime_hashes) != 1 or len({json.dumps(value, sort_keys=True) for value in algorithm_identity.values()}) != 1:
        raise TriplicateComparisonError("TF++ runtime or algorithm identities differ")
    if len(actor_manifest_refs) != 1 or next(iter(actor_manifest_refs))[0] in {"", "None"}:
        raise TriplicateComparisonError("route actor manifest identities differ or are missing")

    evaluations: dict[str, dict[str, Any]] = {}
    evaluation_files: dict[str, Path] = {}
    scenario_ir_sha256 = next(iter(gt_hashes))
    for path in intermediate_paths:
        evaluation = _load_object(path, "M8 intermediate evaluation")
        binding = evaluation.get("input_binding") or {}
        source = binding.get("observed_source")
        route_id = next((key for key, value in EXPECTED_SOURCES.items() if value == source), None)
        if route_id is None:
            raise TriplicateComparisonError(f"M8 evaluation has an unknown source: {path}")
        _validate_intermediate(
            evaluation,
            path,
            route_id=route_id,
            expected_frame_count=expected_frame_count,
            scenario_ir_sha256=scenario_ir_sha256,
        )
        if route_id in evaluations:
            raise TriplicateComparisonError(f"duplicate M8 evaluation: {route_id}")
        evaluations[route_id] = evaluation
        evaluation_files[route_id] = path
    if set(evaluations) != set(ROUTE_ORDER):
        raise TriplicateComparisonError("M8 evaluations do not cover all three routes")

    reconstructed_lidar = [
        (reports["reconstructed"]["frames"][index]["input_payloads"]["lidar_top"].get("sha256"))
        for index in range(expected_frame_count)
    ]
    harmonized_lidar = [
        (reports["harmonized"]["frames"][index]["input_payloads"]["lidar_top"].get("sha256"))
        for index in range(expected_frame_count)
    ]
    if reconstructed_lidar != harmonized_lidar:
        raise TriplicateComparisonError(
            "reconstructed and Harmonizer routes do not share the same LiDAR payloads"
        )

    def _metric_summary(evaluation: Mapping[str, Any]) -> dict[str, Any]:
        dynamic_bev = evaluation.get("dynamic_bev") or {}
        detection = dynamic_bev.get("box_detection") or {}
        per_class = detection.get("per_class") or {}
        vehicle = per_class.get("vehicle") or {}
        pedestrian = per_class.get("pedestrian") or {}

        def _per_class_summary(class_metrics: Mapping[str, Any]) -> dict[str, Any]:
            iou_at_0_25 = class_metrics.get("iou_at_0_25") or {}
            return {
                "ground_truth_count": class_metrics.get("ground_truth_count"),
                "prediction_count": class_metrics.get("prediction_count"),
                "tp": class_metrics.get("tp"),
                "fp": class_metrics.get("fp"),
                "fn": class_metrics.get("fn"),
                "precision": class_metrics.get("precision"),
                "recall": class_metrics.get("recall"),
                "ap25": class_metrics.get("ap25"),
                "ap50": class_metrics.get("ap50"),
                "iou_at_0_25_precision": iou_at_0_25.get("precision"),
                "iou_at_0_25_recall": iou_at_0_25.get("recall"),
            }

        return {
            "waypoints_ade_m": ((evaluation.get("waypoints") or {}).get("ade_m") or {}).get("mean"),
            "waypoints_fde_m": ((evaluation.get("waypoints") or {}).get("fde_m") or {}).get("mean"),
            "route_checkpoints_ade_m": ((evaluation.get("route_checkpoints") or {}).get("ade_m") or {}).get("mean"),
            "target_speed_abs_error_mps": ((evaluation.get("target_speed") or {}).get("absolute_error_mps") or {}).get("mean"),
            "target_speed_nearest_bin_accuracy": (evaluation.get("target_speed") or {}).get("nearest_bin_accuracy"),
            "bev_center_class_accuracy": (evaluation.get("dynamic_bev") or {}).get("center_class_accuracy"),
            "bev_box_recall": ((evaluation.get("dynamic_bev") or {}).get("box_detection") or {}).get("recall"),
            "bev_box_precision": ((evaluation.get("dynamic_bev") or {}).get("box_detection") or {}).get("precision"),
            "bbox_gt_count": detection.get("ground_truth_count"),
            "bbox_prediction_count": detection.get("prediction_count_after_confidence_gate"),
            "bbox_tp": detection.get("tp"),
            "bbox_fp": detection.get("fp"),
            "bbox_fn": detection.get("fn"),
            "bbox_precision": detection.get("precision"),
            "bbox_recall": detection.get("recall"),
            "bbox_f1": detection.get("f1"),
            "bbox_mAP25": detection.get("mAP25"),
            "bbox_mAP50": detection.get("mAP50"),
            "bbox_iou_mean": (detection.get("iou") or {}).get("mean"),
            "bbox_center_error_mean_m": (detection.get("center_error_m") or {}).get("mean"),
            "bbox_yaw_error_mean_deg": (detection.get("yaw_error_deg") or {}).get("mean"),
            "vehicle_ap25": vehicle.get("ap25"),
            "vehicle_ap50": vehicle.get("ap50"),
            "pedestrian_ap25": pedestrian.get("ap25"),
            "pedestrian_ap50": pedestrian.get("ap50"),
            "bbox_per_class": {
                actor_type: _per_class_summary(class_metrics)
                for actor_type, class_metrics in per_class.items()
                if isinstance(class_metrics, Mapping)
            },
            "depth_status": (evaluation.get("depth") or {}).get("status"),
            "control_status": (evaluation.get("control") or {}).get("status"),
        }

    routes = {}
    for route_id in ROUTE_ORDER:
        report = reports[route_id]
        evaluation = evaluations[route_id]
        routes[route_id] = {
            "report_path": str(report_files[route_id].resolve()),
            "report_sha256": _sha256_file(report_files[route_id]),
            "intermediate_evaluation_path": str(evaluation_files[route_id].resolve()),
            "intermediate_evaluation_sha256": _sha256_file(evaluation_files[route_id]),
            "input_route": deepcopy(report["input_route"]),
            "observation_trace_path": report.get("observation_trace_path"),
            "observation_trace_sha256": report.get("observation_trace_sha256"),
            "open_loop_metrics": deepcopy(report.get("metrics") or {}),
            "intermediate_metrics": _metric_summary(evaluation),
        }
    return {
        "schema_version": "open_loop_transfuserpp_triplicate_comparison.v1",
        "status": "ready",
        "route_order": list(ROUTE_ORDER),
        "common_binding": {
            "scene_id": next(iter(scene_ids)),
            "scenario_id": next(iter(scenario_ids)),
            "scenario_ir_sha256": scenario_ir_sha256,
            "opendrive_sha256": next(iter(opendrive_hashes)),
            "runtime_config_sha256": next(iter(runtime_hashes)),
            "algorithm_identity": deepcopy(algorithm_identity["raw_original"]),
            "ground_truth_source": "original_scenario_ir_reference_trajectory_and_actor_tracks",
            "actor_manifest_sha256": next(iter(actor_manifest_refs))[0],
            "actor_manifest_file_sha256": next(iter(actor_manifest_refs))[1],
        },
        "cross_route_checks": {
            "all_routes_completed": True,
            "all_routes_same_frame_count": True,
            "all_routes_same_frame_rgb_lidar_pairing": True,
            "reconstructed_and_harmonized_share_lidar_sha256": True,
            "reconstructed_and_harmonized_use_nurec_lidar_sources": True,
            "harmonizer_changes_rgb_only": True,
            "raw_route_uses_native_rgb_and_lidar": True,
            "formal_actor_aware_bbox_evaluation": True,
            "all_routes_share_actor_manifest": True,
        },
        "routes": routes,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="append", type=Path, required=True)
    parser.add_argument("--intermediate-evaluation", action="append", type=Path, required=True)
    parser.add_argument("--expected-frame-count", type=int, default=39)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            parser.error(f"refusing to overwrite existing output: {args.output}")
        report = compare_routes(
            args.report,
            args.intermediate_evaluation,
            expected_frame_count=args.expected_frame_count,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps({"status": report["status"], "output": str(args.output)}))
        return 0
    except (OSError, ValueError, TypeError, json.JSONDecodeError, TriplicateComparisonError) as exc:
        print(json.dumps({"status": "failed", "error": f"{type(exc).__name__}: {exc}"}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
