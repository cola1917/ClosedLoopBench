from __future__ import annotations

from collections import defaultdict
import math
from statistics import mean, pstdev
from typing import Any

from runtime.scene0061_counterfactual import validate_scene0061_counterfactual_matrix


SCHEMA_VERSION = "counterfactual_suite_evaluation.v1"
_RUNTIME_STATUSES = {"completed", "ego_closed_loop", "interactive_closed_loop"}
_EVIDENCE_CLASSES = {
    "offline_conformance",
    "control_only",
    "perception_eligible",
    "quality_stress",
    "remote_validation_required",
}
_DELTA_METRICS = (
    "route_completion_time_sec",
    "route_progress",
    "average_speed_mps",
    "stopped_time_sec",
    "following_time_sec",
    "min_distance_m",
    "min_ttc",
    "min_pet_sec",
    "max_drac_mps2",
    "collision_count",
    "hard_brake_count",
    "max_jerk",
    "jerk_p95_mps3",
    "control_latency_p95_ms",
    "control_timeout_rate",
    "control_fallback_rate",
    "sensor_dropped_frame_count",
    "max_synchronization_error_ms",
)


class CounterfactualEvaluationError(ValueError):
    """Raised when an input cannot be interpreted without weakening fail-closed semantics."""


def evaluate_counterfactual_suite(
    matrix: dict[str, Any],
    reports: list[dict[str, Any]],
    quality_reports: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    validate_scene0061_counterfactual_matrix(matrix)
    algorithms = {row["algorithm_id"]: row for row in matrix["algorithms"]}
    cases = {row["case_id"]: row for row in matrix["cases"]}
    seeds = list(matrix["seeds"])
    expected = {
        (algorithm_id, case_id, seed)
        for algorithm_id in algorithms
        for case_id in cases
        for seed in seeds
    }
    quality_by_key = _quality_index(quality_reports or [])
    observed: dict[tuple[str, str, int], list[tuple[int, dict[str, Any]]]] = defaultdict(list)
    malformed = []
    for index, report in enumerate(reports):
        try:
            key = _report_key(report)
        except CounterfactualEvaluationError as exc:
            malformed.append({"index": index, "reason": str(exc)})
            continue
        observed[key].append((index, report))

    missing = [_key_row(key) for key in sorted(expected - set(observed))]
    unexpected = [_key_row(key) for key in sorted(set(observed) - expected)]
    duplicates = [
        {**_key_row(key), "report_indices": [index for index, _ in rows]}
        for key, rows in sorted(observed.items())
        if key in expected and len(rows) > 1
    ]
    invalid_runs = []
    accepted: dict[tuple[str, str, int], dict[str, Any]] = {}
    buckets: dict[str, list[dict[str, Any]]] = {name: [] for name in sorted(_EVIDENCE_CLASSES)}
    for key in sorted(expected & set(observed)):
        rows = observed[key]
        if len(rows) != 1:
            continue
        index, report = rows[0]
        evidence_class = _evidence_class(report)
        descriptor = {**_key_row(key), "report_index": index, "status": report.get("status")}
        buckets[evidence_class].append(descriptor)
        reasons = _report_reasons(
            matrix,
            algorithms[key[0]],
            cases[key[1]],
            report,
            quality_by_key.get(key) or quality_by_key.get(key[1]),
        )
        if reasons:
            invalid_runs.append({**descriptor, "reasons": reasons})
        else:
            accepted[key] = report

    coverage = _coverage(algorithms, cases, seeds, observed, accepted)
    baseline_deltas = _baseline_deltas(accepted, cases)
    rankings = _rankings(accepted, algorithms, cases)
    fail_reasons = []
    if missing:
        fail_reasons.append("missing expected algorithm/case/seed runs")
    if unexpected:
        fail_reasons.append("unexpected run identities are present")
    if duplicates:
        fail_reasons.append("duplicate algorithm/case/seed runs are present")
    if malformed:
        fail_reasons.append("malformed reports are present")
    if invalid_runs:
        fail_reasons.append("reports failed identity, evidence-class, KPI, or quality gates")
    ready = not fail_reasons
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "offline_conformance",
        "formal_evidence_status": (
            "remote_validation_required" if not ready else "comparison_ready"
        ),
        "matrix_id": matrix["matrix_id"],
        "matrix_sha256": matrix["immutable_matrix_sha256"],
        "expected_run_count": len(expected),
        "observed_report_count": len(reports),
        "accepted_run_count": len(accepted),
        "ready_for_formal_comparison": ready,
        "acceptance_passed": ready and all(
            (report.get("evaluation") or {}).get("overall_result") == "pass"
            for report in accepted.values()
        ),
        "fail_closed_reasons": fail_reasons,
        "missing": missing,
        "unexpected": unexpected,
        "duplicates": duplicates,
        "malformed": malformed,
        "invalid_runs": invalid_runs,
        "coverage": coverage,
        "case_statistics": _case_statistics(accepted, algorithms, cases),
        "evidence_buckets": buckets,
        "baseline_deltas": baseline_deltas,
        "baseline_delta_statistics": _baseline_delta_statistics(baseline_deltas),
        "rankings": rankings,
        "quality_stress_results": _quality_stress_results(accepted, cases, quality_by_key),
    }


def _report_key(report):
    experiment = report.get("experiment") or {}
    algorithm_id = experiment.get("algorithm_id")
    case_id = experiment.get("case_id")
    seed = experiment.get("seed")
    missing = [name for name, value in (("algorithm_id", algorithm_id), ("case_id", case_id), ("seed", seed)) if value is None]
    if missing:
        raise CounterfactualEvaluationError("report identity is missing: " + ", ".join(missing))
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise CounterfactualEvaluationError("report seed must be an integer")
    return str(algorithm_id), str(case_id), seed


def _quality_index(reports):
    result = {}
    for report in reports:
        experiment = report.get("experiment") or {}
        if all(experiment.get(name) is not None for name in ("algorithm_id", "case_id", "seed")):
            key = _report_key(report)
        else:
            key = report.get("case_id") or experiment.get("case_id")
            if not isinstance(key, str) or not key:
                raise CounterfactualEvaluationError(
                    "render-quality report requires either algorithm/case/seed identity or case_id"
                )
        if key in result:
            raise CounterfactualEvaluationError(f"duplicate render-quality report: {key}")
        result[key] = report
    return result


def _evidence_class(report):
    experiment = report.get("experiment") or {}
    value = (
        report.get("evidence_classification")
        or experiment.get("evidence_classification")
        or report.get("evidence_class")
        or experiment.get("evidence_class")
    )
    if report.get("status") == "offline_conformance":
        value = "offline_conformance"
    if value not in _EVIDENCE_CLASSES:
        return "remote_validation_required"
    return value


def _report_reasons(matrix, algorithm, case, report, quality_report):
    reasons = []
    experiment = report.get("experiment") or {}
    identity = experiment.get("identity") or report.get("identity") or {}
    scene_identity = matrix["scene_identity"]
    expected_identity = {
        "scene_id": scene_identity["scene_id"],
        "scene_version": scene_identity["scene_version"],
        "artifact_sha256": scene_identity["artifact_sha256"],
        "scene_package_sha256": scene_identity["scene_package_sha256"],
        "scenario_ir_sha256": scene_identity["scenario_ir_sha256"],
    }
    actual_identity = {
        "scene_id": report.get("scenario_id") or experiment.get("scene_id") or identity.get("scene_id"),
        "scene_version": experiment.get("scene_version") or identity.get("scene_version"),
        "artifact_sha256": identity.get("artifact_sha256"),
        "scene_package_sha256": identity.get("scene_package_sha256"),
        "scenario_ir_sha256": identity.get("scenario_ir_sha256"),
    }
    for name, expected in expected_identity.items():
        if actual_identity.get(name) != expected:
            reasons.append(f"identity mismatch for {name}: expected {expected!r}, got {actual_identity.get(name)!r}")
    if experiment.get("algorithm_version") != algorithm["algorithm_version"]:
        reasons.append("algorithm_version does not match the frozen matrix")
    if experiment.get("algorithm_config_sha256") != algorithm["config_sha256"]:
        reasons.append("algorithm_config_sha256 does not match the frozen matrix")
    expected_checkpoint = algorithm.get("checkpoint_sha256")
    if experiment.get("checkpoint_sha256") != expected_checkpoint:
        reasons.append("checkpoint identity does not match the frozen matrix")
    evidence_class = _evidence_class(report)
    if report.get("status") not in _RUNTIME_STATUSES:
        reasons.append(f"status {report.get('status')!r} is not remote runtime evidence")
    if evidence_class == "offline_conformance":
        reasons.append("offline_conformance cannot satisfy a remote closed-loop run")
    expected_class = "quality_stress" if case.get("quality_stress_only") else algorithm["execution_class"]
    if evidence_class != expected_class:
        reasons.append(f"evidence_class must be {expected_class!r}, got {evidence_class!r}")
    summary = report.get("summary") or {}
    availability = summary.get("metric_availability") or {}
    for metric in _required_case_metrics(case):
        if summary.get(metric) is None or availability.get(metric) is not True:
            reasons.append(f"required KPI is unavailable: {metric}")
    actor_outcomes = summary.get("actor_outcomes")
    for outcome in case.get("required_actor_outcomes") or []:
        if (
            availability.get("actor_outcomes") is not True
            or not isinstance(actor_outcomes, dict)
            or not _finite(actor_outcomes.get(outcome))
            or float(actor_outcomes[outcome]) <= 0.0
        ):
            reasons.append(f"required actor outcome is unavailable: {outcome}")
    evaluation = report.get("evaluation") or {}
    if evaluation.get("overall_result") not in {"pass", "fail"}:
        reasons.append("overall_result is unavailable")
    quality_class = _quality_class(quality_report or report.get("render_quality"))
    if case.get("quality_stress_only"):
        if quality_class not in {"quality_stress", "rejected"}:
            reasons.append("quality-stress case requires quality_stress or rejected render classification")
        reasons.extend(_quality_identity_reasons(matrix, quality_report))
    elif algorithm.get("perception_algorithm"):
        if not case.get("perception_ranking_allowed"):
            reasons.append("case is excluded from perception ranking")
        elif quality_class != "perception_eligible":
            reasons.append("perception run requires a perception_eligible render-quality report")
        reasons.extend(_quality_identity_reasons(matrix, quality_report))
    return reasons


def _required_case_metrics(case):
    if case.get("quality_stress_only"):
        return ()
    return ("collision_count", "route_progress", "min_ttc")


def _quality_class(report):
    if not isinstance(report, dict):
        return None
    return (
        report.get("evidence_classification")
        or report.get("classification")
        or (report.get("aggregate") or {}).get("evidence_classification")
        or (report.get("aggregate") or {}).get("classification")
    )


def _quality_identity_reasons(matrix, report):
    if not isinstance(report, dict):
        return ["render-quality report is missing"]
    identity = (report.get("experiment") or {}).get("identity") or report.get("identity") or {}
    expected = matrix["scene_identity"]
    reasons = []
    scene_id = report.get("scene_id") or identity.get("scene_id")
    if scene_id is not None and scene_id != expected["scene_id"]:
        reasons.append("render-quality identity mismatch for scene_id")
    artifact_sha = identity.get("artifact_sha256") or (report.get("artifact") or {}).get("sha256")
    if artifact_sha != expected["artifact_sha256"]:
        reasons.append("render-quality identity mismatch for artifact_sha256")
    for key in ("scene_package_sha256", "scenario_ir_sha256"):
        if identity.get(key) is not None and identity.get(key) != expected[key]:
            reasons.append(f"render-quality identity mismatch for {key}")
    return reasons


def _coverage(algorithms, cases, seeds, observed, accepted):
    rows = []
    for algorithm_id in algorithms:
        for case_id in cases:
            valid = [seed for seed in seeds if (algorithm_id, case_id, seed) in accepted]
            present = [seed for seed in seeds if (algorithm_id, case_id, seed) in observed]
            rows.append(
                {
                    "algorithm_id": algorithm_id,
                    "case_id": case_id,
                    "expected_seeds": seeds,
                    "present_seeds": present,
                    "accepted_seeds": valid,
                    "missing_seeds": [seed for seed in seeds if seed not in present],
                    "invalid_seeds": [seed for seed in present if seed not in valid],
                    "triplicate_complete": len(valid) == len(seeds) == 3,
                }
            )
    return rows


def _case_statistics(accepted, algorithms, cases):
    rows = []
    for algorithm_id in algorithms:
        for case_id in cases:
            reports = [
                report
                for (candidate_algorithm, candidate_case, _), report in accepted.items()
                if candidate_algorithm == algorithm_id and candidate_case == case_id
            ]
            results = [(report.get("evaluation") or {}).get("overall_result") for report in reports]
            metric_stats = {}
            for metric in _DELTA_METRICS:
                values = [
                    float((report.get("summary") or {})[metric])
                    for report in reports
                    if _finite((report.get("summary") or {}).get(metric))
                ]
                metric_stats[metric] = {
                    "available_seed_count": len(values),
                    "mean": mean(values) if values else None,
                    "std": pstdev(values) if values else None,
                }
            rows.append(
                {
                    "algorithm_id": algorithm_id,
                    "case_id": case_id,
                    "accepted_seed_count": len(reports),
                    "failure_rate": results.count("fail") / len(reports) if reports else None,
                    "metrics": metric_stats,
                }
            )
    return rows


def _baseline_deltas(accepted, cases):
    rows = []
    for (algorithm_id, case_id, seed), report in sorted(accepted.items()):
        if case_id == "S0_original_replay" or cases[case_id].get("quality_stress_only"):
            continue
        baseline = accepted.get((algorithm_id, "S0_original_replay", seed))
        if baseline is None:
            continue
        baseline_summary = baseline.get("summary") or {}
        edited_summary = report.get("summary") or {}
        deltas = {}
        for metric in _DELTA_METRICS:
            before = baseline_summary.get(metric)
            after = edited_summary.get(metric)
            if _finite(before) and _finite(after):
                deltas[metric] = float(after) - float(before)
            else:
                deltas[metric] = None
        rows.append({"algorithm_id": algorithm_id, "case_id": case_id, "seed": seed, "edited_minus_baseline": deltas})
    return rows


def _baseline_delta_statistics(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["algorithm_id"], row["case_id"])].append(row)
    result = []
    for (algorithm_id, case_id), group in sorted(grouped.items()):
        metrics = {}
        for metric in _DELTA_METRICS:
            values = [
                float(row["edited_minus_baseline"][metric])
                for row in group
                if _finite(row["edited_minus_baseline"].get(metric))
            ]
            metrics[metric] = {
                "available_seed_count": len(values),
                "mean": mean(values) if values else None,
                "std": pstdev(values) if values else None,
            }
        result.append({"algorithm_id": algorithm_id, "case_id": case_id, "metrics": metrics})
    return result


def _rankings(accepted, algorithms, cases):
    buckets = {"control_only": defaultdict(list), "perception_eligible": defaultdict(list)}
    for (algorithm_id, case_id, _), report in accepted.items():
        if cases[case_id].get("quality_stress_only"):
            continue
        category = "perception_eligible" if algorithms[algorithm_id].get("perception_algorithm") else "control_only"
        buckets[category][algorithm_id].append(report)
    return {
        category: [
            _ranking_row(algorithm_id, reports)
            for algorithm_id, reports in sorted(grouped.items())
        ]
        for category, grouped in buckets.items()
    }


def _ranking_row(algorithm_id, reports):
    evaluation_results = [(report.get("evaluation") or {}).get("overall_result") for report in reports]
    stats = {}
    for metric in _DELTA_METRICS:
        values = [float((report.get("summary") or {})[metric]) for report in reports if _finite((report.get("summary") or {}).get(metric))]
        stats[metric] = {
            "available_run_count": len(values),
            "mean": mean(values) if values else None,
            "std": pstdev(values) if values else None,
        }
    return {
        "algorithm_id": algorithm_id,
        "run_count": len(reports),
        "failure_rate": evaluation_results.count("fail") / len(reports) if reports else None,
        "metrics": stats,
    }


def _quality_stress_results(accepted, cases, quality_by_key):
    rows = []
    for key, report in sorted(accepted.items()):
        if not cases[key[1]].get("quality_stress_only"):
            continue
        quality = quality_by_key.get(key) or quality_by_key.get(key[1]) or report.get("render_quality") or {}
        rows.append({**_key_row(key), "classification": _quality_class(quality), "ranked": False})
    return rows


def _key_row(key):
    return {"algorithm_id": key[0], "case_id": key[1], "seed": key[2]}


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))
