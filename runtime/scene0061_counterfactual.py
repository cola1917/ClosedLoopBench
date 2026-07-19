from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
import json
import re
from typing import Any


SCHEMA_VERSION = "scene_counterfactual_matrix.v1"
VEHICLE_TRACK = "c1958768d48640948f6053d04cffd35b"
PEDESTRIAN_TRACK = "71603dd1a2ba4e9daf095535e38310ac"
CASE_IDS = (
    "S0_original_replay",
    "S1_lead_slowdown",
    "S2_lead_hard_brake",
    "S3_lead_longitudinal_shift",
    "S4_pedestrian_early_crossing",
    "S5_pedestrian_yield",
    "S6_pedestrian_noncompliant",
    "S7_lead_removed_quality_stress",
)
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_PEDESTRIAN_ACTIONS = {"speed", "pause", "yield", "abort"}


class CounterfactualMatrixError(ValueError):
    """Raised when the frozen scene-0061 experiment contract is unsafe or ambiguous."""


def build_scene0061_counterfactual_matrix(
    *, created_at: str = "2026-07-19T00:00:00Z"
) -> dict[str, Any]:
    algorithms = [
        _algorithm("reference_pure_pursuit_short", "short", 4.0, 6.0),
        _algorithm("reference_pure_pursuit_long", "long", 9.0, 8.0),
    ]
    matrix: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "matrix_id": "scene0061-light-counterfactual-v1",
        "created_at": created_at,
        "evidence_semantics": {
            "offline_conformance": "interface evidence only; never closes a simulator run",
            "control_only": "physical control evidence without learned perception",
            "perception_eligible": "requires a passing render-quality report and a perception algorithm",
            "quality_stress": "reported separately and excluded from driving rankings",
            "remote_validation_required": True,
        },
        "scene_identity": {
            "scene_id": "cc8c0bf57f984915a77078b10eb33198",
            "scene_name": "scene-0061",
            "scene_version": "formal40k-v1",
            "nurec_run_id": "9aChcizbAsm4oDQKJMdBHM",
            "artifact_sha256": "69e2c36e31e113f9ad66968a0a0a4243c7989dae5582a91a43d150051eaf98b4",
            "scene_package_sha256": "0d6b724b0dea9ff3f97717f893f19baf69904057511ad374cfd510c5cc9b9119",
            "scenario_ir_sha256": "ae340b43c2ecbcf416cb89895e63ea59241b240ff83bc9dc4e6f1632a3f1ded7",
        },
        "immutable_inputs": [
            {
                "role": "nurec_usdz",
                "logical_ref": "formal://scene-0061/9aChcizbAsm4oDQKJMdBHM/artifacts/last.usdz",
                "sha256": "69e2c36e31e113f9ad66968a0a0a4243c7989dae5582a91a43d150051eaf98b4",
                "immutable": True,
            },
            {
                "role": "runtime_validated_scene_package",
                "logical_ref": "evidence://scene0061/runtime/scene_package.json",
                "sha256": "0d6b724b0dea9ff3f97717f893f19baf69904057511ad374cfd510c5cc9b9119",
                "immutable": True,
            },
            {
                "role": "actor_ready_scenario_ir",
                "logical_ref": "evidence://scene0061/runtime/scenario_ir.actor-ready.json",
                "sha256": "ae340b43c2ecbcf416cb89895e63ea59241b240ff83bc9dc4e6f1632a3f1ded7",
                "immutable": True,
            },
            {
                "role": "actor_selection",
                "logical_ref": "repo://examples/scene0061_actor_selection.v1.json",
                "sha256": "7ae4c2bd30efca80ba1f3f8ab746364cceb9c9d530f7a955b675531f19cd3fe9",
                "immutable": True,
            },
            {
                "role": "opendrive",
                "logical_ref": "evidence://scene0061/road.nurec-route-extended-both-v7.xodr",
                "sha256": "d3913c4d0019d4c9165ae90e2a5025703ed5e1b423d688168951428341892537",
                "immutable": True,
            },
        ],
        "actors": {
            "lead_vehicle": {
                "track_id": VEHICLE_TRACK,
                "type": "vehicle",
                "control_mode": "scripted",
                "allowed_edits": ["speed", "brake", "longitudinal_shift", "remove_quality_stress_only"],
                "limits": {"speed_scale": [0.0, 1.25], "longitudinal_shift_m": [-3.0, 3.0]},
            },
            "pedestrian": {
                "track_id": PEDESTRIAN_TRACK,
                "type": "pedestrian",
                "control_mode": "scripted",
                "motion_constraint": "source_reference_corridor",
                "allowed_actions": sorted(_PEDESTRIAN_ACTIONS),
                "free_space_path_allowed": False,
                "skeleton_edit_allowed": False,
            },
        },
        "algorithms": algorithms,
        "seeds": [41, 42, 43],
        "cases": _cases(),
    }
    matrix["immutable_matrix_sha256"] = _matrix_digest(matrix)
    validate_scene0061_counterfactual_matrix(matrix)
    return matrix


def validate_scene0061_counterfactual_matrix(matrix: dict[str, Any]) -> None:
    if matrix.get("schema_version") != SCHEMA_VERSION:
        raise CounterfactualMatrixError("unsupported counterfactual matrix schema")
    identity = matrix.get("scene_identity") or {}
    if identity.get("scene_id") != "cc8c0bf57f984915a77078b10eb33198":
        raise CounterfactualMatrixError("scene identity is not formal scene-0061")
    for name in ("artifact_sha256", "scene_package_sha256", "scenario_ir_sha256"):
        _require_hash(identity.get(name), f"scene_identity.{name}")
    inputs = matrix.get("immutable_inputs")
    if not isinstance(inputs, list) or not inputs:
        raise CounterfactualMatrixError("immutable_inputs must be non-empty")
    for item in inputs:
        if item.get("immutable") is not True:
            raise CounterfactualMatrixError("every immutable input must be marked immutable")
        if not str(item.get("logical_ref") or "").startswith(("formal://", "evidence://", "repo://")):
            raise CounterfactualMatrixError("immutable input must use a logical reference")
        _require_hash(item.get("sha256"), f"immutable input {item.get('role')}")
    actors = matrix.get("actors") or {}
    if (actors.get("lead_vehicle") or {}).get("track_id") != VEHICLE_TRACK:
        raise CounterfactualMatrixError("formal lead vehicle track is required")
    pedestrian = actors.get("pedestrian") or {}
    if pedestrian.get("track_id") != PEDESTRIAN_TRACK:
        raise CounterfactualMatrixError("formal pedestrian track is required")
    if pedestrian.get("motion_constraint") != "source_reference_corridor":
        raise CounterfactualMatrixError("pedestrian must stay in the source reference corridor")
    if pedestrian.get("free_space_path_allowed") is not False or pedestrian.get("skeleton_edit_allowed") is not False:
        raise CounterfactualMatrixError("free-space pedestrian or skeleton edits are forbidden")
    seeds = matrix.get("seeds")
    if not isinstance(seeds, list) or len(seeds) < 3 or len(seeds) != len(set(seeds)):
        raise CounterfactualMatrixError("at least three unique seeds are required")
    algorithms = matrix.get("algorithms")
    if not isinstance(algorithms, list) or not algorithms:
        raise CounterfactualMatrixError("algorithms must be non-empty")
    algorithm_ids = [row.get("algorithm_id") for row in algorithms]
    if len(algorithm_ids) != len(set(algorithm_ids)):
        raise CounterfactualMatrixError("algorithm_id values must be unique")
    for algorithm in algorithms:
        if algorithm.get("plugin_algorithm_id") != "reference_pure_pursuit":
            raise CounterfactualMatrixError("frozen baseline must bind the reference_pure_pursuit plugin")
        _require_hash(algorithm.get("config_sha256"), f"algorithm {algorithm.get('algorithm_id')} config")
        if algorithm.get("config_sha256") != _json_digest(algorithm.get("parameters") or {}):
            raise CounterfactualMatrixError("algorithm config hash does not match parameters")
        if algorithm.get("perception_algorithm") is not False:
            raise CounterfactualMatrixError("frozen local baseline algorithms must be control-only")
    cases = matrix.get("cases")
    if not isinstance(cases, list) or tuple(row.get("case_id") for row in cases) != CASE_IDS:
        raise CounterfactualMatrixError("cases must contain ordered S0-S7 exactly once")
    for case in cases:
        _validate_case(case)
    expected_digest = matrix.get("immutable_matrix_sha256")
    _require_hash(expected_digest, "immutable_matrix_sha256")
    if expected_digest != _matrix_digest(matrix):
        raise CounterfactualMatrixError("immutable_matrix_sha256 does not match matrix contents")


def _algorithm(
    algorithm_id: str,
    lookahead_profile: str,
    lookahead_m: float,
    target_speed_mps: float,
) -> dict[str, Any]:
    parameters = {
        "profile": lookahead_profile,
        "lookahead_m": lookahead_m,
        "target_speed_mps": target_speed_mps,
        "supported_control_hz": 20.0,
        "timeout_sec": 0.05,
    }
    return {
        "algorithm_id": algorithm_id,
        "plugin_algorithm_id": "reference_pure_pursuit",
        "algorithm_version": "reference_pure_pursuit.v1",
        "execution_class": "control_only",
        "perception_algorithm": False,
        "checkpoint_sha256": "not_applicable",
        "parameters": parameters,
        "config_sha256": _json_digest(parameters),
        "remote_validation_required": True,
    }


def _case(
    case_id,
    actor,
    operation,
    parameters,
    expected,
    gates,
    *,
    perception=True,
    stress=False,
    required_actor_outcomes=(),
):
    track = None if actor is None else (VEHICLE_TRACK if actor == "lead_vehicle" else PEDESTRIAN_TRACK)
    source_tracks = [VEHICLE_TRACK, PEDESTRIAN_TRACK] if case_id == "S0_original_replay" else ([] if track is None else [track])
    return {
        "case_id": case_id,
        "scene_identity_ref": "#/scene_identity",
        "immutable_input_roles": [
            "nurec_usdz",
            "runtime_validated_scene_package",
            "actor_ready_scenario_ir",
            "actor_selection",
            "opendrive",
        ],
        "source_actor_tracks": source_tracks,
        "actor_control_mode": "replay" if case_id == "S0_original_replay" else "scripted",
        "edit": {"actor": actor, "operation": operation, "parameters": parameters},
        "trajectory_delta": deepcopy(parameters),
        "expected_behavior": expected,
        "kpi_gate": gates,
        "required_actor_outcomes": list(required_actor_outcomes),
        "perception_ranking_allowed": perception,
        "quality_stress_only": stress,
        "remote_validation_required": True,
    }


def _cases():
    safety = ["collision_count==0", "route_progress>=0.95", "min_ttc_available"]
    return [
        _case("S0_original_replay", None, "none", {}, "source replay baseline", safety),
        _case("S1_lead_slowdown", "lead_vehicle", "speed", {"speed_scale": 0.55}, "ego follows and adapts speed", safety),
        _case("S2_lead_hard_brake", "lead_vehicle", "brake", {"deceleration_mps2": 5.0, "duration_sec": 1.0}, "ego brakes without collision", safety),
        _case("S3_lead_longitudinal_shift", "lead_vehicle", "longitudinal_shift", {"longitudinal_shift_m": 2.0}, "ego response changes with bounded lead pose", safety),
        _case("S4_pedestrian_early_crossing", "pedestrian", "speed", {"action": "speed", "time_shift_sec": -1.0, "corridor": "source_reference_corridor"}, "ego yields to an earlier source-corridor crossing", safety + ["pedestrian_crossing_outcome_available"], required_actor_outcomes=("crossing",)),
        _case("S5_pedestrian_yield", "pedestrian", "yield", {"action": "yield", "pause_sec": 1.0, "corridor": "source_reference_corridor"}, "pedestrian yields and ego proceeds safely", safety + ["pedestrian_yield_outcome_available"], required_actor_outcomes=("yield",)),
        _case("S6_pedestrian_noncompliant", "pedestrian", "speed", {"action": "speed", "speed_scale": 1.0, "yield_enabled": False, "corridor": "source_reference_corridor"}, "non-yielding pedestrian remains on source corridor", safety + ["pedestrian_crossing_outcome_available"], required_actor_outcomes=("crossing",)),
        _case("S7_lead_removed_quality_stress", "lead_vehicle", "remove", {"background_synthesis": False}, "measure black-hole and perception robustness without ranking driving skill", ["render_quality_report_required"], perception=False, stress=True),
    ]


def _validate_case(case):
    if case.get("scene_identity_ref") != "#/scene_identity":
        raise CounterfactualMatrixError(f"{case.get('case_id')} must bind the frozen scene identity")
    required_roles = {"nurec_usdz", "runtime_validated_scene_package", "actor_ready_scenario_ir"}
    if not required_roles.issubset(set(case.get("immutable_input_roles") or [])):
        raise CounterfactualMatrixError(f"{case.get('case_id')} is missing immutable scene inputs")
    if case.get("remote_validation_required") is not True:
        raise CounterfactualMatrixError(f"{case.get('case_id')} must require remote validation")
    edit = case.get("edit") or {}
    actor = edit.get("actor")
    operation = edit.get("operation")
    parameters = edit.get("parameters") or {}
    required_outcomes = case.get("required_actor_outcomes")
    if not isinstance(required_outcomes, list) or any(
        outcome not in {"crossing", "yield", "abort"} for outcome in required_outcomes
    ):
        raise CounterfactualMatrixError(f"{case['case_id']} has invalid required_actor_outcomes")
    if required_outcomes and actor != "pedestrian":
        raise CounterfactualMatrixError("actor outcomes may only be required for pedestrian cases")
    if actor == "pedestrian":
        if operation not in _PEDESTRIAN_ACTIONS or parameters.get("action") not in _PEDESTRIAN_ACTIONS:
            raise CounterfactualMatrixError(f"{case['case_id']} uses a forbidden pedestrian action")
        if parameters.get("corridor") != "source_reference_corridor":
            raise CounterfactualMatrixError(f"{case['case_id']} leaves the source pedestrian corridor")
    if actor == "lead_vehicle" and operation == "longitudinal_shift":
        shift = parameters.get("longitudinal_shift_m")
        if not isinstance(shift, (int, float)) or abs(float(shift)) > 3.0:
            raise CounterfactualMatrixError("lead vehicle longitudinal shift exceeds the light-edit bound")
    if operation == "remove":
        if case.get("quality_stress_only") is not True or case.get("perception_ranking_allowed") is not False:
            raise CounterfactualMatrixError("actor removal must be quality-stress-only and excluded from rankings")


def _matrix_digest(matrix):
    payload = deepcopy(matrix)
    payload.pop("immutable_matrix_sha256", None)
    return _json_digest(payload)


def _json_digest(payload):
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return sha256(encoded).hexdigest()


def _require_hash(value, label):
    if not isinstance(value, str) or not _SHA256.fullmatch(value):
        raise CounterfactualMatrixError(f"{label} must be a lowercase sha256")
