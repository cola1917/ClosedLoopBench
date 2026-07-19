from __future__ import annotations

import argparse
import json
import math
import sys
import time
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.algorithm_backend import AlgorithmBackendError, load_backend
from agents.model_plugin_wrappers import build_external_model_runtime_manifest
from agents.plugin_contract import AlgorithmPluginExecutor, PluginContractError, SAFE_STOP_CONTROL


class _GuardFixturePlugin:
    def __init__(
        self,
        *,
        mode: str = "normal",
        required_cameras: list[str] | None = None,
        requires_lidar: bool = False,
        healthy: bool = True,
        timeout_sec: float = 0.05,
    ) -> None:
        self.mode = mode
        self.healthy = healthy
        self.closed = False
        self.capability = {
            "algorithm_id": "contract_guard_fixture",
            "uses_route": True,
            "uses_ego_state": True,
            "required_rgb_cameras": required_cameras or [],
            "requires_lidar": requires_lidar,
            "is_perception_algorithm": bool(required_cameras or requires_lidar),
            "requires_gpu": False,
            "checkpoint_identity": "not_applicable",
            "supported_control_hz": 20.0,
            "timeout_sec": timeout_sec,
        }

    def initialize(self, config: Mapping[str, Any]) -> None:
        self.closed = False

    def reset(self, scene_context: Mapping[str, Any]) -> None:
        pass

    def predict_control(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        if self.mode == "exception":
            raise RuntimeError("injected backend exception")
        if self.mode == "slow":
            time.sleep(0.005)
        throttle: float = 0.2
        if self.mode == "range":
            throttle = 2.0
        elif self.mode == "nan":
            throttle = math.nan
        source_frame = observation["frame_id"] + (1 if self.mode == "frame_mismatch" else 0)
        return {
            "throttle": throttle,
            "steer": 0.0,
            "brake": 0.0,
            "hand_brake": False,
            "reverse": False,
            "source_frame_id": source_frame,
        }

    def health_check(self) -> dict[str, Any]:
        return {"status": "ready" if self.healthy and not self.closed else "unhealthy"}

    def close(self) -> None:
        self.closed = True


def synthetic_observation(
    capability: Mapping[str, Any], *, frame_id: int = 1
) -> dict[str, Any]:
    observation = {
        "frame_id": frame_id,
        "timestamp": frame_id * 0.05,
        "rgb": {
            camera: f"offline://{camera}/{frame_id}.png"
            for camera in capability["required_rgb_cameras"]
        },
        "lidar": "offline://lidar/1.bin" if capability["requires_lidar"] else None,
        "calibration": {},
        "ego_state": {
            "speed_mps": 2.0,
            "pose": {"x": float(frame_id - 1), "y": 0.0, "yaw": 0.0},
        },
        "route": {
            "route_waypoints": [[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]],
            "route_command": "LANE_FOLLOW",
            "target_point": [10.0, 0.0],
        },
        "sensor_validity": {
            **{camera: True for camera in capability["required_rgb_cameras"]},
            "lidar": True,
        },
        "synchronization": {"frame_id": frame_id, "max_error_sec": 0.0},
    }
    return observation


def _executor(plugin: Any, config: dict[str, Any] | None = None) -> AlgorithmPluginExecutor:
    runtime = AlgorithmPluginExecutor(
        plugin,
        config or {"repo_path": "."},
        evidence_classification="offline_conformance",
    )
    runtime.initialize()
    runtime.reset({"scene_id": "offline-conformance"})
    return runtime


def _case(name: str, passed: bool, **detail: Any) -> dict[str, Any]:
    return {
        "name": name,
        "execution_status": "passed" if passed else "failed",
        "evidence_classification": "offline_conformance",
        "detail": detail,
    }


def _is_safe_stop(result: Mapping[str, Any], reason: str | None = None) -> bool:
    control = result.get("control", {})
    expected = all(control.get(key) == value for key, value in SAFE_STOP_CONTROL.items())
    return (
        result.get("execution_status") == "fallback"
        and expected
        and control.get("status") == "safe_stop"
        and (reason is None or control.get("reason") == reason)
    )


def run_contract_guard_suite() -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []

    normal = _executor(_GuardFixturePlugin())
    normal_result = normal.predict(synthetic_observation(normal.capability))
    cases.append(_case("normal_control", normal_result["execution_status"] == "control"))
    normal.close()

    missing_camera = _executor(_GuardFixturePlugin(required_cameras=["rgb_front"]))
    obs = synthetic_observation(missing_camera.capability)
    obs["rgb"] = {}
    result = missing_camera.predict(obs)
    cases.append(_case("missing_required_camera", _is_safe_stop(result, "missing_sensors")))
    missing_camera.close()

    missing_lidar = _executor(_GuardFixturePlugin(requires_lidar=True))
    obs = synthetic_observation(missing_lidar.capability)
    obs["lidar"] = None
    result = missing_lidar.predict(obs)
    cases.append(_case("missing_lidar", _is_safe_stop(result, "missing_sensors")))
    missing_lidar.close()

    stale = _executor(_GuardFixturePlugin())
    stale.predict(synthetic_observation(stale.capability, frame_id=2))
    result = stale.predict(synthetic_observation(stale.capability, frame_id=2))
    cases.append(_case("stale_frame", _is_safe_stop(result, "stale_frame")))
    stale.close()

    sync = _executor(_GuardFixturePlugin())
    obs = synthetic_observation(sync.capability)
    obs["synchronization"]["frame_id"] = 9
    result = sync.predict(obs)
    cases.append(_case("observation_frame_mismatch", _is_safe_stop(result, "frame_mismatch")))
    sync.close()

    output_frame = _executor(_GuardFixturePlugin(mode="frame_mismatch"))
    result = output_frame.predict(synthetic_observation(output_frame.capability))
    cases.append(_case("control_frame_mismatch", _is_safe_stop(result, "frame_mismatch")))
    output_frame.close()

    timeout = _executor(_GuardFixturePlugin(mode="slow", timeout_sec=0.001))
    result = timeout.predict(synthetic_observation(timeout.capability))
    cases.append(_case("inference_timeout", _is_safe_stop(result, "timeout")))
    timeout.close()

    exception = _executor(_GuardFixturePlugin(mode="exception"))
    result = exception.predict(synthetic_observation(exception.capability))
    cases.append(_case("backend_exception", _is_safe_stop(result, "backend_exception")))
    exception.close()

    invalid_range = _executor(_GuardFixturePlugin(mode="range"))
    result = invalid_range.predict(synthetic_observation(invalid_range.capability))
    cases.append(_case("invalid_control_range", _is_safe_stop(result, "invalid_control")))
    invalid_range.close()

    invalid_nan = _executor(_GuardFixturePlugin(mode="nan"))
    result = invalid_nan.predict(synthetic_observation(invalid_nan.capability))
    cases.append(_case("nan_inf_control", _is_safe_stop(result, "invalid_control")))
    invalid_nan.close()

    health_plugin = _GuardFixturePlugin()
    health = _executor(health_plugin)
    health_plugin.healthy = False
    result = health.predict(synthetic_observation(health.capability))
    cases.append(_case("health_check_failure", _is_safe_stop(result, "health_check_failure")))
    health.close()

    lifecycle_plugin = _GuardFixturePlugin()
    lifecycle = AlgorithmPluginExecutor(
        lifecycle_plugin,
        {"repo_path": "."},
        evidence_classification="offline_conformance",
    )
    lifecycle.initialize()
    before_reset = lifecycle.predict(synthetic_observation(lifecycle.capability))
    lifecycle.reset({})
    lifecycle.close()
    after_close = lifecycle.predict(synthetic_observation(lifecycle.capability))
    cases.append(
        _case(
            "reset_close_lifecycle",
            _is_safe_stop(before_reset, "lifecycle_not_ready")
            and _is_safe_stop(after_close, "lifecycle_not_ready"),
        )
    )

    manifest = build_external_model_runtime_manifest(
        "tcp",
        {
            "repo_path": "definitely-not-present/repo",
            "checkpoint_path": "definitely-not-present/checkpoint.pth",
        },
    )
    cases.append(
        _case(
            "checkpoint_not_found",
            manifest["execution_status"] == "blocked"
            and "checkpoint_path_unavailable" in manifest["problems"]
            and manifest["real_checkpoint_loaded"] is False,
        )
    )

    mismatch = _executor(_GuardFixturePlugin(required_cameras=["rgb_front"]))
    obs = synthetic_observation(mismatch.capability)
    obs["rgb"].pop("rgb_front")
    result = mismatch.predict(obs)
    cases.append(_case("capability_input_mismatch", _is_safe_stop(result, "missing_sensors")))
    mismatch.close()

    fallback_cases = [case for case in cases if case["name"] != "normal_control"]
    cases.append(
        _case(
            "safe_stop_policy",
            all(case["execution_status"] == "passed" for case in fallback_cases),
            guarded_case_count=len(fallback_cases),
        )
    )
    return cases


def run_plugin_conformance(
    plugin_spec: str, config: dict[str, Any]
) -> dict[str, Any]:
    cases = run_contract_guard_suite()
    candidate_error = None
    candidate_identity: dict[str, Any] = {}
    try:
        backend = load_backend(plugin_spec, config)
        candidate = AlgorithmPluginExecutor(
            backend,
            config,
            already_initialized=True,
            evidence_classification="offline_conformance",
        )
        candidate.initialize()
        candidate.reset({"scene_id": "offline-conformance"})
        observation = synthetic_observation(candidate.capability)
        first = candidate.predict(observation)
        candidate.reset({"scene_id": "offline-conformance"})
        second = candidate.predict(deepcopy(observation))
        stable_first = deepcopy(first.get("control"))
        stable_second = deepcopy(second.get("control"))
        stable_first.pop("inference_ms", None)
        stable_second.pop("inference_ms", None)
        candidate_ok = (
            first["execution_status"] == "control"
            and second["execution_status"] == "control"
            and stable_first == stable_second
        )
        candidate_identity = deepcopy(candidate.identity)
        candidate.close()
    except Exception as exc:
        candidate_ok = False
        candidate_error = str(exc)
    cases.append(
        _case(
            "candidate_plugin_lifecycle_and_determinism",
            candidate_ok,
            error=candidate_error,
        )
    )
    passed = all(case["execution_status"] == "passed" for case in cases)
    return {
        "schema_version": "algorithm_plugin_conformance.v1",
        "execution_status": "passed" if passed else "failed",
        "evidence_classification": "offline_conformance",
        "plugin": plugin_spec,
        "plugin_identity": candidate_identity,
        "case_count": len(cases),
        "passed_count": sum(case["execution_status"] == "passed" for case in cases),
        "remote_validation_queue_eligible": passed,
        "remote_validation_required": True,
        "real_carla_nurec_closed_loop": False,
        "cases": cases,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run fail-closed ego plugin conformance checks.")
    parser.add_argument("--plugin", required=True, help="module:factory")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            parser.error(f"refusing to overwrite existing output: {args.output}")
        config = json.loads(args.config.read_text(encoding="utf-8"))
        report = run_plugin_conformance(args.plugin, config)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        print(json.dumps(report, sort_keys=True))
        return 0 if report["execution_status"] == "passed" else 2
    except (OSError, ValueError, json.JSONDecodeError, AlgorithmBackendError, PluginContractError) as exc:
        print(json.dumps({"execution_status": "failed", "error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
