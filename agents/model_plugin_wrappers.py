from __future__ import annotations

import math
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.plugin_contract import canonical_sha256, file_sha256


MODEL_SPECS = {
    "tcp": {
        "algorithm_id": "tcp",
        "required_rgb_cameras": ["rgb_front"],
        "requires_lidar": False,
        "default_control_hz": 20.0,
    },
    "transfuser": {
        "algorithm_id": "transfuser",
        "required_rgb_cameras": ["rgb_front"],
        "requires_lidar": True,
        "default_control_hz": 20.0,
    },
}


def build_external_model_config_schema(algorithm: str) -> dict[str, Any]:
    spec = _spec(algorithm)
    return {
        "schema_version": "external_model_plugin_config.v1",
        "algorithm": spec["algorithm_id"],
        "required": ["repo_path", "checkpoint_path", "repo_sha256", "checkpoint_sha256"],
        "optional": {
            "camera_preprocess": {
                "resize": [900, 256],
                "color_order": "RGB",
                "normalization": "model_owned",
            },
            "supported_control_hz": spec["default_control_hz"],
            "timeout_sec": 0.5,
            "allow_test_backend": False,
            "recorded_controls": [],
        },
        "boundaries": {
            "image_decode": "wrapper_validates references; model repository owns tensor decode",
            "route_command": "wrapper passes command and target point without model-specific encoding",
            "control_normalization": "wrapper emits the common bounded vehicle-control contract",
            "model_code_vendored": False,
        },
    }


def build_external_model_runtime_manifest(
    algorithm: str, config: Mapping[str, Any]
) -> dict[str, Any]:
    spec = _spec(algorithm)
    repo_path = Path(str(config.get("repo_path", ""))).expanduser()
    checkpoint_path = Path(str(config.get("checkpoint_path", ""))).expanduser()
    repo_available = repo_path.is_dir()
    checkpoint_available = checkpoint_path.is_file()
    checkpoint_hash = config.get("checkpoint_sha256")
    actual_checkpoint_hash = file_sha256(checkpoint_path) if checkpoint_available else None
    problems = []
    if not repo_available:
        problems.append("repo_path_unavailable")
    if not checkpoint_available:
        problems.append("checkpoint_path_unavailable")
    if not config.get("repo_sha256"):
        problems.append("repo_sha256_missing")
    elif not _sha256(config.get("repo_sha256")):
        problems.append("repo_sha256_invalid")
    if not checkpoint_hash:
        problems.append("checkpoint_sha256_missing")
    else:
        if not _sha256(checkpoint_hash):
            problems.append("checkpoint_sha256_invalid")
        if actual_checkpoint_hash and str(checkpoint_hash) != actual_checkpoint_hash:
            problems.append("checkpoint_sha256_mismatch")
    runtime_config = {
        key: value
        for key, value in dict(config).items()
        if key not in {"recorded_controls"} and not callable(value)
    }
    return {
        "schema_version": "external_model_runtime_manifest.v1",
        "algorithm": spec["algorithm_id"],
        "execution_status": "prepared" if not problems else "blocked",
        "evidence_classification": "remote_validation_required",
        "real_checkpoint_loaded": False,
        "remote_gpu_validation_required": True,
        "repo_available": repo_available,
        "checkpoint_available": checkpoint_available,
        "repo_sha256": config.get("repo_sha256", "unavailable"),
        "checkpoint_sha256": checkpoint_hash or "unavailable",
        "config_sha256": canonical_sha256(runtime_config),
        "problems": problems,
        "model_code_vendored": False,
    }


class ExternalModelPluginWrapper:
    """Stable boundary for TCP/TransFuser without importing or pretending to run them."""

    def __init__(self, algorithm: str) -> None:
        self.algorithm = _spec(algorithm)["algorithm_id"]
        spec = MODEL_SPECS[self.algorithm]
        self.capability = {
            "algorithm_id": self.algorithm,
            "uses_route": True,
            "uses_ego_state": True,
            "required_rgb_cameras": list(spec["required_rgb_cameras"]),
            "requires_lidar": bool(spec["requires_lidar"]),
            "is_perception_algorithm": True,
            "requires_gpu": True,
            "checkpoint_identity": "external_required",
            "supported_control_hz": spec["default_control_hz"],
            "timeout_sec": 0.5,
        }
        self._initialized = False
        self._closed = False
        self._cursor = 0
        self._test_backend = False
        self.manifest: dict[str, Any] = {}

    def initialize(self, config: Mapping[str, Any]) -> None:
        if config.get("real_checkpoint_loaded") is True:
            raise ValueError(
                "local wrapper cannot claim real_checkpoint_loaded; remote model binding must prove it"
            )
        self.config = deepcopy(dict(config))
        self.manifest = build_external_model_runtime_manifest(self.algorithm, config)
        self.capability["supported_control_hz"] = _positive(
            config.get("supported_control_hz", self.capability["supported_control_hz"]),
            "supported_control_hz",
        )
        self.capability["timeout_sec"] = _positive(
            config.get("timeout_sec", 0.5), "timeout_sec"
        )
        self._recorded_controls = deepcopy(list(config.get("recorded_controls", [])))
        self._test_backend = bool(config.get("allow_test_backend", False))
        if self._test_backend and not self._recorded_controls:
            self._recorded_controls = [
                {
                    "throttle": 0.0,
                    "steer": 0.0,
                    "brake": 1.0,
                    "hand_brake": False,
                    "reverse": False,
                }
            ]
        self._initialized = True
        self._closed = False

    def reset(self, scene_context: Mapping[str, Any]) -> None:
        if not self._initialized or self._closed:
            raise RuntimeError("external model wrapper is not initialized")
        self._cursor = 0
        self.scene_context = deepcopy(dict(scene_context))

    def preprocess_observation(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        rgb = observation.get("rgb", observation.get("sensors", {}))
        result = {
            "frame_id": observation["frame_id"],
            "timestamp": observation.get("timestamp", observation.get("t_sec")),
            "camera_inputs": {
                camera: deepcopy(rgb[camera]) for camera in self.capability["required_rgb_cameras"]
            },
            "calibration": deepcopy(observation.get("calibration", {})),
            "ego_state": deepcopy(observation["ego_state"]),
            "route_command": observation["route"].get("route_command"),
            "target_point": deepcopy(observation["route"].get("target_point")),
            "route_waypoints": deepcopy(observation["route"].get("route_waypoints")),
            "boundary": "model_repository_owns_tensor_conversion_and_command_encoding",
        }
        if self.capability["requires_lidar"]:
            result["lidar"] = deepcopy(observation["lidar"])
        return result

    def predict_control(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        _ = self.preprocess_observation(observation)
        if not self._test_backend:
            raise RuntimeError(
                "real checkpoint inference is not implemented locally; remote GPU binding required"
            )
        candidate = deepcopy(self._recorded_controls[min(self._cursor, len(self._recorded_controls) - 1)])
        self._cursor += 1
        candidate.update(
            {
                "source_frame_id": int(observation["frame_id"]),
                "inference_ms": 0.0,
                "status": "test_recorded_control",
                "reason": None,
            }
        )
        return candidate

    def health_check(self) -> dict[str, Any]:
        if self._test_backend and self._initialized and not self._closed:
            return {
                "status": "ready",
                "backend": "recorded_test_only",
                "real_checkpoint_loaded": False,
                "remote_gpu_validation_required": True,
            }
        return {
            "status": "remote_validation_required",
            "real_checkpoint_loaded": False,
            "remote_gpu_validation_required": True,
            "problems": deepcopy(self.manifest.get("problems", [])),
        }

    def close(self) -> None:
        self._closed = True


def create_tcp_plugin(config: Mapping[str, Any]) -> ExternalModelPluginWrapper:
    plugin = ExternalModelPluginWrapper("tcp")
    plugin.initialize(config)
    return plugin


def create_transfuser_plugin(config: Mapping[str, Any]) -> ExternalModelPluginWrapper:
    plugin = ExternalModelPluginWrapper("transfuser")
    plugin.initialize(config)
    return plugin


def _spec(algorithm: str) -> dict[str, Any]:
    key = str(algorithm).strip().lower()
    if key not in MODEL_SPECS:
        raise ValueError(f"unsupported external model wrapper: {algorithm}")
    return MODEL_SPECS[key]


def _positive(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be finite and positive")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdefABCDEF" for character in value)
    )
