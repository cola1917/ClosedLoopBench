from __future__ import annotations

import hashlib
import json
import math
import time
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


EVIDENCE_CLASSIFICATIONS = (
    "offline_conformance",
    "control_only",
    "perception_eligible",
    "quality_stress",
    "remote_validation_required",
)

SAFE_STOP_CONTROL = {
    "throttle": 0.0,
    "steer": 0.0,
    "brake": 1.0,
    "hand_brake": False,
    "reverse": False,
}

REQUIRED_PLUGIN_METHODS = (
    "initialize",
    "reset",
    "predict_control",
    "health_check",
    "close",
)


class PluginContractError(RuntimeError):
    """Raised when an ego algorithm violates the fail-closed plugin contract."""


def strict_json_loads(text: str) -> Any:
    """Parse JSON while rejecting duplicate object keys at every nesting level.

    Python's default decoder silently keeps the last duplicate key.  That is
    unsafe for immutable runtime and evidence documents because the human- and
    machine-visible values can otherwise disagree.
    """

    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        value: dict[str, Any] = {}
        for key, item in pairs:
            if key in value:
                raise ValueError(f"duplicate JSON object key: {key}")
            value[key] = item
        return value

    return json.loads(text, object_pairs_hook=reject_duplicates)


def strict_json_loads(text: str) -> Any:
    """Parse JSON while rejecting duplicate object keys at every nesting level.

    Python's default decoder silently keeps the last duplicate key.  That is
    unsafe for immutable runtime and evidence documents because the human- and
    machine-visible values can otherwise disagree.
    """

    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        value: dict[str, Any] = {}
        for key, item in pairs:
            if key in value:
                raise ValueError(f"duplicate JSON object key: {key}")
            value[key] = item
        return value

    return json.loads(text, object_pairs_hook=reject_duplicates)


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_plugin_identity(
    config: Mapping[str, Any], capability: Mapping[str, Any]
) -> dict[str, Any]:
    repo_path = Path(str(config.get("repo_path", "."))).expanduser()
    checkpoint_path = config.get("checkpoint_path")
    checkpoint_hash = config.get("checkpoint_sha256")
    actual_checkpoint_hash = (
        file_sha256(str(checkpoint_path))
        if checkpoint_path and Path(str(checkpoint_path)).is_file()
        else None
    )
    if checkpoint_hash is None and actual_checkpoint_hash:
        checkpoint_hash = actual_checkpoint_hash
    if capability.get("checkpoint_identity") == "not_applicable":
        checkpoint_hash = "not_applicable"
    else:
        if not _is_sha256(config.get("repo_sha256")):
            raise PluginContractError("external plugin requires a 64-hex repo_sha256")
        if not _is_sha256(checkpoint_hash):
            raise PluginContractError("external plugin requires a 64-hex checkpoint_sha256")
        if actual_checkpoint_hash and checkpoint_hash != actual_checkpoint_hash:
            raise PluginContractError("external plugin checkpoint_sha256 does not match the file")

    repo_revision = str(config.get("repo_revision") or _read_git_revision(repo_path) or "unknown")
    # A repository identity must survive moving the same checkout between the
    # local and remote hosts.  The resolved path is operational metadata, not
    # source identity; use the immutable revision when no externally supplied
    # content hash is available.
    repo_identity = repo_revision if repo_revision != "unknown" else str(repo_path.resolve())
    repo_hash = str(
        config.get("repo_sha256")
        or hashlib.sha256(repo_identity.encode("utf-8")).hexdigest()
    )
    identity_config = {
        key: value
        for key, value in dict(config).items()
        if key not in {"checkpoint_sha256", "repo_sha256", "config_sha256"}
    }
    return {
        "algorithm_id": capability["algorithm_id"],
        "plugin_schema": "ego_algorithm_plugin.v1",
        "config_sha256": canonical_sha256(identity_config),
        "repo_revision": repo_revision,
        "repo_sha256": repo_hash,
        "checkpoint_sha256": checkpoint_hash or "unavailable",
        "real_checkpoint_loaded": bool(config.get("real_checkpoint_loaded", False)),
    }


def validate_capability(capability: Any) -> dict[str, Any]:
    if not isinstance(capability, Mapping):
        raise PluginContractError("plugin capability must be an object")
    required = {
        "algorithm_id",
        "uses_route",
        "uses_ego_state",
        "required_rgb_cameras",
        "requires_lidar",
        "is_perception_algorithm",
        "requires_gpu",
        "checkpoint_identity",
        "supported_control_hz",
        "timeout_sec",
    }
    missing = sorted(required - set(capability))
    if missing:
        raise PluginContractError(f"plugin capability missing fields: {missing}")
    result = deepcopy(dict(capability))
    if not isinstance(result["algorithm_id"], str) or not result["algorithm_id"]:
        raise PluginContractError("capability algorithm_id must be non-empty")
    cameras = result["required_rgb_cameras"]
    if not isinstance(cameras, list) or any(not isinstance(item, str) or not item for item in cameras):
        raise PluginContractError("required_rgb_cameras must be a list of names")
    for field in (
        "uses_route",
        "uses_ego_state",
        "requires_lidar",
        "is_perception_algorithm",
        "requires_gpu",
    ):
        if not isinstance(result[field], bool):
            raise PluginContractError(f"capability {field} must be boolean")
    if not _finite_positive(result["supported_control_hz"]):
        raise PluginContractError("supported_control_hz must be finite and positive")
    if not _finite_positive(result["timeout_sec"]):
        raise PluginContractError("timeout_sec must be finite and positive")
    return result


def validate_plugin_lifecycle(plugin: Any) -> None:
    missing = [name for name in REQUIRED_PLUGIN_METHODS if not callable(getattr(plugin, name, None))]
    if missing:
        raise PluginContractError(f"plugin missing lifecycle methods: {missing}")
    validate_capability(getattr(plugin, "capability", None))


def validate_observation(
    observation: Any,
    capability: Mapping[str, Any],
    *,
    last_frame_id: int | None = None,
) -> None:
    if not isinstance(observation, Mapping):
        raise PluginContractError("observation must be an object")
    frame_id = observation.get("frame_id")
    if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id < 0:
        raise PluginContractError("observation frame_id must be a non-negative integer")
    if last_frame_id is not None and frame_id <= last_frame_id:
        raise PluginContractError(
            f"stale_frame: source frame {frame_id} is not newer than {last_frame_id}"
        )
    timestamp = observation.get("timestamp", observation.get("t_sec"))
    if not _finite_nonnegative(timestamp):
        raise PluginContractError("observation timestamp must be finite and non-negative")

    validity = observation.get("sensor_validity", {})
    if validity is not None and not isinstance(validity, Mapping):
        raise PluginContractError("sensor_validity must be an object")
    rgb = observation.get("rgb", observation.get("sensors", {}))
    if not isinstance(rgb, Mapping):
        raise PluginContractError("observation rgb/sensors must be an object")
    missing_cameras = [
        camera
        for camera in capability["required_rgb_cameras"]
        if camera not in rgb or rgb[camera] is None or validity.get(camera) is False
    ]
    if missing_cameras:
        raise PluginContractError(f"missing_sensors: RGB cameras {missing_cameras}")
    if capability["requires_lidar"]:
        if observation.get("lidar") is None or validity.get("lidar") is False:
            raise PluginContractError("missing_sensors: lidar")
    if capability["uses_ego_state"] and not isinstance(observation.get("ego_state"), Mapping):
        raise PluginContractError("missing_ego_state")
    if capability["uses_route"] and not isinstance(observation.get("route"), Mapping):
        raise PluginContractError("missing_route")
    synchronization = observation.get("synchronization")
    if synchronization is not None:
        if not isinstance(synchronization, Mapping):
            raise PluginContractError("synchronization must be an object")
        sync_frame = synchronization.get("frame_id", frame_id)
        if sync_frame != frame_id:
            raise PluginContractError(
                f"frame_mismatch: observation={frame_id} synchronization={sync_frame}"
            )


def validate_control(control: Any, *, expected_frame_id: int) -> dict[str, Any]:
    if not isinstance(control, Mapping):
        raise PluginContractError("control must be an object")
    required = (
        "throttle",
        "steer",
        "brake",
        "hand_brake",
        "reverse",
        "source_frame_id",
    )
    missing = [field for field in required if field not in control]
    if missing:
        raise PluginContractError(f"control missing fields: {missing}")
    for field, lower, upper in (
        ("throttle", 0.0, 1.0),
        ("steer", -1.0, 1.0),
        ("brake", 0.0, 1.0),
    ):
        value = control[field]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise PluginContractError(f"invalid_control: {field} must be finite")
        if not lower <= float(value) <= upper:
            raise PluginContractError(f"invalid_control: {field} outside [{lower}, {upper}]")
    if not isinstance(control["hand_brake"], bool) or not isinstance(control["reverse"], bool):
        raise PluginContractError("invalid_control: hand_brake/reverse must be boolean")
    if control["source_frame_id"] != expected_frame_id:
        raise PluginContractError(
            f"frame_mismatch: control={control['source_frame_id']} observation={expected_frame_id}"
        )
    result = deepcopy(dict(control))
    result.update(
        {
            "throttle": float(control["throttle"]),
            "steer": float(control["steer"]),
            "brake": float(control["brake"]),
        }
    )
    return result


class AlgorithmPluginExecutor:
    """Fail-closed lifecycle and per-frame guard around an ego algorithm plugin."""

    def __init__(
        self,
        plugin: Any,
        config: Mapping[str, Any],
        *,
        already_initialized: bool = False,
        evidence_classification: str = "remote_validation_required",
    ):
        validate_plugin_lifecycle(plugin)
        if evidence_classification not in EVIDENCE_CLASSIFICATIONS:
            raise PluginContractError(
                f"unknown evidence_classification: {evidence_classification}"
            )
        self.plugin = plugin
        self.config = deepcopy(dict(config))
        self.capability = validate_capability(plugin.capability)
        self.identity = build_plugin_identity(self.config, self.capability)
        self.evidence_classification = evidence_classification
        self.initialized = bool(already_initialized)
        self.reset_complete = False
        self.closed = False
        self.last_frame_id: int | None = None

    def initialize(self) -> dict[str, Any]:
        if self.closed:
            raise PluginContractError("plugin is already closed")
        if not self.initialized:
            self.plugin.initialize(deepcopy(self.config))
            self.initialized = True
        self._require_healthy()
        return deepcopy(self.identity)

    def reset(self, scene_context: Mapping[str, Any]) -> None:
        if not self.initialized or self.closed:
            raise PluginContractError("plugin must be initialized and open before reset")
        self.plugin.reset(deepcopy(dict(scene_context)))
        self.last_frame_id = None
        self.reset_complete = True

    def predict(
        self, observation: Mapping[str, Any], *, forced_failure: str | None = None
    ) -> dict[str, Any]:
        frame_id = observation.get("frame_id") if isinstance(observation, Mapping) else None
        if not self.initialized or not self.reset_complete or self.closed:
            return self._safe_stop(frame_id, "lifecycle_not_ready")
        try:
            self._require_healthy()
            validate_observation(observation, self.capability, last_frame_id=self.last_frame_id)
        except PluginContractError as exc:
            return self._safe_stop(frame_id, _reason_code(exc))

        if forced_failure in {"backend_exception", "timeout"}:
            return self._safe_stop(frame_id, forced_failure, injected=True)

        started = time.perf_counter()
        try:
            candidate = self.plugin.predict_control(deepcopy(dict(observation)))
        except Exception as exc:
            return self._safe_stop(frame_id, "backend_exception", detail=str(exc))
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if elapsed_ms > float(self.capability["timeout_sec"]) * 1000.0:
            return self._safe_stop(
                frame_id,
                "timeout",
                inference_ms=elapsed_ms,
                timeout_ms=float(self.capability["timeout_sec"]) * 1000.0,
            )
        try:
            control = validate_control(candidate, expected_frame_id=int(frame_id))
        except PluginContractError as exc:
            return self._safe_stop(frame_id, _reason_code(exc), detail=str(exc))
        control["inference_ms"] = elapsed_ms
        control.setdefault("status", "ok")
        control.setdefault("reason", None)
        self.last_frame_id = int(frame_id)
        return {
            "execution_status": "control",
            "evidence_classification": self.evidence_classification,
            "control": control,
            "plugin_identity": deepcopy(self.identity),
        }

    def close(self) -> None:
        if not self.closed:
            self.plugin.close()
            self.closed = True
            self.reset_complete = False

    def _require_healthy(self) -> None:
        try:
            health = self.plugin.health_check()
        except Exception as exc:
            raise PluginContractError(f"health_check_failure: {exc}") from exc
        if health is False:
            raise PluginContractError("health_check_failure")
        if isinstance(health, Mapping) and health.get("status") not in {
            None,
            "ready",
            "healthy",
            "ok",
        }:
            raise PluginContractError(f"health_check_failure: {dict(health)}")

    def _safe_stop(self, frame_id: Any, reason: str, **detail: Any) -> dict[str, Any]:
        control = deepcopy(SAFE_STOP_CONTROL)
        control.update(
            {
                "source_frame_id": frame_id,
                "inference_ms": float(detail.pop("inference_ms", 0.0)),
                "status": "safe_stop",
                "reason": reason,
            }
        )
        return {
            "execution_status": "fallback",
            "evidence_classification": self.evidence_classification,
            "control": control,
            "detail": detail,
            "plugin_identity": deepcopy(self.identity),
        }


def _reason_code(error: Exception) -> str:
    message = str(error)
    known = (
        "stale_frame",
        "frame_mismatch",
        "missing_sensors",
        "missing_ego_state",
        "missing_route",
        "invalid_control",
        "health_check_failure",
    )
    return next((reason for reason in known if message.startswith(reason)), "contract_violation")


def _read_git_revision(repo_path: Path) -> str | None:
    git_path = repo_path / ".git"
    if not git_path.is_dir():
        return None
    try:
        head = (git_path / "HEAD").read_text(encoding="utf-8").strip()
        if head.startswith("ref: "):
            ref = git_path / head[5:]
            return ref.read_text(encoding="utf-8").strip() if ref.is_file() else None
        return head
    except OSError:
        return None


def _finite_positive(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
        and float(value) > 0.0
    )


def _finite_nonnegative(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
        and float(value) >= 0.0
    )


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdefABCDEF" for character in value)
    )
