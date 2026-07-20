from __future__ import annotations

from copy import deepcopy
from typing import Any, Callable, Mapping

from agents.transfuserpp_contract import capability, validate_intermediate_record
from agents.transfuserpp_runtime import TransFuserPPModelRuntime


class TransFuserPPPlugin:
    """Unified ego-plugin wrapper around a real TF++ model runtime."""

    def __init__(
        self,
        runtime_factory: Callable[[Mapping[str, Any]], Any] | None = None,
    ) -> None:
        self.capability = capability()
        self._runtime_factory = runtime_factory or TransFuserPPModelRuntime
        self._initialized = False
        self._reset = False
        self._closed = False

    def initialize(self, config: Mapping[str, Any]) -> None:
        if self._initialized and not self._closed:
            raise RuntimeError("TransFuser++ plugin is already initialized")
        self.config = deepcopy(
            {key: value for key, value in dict(config).items() if not callable(value)}
        )
        self.runtime = self._runtime_factory(dict(config))
        health = self.runtime.health_check()
        if not isinstance(health, Mapping) or health.get("status") != "ready":
            raise RuntimeError(f"TransFuser++ runtime is not ready: {health}")
        if not health.get("real_checkpoint_loaded") and not config.get("allow_test_runtime"):
            raise RuntimeError("TransFuser++ runtime did not prove a real checkpoint load")
        self._initialized = True
        self._reset = False
        self._closed = False

    def reset(self, scene_context: Mapping[str, Any]) -> None:
        if not self._initialized or self._closed:
            raise RuntimeError("TransFuser++ plugin is not initialized")
        self.scene_context = deepcopy(dict(scene_context))
        self.runtime.reset()
        self._reset = True

    def predict_control(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        if not self._initialized or not self._reset or self._closed:
            raise RuntimeError("TransFuser++ plugin lifecycle is not ready")
        record = self.runtime.predict(observation)
        validate_intermediate_record(record)
        control = deepcopy(record["outputs"]["control"])
        control.update(
            {
                "source_frame_id": int(observation["frame_id"]),
                "inference_ms": float(record["latency_ms"]["inference"]),
                "status": "ok",
                "reason": None,
                "intermediate_record_ref": deepcopy(record.get("record_ref")),
                "target_speed_mps": float(record["outputs"]["target_speed_mps"]),
            }
        )
        return control

    def health_check(self) -> dict[str, Any]:
        if not self._initialized or self._closed:
            return {
                "status": "closed" if self._closed else "not_initialized",
                "real_checkpoint_loaded": False,
            }
        return deepcopy(dict(self.runtime.health_check()))

    def close(self) -> None:
        if self._initialized and not self._closed:
            self.runtime.close()
        self._closed = True
        self._reset = False


def create_plugin(config: Mapping[str, Any]) -> TransFuserPPPlugin:
    plugin = TransFuserPPPlugin()
    plugin.initialize(config)
    return plugin
