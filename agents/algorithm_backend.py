from __future__ import annotations

import importlib
import inspect
import sys
from pathlib import Path
from typing import Any


class AlgorithmBackendError(RuntimeError):
    """Raised when an external algorithm backend cannot be used safely."""


def validate_runtime_paths(
    *,
    repo_path: str | Path,
    checkpoint_path: str | Path,
    shared_data_path: str | Path,
) -> dict[str, str]:
    paths = {
        "repo_path": Path(repo_path).expanduser(),
        "checkpoint_path": Path(checkpoint_path).expanduser(),
        "shared_data_path": Path(shared_data_path).expanduser(),
    }
    problems = []
    if not paths["repo_path"].is_dir():
        problems.append(f"algorithm repository is not a directory: {paths['repo_path']}")
    if not paths["checkpoint_path"].is_file():
        problems.append(f"algorithm checkpoint is not a file: {paths['checkpoint_path']}")
    if not paths["shared_data_path"].is_dir():
        problems.append(f"shared data path is not a directory: {paths['shared_data_path']}")
    if problems:
        raise AlgorithmBackendError("; ".join(problems))
    return {name: str(path.resolve()) for name, path in paths.items()}


def load_backend(plugin: str, config: dict[str, Any]) -> Any:
    """Load ``module:factory`` from the mounted algorithm repository."""
    module_name, separator, factory_name = str(plugin).partition(":")
    if not separator or not module_name or not factory_name:
        raise AlgorithmBackendError("plugin must use the form 'module:factory'")

    repo_path = str(config["repo_path"])
    if repo_path not in sys.path:
        sys.path.insert(0, repo_path)
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:
        raise AlgorithmBackendError(f"cannot import backend module {module_name!r}: {exc}") from exc

    factory = getattr(module, factory_name, None)
    if not callable(factory):
        raise AlgorithmBackendError(f"backend factory is not callable: {plugin}")
    try:
        backend = factory(dict(config))
    except Exception as exc:
        raise AlgorithmBackendError(f"backend factory failed: {exc}") from exc

    if not callable(getattr(backend, "predict_control", None)):
        raise AlgorithmBackendError("backend must provide predict_control(observation)")
    _validate_health(backend)
    return backend


def run_backend(backend: Any) -> None:
    """Enter the plugin-owned transport loop (normally its ROS 2 node)."""
    lifecycle = getattr(backend, "run", None) or getattr(backend, "run_forever", None)
    if not callable(lifecycle):
        raise AlgorithmBackendError(
            "backend must provide blocking run() or run_forever(); no fake inference loop is supplied"
        )
    if inspect.iscoroutinefunction(lifecycle):
        raise AlgorithmBackendError("async backend lifecycle is unsupported; provide a blocking run()")
    lifecycle()


def _validate_health(backend: Any) -> None:
    health_check = getattr(backend, "health_check", None)
    if health_check is None:
        return
    if not callable(health_check):
        raise AlgorithmBackendError("backend health_check attribute is not callable")
    try:
        result = health_check()
    except Exception as exc:
        raise AlgorithmBackendError(f"backend health check failed: {exc}") from exc
    if result is False:
        raise AlgorithmBackendError("backend health check reported unhealthy")
    if isinstance(result, dict) and result.get("status") not in {None, "ready", "healthy", "ok"}:
        raise AlgorithmBackendError(f"backend health check reported unhealthy: {result}")
