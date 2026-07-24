from __future__ import annotations

import argparse
import hashlib
import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

from agents.plugin_contract import strict_json_loads


def bind_manifest(
    run_config: Mapping[str, Any],
    manifest: Mapping[str, Any],
    *,
    manifest_path: Path,
    manifest_sha256: str,
    artifact_sha256: str,
    max_midpoint_error_us: int,
) -> dict[str, Any]:
    if manifest.get("schema_version") != "nurec_native_lidar_scan_manifest.v1":
        raise ValueError("unsupported native LiDAR scan manifest")
    if manifest.get("artifact_sha256") != artifact_sha256:
        raise ValueError("native scan manifest artifact SHA-256 mismatch")
    runtime = run_config.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        raise ValueError("run config requires nurec_runtime")
    if manifest.get("runtime_scene_id") != runtime.get("runtime_scene_id"):
        raise ValueError("native scan manifest runtime_scene_id mismatch")
    if int(manifest.get("scene_start_us", -1)) != int(
        runtime.get("scene_start_us", -2)
    ):
        raise ValueError("native scan manifest scene_start_us mismatch")
    lidar_ids = {
        str(item.get("sensor_id") or "")
        for item in runtime.get("lidar_specs") or []
        if isinstance(item, Mapping)
    }
    if manifest.get("sensor_id") not in lidar_ids:
        raise ValueError("native scan manifest sensor_id is not configured")
    if not _is_sha256(manifest_sha256) or not _is_sha256(artifact_sha256):
        raise ValueError("native scan manifest identities must be lowercase SHA-256")
    if int(max_midpoint_error_us) < 0:
        raise ValueError("max_midpoint_error_us must be non-negative")
    if runtime.get("native_scan_manifest") is not None:
        raise ValueError("run config already binds a native scan manifest")

    result = deepcopy(dict(run_config))
    result["nurec_runtime"]["native_scan_manifest"] = {
        "path": str(Path(manifest_path).resolve()),
        "sha256": manifest_sha256,
        "max_midpoint_error_us": int(max_midpoint_error_us),
    }
    experiment = dict(result.get("experiment") or {})
    identity = dict(experiment.get("identity") or {})
    existing_artifact = identity.get("artifact_sha256")
    if existing_artifact is not None and existing_artifact != artifact_sha256:
        raise ValueError("run config artifact identity conflicts with manifest")
    identity.update(
        artifact_sha256=artifact_sha256,
        native_scan_manifest_sha256=manifest_sha256,
    )
    experiment["identity"] = identity
    result["experiment"] = experiment
    return result


def _is_sha256(value: str) -> bool:
    return len(value) == 64 and all(character in "0123456789abcdef" for character in value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Bind a hashed NuRec native scan manifest into a fresh run config."
    )
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--artifact-sha256", required=True)
    parser.add_argument("--max-midpoint-error-us", default=30_000, type=int)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise FileExistsError(f"refusing to overwrite run config: {args.output}")
        run_config = strict_json_loads(args.run_config.read_text(encoding="utf-8"))
        manifest = strict_json_loads(args.manifest.read_text(encoding="utf-8"))
        manifest_sha256 = _sha256(args.manifest)
        result = bind_manifest(
            run_config,
            manifest,
            manifest_path=args.manifest,
            manifest_sha256=manifest_sha256,
            artifact_sha256=args.artifact_sha256,
            max_midpoint_error_us=args.max_midpoint_error_us,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    print(
        json.dumps(
            {
                "status": "passed",
                "output": str(args.output.resolve()),
                "manifest_sha256": manifest_sha256,
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
