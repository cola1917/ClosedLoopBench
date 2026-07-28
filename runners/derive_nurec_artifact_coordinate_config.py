from __future__ import annotations

import argparse
import hashlib
import json
import math
import zipfile
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


def derive_artifact_coordinate_config(
    source: Mapping[str, Any], *, artifact_path: Path
) -> dict[str, Any]:
    """Bind the NRE wire-coordinate transform to the exact USDZ artifact."""

    runtime = source.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        raise ValueError("run config requires nurec_runtime")
    if runtime.get("nre_from_log_transform") is not None:
        raise ValueError("source config already has nre_from_log_transform")
    if not artifact_path.is_file():
        raise FileNotFoundError(f"artifact does not exist: {artifact_path}")
    with zipfile.ZipFile(artifact_path) as archive:
        rig_bytes = archive.read("rig_trajectories.json")
        rig = json.loads(rig_bytes)
    if not isinstance(rig, Mapping):
        raise ValueError("artifact rig_trajectories.json must be an object")
    transform = _invert_rigid(rig.get("T_world_base"))
    derived = deepcopy(dict(source))
    derived_runtime = dict(derived["nurec_runtime"])
    derived_runtime["nre_from_log_transform"] = [
        value for row in transform for value in row
    ]
    derived_runtime["nre_from_log_transform_identity"] = {
        "schema_version": "nurec_artifact_coordinate_transform.v1",
        "direction": "nre_render_from_log_world",
        "source_member": "rig_trajectories.json:T_world_base_inverse",
        "artifact_path": str(artifact_path.resolve()),
        "artifact_sha256": _sha256_file(artifact_path),
        "rig_member_sha256": hashlib.sha256(rig_bytes).hexdigest(),
    }
    derived["nurec_runtime"] = derived_runtime
    return derived


def _invert_rigid(value: Any) -> list[list[float]]:
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("artifact T_world_base must be a 4x4 matrix")
    try:
        matrix = [[float(item) for item in row] for row in value]
    except (TypeError, ValueError) as exc:
        raise ValueError("artifact T_world_base must be numeric") from exc
    if any(len(row) != 4 for row in matrix) or not all(
        math.isfinite(item) for row in matrix for item in row
    ):
        raise ValueError("artifact T_world_base must be finite 4x4")
    if any(abs(matrix[3][index] - expected) > 1e-6 for index, expected in enumerate((0.0, 0.0, 0.0, 1.0))):
        raise ValueError("artifact T_world_base must be rigid homogeneous")
    rotation = [[matrix[row][column] for column in range(3)] for row in range(3)]
    for row in rotation:
        if abs(sum(item * item for item in row) - 1.0) > 1e-4:
            raise ValueError("artifact T_world_base rotation is not orthonormal")
    if any(
        abs(sum(rotation[first][index] * rotation[second][index] for index in range(3))) > 1e-4
        for first in range(3)
        for second in range(first + 1, 3)
    ):
        raise ValueError("artifact T_world_base rotation is not orthonormal")
    transpose = [[rotation[column][row] for column in range(3)] for row in range(3)]
    translation = [matrix[row][3] for row in range(3)]
    inverse_translation = [
        -sum(transpose[row][column] * translation[column] for column in range(3))
        for row in range(3)
    ]
    return [
        [*transpose[0], inverse_translation[0]],
        [*transpose[1], inverse_translation[1]],
        [*transpose[2], inverse_translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Derive an immutable NuRec config with the USDZ wire-coordinate transform."
    )
    parser.add_argument("--run-config", required=True, type=Path)
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error(f"refusing to overwrite config: {args.output}")
    try:
        source = json.loads(args.run_config.read_text(encoding="utf-8"))
        if not isinstance(source, Mapping):
            raise ValueError("run config must be an object")
        derived = derive_artifact_coordinate_config(source, artifact_path=args.artifact)
    except (OSError, ValueError, json.JSONDecodeError, zipfile.BadZipFile, KeyError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(derived, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "passed", "output": str(args.output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
