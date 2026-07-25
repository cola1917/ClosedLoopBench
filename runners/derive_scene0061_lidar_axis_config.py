"""Derive an immutable Scene-0061 config with bound NRE LiDAR axes.

This tool exists for the first fresh execution after r18.  It preserves every
source-config field byte-for-byte in meaning, then adds the separately
measured NRE 26.04 response-to-CARLA-sensor rotation and a source identity.
It deliberately refuses to mutate an already-derived config or overwrite an
output, so no diagnostic can silently inherit an unrecorded axis convention.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping


# The runbook invokes this tool by absolute path from an arbitrary remote
# shell.  In that form Python initially exposes only runners/ on sys.path.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agents.plugin_contract import strict_json_loads
from runtime.scene0061_lidar_axis_normalization import (
    AXIS_NORMALIZATION_SCHEMA,
    validate_lidar_axis_normalization,
)


DERIVATION_SCHEMA = "scene0061_lidar_axis_config_derivation.v1"

# Candidate established by the read-only r18 analysis.  It is not accepted as
# a coordinate proof by itself: the subsequent fresh live run must replay it
# against the raw and normalised payloads plus same-frame CARLA anchors.
R18_RESPONSE_TO_SENSOR = [
    0.0, 0.0, -1.0, 0.0,
    0.0, -1.0, 0.0, 0.0,
    -1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]


class Scene0061LiDARConfigDerivationError(ValueError):
    """Raised when a source config cannot be safely derived."""


def _sha256_bytes(body: bytes) -> str:
    return hashlib.sha256(body).hexdigest()


def lidar_axis_normalization_contract() -> dict[str, Any]:
    """Return the fully validated NRE-response to CARLA-sensor contract."""

    return validate_lidar_axis_normalization(
        {
            "schema_version": AXIS_NORMALIZATION_SCHEMA,
            "source_coordinate_frame": "nre_26_04_lidar_sensor",
            "source_axis_convention": "nre_26_04_render_axes",
            "target_coordinate_frame": "sensor_local",
            "target_axis_convention": "carla_sensor",
            "response_to_sensor": list(R18_RESPONSE_TO_SENSOR),
        }
    )


def derive_lidar_axis_config(
    source_config: Mapping[str, Any],
    *,
    source_path: Path,
    source_sha256: str,
    source_byte_count: int,
) -> dict[str, Any]:
    """Return a fresh config bound to the r18-measured axis declaration.

    A source must be an original runtime config: if it already contains either
    this derivation record or an axis declaration, creating another output
    would obscure which declaration is authoritative.
    """

    if not isinstance(source_config, Mapping):
        raise Scene0061LiDARConfigDerivationError("source run config must be an object")
    if len(source_sha256) != 64 or any(c not in "0123456789abcdef" for c in source_sha256):
        raise Scene0061LiDARConfigDerivationError("source config SHA-256 must be lowercase hex")
    if not isinstance(source_byte_count, int) or isinstance(source_byte_count, bool) or source_byte_count < 1:
        raise Scene0061LiDARConfigDerivationError("source config byte count must be positive")
    if source_config.get("config_derivation") is not None:
        raise Scene0061LiDARConfigDerivationError(
            "source config already contains a derivation record"
        )

    runtime = source_config.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        raise Scene0061LiDARConfigDerivationError("source run config requires nurec_runtime")
    if runtime.get("lidar_axis_normalization") is not None:
        raise Scene0061LiDARConfigDerivationError(
            "source config already declares lidar_axis_normalization"
        )
    lidar_specs = runtime.get("lidar_specs")
    matching_specs = [
        item
        for item in lidar_specs or []
        if isinstance(item, Mapping) and item.get("sensor_id") == "lidar_top"
    ]
    if len(matching_specs) != 1:
        raise Scene0061LiDARConfigDerivationError(
            "source run config requires exactly one lidar_top spec"
        )

    result = deepcopy(dict(source_config))
    result_runtime = dict(runtime)
    normalization = lidar_axis_normalization_contract()
    result_runtime["lidar_axis_normalization"] = normalization
    result["nurec_runtime"] = result_runtime
    result["config_derivation"] = {
        "schema_version": DERIVATION_SCHEMA,
        "kind": "bind_nre_26_04_lidar_response_to_carla_sensor",
        "source_config": {
            "path": str(source_path.expanduser().resolve()),
            "sha256": source_sha256,
            "byte_count": source_byte_count,
        },
        "lidar_axis_normalization_sha256": normalization[
            "response_to_sensor_sha256"
        ],
    }
    return result


def derive_lidar_axis_config_file(source_path: Path, output_path: Path) -> dict[str, Any]:
    """Read a strict source document and write one new derived document."""

    source = source_path.expanduser().resolve()
    output = output_path.expanduser().resolve()
    if not source.is_file():
        raise Scene0061LiDARConfigDerivationError(
            f"source config does not exist: {source}"
        )
    if output.exists():
        raise FileExistsError(f"refusing to overwrite derived config: {output}")
    try:
        source_bytes = source.read_bytes()
        source_config = strict_json_loads(source_bytes.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise Scene0061LiDARConfigDerivationError(
            f"cannot read strict source config {source}: {exc}"
        ) from exc
    result = derive_lidar_axis_config(
        source_config,
        source_path=source,
        source_sha256=_sha256_bytes(source_bytes),
        source_byte_count=len(source_bytes),
    )
    encoded = (json.dumps(result, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    output.parent.mkdir(parents=True, exist_ok=True)
    # ``x`` closes the check/write race as well as protecting historical run
    # inputs from an accidental second invocation.
    with output.open("xb") as stream:
        stream.write(encoded)
    return {
        "output": str(output),
        "output_sha256": _sha256_bytes(encoded),
        "source_config": result["config_derivation"]["source_config"],
        "lidar_axis_normalization": result["nurec_runtime"][
            "lidar_axis_normalization"
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Derive an immutable Scene-0061 config with the measured NRE LiDAR axis contract."
        )
    )
    parser.add_argument("--source-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = derive_lidar_axis_config_file(args.source_config, args.output)
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps({"status": "passed", **result}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
