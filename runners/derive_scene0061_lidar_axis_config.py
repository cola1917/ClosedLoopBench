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

# Coordinate-fixed M8 payloads demonstrate that the NuRec response is already
# in the calibrated sensor-local basis.  The earlier R18 rotation was falsified
# by all three frames: it yielded zero physical-box support while the raw basis
# yielded consistent support.  A fresh live capture still has to establish the
# physical-coordinate evidence; this contract only makes the chosen basis
# explicit and replayable.
RAW_RESPONSE_TO_SENSOR = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]


class Scene0061LiDARConfigDerivationError(ValueError):
    """Raised when a source config cannot be safely derived."""


def _sha256_bytes(body: bytes) -> str:
    return hashlib.sha256(body).hexdigest()


def lidar_axis_normalization_contract() -> dict[str, Any]:
    """Return the replayable raw-NRE-response to calibrated-sensor contract."""

    return validate_lidar_axis_normalization(
        {
            "schema_version": AXIS_NORMALIZATION_SCHEMA,
            "source_coordinate_frame": "nre_26_04_lidar_sensor",
            "source_axis_convention": "nre_26_04_render_axes",
            "target_coordinate_frame": "sensor_local",
            "target_axis_convention": "carla_sensor",
            "response_to_sensor": list(RAW_RESPONSE_TO_SENSOR),
        }
    )


def derive_lidar_axis_config(
    source_config: Mapping[str, Any],
    *,
    source_path: Path,
    source_sha256: str,
    source_byte_count: int,
    supersede_existing_axis: bool = False,
) -> dict[str, Any]:
    """Return a fresh config bound to the r18-measured axis declaration.

    A source may inherit an unrelated configuration derivation such as an M7
    binding or M8 safety contract. A prior LiDAR-axis derivation or an existing
    axis declaration is still rejected, so one output cannot hide a second axis
    contract behind an arbitrary upstream provenance record.
    """

    if not isinstance(source_config, Mapping):
        raise Scene0061LiDARConfigDerivationError("source run config must be an object")
    if len(source_sha256) != 64 or any(c not in "0123456789abcdef" for c in source_sha256):
        raise Scene0061LiDARConfigDerivationError("source config SHA-256 must be lowercase hex")
    if not isinstance(source_byte_count, int) or isinstance(source_byte_count, bool) or source_byte_count < 1:
        raise Scene0061LiDARConfigDerivationError("source config byte count must be positive")
    inherited_derivation = source_config.get("config_derivation")
    if (
        not supersede_existing_axis
        and isinstance(inherited_derivation, Mapping)
        and inherited_derivation.get("schema_version") == DERIVATION_SCHEMA
    ):
        raise Scene0061LiDARConfigDerivationError(
            "source config already contains a LiDAR-axis derivation record"
        )

    runtime = source_config.get("nurec_runtime")
    if not isinstance(runtime, Mapping):
        raise Scene0061LiDARConfigDerivationError("source run config requires nurec_runtime")
    existing_axis = runtime.get("lidar_axis_normalization")
    if existing_axis is not None and not supersede_existing_axis:
        raise Scene0061LiDARConfigDerivationError(
            "source config already declares lidar_axis_normalization"
        )
    if existing_axis is not None:
        try:
            existing_axis = validate_lidar_axis_normalization(existing_axis)
        except LiDARAxisNormalizationError as exc:
            raise Scene0061LiDARConfigDerivationError(
                f"source LiDAR axis contract is invalid: {exc}"
            ) from exc
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
    derivation = {
        "schema_version": DERIVATION_SCHEMA,
        "kind": (
            "supersede_invalid_nre_26_04_lidar_response_axis"
            if supersede_existing_axis
            else "bind_nre_26_04_lidar_response_to_carla_sensor"
        ),
        "source_config": {
            "path": str(source_path.expanduser().resolve()),
            "sha256": source_sha256,
            "byte_count": source_byte_count,
        },
        "lidar_axis_normalization_sha256": normalization[
            "response_to_sensor_sha256"
        ],
    }
    if inherited_derivation is not None:
        derivation["parent_config_derivation"] = deepcopy(inherited_derivation)
    if existing_axis is not None:
        derivation["superseded_lidar_axis_normalization"] = existing_axis
        derivation["supersession_reason"] = (
            "coordinate_fixed_m8_raw_response_has_support_while_prior_axis_does_not"
        )
    result["config_derivation"] = derivation
    return result


def derive_lidar_axis_config_file(
    source_path: Path, output_path: Path, *, supersede_existing_axis: bool = False
) -> dict[str, Any]:
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
        supersede_existing_axis=supersede_existing_axis,
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
    parser.add_argument(
        "--supersede-existing-axis",
        action="store_true",
        help="Explicitly replace an existing, recorded LiDAR axis contract and retain it as provenance.",
    )
    args = parser.parse_args(argv)
    try:
        result = derive_lidar_axis_config_file(
            args.source_config,
            args.output,
            supersede_existing_axis=args.supersede_existing_axis,
        )
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps({"status": "passed", **result}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
