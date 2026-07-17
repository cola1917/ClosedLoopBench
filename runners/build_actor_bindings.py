from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.actor_binding import assert_actor_binding_ready, build_actor_binding_set
from adapters.shared_protocol_validation import validate_document


def load_nurec_track_inventory(path: str | Path) -> list[str]:
    """Load renderer-probed tracks that accepted a dynamic pose request."""

    source = Path(path)
    value = json.loads(source.read_text(encoding="utf-8"))
    try:
        validate_document(value)
    except ValueError as exc:
        raise ValueError(f"invalid canonical NuRec track inventory: {exc}") from exc
    if value.get("schema_version") != "nurec_runtime_track_inventory.v1":
        raise ValueError("NuRec track inventory must use nurec_runtime_track_inventory.v1")
    if isinstance(value, dict) and isinstance(value.get("tracks"), list):
        records = value["tracks"]
    else:
        raise ValueError("NuRec track inventory must contain a tracks array with probe evidence")
    result = []
    for record in records:
        if not isinstance(record, dict):
            raise ValueError("NuRec inventory entries must contain dynamic pose probe evidence")
        track_id = record.get("track_id")
        if not isinstance(track_id, str) or not track_id:
            raise ValueError("every NuRec inventory entry requires a non-empty track_id")
        if record.get("dynamic_object_pose_verified") is True:
            result.append(track_id)
    if len(result) != len(set(result)):
        raise ValueError("NuRec track inventory contains duplicate track IDs")
    return result


def write_actor_bindings(
    scenario_ir_path: str | Path,
    output_path: str | Path,
    *,
    actor_ids: list[str] | None = None,
    nurec_inventory_path: str | Path | None = None,
    control_modes: dict[str, str] | None = None,
    require_ready: bool = False,
) -> dict[str, Any]:
    scenario_ir = json.loads(Path(scenario_ir_path).read_text(encoding="utf-8"))
    nurec_track_ids = (
        load_nurec_track_inventory(nurec_inventory_path)
        if nurec_inventory_path is not None
        else None
    )
    result = build_actor_binding_set(
        scenario_ir,
        selected_actor_ids=actor_ids,
        nurec_track_ids=nurec_track_ids,
        control_modes=control_modes,
    )
    if require_ready:
        assert_actor_binding_ready(result)
    target = Path(output_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def _key_value(values: list[str], option: str) -> dict[str, str]:
    result = {}
    for value in values:
        key, separator, item = value.partition("=")
        if not separator or not key or not item:
            raise ValueError(f"{option} values must use ACTOR_ID=VALUE")
        if key in result:
            raise ValueError(f"{option} contains duplicate actor ID: {key}")
        result[key] = item
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build the nuScenes/Scenario IR/CARLA/NuRec actor identity contract."
    )
    parser.add_argument("--scenario-ir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--actor-id",
        action="append",
        default=[],
        help="Actor to bind; repeat for multiple actors. Defaults to every Scenario IR actor.",
    )
    parser.add_argument(
        "--nurec-track-inventory",
        help="JSON emitted from the built NuRec/NCore asset; omission produces a blocked planning artifact.",
    )
    parser.add_argument(
        "--control-mode",
        action="append",
        default=[],
        metavar="ACTOR_ID=MODE",
        help="Override replay/scripted/traffic_manager for a selected actor.",
    )
    parser.add_argument(
        "--require-ready",
        action="store_true",
        help="Fail instead of writing a claimable artifact unless every selected track is verified.",
    )
    args = parser.parse_args(argv)
    try:
        control_modes = _key_value(args.control_mode, "--control-mode")
        result = write_actor_bindings(
            args.scenario_ir,
            args.output,
            actor_ids=args.actor_id or None,
            nurec_inventory_path=args.nurec_track_inventory,
            control_modes=control_modes,
            require_ready=args.require_ready,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
