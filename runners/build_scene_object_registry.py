from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_object_registry import (
    SceneObjectRegistryError,
    assert_scene_object_coverage_ready,
    audit_scene_object_coverage,
    build_scene_object_registry,
)


def _load(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _static_objects(value: Any) -> list[dict[str, Any]]:
    if isinstance(value, list):
        return value
    if isinstance(value, dict) and isinstance(value.get("objects"), list):
        return value["objects"]
    raise SceneObjectRegistryError(
        "static object manifest must be an array or contain an objects array"
    )


def _roles(values: list[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for value in values:
        actor_id, separator, role = value.partition("=")
        if not separator or not actor_id or not role or actor_id in result:
            raise SceneObjectRegistryError("--role must use unique ACTOR_ID=ROLE values")
        result[actor_id] = role
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build the M6 safety-relevant scene object registry and coverage audit."
    )
    parser.add_argument("--scenario-ir", required=True, type=Path)
    parser.add_argument("--static-object-manifest", type=Path)
    parser.add_argument("--visibility-manifest", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--audit-output", required=True, type=Path)
    parser.add_argument("--role", action="append", default=[], metavar="ACTOR_ID=ROLE")
    parser.add_argument("--require-coverage-ready", action="store_true")
    args = parser.parse_args(argv)
    try:
        registry = build_scene_object_registry(
            _load(args.scenario_ir),
            static_objects=(
                _static_objects(_load(args.static_object_manifest))
                if args.static_object_manifest is not None
                else []
            ),
            role_overrides=_roles(args.role),
        )
        visibility = _load(args.visibility_manifest) if args.visibility_manifest else None
        audit = audit_scene_object_coverage(registry, visibility)
        if args.require_coverage_ready:
            assert_scene_object_coverage_ready(audit)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.audit_output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(registry, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        args.audit_output.write_text(
            json.dumps(audit, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(
        json.dumps(
            {"registry": str(args.output), "audit": str(args.audit_output), "status": audit["status"]},
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
