from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.scene_package import build_scene_package
from adapters.actor_binding import validate_actor_binding_set
from adapters.reconstruction_package import (
    load_reconstruction_package,
    load_reconstruction_result,
    materialize_reconstruction_package,
)
from runners.build_nuscenes_opendrive import write_nuscenes_opendrive
from runners.build_openscenario import write_openscenario
from runners.build_scene_ir_from_nuscenes import write_scene_ir


def build_nuscenes_exchange(
    dataroot: Path,
    version: str,
    scene: str,
    output_dir: Path,
    *,
    radius_m: float = 35.0,
) -> dict[str, Path]:
    """Build the portable P1 exchange bundle for one complete nuScenes scene."""

    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "scene_ir": output_dir / "scene_ir.json",
        "opendrive": output_dir / "road.xodr",
        "openscenario": output_dir / "scenario.xosc",
        "scene_package": output_dir / "scene_package.json",
    }

    write_scene_ir(dataroot, version, scene, paths["scene_ir"])
    return _build_exchange_from_ir(
        dataroot,
        paths,
        radius_m=radius_m,
        reconstruction_package_path=None,
        actor_binding_set_path=None,
    )


def build_exchange_from_scenario_ir(
    dataroot: Path,
    scenario_ir_path: Path,
    output_dir: Path,
    *,
    reconstruction_package_path: Path | None = None,
    reconstruction_result_path: Path | None = None,
    actor_binding_set_path: Path | None = None,
    exchange_root: Path | None = None,
    radius_m: float = 35.0,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "scene_ir": output_dir / "scene_ir.json",
        "opendrive": output_dir / "road.xodr",
        "openscenario": output_dir / "scenario.xosc",
        "scene_package": output_dir / "scene_package.json",
    }
    source_ir = Path(scenario_ir_path).resolve()
    if source_ir != paths["scene_ir"].resolve():
        shutil.copy2(source_ir, paths["scene_ir"])
    return _build_exchange_from_ir(
        dataroot,
        paths,
        radius_m=radius_m,
        reconstruction_package_path=reconstruction_package_path,
        reconstruction_result_path=reconstruction_result_path,
        actor_binding_set_path=actor_binding_set_path,
        exchange_root=exchange_root,
    )


def _build_exchange_from_ir(
    dataroot: Path,
    paths: dict[str, Path],
    *,
    radius_m: float,
    reconstruction_package_path: Path | None,
    reconstruction_result_path: Path | None = None,
    actor_binding_set_path: Path | None = None,
    exchange_root: Path | None = None,
) -> dict[str, Path]:
    write_nuscenes_opendrive(
        dataroot,
        paths["opendrive"],
        scenario_ir_path=paths["scene_ir"],
        radius_m=radius_m,
    )
    write_openscenario(
        paths["scene_ir"],
        paths["openscenario"],
        road_file=paths["opendrive"].name,
    )

    scene_ir = json.loads(paths["scene_ir"].read_text(encoding="utf-8"))
    actor_bindings_name = None
    if actor_binding_set_path is not None:
        binding_source = Path(actor_binding_set_path).resolve()
        actor_bindings = json.loads(binding_source.read_text(encoding="utf-8"))
        validate_actor_binding_set(actor_bindings)
        if actor_bindings["scene_id"] != str(scene_ir["scenario_id"]):
            raise ValueError("Actor Binding Set scene_id does not match Scenario IR")
        binding_target = paths["scene_package"].parent / "actor_bindings.json"
        if binding_source != binding_target.resolve():
            shutil.copy2(binding_source, binding_target)
        paths["actor_bindings"] = binding_target
        actor_bindings_name = binding_target.name
    reconstruction_paths: dict[str, str] = {}
    if reconstruction_package_path is not None and reconstruction_result_path is not None:
        raise ValueError("provide only one reconstruction package or result")
    if reconstruction_result_path is not None:
        if exchange_root is None:
            raise ValueError("exchange_root is required with reconstruction result")
        reconstruction = load_reconstruction_result(
            reconstruction_result_path,
            exchange_root=exchange_root,
            expected_scene_id=str(scene_ir["scenario_id"]),
        )
        reconstruction_paths = materialize_reconstruction_package(
            reconstruction,
            paths["scene_package"].parent,
        )
    elif reconstruction_package_path is not None:
        reconstruction = load_reconstruction_package(
            reconstruction_package_path,
            expected_scene_id=str(scene_ir["scenario_id"]),
        )
        reconstruction_paths = materialize_reconstruction_package(
            reconstruction,
            paths["scene_package"].parent,
        )
    package = build_scene_package(
        scene_ir,
        scene_ir_path=paths["scene_ir"].name,
        openscenario_path=paths["openscenario"].name,
        opendrive_path=paths["opendrive"].name,
        map_source="nuscenes_map_expansion",
        actor_bindings_path=actor_bindings_name,
        nurec_usdz=reconstruction_paths.get("nurec_usdz"),
        nurec_checkpoint=reconstruction_paths.get("nurec_checkpoint"),
        reconstruction_package_path=reconstruction_paths.get("reconstruction_package"),
    )
    paths["scene_package"].write_text(
        json.dumps(package, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return paths


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Build Scene IR, OpenDRIVE, OpenSCENARIO, and a portable Scene Package from nuScenes."
    )
    parser.add_argument("--dataroot", required=True, help="nuScenes root containing maps and version metadata.")
    parser.add_argument("--version", default="v1.0-mini")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--scene", help="nuScenes scene name or token.")
    source.add_argument("--scenario-ir", help="Existing scenario_ir.v1 JSON from TriggerEngine.")
    reconstruction = parser.add_mutually_exclusive_group()
    reconstruction.add_argument("--reconstruction-package")
    reconstruction.add_argument("--reconstruction-result")
    parser.add_argument("--actor-bindings", help="Validated actor_binding_set.v1 JSON to include in the bundle.")
    parser.add_argument("--exchange-root")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--radius-m", type=float, default=35.0)
    args = parser.parse_args(argv)

    if args.scenario_ir:
        paths = build_exchange_from_scenario_ir(
            Path(args.dataroot),
            Path(args.scenario_ir),
            Path(args.output_dir),
            reconstruction_package_path=(
                Path(args.reconstruction_package) if args.reconstruction_package else None
            ),
            reconstruction_result_path=(
                Path(args.reconstruction_result) if args.reconstruction_result else None
            ),
            actor_binding_set_path=(Path(args.actor_bindings) if args.actor_bindings else None),
            exchange_root=Path(args.exchange_root) if args.exchange_root else None,
            radius_m=args.radius_m,
        )
    else:
        if args.reconstruction_package or args.reconstruction_result or args.actor_bindings:
            parser.error("reconstruction or actor binding input requires --scenario-ir")
        paths = build_nuscenes_exchange(
            Path(args.dataroot),
            args.version,
            args.scene,
            Path(args.output_dir),
            radius_m=args.radius_m,
        )
    print(json.dumps({key: str(path) for key, path in paths.items()}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
