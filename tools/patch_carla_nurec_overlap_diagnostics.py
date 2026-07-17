#!/usr/bin/env python3
"""Add per-tick ego/actor bounding-box overlap logging to NuRec replay."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path


BASE_IMPORTS = '''import argparse
import os
import imageio
'''
JSON_IMPORTS = '''import argparse
import json
import os
from pathlib import Path
import imageio
'''

BASE_CLI = '''    argparser.add_argument(
        "--camera-config",
        default="carla_example_camera_config.yaml",
        help="YAML camera grid config (default: bundled CARLA example)",
    )
    args = argparser.parse_args()
'''
COLLISION_CLI = '''    argparser.add_argument(
        "--camera-config",
        default="carla_example_camera_config.yaml",
        help="YAML camera grid config (default: bundled CARLA example)",
    )
    argparser.add_argument(
        "--collision-log",
        default="",
        help="Optional JSON output for ego collision diagnostics",
    )
    args = argparser.parse_args()
'''
OVERLAP_CLI = '''    argparser.add_argument(
        "--camera-config",
        default="carla_example_camera_config.yaml",
        help="YAML camera grid config (default: bundled CARLA example)",
    )
    argparser.add_argument(
        "--overlap-log",
        default="",
        help="Optional JSON output for ego/actor bounding-box overlap diagnostics",
    )
    args = argparser.parse_args()
'''

BASE_STATE = '''        spectator: Optional[carla.Actor] = None
        display: Optional[PygameDisplay] = None
        try:
'''
COLLISION_STATE = '''        spectator: Optional[carla.Actor] = None
        display: Optional[PygameDisplay] = None
        collision_sensor: Optional[carla.Actor] = None
        collision_events = []
        try:
'''
OVERLAP_STATE = '''        spectator: Optional[carla.Actor] = None
        display: Optional[PygameDisplay] = None
        overlap_samples = []
        try:
'''

CAMERA_CALL = '''            spectator, display = add_cameras(
                scenario,
                client,
                args.output_dir,
                args.saveimages,
                resolution_ratio=args.resolution_ratio,
                camera_config_path=args.camera_config,
            )
'''
COLLISION_SENSOR_SUFFIX = '''
            if args.collision_log:
                world = client.get_world()
                ego = scenario.actor_mapping[EGO_TRACK_ID].actor_inst
                collision_bp = world.get_blueprint_library().find("sensor.other.collision")
                collision_sensor = world.spawn_actor(
                    collision_bp, carla.Transform(), attach_to=ego
                )

                def record_collision(event):
                    impulse = event.normal_impulse
                    location = ego.get_location()
                    row = {
                        "frame": event.frame,
                        "other_actor_id": event.other_actor.id,
                        "other_actor_type": event.other_actor.type_id,
                        "normal_impulse": {
                            "x": impulse.x,
                            "y": impulse.y,
                            "z": impulse.z,
                        },
                        "ego_location": {
                            "x": location.x,
                            "y": location.y,
                            "z": location.z,
                        },
                        "scenario_time_seconds": scenario.seconds_since_start(),
                    }
                    collision_events.append(row)
                    logger.warning(
                        "Ego collision: frame=%s other=%s impulse=(%.3f, %.3f, %.3f)",
                        event.frame,
                        event.other_actor.type_id,
                        impulse.x,
                        impulse.y,
                        impulse.z,
                    )

                collision_sensor.listen(record_collision)
'''
COLLISION_CAMERA_BLOCK = CAMERA_CALL + COLLISION_SENSOR_SUFFIX

BASE_TICK = '''                scenario.tick()
                if should_apply_control and scenario.seconds_since_start() > 1:
'''
EGO_OVERLAP_TICK = '''                scenario.tick()
                if args.overlap_log:
                    world = client.get_world()
                    snapshot = world.get_snapshot()
                    ego = scenario.actor_mapping[EGO_TRACK_ID].actor_inst
                    ego_vertices = ego.bounding_box.get_world_vertices(ego.get_transform())
                    ego_points = np.array([[v.x, v.y, v.z] for v in ego_vertices])
                    ego_min = ego_points.min(axis=0)
                    ego_max = ego_points.max(axis=0)
                    for actor in world.get_actors():
                        if actor.id == ego.id or not (
                            actor.type_id.startswith("vehicle.")
                            or actor.type_id.startswith("walker.pedestrian.")
                        ):
                            continue
                        vertices = actor.bounding_box.get_world_vertices(actor.get_transform())
                        points = np.array([[v.x, v.y, v.z] for v in vertices])
                        overlap = np.minimum(ego_max, points.max(axis=0)) - np.maximum(
                            ego_min, points.min(axis=0)
                        )
                        if np.all(overlap > 0.01):
                            overlap_samples.append(
                                {
                                    "frame": snapshot.frame,
                                    "scenario_time_seconds": scenario.seconds_since_start(),
                                    "other_actor_id": actor.id,
                                    "other_actor_type": actor.type_id,
                                    "other_role_name": actor.attributes.get("role_name", ""),
                                    "aabb_overlap_m": {
                                        "x": float(overlap[0]),
                                        "y": float(overlap[1]),
                                        "z": float(overlap[2]),
                                    },
                                }
                            )
                if should_apply_control and scenario.seconds_since_start() > 1:
'''

OVERLAP_TICK = '''                scenario.tick()
                if args.overlap_log:
                    world = client.get_world()
                    snapshot = world.get_snapshot()
                    ego_id = scenario.actor_mapping[EGO_TRACK_ID].actor_inst.id
                    boxes = []
                    for actor in world.get_actors():
                        if not (
                            actor.type_id.startswith("vehicle.")
                            or actor.type_id.startswith("walker.pedestrian.")
                        ):
                            continue
                        transform = actor.get_transform()
                        vertices = actor.bounding_box.get_world_vertices(transform)
                        points = np.array([[v.x, v.y, v.z] for v in vertices])
                        yaw = np.radians(
                            transform.rotation.yaw + actor.bounding_box.rotation.yaw
                        )
                        axes = (
                            np.array([np.cos(yaw), np.sin(yaw)]),
                            np.array([-np.sin(yaw), np.cos(yaw)]),
                        )
                        boxes.append((actor, points, axes))

                    for left_idx, (left, left_points, left_axes) in enumerate(boxes):
                        for right, right_points, right_axes in boxes[left_idx + 1 :]:
                            z_overlap = min(
                                left_points[:, 2].max(), right_points[:, 2].max()
                            ) - max(left_points[:, 2].min(), right_points[:, 2].min())
                            if z_overlap <= 0.01:
                                continue
                            penetrations = []
                            separated = False
                            for axis in left_axes + right_axes:
                                left_projection = left_points[:, :2] @ axis
                                right_projection = right_points[:, :2] @ axis
                                penetration = min(
                                    left_projection.max(), right_projection.max()
                                ) - max(left_projection.min(), right_projection.min())
                                if penetration <= 0.01:
                                    separated = True
                                    break
                                penetrations.append(float(penetration))
                            if separated:
                                continue
                            overlap_samples.append(
                                {
                                    "frame": snapshot.frame,
                                    "scenario_time_seconds": scenario.seconds_since_start(),
                                    "actor_a_id": left.id,
                                    "actor_a_type": left.type_id,
                                    "actor_b_id": right.id,
                                    "actor_b_type": right.type_id,
                                    "involves_ego": left.id == ego_id or right.id == ego_id,
                                    "minimum_horizontal_penetration_m": min(penetrations),
                                    "vertical_penetration_m": float(z_overlap),
                                }
                            )
                if should_apply_control and scenario.seconds_since_start() > 1:
'''

BASE_FINALLY = '''        finally:
            if spectator is not None:
'''
COLLISION_FINALLY = '''        finally:
            if collision_sensor is not None:
                collision_sensor.stop()
                collision_sensor.destroy()
            if args.collision_log:
                collision_path = Path(args.collision_log)
                collision_path.parent.mkdir(parents=True, exist_ok=True)
                collision_path.write_text(
                    json.dumps(
                        {
                            "schema_version": "nurec_collision_diagnostics.v1",
                            "collision_count": len(collision_events),
                            "events": collision_events,
                        },
                        indent=2,
                    )
                    + "\\n",
                    encoding="utf-8",
                )
            if spectator is not None:
'''
EGO_OVERLAP_FINALLY = '''        finally:
            if args.overlap_log:
                overlap_path = Path(args.overlap_log)
                overlap_path.parent.mkdir(parents=True, exist_ok=True)
                overlap_path.write_text(
                    json.dumps(
                        {
                            "schema_version": "nurec_overlap_diagnostics.v1",
                            "method": "world_axis_aligned_actor_bounding_boxes",
                            "kinematic_replay": True,
                            "sample_count": len(overlap_samples),
                            "samples": overlap_samples,
                        },
                        indent=2,
                    )
                    + "\\n",
                    encoding="utf-8",
                )
            if spectator is not None:
'''

OVERLAP_FINALLY = EGO_OVERLAP_FINALLY.replace(
    '"method": "world_axis_aligned_actor_bounding_boxes",',
    '"method": "oriented_actor_bounding_boxes_sat_2d_plus_z",',
)


def replace_variant(text: str, variants, replacement: str, name: str):
    if replacement in text:
        return text, False
    for variant in variants:
        if variant in text:
            return text.replace(variant, replacement, 1), True
    raise RuntimeError(f"expected {name} block was not found")


def apply_patch(target: Path) -> str:
    original = target.read_text(encoding="utf-8")
    if (
        JSON_IMPORTS in original
        and OVERLAP_CLI in original
        and OVERLAP_STATE in original
        and OVERLAP_TICK in original
        and OVERLAP_FINALLY in original
        and COLLISION_SENSOR_SUFFIX not in original
    ):
        return "already_patched"
    patched = original
    changes = []
    if COLLISION_SENSOR_SUFFIX in patched:
        patched = patched.replace(COLLISION_SENSOR_SUFFIX, "", 1)
        changes.append("remove-collision-sensor")
    for variants, replacement, name in (
        ((BASE_IMPORTS,), JSON_IMPORTS, "imports"),
        ((COLLISION_CLI, BASE_CLI), OVERLAP_CLI, "cli"),
        ((COLLISION_STATE, BASE_STATE), OVERLAP_STATE, "state"),
        ((EGO_OVERLAP_TICK, BASE_TICK), OVERLAP_TICK, "overlap-sampling"),
        ((EGO_OVERLAP_FINALLY, COLLISION_FINALLY, BASE_FINALLY), OVERLAP_FINALLY, "report"),
    ):
        patched, changed = replace_variant(patched, variants, replacement, name)
        if changed:
            changes.append(name)
    if not changes:
        return "already_patched"
    backup = target.with_suffix(target.suffix + ".pre-overlap-diagnostics")
    if not backup.exists():
        shutil.copy2(target, backup)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(patched, encoding="utf-8")
    os.replace(temporary, target)
    return "patched:" + ",".join(changes)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=Path)
    args = parser.parse_args()
    if not args.target.is_file():
        parser.error(f"target does not exist: {args.target}")
    print(apply_patch(args.target.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
