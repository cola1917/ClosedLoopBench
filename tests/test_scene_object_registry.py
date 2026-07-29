import json
import tempfile
import unittest
from pathlib import Path


SCENE = "c" * 32
VEHICLE = "1" * 32
PEDESTRIAN = "2" * 32


def _state(t, x, y):
    return {"t_sec": t, "x": x, "y": y, "z": 0.0, "yaw": 0.0}


def _scenario():
    return {
        "schema_version": "scenario_ir.v1",
        "scenario_id": SCENE,
        "actors": [
            {"actor_id": VEHICLE, "source_track_id": VEHICLE, "type": "vehicle", "category": "vehicle.car", "reference_trajectory": [_state(0, 8, 0), _state(1, 9, 0)]},
            {"actor_id": PEDESTRIAN, "source_track_id": PEDESTRIAN, "type": "pedestrian", "category": "human.pedestrian.adult", "reference_trajectory": [_state(0, 3, 2), _state(1, 3, 2)]},
        ],
    }


def _parked_vehicle():
    return {
        "object_id": "static:roadside-parked-vehicle-01",
        "semantic_class": "vehicle",
        "category": "vehicle.truck",
        "source": {"kind": "nre_scene_annotation", "reference": "front-camera frame 0"},
        "placement": {"x": 12, "y": -2.5, "z": 0, "yaw": 0},
    }


class SceneObjectRegistryTests(unittest.TestCase):
    def test_registers_full_dynamic_catalog_static_obstacle_and_boundary(self):
        from adapters.scene_object_registry import build_scene_object_registry

        registry = build_scene_object_registry(
            _scenario(),
            static_objects=[_parked_vehicle()],
            role_overrides={VEHICLE: "controlled_lead_vehicle", PEDESTRIAN: "controlled_pedestrian"},
        )

        self.assertEqual(registry["schema_version"], "scene_object_registry.v1")
        self.assertEqual(registry["summary"]["object_count"], 4)
        self.assertEqual(registry["summary"]["controlled_actor_count"], 2)
        parked = next(record for record in registry["records"] if record["object_id"].startswith("static:"))
        self.assertEqual(parked["role"], "static_obstacle")
        self.assertEqual(parked["carla"]["collision_policy"], "required")
        self.assertFalse(parked["control"]["controllable"])

    def test_registers_source_cones_and_barriers_as_static_collision_proxies(self):
        from adapters.scene_object_registry import build_scene_object_registry

        scenario = _scenario()
        scenario["actors"].append(
            {
                "actor_id": "3" * 32,
                "source_track_id": "3" * 32,
                "type": "object",
                "category": "movable_object.trafficcone",
                "initial_state": _state(0, 4, -2),
                "reference_trajectory": [_state(0, 4, -2)],
            }
        )
        registry = build_scene_object_registry(scenario, static_objects=[])
        cone = next(record for record in registry["records"] if record["object_id"] == "3" * 32)

        self.assertEqual(cone["role"], "static_obstacle")
        self.assertEqual(cone["carla"]["representation"], "static_collision_proxy")
        self.assertEqual(cone["carla"]["blueprint_class"], "static.prop.trafficcone01")
        self.assertEqual(cone["nurec"]["representation"], "source_scene_appearance")

    def test_registers_single_observation_pedestrian_as_static_visible_collision_proxy(self):
        from adapters.scene_object_registry import build_scene_object_registry

        singleton = "5" * 32
        scenario = _scenario()
        scenario["actors"].append(
            {
                "actor_id": singleton,
                "source_track_id": singleton,
                "type": "pedestrian",
                "category": "human.pedestrian.adult",
                "initial_state": _state(0, 4, -2),
                "reference_trajectory": [_state(0, 4, -2)],
            }
        )

        registry = build_scene_object_registry(
            scenario, static_objects=[], nonreplay_static_actor_ids={singleton}
        )
        record = next(item for item in registry["records"] if item["object_id"] == singleton)

        self.assertEqual(record["role"], "static_obstacle")
        self.assertEqual(record["source"]["kind"], "nuscenes_single_observation_track")
        self.assertEqual(record["nurec"]["representation"], "source_scene_appearance")
        self.assertEqual(record["carla"]["blueprint_class"], "walker.pedestrian.*")
        self.assertEqual(record["carla"]["placement"]["x"], 4.0)
        self.assertFalse(record["control"]["controllable"])

    def test_derives_new_m8_registry_without_mutating_prior_actor_identity_set(self):
        from runners.derive_m8_registry_from_m6_plan import derive_registry
        from adapters.scene_object_registry import build_scene_object_registry

        singleton = "6" * 32
        scenario = _scenario()
        scenario["actors"].append(
            {
                "actor_id": singleton,
                "source_track_id": singleton,
                "type": "pedestrian",
                "category": "human.pedestrian.adult",
                "initial_state": _state(0, 4, -2),
                "reference_trajectory": [_state(0, 4, -2), _state(1, 4, -2)],
            }
        )
        prior = build_scene_object_registry(scenario, static_objects=[])
        plan = {"schema_version": "basic_agent_plan.v1", "scenario_id": SCENE, "actors": scenario["actors"]}
        plan["actors"][-1]["reference_trajectory"] = [_state(0, 4, -2)]

        derived, manifest = derive_registry(
            prior, plan, nonreplay_static_actor_ids={singleton}
        )
        record = next(item for item in derived["records"] if item["object_id"] == singleton)

        self.assertEqual(record["role"], "static_obstacle")
        self.assertTrue(manifest["object_id_match"])
        self.assertEqual(manifest["reclassified_records"][0]["object_id"], singleton)

    def test_visible_unknown_or_noncollidable_object_blocks_promotion(self):
        from adapters.scene_object_registry import (
            SceneObjectRegistryError,
            assert_scene_object_coverage_ready,
            audit_scene_object_coverage,
            build_scene_object_registry,
        )

        registry = build_scene_object_registry(_scenario(), static_objects=[_parked_vehicle()])
        audit = audit_scene_object_coverage(
            registry,
            {
                "schema_version": "scene_object_visibility_manifest.v1",
                "scene_id": SCENE,
                "observations": [
                    {"object_id": "static:roadside-parked-vehicle-01", "safety_relevant": True, "camera": "CAM_FRONT", "t_sec": 0},
                    {"object_id": "unregistered:barrier", "safety_relevant": True, "camera": "CAM_FRONT", "t_sec": 0},
                ],
            },
        )
        self.assertEqual(audit["status"], "failed")
        self.assertIn("unregistered_nre_visible_safety_object", audit["issues"])
        with self.assertRaisesRegex(SceneObjectRegistryError, "not ready"):
            assert_scene_object_coverage_ready(audit)

    def test_complete_visibility_manifest_passes_coverage(self):
        from adapters.scene_object_registry import audit_scene_object_coverage, build_scene_object_registry

        registry = build_scene_object_registry(_scenario(), static_objects=[_parked_vehicle()])
        audit = audit_scene_object_coverage(
            registry,
            {
                "schema_version": "scene_object_visibility_manifest.v1",
                "scene_id": SCENE,
                "observations": [
                    {"source_track_id": VEHICLE, "safety_relevant": True, "camera": "CAM_FRONT", "t_sec": 0},
                    {"object_id": "static:roadside-parked-vehicle-01", "safety_relevant": True, "camera": "CAM_FRONT", "t_sec": 0},
                ],
            },
        )
        self.assertEqual(audit["status"], "passed")

    def test_visible_road_boundary_uses_topology_not_a_collision_proxy(self):
        from adapters.scene_object_registry import audit_scene_object_coverage, build_scene_object_registry

        registry = build_scene_object_registry(_scenario(), static_objects=[_parked_vehicle()])
        audit = audit_scene_object_coverage(
            registry,
            {
                "schema_version": "scene_object_visibility_manifest.v1",
                "scene_id": SCENE,
                "observations": [
                    {"object_id": "road_boundary:carla_map", "safety_relevant": True, "camera": "CAM_FRONT", "t_sec": 0},
                ],
            },
        )
        self.assertEqual(audit["status"], "passed")

    def test_cli_writes_immutable_sidecars(self):
        from runners.build_scene_object_registry import main

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario = root / "scenario.json"
            static = root / "static.json"
            visible = root / "visible.json"
            registry = root / "registry.json"
            audit = root / "audit.json"
            scenario.write_text(json.dumps(_scenario()), encoding="utf-8")
            static.write_text(json.dumps({"objects": [_parked_vehicle()]}), encoding="utf-8")
            visible.write_text(json.dumps({"schema_version": "scene_object_visibility_manifest.v1", "scene_id": SCENE, "observations": [{"object_id": "static:roadside-parked-vehicle-01", "safety_relevant": True}]}), encoding="utf-8")
            result = main(["--scenario-ir", str(scenario), "--static-object-manifest", str(static), "--visibility-manifest", str(visible), "--output", str(registry), "--audit-output", str(audit), "--require-coverage-ready"])

            self.assertEqual(result, 0)
            self.assertEqual(json.loads(registry.read_text(encoding="utf-8"))["schema_version"], "scene_object_registry.v1")
            self.assertEqual(json.loads(audit.read_text(encoding="utf-8"))["status"], "passed")

    def test_static_collision_proxy_spawns_at_declared_pose_with_runtime_evidence(self):
        from runners.run_carla_basic_agent import (
            _spawn_static_obstacles,
            _static_obstacle_runtime_evidence,
        )
        from tests.test_basic_agent_runtime_loop import FakeCarlaModule, FakeWorld

        events = []
        world = FakeWorld(events)
        obstacle = {
            "object_id": "static:roadside-parked-vehicle-01",
            "semantic_class": "vehicle",
            "source": {"kind": "nre_scene_annotation"},
            "placement": {"x": 12.0, "y": -2.5, "z": 0.0, "yaw": 0.0},
            "blueprint": "static.prop.*",
            "collision_policy": "required",
        }
        spawned = {}
        _spawn_static_obstacles(FakeCarlaModule(events), world, [obstacle], spawned)
        audit = _static_obstacle_runtime_evidence({"static_obstacles": [obstacle]}, spawned)

        self.assertIn("static:roadside-parked-vehicle-01", spawned)
        self.assertIn("world.try_spawn_actor.role=static.static:roadside-parked-v.x=12.0", events)
        self.assertEqual(audit["status"], "passed")

    def test_derives_an_m6_carla_config_from_immutable_registry(self):
        from adapters.scene_object_registry import (
            attach_dynamic_replay_to_carla_run,
            attach_static_obstacles_to_carla_run,
            build_scene_object_registry,
        )

        registry = build_scene_object_registry(_scenario(), static_objects=[_parked_vehicle()])
        derived = attach_static_obstacles_to_carla_run(
            {"scenario_id": SCENE, "runtime": {"acceptance_evidence": False}},
            registry,
            registry_path="/evidence/registry.json",
            registry_sha256="1" * 64,
        )

        self.assertEqual(derived["scene_object_registry"]["sha256"], "1" * 64)
        self.assertTrue(derived["runtime"]["m6_static_obstacle_required"])
        self.assertEqual(derived["static_obstacles"][0]["collision_policy"], "required")

        from runners.run_carla_basic_agent import build_basic_agent_plan

        plan = build_basic_agent_plan(
            derived,
            max_ticks=1,
            actor_autopilot=False,
            physics_smoke=True,
        )
        self.assertEqual(len(plan["static_obstacles"]), 1)
        self.assertTrue(plan["runtime"]["m6_static_obstacle_required"])

        replay = attach_dynamic_replay_to_carla_run(derived, registry, _scenario())
        self.assertEqual(len(replay["actors"]), 2)
        self.assertTrue(replay["runtime"]["m6_dynamic_replay_required"])
        self.assertTrue(all(actor["closed_loop_level"] == "replay" for actor in replay["actors"]))
        self.assertTrue(all(actor["m6_allow_vertical_pose_calibration"] for actor in replay["actors"]))

        bicycle = _scenario()
        bicycle["actors"].append(
            {
                "actor_id": "4" * 32,
                "source_track_id": "4" * 32,
                "type": "two_wheeler",
                "category": "vehicle.bicycle",
                "initial_state": _state(0, 4, -2),
                "reference_trajectory": [_state(0, 4, -2), _state(1, 4.5, -2)],
            }
        )
        bicycle_registry = build_scene_object_registry(bicycle, static_objects=[])
        bicycle_replay = attach_dynamic_replay_to_carla_run(
            {"scenario_id": SCENE}, bicycle_registry, bicycle
        )
        selected = next(actor for actor in bicycle_replay["actors"] if actor["actor_id"] == "4" * 32)
        self.assertEqual(selected["blueprint"], "vehicle.bh.crossbike")

    def test_m6_actor_vertical_retry_preserves_xy_yaw_and_is_bounded(self):
        from runners.run_carla_basic_agent import _m6_actor_vertical_retry_transforms
        from tests.test_basic_agent_runtime_loop import FakeCarlaModule

        retries = _m6_actor_vertical_retry_transforms(
            FakeCarlaModule([]),
            {
                "initial_state": {"x": 3, "y": 4, "z": 0.6, "yaw": 15},
                "m6_max_vertical_spawn_adjustment_m": 0.1,
            },
        )

        self.assertEqual(len(retries), 2)
        transform, evidence = retries[-1]
        self.assertEqual(transform.location.x, 3.0)
        self.assertEqual(transform.location.y, -4.0)
        self.assertEqual(transform.rotation.yaw, -15.0)
        self.assertEqual(evidence["vertical_adjustment_m"], 0.1)

    def test_runtime_audit_requires_dynamic_static_and_road_coverage(self):
        from runners.audit_scene_object_runtime import audit_scene_object_runtime
        from adapters.scene_object_registry import build_scene_object_registry

        registry = build_scene_object_registry(_scenario(), static_objects=[_parked_vehicle()])
        report = {
            "status": "ego_closed_loop",
            "runtime": {
                "static_obstacle_runtime": {
                    "records": [
                        {
                            "object_id": "static:roadside-parked-vehicle-01",
                            "carla_runtime_actor_id": 4,
                            "status": "passed",
                        }
                    ]
                }
            },
        }
        frames = [{"actor_states": {VEHICLE: {"carla_runtime_actor_id": 2, "spawn_evidence": {}}, PEDESTRIAN: {"carla_runtime_actor_id": 3, "spawn_evidence": {"vertical_adjustment_m": 0.1}}}}]
        audit = audit_scene_object_runtime(registry, report, frames)

        self.assertEqual(audit["status"], "passed")
        self.assertEqual(audit["summary"]["dynamic_runtime_count"], 2)
        self.assertEqual(audit["summary"]["static_runtime_count"], 1)

    def test_nurec_camera_capture_requires_every_requested_camera(self):
        from runners.capture_nurec_camera_calibrations import advertised_camera_calibrations

        config = {
            "scenario_id": SCENE,
            "nurec_runtime": {
                "runtime_scene_id": "scene-0061",
                "camera_specs": [
                    {"sensor_id": "camera_front", "channel": "CAM_FRONT", "width": 1600, "height": 900, "sensor_to_ego": [1] * 16},
                    {"sensor_id": "camera_back", "channel": "CAM_BACK", "width": 1600, "height": 900, "sensor_to_ego": [1] * 16},
                ],
            },
        }
        with self.assertRaisesRegex(ValueError, "did not advertise"):
            advertised_camera_calibrations(config, {"camera_front": object()})

    def test_nurec_camera_capture_binds_source_intrinsics_by_calibration_token(self):
        from runners.capture_nurec_camera_calibrations import attach_nuscenes_intrinsics

        result = attach_nuscenes_intrinsics(
            {"camera_records": [{"sensor_id": "camera_front", "calibrated_sensor_token": "cal-front"}]},
            [{"token": "cal-front", "camera_intrinsic": [[100.0, 0.0, 50.0], [0.0, 101.0, 40.0], [0.0, 0.0, 1.0]]}],
            source_sha256="a" * 64,
        )
        self.assertEqual(result["intrinsics_status"], "passed")
        self.assertEqual(result["camera_records"][0]["intrinsic_matrix_3x3"][0][0], 100.0)

    def test_visibility_manifest_binds_projection_to_complete_nre_payloads(self):
        from adapters.scene_object_registry import build_scene_object_registry
        from adapters.scene_object_visibility import build_visibility_manifest

        registry = build_scene_object_registry(_scenario(), static_objects=[_parked_vehicle()])
        camera_ids = [
            "camera_front", "camera_front_left", "camera_front_right",
            "camera_back", "camera_back_left", "camera_back_right",
        ]
        identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        config = {"nurec_runtime": {"camera_specs": [{"sensor_id": camera, "width": 100, "height": 100, "sensor_to_ego": identity, "calibrated_sensor_token": camera} for camera in camera_ids]}}
        capture = {
            "schema_version": "nurec_camera_calibration_capture.v1",
            "scene_id": SCENE,
            "intrinsics_status": "passed",
            "camera_records": [{"sensor_id": camera, "requested_resolution": {"width": 100, "height": 100}, "calibrated_sensor_token": camera, "intrinsic_matrix_3x3": [[50.0, 0.0, 50.0], [0.0, 50.0, 50.0], [0.0, 0.0, 1.0]], "intrinsics_source": {"table_sha256": "a" * 64}} for camera in camera_ids],
        }
        actor_state = {"render_pose": {"x": 0.0, "y": 0.0, "z": 10.0, "yaw": 0.0}, "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0}}
        frames = [{"world_tick_frame": 10, "simulation_time_sec": 0.1, "ego_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}, "actor_states": {VEHICLE: actor_state, PEDESTRIAN: actor_state}}]
        trace = [{"status": "passed", "frame_id": 10, "records": [{"modality": "rgb", "status": "passed", "sensor_id": camera, "response_metadata": {"materialized_payload": {"sha256": "b" * 64, "relative_path": f"frame/camera_{camera}.jpg"}}} for camera in camera_ids]}]
        manifest = build_visibility_manifest(registry, config, frames, trace, capture)

        self.assertEqual(manifest["summary"]["complete_six_camera_frame_count"], 1)
        self.assertTrue(any(row["object_id"] == VEHICLE for row in manifest["observations"]))
        self.assertTrue(any(row["object_id"] == "road_boundary:carla_map" for row in manifest["observations"]))

    def test_visibility_manifest_skips_dynamic_tracks_absent_from_the_tick(self):
        from adapters.scene_object_visibility import _record_pose_and_extent

        self.assertIsNone(
            _record_pose_and_extent(
                {"object_id": "not-running", "role": "background_replay"},
                {},
            )
        )

    def test_visibility_projection_keeps_canonical_left_right_axis(self):
        from adapters.scene_object_visibility import (
            _canonical_scene_pose,
            _project_box,
            _record_pose_and_extent,
        )

        camera = {
            "camera_front": {
                "matrix": [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
                # nuScenes camera optical -> canonical scene (x-forward, y-left, z-up).
                "sensor_to_ego": [
                    0.0, 0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0, 0.0,
                    0.0, -1.0, 0.0, 1.5,
                    0.0, 0.0, 0.0, 1.0,
                ],
                "width": 100,
                "height": 100,
            }
        }
        ego = _canonical_scene_pose({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}, "ego")
        left_pose, extent = _record_pose_and_extent(
            {"object_id": VEHICLE, "role": "background_replay"},
            {
                VEHICLE: {
                    "render_pose": {"x": 10.0, "y": 2.0, "z": 1.0, "yaw": 0.0},
                    "extent_m": {"x": 0.2, "y": 0.2, "z": 0.8},
                }
            },
        )
        right_pose = _canonical_scene_pose(
            {"x": 10.0, "y": -2.0, "z": 1.0, "yaw": 0.0}, "right actor"
        )

        left_box = _project_box(left_pose, extent, ego, camera, 30.0)["camera_front"]["bbox_xyxy_px"]
        right_box = _project_box(right_pose, extent, ego, camera, 30.0)["camera_front"]["bbox_xyxy_px"]

        self.assertEqual(left_pose[1], 2.0)
        self.assertLess((left_box[0] + left_box[2]) / 2.0, 50.0)
        self.assertGreater((right_box[0] + right_box[2]) / 2.0, 50.0)

    def test_visibility_projection_preserves_left_and_right_camera_assignments(self):
        from math import cos, radians, sin

        from adapters.scene_object_visibility import _project_box

        # Camera optical axes are z-forward/x-right.  Build the pair in the
        # canonical x-forward/y-left frame so a left target cannot leak into
        # the right camera merely because a CARLA-axis reflection was applied.
        def side_camera(yaw_deg: float) -> dict[str, object]:
            yaw = radians(yaw_deg)
            return {
                "matrix": [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
                "sensor_to_ego": [
                    sin(yaw), 0.0, cos(yaw), 1.5,
                    -cos(yaw), 0.0, sin(yaw), 0.5 if yaw_deg > 0.0 else -0.5,
                    0.0, -1.0, 0.0, 1.5,
                    0.0, 0.0, 0.0, 1.0,
                ],
                "width": 100,
                "height": 100,
            }

        cameras = {
            "camera_front_left": side_camera(55.0),
            "camera_front_right": side_camera(-55.0),
        }
        ego = [0.0, 0.0, 0.0, 0.0]
        left_target = _project_box([10.0, 10.0, 1.0, 0.0], [0.5, 0.5, 1.0], ego, cameras, 30.0)
        right_target = _project_box([10.0, -10.0, 1.0, 0.0], [0.5, 0.5, 1.0], ego, cameras, 30.0)

        self.assertEqual(set(left_target), {"camera_front_left"})
        self.assertEqual(set(right_target), {"camera_front_right"})

    def test_m6_replay_binding_freeze_reconciles_embedded_and_sidecar_contracts(self):
        from runners.freeze_scene0061_replay_actor_bindings import freeze_replay_actor_bindings

        actor = {"actor_id": VEHICLE, "closed_loop_level": "replay", "binding": {"sensor_pose_source": "scenario_ir_reference_trajectory", "sensor_pose_reference": "source_track_frame"}}
        config = {"actor_binding": {"selected_actor_ids": [VEHICLE]}, "actors": [actor]}
        bindings = {"bindings": [{"actor_id": VEHICLE, "control": {"mode": "scripted", "ego_responsive": True}, "sensor_sync": {"pose_source": "carla_runtime_actor_pose", "pose_reference": "carla_bounding_box_center"}}]}
        frozen_config, frozen_bindings = freeze_replay_actor_bindings(config, bindings)

        self.assertEqual(frozen_config["actors"][0]["binding"]["sensor_pose_reference"], "source_track_frame")
        self.assertEqual(frozen_bindings["bindings"][0]["sensor_sync"]["pose_source"], "scenario_ir_reference_trajectory")

    def test_nurec_vertical_gate_only_checks_nurec_bound_actors(self):
        from runners.run_carla_basic_agent import _actor_vertical_alignment_issues

        issues = _actor_vertical_alignment_issues(
            {"bound": {"reference_vertical_error_m": 0.1}, "background": {"reference_vertical_error_m": 2.0}},
            actor_ids={"bound"},
            max_error_m=0.25,
        )
        self.assertEqual(issues, [])

    def test_coverage_audit_cli_requires_existing_output_to_be_immutable(self):
        from runners.audit_scene_object_coverage import main

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "audit.json"
            output.write_text("{}", encoding="utf-8")
            with self.assertRaises(SystemExit) as raised:
                main(["--scene-object-registry", "missing.json", "--visibility-manifest", "missing.json", "--output", str(output)])
            self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
