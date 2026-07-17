import copy
import json
import tempfile
import unittest
from pathlib import Path


SCENE_TOKEN = "cc8c0bf57f984915a77078b10eb33198"
VEHICLE_TRACK = "c1958768d48640948f6053d04cffd35b"
PEDESTRIAN_TRACK = "a" * 32


def _actor(actor_id, actor_type, category):
    return {
        "actor_id": actor_id,
        "source_track_id": actor_id,
        "role": "context",
        "type": actor_type,
        "category": category,
        "initial_state": {"t_sec": 0.0, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0, "speed_mps": 1.0},
        "reference_trajectory": [
            {"t_sec": 0.0, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0, "speed_mps": 1.0},
            {"t_sec": 0.5, "x": 1.5, "y": 2.0, "z": 0.0, "yaw": 0.0, "speed_mps": 1.0},
        ],
        "policy_hints": {"mvp": "replay", "final_closed_loop": "reference_conditioned_reactive_rule_based"},
    }


def _scenario_ir():
    return {
        "schema_version": "scenario_ir.v1",
        "scenario_id": SCENE_TOKEN,
        "source": {"dataset": "nuscenes", "scene_token": SCENE_TOKEN},
        "actors": [
            _actor(VEHICLE_TRACK, "vehicle", "vehicle.car"),
            _actor(PEDESTRIAN_TRACK, "pedestrian", "human.pedestrian.adult"),
        ],
    }


class ActorBindingTests(unittest.TestCase):
    def test_verified_vehicle_is_one_multimodal_interactive_identity(self):
        from adapters.actor_binding import build_actor_binding_set

        result = build_actor_binding_set(
            _scenario_ir(),
            selected_actor_ids=[VEHICLE_TRACK],
            nurec_track_ids=[VEHICLE_TRACK],
            control_modes={VEHICLE_TRACK: "scripted"},
        )

        self.assertEqual(result["readiness"]["status"], "ready")
        binding = result["bindings"][0]
        self.assertEqual(binding["source_track_id"], binding["nurec"]["track_id"])
        self.assertEqual(binding["sensor_sync"]["required_modalities"], ["rgb", "lidar"])
        self.assertEqual(binding["sensor_sync"]["pose_source"], "carla_runtime_actor_pose")
        self.assertTrue(binding["control"]["ego_responsive"])

    def test_inventory_is_evidence_not_an_assumption(self):
        from adapters.actor_binding import (
            ActorBindingError,
            assert_actor_binding_ready,
            build_actor_binding_set,
        )

        result = build_actor_binding_set(
            _scenario_ir(),
            selected_actor_ids=[VEHICLE_TRACK],
            control_modes={VEHICLE_TRACK: "traffic_manager"},
        )
        self.assertEqual(result["bindings"][0]["status"], "pending_nurec_track")
        self.assertIn("nurec_track_inventory_not_provided", result["bindings"][0]["issues"])
        with self.assertRaisesRegex(ActorBindingError, "not multimodal-ready"):
            assert_actor_binding_ready(result)

    def test_scripted_pedestrian_is_root_pose_closed_loop_on_source_corridor(self):
        from adapters.actor_binding import build_actor_binding_set

        result = build_actor_binding_set(
            _scenario_ir(),
            selected_actor_ids=[PEDESTRIAN_TRACK],
            nurec_track_ids=[PEDESTRIAN_TRACK],
            control_modes={PEDESTRIAN_TRACK: "scripted"},
        )
        binding = result["bindings"][0]
        self.assertEqual(binding["control"]["corridor_constraint"], "source_reference")
        self.assertEqual(binding["control"]["capabilities"], ["speed", "pause", "yield", "abort"])
        self.assertEqual(binding["carla"]["blueprint"], "walker.pedestrian.*")

    def test_pedestrian_traffic_manager_and_unknown_actor_fail_closed(self):
        from adapters.actor_binding import ActorBindingError, build_actor_binding_set

        with self.assertRaisesRegex(ActorBindingError, "cannot use CARLA TrafficManager"):
            build_actor_binding_set(
                _scenario_ir(),
                selected_actor_ids=[PEDESTRIAN_TRACK],
                nurec_track_ids=[PEDESTRIAN_TRACK],
                control_modes={PEDESTRIAN_TRACK: "traffic_manager"},
            )
        with self.assertRaisesRegex(ActorBindingError, "do not exist"):
            build_actor_binding_set(_scenario_ir(), selected_actor_ids=["missing"])

    def test_binding_is_attached_to_matching_carla_actor_and_mode(self):
        from adapters.actor_binding import ActorBindingError, bind_carla_run_config, build_actor_binding_set

        binding_set = build_actor_binding_set(
            _scenario_ir(),
            selected_actor_ids=[VEHICLE_TRACK],
            nurec_track_ids=[VEHICLE_TRACK],
            control_modes={VEHICLE_TRACK: "scripted"},
        )
        run = {
            "scenario_id": SCENE_TOKEN,
            "actors": [
                {
                    "actor_id": VEHICLE_TRACK,
                    "source_track_id": VEHICLE_TRACK,
                    "closed_loop_level": "scripted",
                }
            ],
        }
        bound = bind_carla_run_config(run, binding_set)
        actor = bound["actors"][0]
        self.assertEqual(actor["role_name"], f"actor.{VEHICLE_TRACK[:24]}")
        self.assertEqual(actor["binding"]["nurec_track_id"], VEHICLE_TRACK)
        self.assertEqual(actor["binding"]["required_modalities"], ["rgb", "lidar"])

        wrong = copy.deepcopy(run)
        wrong["actors"][0]["closed_loop_level"] = "traffic_manager_reactive"
        with self.assertRaisesRegex(ActorBindingError, "control mismatch"):
            bind_carla_run_config(wrong, binding_set)

    def test_cli_materializes_actor_bound_run_config(self):
        from adapters.actor_binding import build_actor_binding_set
        from runners.bind_actor_run_config import main

        binding_set = build_actor_binding_set(
            _scenario_ir(),
            selected_actor_ids=[VEHICLE_TRACK],
            nurec_track_ids=[VEHICLE_TRACK],
            control_modes={VEHICLE_TRACK: "scripted"},
        )
        run = {
            "scenario_id": SCENE_TOKEN,
            "actors": [
                {
                    "actor_id": VEHICLE_TRACK,
                    "source_track_id": VEHICLE_TRACK,
                    "closed_loop_level": "scripted",
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run_path = root / "run.json"
            binding_path = root / "bindings.json"
            output = root / "bound.json"
            run_path.write_text(json.dumps(run), encoding="utf-8")
            binding_path.write_text(json.dumps(binding_set), encoding="utf-8")
            status = main(
                [
                    "--run-config",
                    str(run_path),
                    "--actor-bindings",
                    str(binding_path),
                    "--output",
                    str(output),
                ]
            )
            bound = json.loads(output.read_text(encoding="utf-8"))

        self.assertEqual(status, 0)
        self.assertEqual(bound["actors"][0]["binding"]["nurec_track_id"], VEHICLE_TRACK)


if __name__ == "__main__":
    unittest.main()
