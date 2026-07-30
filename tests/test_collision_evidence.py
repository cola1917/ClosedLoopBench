import unittest
from types import SimpleNamespace


class CollisionEvidenceTests(unittest.TestCase):
    def test_physical_overlap_without_contact_is_a_failed_collision_audit(self):
        from adapters.scene_safety_audit import audit_collision_tick

        registry = {
            "records": [
                {
                    "object_id": "truck",
                    "role": "static_obstacle",
                    "carla": {"collision_policy": "required"},
                }
            ]
        }
        tick = {
            "frame_id": 10,
            "simulation_time_sec": 0.5,
            "ego_state": {
                "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
            },
            "object_states": [
                {
                    "object_id": "truck",
                    "carla_runtime_actor_id": 4,
                    "pose": {"x": 1.5, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                }
            ],
            "collision_events": [],
            "collision_detected": False,
        }

        result = audit_collision_tick(registry, tick)

        self.assertEqual(result["status"], "failed")
        self.assertIn("unattributed_geometric_overlap:truck", result["issues"])
        self.assertEqual(result["records"][0]["minimum_clearance_m"], 0.0)

    def test_physical_overlap_with_attributed_contact_is_passed(self):
        from adapters.scene_safety_audit import audit_collision_tick

        registry = {
            "records": [
                {
                    "object_id": "truck",
                    "role": "static_obstacle",
                    "carla": {"collision_policy": "required"},
                }
            ]
        }
        tick = {
            "frame_id": 10,
            "simulation_time_sec": 0.5,
            "ego_state": {
                "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
            },
            "object_states": [
                {
                    "object_id": "truck",
                    "carla_runtime_actor_id": 4,
                    "pose": {"x": 1.5, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                }
            ],
            "collision_events": [{"object_id": "truck"}],
            "collision_detected": True,
        }

        result = audit_collision_tick(registry, tick)

        self.assertEqual(result["status"], "passed")
        self.assertTrue(result["records"][0]["bounding_box_overlap"])
        self.assertTrue(result["records"][0]["collision_sensor_contact"])

    def test_unregistered_collision_callback_fails_closed(self):
        from adapters.scene_safety_audit import audit_collision_tick

        registry = {
            "records": [
                {
                    "object_id": "truck",
                    "role": "static_obstacle",
                    "carla": {"collision_policy": "required"},
                }
            ]
        }
        result = audit_collision_tick(
            registry,
            {
                "frame_id": 11,
                "simulation_time_sec": 0.55,
                "ego_state": {
                    "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                },
                "object_states": [
                    {
                        "object_id": "truck",
                        "carla_runtime_actor_id": 4,
                        "pose": {"x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                        "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                    }
                ],
                "collision_events": [
                    {
                        "object_id": "unregistered_runtime_actor:99",
                        "carla_runtime_actor_id": 99,
                    }
                ],
                "collision_detected": True,
            },
        )

        self.assertEqual(result["status"], "failed")
        self.assertIn(
            "collision_event_without_registry_state:unregistered_runtime_actor:99",
            result["issues"],
        )

    def test_unregistered_runtime_object_state_fails_closed(self):
        from adapters.scene_safety_audit import audit_collision_tick

        result = audit_collision_tick(
            {
                "records": [
                    {
                        "object_id": "truck",
                        "role": "static_obstacle",
                        "carla": {"collision_policy": "required"},
                    }
                ]
            },
            {
                "frame_id": 11,
                "simulation_time_sec": 0.55,
                "ego_state": {
                    "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                },
                "object_states": [
                    {
                        "object_id": "unregistered_actor",
                        "carla_runtime_actor_id": 9,
                        "pose": {"x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                        "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                    }
                ],
                "collision_events": [],
                "collision_detected": False,
            },
        )

        self.assertEqual(result["status"], "failed")
        self.assertIn(
            "unregistered_carla_object_state:unregistered_actor",
            result["issues"],
        )

    def test_delayed_collision_payload_preserves_native_frame_and_object_id(self):
        from runners.run_carla_basic_agent import (
            _CollisionTracker,
            _attach_collision_event_payloads,
        )

        other_actor = SimpleNamespace(id=4)
        event = SimpleNamespace(frame=101, other_actor=other_actor, normal_impulse=None)
        tracker = _CollisionTracker()
        tracker.on_collision(event)
        rows = [{"collision_events": []}, {"collision_events": []}]
        frame_trace = [
            {
                "world_tick_frame": 100,
                "snapshot_frame": 100,
                "collision_events": [],
            },
            {
                "world_tick_frame": 101,
                "snapshot_frame": 101,
                "collision_events": [],
            },
        ]
        actor = SimpleNamespace(id=4)

        unmatched = _attach_collision_event_payloads(
            rows,
            frame_trace,
            tracker,
            {"truck": actor},
        )

        self.assertEqual(unmatched, 0)
        payload = frame_trace[1]["collision_events"][0]
        self.assertEqual(payload["sensor_event_frame"], 101)
        self.assertEqual(payload["object_id"], "truck")
        self.assertEqual(rows[1]["collision_events"], frame_trace[1]["collision_events"])

    def test_ordinary_acceptance_trace_runs_physical_collision_audit(self):
        from runners.run_carla_basic_agent import _audit_collision_trace

        registry = {
            "records": [
                {
                    "object_id": "truck",
                    "role": "static_obstacle",
                    "carla": {"collision_policy": "required"},
                }
            ]
        }
        frame_trace = [
            {
                "frame_id": 12,
                "simulation_time_sec": 0.6,
                "ego_state": {
                    "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                },
                "object_states": [
                    {
                        "object_id": "truck",
                        "carla_runtime_actor_id": 4,
                        "pose": {"x": 1.5, "y": 0.0, "z": 0.0, "yaw": 0.0},
                        "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                    }
                ],
                "collision_events": [],
                "collision_detected": False,
                "collision_sensor_available": True,
            }
        ]

        rows = _audit_collision_trace(
            {"scene_object_registry": registry},
            frame_trace,
        )

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["status"], "failed")
        self.assertIn("unattributed_geometric_overlap:truck", rows[0]["issues"])

    def test_m8_runtime_audit_fails_without_registry_and_passes_complete_tick(self):
        from runners.run_carla_basic_agent import (
            _audit_runtime_m8_trace,
            _m8_audit_summary,
        )

        row = {
            "frame_id": 10,
            "simulation_time_sec": 0.5,
            "ego_state": {
                "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
            },
            "object_states": [
                {
                    "object_id": "truck",
                    "carla_runtime_actor_id": 4,
                    "pose": {"x": 5.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                    "extent_m": {"x": 1.0, "y": 1.0, "z": 1.0},
                }
            ],
            "collision_events": [],
            "collision_detected": False,
            "collision_sensor_available": True,
            "lane_state": {
                "road_id": 1,
                "lane_id": -1,
                "lane_type": "Driving",
                "is_on_road": True,
                "inside_lane": True,
                "lane_width_m": 3.5,
                "center_distance_m": 0.1,
                "route_progress": 0.2,
                "lane_invasion_events": [],
                "lane_invasion_sensor_available": True,
            },
        }
        registry = {
            "records": [
                {
                    "object_id": "truck",
                    "role": "static_obstacle",
                    "carla": {"collision_policy": "required"},
                }
            ]
        }

        complete = _audit_runtime_m8_trace(
            {"scene_object_registry": registry}, [row]
        )
        complete_summary = _m8_audit_summary(complete)
        self.assertEqual(complete_summary["status"], "failed")
        self.assertEqual(
            complete_summary["missing_streams"], ["visibility", "lidar_world"]
        )
        missing = _audit_runtime_m8_trace({}, [row])
        self.assertEqual(_m8_audit_summary(missing)["status"], "failed")
        self.assertIn(
            "scene_object_registry_missing", missing["collision"][0]["issues"]
        )

    def test_tracker_preserves_event_count_and_frame_identity(self):
        from runners.run_carla_basic_agent import _CollisionTracker

        tracker = _CollisionTracker()
        self.assertFalse(tracker.consume_tick())
        tracker.on_collision(SimpleNamespace(frame=17))
        self.assertTrue(tracker.consume_tick())
        self.assertFalse(tracker.consume_tick())
        self.assertEqual(tracker.event_count, 1)
        self.assertEqual(tracker.event_frames, [17])

    def test_pending_event_can_be_drained_after_last_tick(self):
        from runners.run_carla_basic_agent import _CollisionTracker

        tracker = _CollisionTracker()
        tracker.on_collision(SimpleNamespace(frame=42))
        self.assertTrue(tracker.consume_pending())
        self.assertEqual(tracker.event_count, 1)
        self.assertEqual(tracker.event_frames, [42])

    def test_delayed_collision_callback_is_reconciled_by_carla_frame(self):
        from runners.run_carla_basic_agent import (
            _CollisionTracker,
            _attach_tracker_events,
        )

        tracker = _CollisionTracker()
        tracker.on_collision(SimpleNamespace(frame=101))
        # The callback may have been consumed by the next loop iteration before
        # its native frame is available; reconciliation must remove that stale
        # tick-level marker.
        rows = [{"collision": True}, {"collision": False}]
        frame_trace = [
            {"world_tick_frame": 100},
            {"world_tick_frame": 101},
        ]

        unmatched = _attach_tracker_events(rows, frame_trace, tracker, "collision")

        self.assertEqual(unmatched, 0)
        self.assertFalse(rows[0]["collision"])
        self.assertTrue(rows[1]["collision"])
        self.assertTrue(frame_trace[1]["collision"])

    def test_unmatched_sensor_event_is_visible_instead_of_becoming_zero(self):
        from runners.run_carla_basic_agent import (
            _CollisionTracker,
            _attach_tracker_events,
        )

        tracker = _CollisionTracker()
        tracker.on_collision(SimpleNamespace(frame=999))
        rows = [{"collision": False}]
        frame_trace = [{"world_tick_frame": 100}]

        unmatched = _attach_tracker_events(rows, frame_trace, tracker, "collision")

        self.assertEqual(unmatched, 1)
        self.assertTrue(rows[0]["collision"])
        self.assertEqual(rows[0]["collision_frame_unmatched_event_count"], 1)

    def test_lane_truth_requires_physical_waypoint_fields(self):
        from runners.run_carla_basic_agent import (
            _lane_truth_is_complete,
            _sample_lane_truth,
        )

        location = SimpleNamespace(x=3.0, y=4.0, z=0.0)
        transform = SimpleNamespace(
            location=location,
            rotation=SimpleNamespace(yaw=0.0),
        )
        waypoint = SimpleNamespace(
            road_id=7,
            section_id=0,
            lane_id=-1,
            lane_type="Driving",
            is_junction=False,
            lane_width=3.5,
            id=7001,
            transform=transform,
        )

        class FakeMap:
            def get_waypoint(self, _location, **_kwargs):
                return waypoint

        vehicle = SimpleNamespace(get_transform=lambda: transform)
        carla = SimpleNamespace(
            LaneType=SimpleNamespace(Driving="Driving"),
        )
        truth = _sample_lane_truth(carla, FakeMap(), vehicle)

        self.assertTrue(_lane_truth_is_complete(truth))
        self.assertEqual(truth["road_id"], 7)
        self.assertEqual(truth["lane_id"], -1)
        self.assertTrue(truth["inside_lane"])
        self.assertEqual(truth["center_distance_m"], 0.0)

        unavailable = _sample_lane_truth(carla, None, vehicle)
        self.assertFalse(_lane_truth_is_complete(unavailable))

    def test_lane_invasion_tracker_is_attached_to_the_ego_sensor(self):
        from runners.run_carla_basic_agent import _spawn_lane_invasion_tracker

        class Blueprint:
            def __init__(self, identifier):
                self.identifier = identifier

        class Library:
            def find(self, identifier):
                return Blueprint(identifier)

        class Sensor:
            def __init__(self):
                self.callback = None
                self.destroyed = False

            def listen(self, callback):
                self.callback = callback

            def stop(self):
                pass

            def destroy(self):
                self.destroyed = True

        class World:
            def __init__(self):
                self.sensor = Sensor()
                self.spawn_args = None

            def get_blueprint_library(self):
                return Library()

            def spawn_actor(self, blueprint, transform, attach_to=None):
                self.spawn_args = (blueprint.identifier, transform, attach_to)
                return self.sensor

        carla = SimpleNamespace(Transform=lambda: SimpleNamespace())
        ego = object()
        world = World()

        tracker, sensor = _spawn_lane_invasion_tracker(carla, world, ego)

        self.assertIsNotNone(tracker)
        self.assertIs(sensor, world.sensor)
        self.assertEqual(world.spawn_args[0], "sensor.other.lane_invasion")
        self.assertIs(world.spawn_args[2], ego)
        sensor.callback(SimpleNamespace(frame=23))
        self.assertTrue(tracker.consume_tick(23))
        self.assertEqual(tracker.event_frames, [23])


if __name__ == "__main__":
    unittest.main()
