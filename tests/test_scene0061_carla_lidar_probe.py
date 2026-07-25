import hashlib
import json
import struct
import tempfile
import unittest
from pathlib import Path


class _Blueprint:
    def __init__(self):
        self.attributes = {}

    def set_attribute(self, key, value):
        self.attributes[key] = value


class _Library:
    def __init__(self, blueprint):
        self.blueprint = blueprint

    def filter(self, key):
        return [self.blueprint] if key == "sensor.lidar.ray_cast" else []


class _Sensor:
    def __init__(self):
        self.callback = None
        self.stopped = False
        self.destroyed = False
        self.transform = None

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.stopped = True

    def destroy(self):
        self.destroyed = True

    def get_transform(self):
        return self.transform


class _World:
    def __init__(self):
        self.blueprint = _Blueprint()
        self.sensor = _Sensor()
        self.spawn = None

    def get_blueprint_library(self):
        return _Library(self.blueprint)

    def spawn_actor(self, blueprint, transform, attach_to):
        self.spawn = (blueprint, transform, attach_to)
        return self.sensor


class _Carla:
    class Location:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class Rotation:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class Transform:
        def __init__(self, location, rotation):
            self.location = location
            self.rotation = rotation


class _Measurement:
    def __init__(self, frame, raw_data):
        self.frame = frame
        self.raw_data = raw_data


class _Actor:
    def __init__(self, transform):
        self.transform = transform

    def get_transform(self):
        return self.transform


def _matrix():
    return [
        1.0, 0.0, 0.0, 1.2,
        0.0, 1.0, 0.0, -0.3,
        0.0, 0.0, 1.0, 2.1,
        0.0, 0.0, 0.0, 1.0,
    ]


class CarlaNativeLidarProbeTests(unittest.TestCase):
    def _probe(self, root):
        from runtime.scene0061_carla_lidar_probe import CarlaNativeLidarProbe

        world = _World()
        ego = _Actor(
            _Carla.Transform(
                _Carla.Location(x=1.0, y=2.0, z=3.0),
                _Carla.Rotation(roll=0.0, pitch=0.0, yaw=90.0),
            )
        )
        probe = CarlaNativeLidarProbe(
            carla_module=_Carla,
            world=world,
            ego_vehicle=ego,
            output_dir=root,
            sensor_to_ego=_matrix(),
        )
        probe.start()
        world.sensor.transform = _Carla.Transform(
            _Carla.Location(x=1.0, y=4.0, z=3.0),
            _Carla.Rotation(roll=0.0, pitch=0.0, yaw=90.0),
        )
        return probe, world

    def test_persists_exact_callback_frame_as_hashed_independent_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            probe, world = self._probe(Path(directory))
            raw = b"".join(struct.pack("<4f", *row) for row in ((1, 2, 3, .5), (2, 4, 6, .4)))
            world.sensor.callback(_Measurement(41, raw))
            output = probe.persist_frame(41)
            capture = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(capture["carla_frame_id"], 41)
            self.assertEqual(capture["capture_source"], "carla_sensor.lidar.ray_cast_callback_raw_data")
            self.assertEqual(capture["coordinate_frame"], "carla_sensor")
            ref = capture["raw_xyzi_ref"]
            self.assertEqual(ref["sha256"], hashlib.sha256(raw).hexdigest())
            self.assertEqual(ref["byte_count"], len(raw))
            self.assertEqual(Path(ref["path"]).read_bytes(), raw)
            self.assertEqual(world.spawn[1].location.z, 2.1)
            # The evidence pose is derived from actual actor API observations,
            # not the requested configuration matrix (whose x is 1.2).
            self.assertAlmostEqual(capture["sensor_to_ego"][3], 2.0)
            self.assertAlmostEqual(capture["sensor_to_ego"][7], 0.0)
            self.assertEqual(capture["requested_sensor_to_ego"], _matrix())
            self.assertEqual(capture["sensor_to_ego_observation"], "carla_actor_get_transform")
            self.assertAlmostEqual(capture["observed_sensor_world_transform"][3], 1.0)
            self.assertAlmostEqual(capture["observed_ego_world_transform"][7], 2.0)
            probe.close()
            self.assertTrue(world.sensor.stopped)
            self.assertTrue(world.sensor.destroyed)

    def test_rejects_missing_or_malformed_same_frame_callback(self):
        from runtime.scene0061_carla_lidar_probe import CarlaNativeLidarProbeError

        with tempfile.TemporaryDirectory() as directory:
            probe, world = self._probe(Path(directory))
            world.sensor.callback(_Measurement(12, b"bad"))
            with self.assertRaisesRegex(CarlaNativeLidarProbeError, "did not produce requested frame 12"):
                probe.persist_frame(12)
            with self.assertRaisesRegex(CarlaNativeLidarProbeError, "did not produce requested frame 13"):
                probe.persist_frame(13)

    def test_never_overwrites_capture_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            probe, world = self._probe(Path(directory))
            raw = struct.pack("<4f", 1, 2, 3, .5)
            world.sensor.callback(_Measurement(8, raw))
            probe.persist_frame(8)
            world.sensor.callback(_Measurement(8, raw))
            with self.assertRaises(FileExistsError):
                probe.persist_frame(8)

    def test_fails_closed_when_carla_cannot_observe_actor_transforms(self):
        from runtime.scene0061_carla_lidar_probe import (
            CarlaNativeLidarProbe,
            CarlaNativeLidarProbeError,
        )

        with tempfile.TemporaryDirectory() as directory:
            world = _World()
            probe = CarlaNativeLidarProbe(
                carla_module=_Carla,
                world=world,
                ego_vehicle=object(),
                output_dir=Path(directory),
                sensor_to_ego=_matrix(),
            )
            probe.start()
            world.sensor.transform = _Carla.Transform(
                _Carla.Location(x=0.0, y=0.0, z=0.0),
                _Carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0),
            )
            world.sensor.callback(_Measurement(9, struct.pack("<4f", 1, 2, 3, .5)))
            with self.assertRaisesRegex(CarlaNativeLidarProbeError, "cannot independently observe"):
                probe.persist_frame(9)
            probe.close()


if __name__ == "__main__":
    unittest.main()
