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

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.stopped = True

    def destroy(self):
        self.destroyed = True


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
        probe = CarlaNativeLidarProbe(
            carla_module=_Carla,
            world=world,
            ego_vehicle=object(),
            output_dir=root,
            sensor_to_ego=_matrix(),
        )
        probe.start()
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


if __name__ == "__main__":
    unittest.main()
