import hashlib
import itertools
import json
import struct
import tempfile
import unittest
from pathlib import Path


IDENTITY = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]


def _ref(path: Path, frame_id: int) -> dict:
    return {
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "byte_count": path.stat().st_size,
        "encoding": "float32_xyzi_little_endian",
        "carla_frame_id": frame_id,
    }


class Scene0061LiDARAxisCollectorTests(unittest.TestCase):
    def _inputs(self, root: Path):
        frame_id = 81
        points = [
            (1.0, 0.0, 0.0, 0.5),
            (0.0, 2.0, 0.0, 0.5),
            (0.0, 0.0, 3.0, 0.5),
            (2.0, 2.0, 2.0, 0.5),
        ]
        raw = b"".join(struct.pack("<4f", *point) for point in points)
        nurec_path = root / "nurec.xyzi"
        capture_points_path = root / "carla-native.xyzi"
        nurec_path.write_bytes(raw)
        capture_points_path.write_bytes(raw)
        capture = {
            "schema_version": "scene0061_carla_native_lidar_capture.v1",
            "status": "passed",
            "carla_frame_id": frame_id,
            "coordinate_frame": "carla_sensor",
            "sensor_to_ego_observation": "carla_actor_get_transform",
            "sensor_to_ego": IDENTITY,
            "observed_sensor_world_transform": IDENTITY,
            "observed_ego_world_transform": IDENTITY,
            "raw_xyzi_ref": _ref(capture_points_path, frame_id),
        }
        capture_path = root / "carla-native.json"
        capture_path.write_text(json.dumps(capture), encoding="utf-8")
        native_scan = root / "native-scan.json"
        native_scan.write_text("{\"scan\": 0}", encoding="utf-8")
        run_config = {
            "experiment": {"scene_id": "a" * 32, "artifact_sha256": "b" * 64},
            "nurec_runtime": {
                "runtime_scene_id": "scene-0061",
                "lidar_specs": [{"sensor_id": "lidar_top", "model": "AT128", "sensor_to_ego": IDENTITY}],
            },
        }
        nurec_evidence = {
            "frame_id": frame_id,
            "records": [{
                "modality": "lidar", "sensor_id": "lidar_top", "status": "passed",
                "response_metadata": {
                    "point_count": len(points),
                    "materialized_payload": {**_ref(nurec_path, frame_id), "coordinate_frame": "unverified"},
                },
            }],
            "dispatch": {"temporal_alignment": {"native_scan_index": 0}},
        }
        return run_config, nurec_evidence, {"world_tick_frame": frame_id}, capture_path, native_scan

    def test_collects_only_hash_bound_same_frame_native_carla_anchors(self):
        from runtime.scene0061_lidar_axis_collector import collect_lidar_axis_evidence
        from runtime.scene0061_lidar_axis_gate import validate_lidar_axis_evidence

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            evidence = collect_lidar_axis_evidence(
                run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                native_capture_path=values[3], native_scan_manifest_path=values[4],
            )
            self.assertEqual(evidence["axis_validation"]["evidence_source"], "scene0061_carla_native_lidar_capture.v1")
            self.assertGreaterEqual(len(evidence["axis_validation"]["anchors"]), 4)
            result = validate_lidar_axis_evidence(
                evidence, sensor_to_ego=IDENTITY, live_render_lidar=evidence["live_render_lidar"]
            )
            self.assertEqual(result["status"], "passed")

    def test_rejects_capture_from_another_frame_before_writing_a_claim(self):
        from runtime.scene0061_lidar_axis_collector import (
            LiDARAxisCollectionError,
            collect_lidar_axis_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            capture = json.loads(values[3].read_text(encoding="utf-8"))
            capture["carla_frame_id"] = 82
            values[3].write_text(json.dumps(capture), encoding="utf-8")
            with self.assertRaisesRegex(LiDARAxisCollectionError, "same-frame"):
                collect_lidar_axis_evidence(
                    run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                    native_capture_path=values[3], native_scan_manifest_path=values[4],
                )

    def test_gate_rejects_anchor_not_present_in_independent_capture(self):
        from runtime.scene0061_lidar_axis_collector import collect_lidar_axis_evidence
        from runtime.scene0061_lidar_axis_gate import LiDARAxisEvidenceError, validate_lidar_axis_evidence

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            evidence = collect_lidar_axis_evidence(
                run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                native_capture_path=values[3], native_scan_manifest_path=values[4],
            )
            evidence["axis_validation"]["anchors"][0]["carla_ego_point_m"][0] += 0.01
            with self.assertRaisesRegex(LiDARAxisEvidenceError, "independent CARLA LiDAR capture"):
                validate_lidar_axis_evidence(
                    evidence, sensor_to_ego=IDENTITY, live_render_lidar=evidence["live_render_lidar"]
                )

    def test_rejects_capture_relative_pose_that_does_not_rederive_from_carla_observations(self):
        from runtime.scene0061_lidar_axis_collector import (
            LiDARAxisCollectionError,
            collect_lidar_axis_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            capture = json.loads(values[3].read_text(encoding="utf-8"))
            capture["sensor_to_ego"][3] = 1.0
            values[3].write_text(json.dumps(capture), encoding="utf-8")
            with self.assertRaisesRegex(
                LiDARAxisCollectionError,
                "does not match observed actor transforms",
            ):
                collect_lidar_axis_evidence(
                    run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                    native_capture_path=values[3], native_scan_manifest_path=values[4],
                )

    def test_inherits_payload_frame_from_verified_nurec_trace_when_not_repeated(self):
        from runtime.scene0061_lidar_axis_collector import collect_lidar_axis_evidence

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            payload = values[1]["records"][0]["response_metadata"]["materialized_payload"]
            payload.pop("carla_frame_id")
            evidence = collect_lidar_axis_evidence(
                run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                native_capture_path=values[3], native_scan_manifest_path=values[4],
            )
            self.assertEqual(
                evidence["axis_validation"]["payload_ref"]["carla_frame_id"],
                values[1]["frame_id"],
            )

    def test_rejects_explicit_payload_frame_that_conflicts_with_verified_trace(self):
        from runtime.scene0061_lidar_axis_collector import (
            LiDARAxisCollectionError,
            collect_lidar_axis_evidence,
        )

        with tempfile.TemporaryDirectory() as directory:
            values = self._inputs(Path(directory))
            values[1]["records"][0]["response_metadata"]["materialized_payload"][
                "carla_frame_id"
            ] = values[1]["frame_id"] + 1
            with self.assertRaisesRegex(LiDARAxisCollectionError, "wrong frame or encoding"):
                collect_lidar_axis_evidence(
                    run_config=values[0], nurec_evidence=values[1], frame_trace=values[2],
                    native_capture_path=values[3], native_scan_manifest_path=values[4],
                )

    def test_anchor_selection_handles_large_irrelevant_cloud_deterministically(self):
        from runtime.scene0061_lidar_axis_collector import _select_anchors

        # A real CARLA callback can be much larger than a unit-test fixture.
        # The matching anchors are deliberately placed after thousands of
        # distractors to exercise the spatial lookup without changing the
        # deterministic source/capture-index bindings.
        identity = IDENTITY
        distractors = [(100.0 + index, 100.0, 100.0, 1.0) for index in range(10_000)]
        anchors = [
            (1.0, 0.0, 0.0, 0.5),
            (0.0, 2.0, 0.0, 0.5),
            (0.0, 0.0, 3.0, 0.5),
            (2.0, 2.0, 2.0, 0.5),
        ]
        selected = _select_anchors(
            nurec_points=anchors,
            nurec_to_ego=identity,
            capture_points=distractors + anchors,
            capture_to_ego=identity,
            tolerance_m=0.05,
        )
        self.assertEqual([row["source_point_index"] for row in selected], [0, 1, 2, 3])
        self.assertEqual(
            [row["independent_capture_point_index"] for row in selected],
            [10_000, 10_001, 10_002, 10_003],
        )

    def test_rejects_every_alternate_proper_signed_axis_permutation(self):
        """A cloud encoded in any other right-handed axis basis cannot pass.

        Proper signed permutations cover all 24 orientation-preserving ways to
        relabel and flip the three sensor axes.  The identity is the declared
        CARLA sensor basis; each of the other 23 must fail before an axis claim
        is written, even though the points are otherwise real, same-frame, and
        hash-bound.
        """
        from runtime.scene0061_lidar_axis_collector import (
            LiDARAxisCollectionError,
            collect_lidar_axis_evidence,
        )

        native_points = [
            (1.25, 2.5, 4.75, 0.1),
            (6.5, 10.25, 15.75, 0.2),
            (21.5, 28.25, 36.75, 0.3),
            (45.5, 55.25, 66.75, 0.4),
        ]
        for permutation in itertools.permutations(range(3)):
            inversions = sum(
                permutation[left] > permutation[right]
                for left in range(3)
                for right in range(left + 1, 3)
            )
            parity = 1 if inversions % 2 == 0 else -1
            for signs in itertools.product((-1.0, 1.0), repeat=3):
                if parity * signs[0] * signs[1] * signs[2] != 1.0:
                    continue
                if permutation == (0, 1, 2) and signs == (1.0, 1.0, 1.0):
                    continue
                with self.subTest(permutation=permutation, signs=signs), tempfile.TemporaryDirectory() as directory:
                    values = self._inputs(Path(directory))
                    nurec_path = Path(
                        values[1]["records"][0]["response_metadata"]["materialized_payload"]["path"]
                    )
                    capture = json.loads(values[3].read_text(encoding="utf-8"))
                    capture_path = Path(capture["raw_xyzi_ref"]["path"])
                    capture_path.write_bytes(
                        b"".join(struct.pack("<4f", *point) for point in native_points)
                    )
                    capture["raw_xyzi_ref"] = _ref(capture_path, values[1]["frame_id"])
                    values[3].write_text(json.dumps(capture), encoding="utf-8")
                    alternate = [
                        tuple(signs[index] * point[permutation[index]] for index in range(3))
                        + (point[3],)
                        for point in native_points
                    ]
                    nurec_path.write_bytes(
                        b"".join(struct.pack("<4f", *point) for point in alternate)
                    )
                    metadata = values[1]["records"][0]["response_metadata"]
                    metadata["point_count"] = len(alternate)
                    metadata["materialized_payload"] = {
                        **_ref(nurec_path, values[1]["frame_id"]),
                        "coordinate_frame": "unverified",
                    }
                    with self.assertRaisesRegex(
                        LiDARAxisCollectionError,
                        "fewer than four non-coplanar",
                    ):
                        collect_lidar_axis_evidence(
                            run_config=values[0],
                            nurec_evidence=values[1],
                            frame_trace=values[2],
                            native_capture_path=values[3],
                            native_scan_manifest_path=values[4],
                        )


if __name__ == "__main__":
    unittest.main()
