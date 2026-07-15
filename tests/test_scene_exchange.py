import json
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


SCENE_TOKEN = "cc8c0bf57f984915a77078b10eb33198"


def _coordinate_frame():
    return {
        "name": "scene_local_ego_start",
        "units": {"position": "meter", "time": "second", "yaw": "degree"},
        "handedness": "right",
        "x_axis": "initial_ego_forward",
        "y_axis": "initial_ego_left",
        "origin_global_translation": [0.0, 0.0, 0.0],
        "origin_global_rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
        "origin_global_yaw_deg": 0.0,
        "transform": "local_xy = R(-origin_yaw) * (global_xy - origin_xy)",
    }


def _write_bundle(root: Path, scene_id: str = SCENE_TOKEN) -> Path:
    bundle = root / "bundle"
    bundle.mkdir()
    (bundle / "scene_ir.json").write_text("{}\n", encoding="utf-8")
    (bundle / "road.xodr").write_text("<OpenDRIVE/>\n", encoding="utf-8")
    (bundle / "scenario.xosc").write_text("<OpenSCENARIO/>\n", encoding="utf-8")
    package = {
        "schema_version": "closed_loop_scene_package.v1",
        "scene_id": scene_id,
        "source": {"dataset": "nuscenes", "scene_id": scene_id, "scene_token": scene_id},
        "coordinate_frame": _coordinate_frame(),
        "motion": {"scene_ir": "scene_ir.json"},
        "map": {
            "location": "singapore-onenorth",
            "opendrive": "road.xodr",
            "source": "nuscenes_map_expansion",
        },
        "scenario": {"openscenario": "scenario.xosc"},
        "visual": {
            "nurec_usdz": None,
            "nurec_checkpoint": None,
            "reconstruction_package": None,
        },
        "alignment": {
            "sim_from_log_transform": [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
            "matrix_layout": "row_major_4x4",
            "source_frame": "nuscenes_global",
            "target_frame": "scene_local_ego_start",
            "status": "log_to_sim_defined",
        },
    }
    (bundle / "scene_package.json").write_text(json.dumps(package), encoding="utf-8")
    return bundle


class SceneExchangeTests(unittest.TestCase):
    def test_readable_scene_name_is_rejected_as_package_identity(self):
        from adapters.scene_exchange import SceneExchangeError, validate_scene_package

        with tempfile.TemporaryDirectory() as directory:
            bundle = _write_bundle(Path(directory), "scene-0061")
            with self.assertRaisesRegex(SceneExchangeError, "required pattern"):
                validate_scene_package(bundle)

    def test_publish_can_derive_token_from_package(self):
        from adapters.scene_exchange import publish_scene_version

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = publish_scene_version(
                _write_bundle(root),
                root / "shared",
                None,
                "v001",
            )

            self.assertEqual(
                target,
                root / "shared" / "scenes" / SCENE_TOKEN / "v001",
            )

    def test_publish_and_consume_ready_version(self):
        from adapters.scene_exchange import consume_scene_version, publish_scene_version

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = _write_bundle(root)
            target = publish_scene_version(bundle, root / "shared", SCENE_TOKEN, "v001")
            result = consume_scene_version(root / "shared", SCENE_TOKEN, "v001")

            self.assertEqual(target, root / "shared" / "scenes" / SCENE_TOKEN / "v001")
            self.assertTrue((target / "READY.json").is_file())
            self.assertEqual(result["version"], "v001")
            self.assertEqual(Path(result["scene_ir"]), target / "scene_ir.json")

    def test_unready_version_is_not_listed_or_consumed(self):
        from adapters.scene_exchange import (
            SceneExchangeError,
            consume_scene_version,
            list_ready_versions,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = _write_bundle(root)
            target = root / "shared" / "scenes" / SCENE_TOKEN / "v001"
            target.parent.mkdir(parents=True)
            target.mkdir()
            for source in bundle.iterdir():
                (target / source.name).write_bytes(source.read_bytes())

            self.assertEqual(list_ready_versions(root / "shared", SCENE_TOKEN), [])
            with self.assertRaisesRegex(SceneExchangeError, "not ready"):
                consume_scene_version(root / "shared", SCENE_TOKEN, "v001")

    def test_manifest_path_traversal_is_rejected(self):
        from adapters.scene_exchange import SceneExchangeError, validate_scene_package

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = _write_bundle(root)
            package_path = bundle / "scene_package.json"
            package = json.loads(package_path.read_text(encoding="utf-8"))
            package["motion"]["scene_ir"] = "../outside.json"
            package_path.write_text(json.dumps(package), encoding="utf-8")
            (root / "outside.json").write_text("{}", encoding="utf-8")

            with self.assertRaisesRegex(SceneExchangeError, "contained relative path"):
                validate_scene_package(bundle)

    def test_duplicate_publish_never_overwrites_immutable_version(self):
        from adapters.scene_exchange import SceneVersionExistsError, publish_scene_version

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = _write_bundle(root)
            shared = root / "shared"
            publish_scene_version(bundle, shared, SCENE_TOKEN, "v001")
            original = (shared / "scenes" / SCENE_TOKEN / "v001" / "scene_ir.json").read_bytes()
            (bundle / "scene_ir.json").write_text('{"changed": true}\n', encoding="utf-8")

            with self.assertRaises(SceneVersionExistsError):
                publish_scene_version(bundle, shared, SCENE_TOKEN, "v001")
            self.assertEqual(
                (shared / "scenes" / SCENE_TOKEN / "v001" / "scene_ir.json").read_bytes(),
                original,
            )

    def test_concurrent_publish_has_exactly_one_winner(self):
        from adapters.scene_exchange import SceneVersionExistsError, publish_scene_version

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = _write_bundle(root)
            shared = root / "shared"

            def publish():
                try:
                    publish_scene_version(bundle, shared, SCENE_TOKEN, "v001")
                    return "published"
                except SceneVersionExistsError:
                    return "exists"

            with ThreadPoolExecutor(max_workers=2) as pool:
                outcomes = list(pool.map(lambda _: publish(), range(2)))

            self.assertCountEqual(outcomes, ["published", "exists"])
            self.assertFalse(
                list((shared / "scenes" / SCENE_TOKEN).glob(".publishing-*"))
            )

    def test_modified_manifest_after_publish_is_rejected(self):
        from adapters.scene_exchange import SceneExchangeError, consume_scene_version, publish_scene_version

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = publish_scene_version(
                _write_bundle(root), root / "shared", SCENE_TOKEN, "v001"
            )
            (target / "scene_package.json").write_text("{}\n", encoding="utf-8")

            with self.assertRaisesRegex(SceneExchangeError, "changed after publication"):
                consume_scene_version(root / "shared", SCENE_TOKEN, "v001")

    def test_modified_artifact_after_publish_is_rejected(self):
        from adapters.scene_exchange import SceneExchangeError, consume_scene_version, publish_scene_version

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = publish_scene_version(
                _write_bundle(root), root / "shared", SCENE_TOKEN, "v001"
            )
            (target / "road.xodr").write_text("<changed/>", encoding="utf-8")

            with self.assertRaisesRegex(SceneExchangeError, "artifacts changed"):
                consume_scene_version(root / "shared", SCENE_TOKEN, "v001")


if __name__ == "__main__":
    unittest.main()
