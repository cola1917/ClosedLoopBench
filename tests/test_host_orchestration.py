import json
import hashlib
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch


class HostOrchestrationTests(unittest.TestCase):
    def test_prepare_host_run_connects_ready_scene_to_native_carla_plan(self):
        from runners.run_host_closed_loop import prepare_host_run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scene_ir = root / "scene_ir.json"
            scene_ir.write_text("{}", encoding="utf-8")
            resolved = {
                "scene_id": "scene-0061",
                "version": "v001",
                "scene_package": str(root / "scene_package.json"),
                "scene_ir": str(scene_ir),
                "nurec_usdz": None,
                "nurec_checkpoint": None,
            }

            def write_config(_source, output, **kwargs):
                output.write_text(json.dumps({"scenario_id": "scene-0061"}), encoding="utf-8")
                self.assertEqual(kwargs["algorithm_id"], "tcp")

            def write_ego(_config, output, **kwargs):
                output.write_text(json.dumps({"scenario_id": "scene-0061"}), encoding="utf-8")
                self.assertEqual(kwargs["ego_driver"], "ros2_control")

            with patch(
                "runners.run_host_closed_loop.consume_scene_version", return_value=resolved
            ), patch(
                "runners.run_host_closed_loop.write_carla_run_config", side_effect=write_config
            ), patch(
                "runners.run_host_closed_loop.write_basic_agent_plan", side_effect=write_ego
            ):
                plan = prepare_host_run(
                    exchange_root=root / "shared",
                    scene_id="scene-0061",
                    version="v001",
                    run_dir=root / "run",
                    algorithm_id="tcp",
                    algorithm_version="commit-123",
                    ego_driver="ros2_control",
                )

            self.assertEqual(plan["carla"]["deployment"], "native_host")
            self.assertEqual(plan["carla"]["expected_version"], "0.9.16")
            self.assertEqual(plan["algorithm"]["id"], "tcp")
            self.assertTrue(Path(plan["artifacts"]["host_plan"]).is_file())

    def test_evaluation_protocol_request_maps_losslessly_to_host_run(self):
        from runners.run_host_closed_loop import prepare_host_run_from_evaluation_request

        schema_path = (
            Path(__file__).parents[2]
            / "SceneExchangeContracts"
            / "src"
            / "scene_exchange_contracts"
            / "schemas"
            / "shared_exchange_protocol"
            / "evaluation_run_request.schema.json"
        )
        request = json.loads(schema_path.read_text(encoding="utf-8"))["examples"][0]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scene_package = root / request["payload"]["scene_package"]["path"]
            scene_package.parent.mkdir(parents=True)
            content = b'{"schema_version":"closed_loop_scene_package.v1"}\n'
            scene_package.write_bytes(content)
            request["payload"]["scene_package"]["size_bytes"] = len(content)
            request["payload"]["scene_package"]["sha256"] = hashlib.sha256(content).hexdigest()

            with patch("runners.run_host_closed_loop.prepare_host_run") as prepare:
                prepare.return_value = {"status": "planned"}
                result = prepare_host_run_from_evaluation_request(
                    request,
                    exchange_root=root,
                )

            self.assertEqual(result, {"status": "planned"})
            kwargs = prepare.call_args.kwargs
            self.assertEqual(kwargs["run_id"], request["payload"]["run_id"])
            self.assertEqual(
                kwargs["scene_id"],
                "cc8c0bf57f984915a77078b10eb33198",
            )
            self.assertEqual(kwargs["scene_version"] if "scene_version" in kwargs else kwargs["version"], "v001")
            self.assertEqual(kwargs["odd_id"], "clear-day")
            self.assertEqual(kwargs["seed"], 42)
            self.assertEqual(kwargs["actor_control_mode"], "traffic_manager")
            self.assertEqual(kwargs["fixed_delta_seconds"], 0.05)

    def test_external_algorithm_requires_matching_live_ready_file(self):
        from runtime.host_orchestration import validate_host_runtime

        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / "algorithm.ready.json"
            ready.write_text(
                json.dumps({
                    "status": "ready",
                    "algorithm_id": "tcp",
                    "heartbeat_unix": time.time(),
                }),
                encoding="utf-8",
            )
            plan = {
                "carla": {
                    "host": "127.0.0.1",
                    "port": 2000,
                    "expected_version": "0.9.16",
                },
                "algorithm": {
                    "id": "tcp",
                    "driver": "ros2_control",
                    "ready_file": str(ready),
                },
            }
            result = validate_host_runtime(
                plan,
                probe=lambda _config: {
                    "status": "available",
                    "carla_version": "0.9.16",
                },
            )
            self.assertEqual(result["status"], "ready")

    def test_stale_algorithm_heartbeat_fails_before_execution(self):
        from runtime.host_orchestration import HostRuntimeError, validate_host_runtime

        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / "algorithm.ready.json"
            ready.write_text(
                json.dumps({
                    "status": "ready",
                    "algorithm_id": "tcp",
                    "heartbeat_unix": 10.0,
                }),
                encoding="utf-8",
            )
            plan = {
                "carla": {
                    "host": "127.0.0.1",
                    "port": 2000,
                    "expected_version": "0.9.16",
                },
                "algorithm": {
                    "id": "tcp",
                    "driver": "ros2_control",
                    "ready_file": str(ready),
                },
            }
            with self.assertRaisesRegex(HostRuntimeError, "heartbeat is stale"):
                validate_host_runtime(
                    plan,
                    probe=lambda _config: {
                        "status": "available",
                        "carla_version": "0.9.16",
                    },
                    clock=lambda: 100.0,
                )

    def test_carla_version_mismatch_fails_before_execution(self):
        from runtime.host_orchestration import HostRuntimeError, validate_host_runtime

        plan = {
            "carla": {
                "host": "127.0.0.1",
                "port": 2000,
                "expected_version": "0.9.16",
            },
            "algorithm": {"id": "basic_agent", "driver": "basic_agent"},
        }
        with self.assertRaisesRegex(HostRuntimeError, "version mismatch"):
            validate_host_runtime(
                plan,
                probe=lambda _config: {
                    "status": "available",
                    "carla_version": "0.9.15",
                },
            )

    def test_basic_agent_does_not_require_algorithm_container(self):
        from runtime.host_orchestration import validate_host_runtime

        result = validate_host_runtime(
            {
                "carla": {
                    "host": "localhost",
                    "port": 2000,
                    "expected_version": "0.9.16",
                },
                "algorithm": {"id": "basic_agent", "driver": "basic_agent"},
            },
            probe=lambda _config: {
                "status": "available",
                "carla_version": "0.9.16",
            },
        )
        self.assertEqual(result["algorithm_id"], "basic_agent")


if __name__ == "__main__":
    unittest.main()
