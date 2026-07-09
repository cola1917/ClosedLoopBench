import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path


class TcpAdapterCliTests(unittest.TestCase):
    def test_plan_tcp_adapter_cli_writes_runtime_plan(self):
        from runners.plan_tcp_adapter import main

        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "tcp_runtime_plan.json"
            stdout = io.StringIO()

            with contextlib.redirect_stdout(stdout):
                exit_code = main([
                    "--scenario-id",
                    "scene-tcp-cli",
                    "--runtime-path",
                    "E:/models/TCP",
                    "--checkpoint-path",
                    "E:/models/TCP/tcp.pth",
                    "--output",
                    str(output),
                ])

            self.assertEqual(exit_code, 0)
            cli_result = json.loads(stdout.getvalue())
            self.assertEqual(cli_result["status"], "planned")
            self.assertEqual(cli_result["tcp_runtime_plan"], str(output))
            plan = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(plan["scenario_id"], "scene-tcp-cli")
            self.assertEqual(plan["plugin"], "external_ros2_tcp")
            self.assertEqual(plan["runtime"]["repo_path"], "E:/models/TCP")
            self.assertEqual(plan["io"]["inputs"]["sensor_profile"]["required_cameras"], ["rgb_front"])


if __name__ == "__main__":
    unittest.main()
