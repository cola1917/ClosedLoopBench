from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class Scene0061EntrypointGuardTests(unittest.TestCase):
    def test_nurec_replay_requires_route_map_and_source_audits(self):
        script = (ROOT / "tools" / "run_scene0061_nurec_replay.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--require-route-map-integration", script)
        self.assertIn("--require-route-source-audit", script)
        self.assertIn("--expected-sha256", script)
        self.assertIn("--expected-ego-corridor-count 0", script)

    def test_pure_pursuit_default_requires_route_map_and_source_audits(self):
        script = (ROOT / "tools" / "remote_run_pure_pursuit.sh").read_text(
            encoding="utf-8"
        )
        default_branch = script.split("if test \"$ALLOW_CORRIDOR_ONLY_XODR\"", 1)[1]
        default_branch = default_branch.split("fi\nPYTHONPATH", 1)[0]
        self.assertIn("--require-route-map-integration", default_branch)
        self.assertIn("--require-route-source-audit", default_branch)
        self.assertIn("--expected-sha256", default_branch)
        self.assertIn("--expected-ego-corridor-count 0", default_branch)
        self.assertIn("--no-map-topology", script)

    def test_carla_runtime_audit_requires_the_same_topology_contract(self):
        source = (ROOT / "tools" / "audit_carla_xodr_runtime.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("require_route_map_integration=True", source)
        self.assertIn("require_route_source_audit=True", source)
        self.assertIn("require_connector_evidence=True", source)


if __name__ == "__main__":
    unittest.main()
