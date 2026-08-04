import types
import unittest


class OfflineAcceptanceTests(unittest.TestCase):
    def test_all_offline_gates_pass_structurally(self):
        from runners.run_offline_acceptance import CORE_MVP_GATES, run_offline_acceptance

        calls = []

        def runner(command, **kwargs):
            calls.append((command, kwargs))
            return types.SimpleNamespace(returncode=0, stdout="ok", stderr="")

        result = run_offline_acceptance(python_executable="python", runner=runner)
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["scope"], "core_mvp")
        self.assertFalse(result["environment_required"])
        self.assertEqual(result["passed_gate_count"], len(CORE_MVP_GATES))
        self.assertEqual(len(calls), len(CORE_MVP_GATES))
        self.assertTrue(all(call[0][1:3] == ["-m", "unittest"] for call in calls))

    def test_extended_scope_is_explicit(self):
        from runners.run_offline_acceptance import (
            CORE_MVP_GATES,
            EXTENDED_OFFLINE_GATES,
            run_offline_acceptance,
        )

        calls = []

        def runner(command, **kwargs):
            calls.append((command, kwargs))
            return types.SimpleNamespace(returncode=0, stdout="ok", stderr="")

        result = run_offline_acceptance(runner=runner, extended=True)
        self.assertEqual(result["scope"], "extended_offline")
        self.assertEqual(
            result["gate_count"], len(CORE_MVP_GATES) + len(EXTENDED_OFFLINE_GATES)
        )
        self.assertEqual(len(calls), result["gate_count"])

    def test_one_failed_gate_fails_the_acceptance_artifact(self):
        from runners.run_offline_acceptance import run_offline_acceptance

        counter = {"value": 0}

        def runner(_command, **_kwargs):
            counter["value"] += 1
            code = 1 if counter["value"] == 2 else 0
            return types.SimpleNamespace(returncode=code, stdout="", stderr="failure")

        result = run_offline_acceptance(runner=runner)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["passed_gate_count"], result["gate_count"] - 1)
        self.assertEqual(result["gates"][1]["stderr_tail"], ["failure"])


if __name__ == "__main__":
    unittest.main()
