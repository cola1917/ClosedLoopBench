import unittest
from unittest.mock import Mock, patch


class RunnerRegistryTests(unittest.TestCase):
    def test_current_runner_surface_is_classified_and_canonicalized(self):
        from runners.runner_registry import CANONICAL_RUNNERS, runner_inventory

        inventory = runner_inventory()
        self.assertGreaterEqual(inventory["top_level_count"], 75)
        self.assertEqual(inventory["unclassified"], [])
        self.assertEqual(
            inventory["canonical_commands"],
            [spec.command for spec in CANONICAL_RUNNERS],
        )
        self.assertIn("runtime", inventory["groups"])
        self.assertIn("diagnostic", inventory["groups"])

    def test_canonical_dispatch_forwards_module_arguments(self):
        from runners import __main__ as runner_cli

        fake_module = Mock()
        fake_module.main.return_value = 7
        with patch.object(runner_cli, "import_module", return_value=fake_module) as importer:
            result = runner_cli.main(["offline-acceptance", "--output", "result.json"])

        self.assertEqual(result, 7)
        importer.assert_called_once_with("runners.run_offline_acceptance")
        fake_module.main.assert_called_once_with(["--output", "result.json"])


if __name__ == "__main__":
    unittest.main()
