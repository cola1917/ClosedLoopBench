import unittest
from pathlib import Path
from unittest.mock import patch


class EsminiToolingContractTests(unittest.TestCase):
    def test_finds_esmini_from_environment_variable_first(self):
        from tools.esmini import find_esmini

        with patch.dict("os.environ", {"ESMINI_BIN": "C:/tools/esmini/esmini.exe"}, clear=True):
            self.assertEqual(find_esmini(), Path("C:/tools/esmini/esmini.exe"))

    def test_returns_none_when_esmini_is_missing(self):
        from tools.esmini import find_esmini

        with (
            patch.dict("os.environ", {}, clear=True),
            patch("pathlib.Path.exists", return_value=False),
            patch("shutil.which", return_value=None),
        ):
            self.assertIsNone(find_esmini())

    def test_smoke_command_is_headless_by_default(self):
        from runners.esmini_smoke import build_esmini_command

        command = build_esmini_command(Path("tools/esmini/bin/esmini.exe"), Path("outputs/scene/scenario.xosc"))
        normalized_command = [item.replace("\\", "/") for item in command]

        self.assertIn("--headless", command)
        self.assertIn("--osc", command)
        self.assertIn(Path("outputs/scene/scenario.xosc").as_posix(), normalized_command)


if __name__ == "__main__":
    unittest.main()
