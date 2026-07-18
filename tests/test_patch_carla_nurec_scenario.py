from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT_ROOT / "tools" / "patch_carla_nurec_scenario.py"


class PatchNurecScenarioTest(unittest.TestCase):
    def test_accepts_v4_pedestrian_label(self) -> None:
        """The vendored CARLA loader maps v4 ``pedestrian`` tracks to walkers."""

        source = """
                reference_poly=calibration[\"camera_model\"][\"parameters\"][\"reference_poly\"],
                pixeldist_to_angle_poly=calibration[\"camera_model\"][\"parameters\"][\"pixeldist_to_angle_poly\"],
                angle_to_pixeldist_poly=calibration[\"camera_model\"][\"parameters\"][\"angle_to_pixeldist_poly\"],
                max_angle=calibration[\"camera_model\"][\"parameters\"][\"max_angle\"],
                linear_cde=calibration[\"camera_model\"][\"parameters\"][\"linear_cde\"],
        self.tracks = Tracks(
            track_data, self.metadata[\"pose-range\"][\"start-timestamp_us\"]
        )
            if not (track.label in VEHICLE_LABELS or track.label == \"person\"):
                continue

            best_fit_blueprint = self.blueprint_library.get_best_fit_blueprint(
                track.dims, track.label != \"person\"
            )
"""
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "nurec_integration.py"
            target.write_text(source, encoding="utf-8")

            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(target)],
                cwd=PROJECT_ROOT,
                check=True,
                capture_output=True,
                text=True,
            )

            patched = target.read_text(encoding="utf-8")
        self.assertIn("patched:", result.stdout)
        self.assertIn('track.label in {"person", "pedestrian"}', patched)
        self.assertIn(
            "get_best_fit_blueprint(\n                track.dims, is_vehicle_track",
            patched,
        )
        self.assertIn(
            "reference_poly=calibration[\"camera_model\"][\"parameters\"].get",
            patched,
        )
        self.assertIn('self.metadata["pose-range"] = {', patched)
