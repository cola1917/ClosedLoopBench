import json
import tempfile
import unittest
from pathlib import Path

from tests.test_multimodal_closed_loop_acceptance import _result


class CosmosTransferTests(unittest.TestCase):
    def _inputs(self, root: Path):
        run = root / "accepted.json"
        rgb = root / "rgb.mp4"
        edge = root / "edge.mp4"
        run.write_text(json.dumps(_result()), encoding="utf-8")
        rgb.write_bytes(b"rgb-video")
        edge.write_bytes(b"edge-video")
        return run, rgb, edge

    def test_builds_offline_video_job_outside_the_acceptance_path(self):
        from adapters.cosmos_transfer import build_cosmos_transfer_job
        from adapters.shared_protocol_validation import validate_document

        with tempfile.TemporaryDirectory() as directory:
            run, rgb, edge = self._inputs(Path(directory))
            job = build_cosmos_transfer_job(
                run,
                rgb,
                {"edge": edge},
                prompt="Preserve traffic geometry and improve photorealism.",
                frame_count=93,
                frames_per_sec=16.0,
                width=1280,
                height=720,
            )

        validate_document(job)
        self.assertFalse(job["execution"]["realtime"])
        self.assertFalse(job["execution"]["consumes_usdz"])
        self.assertFalse(job["boundary"]["part_of_sensor_acceptance"])
        self.assertEqual(job["request"]["endpoint"], "/v1/infer")
        self.assertEqual(job["controls"][0]["type"], "edge")

    def test_rejects_usdz_input_or_missing_control_video(self):
        from adapters.cosmos_transfer import CosmosTransferError, build_cosmos_transfer_job

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, rgb, _ = self._inputs(root)
            usdz = root / "last.usdz"
            usdz.write_bytes(b"usdz")
            kwargs = {
                "prompt": "photorealistic",
                "frame_count": 93,
                "frames_per_sec": 16.0,
                "width": 1280,
                "height": 720,
            }
            with self.assertRaisesRegex(CosmosTransferError, "must be .mp4"):
                build_cosmos_transfer_job(run, usdz, {"edge": rgb}, **kwargs)
            with self.assertRaisesRegex(CosmosTransferError, "at least one control"):
                build_cosmos_transfer_job(run, rgb, {}, **kwargs)

    def test_rejects_run_that_did_not_pass_multimodal_acceptance(self):
        from adapters.cosmos_transfer import CosmosTransferError, build_cosmos_transfer_job

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, rgb, edge = self._inputs(root)
            failed = _result()
            failed["report"]["runtime"]["multimodal_sensor"]["status"] = "failed"
            run.write_text(json.dumps(failed), encoding="utf-8")
            with self.assertRaisesRegex(CosmosTransferError, "accepted run is invalid"):
                build_cosmos_transfer_job(
                    run,
                    rgb,
                    {"edge": edge},
                    prompt="photorealistic",
                    frame_count=93,
                    frames_per_sec=16.0,
                    width=1280,
                    height=720,
                )

    def test_pre_submission_verifier_rejects_changed_video(self):
        from adapters.cosmos_transfer import (
            CosmosTransferError,
            build_cosmos_transfer_job,
            verify_cosmos_transfer_job_files,
        )

        with tempfile.TemporaryDirectory() as directory:
            run, rgb, edge = self._inputs(Path(directory))
            job = build_cosmos_transfer_job(
                run,
                rgb,
                {"edge": edge},
                prompt="photorealistic",
                frame_count=93,
                frames_per_sec=16.0,
                width=1280,
                height=720,
            )
            self.assertEqual(verify_cosmos_transfer_job_files(job)["status"], "passed")
            edge.write_bytes(b"changed-after-packaging")
            with self.assertRaisesRegex(CosmosTransferError, "changed after packaging"):
                verify_cosmos_transfer_job_files(job)


if __name__ == "__main__":
    unittest.main()
