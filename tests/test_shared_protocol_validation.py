import copy
import json
import unittest
from pathlib import Path


SCHEMAS = (
    Path(__file__).parents[2]
    / "SceneExchangeContracts"
    / "src"
    / "scene_exchange_contracts"
    / "schemas"
    / "shared_exchange_protocol"
)


class SharedProtocolValidationTests(unittest.TestCase):
    def test_every_schema_example_passes_production_validator(self):
        from adapters.shared_protocol_validation import validate_shared_document

        for path in SCHEMAS.glob("*.schema.json"):
            schema = json.loads(path.read_text(encoding="utf-8"))
            for example in schema.get("examples", []):
                with self.subTest(schema=path.name):
                    validate_shared_document(example)

    def test_path_traversal_and_wrong_owner_fail(self):
        from adapters.shared_protocol_validation import SharedProtocolValidationError, validate_shared_document

        schema = json.loads(
            (SCHEMAS / "evaluation_run_request.schema.json").read_text(encoding="utf-8")
        )
        message = copy.deepcopy(schema["examples"][0])
        message["payload"]["scene_package"]["path"] = "../scene_package.json"
        with self.assertRaises(SharedProtocolValidationError):
            validate_shared_document(message)

        message = copy.deepcopy(schema["examples"][0])
        message["producer"]["project"] = "NeuralSceneBridge"
        with self.assertRaises(SharedProtocolValidationError):
            validate_shared_document(message)

    def test_failed_result_requires_structured_error(self):
        from adapters.shared_protocol_validation import SharedProtocolValidationError, validate_shared_document

        schema = json.loads(
            (SCHEMAS / "evaluation_run_result.schema.json").read_text(encoding="utf-8")
        )
        result = copy.deepcopy(schema["examples"][0])
        result["payload"]["status"] = "failed"
        result["payload"].pop("error", None)
        with self.assertRaises(SharedProtocolValidationError):
            validate_shared_document(result)

    def test_impossible_time_order_and_window_are_rejected(self):
        from adapters.shared_protocol_validation import SharedProtocolValidationError, validate_shared_document

        reconstruction = json.loads(
            (SCHEMAS / "reconstruction_request.schema.json").read_text(encoding="utf-8")
        )["examples"][0]
        reconstruction = copy.deepcopy(reconstruction)
        reconstruction["payload"]["reconstruction_window"] = {
            "start_sec": 10.0,
            "end_sec": 2.0,
        }
        with self.assertRaisesRegex(SharedProtocolValidationError, "greater than"):
            validate_shared_document(reconstruction)

        claim = json.loads(
            (SCHEMAS / "shared_job_claim.schema.json").read_text(encoding="utf-8")
        )["examples"][0]
        claim = copy.deepcopy(claim)
        claim["payload"]["lease_expires_at"] = claim["payload"]["claimed_at"]
        with self.assertRaisesRegex(SharedProtocolValidationError, "later than"):
            validate_shared_document(claim)


if __name__ == "__main__":
    unittest.main()
