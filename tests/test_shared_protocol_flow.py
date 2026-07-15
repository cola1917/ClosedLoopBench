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


def _example(name):
    return json.loads((SCHEMAS / f"{name}.schema.json").read_text(encoding="utf-8"))[
        "examples"
    ][0]


class SharedProtocolFlowTests(unittest.TestCase):
    def test_scene_token_golden_chain_is_correlated_and_causal(self):
        from adapters.shared_protocol_validation import validate_shared_document

        names = (
            "scene_selection_request",
            "reconstruction_request",
            "reconstruction_result",
            "evaluation_run_request",
            "evaluation_run_result",
        )
        chain = [_example(name) for name in names]
        for message in chain:
            validate_shared_document(message)
            self.assertEqual(message["protocol_version"], "shared_exchange_protocol.v1")
            self.assertEqual(
                message["correlation"]["correlation_id"],
                "flow-cc8c0bf57f984915a77078b10eb33198",
            )
            self.assertEqual(message["correlation"]["root_message_id"], chain[0]["message_id"])

        for previous, current in zip(chain, chain[1:]):
            self.assertEqual(
                current["correlation"]["causation_message_id"], previous["message_id"]
            )

    def test_reconstruction_and_scene_package_ownership_survive_the_chain(self):
        reconstruction = _example("reconstruction_result")
        request = _example("evaluation_run_request")
        result = _example("evaluation_run_result")

        reconstruction_package = next(
            artifact
            for artifact in reconstruction["payload"]["artifacts"]
            if artifact["role"] == "reconstruction_package"
        )
        self.assertNotEqual(
            reconstruction_package["path"],
            request["payload"]["scene_package"]["path"],
        )
        self.assertFalse(
            [
                artifact
                for artifact in reconstruction["payload"]["artifacts"]
                if artifact["role"] == "scene_package"
            ]
        )
        self.assertEqual(request["payload"]["scene_id"], reconstruction["payload"]["scene_id"])
        self.assertEqual(
            request["payload"]["scene_version"], reconstruction["payload"]["scene_version"]
        )
        self.assertEqual(request["payload"]["scene_package"]["role"], "scene_package")
        self.assertEqual(result["payload"]["run_id"], request["payload"]["run_id"])
        self.assertEqual(
            result["payload"]["algorithm"]["algorithm_id"],
            request["payload"]["algorithm"]["algorithm_id"],
        )
        self.assertEqual(result["payload"]["odd"]["odd_id"], request["payload"]["odd"]["odd_id"])


if __name__ == "__main__":
    unittest.main()
