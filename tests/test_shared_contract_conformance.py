import unittest


class SharedContractConformanceTests(unittest.TestCase):
    def test_uses_canonical_contract_suite(self):
        from adapters.shared_protocol_validation import schema_digest
        from scene_exchange_contracts.conformance import run_conformance_suite

        report = run_conformance_suite()
        self.assertEqual(report["schema_count"], 14)
        self.assertEqual(report["digests"]["scenario_ir.v1"], schema_digest("scenario_ir.v1"))


if __name__ == "__main__":
    unittest.main()
