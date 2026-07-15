import copy
import json
import re
import unittest
from datetime import datetime
from pathlib import Path
from typing import Any


SCHEMA_ROOT = (
    Path(__file__).resolve().parents[2]
    / "SceneExchangeContracts"
    / "src"
    / "scene_exchange_contracts"
    / "schemas"
    / "shared_exchange_protocol"
)
SCHEMA_FILES = {
    "shared_envelope.schema.json",
    "shared_artifact_ref.schema.json",
    "shared_job_request.schema.json",
    "shared_job_claim.schema.json",
    "shared_job_result.schema.json",
    "scene_selection_request.schema.json",
    "reconstruction_request.schema.json",
    "reconstruction_result.schema.json",
    "evaluation_run_request.schema.json",
    "evaluation_run_result.schema.json",
}


class SchemaViolation(AssertionError):
    pass


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _resolve_ref(current_path: Path, ref: str) -> tuple[Path, dict[str, Any]]:
    file_ref, separator, fragment = ref.partition("#")
    target_path = (current_path.parent / file_ref).resolve() if file_ref else current_path.resolve()
    try:
        target_path.relative_to(SCHEMA_ROOT.resolve())
    except ValueError as error:
        raise SchemaViolation(f"reference leaves schema directory: {ref}") from error
    if not target_path.is_file():
        raise SchemaViolation(f"missing schema reference: {ref}")

    target: Any = _load(target_path)
    if separator and fragment:
        if not fragment.startswith("/"):
            raise SchemaViolation(f"unsupported non-pointer fragment: {ref}")
        for encoded_token in fragment[1:].split("/"):
            token = encoded_token.replace("~1", "/").replace("~0", "~")
            if not isinstance(target, dict) or token not in target:
                raise SchemaViolation(f"missing JSON pointer in reference: {ref}")
            target = target[token]
    if not isinstance(target, dict):
        raise SchemaViolation(f"reference does not resolve to a schema object: {ref}")
    return target_path, target


def _is_type(value: Any, expected: str) -> bool:
    if expected == "null":
        return value is None
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected == "string":
        return isinstance(value, str)
    if expected == "array":
        return isinstance(value, list)
    if expected == "object":
        return isinstance(value, dict)
    raise SchemaViolation(f"test validator does not support type {expected!r}")


def _validate(instance: Any, schema: dict[str, Any], current_path: Path, location: str = "$") -> None:
    if "$ref" in schema:
        ref_path, ref_schema = _resolve_ref(current_path, schema["$ref"])
        _validate(instance, ref_schema, ref_path, location)

    for child in schema.get("allOf", []):
        _validate(instance, child, current_path, location)

    condition = schema.get("if")
    if isinstance(condition, dict):
        try:
            _validate(instance, condition, current_path, location)
        except SchemaViolation:
            branch = schema.get("else")
        else:
            branch = schema.get("then")
        if isinstance(branch, dict):
            _validate(instance, branch, current_path, location)

    if "const" in schema and instance != schema["const"]:
        raise SchemaViolation(f"{location} must equal {schema['const']!r}")
    if "enum" in schema and instance not in schema["enum"]:
        raise SchemaViolation(f"{location} is not in {schema['enum']!r}")

    expected_types = schema.get("type")
    if expected_types is not None:
        if isinstance(expected_types, str):
            expected_types = [expected_types]
        if not any(_is_type(instance, expected) for expected in expected_types):
            raise SchemaViolation(f"{location} has the wrong type")

    if isinstance(instance, dict):
        for name in schema.get("required", []):
            if name not in instance:
                raise SchemaViolation(f"{location}.{name} is required")
        properties = schema.get("properties", {})
        for name, child in properties.items():
            if name in instance:
                _validate(instance[name], child, current_path, f"{location}.{name}")
        extras = set(instance) - set(properties)
        additional = schema.get("additionalProperties", True)
        if additional is False and extras:
            raise SchemaViolation(f"{location} has unexpected fields: {sorted(extras)}")
        if isinstance(additional, dict):
            for name in extras:
                _validate(instance[name], additional, current_path, f"{location}.{name}")

    if isinstance(instance, list):
        if len(instance) < schema.get("minItems", 0):
            raise SchemaViolation(f"{location} has too few items")
        if "maxItems" in schema and len(instance) > schema["maxItems"]:
            raise SchemaViolation(f"{location} has too many items")
        if schema.get("uniqueItems"):
            encoded = [json.dumps(item, sort_keys=True, separators=(",", ":")) for item in instance]
            if len(encoded) != len(set(encoded)):
                raise SchemaViolation(f"{location} has duplicate items")
        item_schema = schema.get("items")
        if isinstance(item_schema, dict):
            for index, item in enumerate(instance):
                _validate(item, item_schema, current_path, f"{location}[{index}]")

    if isinstance(instance, str):
        if len(instance) < schema.get("minLength", 0):
            raise SchemaViolation(f"{location} is too short")
        if "maxLength" in schema and len(instance) > schema["maxLength"]:
            raise SchemaViolation(f"{location} is too long")
        if "pattern" in schema and re.search(schema["pattern"], instance) is None:
            raise SchemaViolation(f"{location} does not match {schema['pattern']!r}")
        if schema.get("format") == "date-time":
            try:
                parsed = datetime.fromisoformat(instance.replace("Z", "+00:00"))
            except ValueError as error:
                raise SchemaViolation(f"{location} is not an ISO date-time") from error
            if parsed.tzinfo is None:
                raise SchemaViolation(f"{location} must include a timezone")

    if isinstance(instance, (int, float)) and not isinstance(instance, bool):
        if "minimum" in schema and instance < schema["minimum"]:
            raise SchemaViolation(f"{location} is below minimum")
        if "maximum" in schema and instance > schema["maximum"]:
            raise SchemaViolation(f"{location} is above maximum")
        if "exclusiveMinimum" in schema and instance <= schema["exclusiveMinimum"]:
            raise SchemaViolation(f"{location} is not above exclusiveMinimum")


def _iter_refs(value: Any):
    if isinstance(value, dict):
        if "$ref" in value:
            yield value["$ref"]
        for child in value.values():
            yield from _iter_refs(child)
    elif isinstance(value, list):
        for child in value:
            yield from _iter_refs(child)


def _payload_schema(schema: dict[str, Any]) -> dict[str, Any]:
    return schema["allOf"][1]["properties"]["payload"]


class SharedProtocolSchemaTests(unittest.TestCase):
    def test_schema_inventory_versions_and_references_are_closed(self):
        self.assertEqual({path.name for path in SCHEMA_ROOT.glob("*.json")}, SCHEMA_FILES)
        expected_versions = {
            "shared_artifact_ref.schema.json": "shared_artifact_ref.v1",
            "shared_job_request.schema.json": "shared_job_request.v1",
            "shared_job_claim.schema.json": "shared_job_claim.v1",
            "shared_job_result.schema.json": "shared_job_result.v1",
            "scene_selection_request.schema.json": "scene_selection_request.v1",
            "reconstruction_request.schema.json": "reconstruction_request.v1",
            "reconstruction_result.schema.json": "reconstruction_result.v1",
            "evaluation_run_request.schema.json": "evaluation_run_request.v1",
            "evaluation_run_result.schema.json": "evaluation_run_result.v1",
        }

        ids: set[str] = set()
        for name in sorted(SCHEMA_FILES):
            path = SCHEMA_ROOT / name
            schema = _load(path)
            self.assertEqual(schema["$schema"], "https://json-schema.org/draft/2020-12/schema")
            self.assertTrue(schema["$id"].startswith("https://scene-exchange-contracts.local/"))
            self.assertNotIn(schema["$id"], ids)
            ids.add(schema["$id"])
            for ref in _iter_refs(schema):
                _resolve_ref(path, ref)

            if name in expected_versions:
                if name == "shared_artifact_ref.schema.json":
                    version = schema["properties"]["schema_version"]["const"]
                else:
                    version = schema["allOf"][1]["properties"]["schema_version"]["const"]
                self.assertEqual(version, expected_versions[name])

    def test_envelope_requires_producer_correlation_idempotency_and_error_contract(self):
        schema = _load(SCHEMA_ROOT / "shared_envelope.schema.json")
        self.assertEqual(
            schema["properties"]["protocol_version"]["const"],
            "shared_exchange_protocol.v1",
        )
        self.assertTrue(
            {
                "protocol_version",
                "schema_version",
                "message_id",
                "message_type",
                "created_at",
                "producer",
                "correlation",
                "idempotency",
                "payload",
            }.issubset(schema["required"])
        )
        self.assertEqual(
            schema["$defs"]["producer"]["properties"]["project"]["enum"],
            ["TriggerEngine", "NeuralSceneBridge", "ClosedLoopBench"],
        )
        self.assertEqual(
            set(schema["$defs"]["correlation"]["required"]),
            {"correlation_id", "root_message_id"},
        )
        self.assertEqual(set(schema["$defs"]["idempotency"]["required"]), {"key", "scope"})
        self.assertEqual(
            set(schema["$defs"]["error"]["required"]),
            {"code", "message", "retryable"},
        )
        safe_id_pattern = re.compile(schema["$defs"]["safe_id"]["pattern"])
        self.assertIsNone(safe_id_pattern.search("message:windows-unsafe"))

    def test_examples_satisfy_the_dependency_free_contract_validator(self):
        for name in sorted(SCHEMA_FILES - {"shared_envelope.schema.json"}):
            path = SCHEMA_ROOT / name
            schema = _load(path)
            self.assertTrue(schema.get("examples"), name)
            for example in schema["examples"]:
                _validate(example, schema, path)

    def test_minimal_scene_id_only_selection_is_supported(self):
        path = SCHEMA_ROOT / "scene_selection_request.schema.json"
        schema = _load(path)
        message = copy.deepcopy(schema["examples"][0])
        message["payload"] = {
            "scene_id": "cc8c0bf57f984915a77078b10eb33198",
            "source": {"dataset": "nuscenes"},
        }
        _validate(message, schema, path)

    def test_shared_paths_are_portable_relative_paths(self):
        schema = _load(SCHEMA_ROOT / "shared_artifact_ref.schema.json")
        pattern = re.compile(schema["$defs"]["relative_path"]["pattern"])
        for valid in [
            "scenes/cc8c0bf57f984915a77078b10eb33198/v001/scene_package.json",
            "runs/run_001/report.json",
            ".staging/job-001/request.json",
        ]:
            self.assertIsNotNone(pattern.search(valid), valid)
        for invalid in [
            "",
            "/absolute/report.json",
            "C:/sim-data/report.json",
            "C:report.json",
            "../outside.json",
            "scenes/../outside.json",
            "scenes\\scene-0061\\report.json",
            "scenes//report.json",
            ".",
            "scenes/./report.json",
        ]:
            self.assertIsNone(pattern.search(invalid), invalid)

    def test_failure_results_require_structured_error(self):
        for name in [
            "shared_job_result.schema.json",
            "reconstruction_result.schema.json",
            "evaluation_run_result.schema.json",
        ]:
            path = SCHEMA_ROOT / name
            schema = _load(path)
            message = copy.deepcopy(schema["examples"][0])
            message["payload"]["status"] = "failed"
            message["payload"].pop("error", None)
            with self.assertRaises(SchemaViolation, msg=name):
                _validate(message, schema, path)
            message["payload"]["error"] = {
                "code": "RUNTIME_FAILED",
                "message": "Worker exited before producing a complete result.",
                "retryable": True,
            }
            _validate(message, schema, path)

    def test_status_enums_are_explicit_and_match_current_runtime(self):
        reconstruction = _payload_schema(
            _load(SCHEMA_ROOT / "reconstruction_result.schema.json")
        )
        evaluation = _payload_schema(
            _load(SCHEMA_ROOT / "evaluation_run_result.schema.json")
        )
        self.assertEqual(
            reconstruction["properties"]["status"]["enum"],
            ["succeeded", "partial", "failed", "cancelled"],
        )
        self.assertEqual(
            evaluation["properties"]["runtime_status"]["enum"],
            [
                "not_run",
                "planned",
                "completed",
                "ego_closed_loop",
                "interactive_closed_loop",
                "failed",
            ],
        )

    def test_business_producer_ownership_is_enforced(self):
        cases = [
            ("scene_selection_request.schema.json", "NeuralSceneBridge"),
            ("reconstruction_result.schema.json", "TriggerEngine"),
            ("evaluation_run_result.schema.json", "NeuralSceneBridge"),
        ]
        for name, invalid_project in cases:
            path = SCHEMA_ROOT / name
            schema = _load(path)
            message = copy.deepcopy(schema["examples"][0])
            message["producer"]["project"] = invalid_project
            with self.assertRaises(SchemaViolation, msg=name):
                _validate(message, schema, path)


if __name__ == "__main__":
    unittest.main()
