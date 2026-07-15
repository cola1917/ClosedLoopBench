import copy
import hashlib
import json
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
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
    schema = json.loads((SCHEMAS / name).read_text(encoding="utf-8"))
    return copy.deepcopy(schema["examples"][0])


def _refs(value):
    if isinstance(value, dict):
        if value.get("schema_version") == "shared_artifact_ref.v1":
            yield value
        for child in value.values():
            yield from _refs(child)
    elif isinstance(value, list):
        for child in value:
            yield from _refs(child)


def _materialize(root, message):
    for index, ref in enumerate(_refs(message)):
        path = root / Path(ref["path"])
        path.parent.mkdir(parents=True, exist_ok=True)
        content = f"artifact-{index}-{ref['role']}\n".encode()
        if path.exists():
            content = path.read_bytes()
        else:
            path.write_bytes(content)
        ref["size_bytes"] = len(content)
        ref["sha256"] = hashlib.sha256(content).hexdigest()
    return message


def _job_request(root):
    message = _example("shared_job_request.schema.json")
    business = _example("reconstruction_request.schema.json")
    reference = message["payload"]["request_artifact"]
    path = root / reference["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    content = (json.dumps(business, indent=2) + "\n").encode()
    path.write_bytes(content)
    reference["size_bytes"] = len(content)
    reference["sha256"] = hashlib.sha256(content).hexdigest()
    return message


class SharedMessageStoreTests(unittest.TestCase):
    def test_publish_load_and_list_schema_valid_message(self):
        from adapters.shared_message_store import list_messages, load_message, publish_message

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            message = _job_request(root)
            target = publish_message(root, message)
            loaded = load_message(root, "shared.job.request", message["message_id"])
            self.assertTrue((target / "READY.json").is_file())
            self.assertEqual(loaded["payload"]["job_kind"], "reconstruction")
            self.assertEqual(list_messages(root, "shared.job.request"), [message["message_id"]])

    def test_invalid_schema_is_rejected_before_publication(self):
        from adapters.shared_message_store import SharedMessageError, publish_message

        with tempfile.TemporaryDirectory() as directory:
            message = _example("scene_selection_request.schema.json")
            message["producer"]["project"] = "ClosedLoopBench"
            with self.assertRaises(SharedMessageError):
                publish_message(Path(directory), message)

    def test_changed_message_and_artifact_are_rejected(self):
        from adapters.shared_message_store import SharedMessageError, load_message, publish_message

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            message = _job_request(root)
            target = publish_message(root, message)
            (target / "message.json").write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(SharedMessageError, "changed after publication"):
                load_message(root, "shared.job.request", message["message_id"])

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            message = _job_request(root)
            publish_message(root, message)
            (root / message["payload"]["request_artifact"]["path"]).write_text(
                "changed", encoding="utf-8"
            )
            with self.assertRaisesRegex(SharedMessageError, "artifact (size|digest) mismatch"):
                load_message(root, "shared.job.request", message["message_id"])

    def test_concurrent_publish_has_one_winner(self):
        from adapters.shared_message_store import SharedMessageExistsError, publish_message

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            message = _job_request(root)

            def publish():
                try:
                    publish_message(root, message)
                    return "published"
                except SharedMessageExistsError:
                    return "exists"

            with ThreadPoolExecutor(max_workers=2) as pool:
                outcomes = list(pool.map(lambda _: publish(), range(2)))
            self.assertCountEqual(outcomes, ["published", "exists"])

    def test_concurrent_claim_has_one_winner_per_attempt(self):
        from adapters.shared_message_store import SharedMessageClaimedError, claim_job, publish_message

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            request = _job_request(root)
            publish_message(root, request)

            def claim(worker):
                envelope = _example("shared_job_claim.schema.json")
                envelope["message_id"] = f"msg-claim-{worker}"
                envelope["producer"]["instance_id"] = worker
                envelope["payload"]["worker_id"] = worker
                envelope["payload"]["lease_id"] = f"lease-{worker}"
                try:
                    claim_job(root, request["message_id"], envelope)
                    return "claimed"
                except SharedMessageClaimedError:
                    return "busy"

            with ThreadPoolExecutor(max_workers=2) as pool:
                outcomes = list(pool.map(claim, ("worker-1", "worker-2")))
            self.assertCountEqual(outcomes, ["claimed", "busy"])

    def test_attempt_limit_and_terminal_result_links_are_enforced(self):
        from adapters.shared_message_store import (
            SharedMessageError,
            claim_job,
            complete_job,
            load_message,
            publish_message,
        )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            request = _job_request(root)
            publish_message(root, request)
            claim = _example("shared_job_claim.schema.json")
            claim_job(root, request["message_id"], claim)

            excessive = copy.deepcopy(claim)
            excessive["message_id"] = "msg-claim-attempt-99"
            excessive["payload"]["attempt"] = 99
            with self.assertRaisesRegex(SharedMessageError, "max_attempts"):
                claim_job(root, request["message_id"], excessive)

            result = _materialize(root, _example("shared_job_result.schema.json"))
            target = complete_job(root, request["message_id"], result)
            self.assertTrue((target / "READY.json").is_file())
            loaded = load_message(root, "shared.job.result", result["message_id"])
            self.assertEqual(loaded["payload"]["status"], "succeeded")

            duplicate = copy.deepcopy(result)
            duplicate["message_id"] = "msg-job-result-duplicate"
            duplicate["idempotency"]["key"] += "/duplicate"
            with self.assertRaisesRegex(Exception, "terminal result"):
                complete_job(root, request["message_id"], duplicate)

    def test_next_attempt_requires_expired_lease_without_terminal_result(self):
        from adapters.shared_message_store import SharedMessageError, claim_job, publish_message

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            request = _job_request(root)
            publish_message(root, request)
            first = _example("shared_job_claim.schema.json")
            claim_job(root, request["message_id"], first)

            second = copy.deepcopy(first)
            second["message_id"] = "msg-job-claim-002"
            second["idempotency"]["key"] = (
                "reconstruction-cc8c0bf57f984915a77078b10eb33198-v001/attempt-2/claim"
            )
            second["payload"]["attempt"] = 2
            second["payload"]["lease_id"] = "lease-reconstruction-002"
            second["payload"]["claimed_at"] = "2026-07-13T12:29:00Z"
            second["payload"]["lease_expires_at"] = "2026-07-13T12:59:00Z"
            with self.assertRaisesRegex(SharedMessageError, "lease has not expired"):
                claim_job(root, request["message_id"], second)

            second["payload"]["claimed_at"] = "2026-07-13T12:31:00Z"
            second["payload"]["lease_expires_at"] = "2026-07-13T13:01:00Z"
            target = claim_job(root, request["message_id"], second)
            self.assertEqual(target.name, "attempt-0002")

    def test_idempotency_key_cannot_name_two_messages(self):
        from adapters.shared_message_store import SharedIdempotencyConflictError, publish_message

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = _example("scene_selection_request.schema.json")
            publish_message(root, first)
            second = copy.deepcopy(first)
            second["message_id"] = "msg-selection-conflict"
            second["correlation"]["root_message_id"] = second["message_id"]
            with self.assertRaises(SharedIdempotencyConflictError):
                publish_message(root, second)

    def test_publish_artifact_is_atomic_and_immutable(self):
        from adapters.shared_message_store import SharedMessageExistsError, publish_artifact

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "local.json"
            source.write_text('{"scene_id":"scene-0061"}\n', encoding="utf-8")
            ref = publish_artifact(
                root,
                source,
                "requests/job-001/request.json",
                role="job_request",
                media_type="application/json",
            )
            self.assertEqual(ref["path"], "requests/job-001/request.json")
            with self.assertRaises(SharedMessageExistsError):
                publish_artifact(
                    root,
                    source,
                    ref["path"],
                    role="job_request",
                    media_type="application/json",
                )


if __name__ == "__main__":
    unittest.main()
