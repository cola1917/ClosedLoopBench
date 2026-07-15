from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import uuid
from pathlib import Path
from typing import Any, Callable, Iterable

from adapters.shared_protocol_validation import (
    SharedProtocolValidationError,
    validate_artifact_reference,
    validate_shared_document,
)


PROTOCOL_VERSION = "shared_exchange_protocol.v1"
MESSAGE_NAME = "message.json"
READY_NAME = "READY.json"
CLAIM_NAME = "claim.json"
_SAFE_SEGMENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class SharedMessageError(ValueError):
    """Raised when a protocol message or artifact is unsafe or inconsistent."""


class SharedMessageExistsError(FileExistsError):
    """Raised when an immutable message or artifact already exists."""


class SharedMessageClaimedError(FileExistsError):
    """Raised when another worker already owns the same job attempt."""


class SharedIdempotencyConflictError(FileExistsError):
    """Raised when one idempotency key is reused for a different message."""


def _safe_segment(value: Any, label: str) -> str:
    normalized = str(value or "")
    if not _SAFE_SEGMENT.fullmatch(normalized) or normalized in {".", ".."}:
        raise SharedMessageError(f"invalid {label}: {normalized!r}")
    return normalized


def _identity(message: dict[str, Any]) -> tuple[str, str]:
    if message.get("protocol_version") != PROTOCOL_VERSION:
        raise SharedMessageError(
            f"unsupported protocol version: {message.get('protocol_version')!r}"
        )
    return (
        _safe_segment(message.get("message_type"), "message type"),
        _safe_segment(message.get("message_id"), "message id"),
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _reserve_record(path: Path, payload: dict[str, Any], conflict_message: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.parent / f".{path.name}.reserving-{uuid.uuid4().hex}"
    try:
        staging.mkdir()
        (staging / "record.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        try:
            os.rename(staging, path)
            return
        except OSError as exc:
            if not path.exists():
                raise
            try:
                existing = json.loads((path / "record.json").read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as read_exc:
                raise SharedMessageError(f"invalid reservation record: {path}") from read_exc
            if existing != payload:
                raise SharedIdempotencyConflictError(conflict_message) from exc
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def _idempotency_path(exchange_root: Path, message: dict[str, Any]) -> Path:
    idempotency = message["idempotency"]
    scope = _safe_segment(idempotency["scope"], "idempotency scope")
    key = str(idempotency["key"])
    key_hash = hashlib.sha256(key.encode("utf-8")).hexdigest()
    return _rooted(exchange_root, "idempotency", scope, key_hash)


def _reserve_idempotency(exchange_root: Path, message: dict[str, Any]) -> None:
    message_type, message_id = _identity(message)
    payload = {
        "protocol_version": PROTOCOL_VERSION,
        "scope": message["idempotency"]["scope"],
        "key": message["idempotency"]["key"],
        "message_type": message_type,
        "message_id": message_id,
    }
    _reserve_record(
        _idempotency_path(exchange_root, message),
        payload,
        f"idempotency key already belongs to another message: {payload['key']}",
    )


def _validate_idempotency(exchange_root: Path, message: dict[str, Any]) -> None:
    path = _idempotency_path(exchange_root, message) / "record.json"
    if not path.is_file():
        raise SharedMessageError("message has no idempotency reservation")
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SharedMessageError("invalid idempotency reservation") from exc
    expected = {
        "protocol_version": PROTOCOL_VERSION,
        "scope": message["idempotency"]["scope"],
        "key": message["idempotency"]["key"],
        "message_type": message["message_type"],
        "message_id": message["message_id"],
    }
    if record != expected:
        raise SharedMessageError("idempotency reservation does not match the message")


def _rooted(exchange_root: Path, *parts: str, create: bool = False) -> Path:
    root = Path(exchange_root).resolve()
    path = root.joinpath(*parts)
    if create:
        path.mkdir(parents=True, exist_ok=True)
    resolved = path.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise SharedMessageError("protocol path escapes the exchange root") from exc
    return resolved


def _contained_artifact(exchange_root: Path, relative: str) -> Path:
    raw = str(relative)
    candidate = Path(raw)
    if (
        not raw
        or "\\" in raw
        or candidate.is_absolute()
        or candidate.drive
        or any(part in {"", ".", ".."} for part in raw.split("/"))
    ):
        raise SharedMessageError(f"artifact path must be relative: {relative!r}")
    return _rooted(exchange_root, *candidate.parts)


def publish_artifact(
    exchange_root: Path,
    source: Path,
    relative_path: str,
    *,
    role: str,
    media_type: str,
    content_schema: str | None = None,
) -> dict[str, Any]:
    """Publish one immutable file atomically and return its content reference."""

    source = Path(source).resolve()
    if not source.is_file() or source.is_symlink():
        raise SharedMessageError(f"artifact source must be a regular file: {source}")
    target = _contained_artifact(exchange_root, relative_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise SharedMessageExistsError(f"artifact already exists: {target}")
    staging = target.parent / f".{target.name}.publishing-{uuid.uuid4().hex}"
    try:
        shutil.copyfile(source, staging)
        try:
            os.rename(staging, target)
        except OSError as exc:
            if target.exists():
                raise SharedMessageExistsError(f"artifact already exists: {target}") from exc
            raise
    finally:
        if staging.exists():
            staging.unlink()
    reference: dict[str, Any] = {
        "schema_version": "shared_artifact_ref.v1",
        "path": Path(relative_path).as_posix(),
        "role": role,
        "media_type": media_type,
        "sha256": _sha256(target),
        "size_bytes": target.stat().st_size,
        "immutable": True,
    }
    if content_schema:
        reference["content_schema"] = content_schema
    validate_artifact_reference(reference)
    return reference


def reference_existing_artifact(
    exchange_root: Path,
    path: Path,
    *,
    role: str,
    media_type: str,
    content_schema: str | None = None,
) -> dict[str, Any]:
    """Create a digest reference for an already finalized file under the exchange root."""

    root = Path(exchange_root).resolve()
    resolved = Path(path).resolve()
    try:
        relative = resolved.relative_to(root).as_posix()
    except ValueError as exc:
        raise SharedMessageError("artifact is outside the exchange root") from exc
    if not resolved.is_file() or resolved.is_symlink():
        raise SharedMessageError(f"artifact must be a regular file: {resolved}")
    reference: dict[str, Any] = {
        "schema_version": "shared_artifact_ref.v1",
        "path": relative,
        "role": role,
        "media_type": media_type,
        "sha256": _sha256(resolved),
        "size_bytes": resolved.stat().st_size,
        "immutable": True,
    }
    if content_schema:
        reference["content_schema"] = content_schema
    validate_artifact_reference(reference)
    return reference


def validate_artifact_on_disk(exchange_root: Path, reference: dict[str, Any]) -> Path:
    validate_artifact_reference(reference)
    path = _contained_artifact(exchange_root, str(reference["path"]))
    if not path.is_file() or path.is_symlink():
        raise SharedMessageError(f"referenced artifact does not exist: {reference['path']}")
    if path.stat().st_size != reference["size_bytes"]:
        raise SharedMessageError(f"artifact size mismatch: {reference['path']}")
    if _sha256(path) != reference["sha256"]:
        raise SharedMessageError(f"artifact digest mismatch: {reference['path']}")
    return path


def _artifact_references(value: Any) -> Iterable[dict[str, Any]]:
    if isinstance(value, dict):
        if value.get("schema_version") == "shared_artifact_ref.v1":
            yield value
        for child in value.values():
            yield from _artifact_references(child)
    elif isinstance(value, list):
        for child in value:
            yield from _artifact_references(child)


def _validate_document(
    exchange_root: Path,
    message: dict[str, Any],
    validate: Callable[[dict[str, Any]], None] | None,
    verify_artifacts: bool,
) -> None:
    try:
        (validate or validate_shared_document)(message)
    except SharedProtocolValidationError as exc:
        raise SharedMessageError(str(exc)) from exc
    if verify_artifacts:
        for reference in _artifact_references(message):
            validate_artifact_on_disk(exchange_root, reference)


def publish_message(
    exchange_root: Path,
    message: dict[str, Any],
    *,
    validate: Callable[[dict[str, Any]], None] | None = None,
    verify_artifacts: bool = True,
) -> Path:
    """Atomically publish one immutable, Schema-valid protocol message."""

    message_type, message_id = _identity(message)
    _validate_document(exchange_root, message, validate, verify_artifacts)
    _reserve_idempotency(exchange_root, message)
    type_root = _rooted(exchange_root, "messages", message_type, create=True)
    target = _rooted(exchange_root, "messages", message_type, message_id)
    if target.exists():
        raise SharedMessageExistsError(f"message already exists: {target}")
    staging = type_root / f".publishing-{message_id}-{uuid.uuid4().hex}"
    try:
        staging.mkdir()
        message_path = staging / MESSAGE_NAME
        message_path.write_text(
            json.dumps(message, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        ready = {
            "protocol_version": PROTOCOL_VERSION,
            "message_type": message_type,
            "message_id": message_id,
            "message": MESSAGE_NAME,
            "message_sha256": _sha256(message_path),
            "size_bytes": message_path.stat().st_size,
        }
        (staging / READY_NAME).write_text(
            json.dumps(ready, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        try:
            os.rename(staging, target)
        except OSError as exc:
            if target.exists():
                raise SharedMessageExistsError(f"message already exists: {target}") from exc
            raise
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return target


def load_message(
    exchange_root: Path,
    message_type: str,
    message_id: str,
    *,
    validate: Callable[[dict[str, Any]], None] | None = None,
    verify_artifacts: bool = True,
) -> dict[str, Any]:
    message_type = _safe_segment(message_type, "message type")
    message_id = _safe_segment(message_id, "message id")
    message_dir = _rooted(exchange_root, "messages", message_type, message_id)
    ready_path = message_dir / READY_NAME
    message_path = message_dir / MESSAGE_NAME
    if not ready_path.is_file() or not message_path.is_file():
        raise SharedMessageError(f"message is not ready: {message_dir}")
    try:
        ready = json.loads(ready_path.read_text(encoding="utf-8"))
        message = json.loads(message_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SharedMessageError(f"invalid published message: {exc}") from exc
    if ready.get("protocol_version") != PROTOCOL_VERSION:
        raise SharedMessageError("READY protocol version mismatch")
    if ready.get("message_type") != message_type or ready.get("message_id") != message_id:
        raise SharedMessageError("READY identity mismatch")
    if ready.get("message_sha256") != _sha256(message_path):
        raise SharedMessageError("message changed after publication")
    if ready.get("size_bytes") != message_path.stat().st_size:
        raise SharedMessageError("message size changed after publication")
    if _identity(message) != (message_type, message_id):
        raise SharedMessageError("message identity does not match its directory")
    _validate_document(exchange_root, message, validate, verify_artifacts)
    _validate_idempotency(exchange_root, message)
    return message


def list_messages(exchange_root: Path, message_type: str) -> list[str]:
    message_type = _safe_segment(message_type, "message type")
    type_root = _rooted(exchange_root, "messages", message_type)
    if not type_root.is_dir():
        return []
    result = []
    for candidate in type_root.iterdir():
        if not candidate.is_dir() or candidate.name.startswith("."):
            continue
        try:
            load_message(exchange_root, message_type, candidate.name)
        except SharedMessageError:
            continue
        result.append(candidate.name)
    return sorted(result)


def claim_job(exchange_root: Path, request_message_id: str, claim: dict[str, Any]) -> Path:
    """Atomically claim one immutable shared.job.request attempt."""

    request = load_message(exchange_root, "shared.job.request", request_message_id)
    _validate_document(exchange_root, claim, None, True)
    if claim.get("schema_version") != "shared_job_claim.v1":
        raise SharedMessageError("claim must use shared_job_claim.v1")
    payload = claim["payload"]
    request_payload = request["payload"]
    business_path = validate_artifact_on_disk(
        exchange_root, request_payload["request_artifact"]
    )
    try:
        business_request = json.loads(business_path.read_text(encoding="utf-8"))
        validate_shared_document(business_request)
    except (OSError, json.JSONDecodeError, SharedProtocolValidationError) as exc:
        raise SharedMessageError(f"invalid business request artifact: {exc}") from exc
    if business_request.get("message_id") != request_payload["request_message_id"]:
        raise SharedMessageError("job request_artifact message ID does not match")
    if request["correlation"].get("causation_message_id") != business_request["message_id"]:
        raise SharedMessageError("job request causation does not match its business request")
    expected_business_schema = {
        "reconstruction": "reconstruction_request.v1",
        "evaluation": "evaluation_run_request.v1",
    }[request_payload["job_kind"]]
    if business_request.get("schema_version") != expected_business_schema:
        raise SharedMessageError("business request type does not match job_kind")
    if payload["request_message_id"] != request_message_id:
        raise SharedMessageError("claim request_message_id does not match the job request")
    for field in ("job_id", "job_kind"):
        if payload[field] != request_payload[field]:
            raise SharedMessageError(f"claim {field} does not match the job request")
    if claim["producer"]["project"] != request_payload["target_project"]:
        raise SharedMessageError("claim producer is not the job target project")
    attempt = int(payload["attempt"])
    if attempt > int(request_payload["max_attempts"]):
        raise SharedMessageError("claim attempt exceeds max_attempts")
    if claim["correlation"]["correlation_id"] != request["correlation"]["correlation_id"]:
        raise SharedMessageError("claim correlation_id does not match the job request")
    if claim["correlation"].get("causation_message_id") != request_message_id:
        raise SharedMessageError("claim causation_message_id does not match the job request")

    if attempt > 1:
        previous = load_job_claim(exchange_root, request_message_id, attempt - 1)
        previous_terminal = _terminal_record_path(
            exchange_root, request_message_id, attempt - 1
        )
        if previous_terminal.is_file():
            terminal_record = json.loads(previous_terminal.read_text(encoding="utf-8"))
            terminal = load_message(
                exchange_root,
                terminal_record["message_type"],
                terminal_record["message_id"],
            )
            retryable = (
                terminal["payload"]["status"] == "failed"
                and bool((terminal["payload"].get("error") or {}).get("retryable"))
            )
            if not retryable:
                raise SharedMessageError("previous attempt is terminal and not retryable")
        else:
            previous_expiry = previous["payload"]["lease_expires_at"]
            current_claimed = payload["claimed_at"]
            from datetime import datetime

            parse = lambda value: datetime.fromisoformat(value.replace("Z", "+00:00"))
            if parse(current_claimed) <= parse(previous_expiry):
                raise SharedMessageError("previous attempt lease has not expired")

    try:
        _reserve_idempotency(exchange_root, claim)
    except SharedIdempotencyConflictError as exc:
        raise SharedMessageClaimedError("job attempt already has a claim identity") from exc

    attempt_name = f"attempt-{attempt:04d}"
    request_claim_root = _rooted(
        exchange_root, "claims", "shared.job.request", request_message_id, create=True
    )
    target = _rooted(
        exchange_root,
        "claims",
        "shared.job.request",
        request_message_id,
        attempt_name,
    )
    if target.exists():
        raise SharedMessageClaimedError(f"job attempt is already claimed: {target}")
    staging = request_claim_root / f".claiming-{attempt_name}-{uuid.uuid4().hex}"
    try:
        staging.mkdir()
        claim_path = staging / CLAIM_NAME
        claim_path.write_text(
            json.dumps(claim, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        (staging / READY_NAME).write_text(
            json.dumps(
                {
                    "protocol_version": PROTOCOL_VERSION,
                    "message_id": claim["message_id"],
                    "message_sha256": _sha256(claim_path),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        try:
            os.rename(staging, target)
        except OSError as exc:
            if target.exists():
                raise SharedMessageClaimedError(
                    f"job attempt is already claimed: {target}"
                ) from exc
            raise
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    return target


def load_job_claim(
    exchange_root: Path, request_message_id: str, attempt: int
) -> dict[str, Any]:
    if int(attempt) < 1:
        raise SharedMessageError("claim attempt must be positive")
    claim_dir = _rooted(
        exchange_root,
        "claims",
        "shared.job.request",
        _safe_segment(request_message_id, "request message id"),
        f"attempt-{int(attempt):04d}",
    )
    claim_path = claim_dir / CLAIM_NAME
    ready_path = claim_dir / READY_NAME
    if not claim_path.is_file() or not ready_path.is_file():
        raise SharedMessageError(f"job claim is not ready: {claim_dir}")
    claim = json.loads(claim_path.read_text(encoding="utf-8"))
    ready = json.loads(ready_path.read_text(encoding="utf-8"))
    if ready.get("message_id") != claim.get("message_id"):
        raise SharedMessageError("claim READY identity mismatch")
    if ready.get("message_sha256") != _sha256(claim_path):
        raise SharedMessageError("claim changed after publication")
    _validate_document(exchange_root, claim, None, True)
    _validate_idempotency(exchange_root, claim)
    return claim


def complete_job(
    exchange_root: Path,
    request_message_id: str,
    result: dict[str, Any],
) -> Path:
    """Validate job/request/claim links and publish one terminal result."""

    request = load_message(exchange_root, "shared.job.request", request_message_id)
    _validate_document(exchange_root, result, None, True)
    if result.get("schema_version") != "shared_job_result.v1":
        raise SharedMessageError("terminal job result must use shared_job_result.v1")
    payload = result["payload"]
    request_payload = request["payload"]
    if payload["request_message_id"] != request_message_id:
        raise SharedMessageError("result request_message_id does not match")
    for field in ("job_id", "job_kind"):
        if payload[field] != request_payload[field]:
            raise SharedMessageError(f"result {field} does not match the job request")
    claim = load_job_claim(exchange_root, request_message_id, int(payload["attempt"]))
    if payload["claim_message_id"] != claim["message_id"]:
        raise SharedMessageError("result claim_message_id does not match the claimed attempt")
    if result["producer"]["project"] != request_payload["target_project"]:
        raise SharedMessageError("result producer is not the target project")
    if result["correlation"]["correlation_id"] != request["correlation"]["correlation_id"]:
        raise SharedMessageError("result correlation_id does not match the job request")
    if result["correlation"].get("causation_message_id") != claim["message_id"]:
        raise SharedMessageError("result causation_message_id does not match the claim")

    terminal_path = _terminal_record_path(
        exchange_root, request_message_id, int(payload["attempt"]), directory=True
    )
    terminal_record = {
        "protocol_version": PROTOCOL_VERSION,
        "request_message_id": request_message_id,
        "attempt": int(payload["attempt"]),
        "message_type": result["message_type"],
        "message_id": result["message_id"],
    }
    _reserve_record(
        terminal_path,
        terminal_record,
        "job attempt already has a different terminal result",
    )
    return publish_message(exchange_root, result)


def _terminal_record_path(
    exchange_root: Path,
    request_message_id: str,
    attempt: int,
    *,
    directory: bool = False,
) -> Path:
    base = _rooted(
        exchange_root,
        "terminals",
        "shared.job.request",
        _safe_segment(request_message_id, "request message id"),
        f"attempt-{int(attempt):04d}",
    )
    return base if directory else base / "record.json"
