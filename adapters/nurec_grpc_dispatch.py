from __future__ import annotations

import hashlib
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Callable, Mapping

from adapters.nurec_multimodal import (
    NuRecMultimodalError,
    build_nurec_multimodal_evidence,
    materialize_nurec_rpc_requests,
    validate_nurec_multimodal_evidence,
)


Encoder = Callable[[Mapping[str, Any]], Mapping[str, Any]]
RpcCall = Callable[[Any], Any]
ResponseBytes = Callable[[Any], bytes]
ResponseInspector = Callable[[Mapping[str, Any], Any, bytes], Mapping[str, Any] | None]


def dispatch_nurec_multimodal_frame(
    frame: Mapping[str, Any],
    *,
    encode_rgb: Encoder,
    encode_lidar: Encoder,
    render_rgb: RpcCall,
    render_lidar: RpcCall,
    response_bytes: ResponseBytes | None = None,
    response_inspector: ResponseInspector | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    max_latency_ms: float | None = None,
    concurrency: int = 1,
    max_attempts: int = 1,
    retry_backoff_sec: float = 0.5,
) -> dict[str, Any]:
    """Dispatch all sensors while preserving the SDK-neutral synchronization gate.

    A version-specific encoder must return ``wire_request`` plus the exact
    ``dynamic_object_sha256`` and ``frame_id`` it encoded. This prevents an SDK
    adapter from silently dropping or re-timing actor poses.

    ``concurrency`` > 1 dispatches the frame's per-sensor requests from a
    thread pool instead of serially. Evidence ordering stays identical to the
    serial path (responses are collected in request order), and the frame-level
    synchronization gate is unchanged: the function still returns only after
    every request has completed. Requires thread-safe encoders/inspectors
    (gRPC channels/stubs are thread-safe; payload materialization writes one
    distinct file per sensor).
    """

    serializer = response_bytes or _default_response_bytes
    payloads = list(materialize_nurec_rpc_requests(frame))

    attempts_allowed = max(1, int(max_attempts))

    def _dispatch_one(payload: Mapping[str, Any]) -> dict[str, Any]:
        encoder = encode_rgb if payload["modality"] == "rgb" else encode_lidar
        rpc = render_rgb if payload["modality"] == "rgb" else render_lidar
        started = monotonic()
        inspection = None
        status = "error"
        payload_sha256 = None
        error = None
        attempts_used = 0
        for attempt in range(1, attempts_allowed + 1):
            attempts_used = attempt
            inspection = None
            try:
                encoded = encoder(payload)
                _validate_encoded_request(payload, encoded)
                response = rpc(encoded["wire_request"])
                body = serializer(response)
                if not isinstance(body, bytes):
                    raise NuRecMultimodalError(
                        "NuRec response serializer must return bytes"
                    )
                if response_inspector is not None:
                    inspection = response_inspector(payload, response, body)
                    if inspection is not None and not isinstance(inspection, Mapping):
                        raise NuRecMultimodalError(
                            "NuRec response inspector must return mapping metadata or None"
                        )
                status = "ok"
                payload_sha256 = hashlib.sha256(body).hexdigest()
                error = None
                break
            except Exception as exc:  # evidence must include partial RPC failure
                status = "error"
                payload_sha256 = None
                error = f"{type(exc).__name__}: {exc}"
                if attempt < attempts_allowed:
                    time.sleep(max(0.0, retry_backoff_sec) * attempt)
        latency_ms = max(0.0, (monotonic() - started) * 1000.0)
        response_record = {
            "request_id": payload["request_id"],
            "status": status,
            "frame_id": payload["frame_id"],
            "dynamic_object_sha256": payload["dynamic_object_sha256"],
            "payload_sha256": payload_sha256,
            "latency_ms": latency_ms,
            "verification_source": "client_encoder_and_rpc_wrapper",
        }
        if attempts_used > 1:
            response_record["attempts"] = attempts_used
        if error is not None:
            response_record["error"] = error
        if inspection is not None:
            response_record["response_metadata"] = dict(inspection)
        return response_record

    workers = max(1, int(concurrency))
    if workers > 1 and len(payloads) > 1:
        with ThreadPoolExecutor(max_workers=min(workers, len(payloads))) as pool:
            responses = list(pool.map(_dispatch_one, payloads))
    else:
        responses = [_dispatch_one(payload) for payload in payloads]
    evidence = build_nurec_multimodal_evidence(
        frame,
        responses,
        max_latency_ms=max_latency_ms,
    )
    evidence["dispatch"] = {
        "sdk_boundary": "injected_version_specific_encoder",
        "dynamic_object_verification": "encoder_echo_checked_before_rpc",
        "response_digest": "sha256_of_serialized_rpc_response",
        "response_validation": (
            "injected_modality_specific_inspector"
            if response_inspector is not None
            else "serialized_bytes_only"
        ),
    }
    validate_nurec_multimodal_evidence(evidence)
    return evidence


def _validate_encoded_request(payload: Mapping[str, Any], encoded: Any) -> None:
    if not isinstance(encoded, Mapping) or "wire_request" not in encoded:
        raise NuRecMultimodalError("NuRec encoder must return wire_request metadata")
    if encoded.get("frame_id") != payload["frame_id"]:
        raise NuRecMultimodalError("NuRec encoder changed frame_id")
    if encoded.get("dynamic_object_sha256") != payload["dynamic_object_sha256"]:
        raise NuRecMultimodalError("NuRec encoder changed or dropped dynamic objects")
    if encoded.get("modality") != payload["modality"]:
        raise NuRecMultimodalError("NuRec encoder changed modality")


def _default_response_bytes(response: Any) -> bytes:
    if isinstance(response, bytes):
        return response
    serializer = getattr(response, "SerializeToString", None)
    if callable(serializer):
        value = serializer()
        if isinstance(value, bytes):
            return value
    raise NuRecMultimodalError(
        "NuRec response must be bytes/protobuf or provide response_bytes"
    )
