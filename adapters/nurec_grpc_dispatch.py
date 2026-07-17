from __future__ import annotations

import hashlib
import time
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


def dispatch_nurec_multimodal_frame(
    frame: Mapping[str, Any],
    *,
    encode_rgb: Encoder,
    encode_lidar: Encoder,
    render_rgb: RpcCall,
    render_lidar: RpcCall,
    response_bytes: ResponseBytes | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    max_latency_ms: float | None = None,
) -> dict[str, Any]:
    """Dispatch all sensors while preserving the SDK-neutral synchronization gate.

    A version-specific encoder must return ``wire_request`` plus the exact
    ``dynamic_object_sha256`` and ``frame_id`` it encoded. This prevents an SDK
    adapter from silently dropping or re-timing actor poses.
    """

    serializer = response_bytes or _default_response_bytes
    responses = []
    for payload in materialize_nurec_rpc_requests(frame):
        encoder = encode_rgb if payload["modality"] == "rgb" else encode_lidar
        rpc = render_rgb if payload["modality"] == "rgb" else render_lidar
        started = monotonic()
        try:
            encoded = encoder(payload)
            _validate_encoded_request(payload, encoded)
            response = rpc(encoded["wire_request"])
            body = serializer(response)
            if not isinstance(body, bytes):
                raise NuRecMultimodalError("NuRec response serializer must return bytes")
            status = "ok"
            payload_sha256 = hashlib.sha256(body).hexdigest()
            error = None
        except Exception as exc:  # evidence must include partial RPC failure
            status = "error"
            payload_sha256 = None
            error = f"{type(exc).__name__}: {exc}"
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
        if error is not None:
            response_record["error"] = error
        responses.append(response_record)
    evidence = build_nurec_multimodal_evidence(
        frame,
        responses,
        max_latency_ms=max_latency_ms,
    )
    evidence["dispatch"] = {
        "sdk_boundary": "injected_version_specific_encoder",
        "dynamic_object_verification": "encoder_echo_checked_before_rpc",
        "response_digest": "sha256_of_serialized_rpc_response",
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
