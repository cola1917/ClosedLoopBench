"""Compatibility import for the canonical SceneExchangeContracts validator."""

from __future__ import annotations

import sys
from pathlib import Path


_CONTRACT_SRC = Path(__file__).resolve().parents[2] / "SceneExchangeContracts" / "src"
if str(_CONTRACT_SRC) not in sys.path:
    sys.path.insert(0, str(_CONTRACT_SRC))

from scene_exchange_contracts.validation import (  # noqa: E402,F401
    ContractValidationError,
    SharedProtocolValidationError,
    schema_digest,
    schema_path,
    validate_artifact_reference,
    validate_document,
    validate_shared_document,
)

__all__ = [
    "ContractValidationError",
    "SharedProtocolValidationError",
    "schema_digest",
    "schema_path",
    "validate_artifact_reference",
    "validate_document",
    "validate_shared_document",
]
