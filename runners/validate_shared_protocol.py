from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.shared_message_store import validate_artifact_on_disk
from adapters.shared_protocol_validation import validate_shared_document


def _references(value):
    if isinstance(value, dict):
        if value.get("schema_version") == "shared_artifact_ref.v1":
            yield value
        for child in value.values():
            yield from _references(child)
    elif isinstance(value, list):
        for child in value:
            yield from _references(child)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Validate a shared_exchange_protocol.v1 JSON document.")
    parser.add_argument("--document", required=True)
    parser.add_argument(
        "--exchange-root",
        help="When provided, also verify every referenced artifact size and SHA-256.",
    )
    args = parser.parse_args(argv)
    document = json.loads(Path(args.document).read_text(encoding="utf-8"))
    validate_shared_document(document)
    references = list(_references(document))
    if args.exchange_root:
        for reference in references:
            validate_artifact_on_disk(Path(args.exchange_root), reference)
    print(
        json.dumps(
            {
                "status": "valid",
                "schema_version": document["schema_version"],
                "message_id": document.get("message_id"),
                "artifact_references": len(references),
                "artifacts_verified": bool(args.exchange_root),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
