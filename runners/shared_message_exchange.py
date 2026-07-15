from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from adapters.shared_message_store import (
    claim_job,
    complete_job,
    list_messages,
    load_job_claim,
    load_message,
    publish_artifact,
    publish_message,
)


def _read(path: str) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Operate shared_exchange_protocol.v1.")
    parser.add_argument("--exchange-root", required=True)
    commands = parser.add_subparsers(dest="command", required=True)
    publish = commands.add_parser("publish-message")
    publish.add_argument("--message", required=True)
    read = commands.add_parser("read-message")
    read.add_argument("--message-type", required=True)
    read.add_argument("--message-id", required=True)
    listing = commands.add_parser("list-messages")
    listing.add_argument("--message-type", required=True)
    artifact = commands.add_parser("publish-artifact")
    artifact.add_argument("--source", required=True)
    artifact.add_argument("--path", required=True)
    artifact.add_argument("--role", required=True)
    artifact.add_argument("--media-type", required=True)
    artifact.add_argument("--content-schema")
    claim = commands.add_parser("claim-job")
    claim.add_argument("--request-message-id", required=True)
    claim.add_argument("--claim", required=True)
    read_claim = commands.add_parser("read-claim")
    read_claim.add_argument("--request-message-id", required=True)
    read_claim.add_argument("--attempt", required=True, type=int)
    complete = commands.add_parser("complete-job")
    complete.add_argument("--request-message-id", required=True)
    complete.add_argument("--result", required=True)
    args = parser.parse_args(argv)
    root = Path(args.exchange_root)

    if args.command == "publish-message":
        payload = {"status": "published", "path": str(publish_message(root, _read(args.message)))}
    elif args.command == "read-message":
        payload = load_message(root, args.message_type, args.message_id)
    elif args.command == "list-messages":
        payload = {"message_type": args.message_type, "message_ids": list_messages(root, args.message_type)}
    elif args.command == "publish-artifact":
        payload = publish_artifact(
            root,
            Path(args.source),
            args.path,
            role=args.role,
            media_type=args.media_type,
            content_schema=args.content_schema,
        )
    elif args.command == "claim-job":
        payload = {
            "status": "claimed",
            "path": str(claim_job(root, args.request_message_id, _read(args.claim))),
        }
    elif args.command == "read-claim":
        payload = load_job_claim(root, args.request_message_id, args.attempt)
    else:
        payload = {
            "status": "completed",
            "path": str(complete_job(root, args.request_message_id, _read(args.result))),
        }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
