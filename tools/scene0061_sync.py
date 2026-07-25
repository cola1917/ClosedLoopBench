"""Auditable Git synchronization helpers for scene-0061 runtime work.

This tool deliberately does not infer cross-machine synchronization from a
remote-tracking branch.  A runtime is valid only when the intended commit and
the remote checkout resolve to the same object ID.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any, Sequence


class Scene0061SyncError(RuntimeError):
    """Raised when a bundle or an exact commit check cannot be verified."""


def _run(command: Sequence[str], *, cwd: Path | None = None) -> str:
    try:
        completed = subprocess.run(
            list(command),
            cwd=str(cwd) if cwd is not None else None,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        detail = (
            exc.stderr.strip()
            if isinstance(exc, subprocess.CalledProcessError) and exc.stderr
            else str(exc)
        )
        raise Scene0061SyncError(f"command failed: {' '.join(command)}: {detail}") from exc
    return completed.stdout.strip()


def git_sha(repo: Path, revision: str = "HEAD") -> str:
    repo = Path(repo).resolve()
    sha = _run(("git", "rev-parse", "--verify", f"{revision}^{{commit}}"), cwd=repo)
    if len(sha) != 40 or any(character not in "0123456789abcdef" for character in sha):
        raise Scene0061SyncError(f"git returned an invalid commit SHA for {revision}: {sha}")
    return sha


def remote_git_sha(
    remote_host: str, remote_repo: str, *, ssh_port: int = 22, ssh_bin: str = "ssh"
) -> str:
    if not remote_host or not remote_repo:
        raise Scene0061SyncError("remote host and repository are required")
    # Arguments remain discrete until SSH passes the fixed git invocation; no
    # shell interpolation is used for the local process.
    return _run(
        (
            ssh_bin,
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=10",
            "-p",
            str(int(ssh_port)),
            remote_host,
            "git",
            "-C",
            remote_repo,
            "rev-parse",
            "--verify",
            "HEAD^{commit}",
        )
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _bundle_heads(repo: Path, bundle: Path) -> list[dict[str, str]]:
    lines = _run(("git", "bundle", "list-heads", str(bundle)), cwd=repo).splitlines()
    heads = []
    for line in lines:
        parts = line.split(maxsplit=1)
        if len(parts) != 2:
            raise Scene0061SyncError(f"invalid bundle head record: {line}")
        heads.append({"commit": parts[0], "ref": parts[1]})
    if not heads:
        raise Scene0061SyncError("bundle has no heads")
    return heads


def verify_bundle(repo: Path, bundle: Path, *, expected_head: str | None = None) -> dict[str, Any]:
    repo = Path(repo).resolve()
    bundle = Path(bundle).resolve()
    if not bundle.is_file():
        raise Scene0061SyncError(f"bundle does not exist: {bundle}")
    _run(("git", "bundle", "verify", str(bundle)), cwd=repo)
    heads = _bundle_heads(repo, bundle)
    if expected_head is not None and expected_head not in {item["commit"] for item in heads}:
        raise Scene0061SyncError(
            f"bundle does not advertise expected commit {expected_head}"
        )
    return {
        "schema_version": "scene0061_git_bundle.v1",
        "bundle_path": str(bundle),
        "bundle_sha256": _sha256(bundle),
        "heads": heads,
        "expected_head": expected_head,
        "verified": True,
    }


def create_bundle(
    repo: Path, base: str, output: Path, *, manifest: Path | None = None
) -> dict[str, Any]:
    repo = Path(repo).resolve()
    output = Path(output).resolve()
    if output.exists():
        raise Scene0061SyncError(
            f"refusing to overwrite existing bundle: {output}; choose a new output path"
        )
    base_sha = git_sha(repo, base)
    head_sha = git_sha(repo)
    _run(("git", "merge-base", "--is-ancestor", base_sha, head_sha), cwd=repo)
    output.parent.mkdir(parents=True, exist_ok=True)
    # `git bundle create` expects a positive ref plus an explicit exclusion;
    # the range notation alone asks it to advertise no refs and produces an
    # empty bundle.  The remote is required to have `base_sha`, while the
    # advertised HEAD makes the fast-forward target unambiguous.
    _run(
        ("git", "bundle", "create", str(output), f"{base_sha}..HEAD"),
        cwd=repo,
    )
    result = verify_bundle(repo, output, expected_head=head_sha)
    result.update({"base_commit": base_sha, "head_commit": head_sha})
    manifest_path = (
        Path(manifest).resolve()
        if manifest is not None
        else output.with_suffix(output.suffix + ".manifest.json")
    )
    if manifest_path.exists():
        raise Scene0061SyncError(
            f"refusing to overwrite existing bundle manifest: {manifest_path}"
        )
    manifest_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    result["manifest_path"] = str(manifest_path)
    return result


def compare_commits(
    repo: Path, remote_host: str, remote_repo: str, *, ssh_port: int = 22
) -> dict[str, Any]:
    local = git_sha(repo)
    remote = remote_git_sha(remote_host, remote_repo, ssh_port=ssh_port)
    return {
        "schema_version": "scene0061_sync_status.v1",
        "local_head": local,
        "remote_head": remote,
        "equal": local == remote,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    status = commands.add_parser("status", help="Compare exact local and remote HEAD commits.")
    status.add_argument("--repo", type=Path, required=True)
    status.add_argument("--remote-host", required=True)
    status.add_argument("--remote-repo", required=True)
    status.add_argument("--ssh-port", type=int, default=22)
    status.add_argument("--require-equal", action="store_true")
    make = commands.add_parser("make-bundle", help="Create a verified fast-forward bundle.")
    make.add_argument("--repo", type=Path, required=True)
    make.add_argument("--base", required=True, help="Commit already present on the remote.")
    make.add_argument("--output", type=Path, required=True)
    make.add_argument("--manifest", type=Path)
    verify = commands.add_parser("verify-bundle", help="Verify bundle integrity and advertised HEAD.")
    verify.add_argument("--repo", type=Path, required=True)
    verify.add_argument("--bundle", type=Path, required=True)
    verify.add_argument("--expected-head")
    args = parser.parse_args(argv)
    try:
        if args.command == "status":
            result = compare_commits(
                args.repo, args.remote_host, args.remote_repo, ssh_port=args.ssh_port
            )
            if args.require_equal and not result["equal"]:
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 2
        elif args.command == "make-bundle":
            result = create_bundle(args.repo, args.base, args.output, manifest=args.manifest)
        else:
            result = verify_bundle(args.repo, args.bundle, expected_head=args.expected_head)
    except Scene0061SyncError as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
