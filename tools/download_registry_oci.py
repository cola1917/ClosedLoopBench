#!/usr/bin/env python3
"""Download a registry image into a verified OCI image layout.

This is intended for hosts where the Docker Hub registry is unreachable but an
HTTP registry proxy is available. Large blobs are fetched with parallel Range
requests, resumed from part files, and verified against their registry digest.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any


MANIFEST_ACCEPT = ", ".join(
    (
        "application/vnd.oci.image.manifest.v1+json",
        "application/vnd.docker.distribution.manifest.v2+json",
        "application/vnd.oci.image.index.v1+json",
        "application/vnd.docker.distribution.manifest.list.v2+json",
    )
)
TOKEN_RE = re.compile(r'(\w+)="([^"]*)"')
PRINT_LOCK = threading.Lock()


class RegistryTokenProvider:
    """Share and periodically refresh a registry bearer token across workers."""

    def __init__(self, registry: str, repository: str) -> None:
        self.registry = registry
        self.repository = repository
        self._lock = threading.Lock()
        self._token: str | None = None
        self._issued_at = 0.0

    def get(self, *, force: bool = False) -> str | None:
        # docker.1ms.run currently issues 25-minute tokens. Refresh at 20
        # minutes so a reconnect during a long layer download stays valid.
        with self._lock:
            if force or self._token is None or time.monotonic() - self._issued_at >= 1200:
                self._token = registry_token(self.registry, self.repository)
                self._issued_at = time.monotonic()
                log("refreshed registry bearer token")
            return self._token


def log(message: str) -> None:
    with PRINT_LOCK:
        print(message, flush=True)


def request_bytes(url: str, headers: dict[str, str] | None = None) -> tuple[bytes, Any]:
    request = urllib.request.Request(url, headers=headers or {})
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return response.read(), response.headers
    except urllib.error.HTTPError as exc:
        if exc.code != 401:
            raise
        challenge = exc.headers.get("WWW-Authenticate", "")
        if not challenge.lower().startswith("bearer "):
            raise
        fields = dict(TOKEN_RE.findall(challenge[7:]))
        realm = fields.pop("realm")
        token_url = realm + "?" + urllib.parse.urlencode(fields)
        token_payload, _ = request_bytes(token_url)
        token_data = json.loads(token_payload)
        token = token_data.get("token") or token_data.get("access_token")
        if not token:
            raise RuntimeError("registry token response did not contain a token")
        authorized = dict(headers or {})
        authorized["Authorization"] = f"Bearer {token}"
        with urllib.request.urlopen(
            urllib.request.Request(url, headers=authorized), timeout=30
        ) as response:
            return response.read(), response.headers


def registry_token(registry: str, repository: str) -> str | None:
    url = f"https://{registry}/v2/{repository}/manifests/latest"
    request = urllib.request.Request(url, headers={"Accept": MANIFEST_ACCEPT})
    try:
        urllib.request.urlopen(request, timeout=20).close()
        return None
    except urllib.error.HTTPError as exc:
        if exc.code != 401:
            return None
        challenge = exc.headers.get("WWW-Authenticate", "")
        if not challenge.lower().startswith("bearer "):
            return None
        fields = dict(TOKEN_RE.findall(challenge[7:]))
        realm = fields.pop("realm")
        fields["scope"] = f"repository:{repository}:pull"
        payload, _ = request_bytes(realm + "?" + urllib.parse.urlencode(fields))
        data = json.loads(payload)
        return data.get("token") or data.get("access_token")


def authenticated_bytes(url: str, token: str | None, accept: str | None = None) -> bytes:
    headers: dict[str, str] = {}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    if accept:
        headers["Accept"] = accept
    payload, _ = request_bytes(url, headers)
    return payload


def digest_path(layout: Path, digest: str) -> Path:
    algorithm, value = digest.split(":", 1)
    if algorithm != "sha256":
        raise ValueError(f"unsupported digest algorithm: {algorithm}")
    return layout / "blobs" / algorithm / value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fetch_range(
    *,
    url: str,
    token_provider: RegistryTokenProvider,
    part: Path,
    start: int,
    end: int,
) -> None:
    expected = end - start + 1
    last_exit = 0
    for attempt in range(1, 31):
        current = part.stat().st_size if part.exists() else 0
        if current > expected:
            log(
                f"discard oversized range {part.name}: "
                f"expected={expected}, actual={current}"
            )
            part.unlink()
            current = 0
        if current == expected:
            return
        # Retry outside curl so every attempt recomputes the absolute Range
        # from the bytes already persisted. Curl's internal HTTP/2 retry can
        # replay a partial response into an append-only output file.
        token = token_provider.get()
        headers = ["--header", f"Authorization: Bearer {token}"] if token else []
        with part.open("ab") as output:
            command = [
                "curl",
                "--fail",
                "--location",
                "--http1.1",
                "--silent",
                "--show-error",
                "--connect-timeout",
                "20",
                "--max-time",
                "300",
                "--speed-limit",
                "65536",
                "--speed-time",
                "60",
                "--range",
                f"{start + current}-{end}",
                *headers,
                url,
            ]
            completed = subprocess.run(command, stdout=output, check=False)
        last_exit = completed.returncode
        if last_exit == 22:
            token_provider.get(force=True)
        actual = part.stat().st_size if part.exists() else 0
        if actual == expected:
            return
        if actual > expected:
            raise RuntimeError(
                f"range response exceeded its requested size for {part.name}: "
                f"expected={expected}, actual={actual}"
            )
        log(
            f"retry range {part.name} attempt={attempt} exit={last_exit} "
            f"persisted={actual}/{expected}"
        )
        time.sleep(min(attempt, 5))
    raise RuntimeError(
        f"range download exhausted retries for {part.name}: "
        f"exit={last_exit}, expected={expected}, "
        f"actual={part.stat().st_size if part.exists() else 0}"
    )


def download_blob(
    *,
    registry: str,
    repository: str,
    descriptor: dict[str, Any],
    layout: Path,
    token_provider: RegistryTokenProvider,
    connections: int,
    split_threshold: int,
) -> Path:
    digest = str(descriptor["digest"])
    size = int(descriptor["size"])
    target = digest_path(layout, digest)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_file() and target.stat().st_size == size:
        if sha256_file(target) == digest.split(":", 1)[1]:
            log(f"cached {digest[:19]} {size / 1e9:.3f} GB")
            return target
        target.unlink()

    count = connections if size >= split_threshold else 1
    part_dir = layout / ".parts" / digest.split(":", 1)[1]
    part_dir.mkdir(parents=True, exist_ok=True)
    chunk = (size + count - 1) // count
    ranges: list[tuple[Path, int, int]] = []
    for index in range(count):
        start = index * chunk
        if start >= size:
            break
        end = min(size - 1, start + chunk - 1)
        ranges.append((part_dir / f"part.{index:03d}", start, end))
    url = f"https://{registry}/v2/{repository}/blobs/{digest}"
    log(f"download {digest[:19]} {size / 1e9:.3f} GB using {len(ranges)} ranges")
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(ranges)) as pool:
        futures = [
            pool.submit(
                fetch_range,
                url=url,
                token_provider=token_provider,
                part=part,
                start=start,
                end=end,
            )
            for part, start, end in ranges
        ]
        for future in concurrent.futures.as_completed(futures):
            future.result()

    temporary = target.with_suffix(".assembling")
    with temporary.open("wb") as output:
        for part, _, _ in ranges:
            with part.open("rb") as source:
                shutil.copyfileobj(source, output, 8 * 1024 * 1024)
    if temporary.stat().st_size != size:
        raise RuntimeError(f"assembled size mismatch for {digest}")
    actual_digest = sha256_file(temporary)
    if actual_digest != digest.split(":", 1)[1]:
        raise RuntimeError(
            f"digest mismatch for {digest}: downloaded sha256:{actual_digest}"
        )
    os.replace(temporary, target)
    shutil.rmtree(part_dir)
    log(f"verified {digest}")
    return target


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--repository", required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--connections", type=int, default=12)
    parser.add_argument("--parallel-blobs", type=int, default=2)
    parser.add_argument("--split-threshold-mb", type=int, default=32)
    args = parser.parse_args()
    if args.connections < 1 or args.parallel_blobs < 1:
        parser.error("connection counts must be positive")

    started = time.monotonic()
    layout = args.output.resolve()
    layout.mkdir(parents=True, exist_ok=True)
    token_provider = RegistryTokenProvider(args.registry, args.repository)
    token = token_provider.get()
    manifest_url = (
        f"https://{args.registry}/v2/{args.repository}/manifests/{args.tag}"
    )
    manifest_bytes = authenticated_bytes(manifest_url, token, MANIFEST_ACCEPT)
    manifest = json.loads(manifest_bytes)
    media_type = manifest.get("mediaType", "")
    if "manifest.list" in media_type or "image.index" in media_type:
        raise RuntimeError("multi-platform indexes are not supported; select a platform digest")

    manifest_digest = "sha256:" + hashlib.sha256(manifest_bytes).hexdigest()
    manifest_descriptor = {
        "mediaType": media_type,
        "digest": manifest_digest,
        "size": len(manifest_bytes),
        "annotations": {"org.opencontainers.image.ref.name": args.tag},
    }
    manifest_target = digest_path(layout, manifest_digest)
    manifest_target.parent.mkdir(parents=True, exist_ok=True)
    manifest_target.write_bytes(manifest_bytes)

    descriptors = [manifest["config"], *manifest.get("layers", [])]
    split_threshold = args.split_threshold_mb * 1024 * 1024
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.parallel_blobs) as pool:
        futures = [
            pool.submit(
                download_blob,
                registry=args.registry,
                repository=args.repository,
                descriptor=descriptor,
                layout=layout,
                token_provider=token_provider,
                connections=args.connections,
                split_threshold=split_threshold,
            )
            for descriptor in descriptors
        ]
        for future in concurrent.futures.as_completed(futures):
            future.result()

    (layout / "oci-layout").write_text(
        json.dumps({"imageLayoutVersion": "1.0.0"}) + "\n", encoding="utf-8"
    )
    (layout / "index.json").write_text(
        json.dumps({"schemaVersion": 2, "manifests": [manifest_descriptor]}, indent=2)
        + "\n",
        encoding="utf-8",
    )
    elapsed = time.monotonic() - started
    log(f"OCI layout complete: {layout} ({elapsed:.1f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
