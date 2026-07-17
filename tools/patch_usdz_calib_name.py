#!/usr/bin/env python3
"""Create a byte-layout-preserving USDZ copy with a legacy calib name.

NuRec 26.04 exports ``free-pose-calib`` even when calibration is disabled,
while the CARLA NuRec gRPC 0.2.0 runtime only registers ``skip-calib`` and
``direct-calib``.  USDZ files are ZIP archives with alignment constraints, so
this tool patches the stored YAML member in place and updates both ZIP CRC
fields without rebuilding the archive.
"""

from __future__ import annotations

import argparse
import shutil
import struct
import zlib
import zipfile
from pathlib import Path


LOCAL_HEADER = struct.Struct("<IHHHHHIIIHH")
CENTRAL_HEADER = struct.Struct("<IHHHHHHIIIHHHHHII")
LOCAL_SIGNATURE = 0x04034B50
CENTRAL_SIGNATURE = 0x02014B50


def patch_archive(
    source: Path,
    output: Path,
    member: str,
    old: bytes,
    new: bytes,
) -> None:
    if len(new) > len(old):
        raise ValueError("replacement must not be longer than the original")
    replacement = new + b" " * (len(old) - len(new))

    source = source.resolve()
    output = output.resolve()
    if source == output:
        raise ValueError("output must be a new file; the source is preserved")
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, output)

    with zipfile.ZipFile(output, "r") as archive:
        info = archive.getinfo(member)
        if info.compress_type != zipfile.ZIP_STORED:
            raise RuntimeError(f"{member} is compressed; layout-safe patch is unavailable")
        if info.flag_bits & 0x08:
            raise RuntimeError(f"{member} uses a data descriptor; refusing unsafe patch")
        central_start = archive.start_dir

    with output.open("r+b") as handle:
        handle.seek(info.header_offset)
        local_raw = handle.read(LOCAL_HEADER.size)
        local = LOCAL_HEADER.unpack(local_raw)
        if local[0] != LOCAL_SIGNATURE:
            raise RuntimeError("invalid local ZIP header signature")
        filename_length, extra_length = local[-2:]
        data_offset = info.header_offset + LOCAL_HEADER.size + filename_length + extra_length

        handle.seek(data_offset)
        payload = handle.read(info.file_size)
        count = payload.count(old)
        if count != 1:
            raise RuntimeError(f"expected exactly one {old!r} in {member}, found {count}")
        patched = payload.replace(old, replacement, 1)
        crc = zlib.crc32(patched) & 0xFFFFFFFF

        handle.seek(data_offset)
        handle.write(patched)
        handle.seek(info.header_offset + 14)
        handle.write(struct.pack("<I", crc))

        handle.seek(central_start)
        central_matches = 0
        while True:
            position = handle.tell()
            signature_raw = handle.read(4)
            if signature_raw != b"PK\x01\x02":
                break
            handle.seek(position)
            central_raw = handle.read(CENTRAL_HEADER.size)
            central = CENTRAL_HEADER.unpack(central_raw)
            if central[0] != CENTRAL_SIGNATURE:
                raise RuntimeError("invalid central ZIP header signature")
            filename_length = central[10]
            extra_length = central[11]
            comment_length = central[12]
            filename = handle.read(filename_length).decode("utf-8")
            if filename == member:
                handle.seek(position + 16)
                handle.write(struct.pack("<I", crc))
                central_matches += 1
            handle.seek(
                position
                + CENTRAL_HEADER.size
                + filename_length
                + extra_length
                + comment_length
            )
        if central_matches != 1:
            raise RuntimeError(
                f"expected one central directory entry for {member}, found {central_matches}"
            )

    with zipfile.ZipFile(output, "r") as archive:
        verified = archive.read(member)
        if old in verified or replacement not in verified:
            raise RuntimeError("patched member verification failed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--member", default="parsed_config.yaml")
    parser.add_argument("--old", default="free-pose-calib")
    parser.add_argument("--new", default="skip-calib")
    args = parser.parse_args()
    patch_archive(
        args.source,
        args.output,
        args.member,
        args.old.encode("utf-8"),
        args.new.encode("utf-8"),
    )
    print(f"created compatible USDZ: {args.output.resolve()}")


if __name__ == "__main__":
    main()
