"""Audit the roads that esmini's OpenDRIVE sampler can actually materialize."""

from __future__ import annotations

import hashlib
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class EsminiXodrAuditError(RuntimeError):
    """Raised when the esmini road sampler cannot be executed."""


def parse_odrplot_lanes(text: str) -> tuple[set[str], int]:
    """Return sampled road IDs and lane sample count from ``odrplot`` CSV."""

    road_ids: set[str] = set()
    lane_sample_count = 0
    for raw_line in text.splitlines():
        fields = [field.strip() for field in raw_line.split(",")]
        if len(fields) < 2 or fields[0] != "lane":
            continue
        road_id = fields[1]
        if not road_id:
            continue
        road_ids.add(road_id)
        lane_sample_count += 1
    return road_ids, lane_sample_count


def evaluate_odrplot_result(
    *,
    road_ids: set[str],
    sampled_road_ids: set[str],
    lane_sample_count: int,
    returncode: int,
    command: list[str],
    stdout: str,
    stderr: str,
    artifact_sha256: str,
    expected_sha256: str | None,
) -> dict[str, Any]:
    """Evaluate sampler output without depending on an installed esmini binary."""

    missing = sorted(road_ids - sampled_road_ids, key=_numeric_or_text)
    extra = sorted(sampled_road_ids - road_ids, key=_numeric_or_text)
    errors: list[str] = []
    if expected_sha256 is not None and artifact_sha256 != expected_sha256:
        errors.append(
            "OpenDRIVE SHA-256 mismatch: "
            f"expected={expected_sha256} actual={artifact_sha256}"
        )
    if returncode != 0:
        errors.append(f"odrplot returned non-zero status: {returncode}")
    if lane_sample_count <= 0:
        errors.append("odrplot produced no lane samples")
    if missing:
        errors.append(
            "odrplot did not materialize road IDs: " + ", ".join(missing[:20])
        )
    if extra:
        errors.append("odrplot reported unknown road IDs: " + ", ".join(extra[:20]))
    return {
        "schema_version": "esmini_xodr_runtime_audit.v1",
        "status": "passed" if not errors else "failed",
        "artifact_sha256": artifact_sha256,
        "expected_artifact_sha256": expected_sha256,
        "road_count": len(road_ids),
        "sampled_road_count": len(sampled_road_ids),
        "lane_sample_count": lane_sample_count,
        "missing_road_ids": missing,
        "unknown_road_ids": extra,
        "command": command,
        "returncode": returncode,
        "stdout": stdout,
        "stderr": stderr,
        "errors": errors,
    }


def audit_xodr_with_odrplot(
    xodr_path: Path,
    odrplot_path: Path,
    *,
    sample_step_m: float = 1.0,
    expected_sha256: str | None = None,
    timeout_sec: float = 60.0,
) -> dict[str, Any]:
    """Run ``odrplot`` and require every XML road to produce lane samples."""

    xodr = Path(xodr_path).expanduser().resolve()
    odrplot = Path(odrplot_path).expanduser().resolve()
    if sample_step_m <= 0.0:
        raise ValueError("sample_step_m must be positive")
    if timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be positive")
    if expected_sha256 is not None and not _SHA256.fullmatch(expected_sha256):
        raise ValueError("expected_sha256 must be a lowercase sha256")
    if not xodr.is_file():
        raise EsminiXodrAuditError(f"OpenDRIVE does not exist: {xodr}")
    if not odrplot.is_file():
        raise EsminiXodrAuditError(f"odrplot executable does not exist: {odrplot}")

    try:
        root = ET.parse(xodr).getroot()
    except (OSError, ET.ParseError) as exc:
        raise EsminiXodrAuditError(f"cannot parse OpenDRIVE: {xodr}") from exc
    if root.tag != "OpenDRIVE":
        raise EsminiXodrAuditError(f"not an OpenDRIVE document: {xodr}")
    roads = root.findall("./road")
    road_ids = {str(road.attrib.get("id", "")) for road in roads}
    if "" in road_ids or len(road_ids) != len(roads):
        raise EsminiXodrAuditError("OpenDRIVE contains missing or duplicate road IDs")

    artifact_sha256 = _sha256_file(xodr)
    command = [str(odrplot), str(xodr), "odrplot.csv", f"{sample_step_m:g}"]
    with tempfile.TemporaryDirectory(prefix="closedloopbench-odrplot-") as directory:
        output = Path(directory) / "odrplot.csv"
        command[2] = str(output)
        try:
            completed = subprocess.run(
                command,
                cwd=directory,
                text=True,
                capture_output=True,
                check=False,
                timeout=timeout_sec,
            )
            stdout = completed.stdout
            stderr = completed.stderr
            returncode = completed.returncode
        except subprocess.TimeoutExpired as exc:
            stdout = _text_output(exc.stdout)
            stderr = _text_output(exc.stderr)
            returncode = 124
            command = [*command, "<timeout>"]
        sampled_road_ids: set[str] = set()
        lane_sample_count = 0
        if output.is_file():
            sampled_road_ids, lane_sample_count = parse_odrplot_lanes(
                output.read_text(encoding="utf-8", errors="replace")
            )

    report = evaluate_odrplot_result(
        road_ids=road_ids,
        sampled_road_ids=sampled_road_ids,
        lane_sample_count=lane_sample_count,
        returncode=returncode,
        command=command,
        stdout=stdout,
        stderr=stderr,
        artifact_sha256=artifact_sha256,
        expected_sha256=expected_sha256,
    )
    report.update(
        {
            "xodr": str(xodr),
            "odrplot": str(odrplot),
            "sample_step_m": sample_step_m,
        }
    )
    return report


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _numeric_or_text(value: str) -> tuple[int, int | str]:
    try:
        return 0, int(value)
    except ValueError:
        return 1, value


def _text_output(value: bytes | str | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)
