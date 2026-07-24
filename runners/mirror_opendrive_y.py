from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def mirror_centered_single_lane_opendrive(
    source: Path,
    output: Path,
    *,
    expected_source_sha256: str,
) -> dict[str, object]:
    source = Path(source)
    output = Path(output)
    if not source.is_file():
        raise FileNotFoundError(f"OpenDRIVE source does not exist: {source}")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite mirrored OpenDRIVE: {output}")
    source_sha256 = _sha256_file(source)
    if source_sha256 != expected_source_sha256:
        raise ValueError("OpenDRIVE source SHA-256 mismatch")

    tree = ET.parse(source)
    root = tree.getroot()
    roads = root.findall("road")
    if len(roads) != 1:
        raise ValueError("Y mirroring requires exactly one OpenDRIVE road")
    _require_centered_single_right_lane(roads[0])

    header = root.find("header")
    if header is not None and all(
        key in header.attrib for key in ("north", "south")
    ):
        north = float(header.attrib["north"])
        south = float(header.attrib["south"])
        header.set("north", _number(-south))
        header.set("south", _number(-north))

    geometries = root.findall("./road/planView/geometry")
    if not geometries:
        raise ValueError("OpenDRIVE road has no planView geometry")
    for geometry in geometries:
        geometry.set("y", _number(-float(geometry.attrib["y"])))
        geometry.set("hdg", _number(-float(geometry.attrib["hdg"])))
        arc = geometry.find("arc")
        if arc is not None:
            arc.set("curvature", _number(-float(arc.attrib["curvature"])))
        spiral = geometry.find("spiral")
        if spiral is not None:
            for key in ("curvStart", "curvEnd"):
                spiral.set(key, _number(-float(spiral.attrib[key])))
        poly3 = geometry.find("poly3")
        if poly3 is not None:
            for key in ("a", "b", "c", "d"):
                poly3.set(key, _number(-float(poly3.attrib[key])))
        param_poly3 = geometry.find("paramPoly3")
        if param_poly3 is not None:
            for key in ("aV", "bV", "cV", "dV"):
                param_poly3.set(key, _number(-float(param_poly3.attrib[key])))

    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return {
        "status": "passed",
        "source": str(source.resolve()),
        "source_sha256": source_sha256,
        "output": str(output.resolve()),
        "output_sha256": _sha256_file(output),
        "geometry_count": len(geometries),
        "transform": "reflect_global_y",
        "lane_layout": "centered_single_right_driving_lane",
    }


def _require_centered_single_right_lane(road: ET.Element) -> None:
    sections = road.findall("./lanes/laneSection")
    offsets = road.findall("./lanes/laneOffset")
    if len(sections) != 1 or len(offsets) != 1:
        raise ValueError("Y mirroring requires one lane section and one lane offset")
    left = sections[0].find("left")
    right_lanes = sections[0].findall("./right/lane")
    if left is not None and list(left):
        raise ValueError("Y mirroring does not accept left lanes")
    if len(right_lanes) != 1 or right_lanes[0].attrib.get("id") != "-1":
        raise ValueError("Y mirroring requires one right lane with id -1")
    if right_lanes[0].attrib.get("type") != "driving":
        raise ValueError("Y mirroring requires a driving lane")
    widths = right_lanes[0].findall("width")
    if len(widths) != 1:
        raise ValueError("Y mirroring requires one constant lane width")
    width = float(widths[0].attrib["a"])
    offset = float(offsets[0].attrib["a"])
    if any(abs(float(widths[0].attrib.get(key, "0"))) > 1e-12 for key in "bcd"):
        raise ValueError("Y mirroring requires a constant lane width")
    if any(abs(float(offsets[0].attrib.get(key, "0"))) > 1e-12 for key in "bcd"):
        raise ValueError("Y mirroring requires a constant lane offset")
    if not math.isclose(offset, width / 2.0, abs_tol=1e-9):
        raise ValueError("right driving lane must be centered on the reference line")


def _number(value: float) -> str:
    if abs(value) < 5e-13:
        value = 0.0
    return f"{value:.12f}".rstrip("0").rstrip(".") or "0"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Mirror one centered single-lane OpenDRIVE road across global Y."
    )
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--expected-source-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = mirror_centered_single_lane_opendrive(
            args.source,
            args.output,
            expected_source_sha256=args.expected_source_sha256,
        )
    except (OSError, ValueError, ET.ParseError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}))
        return 2
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
