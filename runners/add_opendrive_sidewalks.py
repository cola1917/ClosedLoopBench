from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def add_symmetric_sidewalks(
    source: Path,
    output: Path,
    *,
    expected_source_sha256: str,
    sidewalk_width_m: float = 8.0,
) -> dict[str, object]:
    source = Path(source)
    output = Path(output)
    if not source.is_file():
        raise FileNotFoundError(f"OpenDRIVE source does not exist: {source}")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite sidewalk OpenDRIVE: {output}")
    source_sha256 = _sha256_file(source)
    if source_sha256 != expected_source_sha256:
        raise ValueError("OpenDRIVE source SHA-256 mismatch")
    if not math.isfinite(sidewalk_width_m) or sidewalk_width_m <= 0.0:
        raise ValueError("sidewalk width must be finite and positive")

    tree = ET.parse(source)
    root = tree.getroot()
    roads = root.findall("road")
    if len(roads) != 1:
        raise ValueError("sidewalk augmentation requires exactly one OpenDRIVE road")
    section, driving_lane = _require_centered_single_right_lane(roads[0])

    left = section.find("left")
    if left is None:
        left = ET.Element("left")
        center = section.find("center")
        section.insert(list(section).index(center) if center is not None else 0, left)
    left.append(_sidewalk_lane(1, sidewalk_width_m))
    right = section.find("right")
    if right is None:
        raise ValueError("sidewalk augmentation requires a right lane group")
    right.append(_sidewalk_lane(-2, sidewalk_width_m))

    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return {
        "status": "passed",
        "source": str(source.resolve()),
        "source_sha256": source_sha256,
        "output": str(output.resolve()),
        "output_sha256": _sha256_file(output),
        "driving_lane_id": int(driving_lane.attrib["id"]),
        "sidewalk_lane_ids": [1, -2],
        "sidewalk_width_m": float(sidewalk_width_m),
        "lane_layout": "centered_single_driving_lane_with_symmetric_sidewalks",
    }


def _require_centered_single_right_lane(
    road: ET.Element,
) -> tuple[ET.Element, ET.Element]:
    sections = road.findall("./lanes/laneSection")
    offsets = road.findall("./lanes/laneOffset")
    if len(sections) != 1 or len(offsets) != 1:
        raise ValueError("sidewalk augmentation requires one lane section and one lane offset")
    section = sections[0]
    left_lanes = section.findall("./left/lane")
    right_lanes = section.findall("./right/lane")
    if left_lanes:
        raise ValueError("sidewalk augmentation requires no existing left lanes")
    if len(right_lanes) != 1 or right_lanes[0].attrib.get("id") != "-1":
        raise ValueError("sidewalk augmentation requires one right lane with id -1")
    driving_lane = right_lanes[0]
    if driving_lane.attrib.get("type") != "driving":
        raise ValueError("lane -1 must be a driving lane")
    widths = driving_lane.findall("width")
    if len(widths) != 1:
        raise ValueError("driving lane must have one constant width")
    width = float(widths[0].attrib["a"])
    offset = float(offsets[0].attrib["a"])
    if any(abs(float(widths[0].attrib.get(key, "0"))) > 1e-12 for key in "bcd"):
        raise ValueError("driving lane width must be constant")
    if any(abs(float(offsets[0].attrib.get(key, "0"))) > 1e-12 for key in "bcd"):
        raise ValueError("lane offset must be constant")
    if not math.isclose(offset, width / 2.0, abs_tol=1e-9):
        raise ValueError("right driving lane must be centered on the reference line")
    return section, driving_lane


def _sidewalk_lane(lane_id: int, width_m: float) -> ET.Element:
    lane = ET.Element(
        "lane", {"id": str(lane_id), "type": "sidewalk", "level": "false"}
    )
    ET.SubElement(
        lane,
        "width",
        {
            "sOffset": "0",
            "a": _number(width_m),
            "b": "0",
            "c": "0",
            "d": "0",
        },
    )
    ET.SubElement(
        lane,
        "roadMark",
        {
            "sOffset": "0",
            "type": "none",
            "weight": "standard",
            "color": "standard",
            "width": "0",
            "laneChange": "none",
        },
    )
    return lane


def _number(value: float) -> str:
    return f"{value:.12f}".rstrip("0").rstrip(".") or "0"


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Add symmetric physical sidewalks to a centered single-lane OpenDRIVE road."
    )
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--expected-source-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--sidewalk-width-m", type=float, default=8.0)
    args = parser.parse_args(argv)
    try:
        result = add_symmetric_sidewalks(
            args.source,
            args.output,
            expected_source_sha256=args.expected_source_sha256,
            sidewalk_width_m=args.sidewalk_width_m,
        )
    except (OSError, ValueError, ET.ParseError) as exc:
        print(json.dumps({"status": "failed", "detail": str(exc)}))
        return 2
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
