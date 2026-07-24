import hashlib
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


XODR = """<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
  <header north="4" south="-2" east="9" west="-3" />
  <road id="1" length="5" junction="-1">
    <planView>
      <geometry s="0" x="1" y="2" hdg="0.5" length="5"><arc curvature="0.1" /></geometry>
    </planView>
    <lanes>
      <laneOffset s="0" a="1.85" b="0" c="0" d="0" />
      <laneSection s="0">
        <center><lane id="0" type="none" /></center>
        <right>
          <lane id="-1" type="driving">
            <width sOffset="0" a="3.7" b="0" c="0" d="0" />
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""


class MirrorOpenDriveYTests(unittest.TestCase):
    def test_mirrors_geometry_and_preserves_centered_lane(self):
        from runners.mirror_opendrive_y import mirror_centered_single_lane_opendrive

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xodr"
            output = root / "mirrored.xodr"
            source.write_text(XODR, encoding="utf-8")
            source_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()

            result = mirror_centered_single_lane_opendrive(
                source,
                output,
                expected_source_sha256=source_sha256,
            )

            document = ET.parse(output).getroot()
            header = document.find("header")
            geometry = document.find("./road/planView/geometry")
            self.assertEqual(float(header.attrib["north"]), 2.0)
            self.assertEqual(float(header.attrib["south"]), -4.0)
            self.assertEqual(float(geometry.attrib["y"]), -2.0)
            self.assertEqual(float(geometry.attrib["hdg"]), -0.5)
            self.assertEqual(float(geometry.find("arc").attrib["curvature"]), -0.1)
            self.assertEqual(result["geometry_count"], 1)
            self.assertEqual(
                result["output_sha256"],
                hashlib.sha256(output.read_bytes()).hexdigest(),
            )

    def test_rejects_non_centered_lane_and_existing_output(self):
        from runners.mirror_opendrive_y import mirror_centered_single_lane_opendrive

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xodr"
            output = root / "mirrored.xodr"
            source.write_text(XODR.replace('a="1.85"', 'a="0.0"', 1), encoding="utf-8")
            source_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()
            with self.assertRaisesRegex(ValueError, "centered"):
                mirror_centered_single_lane_opendrive(
                    source,
                    output,
                    expected_source_sha256=source_sha256,
                )

            output.write_text("occupied", encoding="utf-8")
            with self.assertRaisesRegex(FileExistsError, "overwrite"):
                mirror_centered_single_lane_opendrive(
                    source,
                    output,
                    expected_source_sha256=source_sha256,
                )


if __name__ == "__main__":
    unittest.main()
