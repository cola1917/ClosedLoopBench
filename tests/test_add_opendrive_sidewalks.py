import hashlib
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


XODR = """<?xml version="1.0" encoding="utf-8"?>
<OpenDRIVE>
  <road id="1" length="5" junction="-1">
    <planView><geometry s="0" x="1" y="2" hdg="0" length="5"><line /></geometry></planView>
    <lanes>
      <laneOffset s="0" a="1.85" b="0" c="0" d="0" />
      <laneSection s="0">
        <center><lane id="0" type="none" /></center>
        <right><lane id="-1" type="driving"><width sOffset="0" a="3.7" b="0" c="0" d="0" /></lane></right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""


class AddOpenDriveSidewalksTests(unittest.TestCase):
    def test_adds_symmetric_sidewalks_and_reports_immutable_identity(self):
        from runners.add_opendrive_sidewalks import add_symmetric_sidewalks

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xodr"
            output = root / "sidewalks.xodr"
            source.write_text(XODR, encoding="utf-8")
            source_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()

            result = add_symmetric_sidewalks(
                source,
                output,
                expected_source_sha256=source_sha256,
                sidewalk_width_m=8.0,
            )

            document = ET.parse(output).getroot()
            lanes = document.findall("./road/lanes/laneSection/*/lane")
            by_id = {int(lane.attrib["id"]): lane for lane in lanes}
            self.assertEqual(set(by_id), {1, 0, -1, -2})
            self.assertEqual(by_id[1].attrib["type"], "sidewalk")
            self.assertEqual(by_id[-2].attrib["type"], "sidewalk")
            self.assertEqual(float(by_id[1].find("width").attrib["a"]), 8.0)
            self.assertEqual(result["source_sha256"], source_sha256)
            self.assertEqual(
                result["output_sha256"], hashlib.sha256(output.read_bytes()).hexdigest()
            )

    def test_rejects_wrong_source_identity_and_existing_output(self):
        from runners.add_opendrive_sidewalks import add_symmetric_sidewalks

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xodr"
            output = root / "sidewalks.xodr"
            source.write_text(XODR, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                add_symmetric_sidewalks(
                    source, output, expected_source_sha256="0" * 64
                )
            output.write_text("occupied", encoding="utf-8")
            with self.assertRaisesRegex(FileExistsError, "overwrite"):
                add_symmetric_sidewalks(
                    source,
                    output,
                    expected_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                )


if __name__ == "__main__":
    unittest.main()
