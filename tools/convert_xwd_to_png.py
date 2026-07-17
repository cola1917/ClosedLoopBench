from __future__ import annotations

import argparse
import struct
from pathlib import Path

from PIL import Image


def convert_xwd(source: Path, output: Path) -> Path:
    payload = source.read_bytes()
    if len(payload) < 100:
        raise ValueError("XWD file is shorter than its fixed header")
    header = struct.unpack(">25I", payload[:100])
    (
        header_size,
        version,
        _pixmap_format,
        _depth,
        width,
        height,
        _xoffset,
        byte_order,
        _bitmap_unit,
        _bitmap_bit_order,
        _bitmap_pad,
        bits_per_pixel,
        bytes_per_line,
        _visual_class,
        red_mask,
        green_mask,
        blue_mask,
        _bits_per_rgb,
        _colormap_entries,
        color_count,
        *_window,
    ) = header
    if version != 7:
        raise ValueError(f"unsupported XWD version: {version}")
    if (bits_per_pixel, byte_order, red_mask, green_mask, blue_mask) != (
        24,
        0,
        0xFF0000,
        0x00FF00,
        0x0000FF,
    ):
        raise ValueError("unsupported XWD pixel layout")
    data_offset = header_size + color_count * 12
    expected = bytes_per_line * height
    pixels = payload[data_offset : data_offset + expected]
    if len(pixels) != expected:
        raise ValueError("truncated XWD pixel payload")
    image = Image.frombytes(
        "RGB", (width, height), pixels, "raw", "BGR", bytes_per_line, 1
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert the CARLA VNC XWD capture to PNG.")
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(convert_xwd(args.source, args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
