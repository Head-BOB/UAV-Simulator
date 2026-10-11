# Tier: D
"""Tests for flow_image: colour map by hand, PNG structure (signature, chunks, CRC, inflated size), pixel placement, refusals."""
import json
import struct
import tempfile
import unittest
import zlib
from pathlib import Path

import numpy as np

import flow_image as F


def parse_png(data: bytes):
    assert data[:8] == F.PNG_SIGNATURE
    pos, chunks = 8, []
    while pos < len(data):
        (n,) = struct.unpack(">I", data[pos:pos + 4])
        kind, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + n]
        (crc,) = struct.unpack(">I", data[pos + 8 + n:pos + 12 + n])
        assert crc == zlib.crc32(kind + body) & 0xFFFFFFFF, kind
        chunks.append((kind, body))
        pos += 12 + n
    return chunks


def pixels(data: bytes):
    chunks = parse_png(data)
    w, h = struct.unpack(">II", chunks[0][1][:8])
    raw = zlib.decompress(b"".join(b for k, b in chunks if k == b"IDAT"))
    rows = []
    for r in range(h):
        line = raw[r * (1 + 3 * w):(r + 1) * (1 + 3 * w)]
        assert line[0] == 0
        rows.append([tuple(line[1 + 3 * c:4 + 3 * c]) for c in range(w)])
    return w, h, rows


class ColourTests(unittest.TestCase):
    def test_stops_and_midpoints_by_hand(self):
        self.assertEqual(F.colour(0.0), (20, 20, 100))
        self.assertEqual(F.colour(0.25), (30, 100, 200))
        self.assertEqual(F.colour(0.5), (40, 200, 200))
        self.assertEqual(F.colour(0.75), (240, 220, 60))
        self.assertEqual(F.colour(1.0), (200, 30, 30))
        # halfway between stops 0 and 0.25 (t = 0.125): (25, 60, 150); between 0.75 and 1 (t = 0.875): (220, 125, 45)
        self.assertEqual(F.colour(0.125), (25, 60, 150))
        self.assertEqual(F.colour(0.875), (220, 125, 45))

    def test_clamping(self):
        self.assertEqual(F.colour(-3.0), F.colour(0.0))
        self.assertEqual(F.colour(7.0), F.colour(1.0))


class PngTests(unittest.TestCase):
    def test_structure_of_a_2x2_image(self):
        rgb = np.array([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]], dtype=np.uint8)
        data = F.encode_png(rgb)
        chunks = parse_png(data)
        self.assertEqual([k for k, _ in chunks], [b"IHDR", b"IDAT", b"IEND"])
        self.assertEqual(chunks[0][1], struct.pack(">IIBBBBB", 2, 2, 8, 2, 0, 0, 0))
        self.assertEqual(len(zlib.decompress(chunks[1][1])), 2 * (1 + 3 * 2))          # rows x (filter byte + 3 x width)
        self.assertEqual(chunks[2][1], b"")
        w, h, rows = pixels(data)
        self.assertEqual((w, h, rows), (2, 2, [[(1, 2, 3), (4, 5, 6)], [(7, 8, 9), (10, 11, 12)]]))

    def test_refuses_the_wrong_array(self):
        for bad in (np.zeros((2, 2), dtype=np.uint8), np.zeros((2, 2, 3), dtype=np.float32), np.zeros((2, 2, 4), dtype=np.uint8)):
            with self.assertRaises(F.ImageError):
                F.encode_png(bad)

    def test_deterministic(self):
        rgb = np.arange(48, dtype=np.uint8).reshape(4, 4, 3)
        self.assertEqual(F.encode_png(rgb), F.encode_png(rgb))


class ImageTests(unittest.TestCase):
    def slice_file(self, t, values):
        p = Path(t) / "s.f32"
        p.write_bytes(np.array(values, dtype="<f4").tobytes())
        return p

    def test_pixel_for_pixel_placement_and_flip(self):
        # nx 2, nz 2, freestream 10 m/s -> full scale 15; z = 0 row is [0, 7.5] (solid, half scale), z = 1 row is [15, 3.75]
        with tempfile.TemporaryDirectory() as t:
            p = self.slice_file(t, [0.0, 7.5, 15.0, 3.75])
            r = F.make_image(p, 2, 2, 10.0, Path(t) / "o.png", "case")
            w, h, rows = pixels((Path(t) / "o.png").read_bytes())
            self.assertEqual((w, h), (2, 2))
            # top image row = largest z = [15 -> t 1.0 red, 3.75 -> t 0.25]; bottom = [solid grey, 7.5 -> t 0.5]
            self.assertEqual(rows[0], [(200, 30, 30), (30, 100, 200)])
            self.assertEqual(rows[1], [(60, 60, 60), (40, 200, 200)])
            self.assertAlmostEqual(r["full_scale_mps"], 15.0)
            self.assertAlmostEqual(r["max_speed_mps"], 15.0)

    def test_a_non_square_slice_keeps_x_along_the_row(self):
        # nx 3, nz 2: file order is x fastest then z: row z=0 = [10, 20, 30], row z=1 = [1, 2, 3]; speed 20 -> full scale 30
        with tempfile.TemporaryDirectory() as t:
            p = self.slice_file(t, [10.0, 20.0, 30.0, 1.0, 2.0, 3.0])
            F.make_image(p, 3, 2, 20.0, Path(t) / "o.png")
            w, h, rows = pixels((Path(t) / "o.png").read_bytes())
            self.assertEqual((w, h), (3, 2))
            self.assertEqual(rows[0], [F.colour(1 / 30), F.colour(2 / 30), F.colour(3 / 30)])      # top row = z = 1
            self.assertEqual(rows[1], [F.colour(10 / 30), F.colour(20 / 30), F.colour(1.0)])

    def test_non_finite_is_magenta_and_scale_repeats_pixels(self):
        with tempfile.TemporaryDirectory() as t:
            p = self.slice_file(t, [float("nan"), 5.0])
            F.make_image(p, 2, 1, 10.0, Path(t) / "o.png", scale=3)
            w, h, rows = pixels((Path(t) / "o.png").read_bytes())
            self.assertEqual((w, h), (6, 3))
            self.assertTrue(all(row[0:3] == [(255, 0, 255)] * 3 for row in rows))
            self.assertTrue(all(row[3:6] == [F.colour(5.0 / 15.0)] * 3 for row in rows))

    def test_legend_text(self):
        with tempfile.TemporaryDirectory() as t:
            p = self.slice_file(t, [1.0, 2.0])
            F.make_image(p, 2, 1, 10.0, Path(t) / "o.png", "v10_ap00_bp00", full_scale=20.0)
            text = (Path(t) / "o.legend.txt").read_text(encoding="utf-8")
            for needle in ("v10_ap00_bp00", "0 .. 20 m/s", "freestream 10 m/s", "PRELIMINARY CFD, UNVALIDATED", "solid cells"):
                self.assertIn(needle, text)

    def test_refusals(self):
        with tempfile.TemporaryDirectory() as t:
            p = self.slice_file(t, [1.0, 2.0, 3.0])
            with self.assertRaises(F.ImageError):
                F.read_slice(p, 2, 2)                      # 3 floats given, 4 expected
            with self.assertRaises(F.ImageError):
                F.read_slice(p, 0, 3)
            with self.assertRaises(F.ImageError):
                F.to_rgb(np.ones((1, 1)), 0.0)
            with self.assertRaises(F.ImageError):
                F.upscale(np.zeros((1, 1, 3), dtype=np.uint8), 0)

    def test_cli_with_case_dir_and_errors(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t) / "case"
            d.mkdir()
            (d / "case.json").write_text(json.dumps({"case_id": "c1", "speed_mps": 10.0, "grid": {"nx": 2, "ny": 2, "nz": 1}}), encoding="utf-8")
            (d / "slice_y_mid.f32").write_bytes(np.array([4.0, 8.0], dtype="<f4").tobytes())
            self.assertEqual(F.main(["--case-dir", str(d), "--out", str(Path(t) / "a.png")]), 0)
            self.assertTrue((Path(t) / "a.png").exists() and (Path(t) / "a.legend.txt").exists())
            self.assertEqual(F.main(["--out", str(Path(t) / "b.png")]), 1)
            self.assertEqual(F.main(["--case-dir", str(Path(t) / "nope"), "--out", str(Path(t) / "c.png")]), 1)


if __name__ == "__main__":
    unittest.main()
