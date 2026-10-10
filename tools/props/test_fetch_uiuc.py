# Tier: D
"""Tests for fetch_uiuc.py. The sample text below is in the documented format; its numbers are a
format sample (rows copied from the site by an AI reader), used ONLY for parser tests."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fetch_uiuc as f  # noqa: E402

STATIC = """RPM CT CP
 3460.000  0.099488  0.031795
 3960.000  0.099843  0.031623
 4453.333  0.101437  0.032206
 6000.000  0.102003  0.032672
"""
WIND = """J CT CP eta
0.151095  0.080788  0.032338  0.377474
0.600657 -0.000401  0.009052 -0.026660
"""


class ParseTests(unittest.TestCase):
    def test_static_rows(self):
        rows = f.parse_static(STATIC)
        self.assertEqual(len(rows), 4)
        self.assertEqual(rows[0], {"rpm": 3460.0, "ct": 0.099488, "cp": 0.031795})

    def test_wind_rows_keep_negative_ct(self):
        rows = f.parse_wind(WIND)
        self.assertEqual(rows[1]["ct"], -0.000401)
        self.assertEqual(rows[0]["eta"], 0.377474)

    def test_wrong_header_rejected(self):
        with self.assertRaises(f.FormatError):
            f.parse_static("J CT CP eta\n1 2 3 4\n")
        with self.assertRaises(f.FormatError):
            f.parse_wind(STATIC)

    def test_bad_rows_rejected(self):
        for text in ("RPM CT CP\n1000 0.1\n", "RPM CT CP\n1000 0.1 abc\n", "RPM CT CP\n1000 nan 0.1\n",
                     "RPM CT CP\n", "", "RPM CT CP\n0 0.1 0.1\n", "RPM CT CP\n-5 0.1 0.1\n"):
            with self.assertRaises(f.FormatError, msg=text):
                f.parse_static(text)


class FitTests(unittest.TestCase):
    def test_hand_calculated_fit(self):
        # Band [3000, 4500] keeps rows 3460, 3960, 4453.333 (6000 is outside).
        # Ct = (0.099488 + 0.099843 + 0.101437) / 3 = 0.300768 / 3 = 0.100256
        # Cp = (0.031795 + 0.031623 + 0.032206) / 3 = 0.095624 / 3 = 0.0318746667
        # kappa = Cp * D / (2 pi Ct), D = 12 * 0.0254 = 0.3048 m
        #       = 0.0318746667 * 0.3048 / (6.283185307 * 0.100256) = 0.009715 / 0.629927 = 0.015423
        fit = f.static_fit(f.parse_static(STATIC))
        self.assertEqual(fit["rows_used"], 3)
        self.assertAlmostEqual(fit["ct"], 0.100256, places=9)
        self.assertAlmostEqual(fit["cp"], 0.0318746667, places=9)
        self.assertAlmostEqual(fit["kappa_m"], 0.015423, delta=2e-6)

    def test_band_edges_inclusive_and_empty_band(self):
        rows = f.parse_static(STATIC)
        self.assertEqual(f.static_fit(rows, 3460.0, 3460.0)["rows_used"], 1)
        with self.assertRaises(f.FormatError):
            f.static_fit(rows, 100.0, 200.0)

    def test_nonpositive_coefficients_rejected(self):
        with self.assertRaises(f.FormatError):
            f.static_fit([{"rpm": 3500.0, "ct": -0.1, "cp": 0.03}])

    def test_diameter_scales_kappa_linearly(self):
        rows = f.parse_static(STATIC)
        a = f.static_fit(rows, diameter_m=0.3)["kappa_m"]
        b = f.static_fit(rows, diameter_m=0.6)["kappa_m"]
        self.assertAlmostEqual(b / a, 2.0, places=12)


class PinTests(unittest.TestCase):
    def test_sha256_known_vector(self):
        # sha256("abc") from FIPS 180-4 appendix B.1.
        self.assertEqual(f.sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")

    def test_new_pin_then_match_then_mismatch(self):
        pins: dict = {}
        self.assertTrue(f.check_pin(pins, "a", "x"))
        self.assertFalse(f.check_pin(pins, "a", "x"))
        with self.assertRaises(f.FormatError):
            f.check_pin(pins, "a", "y")

    def test_build_uses_all_files_and_pins(self):
        files = {f.STATIC_FILE: STATIC.encode()}
        files.update({n: WIND.encode() for n in f.WIND_FILES})
        pins: dict = {}
        out, new = f.build(files, pins)
        self.assertTrue(new)
        self.assertEqual(len(pins), 7)
        self.assertEqual(len(out["wind_tunnel"]), 6)
        self.assertTrue(math.isclose(out["diameter_m"], 0.3048))
        files[f.WIND_FILES[0]] = b"J CT CP eta\n0.1 0.1 0.1 0.1\n"
        with self.assertRaises(f.FormatError):  # content changed -> pin mismatch
            f.build(files, pins)


if __name__ == "__main__":
    unittest.main()
