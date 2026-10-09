# Tier: D
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from calculix_dat import parse_dat  # noqa: E402

# Hand-written sample in the ASSUMED CalculiX layout (TODO(HUMAN-VERIFY), B-9). Every number
# below is checked by direct comparison with the literal in the test.
SAMPLE = """\

 displacements (vx,vy,vz) for set NALL and time  0.1000000E+01

         1  0.000000E+00  0.000000E+00  0.000000E+00
         2  1.250000E-02 -2.500000E-01  3.000000E+00

 stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set EALL and time  0.1000000E+01

         1   1  1.000000E+02 -5.000000E+01  0.000000E+00  2.500000E+01  0.000000E+00 -1.000000E+00
         1   2  2.000000E+02  0.000000E+00  0.000000E+00  0.000000E+00  0.000000E+00  0.000000E+00
"""


class ParseDatTest(unittest.TestCase):
    def test_sample_every_number(self):
        blocks = parse_dat(SAMPLE)
        self.assertEqual([b.name for b in blocks], ["displacements", "stresses"])
        disp, stress = blocks
        self.assertEqual(disp.set_name, "NALL")
        self.assertEqual(disp.time, 1.0)
        self.assertEqual(disp.columns, ("vx", "vy", "vz"))
        self.assertEqual(disp.rows, {
            (1,): (0.0, 0.0, 0.0),
            (2,): (0.0125, -0.25, 3.0),
        })
        self.assertEqual(stress.set_name, "EALL")
        self.assertEqual(stress.columns, ("sxx", "syy", "szz", "sxy", "sxz", "syz"))
        self.assertEqual(stress.rows, {
            (1, 1): (100.0, -50.0, 0.0, 25.0, 0.0, -1.0),
            (1, 2): (200.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        })

    def test_two_time_steps_keep_file_order(self):
        text = (
            " displacements (vx,vy,vz) for set N and time  0.5E+00\n\n 1 1.0 2.0 3.0\n"
            " displacements (vx,vy,vz) for set N and time  1.0E+00\n\n 1 4.0 5.0 6.0\n"
        )
        blocks = parse_dat(text)
        self.assertEqual([b.time for b in blocks], [0.5, 1.0])
        self.assertEqual(blocks[0].rows[(1,)], (1.0, 2.0, 3.0))
        self.assertEqual(blocks[1].rows[(1,)], (4.0, 5.0, 6.0))

    def assertBad(self, text, fragment):
        with self.assertRaises(ValueError) as ctx:
            parse_dat(text)
        self.assertIn(fragment, str(ctx.exception))

    def test_unknown_block(self):
        self.assertBad(" forces (fx,fy,fz) for set N and time 1.0\n 1 1 2 3\n", "unsupported block")

    def test_data_before_header(self):
        self.assertBad(" 1 1.0 2.0 3.0\n", "data before any block header")

    def test_empty_and_blank(self):
        self.assertBad("", "no result blocks")
        self.assertBad("\n   \n", "no result blocks")

    def test_wrong_column_count(self):
        head = " displacements (vx,vy,vz) for set N and time 1.0\n"
        self.assertBad(head + " 1 1.0 2.0\n", "expected 4 columns")
        self.assertBad(head + " 1 1.0 2.0 3.0 4.0\n", "expected 4 columns")

    def test_stress_needs_two_keys(self):
        head = " stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set E and time 1.0\n"
        self.assertBad(head + " 1 1.0 2.0 3.0 4.0 5.0 6.0\n", "expected 8 columns")

    def test_bad_keys(self):
        head = " displacements (vx,vy,vz) for set N and time 1.0\n"
        for key in ("0", "-1", "1.5", "x"):
            with self.subTest(key=key):
                self.assertBad(head + f" {key} 1.0 2.0 3.0\n", "key must be an integer >= 1")

    def test_duplicate_key(self):
        head = " displacements (vx,vy,vz) for set N and time 1.0\n"
        self.assertBad(head + " 1 1.0 2.0 3.0\n 1 4.0 5.0 6.0\n", "duplicate key (1,)")

    def test_nan_inf_and_fortran_exponent(self):
        head = " displacements (vx,vy,vz) for set N and time 1.0\n"
        for bad in ("nan", "inf", "-inf", "NaN", "1.0-100", "1.0D+00", "1,5"):
            with self.subTest(bad=bad):
                self.assertBad(head + f" 1 {bad} 2.0 3.0\n", "line 2")

    def test_overflow_value_rejected(self):
        # 1e999 matches the number syntax but parses to inf; must be rejected as non-finite.
        head = " displacements (vx,vy,vz) for set N and time 1.0\n"
        self.assertBad(head + " 1 1e999 2.0 3.0\n", "non-finite")

    def test_bad_time(self):
        self.assertBad(" displacements (vx,vy,vz) for set N and time abc\n", "line 1")


if __name__ == "__main__":
    unittest.main()
