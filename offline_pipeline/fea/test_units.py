# Tier: D
import math
import os
import sys
import unittest
from fractions import Fraction

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from units import from_calculix, to_calculix  # noqa: E402


class UnitsTest(unittest.TestCase):
    def test_density_carbon_fibre_1600_kg_m3(self):
        # Plan WP-7 A.2: 1600 kg/m3 = 1.6e-9 tonne/mm3. 1600 / 1e12 is one correctly
        # rounded division, equal to the literal 1.6e-9 (checked).
        self.assertEqual(to_calculix(1600.0, "density"), 1.6e-9)

    def test_length_area_second_moment(self):
        # 0.25 m = 250 mm; 1e-4 m^2 = 100 mm^2 (x1e6); 1e-8 m^4 = 1e4 mm^4 (x1e12).
        self.assertEqual(to_calculix(0.25, "length"), 250.0)
        self.assertEqual(to_calculix(1e-4, "area"), 100.0)
        self.assertEqual(to_calculix(1e-8, "second_moment_of_area"), 1e4)

    def test_stress_and_modulus(self):
        # 200e6 Pa = 200 MPa; 69e9 Pa = 69000 MPa (test value only, not a material claim).
        self.assertEqual(to_calculix(200e6, "stress"), 200.0)
        self.assertEqual(to_calculix(69e9, "stress"), 69000.0)

    def test_mass_force_time_acceleration(self):
        # 1.5 kg = 0.0015 t; N and s unchanged; 9 m/s^2 = 9000 mm/s^2.
        self.assertEqual(to_calculix(1.5, "mass"), 0.0015)
        self.assertEqual(to_calculix(9.0, "force"), 9.0)
        self.assertEqual(to_calculix(2.0, "time"), 2.0)
        self.assertEqual(to_calculix(9.0, "acceleration"), 9000.0)

    def test_dimensional_consistency(self):
        # F = m a: 2 kg x 3 m/s^2 = 6 N. In CalculiX units 0.002 t x 3000 mm/s^2 = 6 N.
        m = to_calculix(2.0, "mass")
        a = to_calculix(3.0, "acceleration")
        self.assertEqual(m * a, to_calculix(6.0, "force"))

    def test_round_trip(self):
        # Two correctly rounded operations: relative error <= 2 * 2^-53 ~ 2.2e-16 < 1e-15.
        for q in ("length", "area", "second_moment_of_area", "force", "stress",
                  "mass", "density", "time", "acceleration"):
            for x in (0.37, 1234.5, -8.25e3):
                back = from_calculix(to_calculix(x, q), q)
                self.assertLessEqual(abs(back - x), 1e-15 * abs(x), (q, x))

    def test_negative_exponent_is_correctly_rounded_division(self):
        # Independent reference: exact rational arithmetic. SI x 10^k with k < 0 must be
        # the correctly rounded value of x / 10^n, i.e. float(Fraction(x) / 10^n). A multiply
        # by the inexact double 10^-n can differ from it by one ulp for some x.
        for q, n in (("stress", 6), ("mass", 3), ("density", 12)):
            for x in (0.1, 0.3, 7.7, 123456.789, 1.0e-3, 3.3e5):
                expected = float(Fraction(x) / 10**n)
                self.assertEqual(to_calculix(x, q), expected, (q, x))
                self.assertEqual(from_calculix(x, q), float(Fraction(x) * 10**n), (q, x))

    def test_rejects_bad_input(self):
        for bad in (math.nan, math.inf, -math.inf, True, "1", None):
            with self.assertRaises(ValueError):
                to_calculix(bad, "length")
            with self.assertRaises(ValueError):
                from_calculix(bad, "length")
        with self.assertRaises(ValueError):
            to_calculix(1.0, "pressure")
        with self.assertRaises(ValueError):
            to_calculix(1e300, "second_moment_of_area")  # 1e300 x 1e12 overflows


if __name__ == "__main__":
    unittest.main()
