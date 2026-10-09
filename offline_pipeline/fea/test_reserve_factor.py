# Tier: D
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from reserve_factor import reserve_factor  # noqa: E402
from units import to_calculix  # noqa: E402


class ReserveFactorTest(unittest.TestCase):
    def test_rf_hand_calc(self):
        # 600 / (200 x 1.5) = 600 / 300 = 2.0
        self.assertEqual(reserve_factor(600.0, 200.0, 1.5), 2.0)

    def test_rf_exactly_one_at_boundary(self):
        # 300 / (200 x 1.5) = 1.0 (the failure threshold; RF < 1 fails)
        self.assertEqual(reserve_factor(300.0, 200.0, 1.5), 1.0)

    def test_rf_below_one_is_failure_case(self):
        # 150 / (200 x 1.5) = 0.5
        self.assertEqual(reserve_factor(150.0, 200.0, 1.5), 0.5)

    def test_rf_same_case_mm_and_m(self):
        # SI: 6e8 / (2e8 x 1.5) = 2.0. CalculiX units: 600 / (200 x 1.5) = 2.0.
        # Exact here because these conversions are exact; arbitrary values can differ
        # by a few ulp. The plan's rule really targets a full CalculiX run (open, B-8/B-9).
        si = reserve_factor(600e6, 200e6, 1.5)
        mpa = reserve_factor(
            to_calculix(600e6, "stress"), to_calculix(200e6, "stress"), 1.5
        )
        self.assertEqual(si, 2.0)
        self.assertEqual(si, mpa)

    def test_rf_fos_one_accepted(self):
        # 400 / (200 x 1.0) = 2.0
        self.assertEqual(reserve_factor(400.0, 200.0, 1.0), 2.0)

    def test_rf_rejects_bad_input(self):
        bad_calls = [
            (600.0, 0.0, 1.5), (600.0, -1.0, 1.5),
            (0.0, 200.0, 1.5), (-5.0, 200.0, 1.5),
            (600.0, 200.0, 0.99), (600.0, 200.0, 0.0),
            (True, 200.0, 1.5), (600.0, True, 1.5), (600.0, 200.0, True),
            ("600", 200.0, 1.5), (1e308, 1e-308, 1.5),
        ]
        for bad in (math.nan, math.inf, -math.inf):
            bad_calls += [(bad, 200.0, 1.5), (600.0, bad, 1.5), (600.0, 200.0, bad)]
        for args in bad_calls:
            with self.assertRaises(ValueError, msg=str(args)):
                reserve_factor(*args)


if __name__ == "__main__":
    unittest.main()
