# Tier: D
import math
import unittest

import power as P


def series(rows):
    cols = {}
    for r in rows:
        for k, v in r.items():
            cols.setdefault(k, []).append(v)
    return cols


class PowerTests(unittest.TestCase):
    def test_shaft_power_by_hand(self):
        # 4 motors x 0.2 N m x 3000 rpm x 2 pi / 60 = 4 x 0.2 x 314.159265 = 251.327412 W
        row = {"t_s": 0.0, "battery_w": 300.0, "motor_copper_w": 0, "motor_noload_w": 0, "esc_loss_w": 0}
        for m in range(1, 5):
            row[f"torque_nm_{m}"] = 0.2
            row[f"rpm_{m}"] = 3000.0
        s = series([row])
        self.assertAlmostEqual(P.shaft_power_w(s, 0), 251.32741228718345, places=9)
        b = P.breakdown(s)
        # losses 300 - 251.327412 = 48.672588 W = 16.2241960 % of 300
        self.assertAlmostEqual(b["loss_total_w"], 48.67258771281655, places=9)
        self.assertAlmostEqual(b["percent_of_pack"]["loss_total"], 16.22419590427, places=8)
        self.assertAlmostEqual(b["percent_of_pack"]["shaft"] + b["percent_of_pack"]["loss_total"], 100.0, places=9)

    def test_split_adds_up_and_remainder_is_what_is_left(self):
        row = {"t_s": 0.0, "battery_w": 300.0, "motor_copper_w": 20.0, "motor_noload_w": 10.0, "esc_loss_w": 6.0}
        for m in range(1, 5):
            row[f"torque_nm_{m}"] = 0.2
            row[f"rpm_{m}"] = 3000.0
        b = P.breakdown(series([row]))
        self.assertAlmostEqual(b["other_w"], 300.0 - 251.32741228718345 - 36.0, places=9)
        total = b["shaft_w"] + b["motor_copper_w"] + b["motor_noload_w"] + b["esc_loss_w"] + b["other_w"]
        self.assertAlmostEqual(total, b["pack_w"], places=9)

    def test_time_average_with_unequal_steps(self):
        # y = 0 at t=0, 10 at t=1, 10 at t=4: trapezoids 0.5 x 10 x 1 = 5 and 10 x 3 = 30; mean = 35 / 4 = 8.75
        self.assertAlmostEqual(P.time_average([0.0, 1.0, 4.0], [0.0, 10.0, 10.0]), 8.75, places=12)
        self.assertEqual(P.time_average([2.0], [7.0]), 7.0)
        for t, y in (([0.0, 0.0], [1.0, 1.0]), ([1.0, 0.0], [1.0, 1.0]), ([], []), ([0.0, 1.0], [1.0])):
            with self.assertRaises(P.PowerError):
                P.time_average(t, y)

    def test_float_noise_in_the_remainder_is_zero(self):
        row = {"t_s": 0.0, "battery_w": 251.32741228718345 + 1e-12, "motor_copper_w": 0, "motor_noload_w": 0, "esc_loss_w": 0}
        for m in range(1, 5):
            row[f"torque_nm_{m}"] = 0.2
            row[f"rpm_{m}"] = 3000.0
        self.assertEqual(P.breakdown(series([row]))["other_w"], 0.0)

    def test_zero_pack_power_gives_no_percent(self):
        row = {"t_s": 0.0, "battery_w": 0.0, "motor_copper_w": 0, "motor_noload_w": 0, "esc_loss_w": 0}
        for m in range(1, 5):
            row[f"torque_nm_{m}"] = 0.0
            row[f"rpm_{m}"] = 0.0
        self.assertIsNone(P.breakdown(series([row]))["percent_of_pack"]["shaft"])
        self.assertTrue(math.isclose(P.TWO_PI_OVER_60, 0.10471975511965977))


if __name__ == "__main__":
    unittest.main()
