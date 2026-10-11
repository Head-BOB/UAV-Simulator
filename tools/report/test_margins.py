# Tier: D
import tempfile
import unittest
from pathlib import Path

import campaign_data as D
import margins as M
import testkit as K


def load(tests, **kw):
    t = tempfile.TemporaryDirectory()
    c = D.load_campaign(K.write_campaign(Path(t.name) / "c", tests, **kw))
    t.cleanup()
    return c


class MarginTests(unittest.TestCase):
    def test_current_margin_by_hand(self):
        # 25.74 A rating / 23 A peak = 1.119130434...; the example of the plan 23 / 25.74 would be the inverse (load fraction 0.8936)
        self.assertAlmostEqual(M.current_margin(25.74, 23.0), 1.1191304347826, places=12)
        self.assertEqual(M.current_margin(10.0, 0.0), M.MARGIN_CAP)
        self.assertEqual(M.current_margin(1000.0, 1.0), M.MARGIN_CAP)
        with self.assertRaises(M.MarginError):
            M.current_margin(0.0, 1.0)

    def test_rise_margin_by_hand(self):
        # limit 120, peak 70, ambient 20: (120 - 20) / (70 - 20) = 2.0. Absolute ratio 120 / 70 = 1.714 would be wrong
        self.assertEqual(M.rise_margin(120.0, 70.0, 20.0), (2.0, False))
        # a peak at the limit has margin 1; above the limit below 1: (100 - 20) / (120 - 20) = 0.8
        self.assertAlmostEqual(M.rise_margin(100.0, 120.0, 20.0)[0], 0.8, places=12)
        # 0.05 K above ambient is "no heating"
        self.assertEqual(M.rise_margin(120.0, 20.05, 20.0), (M.MARGIN_CAP, True))
        with self.assertRaises(M.MarginError):
            M.rise_margin(20.0, 25.0, 20.0)

    def test_rows_ranking_and_limiting_metric(self):
        c = load({"climb": {}, "endurance": {}})
        rows = M.compute_margins(c, K.POWERTRAIN)
        by = {r["slot"]: r for r in rows}
        # default peaks: motor 20 A, pack 60 A, winding 70, esc 50, battery 30, reserve 2.0, ambient 20
        self.assertAlmostEqual(by["motor_current"]["margin"], 25.74 / 20.0, places=12)
        self.assertAlmostEqual(by["esc_current"]["margin"], 40.0 / 20.0, places=12)        # the ESC carries the motor current
        self.assertAlmostEqual(by["pack_current"]["margin"], 125.0 / 60.0, places=12)
        self.assertAlmostEqual(by["winding_temp"]["margin"], 100.0 / 50.0, places=12)
        self.assertAlmostEqual(by["esc_temp"]["margin"], 80.0 / 30.0, places=12)
        self.assertAlmostEqual(by["battery_temp"]["margin"], 40.0 / 10.0, places=12)
        self.assertEqual(by["thrust_reserve"]["margin"], 2.0)
        # ascending order: 1.287 (motor), 2.0 (esc, winding, reserve: ties by slot id), 2.083, 2.667, 4.0
        self.assertEqual([r["slot"] for r in rows],
                         ["motor_current", "esc_current", "thrust_reserve", "winding_temp", "pack_current", "esc_temp", "battery_temp"])
        self.assertEqual(rows[0]["slot"], "motor_current")

    def test_peak_is_the_maximum_over_tests_and_reserve_the_minimum(self):
        c = load({"climb": {"peaks": {"motor_current_a": 10.0, "min_thrust_reserve_ratio": 3.0}},
                  "endurance": {"peaks": {"motor_current_a": 22.0, "min_thrust_reserve_ratio": 1.5}}})
        by = {r["slot"]: r for r in M.compute_margins(c, K.POWERTRAIN)}
        self.assertEqual((by["motor_current"]["peak"], by["motor_current"]["peak_test"]), (22.0, "endurance"))
        self.assertEqual((by["thrust_reserve"]["peak"], by["thrust_reserve"]["peak_test"]), (1.5, "endurance"))

    def test_uses_the_ambient_of_the_manifest(self):
        cond = dict(K.CONDITIONS, ambient_c=40.0)
        c = load({"climb": {}}, conditions=cond)
        by = {r["slot"]: r for r in M.compute_margins(c, K.POWERTRAIN)}
        self.assertAlmostEqual(by["winding_temp"]["margin"], (120.0 - 40.0) / (70.0 - 40.0), places=12)

    def test_no_heating_flag_and_missing_data(self):
        c = load({"climb": {"peaks": {"battery_c": 20.02}}})
        by = {r["slot"]: r for r in M.compute_margins(c, K.POWERTRAIN)}
        self.assertTrue(by["battery_temp"]["no_heating"])
        self.assertEqual(by["battery_temp"]["margin"], M.MARGIN_CAP)
        bad = dict(K.POWERTRAIN)
        del bad["esc_temp_limit_c"]
        with self.assertRaises(M.MarginError):
            M.compute_margins(c, bad)


if __name__ == "__main__":
    unittest.main()
