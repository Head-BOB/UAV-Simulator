# Tier: D
"""Tests for compat.py: each rule has a passing case, a failing case and its boundary."""
import unittest

import compat as C
from testsupport import synth


def rules(findings, severity=None):
    return sorted(f.rule for f in findings if severity in (None, f.severity))


def perf(**over):
    base = {"motor_id": "m", "esc_id": "e", "battery_id": "b", "frame_id": "f", "hover_throttle": 0.4, "thrust_to_weight": 3.0,
            "full_current_a": 20.0, "hover_current_a": 5.0, "motor_max_current_a": 23.0, "esc_max_current_a": 40.0,
            "pack_limit_a": 100.0, "hover_pack_a": 10.0, "full_pack_a": 80.0, "arm_margin": 5.0}
    base.update(over)
    return base


class ArmsTests(unittest.TestCase):
    def test_four_arms_pass_six_fail(self):
        self.assertEqual(C.arms_finding(synth("frame", arm_count=4)), [])
        f = C.arms_finding(synth("frame", arm_count=6))
        self.assertEqual(rules(f, "error"), ["E-ARMS"])


class ClearanceTests(unittest.TestCase):
    def test_reference_geometry_clears(self):
        # s = 0.4, D = 0.3048: gap 0.0952 (> 5 % of D = 0.01524); inner tip 0.28284 - 0.1524 = 0.13044 > body corner 0.0849
        self.assertEqual(C.clearance_findings("f", "p", 0.4, 0.3048, 0.06 * 2 ** 0.5), [])

    def test_gap_exactly_zero_fails(self):
        # D == s: adjacent tips touch (gap 0) -> error E-CLEAR-TIP
        f = C.clearance_findings("f", "p", 0.4, 0.4, 0.05)
        self.assertIn("E-CLEAR-TIP", rules(f, "error"))
        # a hair less than s leaves a positive gap, which is only a warning
        g = C.clearance_findings("f", "p", 0.4, 0.3999, 0.05)
        self.assertNotIn("E-CLEAR-TIP", rules(g, "error"))

    def test_small_gap_is_a_warning(self):
        # gap 0.4 - 0.39 = 0.01 < 0.05 * 0.39 = 0.0195 -> warning only (inner tip 0.2828 - 0.195 = 0.0878 > 0.05)
        f = C.clearance_findings("f", "p", 0.4, 0.39, 0.05)
        self.assertEqual(rules(f, "warning"), ["E-CLEAR-TIP"])
        self.assertEqual(rules(f, "error"), [])

    def test_gap_at_the_warning_boundary_passes(self):
        # gap = exactly 5 % of D: D = 0.4 / 1.05 = 0.380952..., gap = 0.019048 = 0.05 D -> not below -> no warning
        d = 0.4 / 1.05
        self.assertEqual(rules(C.clearance_findings("f", "p", 0.4, d, 0.05), "warning"), [])

    def test_body_corner_equal_to_the_inner_tip_fails(self):
        inner = 0.4 / 2 ** 0.5 - 0.3048 / 2.0     # the exact inner tip radius
        self.assertEqual(rules(C.clearance_findings("f", "p", 0.4, 0.3048, inner), "error"), ["E-CLEAR-BODY"])

    def test_body_corner_must_be_inside_the_inner_tip(self):
        # inner tip radius = 0.2828427 - 0.1524 = 0.1304427: corner 0.13 clears, 0.1304427 or more does not
        self.assertEqual(rules(C.clearance_findings("f", "p", 0.4, 0.3048, 0.13), "error"), [])
        self.assertEqual(rules(C.clearance_findings("f", "p", 0.4, 0.3048, 0.14), "error"), ["E-CLEAR-BODY"])


class PerformanceTests(unittest.TestCase):
    def test_benign_numbers_give_nothing(self):
        self.assertEqual(C.performance_findings(perf()), [])

    def test_hover_throttle(self):
        self.assertEqual(rules(C.performance_findings(perf(hover_throttle=1.0001)), "error"), ["E-HOVER"])
        self.assertEqual(rules(C.performance_findings(perf(hover_throttle=1.0))), ["W-HOVER-THR"])   # exactly 1.0 is allowed (core: <= 1)
        self.assertEqual(rules(C.performance_findings(perf(hover_throttle=0.7))), [])                # exactly the warn limit: not above
        self.assertEqual(rules(C.performance_findings(perf(hover_throttle=0.7001))), ["W-HOVER-THR"])

    def test_nan_hover_throttle_is_an_error(self):
        self.assertEqual(rules(C.performance_findings(perf(hover_throttle=float("nan"))), "error"), ["E-HOVER"])

    def test_thrust_to_weight(self):
        self.assertEqual(rules(C.performance_findings(perf(thrust_to_weight=0.99)), "error"), ["E-TW"])
        self.assertEqual(rules(C.performance_findings(perf(thrust_to_weight=1.0))), ["W-TW"])
        self.assertEqual(rules(C.performance_findings(perf(thrust_to_weight=2.0))), [])

    def test_full_throttle_currents(self):
        self.assertEqual(rules(C.performance_findings(perf(full_current_a=23.0))), [])                       # equal to the rating: fine
        self.assertEqual(rules(C.performance_findings(perf(full_current_a=23.1))), ["W-FULL-I-MOTOR"])
        self.assertEqual(rules(C.performance_findings(perf(full_current_a=41.0, motor_max_current_a=50.0))), ["W-FULL-I-ESC"])

    def test_hover_motor_current_is_an_error(self):
        self.assertEqual(rules(C.performance_findings(perf(hover_current_a=23.5)), "error"), ["E-HOVER-I-MOTOR"])

    def test_pack_currents(self):
        self.assertEqual(rules(C.performance_findings(perf(hover_pack_a=100.1)), "error"), ["E-HOVER-I"])
        self.assertEqual(rules(C.performance_findings(perf(hover_pack_a=100.0)), "error"), [])               # exactly the rating: warning, not error
        self.assertEqual(rules(C.performance_findings(perf(hover_pack_a=70.0))), [])                         # 70 % exactly: not above
        self.assertEqual(rules(C.performance_findings(perf(hover_pack_a=70.1))), ["W-HOVER-I"])
        self.assertEqual(rules(C.performance_findings(perf(full_pack_a=100.1))), ["W-FULL-I-PACK"])

    def test_arm_margin(self):
        self.assertEqual(rules(C.performance_findings(perf(arm_margin=0.99)), "error"), ["E-ARM-STRESS"])
        self.assertEqual(rules(C.performance_findings(perf(arm_margin=1.0))), ["W-ARM-STRESS"])
        self.assertEqual(rules(C.performance_findings(perf(arm_margin=1.5))), [])
        self.assertEqual(C.performance_findings(perf(arm_margin=None)), [])

    def test_messages_name_the_parts(self):
        f = C.performance_findings(perf(full_current_a=30.0))[0]
        self.assertEqual(f.parts, ("m",))
        self.assertIn("30.0 A", f.message)


class InertiaTests(unittest.TestCase):
    def test_triangle_inequality(self):
        self.assertEqual(C.inertia_finding([0.03, 0.03, 0.055]), [])
        self.assertEqual(rules(C.inertia_finding([0.01, 0.01, 0.05])), ["E-INERTIA"])


class PairTests(unittest.TestCase):
    def parts(self, **over):
        p = {"motor": synth("motor", "m"), "esc": synth("esc", "e"), "propeller": synth("propeller", "p"),
             "battery": synth("battery", "b"), "frame": synth("frame", "f")}
        p.update(over)
        return p

    def test_synthetic_set_is_compatible_except_mount(self):
        # synth motor mount "mount_pattern_x" vs frame "motor_mount_pattern_x": the labels differ, so exactly E-MOUNT
        self.assertEqual(rules(C.pair_findings(self.parts())), ["E-MOUNT"])

    def test_each_rule_id_is_reachable(self):
        ok = self.parts(motor=synth("motor", "m", mount_pattern="A"), frame=synth("frame", "f", motor_mount_pattern="A"))
        self.assertEqual(C.pair_findings(ok), [])
        cases = {
            "E-CELLS-MOTOR": {"battery": synth("battery", "b", cells=6)},
            "E-CELLS-ESC": {"esc": synth("esc", "e", cells_min=5, cells_max=6)},
            "E-ESC-I": {"esc": synth("esc", "e", max_continuous_current_a=10.0)},
            "E-PROP-RANGE": {"propeller": synth("propeller", "p", diameter_m=0.5)},
            "E-SHAFT": {"propeller": synth("propeller", "p", bore_m=0.002)},
            "E-MOUNT": {"frame": synth("frame", "f", motor_mount_pattern="B")},
            "E-FRAME-PROP": {"frame": synth("frame", "f", motor_mount_pattern="A", max_prop_diameter_m=0.25)},
        }
        for rule, over in cases.items():
            with self.subTest(rule=rule):
                p = dict(ok, **over)
                if rule == "E-CELLS-MOTOR":
                    p["esc"] = synth("esc", "e", cells_min=3, cells_max=6)
                self.assertIn(rule, rules(C.pair_findings(p), "error"))


if __name__ == "__main__":
    unittest.main()
