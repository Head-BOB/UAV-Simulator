# Tier: D
"""Tests for estimates.py. Hand values use the APC 12x6E static fit (Ct 0.100256, Cp 0.031875) and the MN3508 KV700 numbers."""
import json
import math
import unittest
from pathlib import Path

import estimates as E
import partsdb as P

CT, CP, D = 0.100256, 0.031874666666666664, 0.3048
KV, RM, I0 = 700.0, 0.072, 0.5


class ConstantTests(unittest.TestCase):
    def test_density_is_the_core_isa_sea_level_value(self):
        # 101325 / (287.05 * 288.15) = 1.2250122659906946 (the value recorded for the core's air_density(0, standard))
        self.assertAlmostEqual(E.RHO_SL_KGPM3, 1.2250122659906946, places=15)

    def test_gravity(self):
        self.assertEqual(E.G_MPS2, 9.80665)


class MotorPropFormulaTests(unittest.TestCase):
    def test_kt(self):
        # Kt = 60 / (2 pi * 700) = 0.0136418...
        self.assertAlmostEqual(E.kt_nm_per_a(700.0), 60.0 / (2.0 * math.pi * 700.0), places=15)
        self.assertAlmostEqual(E.kt_nm_per_a(700.0), 0.0136419, places=6)  # 60 / 4398.23

    def test_prop_torque_hand_point(self):
        # n = 150 rps: rho Cp = 1.2250123 * 0.0318747 = 0.0390477; n^2 = 22500 -> 878.57; D^5 = 0.3048^5 = 0.00263080;
        # Q = 878.57 * 0.00263080 / (2 pi) = 2.31137 / 6.28319 = 0.367866 N m
        self.assertAlmostEqual(E.prop_torque_nm(CP, 1.2250122659906946, 150.0, D), 0.36787, places=4)

    def test_thrust_hand_point(self):
        # n = 150: Ct rho n^2 D^4 = 0.100256 * 1.2250123 * 22500 * 0.0086309 = 23.85 N (D^4 = 0.3048^4 = 0.00863095)
        self.assertAlmostEqual(E.prop_thrust_n(CT, 1.2250122659906946, 150.0, D), 23.85, places=1)

    def test_current_from_torque(self):
        # I = Q / Kt + I0 = 0.367866 / 0.0136418 + 0.5 = 26.9667 + 0.5 = 27.467 A
        self.assertAlmostEqual(E.motor_current_a(KV, I0, 0.367866), 27.467, places=2)

    def test_kappa_is_the_reference_value(self):
        # shipped quad_ref_build_config.json: torque_coeff 0.015423 = Cp D / (2 pi Ct) with the same fit
        self.assertAlmostEqual(E.kappa_m(CP, D, CT), 0.015423, places=6)


class FullThrottleTests(unittest.TestCase):
    def setUp(self):
        self.s = E.solve_full_throttle(KV, RM, I0, 14.8, CT, CP, D)

    def test_torque_balance_residual_is_tiny(self):
        # At the answer the motor speed from the voltage equation equals the propeller speed.
        n = self.s["rpm"] / 60.0
        cur = E.motor_current_a(KV, I0, E.prop_torque_nm(CP, E.RHO_SL_KGPM3, n, D))
        self.assertAlmostEqual(KV * (14.8 - cur * RM), self.s["rpm"], places=6)

    def test_agrees_with_an_independent_fixed_point_solve(self):
        # Different algorithm: damped fixed-point iteration on rpm = Kv (V - I(rpm) Rm).
        rpm = 5000.0
        for _ in range(2000):
            n = rpm / 60.0
            cur = E.motor_current_a(KV, I0, E.prop_torque_nm(CP, E.RHO_SL_KGPM3, n, D))
            rpm = 0.5 * rpm + 0.5 * KV * (14.8 - cur * RM)
        self.assertAlmostEqual(self.s["rpm"], rpm, places=4)

    def test_hand_estimate_of_the_reference_point(self):
        # Hand iteration: n = 150 rps gives I = 27.47 A -> rpm = 700 (14.8 - 27.47 * 0.072) = 8978 -> converges near 8978 rpm, 27.3 A.
        self.assertAlmostEqual(self.s["rpm"], 8978.0, delta=15.0)
        self.assertAlmostEqual(self.s["current_a"], 27.3, delta=0.3)

    def test_power_bookkeeping(self):
        # electrical = V I; shaft = Q * omega; the difference is copper loss I^2 Rm plus the no-load loss (I0 * back-EMF)
        s = self.s
        self.assertAlmostEqual(s["electrical_w"], 14.8 * s["current_a"], places=9)
        omega = s["rpm"] * 2.0 * math.pi / 60.0
        self.assertAlmostEqual(s["shaft_w"], s["torque_nm"] * omega, places=6)
        back_emf = 14.8 - s["current_a"] * RM
        loss = s["current_a"] ** 2 * RM + I0 * back_emf
        self.assertAlmostEqual(s["electrical_w"], s["shaft_w"] + loss, delta=0.01 * s["electrical_w"])

    def test_zero_load_limit(self):
        # a vanishingly small propeller leaves the motor at its no-load speed Kv (V - I0 Rm)
        s = E.solve_full_throttle(KV, RM, I0, 14.8, CT, CP, 0.005)
        self.assertAlmostEqual(s["rpm"], KV * (14.8 - I0 * RM), delta=20.0)

    def test_monotonic_in_voltage(self):
        a = E.solve_full_throttle(KV, RM, I0, 11.1, CT, CP, D)["rpm"]
        b = E.solve_full_throttle(KV, RM, I0, 14.8, CT, CP, D)["rpm"]
        c = E.solve_full_throttle(KV, RM, I0, 22.2, CT, CP, D)["rpm"]
        self.assertLess(a, b)
        self.assertLess(b, c)

    def test_deterministic_bits(self):
        again = E.solve_full_throttle(KV, RM, I0, 14.8, CT, CP, D)
        self.assertEqual(json.dumps(again), json.dumps(self.s))

    def test_loop_is_bounded(self):
        self.assertEqual(self.s["iterations"], E.BISECTION_STEPS)

    def test_bad_inputs(self):
        for kw in ({"kv": -1}, {"rm_ohm": 0}, {"v_v": float("nan")}, {"d_m": 0}, {"i0_a": -0.1}):
            args = dict(kv=KV, rm_ohm=RM, i0_a=I0, v_v=14.8, ct=CT, cp=CP, d_m=D)
            args.update(kw)
            with self.assertRaises(ValueError):
                E.solve_full_throttle(**args)
        with self.assertRaises(ValueError):
            E.solve_full_throttle(KV, RM, 300.0, 14.8, CT, CP, D)  # I0 Rm = 21.6 V > 14.8 V


class HoverTests(unittest.TestCase):
    def test_hover_throttle_matches_the_rust_formula_value(self):
        # tested in Rust: aero::tests::test_hover_throttle_reference_build_config -> 0.43861 for 1.516 kg, max_rpm 8100
        # hand: m g = 14.86683; D^4 = 0.00863095; (8100/60)^2 = 18225; denominator = 4 * 0.100256 * rho * 18225 * 0.00863095
        self.assertAlmostEqual(E.hover_throttle(1.516, 0.100256, 0.3048, 8100.0, 1.225), 0.43861, places=4)

    def test_hover_throttle_scales_inversely_with_rpm(self):
        a = E.hover_throttle(1.516, CT, D, 8000.0)
        b = E.hover_throttle(1.516, CT, D, 4000.0)
        self.assertAlmostEqual(b / a, 2.0, places=12)

    def test_hover_point_hand_values(self):
        # 1.516 kg: T = 14.86683 / 4 = 3.71671 N per motor; n = sqrt(T / (Ct rho D^4)) = sqrt(3.71671 / 0.00105995...) ...
        # Ct rho D^4 = 0.100256 * 1.2250123 * 0.00863095 = 0.00106000 -> n^2 = 3506.4 -> n = 59.215 rps = 3552.9 rpm
        # Q = Cp rho n^2 D^5 / (2 pi) = 0.0318747 * 1.2250123 * 3506.4 * 0.0026308 / 6.28319 = 0.05732 N m
        # I = 0.05732 / 0.0136418 + 0.5 = 4.702 A (matches the demo's hover current 4.702 A)
        h = E.hover_point(KV, RM, I0, CT, CP, D, 1.516)
        self.assertAlmostEqual(h["rpm"], 3552.9, delta=1.0)
        self.assertAlmostEqual(h["current_a"], 4.702, delta=0.01)
        self.assertAlmostEqual(h["thrust_n"], 3.71671, places=4)

    def test_hover_voltage_closes_the_motor_equation(self):
        h = E.hover_point(KV, RM, I0, CT, CP, D, 1.516)
        self.assertAlmostEqual(KV * (h["terminal_voltage_v"] - h["current_a"] * RM), h["rpm"], places=6)

    def test_pack_current(self):
        # 4 motors at 60 W each, ESC 0.95, avionics 5 W, 14.8 V -> (240 / 0.95 + 5) / 14.8 = 17.4 A
        self.assertAlmostEqual(E.pack_current_a(60.0, 4, 0.95, 14.8, 5.0), (240.0 / 0.95 + 5.0) / 14.8, places=12)
        with self.assertRaises(ValueError):
            E.pack_current_a(60.0, 4, 1.2, 14.8, 0.0)


class StaticFitTests(unittest.TestCase):
    def test_reads_ct_cp_and_follows_scaled_from(self):
        recs, _ = P.load_records(P.DEFAULT_DB)
        measured = E.load_static_fit(recs["apc_12x6e"], recs, P.REPO_ROOT)
        self.assertAlmostEqual(measured["ct"], 0.100256, places=6)
        self.assertEqual(measured["basis"], "UIUC_MEASURED")
        scaled = E.load_static_fit(recs["tmotor_cf_13x4_4"], recs, P.REPO_ROOT)
        self.assertEqual(scaled["basis"], "SCALED_FROM:apc_12x6e")
        self.assertEqual(scaled["ct"], measured["ct"])

    def test_scaled_from_must_be_measured(self):
        recs, _ = P.load_records(P.DEFAULT_DB)
        bad = dict(recs["tmotor_cf_13x4_4"], scaled_from="tmotor_cf_14x4_8")
        with self.assertRaises(ValueError):
            E.load_static_fit(bad, recs, P.REPO_ROOT)


if __name__ == "__main__":
    unittest.main()
