# Tier: D
"""Hand-calculated tests for the powertrain export (powertrain_config.json) and its place in the builder outputs."""
import copy
import json
import math
import unittest
from pathlib import Path

import build_vehicle as B
import partsdb as P
import powertrain_export as PX
import prop_table as PT

HERE = Path(__file__).resolve().parent
REF = json.loads((HERE / "builds" / "reference_quad.build.json").read_text(encoding="utf-8"))
RECS, _ = P.load_records(P.DEFAULT_DB)


def parts():
    return RECS["tmotor_mn3508_kv700"], RECS["tmotor_air_40a_2_6s"], RECS["apc_12x6e"], RECS["ref_battery_generic_4s_5000"]


VEHICLE = {"prop_diameter_m": 0.3048, "thrust_coeff": 0.100256, "torque_coeff": 0.015423}
CP = 0.031874666666666664


def export(**edits):
    motor, esc, prop, bat = (copy.deepcopy(p) for p in parts())
    for rec, field, value in edits.get("set", []):
        {"motor": motor, "esc": esc, "battery": bat}[rec][field]["value"] = value
    return PX.build_powertrain(motor, esc, prop, bat, VEHICLE, CP)


class ValuesTests(unittest.TestCase):
    def setUp(self):
        self.cfg, self.prov, self.warns = export()

    def test_electrical_values_come_from_the_records(self):
        c = self.cfg
        self.assertEqual((c["motor_kv_rpm_per_v"], c["motor_resistance_ohm"], c["motor_no_load_current_a"]), (700, 0.072, 0.5))
        self.assertEqual((c["motor_max_current_a"], c["motor_winding_temp_limit_c"]), (23, 120))
        self.assertEqual((c["esc_efficiency"], c["esc_max_current_a"], c["esc_temp_limit_c"]), (0.95, 40.0, 100.0))
        self.assertEqual((c["battery_capacity_ah"], c["battery_internal_resistance_ohm"], c["battery_cutoff_v_per_cell"]), (5.0, 0.02, 3.3))
        self.assertEqual((c["battery_cells"], c["battery_temp_limit_c"]), (4, 60.0))

    def test_propeller_constants_match_the_vehicle_config(self):
        # the core refuses a pair that disagrees: Ct and D are copied, kappa = Cp D / (2 pi Ct) must be within 1e-3 of the vehicle's
        c = self.cfg
        self.assertEqual((c["prop_diameter_m"], c["prop_ct"]), (0.3048, 0.100256))
        kappa = c["prop_cp"] * c["prop_diameter_m"] / (2 * math.pi * c["prop_ct"])
        self.assertLess(abs(kappa - VEHICLE["torque_coeff"]) / kappa, 1e-3)

    def test_pack_rating_is_c_times_ah(self):
        self.assertEqual(self.cfg["battery_max_continuous_a"], 25 * 5.0)   # 125 A

    def test_thermal_resistance_split(self):
        # record Rth 0.6 K/W: 0.3 x 0.6 = 0.18 (winding-case) and 0.7 x 0.6 = 0.42 (case-ambient)
        self.assertAlmostEqual(self.cfg["motor_r_winding_case_k_per_w"], 0.18, places=12)
        self.assertAlmostEqual(self.cfg["motor_r_case_ambient_k_per_w"], 0.42, places=12)
        self.assertAlmostEqual(self.cfg["motor_r_winding_case_k_per_w"] + self.cfg["motor_r_case_ambient_k_per_w"], 0.6, places=12)

    def test_heat_capacities_from_mass(self):
        # motor 0.104 kg x 450 = 46.8 J/K, winding 25 % = 11.7, case 75 % = 35.1; ESC 0.030 x 900 = 27; pack 0.45 x 1000 = 450
        c = self.cfg
        self.assertAlmostEqual(c["motor_c_winding_j_per_k"], 11.7, places=12)
        self.assertAlmostEqual(c["motor_c_case_j_per_k"], 35.1, places=12)
        self.assertAlmostEqual(c["esc_c_j_per_k"], 27.0, places=12)
        self.assertAlmostEqual(c["battery_c_j_per_k"], 450.0, places=12)

    def test_no_float_noise_in_the_file(self):
        text = json.dumps(self.cfg)
        self.assertNotIn("99999999", text)
        self.assertNotIn("00000000", text)

    def test_parameters_are_not_verified_while_anything_is_assumed(self):
        self.assertEqual(self.cfg["parameters_verified"], 0)
        self.assertIn("builder_assumptions", self.prov)
        self.assertTrue(any(a["field"] == "thermal_resistance_k_per_w" for a in self.prov["assumed_part_inputs"]))

    def test_ocv_curve_is_copied(self):
        self.assertEqual(len(self.cfg["ocv_curve"]), 8)
        self.assertEqual(self.cfg["ocv_curve"][0], [0.0, 3.3])
        self.assertEqual(self.cfg["ocv_curve"][-1], [1.0, 4.2])

    def test_record_time_constant_mismatch_is_reported_not_used(self):
        # record tau 150 s, mass-based R x C = 0.6 x 46.8 = 28.08 s: factor 5.34 > 2
        self.assertEqual(len(self.warns), 1)
        self.assertIn("5.3", self.warns[0])
        self.assertAlmostEqual(self.prov["derived"]["motor_tau_from_rc_s"], 28.08, places=9)
        self.assertEqual(self.prov["derived"]["motor_tau_record_s"], 150.0)

    def test_consistent_time_constant_gives_no_warning(self):
        _, _, warns = export(set=[("motor", "thermal_time_constant_s", 28.0)])
        self.assertEqual(warns, [])

    def test_boundary_of_the_tau_warning(self):
        # factor exactly 2 is not a warning, just above is
        _, _, ok = export(set=[("motor", "thermal_time_constant_s", 2 * 28.08)])
        _, _, bad = export(set=[("motor", "thermal_time_constant_s", 2.01 * 28.08)])
        self.assertEqual((len(ok), len(bad)), (0, 1))


class StabilityTests(unittest.TestCase):
    def test_smallest_time_constant_hand_value(self):
        # worst case cooling: 100 m/s + 20 m/s wash -> 1 + 24^0.8 = 13.70; ESC/battery: 1 + 20^0.8 = 11.99
        # case: 0.42 / 13.70 x 35.1 = 1.076 s (smallest); winding: 0.18 x 11.7 = 2.106; ESC 2/11.99 x 27 = 4.50; battery 112.6
        cfg, _, _ = export()
        self.assertAlmostEqual(PX.smallest_time_constant_s(cfg), 1.076, delta=0.002)

    def test_too_fast_network_is_refused(self):
        # dt 0.01 needs tau_min >= 0.05 s: a 0.001 J/K case capacity gives 0.42/13.7 x 0.00075 = 2e-5 s
        motor, esc, prop, bat = (copy.deepcopy(p) for p in parts())
        motor["mass_kg"]["value"] = 1e-6
        with self.assertRaises(PX.PowertrainExportError):
            PX.build_powertrain(motor, esc, prop, bat, VEHICLE, CP)

    def test_long_ocv_curve_refused(self):
        motor, esc, prop, bat = (copy.deepcopy(p) for p in parts())
        bat["ocv_curve"]["value"] = [[i / 16.0, 3.3 + 0.05 * i] for i in range(17)]
        with self.assertRaises(PX.PowertrainExportError):
            PX.build_powertrain(motor, esc, prop, bat, VEHICLE, CP)

    def test_constants_match_the_rust_core(self):
        src = (P.REPO_ROOT / "physics_core" / "src" / "powertrain.rs").read_text(encoding="utf-8")
        self.assertIn(f"const COOLING_EXPONENT: f64 = {PX.COOLING_EXPONENT};", src)
        self.assertIn(f"const THERMAL_STEP_FRACTION: f64 = {PX.THERMAL_STEP_FRACTION};", src)
        types_src = (P.REPO_ROOT / "physics_core" / "src" / "types.rs").read_text(encoding="utf-8")
        self.assertIn(f"pub const MAX_OCV_POINTS: usize = {PX.MAX_OCV_POINTS};", types_src)


class BuilderIntegrationTests(unittest.TestCase):
    def test_builder_writes_the_fourth_file_and_reports_it(self):
        res = B.build(REF)
        self.assertTrue(res.ok, res.findings)
        self.assertEqual(sorted(B.render(res)), ["assembly.json", "controller_config.json", "powertrain_config.json", "vehicle_build.json", "vehicle_config.json"])
        self.assertEqual(res.powertrain["battery_cells"], 4)
        self.assertIn("W-THERMAL-TAU", {f.rule for f in res.findings})
        self.assertIn("powertrain", res.sidecar)

    def test_hover_duty_estimate_hand_value(self):
        # hover terminal voltage 5.4514 V on a 14.8 V pack -> duty 0.3683 (NOT the config's hover_throttle 0.398, which is an rpm fraction)
        res = B.build(REF)
        self.assertAlmostEqual(res.sidecar["hover_duty_estimate"], 5.4514 / 14.8, places=4)

    def test_powertrain_ct_equals_the_vehicle_ct_exactly(self):
        res = B.build(REF)
        self.assertEqual(res.powertrain["prop_ct"], res.config["thrust_coeff"])
        self.assertEqual(res.powertrain["prop_diameter_m"], res.config["prop_diameter_m"])


class PropellerTableTests(unittest.TestCase):
    REAL_JSON = json.loads((P.REPO_ROOT / "offline_pipeline/props/apce_12x6.json").read_text(encoding="utf-8"))

    def test_reference_build_carries_the_uiuc_table(self):
        res = B.build(REF)
        curves = res.powertrain["prop_curves"]
        self.assertEqual([c["rpm"] for c in curves], [3040.0, 4030.0, 5024.5, 6018.5])
        self.assertEqual(curves, PT.build_curves(self.REAL_JSON))
        self.assertEqual(res.sidecar["powertrain"]["propeller_model"], "table")

    def test_sidecar_reports_the_table_range_and_the_hover_difference(self):
        m = B.build(REF).sidecar["powertrain"]["propeller_table"]
        self.assertEqual(m["measured_rpm_range"], [3040.0, 6018.5])
        self.assertFalse(m["stand_in"])
        self.assertFalse(m["hover_outside_data"])
        # hover Ct from the table is about 1 % below the static fit 0.100256 (hand estimate 0.09927 at 3553 rpm)
        self.assertAlmostEqual(m["hover_ct_table"], 0.0993, delta=5e-4)
        self.assertAlmostEqual(m["hover_ct_table_minus_static_percent"], -0.95, delta=0.5)
        self.assertEqual(len(m["source_sha256"]), 64)

    def test_scaled_propeller_is_marked_a_stand_in_and_outside_the_data(self):
        spec = {"name": "fpv_test", "vehicle_class": "quad_fpv_5in",
                "parts": {"frame": "holybro_kopis2_220_frame", "motor": "tmotor_velox_v2207_v3_kv1750", "esc": "holybro_tekko32_f4_4in1_50a",
                          "propeller": "tmotor_t5143s_prop", "battery": "tattu_rline_v6_1300_6s_160c", "flight_controller": "holybro_kakute_h7_v2"}}
        res = B.build(spec)
        self.assertTrue(res.ok, res.findings)
        m = res.sidecar["powertrain"]["propeller_table"]
        self.assertTrue(m["stand_in"])
        self.assertEqual(m["basis"], "SCALED_FROM:apc_12x6e")
        # a 5 in prop spins far faster than the 12x6E data (6018.5 rpm): the whole operating range is held at the edge and flagged
        self.assertTrue(m["hover_outside_data"])
        self.assertGreater(m["hover_rpm_estimate"], 6018.5)

    def test_compact_format_is_valid_json_and_round_trips(self):
        res = B.build(REF)
        text = PX.format_powertrain(res.powertrain)
        self.assertEqual(json.loads(text), res.powertrain)
        self.assertLess(text.count("\n"), 400)               # one point per line, not one number per line
        self.assertTrue(text.endswith("}\n"))
        self.assertEqual(text, PX.format_powertrain(res.powertrain))

    def test_without_curves_the_static_model_is_kept(self):
        cfg, prov, _ = export()
        self.assertNotIn("prop_curves", cfg)
        self.assertEqual(prov["propeller_model"], "static")

    def test_table_values_are_not_tidied(self):
        # computed blend values keep full precision; only the scalar electrical/thermal numbers are rounded to 12 significant digits
        curves = B.build(REF).powertrain["prop_curves"]
        self.assertTrue(any(len(repr(p[1])) > 14 for c in curves for p in c["points"]))


if __name__ == "__main__":
    unittest.main()
