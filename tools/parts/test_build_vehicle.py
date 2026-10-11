# Tier: D
"""End-to-end tests for build_vehicle: the reference build against hand calculations, refusal paths, determinism, CLI."""
import contextlib
import copy
import io
import json
import math
import shutil
import tempfile
import unittest
from pathlib import Path

import build_vehicle as B
import estimates as E
import partsdb as P

HERE = Path(__file__).resolve().parent
REF_SPEC = json.loads((HERE / "builds" / "reference_quad.build.json").read_text(encoding="utf-8"))
DEMO_SPEC = json.loads((HERE / "builds" / "reference_quad_demo_esc.build.json").read_text(encoding="utf-8"))
REAL = P.DEFAULT_DB


class DbCopy:
    """A throw-away copy of the real database (+ the UIUC prop file) that tests may edit."""

    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self._tmp.name)
        shutil.copytree(REAL, self.repo / "parts_db", ignore=shutil.ignore_patterns("source_cache", "parts_index.sqlite"))
        (self.repo / "offline_pipeline" / "props").mkdir(parents=True)
        shutil.copy(P.REPO_ROOT / "offline_pipeline/props/apce_12x6.json", self.repo / "offline_pipeline/props/apce_12x6.json")
        self.root = self.repo / "parts_db"

    def edit(self, part_id, fn):
        recs, _ = P.load_records(self.root)
        rec = copy.deepcopy(recs[part_id])
        fn(rec)
        (self.root / "parts" / rec["category"] / f"{part_id}.json").write_text(P.canonical_json(rec), encoding="utf-8")

    def close(self):
        self._tmp.cleanup()


def rules(res, severity=None):
    return sorted({f.rule for f in res.findings if severity in (None, f.severity)})


class ReferenceBuildTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = B.build(REF_SPEC)
        cls.cfg = cls.res.config

    def test_builds_with_the_expected_warning_only(self):
        self.assertTrue(self.res.ok, self.res.findings)
        self.assertEqual(rules(self.res, "error"), [])
        self.assertIn("W-FULL-I-MOTOR", rules(self.res, "warning"))   # ~27 A estimated vs the 23 A (180 s) rating

    def test_config_has_exactly_the_core_fields_in_order(self):
        self.assertEqual(list(self.cfg), ["mass_kg", "inertia_kgm2", "motor_pos_m", "motor_spin", "prop_diameter_m",
                                          "thrust_coeff", "torque_coeff", "max_rpm"])

    def test_mass_hand_sum(self):
        # frame 0.35 + wiring 0.12 + battery 0.45 + 4 x (motor 0.104 + AIR 40A ESC 0.030 + prop 0.020 = 0.154) = 1.536 kg
        self.assertAlmostEqual(self.cfg["mass_kg"], 1.536, places=9)

    def test_motor_positions_relative_to_the_cg(self):
        # (+-0.2, +-0.2) from the 0.4 m spacing; z = propeller plane (-0.042) minus CG z (+0.010026) = -0.052026
        for (x, y, z), (sx, sy) in zip(self.cfg["motor_pos_m"], [(1, 1), (1, -1), (-1, 1), (-1, -1)]):
            self.assertAlmostEqual(x, 0.2 * sx, places=5)   # CG x is ~0 up to the battery/ESC layout asymmetry
            self.assertAlmostEqual(y, 0.2 * sy, places=6)
            self.assertAlmostEqual(z, -0.052026, places=5)
        self.assertEqual(self.cfg["motor_spin"], [1.0, -1.0, -1.0, 1.0])

    def test_cg_height_hand_calculation(self):
        # sum(m z)/M: motors 4 x 0.104 x -0.02125 = -0.00884; ESCs 4 x 0.03 x -0.02125 = -0.00255; props 4 x 0.02 x -0.042 = -0.00336;
        # battery 0.45 x (0.05 + 0.017 = 0.067) = +0.03015; frame, arms, wiring at z = 0 -> 0.0154 / 1.536 = 0.0100260
        cg = self.res.sidecar["mass_properties"]["cg_from_frame_centre_m"]
        self.assertAlmostEqual(cg[2], 0.0154 / 1.536, places=7)
        self.assertAlmostEqual(cg[1], 0.0, places=12)   # left-right symmetric

    def test_izz_hand_calculation(self):
        # Every term below is written out from the shapes; none calls the code under test.
        pi = math.pi
        arm_m = 1600.0 * (pi / 4 * (0.016 ** 2 - 0.013 ** 2)) * (0.4 * math.sqrt(2) / 2)       # 0.0309225 kg each
        body_m = 0.35 - 4 * arm_m                                                              # 0.22631 kg
        izz = body_m * (0.2 ** 2 + 0.2 ** 2) / 12                                              # central box 0.2 x 0.2 x 0.1
        L = 0.4 * math.sqrt(2) / 2
        izz += 4 * (arm_m * L ** 2 / 12 + arm_m * (L / 2) ** 2)                                # rod about its CG + parallel axis (r = L/2)
        r2 = 0.2 ** 2 + 0.2 ** 2
        izz += 4 * (0.104 * r2 + 0.104 * (0.0418 / 2) ** 2 / 2)                                 # motor: m r2 + solid cylinder m rad^2 / 2
        izz += 4 * 0.03 * r2 + 4 * 0.03 * (0.06 ** 2 + 0.025 ** 2) / 12                         # four ESC boxes (0.06 x 0.025 x 0.01) at the motors
        izz += 4 * (0.02 * r2 + 0.02 * (0.3048 / 2) ** 2 / 2)                                   # props: m r2 + thin disc m rad^2 / 2
        izz += 0.45 * (0.138 ** 2 + 0.043 ** 2) / 12                                            # battery box, on the axis
        self.assertAlmostEqual(self.cfg["inertia_kgm2"][2], izz, delta=2e-6)

    def test_inertia_is_symmetric_enough_and_positive(self):
        ixx, iyy, izz = self.cfg["inertia_kgm2"]
        self.assertTrue(0 < ixx < izz and 0 < iyy < izz)
        # cross-check against the old lumped estimate is reported, not asserted equal
        x = self.res.sidecar["inertia_cross_check_old_lumped"]
        self.assertFalse(x["asserted_equal"])
        self.assertEqual(len(x["new_minus_old_percent"]), 3)

    def test_propeller_constants(self):
        self.assertEqual(self.cfg["prop_diameter_m"], 0.3048)
        self.assertAlmostEqual(self.cfg["thrust_coeff"], 0.100256, places=6)
        self.assertAlmostEqual(self.cfg["torque_coeff"], 0.015423, places=6)   # same as the shipped reference config

    def test_max_rpm_is_the_motor_prop_balance(self):
        # hand iteration at 14.8 V (Kv 700, Rm 0.072, I0 0.5, Cp 0.0318747): converges near 8981 rpm
        self.assertAlmostEqual(self.cfg["max_rpm"], 8981.4, delta=1.0)
        solved = E.solve_full_throttle(700.0, 0.072, 0.5, 14.8, 0.100256, 0.031874666666666664, 0.3048)
        self.assertAlmostEqual(self.cfg["max_rpm"], round(solved["rpm"], 3), places=9)

    def test_hover_throttle_uses_the_rounded_config_values(self):
        c = self.cfg
        expect = E.hover_throttle(c["mass_kg"], c["thrust_coeff"], c["prop_diameter_m"], c["max_rpm"])
        self.assertAlmostEqual(self.res.sidecar["estimates"]["hover_throttle"], expect, places=15)
        self.assertAlmostEqual(expect, 0.3982, places=4)

    def test_thrust_to_weight_hand_value(self):
        # 4 x 23.752 N / (1.536 kg x 9.80665) = 95.007 / 15.0630 = 6.307
        self.assertAlmostEqual(self.res.sidecar["estimates"]["thrust_to_weight"], 4 * 23.7518 / (1.536 * 9.80665), places=3)
        self.assertAlmostEqual(self.res.sidecar["estimates"]["thrust_to_weight"], 6.307, places=2)

    def test_pack_currents_hand_formula(self):
        # (4 x motor electrical power / ESC efficiency + avionics) / pack voltage; the reference build has no powered avionics
        e = self.res.sidecar["estimates"]
        hover_w = e["hover_point"]["electrical_w"]
        self.assertAlmostEqual(e["pack_hover_a"], (4 * hover_w / 0.95) / 14.8, places=9)
        self.assertAlmostEqual(e["pack_full_a"], (4 * e["full_throttle"]["electrical_w"] / 0.95) / 14.8, places=9)

    def test_arm_bending_hand_value(self):
        # full-throttle thrust 23.752 N x arm 0.282843 m = 6.7183 N m; Z = 2.268758e-7 -> 29.61 MPa
        s = self.res.sidecar["arm"]["max_bending_stress_pa_at_full_throttle"]
        self.assertAlmostEqual(s / 1e6, 29.61, delta=0.05)

    def test_sidecar_lists_assumed_inputs_and_parts(self):
        sc = self.res.sidecar
        self.assertGreater(len(sc["assumed_inputs"]), 0)
        self.assertTrue(any(a["part_id"] == "ref_frame_quad_400" and a["field"] == "mass_kg" for a in sc["assumed_inputs"]))
        self.assertEqual({p["role"] for p in sc["parts"]}, {"frame", "motor", "esc", "propeller", "battery", "wiring"})
        self.assertEqual(sc["database"]["schema_version"], "1.0.0")

    def test_assumed_list_has_a_todo_for_every_entry(self):
        for a in self.res.sidecar["assumed_inputs"]:
            self.assertIn("TODO(HUMAN-VERIFY", a["note"], a)

    def test_assembly_matches_the_config(self):
        asm = self.res.assembly
        total = sum(p["mass_kg"] for p in asm["parts"])
        self.assertAlmostEqual(total, self.cfg["mass_kg"], places=5)
        self.assertAlmostEqual(asm["totals"]["mass_kg"], total, places=9)
        self.assertEqual(sum(1 for p in asm["parts"] if p["role"] == "motor"), 4)
        self.assertEqual(sum(1 for p in asm["parts"] if p["role"] == "propeller"), 4)
        self.assertEqual(sum(1 for p in asm["parts"] if p["part_id"] == "ref_frame_quad_400" and p["instance"].startswith("arm_")), 4)
        self.assertTrue(all(p["mesh"] == "stand_in" for p in asm["parts"]))
        spins = [p["visual"]["spin"] for p in asm["parts"] if p["role"] == "propeller"]
        self.assertEqual(spins, [1.0, -1.0, -1.0, 1.0])


class RefusalTests(unittest.TestCase):
    def test_demo_esc_is_refused_with_the_rule_id(self):
        res = B.build(DEMO_SPEC)
        self.assertFalse(res.ok)
        self.assertIn("E-ESC-I", rules(res, "error"))
        self.assertIsNone(res.config)

    def test_unknown_id_wrong_category_missing_part(self):
        spec = copy.deepcopy(REF_SPEC)
        spec["parts"]["motor"] = "no_such_motor"
        spec["parts"]["esc"] = "apc_12x6e"            # a propeller in the esc slot
        del spec["parts"]["battery"]
        res = B.build(spec)
        msgs = " | ".join(f.message for f in res.findings)
        self.assertFalse(res.ok)
        self.assertEqual(rules(res, "error"), ["E-BUILD"])
        self.assertIn("unknown part id 'no_such_motor'", msgs)
        self.assertIn("is a propeller, not a esc", msgs)
        self.assertIn("required part 'battery' is missing", msgs)

    def test_unknown_keys_and_class(self):
        spec = dict(copy.deepcopy(REF_SPEC), surprise=1, vehicle_class="hexacopter")
        res = B.build(spec)
        self.assertEqual(rules(res, "error"), ["E-BUILD"])
        self.assertTrue(any("surprise" in f.message for f in res.findings))

    def test_frame_class_must_match(self):
        spec = copy.deepcopy(REF_SPEC)
        spec["vehicle_class"] = "quad_fpv_5in"
        res = B.build(spec)
        self.assertIn("E-CLASS", rules(res, "error"))

    def test_hex_frame_refused(self):
        db = DbCopy()
        self.addCleanup(db.close)
        db.edit("ref_frame_quad_400", lambda r: r["arm_count"].update(value=6))
        res = B.build(REF_SPEC, db.root)
        self.assertIn("E-ARMS", rules(res, "error"))
        self.assertFalse(res.ok)

    def test_prop_overlap_refused(self):
        db = DbCopy()
        self.addCleanup(db.close)
        db.edit("ref_frame_quad_400", lambda r: r["motor_spacing_adjacent_m"].update(value=0.30))
        res = B.build(REF_SPEC, db.root)
        self.assertIn("E-CLEAR-TIP", rules(res, "error"))

    def test_hover_impossible_refused(self):
        db = DbCopy()
        self.addCleanup(db.close)
        db.edit("ref_frame_quad_400", lambda r: r["mass_kg"].update(value=2.0))   # arms stay 0.124 kg, still fine
        db.edit("ref_avionics_wiring_lump", lambda r: r["mass_kg"].update(value=30.0))
        res = B.build(REF_SPEC, db.root)
        self.assertFalse(res.ok)
        self.assertTrue({"E-HOVER", "E-TW"} & set(rules(res, "error")), res.findings)

    def test_bad_override_and_payload(self):
        spec = copy.deepcopy(REF_SPEC)
        spec["position_override_m"] = {"ref_frame_quad_400": [0.0, float("nan"), 0.0], "not_in_build": [0, 0, 0]}
        res = B.build(spec)
        self.assertEqual(rules(res, "error"), ["E-BUILD"])
        self.assertEqual(sum("position_override_m" in f.message for f in res.findings), 2)
        spec = copy.deepcopy(REF_SPEC)
        spec["payload"] = {"id": "apc_12x6e"}
        self.assertIn("E-BUILD", rules(B.build(spec), "error"))

    def test_payload_role_in_parts_is_refused_with_a_clear_message(self):
        # A payload is a test condition (mass, position), not a vehicle part. It used to surface as "unknown role 'payload'".
        spec = copy.deepcopy(REF_SPEC)
        spec["parts"]["payload"] = "payload_test_mass_500g"
        res = B.build(spec)
        self.assertFalse(res.ok)
        self.assertEqual(rules(res, "error"), ["E-PAYLOAD"])
        msg = next(f.message for f in res.findings if f.rule == "E-PAYLOAD")
        self.assertIn("payload is a test condition, not a vehicle part", msg)
        self.assertIn("Conditions", msg)
        self.assertFalse(any("unknown role" in f.message for f in res.findings))

    def test_fifteen_inch_prop_on_a_220_frame_is_an_error_at_the_tip_gap(self):
        # 5 in frame + MN3110 + 15x5 prop on 6S. Hand value: gap = adjacent motor spacing - D, both from the records
        # (frame 0.1556 m, prop 15 in = 0.381 m), so the props overlap by 0.2254 m.
        spec = {"name": "absurd", "vehicle_class": "quad_fpv_5in",
                "parts": {"frame": "holybro_kopis2_220_frame", "motor": "tmotor_mn3110_kv700", "esc": "holybro_tekko32_f4_4in1_50a",
                          "propeller": "tmotor_cf_15x5", "battery": "tattu_rline_v6_1300_6s_160c"}}
        recs, _ = P.load_records(REAL)
        spacing = recs["holybro_kopis2_220_frame"]["motor_spacing_adjacent_m"]["value"]
        d_m = recs["tmotor_cf_15x5"]["diameter_m"]["value"]
        res = B.build(spec)
        self.assertFalse(res.ok)
        self.assertIsNone(res.config)
        self.assertTrue({"E-CLEAR-TIP", "E-CLEAR-BODY", "E-FRAME-PROP", "E-PROP-RANGE", "E-MOUNT"} <= set(rules(res, "error")), res.findings)
        tip = next(f for f in res.findings if f.rule == "E-CLEAR-TIP")
        self.assertEqual(tip.severity, "error")
        self.assertIn(f"{spacing - d_m:.4f} m", tip.message)
        self.assertLess(spacing - d_m, 0.0)

    def test_corrupt_database_blocks_the_build(self):
        db = DbCopy()
        self.addCleanup(db.close)
        db.edit("tmotor_mn3508_kv700", lambda r: r["kv_rpm_per_v"].pop("provenance"))
        res = B.build(REF_SPEC, db.root)
        self.assertIn("E-DB", rules(res, "error"))

    def test_esc_channel_count_other_than_1_or_4_refused(self):
        db = DbCopy()
        self.addCleanup(db.close)
        db.edit("tmotor_air_40a_2_6s", lambda r: r["channels"].update(value=2))
        res = B.build(REF_SPEC, db.root)
        self.assertFalse(res.ok)
        self.assertTrue(any("channels must be 1 or 4" in f.message for f in res.findings))

    def test_wide_body_corner_refused_using_the_sqrt2(self):
        # half width 0.095 -> corner 0.095 x sqrt2 = 0.13435 > inner tip radius 0.13044 (a missing sqrt2 would give 0.095 and pass)
        db = DbCopy()
        self.addCleanup(db.close)
        db.edit("ref_frame_quad_400", lambda r: r["body_half_width_m"].update(value=0.095))
        res = B.build(REF_SPEC, db.root)
        self.assertIn("E-CLEAR-BODY", rules(res, "error"))

    def test_missing_central_box_is_a_layout_error(self):
        db = DbCopy()
        self.addCleanup(db.close)
        db.edit("ref_frame_quad_400", lambda r: r.pop("central_body_box_m"))
        res = B.build(REF_SPEC, db.root)
        self.assertIn("E-LAYOUT", rules(res, "error"))


class PayloadAndVariantTests(unittest.TestCase):
    def test_payload_moves_the_cg(self):
        base = B.build(REF_SPEC)
        spec = copy.deepcopy(REF_SPEC)
        spec["payload"] = {"id": "payload_test_mass_500g", "position_m": [0.1, 0.0, 0.05]}
        res = B.build(spec)
        self.assertTrue(res.ok, res.findings)
        # M = 1.536 + 0.5 = 2.036; payload at x = 0.1 shifts the CG x by 0.5 * 0.1 / 2.036 = 0.0245580
        cg_x = res.sidecar["mass_properties"]["cg_from_frame_centre_m"][0]
        base_x = base.sidecar["mass_properties"]["cg_from_frame_centre_m"][0]
        self.assertAlmostEqual(res.config["mass_kg"], 2.036, places=9)
        self.assertAlmostEqual(cg_x, (base_x * 1.536 + 0.5 * 0.1) / 2.036, places=9)
        # the motors are measured from the CG, so the front motors move closer by the same amount
        self.assertAlmostEqual(res.config["motor_pos_m"][0][0], 0.2 - cg_x, places=5)

    def test_payload_default_slot_is_below_the_battery(self):
        spec = copy.deepcopy(REF_SPEC)
        spec["payload"] = {"id": "payload_test_mass_250g"}
        res = B.build(spec)
        pl = next(p for p in res.assembly["parts"] if p["role"] == "payload")
        side = (0.25 / 1000.0) ** (1.0 / 3.0)
        # belly slot: z = body_h/2 + battery_h + c/2 = 0.05 + 0.034 + side/2
        self.assertAlmostEqual(pl["position_m"][2], 0.05 + 0.034 + side / 2, places=5)

    def test_override_moves_a_part(self):
        spec = copy.deepcopy(REF_SPEC)
        spec["position_override_m"] = {"ref_battery_generic_4s_5000": [0.05, 0.0, 0.067]}
        res = B.build(spec)
        # battery 0.45 kg moved 0.05 m forward: CG x = 0.45 * 0.05 / 1.536 = 0.0146484
        self.assertAlmostEqual(res.sidecar["mass_properties"]["cg_from_frame_centre_m"][0], 0.45 * 0.05 / 1.536, places=7)

    def test_four_in_one_esc_and_optional_parts(self):
        spec = copy.deepcopy(REF_SPEC)
        spec["parts"]["esc"] = "holybro_tekko32_f4_4in1_50a"
        spec["parts"].update({"flight_controller": "holybro_pixhawk_6c", "receiver": "radiomaster_rp1_v2", "gps": "holybro_m10_gps",
                              "pdb": "holybro_pm02d", "camera": "runcam_phoenix_2", "gimbal": "siyi_a8_mini"})
        res = B.build(spec)
        self.assertTrue(res.ok, res.findings)
        escs = [p for p in res.assembly["parts"] if p["role"] == "esc"]
        self.assertEqual([p["instance"] for p in escs], ["4in1"])
        # mass: 0.35 + 0.12 + 0.45 + 4 x (0.104 + 0.02) + 0.0138 (4-in-1) + FC 0.0593 + RX 0.0022 + GPS 0.032 + PDB 0.02 + cam 0.009 + gimbal 0.095
        expect = 0.35 + 0.12 + 0.45 + 4 * 0.124 + 0.0138 + 0.0593 + 0.0022 + 0.032 + 0.02 + 0.009 + 0.095
        self.assertAlmostEqual(res.config["mass_kg"], expect, places=5)

    def test_avionics_power_enters_the_pack_current(self):
        spec = copy.deepcopy(REF_SPEC)
        spec["parts"].update({"flight_controller": "holybro_pixhawk_6c", "receiver": "radiomaster_rp1_v2", "gps": "holybro_m10_gps",
                              "pdb": "holybro_pm02d", "camera": "runcam_phoenix_2", "gimbal": "siyi_a8_mini"})
        res = B.build(spec)
        # avionics W = 0.3 x 5 (FC) + 0.1 x 5 (RX) + 0.2 x 5 (GPS) + 0.05 x 14.8 (PDB) + 0.2 x 5 (camera) + 0.337838 x 14.8 (gimbal, 5 W / 14.8 V)
        avionics_w = 1.5 + 0.5 + 1.0 + 0.74 + 1.0 + 0.337838 * 14.8
        e = res.sidecar["estimates"]
        self.assertAlmostEqual(e["pack_hover_a"], (4 * e["hover_point"]["electrical_w"] / 0.95 + avionics_w) / 14.8, delta=0.002)
        self.assertAlmostEqual(e["pack_full_a"], (4 * e["full_throttle"]["electrical_w"] / 0.95 + avionics_w) / 14.8, delta=0.002)

    def test_motor_visuals_carry_the_spin(self):
        spins = [p["visual"]["spin"] for p in B.build(REF_SPEC).assembly["parts"] if p["role"] == "motor"]
        self.assertEqual(spins, [1.0, -1.0, -1.0, 1.0])

    def test_fpv_build_class(self):
        spec = {"name": "fpv_test", "vehicle_class": "quad_fpv_5in",
                "parts": {"frame": "holybro_kopis2_220_frame", "motor": "tmotor_velox_v2207_v3_kv1750", "esc": "holybro_tekko32_f4_4in1_50a",
                          "propeller": "tmotor_t5143s_prop", "battery": "tattu_rline_v6_1300_6s_160c", "flight_controller": "holybro_kakute_h7_v2"}}
        res = B.build(spec)
        self.assertTrue(res.ok, res.findings)
        self.assertEqual(res.sidecar["estimates"]["ct_cp_basis"], "SCALED_FROM:apc_12x6e")
        self.assertTrue(any(a["field"] == "ct_cp_scaling_assumed" for a in res.sidecar["assumed_inputs"]))


class DeterminismAndFilesTests(unittest.TestCase):
    def test_two_builds_are_byte_identical(self):
        a, b = B.render(B.build(REF_SPEC)), B.render(B.build(REF_SPEC))
        self.assertEqual(a, b)

    def test_golden_outputs_are_current(self):
        stale = B.check_outputs(B.build(REF_SPEC), HERE / "builds" / "reference_quad")
        self.assertEqual(stale, [], "run: python tools/parts/build_vehicle.py tools/parts/builds/reference_quad.build.json --out tools/parts/builds/reference_quad")

    def test_fpv_golden_outputs_are_current(self):
        spec = json.loads((HERE / "builds" / "fpv_5in.build.json").read_text(encoding="utf-8"))
        stale = B.check_outputs(B.build(spec), HERE / "builds" / "fpv_5in")
        self.assertEqual(stale, [], "run: python tools/parts/build_vehicle.py tools/parts/builds/fpv_5in.build.json --out tools/parts/builds/fpv_5in")

    def test_check_detects_stale_and_missing(self):
        res = B.build(REF_SPEC)
        with tempfile.TemporaryDirectory() as t:
            out = Path(t)
            self.assertEqual(sorted(B.check_outputs(res, out)), ["assembly.json", "controller_config.json", "powertrain_config.json", "vehicle_build.json", "vehicle_config.json"])
            B.write_outputs(res, out)
            self.assertEqual(B.check_outputs(res, out), [])
            (out / "vehicle_config.json").write_text("{}", encoding="utf-8")
            self.assertEqual(B.check_outputs(res, out), ["vehicle_config.json"])

    def test_config_text_is_valid_json_with_lf_only(self):
        text = B.render(B.build(REF_SPEC))["vehicle_config.json"]
        self.assertNotIn("\r", text)
        self.assertEqual(json.loads(text), B.build(REF_SPEC).config)


def run_cli(args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = B.main(args)
    return code, out.getvalue(), err.getvalue()


class CliTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.out = Path(self._tmp.name) / "out"

    def test_build_then_check(self):
        code, out, _ = run_cli([str(HERE / "builds/reference_quad.build.json"), "--out", str(self.out)])
        self.assertEqual(code, 0)
        self.assertIn("built reference_quad: mass 1.536 kg", out)
        code, out, _ = run_cli([str(HERE / "builds/reference_quad.build.json"), "--out", str(self.out), "--check"])
        self.assertEqual(code, 0)
        self.assertTrue(out.strip().endswith("build outputs are up to date"), out)
        (self.out / "assembly.json").write_text("{}", encoding="utf-8")
        code, _, err = run_cli([str(HERE / "builds/reference_quad.build.json"), "--out", str(self.out), "--check"])
        self.assertEqual(code, 2)
        self.assertIn("assembly.json", err)

    def test_refused_build_writes_nothing(self):
        code, _, err = run_cli([str(HERE / "builds/reference_quad_demo_esc.build.json"), "--out", str(self.out)])
        self.assertEqual(code, 1)
        self.assertIn("E-ESC-I", err)
        self.assertIn("nothing written", err)
        self.assertFalse(self.out.exists())

    def test_unreadable_build_file(self):
        code, _, err = run_cli([str(Path(self._tmp.name) / "missing.json"), "--out", str(self.out)])
        self.assertEqual(code, 1)
        self.assertIn("cannot read build file", err)


if __name__ == "__main__":
    unittest.main()
