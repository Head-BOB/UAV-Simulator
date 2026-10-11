# Tier: D
"""Tests for the demo sweep profile and sweep_time_plan.py: hand-calculated grids, steps, lattice updates and times."""
import copy
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

import fluidx3d_quad_sweep as Q
import sweep_time_plan as P
import vehicle_sweep as S

ROOT = Path(__file__).resolve().parents[2]
BUILDS = ROOT / "tools" / "parts" / "builds"


class DemoProfileTests(unittest.TestCase):
    def test_default_config_is_untouched_by_the_existence_of_profiles(self):
        cfg = S.load_class_config()
        self.assertNotIn("profile", cfg)
        # 4 speeds x (4 alpha values + 4 beta values at the non-zero angles... = 5 alpha + 4 beta) = 4 x 9 = 36 cases, the committed sample's row count
        self.assertEqual(len(Q.sweep_cases(cfg["classes"]["quad_standard"]["speeds_mps"], cfg["classes"]["quad_standard"]["angles_deg"])), 36)

    def test_demo_profile_case_counts_by_hand(self):
        cfg = S.load_class_config(profile="demo")
        self.assertEqual(cfg["profile"], "demo")
        for name, c in cfg["classes"].items():
            # 3 angles (-a, 0, +a): alpha line has 3 points (beta = 0), beta line adds the 2 non-zero points -> 5 attitudes per speed; 3 speeds -> 15 cases
            self.assertEqual(len(c["speeds_mps"]), 3, name)
            self.assertEqual(len(Q.sweep_cases(c["speeds_mps"], c["angles_deg"])), 15, name)
            self.assertEqual(c["cells_across_span"], 60, name)

    def test_demo_design_identifies_the_five_coefficients_per_axis(self):
        # The model form is F = V^2 (c0 + c1 a + c2 b + c3 a^2 + c4 b^2): 5 unknowns per axis, same coefficients at every speed.
        # Design matrix rows [1, a, b, a^2, b^2] of the 5 attitudes of ONE speed must have rank 5, or the fit would be under-determined.
        cfg = S.load_class_config(profile="demo")
        for name, c in cfg["classes"].items():
            attitudes = {(a, b) for _, a, b in Q.sweep_cases(c["speeds_mps"][:1], c["angles_deg"])}
            self.assertEqual(len(attitudes), 5, name)
            rows = [[1.0, math.radians(a), math.radians(b), math.radians(a) ** 2, math.radians(b) ** 2] for a, b in sorted(attitudes)]
            self.assertEqual(int(np.linalg.matrix_rank(np.array(rows))), 5, name)
            # equations over all speeds: 15 for 5 unknowns -> 10 residual degrees of freedom
            self.assertEqual(len(Q.sweep_cases(c["speeds_mps"], c["angles_deg"])) - 5, 10, name)

    def test_demo_refinement_pair_is_the_default_ratio_at_lower_cost(self):
        cfg = S.load_class_config(profile="demo")
        r = cfg["sanity"]["refinement"]
        self.assertEqual((r["coarse_cells_across_span"], r["fine_cells_across_span"]), (40, 60))
        # the fine case is the demo sweep's own resolution; the ratio 60 / 40 = 1.5 equals the default 100 / 66 = 1.515 to two digits' practice
        self.assertEqual(r["fine_cells_across_span"], cfg["classes"]["quad_standard"]["cells_across_span"])
        self.assertLess(r["speed_index"], len(cfg["classes"]["quad_standard"]["speeds_mps"]))

    def test_unknown_profile_and_unknown_class_are_refused(self):
        with self.assertRaises(S.SweepError):
            S.load_class_config(profile="nope")
        raw = json.loads((S.CLASS_FILE).read_text(encoding="utf-8"))
        raw["profiles"]["demo"]["classes"]["ghost"] = {"cells_across_span": 60}
        with tempfile.TemporaryDirectory() as t:
            p = Path(t) / "c.json"
            p.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(S.SweepError):
                S.load_class_config(p, profile="demo")

    def test_a_profile_value_out_of_range_is_refused_like_any_other(self):
        raw = json.loads((S.CLASS_FILE).read_text(encoding="utf-8"))
        raw["profiles"]["demo"]["classes"]["quad_standard"]["cells_across_span"] = 5
        with tempfile.TemporaryDirectory() as t:
            p = Path(t) / "c.json"
            p.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(S.SweepError):
                S.load_class_config(p, profile="demo")

    def test_demo_cases_generate_for_both_classes_and_record_the_profile(self):
        for build, cls in (("reference_quad", "quad_standard"), ("fpv_5in", "quad_fpv_5in")):
            with tempfile.TemporaryDirectory() as t:
                m = S.generate(BUILDS / build, Path(t) / "s", 55, 6.0, None, None, None, None, "omit", "demo")
                doc = json.loads((Path(t) / "s" / "sweep_manifest.json").read_text(encoding="utf-8"))
                self.assertEqual(len(m), 15, cls)
                self.assertEqual(doc["profile"], "demo")
                # 4 x 60 = 240; 2 x 60 = 120 (both even already)
                self.assertEqual(m[0]["grid"], {"nx": 240, "ny": 120, "nz": 120}, cls)
                # steps = ceil(5 x 240 / 0.075) = 16000
                self.assertEqual(m[0]["total_steps"], 16000, cls)

    def test_a_default_sweep_manifest_carries_no_profile_key(self):
        with tempfile.TemporaryDirectory() as t:
            S.generate(BUILDS / "reference_quad", Path(t) / "s", 55, 6.0, None, None, None, 1, "omit")
            doc = json.loads((Path(t) / "s" / "sweep_manifest.json").read_text(encoding="utf-8"))
            self.assertNotIn("profile", doc)


class TimePlanTests(unittest.TestCase):
    PROFILES = ["default", "demo"]

    def test_lattice_updates_by_hand(self):
        d = P.class_row(P.profile_cfg("default"), "quad_standard")
        m = P.class_row(P.profile_cfg("demo"), "quad_standard")
        # default: 400 x 200 x 200 cells, steps = ceil(5 x 400 / 0.075) = ceil(26666.67) = 26667, updates = 16,000,000 x 26667
        self.assertEqual((d["grid"], d["steps"], d["updates_per_case"], d["cases"]), ((400, 200, 200), 26667, 426_672_000_000, 36))
        # demo: 240 x 120 x 120 = 3,456,000 cells, steps = 5 x 240 / 0.075 = 16000, updates = 55,296,000,000
        self.assertEqual((m["grid"], m["steps"], m["updates_per_case"], m["cases"]), ((240, 120, 120), 16000, 55_296_000_000, 15))

    def test_sanity_case_updates_by_hand(self):
        s = P.sanity_rows(P.profile_cfg("demo"))
        # sphere: 16 x 28 = 448, 8 x 28 = 224, 224 -> 22,478,848 cells; steps = ceil(5 x 448 / 0.075) = ceil(29866.67) = 29867
        self.assertEqual(s["sphere"], 671_375_753_216)
        # coarse 40 cells: 160 x 80 x 80 = 1,024,000 cells, steps = ceil(10666.67) = 10667; fine 60 cells: the demo case above
        self.assertEqual(s["refine_coarse"], 10_923_008_000)
        self.assertEqual(s["refine_fine"], 55_296_000_000)

    def test_totals_from_one_measured_case(self):
        plan = P.build_plan(100.0, "demo", "quad_standard", self.PROFILES)
        by = {(r["profile"], r["class"]): r for r in plan["rows"]}
        meas = by[("demo", "quad_standard")]
        self.assertEqual(meas["basis"], "MEASURED")
        self.assertAlmostEqual(meas["seconds_per_case"], 100.0, places=9)
        self.assertAlmostEqual(meas["total_s"], 1500.0, places=6)            # 15 cases x 100 s
        # the default case does 426.672e9 / 55.296e9 = 7.7161458333 times the work: 771.61458333 s, 36 cases = 27778.125 s (7.72 h)
        d = by[("default", "quad_standard")]
        self.assertTrue(d["basis"].startswith("ASSUMED"))
        self.assertAlmostEqual(d["seconds_per_case"], 771.6145833333, places=6)
        self.assertAlmostEqual(d["total_s"], 27778.125, places=3)
        # the other demo class has the same grid -> same time per case
        self.assertAlmostEqual(by[("demo", "quad_fpv_5in")]["seconds_per_case"], 100.0, places=9)
        # sphere: 671,375,753,216 / 55,296,000,000 x 100 s = 1214.1 s
        self.assertAlmostEqual(plan["sanity"]["demo"]["sphere_s"], 671_375_753_216 / 55_296_000_000 * 100.0, places=6)

    def test_measuring_the_default_profile_scales_the_other_way(self):
        plan = P.build_plan(600.0, "default", "quad_standard", self.PROFILES)
        by = {(r["profile"], r["class"]): r for r in plan["rows"]}
        self.assertEqual(by[("default", "quad_standard")]["basis"], "MEASURED")
        # demo per case = 600 s x 55.296e9 / 426.672e9 = 77.76 s
        self.assertAlmostEqual(by[("demo", "quad_standard")]["seconds_per_case"], 600.0 * 55_296_000_000 / 426_672_000_000, places=6)

    def test_bad_measurements_and_selections_are_refused(self):
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            with self.assertRaises(P.PlanError):
                P.build_plan(bad, "demo", "quad_standard", self.PROFILES)
        with self.assertRaises(P.PlanError):
            P.build_plan(10.0, "nope", "quad_standard", self.PROFILES)
        with self.assertRaises(P.PlanError):
            P.build_plan(10.0, "demo", "ghost", self.PROFILES)

    def test_the_text_labels_what_is_measured_and_what_is_assumed(self):
        text = P.render(P.build_plan(100.0, "demo", "quad_standard", self.PROFILES))
        self.assertIn("MEASURED", text)
        self.assertIn("ASSUMED", text)
        self.assertIn("15", text)
        self.assertIn("5 per body axis", text)
        # 3 speeds x 5 attitudes = 15 equations per axis
        self.assertIn("15 equations per axis for 5 unknowns", text)

    def test_commands_are_in_the_required_order_and_use_the_profile(self):
        cfg = P.profile_cfg("demo")
        cmds = P.commands("tools/parts/builds/reference_quad", "demo", None, cfg, "quad_standard")
        titles = [t for t, _ in cmds]
        order = ["bytes_per_cell", "one timed case", "sphere check", "grid-refinement", "the sweep", "the trainer", "model validation", "into the package"]
        positions = [next(i for i, t in enumerate(titles) if key in t) for key in order]
        self.assertEqual(positions, sorted(positions))
        text = P.render_commands(cmds)
        self.assertIn("<BYTES_PER_CELL>", text)                                 # never a default number for the memory per cell
        self.assertIn("--profile demo", text)
        self.assertIn("generate-sphere", text)
        self.assertIn("generate-refinement", text)
        self.assertIn("--cfd-dir", text)
        self.assertIn("--onnxruntime-zip", text)
        self.assertIn("--aero-dir", text)
        # a given value is used as is
        self.assertIn("--bytes-per-cell 57", P.render_commands(P.commands("b", "demo", 57, cfg, "quad_standard")))
        # the default profile has no --profile flag
        self.assertNotIn("--profile", " ".join(" ".join(lines) for t, lines in P.commands("b", "default", 57, P.profile_cfg("default"), "quad_standard")[2:7]))

    def test_cli_prints_the_table_and_refuses_a_missing_measurement(self):
        self.assertEqual(P.main(["--seconds-per-case", "100", "--measured-profile", "demo", "--no-commands"]), 0)
        self.assertEqual(P.main(["--seconds-per-case", "0", "--no-commands"]), 1)
        self.assertEqual(P.main(["--seconds-per-case", "10", "--measured-profile", "demo", "--profile", "ghost"]), 1)


if __name__ == "__main__":
    unittest.main()
