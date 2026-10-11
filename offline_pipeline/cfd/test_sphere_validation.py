# Tier: D
"""Tests for sphere_validation: reference drag values by hand, icosphere geometry, lattice numbers, report statuses and deviations."""
import json
import math
import tempfile
import unittest
from pathlib import Path

import sphere_validation as SV

S = SV.S
ROOT = Path(__file__).resolve().parents[2]
REF = ROOT / "tools" / "parts" / "builds" / "reference_quad"
CFG = S.load_class_config()


class ReferenceTests(unittest.TestCase):
    def test_schiller_naumann_by_hand(self):
        # Re = 1000: 1000^0.687 = exp(0.687 ln 1000) = exp(4.745569) = 115.0716; 24/1000 (1 + 0.15 x 115.0716) = 0.024 x 18.26074 = 0.438258...
        self.assertAlmostEqual(SV.schiller_naumann_cd(1000.0), 0.024 * (1.0 + 0.15 * math.exp(0.687 * math.log(1000.0))), places=12)
        self.assertAlmostEqual(SV.schiller_naumann_cd(1000.0), 0.438288140019997, places=12)
        # Re = 100: 100^0.687 = 23.6600 ; 0.24 (1 + 3.5490) = 1.09173
        self.assertAlmostEqual(SV.schiller_naumann_cd(100.0), 1.0917310910948732, places=12)
        # Re = 1: 24 (1 + 0.15) = 27.6
        self.assertAlmostEqual(SV.schiller_naumann_cd(1.0), 27.6, places=12)
        for bad in (0.0, -5.0, float("nan"), float("inf")):
            with self.assertRaises(S.SweepError):
                SV.schiller_naumann_cd(bad)


class IcosphereTests(unittest.TestCase):
    def test_triangle_counts(self):
        for s in range(4):
            self.assertEqual(len(SV.icosphere(1.0, s).t), 20 * 4 ** s)

    def test_icosahedron_volume_by_hand(self):
        # inscribed regular icosahedron, circumradius R = 1: edge a = 1 / sin(2 pi / 5) = 1.0514622, V = 5 (3 + sqrt 5) / 12 a^3 = 2.536151
        a = 1.0 / math.sin(2.0 * math.pi / 5.0)
        want = 5.0 * (3.0 + math.sqrt(5.0)) / 12.0 * a ** 3
        self.assertAlmostEqual(S.G.signed_volume(SV.icosphere(1.0, 0)), want, places=12)
        self.assertAlmostEqual(want, 2.5361507, places=6)

    def test_volume_approaches_the_sphere(self):
        sphere = 4.0 / 3.0 * math.pi
        ratios = [S.G.signed_volume(SV.icosphere(1.0, s)) / sphere for s in range(4)]
        self.assertTrue(all(b > a for a, b in zip(ratios, ratios[1:])))
        self.assertTrue(0.60 < ratios[0] < 0.61 and 0.97 < ratios[3] < 1.0)
        self.assertAlmostEqual(S.G.signed_volume(SV.icosphere(0.05, 3)) / (4.0 / 3.0 * math.pi * 0.05 ** 3), ratios[3], places=12)   # scales with R^3

    def test_vertices_lie_on_the_sphere(self):
        m = SV.icosphere(2.5, 3)
        self.assertTrue(all(abs(math.sqrt(sum(c * c for c in v)) - 2.5) < 1e-12 for v in m.v))


class CaseTests(unittest.TestCase):
    def setUp(self):
        self.memory = {"bytes_per_cell": 55, "vram_bytes": 6 * 1024 ** 3, "headroom_fraction": 0.15}

    def test_sphere_case_numbers_by_hand(self):
        c = SV.sphere_case(CFG, self.memory)
        # D = 0.1 m in 28 cells: dx = 0.1 / 28; U = Re nu / D = 1000 x 1.4607e-5 / 0.1 = 0.14607 m/s; nu_lb = u_lb D_cells / Re = 0.075 x 28 / 1000 = 0.0021
        self.assertAlmostEqual(c["dx_m"], 0.1 / 28, places=12)
        self.assertAlmostEqual(c["speed_mps"], 0.14607, places=12)
        self.assertAlmostEqual(c["nu_lb"], 0.0021, places=12)
        self.assertAlmostEqual(c["tau"], 0.5063, places=12)
        # domain 16 x 8 x 8 diameters = 448 x 224 x 224 = 22,478,848 cells; steps ceil(5 x 448 / 0.075) = 29867; sphere at 4 D = 112 cells
        self.assertEqual((c["grid"]["nx"], c["grid"]["ny"], c["grid"]["nz"]), (448, 224, 224))
        self.assertEqual(c["total_steps"], 29867)
        self.assertEqual(c["lattice_updates"], 448 * 224 * 224 * 29867)
        self.assertEqual(c["cg_cells"], [112.0, 112.0, 112.0])
        self.assertAlmostEqual(c["ref_area_m2"], math.pi * 0.1 ** 2 / 4.0, places=15)
        self.assertAlmostEqual(c["expected_cd"], 0.438288140019997, places=12)
        self.assertIn("UNVERIFIED", c["expected_cd_source"])

    def test_sphere_case_refusals(self):
        with self.assertRaises(S.SweepError):
            SV.sphere_case(CFG, {**self.memory, "bytes_per_cell": 400})
        big = json.loads(json.dumps(CFG))
        big["sanity"]["sphere"]["diameter_cells"] = 40            # 640 x 320 x 320 = 65.5 M cells
        with self.assertRaises(S.SweepError):
            SV.sphere_case(big, self.memory)

    def test_sphere_cd_by_hand(self):
        # q = 0.5 x 1.225 x 0.14607^2 = 0.0130688; A = pi (0.1)^2 / 4 = 7.853982e-3; F = 0.5 x q x A = 5.13209e-5 N gives Cd = 0.5
        case = {"speed_mps": 0.14607, "ref_area_m2": math.pi * 0.01 / 4.0}
        f = 0.5 * 0.5 * 1.225 * 0.14607 ** 2 * math.pi * 0.01 / 4.0
        self.assertAlmostEqual(SV.sphere_cd(case, f"{f!r},0,0,0,0,0,0,1,1", 1.225), 0.5, places=12)
        for bad in ("-1.0,0,0,0,0,0,0,1,1", "0.0,0,0,0,0,0,0,1,1", "nan,0,0,0,0,0,0,1,1", "1,2,3"):
            with self.assertRaises((S.SweepError, ValueError)):
                SV.sphere_cd(case, bad, 1.225)


class ReportTests(unittest.TestCase):
    def test_not_run_when_nothing_exists(self):
        r = SV.build_report(None, None)
        self.assertEqual((r["status"], r["sphere"]["status"], r["refinement"]["status"]), ("NOT_RUN", "NOT_RUN", "NOT_RUN"))
        self.assertIn("NOT been run", r["statement"])
        with tempfile.TemporaryDirectory() as t:
            self.assertEqual(SV.build_report(Path(t), Path(t))["status"], "NOT_RUN")        # directories without manifests

    def test_generated_but_not_run_is_still_not_run(self):
        with tempfile.TemporaryDirectory() as t:
            sph = Path(t) / "sph"
            SV.generate_sphere(sph, 55, 6.0)
            ref = Path(t) / "ref"
            SV.generate_refinement(REF, ref, 55, 6.0)
            r = SV.build_report(sph, ref)
            self.assertEqual((r["status"], r["sphere"]["status"], r["refinement"]["status"]), ("NOT_RUN", "NOT_RUN", "NOT_RUN"))
            self.assertAlmostEqual(r["sphere"]["expected_cd"], 0.438288140019997, places=12)

    def test_sphere_folder_is_runnable_and_collectable(self):
        with tempfile.TemporaryDirectory() as t:
            sph = Path(t) / "sph"
            c = SV.generate_sphere(sph, 55, 6.0)
            self.assertIn("vehicle_sweep.py", (sph / "run_sweep.ps1").read_text(encoding="utf-8"))
            self.assertTrue((sph / c["case_id"] / "setup.cpp").exists() and (sph / c["case_id"] / "vehicle.stl").exists())
            (sph / c["case_id"] / "raw_result.csv").write_text(S.RAW_HEADER + "\n5.0,0,0,0,0,0,0.1,1,1\n", encoding="utf-8")
            out = S.collect(sph)                          # the script's last step must not crash on a sphere sweep
            self.assertIn("sphere_re1000", out.read_text(encoding="utf-8"))

    def test_sphere_run_gives_the_deviation_and_a_partial_status(self):
        with tempfile.TemporaryDirectory() as t:
            sph = Path(t) / "sph"
            c = SV.generate_sphere(sph, 55, 6.0)
            f = 0.5 * 0.5 * 1.225 * c["speed_mps"] ** 2 * c["ref_area_m2"]          # Cd = 0.5
            (sph / c["case_id"] / "raw_result.csv").write_text(S.RAW_HEADER + f"\n{f!r},0,0,0,0,0,0.1,29867,7466\n", encoding="utf-8")
            r = SV.build_report(sph, None)
            self.assertEqual((r["status"], r["sphere"]["status"]), ("PARTIAL", "RUN"))
            self.assertAlmostEqual(r["sphere"]["measured_cd"], 0.5, places=9)
            # (0.5 - 0.438288140019997) / 0.438288140019997 = +14.0799 %
            self.assertAlmostEqual(r["sphere"]["deviation_percent"], 100.0 * (0.5 - 0.438288140019997) / 0.438288140019997, places=9)
            self.assertIn("no tolerance", r["provenance"])

    def test_refinement_change_by_hand_and_full_run(self):
        with tempfile.TemporaryDirectory() as t:
            sph, ref = Path(t) / "sph", Path(t) / "ref"
            c = SV.generate_sphere(sph, 55, 6.0)
            SV.generate_refinement(REF, ref, 55, 6.0)
            f = 0.5 * 0.5 * 1.225 * c["speed_mps"] ** 2 * c["ref_area_m2"]
            (sph / c["case_id"] / "raw_result.csv").write_text(S.RAW_HEADER + f"\n{f!r},0,0,0,0,0,0.1,1,1\n", encoding="utf-8")
            # alpha = beta = 0: body fx = -fx_sim, and q, A_ref are shared, so cx_coarse : cx_fine = -10 : -11 -> change = (-11 - (-10)) / 11 x 100 = -9.0909 %
            for label, force in (("coarse", 10.0), ("fine", 11.0)):
                case = json.loads((ref / label / "sweep_manifest.json").read_text(encoding="utf-8"))["cases"][0]
                (ref / label / case["case_id"] / "raw_result.csv").write_text(S.RAW_HEADER + f"\n{force},0,0,0,0,0,0.1,1,1\n", encoding="utf-8")
            r = SV.build_report(sph, ref)
            self.assertEqual((r["status"], r["refinement"]["status"]), ("RUN", "RUN"))
            self.assertAlmostEqual(r["refinement"]["drag_coefficient_change_percent"], -100.0 / 11.0, places=9)
            self.assertEqual(r["refinement"]["cases"]["coarse"]["cells_across_span"], 66)
            self.assertEqual(r["refinement"]["cases"]["fine"]["cells_across_span"], 100)
            self.assertIn("Both checks have been run", r["statement"])

    def test_refinement_with_one_missing_result_stays_not_run(self):
        with tempfile.TemporaryDirectory() as t:
            ref = Path(t) / "ref"
            SV.generate_refinement(REF, ref, 55, 6.0)
            case = json.loads((ref / "coarse" / "sweep_manifest.json").read_text(encoding="utf-8"))["cases"][0]
            (ref / "coarse" / case["case_id"] / "raw_result.csv").write_text(S.RAW_HEADER + "\n10.0,0,0,0,0,0,0.1,1,1\n", encoding="utf-8")
            self.assertEqual(SV.build_report(None, ref)["refinement"]["status"], "NOT_RUN")

    def test_refinement_pair_uses_the_middle_speed_zero_angle_and_the_stated_resolutions(self):
        with tempfile.TemporaryDirectory() as t:
            cases = SV.generate_refinement(REF, Path(t), 55, 6.0)
            self.assertEqual([c[0] for c in cases], ["coarse", "fine"])
            for _, cells, case in cases:
                self.assertEqual((case["speed_mps"], case["alpha_deg"], case["beta_deg"]), (10.0, 0.0, 0.0))
                self.assertAlmostEqual(case["span_m"] / case["dx_m"], cells, places=9)
            self.assertAlmostEqual(cases[1][1] / cases[0][1], 100 / 66, places=12)


class CliTests(unittest.TestCase):
    def test_cli(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            self.assertEqual(SV.main(["generate-sphere", "--out", str(d / "s"), "--bytes-per-cell", "55"]), 0)
            self.assertEqual(SV.main(["generate-refinement", "--build", str(REF), "--out", str(d / "r"), "--bytes-per-cell", "55"]), 0)
            self.assertEqual(SV.main(["report", "--sphere", str(d / "s"), "--refinement", str(d / "r"), "--out", str(d / "rep.json")]), 0)
            self.assertEqual(json.loads((d / "rep.json").read_text(encoding="utf-8"))["status"], "NOT_RUN")
            self.assertEqual(SV.main(["generate-sphere", "--out", str(d / "x"), "--bytes-per-cell", "9000"]), 1)


if __name__ == "__main__":
    unittest.main()
