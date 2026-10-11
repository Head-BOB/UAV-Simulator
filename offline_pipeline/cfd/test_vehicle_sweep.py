# Tier: D
"""Tests for vehicle_sweep: hand-calculated grid / lattice numbers, VRAM and domain refusals, rotation and CG placement, collector, synthetic sample."""
import copy
import csv
import io
import json
import math
import tempfile
import unittest
from pathlib import Path

import vehicle_sweep as S

ROOT = Path(__file__).resolve().parents[2]
BUILDS = ROOT / "tools" / "parts" / "builds"
REF = BUILDS / "reference_quad"
FPV = BUILDS / "fpv_5in"
CFG = S.load_class_config()
HERE_SAMPLE = Path(__file__).resolve().parent / "sample"


def ctx(build=REF, bytes_per_cell=55, cells=None, props="omit"):
    return S.prepare(build, bytes_per_cell, 6.0, cells, props, CFG)


class ConfigTests(unittest.TestCase):
    def test_committed_config_loads_and_is_declared_assumed(self):
        self.assertEqual(CFG["provenance"]["source_type"], "ASSUMED")
        self.assertEqual(sorted(CFG["classes"]), ["quad_fpv_5in", "quad_standard"])
        self.assertEqual(CFG["domain"]["max_cells"], 300 ** 3)

    def test_refusals(self):
        def bad(edit):
            c = copy.deepcopy(CFG)
            edit(c)
            with tempfile.TemporaryDirectory() as t:
                p = Path(t) / "c.json"
                p.write_text(json.dumps(c), encoding="utf-8")
                with self.assertRaises(S.SweepError):
                    S.load_class_config(p)
        bad(lambda c: c.update(schema_version=2))
        bad(lambda c: c["provenance"].update(source_type="CITED"))
        bad(lambda c: c["lbm"].update(velocity=0.5))
        bad(lambda c: c["domain"].update(spans_xyz=[4.0, 0.0, 2.0]))
        bad(lambda c: c["domain"].update(vram_headroom_fraction=1.0))
        bad(lambda c: c["classes"]["quad_standard"].update(speeds_mps=[0.0]))
        bad(lambda c: c["classes"]["quad_standard"].update(angles_deg=[95.0]))
        bad(lambda c: c["classes"]["quad_standard"].update(cells_across_span=5))


class NumbersTests(unittest.TestCase):
    def test_grid_by_hand(self):
        # span 0.4 m, dx 0.004: 4 x 100 = 400, 2 x 100 = 200 -> 400 x 200 x 200 = 16,000,000 cells
        self.assertEqual(S.grid_for(0.4, 0.004, (4.0, 2.0, 2.0)), (400, 200, 200))
        # span 0.45, dx 0.004: 450, 225 -> rounded up to the even 226 (the demo sweep's 450 x 226 x 226 = 22,984,200 cells)
        self.assertEqual(S.grid_for(0.45, 0.004, (4.0, 2.0, 2.0)), (450, 226, 226))

    def test_grid_rounds_a_fractional_cell_count_up_to_an_even_number(self):
        # span 0.4, dx 0.0042: 4 x 95.238 = 380.95 -> 381 -> 382; 2 x 95.238 = 190.48 -> 191 -> 192
        self.assertEqual(S.grid_for(0.4, 0.0042, (4.0, 2.0, 2.0)), (382, 192, 192))

    def test_steps_by_hand(self):
        # 5 flow-throughs x 400 cells / 0.075 = 26666.67 -> 26667 steps
        self.assertEqual(S.total_steps(400, CFG["lbm"]), 26667)
        self.assertEqual(S.total_steps(450, CFG["lbm"]), 30000)

    def test_lattice_viscosity_and_tau_by_hand(self):
        # nu_lb = nu u_lb / (V dx) = 1.4607e-5 x 0.075 / (10 x 0.004) = 2.7388125e-5; tau = 3 nu_lb + 0.5 = 0.500082164
        nu = S.lattice_viscosity(1.4607e-5, 0.075, 10.0, 0.004)
        self.assertAlmostEqual(nu, 2.7388125e-5, places=14)
        self.assertAlmostEqual(3 * nu + 0.5, 0.500082164375, places=12)

    def test_reference_quad_case_numbers(self):
        c = ctx()
        case = S.make_case(5.0, 0.0, 0.0, c)
        # span = 2 x 0.2199 = 0.4398 m, 100 cells -> dx 0.004398; Re = 5 x 0.4398 / 1.4607e-5 = 150544.3
        self.assertAlmostEqual(case["span_m"], 0.4398, places=9)
        self.assertAlmostEqual(case["dx_m"], 0.004398, places=9)
        self.assertEqual((case["grid"]["nx"], case["grid"]["ny"], case["grid"]["nz"]), (400, 200, 200))
        self.assertAlmostEqual(case["reynolds"], 5.0 * 0.4398 / 1.4607e-5, places=3)
        self.assertEqual(case["lattice_updates"], 400 * 200 * 200 * 26667)
        # nu_lb = 1.4607e-5 x 0.075 / (5 x 0.004398) = 4.98192e-5, tau = 3 nu_lb + 0.5 = 0.50014946
        self.assertAlmostEqual(case["nu_lb"], 1.4607e-5 * 0.075 / (5.0 * 0.004398), places=14)
        self.assertAlmostEqual(case["tau"], 0.5 + 3.0 * 1.4607e-5 * 0.075 / (5.0 * 0.004398), places=14)
        self.assertAlmostEqual(case["mach_lb"], 0.075 * math.sqrt(3.0), places=12)
        self.assertEqual(case["cg_cells"], [100.0, 100.0, 100.0])
        self.assertEqual(case["vram_required_bytes"], 16_000_000 * 55)
        self.assertEqual(case["propellers"], "omit")

    def test_bbox_centre_offset_from_the_cg_for_zero_angles(self):
        # R = diag(-1, -1, 1) at alpha = beta = 0. Mesh z range -0.049 .. 0.083, CG z = 0.0100260417: centre offset
        # (0.017 - 0.0100260417) / 0.004398 = 1.585711 cells; x and y ranges are symmetric about the CG (x, y of the CG are 0)
        case = S.make_case(5.0, 0.0, 0.0, ctx())
        bx, by, bz = case["bbox_centre_from_cg_cells"]
        self.assertAlmostEqual(bx, 0.0, places=6)
        self.assertAlmostEqual(by, 0.0, places=6)
        self.assertAlmostEqual(bz, 1.5857113081703802, places=6)

    def test_fpv_case_lies_in_the_domain(self):
        case = S.make_case(40.0, -40.0, 0.0, ctx(FPV))
        self.assertEqual(case["grid"], {"nx": 400, "ny": 200, "nz": 200})
        self.assertGreater(case["nu_lb"], CFG["lbm"]["min_lattice_viscosity"])


class PlacementTests(unittest.TestCase):
    def test_place_mesh_translates_to_the_cg_rotates_and_scales(self):
        m = S.G.Mesh()
        m.add_box((1.0, 2.0, 3.0), (0.2, 0.2, 0.2))
        rows = S.Q.body_to_sim_rotation(0.0, 0.0)   # diag(-1, -1, 1)
        placed, centre, lo, hi = S.place_mesh(m, rows, (1.0, 2.0, 2.9), 0.1)
        # box centre relative to the CG = (0, 0, 0.1) -> (0, 0, 1) cells; x and y are mirrored but symmetric
        self.assertAlmostEqual(centre[0], 0.0, places=12)
        self.assertAlmostEqual(centre[1], 0.0, places=12)
        self.assertAlmostEqual(centre[2], 1.0, places=12)
        self.assertAlmostEqual(hi[0] - lo[0], 2.0, places=12)       # 0.2 m / 0.1 = 2 cells
        # the nose of the vehicle (body +x) points upstream: a body point 0.3 m ahead of the CG sits at -3 cells in x_sim
        m2 = S.G.Mesh()
        m2.v = [(1.3, 2.0, 3.0), (1.0, 2.0, 3.0), (1.0, 2.1, 3.0)]
        m2.t = [(0, 1, 2)]
        p2, *_ = S.place_mesh(m2, rows, (1.0, 2.0, 3.0), 0.1)
        self.assertAlmostEqual(p2.v[0][0], -3.0, places=12)


class RefusalTests(unittest.TestCase):
    def test_too_many_cells_is_refused(self):
        # 150 cells across: 600 x 300 x 300 = 54,000,000 > 27,000,000
        with self.assertRaises(S.SweepError) as e:
            S.make_case(5.0, 0.0, 0.0, ctx(cells=150))
        self.assertIn("exceeds", str(e.exception))

    def test_vram_formula_refuses(self):
        # 16,000,000 cells x 400 B = 6.4e9 B > 6 GiB x 0.85 = 5.48e9 B
        with self.assertRaises(S.SweepError) as e:
            S.make_case(5.0, 0.0, 0.0, ctx(bytes_per_cell=400))
        self.assertIn("allowed", str(e.exception))
        S.make_case(5.0, 0.0, 0.0, ctx(bytes_per_cell=300))      # 4.8e9 B fits

    def test_lattice_viscosity_floor_refuses(self):
        # V = 1000 m/s: nu_lb = 1.4607e-5 x 0.075 / (1000 x 0.004398) = 2.5e-7 < 5e-6
        with self.assertRaises(S.SweepError) as e:
            S.make_case(1000.0, 0.0, 0.0, ctx())
        self.assertIn("lattice viscosity", str(e.exception))

    def test_unknown_class_and_geometry_errors_are_sweep_errors(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            a = json.loads((REF / "assembly.json").read_text(encoding="utf-8"))
            a["vehicle_class"] = "hexacopter"
            (d / "assembly.json").write_text(json.dumps(a), encoding="utf-8")
            with self.assertRaises(S.SweepError):
                S.prepare(d, 55, 6.0, None, "omit", CFG)
            b = json.loads((REF / "assembly.json").read_text(encoding="utf-8"))
            next(p for p in b["parts"] if p["instance"] == "pack")["position_m"] = [0.0, 0.0, 0.0]
            (d / "assembly.json").write_text(json.dumps(b), encoding="utf-8")
            with self.assertRaises(S.SweepError):
                S.prepare(d, 55, 6.0, None, "omit", CFG)

    def test_a_vehicle_that_does_not_fit_the_domain_is_refused(self):
        c = ctx()
        c["cfg"] = copy.deepcopy(CFG)
        c["cfg"]["domain"]["spans_xyz"] = [4.0, 1.0, 1.0]       # only 100 cells across: the 100-cell vehicle plus the margin does not fit
        with self.assertRaises(S.SweepError) as e:
            S.make_case(5.0, 0.0, 0.0, c)
        self.assertIn("does not fit the domain", str(e.exception))
        # one side only: the vehicle starts 0.2 spans from the inlet, but about 0.5 span of it lies upstream of its CG
        c2 = ctx()
        c2["cfg"] = copy.deepcopy(CFG)
        c2["cfg"]["domain"]["inlet_spans"] = 0.2
        with self.assertRaises(S.SweepError) as e2:
            S.make_case(5.0, 0.0, 0.0, c2)
        self.assertIn("axis 0", str(e2.exception))


class GenerateAndCollectTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.out = Path(cls.tmp.name) / "sweep"
        cls.cases = S.generate(REF, cls.out, 55, 6.0, None, None, None, 3)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_files_manifest_and_hashes(self):
        self.assertEqual(len(self.cases), 3)
        doc = json.loads((self.out / "sweep_manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(doc["provenance"], "preliminary CFD, unvalidated")
        self.assertEqual(doc["vehicle_class"], "quad_standard")
        geo = json.loads((self.out / "geometry.json").read_text(encoding="utf-8"))
        self.assertEqual(doc["geometry_sha256"], geo["stl_sha256"])
        for case in doc["cases"]:
            d = self.out / case["case_id"]
            self.assertTrue((d / "vehicle.stl").exists() and (d / "setup.cpp").exists() and (d / "case.json").exists())
            import hashlib
            self.assertEqual(hashlib.sha256((d / "vehicle.stl").read_bytes()).hexdigest(), case["stl_sha256"])
        self.assertIn("vehicle_sweep.py", (self.out / "run_sweep.ps1").read_text(encoding="utf-8"))

    def test_setup_has_no_placeholders_and_carries_the_numbers(self):
        case = self.cases[0]
        text = (self.out / case["case_id"] / "setup.cpp").read_text(encoding="utf-8")
        self.assertNotIn("@@", text)
        self.assertIn("Nx=400u, Ny=200u, Nz=200u", text)
        self.assertIn("TODO(HUMAN-VERIFY: matches FluidX3D 3.8)", text)
        self.assertIn("slice_y_mid.f32", text)
        self.assertIn(f"{case['total_steps']}ull", text)

    def test_check_reports_fit_and_refuses(self):
        p = self.out / self.cases[0]["case_id"] / "case.json"
        self.assertIn("fits", S.check_case(p))
        self.assertIn("fits", S.check_case(p, free_vram_mib=5000))
        with self.assertRaises(S.SweepError):
            S.check_case(p, free_vram_mib=100)          # 839 MiB needed
        # 839 MiB needed, 15 % headroom kept: 900 MiB free leaves 765 MiB, which is not enough; 1000 MiB leaves 850 MiB
        with self.assertRaises(S.SweepError):
            S.check_case(p, free_vram_mib=900)
        self.assertIn("fits", S.check_case(p, free_vram_mib=1000))
        bad = json.loads(p.read_text(encoding="utf-8"))
        bad["memory"]["bytes_per_cell"] = 4000
        q = Path(self.tmp.name) / "bad_case.json"
        q.write_text(json.dumps(bad), encoding="utf-8")
        with self.assertRaises(S.SweepError):
            S.check_case(q)

    def test_collect_row_hand_values(self):
        # alpha = beta = 0: R = diag(-1, -1, 1), body force = R^T f_sim = (-fx, -fy, fz)
        # q = 0.5 x 1.225 x 10^2 = 61.25 Pa, A_ref 0.025 m^2, L_ref 0.44 m: C = F / (q A) = F / 1.53125, Cm = M / (q A L) = M / 0.67375
        case = {"case_id": "t", "speed_mps": 10.0, "alpha_deg": 0.0, "beta_deg": 0.0, "ref_area_m2": 0.025, "ref_length_m": 0.44,
                "rotation_rows": [list(r) for r in S.Q.body_to_sim_rotation(0.0, 0.0)]}
        r = S.collect_row(case, "10.0,0.5,-2.0,0.01,0.02,0.03,0.1,26667,6666", 1.225)
        self.assertEqual(tuple(round(x, 12) for x in r["f"]), (-10.0, -0.5, -2.0))
        self.assertEqual(tuple(round(x, 12) for x in r["m"]), (-0.01, -0.02, 0.03))
        for got, want in zip(r["c"], (-6.530612244897959, -0.32653061224489793, -1.3061224489795917)):
            self.assertAlmostEqual(got, want, places=12)
        for got, want in zip(r["cm"], (-0.014842300556586271, -0.029684601113172542, 0.04452690166975881)):
            self.assertAlmostEqual(got, want, places=12)

    def test_collect_row_rotates_with_the_angle_of_attack(self):
        # alpha = 20 deg: the air direction is e1 = -(cos a, 0, sin a) in the body frame. A drag of 10 N along +x_sim is 10 e1 in body axes:
        # (-10 cos 20, 0, -10 sin 20) = (-9.396926, 0, -3.420201); a sideways force 2 N along +y_sim is 2 e2, e2 = e3 x e1 with
        # e3 = (-sin a, 0, cos a)... computed here from the definition of the rows, then compared with the tool
        a = math.radians(20.0)
        e1 = (-math.cos(a), 0.0, -math.sin(a))
        case = {"case_id": "t", "speed_mps": 10.0, "alpha_deg": 20.0, "beta_deg": 0.0, "ref_area_m2": 0.025, "ref_length_m": 0.44,
                "rotation_rows": [list(r) for r in S.Q.body_to_sim_rotation(20.0, 0.0)]}
        r = S.collect_row(case, "10.0,0,0,0,0,0,0.1,1,1", 1.225)
        for got, want in zip(r["f"], (10.0 * e1[0], 0.0, 10.0 * e1[2])):
            self.assertAlmostEqual(got, want, places=12)
        self.assertAlmostEqual(r["f"][0], -9.396926207859083, places=9)
        self.assertAlmostEqual(r["f"][2], -3.4202014332566873, places=9)
        # the x_sim axis is the drag direction: the force magnitude is unchanged by the rotation
        self.assertAlmostEqual(math.sqrt(sum(x * x for x in r["f"])), 10.0, places=12)

    def test_collect_row_is_the_inverse_rotation_for_a_general_angle(self):
        # the rows R map body to simulation axes: R f_body must give back f_sim, for any alpha / beta and any force (independent dot products)
        for alpha, beta in ((20.0, 10.0), (-35.0, 5.0), (0.0, -25.0)):
            rows = S.Q.body_to_sim_rotation(alpha, beta)
            case = {"case_id": "t", "speed_mps": 10.0, "alpha_deg": alpha, "beta_deg": beta, "ref_area_m2": 0.025, "ref_length_m": 0.44,
                    "rotation_rows": [list(r) for r in rows]}
            f_sim = (10.0, 2.0, -3.0)
            r = S.collect_row(case, "10.0,2.0,-3.0,0.1,-0.2,0.3,0.1,1,1", 1.225)
            for k in range(3):
                self.assertAlmostEqual(sum(rows[k][i] * r["f"][i] for i in range(3)), f_sim[k], places=12, msg=f"{alpha} {beta}")
            t_sim = (0.1, -0.2, 0.3)
            for k in range(3):
                self.assertAlmostEqual(sum(rows[k][i] * r["m"][i] for i in range(3)), t_sim[k], places=12)

    def test_collect_row_refusals(self):
        case = {"case_id": "t", "speed_mps": 10.0, "alpha_deg": 0.0, "beta_deg": 0.0, "ref_area_m2": 0.025, "ref_length_m": 0.44,
                "rotation_rows": [list(r) for r in S.Q.body_to_sim_rotation(0.0, 0.0)]}
        for line in ("-1.0,0,0,0,0,0,0.1,1,1", "0.0,0,0,0,0,0,0.1,1,1", "nan,0,0,0,0,0,0.1,1,1", "1,2,3", "inf,0,0,0,0,0,0,1,1"):
            with self.assertRaises((S.SweepError, ValueError), msg=line):
                S.collect_row(case, line, 1.225)

    def test_collect_writes_the_csv_with_provenance_and_skips_missing_cases(self):
        sweep = Path(self.tmp.name) / "collect"
        S.generate(REF, sweep, 55, 6.0, None, None, None, 3)
        first = json.loads((sweep / "sweep_manifest.json").read_text(encoding="utf-8"))["cases"][0]
        (sweep / first["case_id"] / "raw_result.csv").write_text(S.RAW_HEADER + "\n12.0,0.5,-2.0,0.01,0.02,0.03,0.1,26667,6666\n", encoding="utf-8")
        out = S.collect(sweep)
        rows = list(csv.DictReader(io.StringIO(out.read_text(encoding="utf-8"))))
        self.assertEqual(len(rows), 1)                                    # the other two cases have no result: skipped
        self.assertEqual(rows[0]["provenance"], "preliminary CFD, unvalidated")
        self.assertEqual(rows[0]["propellers"], "omit")
        self.assertEqual(len(rows[0]["geometry_sha256"]), 64)
        self.assertEqual(rows[0]["solver"], "FluidX3D 3.8")
        (sweep / first["case_id"] / "raw_result.csv").write_text("wrong\n1\n", encoding="utf-8")
        with self.assertRaises(S.SweepError):
            S.collect(sweep)


class SyntheticTests(unittest.TestCase):
    def test_synthetic_sample_is_labelled_and_complete(self):
        text = S.synthetic_csv(REF)
        rows = list(csv.DictReader(io.StringIO(text)))
        self.assertEqual(len(rows), 4 * (5 + 4))                          # 4 speeds x (5 alphas at beta 0 + 4 betas at alpha 0)
        self.assertTrue(all(r["provenance"] == "SYNTHETIC formula, not CFD" and r["solver"] == "SYNTHETIC formula" for r in rows))
        self.assertEqual(text, S.synthetic_csv(REF))                       # deterministic
        row = next(r for r in rows if r["speed_mps"] == "10.0" and r["alpha_deg"] == "0.0" and r["beta_deg"] == "0.0")
        # F_x = A_ref x V^2 x c0 = A_ref x 100 x (-0.05)
        self.assertAlmostEqual(float(row["fx_body_n"]), -5.0 * float(row["ref_area_m2"]), places=6)

    def test_committed_sample_csvs_are_current(self):
        sample = HERE_SAMPLE
        self.assertEqual((sample / "synthetic_quad_standard_cfd.csv").read_text(encoding="utf-8"), S.synthetic_csv(REF))
        self.assertEqual((sample / "synthetic_quad_fpv_5in_cfd.csv").read_text(encoding="utf-8"), S.synthetic_csv(FPV))
        self.assertTrue(all("SYNTHETIC" in line for line in (sample / "synthetic_quad_standard_cfd.csv").read_text(encoding="utf-8").splitlines()[1:]))

    def test_synthetic_classes_differ(self):
        self.assertNotEqual(S.synthetic_csv(REF), S.synthetic_csv(FPV))
        self.assertEqual(len(list(csv.DictReader(io.StringIO(S.synthetic_csv(FPV))))), 36)


class CliTests(unittest.TestCase):
    def test_cli_generate_check_synthetic_and_errors(self):
        with tempfile.TemporaryDirectory() as t:
            out = Path(t) / "s"
            self.assertEqual(S.main(["generate", "--build", str(REF), "--out", str(out), "--bytes-per-cell", "55", "--max-cases", "1"]), 0)
            case = next(out.glob("v*/case.json"))
            self.assertEqual(S.main(["check", "--case", str(case)]), 0)
            self.assertEqual(S.main(["synthetic", "--build", str(FPV), "--out", str(Path(t) / "syn.csv")]), 0)
            self.assertTrue((Path(t) / "syn.csv").read_text(encoding="utf-8").startswith("case_id,"))
            self.assertEqual(S.main(["generate", "--build", str(REF), "--out", str(Path(t) / "x"), "--bytes-per-cell", "5000"]), 1)
            self.assertFalse((Path(t) / "x").exists())                     # nothing is written when a case is refused
            self.assertEqual(S.main(["check", "--case", str(case), "--free-vram-mib", "10"]), 1)


if __name__ == "__main__":
    unittest.main()
