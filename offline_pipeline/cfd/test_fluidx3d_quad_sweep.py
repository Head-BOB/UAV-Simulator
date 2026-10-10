# Tier: D
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fluidx3d_quad_sweep as s  # noqa: E402

geo = s.geo
BASE = geo.build_quad(json.loads(geo.CATALOG.read_text()), json.loads(geo.CONFIG.read_text()))
MEM = {"bytes_per_cell": 100, "vram_bytes": 6 * 1024 ** 3, "headroom_fraction": 0.15}


def det(r):
    return (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))


class RotationTests(unittest.TestCase):
    def test_zero_angles_hand_value(self):
        # V_b = (1,0,0) -> e1 = (-1,0,0); e3 = (0,0,1); e2 = e3 x e1 = (0,-1,0): R = diag(-1,-1,1).
        r = s.body_to_sim_rotation(0.0, 0.0)
        for got, want in zip(r, ((-1, 0, 0), (0, -1, 0), (0, 0, 1))):
            for g, w in zip(got, want):
                self.assertAlmostEqual(g, w + 0.0, places=12)

    def test_proper_rotation_and_air_maps_to_plus_x(self):
        for a, b in ((20, 0), (-20, 0), (0, 10), (0, -20), (10, 10), (5, -15)):
            r = s.body_to_sim_rotation(a, b)
            self.assertAlmostEqual(det(r), 1.0, places=12)
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(sum(r[i][k] * r[j][k] for k in range(3)), 1.0 if i == j else 0.0, places=12)
            ar, br = math.radians(a), math.radians(b)
            air = (-math.cos(ar) * math.cos(br), -math.sin(br), -math.sin(ar) * math.cos(br))
            x = s.mat_vec(r, air)
            for g, w in zip(x, (1.0, 0.0, 0.0)):
                self.assertAlmostEqual(g, w, places=12)

    def test_bad_angles_rejected(self):
        for a, b in ((90.0, 0.0), (0.0, -90.0), (math.nan, 0.0), (0.0, math.inf)):
            with self.assertRaises(s.SweepError):
                s.body_to_sim_rotation(a, b)

    def test_rotated_mesh_stays_watertight_volume_scales(self):
        dx = 0.004
        m = s.rotated_scaled_mesh(BASE, s.body_to_sim_rotation(20.0, -10.0), dx)
        geo.check_watertight(m)
        self.assertAlmostEqual(geo.signed_volume(m), geo.signed_volume(BASE) / dx ** 3, delta=1e-6 * geo.signed_volume(BASE) / dx ** 3)

    def test_bbox_stays_centred_on_body_origin(self):
        # Shape is point-symmetric about the body origin, so any rotation keeps the bbox centred there:
        # setup.cpp relies on this (bbox centre = body origin = torque reference point).
        lo, hi = geo.bounding_box(s.rotated_scaled_mesh(BASE, s.body_to_sim_rotation(15.0, 10.0), 0.004))
        for i in range(3):
            self.assertAlmostEqual(lo[i], -hi[i], places=9)


class PlanTests(unittest.TestCase):
    def test_grid_hand_value(self):
        # 4*0.45/0.004 = 450 (even); 2*0.45/0.004 = 225 -> 226.
        self.assertEqual(s.grid_for(0.004), (450, 226, 226))
        self.assertLessEqual(450 * 226 * 226, 300 ** 3)

    def test_total_steps_hand_value(self):
        # 5 flow-throughs * 450 cells / 0.075 = 30000
        self.assertEqual(s.total_steps(450), 30000)

    def test_case_count_and_pruning(self):
        c = s.sweep_cases(s.DEFAULT_SPEEDS_MPS, s.DEFAULT_ANGLES_DEG)
        self.assertEqual(len(c), 4 * (5 + 4))
        self.assertTrue(all(a == 0.0 or b == 0.0 for _, a, b in c))
        self.assertEqual(len(set(s.case_id(*x) for x in c)), len(c))
        self.assertEqual(len(s.sweep_cases((5.0,), (0.0,), 1)), 1)

    def test_zero_speed_rejected(self):
        with self.assertRaises(s.SweepError):
            s.sweep_cases((0.0,), (0.0,))

    def test_case_id_format(self):
        self.assertEqual(s.case_id(5.0, -10.0, 0.0), "v05_am10_bp00")

    def test_too_many_cells_refused(self):
        with self.assertRaises(s.SweepError):
            s.make_case(5.0, 0.0, 0.0, 0.002, MEM, BASE)  # 900 x 450 x 450 cells

    def test_vram_refusal(self):
        big = dict(MEM, bytes_per_cell=1000)  # 22.98e6 cells * 1000 B = 21.4 GiB
        with self.assertRaises(s.SweepError):
            s.make_case(5.0, 0.0, 0.0, 0.004, big, BASE)

    def test_vram_boundary_exact(self):
        # required = 450*226*226 * b ; limit = 6 GiB * 0.85 = 5476083302.4: b = 238 fits, 239 does not.
        n = 450 * 226 * 226
        self.assertLessEqual(n * 238, 6 * 1024 ** 3 * 0.85)
        s.make_case(5.0, 0.0, 0.0, 0.004, dict(MEM, bytes_per_cell=238), BASE)
        with self.assertRaises(s.SweepError):
            s.make_case(5.0, 0.0, 0.0, 0.004, dict(MEM, bytes_per_cell=239), BASE)


class RenderAndCollectTests(unittest.TestCase):
    def test_setup_has_values_and_no_placeholder(self):
        case = s.make_case(10.0, 5.0 + 5.0, 0.0, 0.004, MEM, BASE)
        text = s.render_setup(case)
        self.assertNotIn("@@", text)
        self.assertIn("Nx=450u, Ny=226u, Nz=226u", text)
        self.assertIn("si_u = 10.0f", text)
        self.assertIn("30000ull", text)
        self.assertIn("TODO(HUMAN-VERIFY", text)

    def test_generate_collect_end_to_end(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "sweep"
            m = s.generate(out, 100, 6.0, 0.004, (10.0,), (0.0, 10.0), None)
            self.assertEqual(len(m), 3)  # a 0, a 10, b 10
            self.assertTrue((out / "run_sweep.ps1").exists() and (out / m[0]["case_id"] / "quad.stl").exists())
            for c in m:  # fake raw results: drag 4 N along the flow, lift 2 N, pitch moment 0.5 N m (sim frame)
                (out / c["case_id"] / "raw_result.csv").write_text(s.RAW_HEADER + "\n4,0.5,2,0.1,0.5,0.2,0.01,30000,7500\n")
            import csv as csvmod
            with s.collect(out).open(newline="") as fh:
                csv = list(csvmod.reader(fh))  # the provenance text has a comma, so it must round-trip quoted
            self.assertEqual(",".join(csv[0]), s.OUT_HEADER)
            self.assertEqual(len(csv), 4)
            self.assertTrue(all(len(r) == len(csv[0]) for r in csv))
            first = csv[1]
            self.assertEqual(first[0], "v10_ap00_bp00")
            # alpha = beta = 0: R = diag(-1,-1,1), R^T f = (-4, -0.5, 2); torque (-0.1, -0.5, 0.2).
            self.assertAlmostEqual(float(first[4]), -4.0, places=9)
            self.assertAlmostEqual(float(first[5]), -0.5, places=9)
            self.assertAlmostEqual(float(first[6]), 2.0, places=9)
            self.assertAlmostEqual(float(first[8]), -0.5, places=9)
            # q = 0.5 * 1.225 * 100 = 61.25; cx = -4 / (61.25 * 0.04418)
            self.assertAlmostEqual(float(first[10]), -4.0 / (61.25 * s.A_REF_M2), places=7)
            self.assertEqual(first[-1], s.PROVENANCE)

    def test_generate_writes_nothing_if_any_case_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "sweep"
            with self.assertRaises(s.SweepError):
                s.generate(out, 100000, 6.0, 0.004, (10.0,), (0.0,), None)
            self.assertFalse(out.exists())

    def test_negative_drag_rejected(self):
        case = s.make_case(10.0, 0.0, 0.0, 0.004, MEM, BASE)
        with self.assertRaises(s.SweepError):
            s.collect_row(case, "-1,0,0,0,0,0,0,30000,7500")
        for bad in ("1,2,3", "1,0,0,0,0,nan,0,30000,7500"):
            with self.assertRaises(s.SweepError):
                s.collect_row(case, bad)

    def test_check_case_free_vram(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = s.make_case(10.0, 0.0, 0.0, 0.004, MEM, BASE)
            case.pop("_stl_text")
            p = Path(tmp) / "case.json"
            p.write_text(json.dumps(case))
            self.assertIn("fits", s.check_case(p, None))
            # required = 22984200 * 100 B = 2191 MiB (2.2984e9 B); with 15 % headroom needs >= 2578 MiB free.
            self.assertIn("fits", s.check_case(p, 2600))
            with self.assertRaises(s.SweepError):
                s.check_case(p, 2500)



if __name__ == "__main__":
    unittest.main()
