# Tier: D
"""Hand-calculated tests for mechanics.py. Every expected value has its derivation in a comment."""
import math
import unittest

import mechanics as M


class ShapeInertiaTests(unittest.TestCase):
    def test_box(self):
        # 1 kg, 0.2 x 0.2 x 0.1: Ixx = (0.04 + 0.01)/12 = 0.0041666..., Iyy same, Izz = 0.08/12 = 0.0066666...
        t = M.box_inertia(1.0, (0.2, 0.2, 0.1))
        self.assertAlmostEqual(t[0][0], 0.05 / 12, places=14)
        self.assertAlmostEqual(t[1][1], 0.05 / 12, places=14)
        self.assertAlmostEqual(t[2][2], 0.08 / 12, places=14)
        self.assertEqual(t[0][1], 0.0)

    def test_box_unequal_edges_use_the_other_two(self):
        # 2 kg, edges (1, 2, 3): Ixx = 2 (4 + 9)/12 = 13/6; Iyy = 2 (1 + 9)/12 = 10/6; Izz = 2 (1 + 4)/12 = 5/6
        t = M.box_inertia(2.0, (1.0, 2.0, 3.0))
        self.assertAlmostEqual(t[0][0], 13 / 6, places=14)
        self.assertAlmostEqual(t[1][1], 10 / 6, places=14)
        self.assertAlmostEqual(t[2][2], 5 / 6, places=14)

    def test_cylinder(self):
        # 1 kg, D = 0.2 (r = 0.1), h = 0.1: Izz = 0.5 * 0.01 = 0.005; Ixx = (3 * 0.01 + 0.01)/12 = 0.04/12
        t = M.cylinder_inertia(1.0, 0.2, 0.1)
        self.assertAlmostEqual(t[2][2], 0.005, places=14)
        self.assertAlmostEqual(t[0][0], 0.04 / 12, places=14)
        self.assertEqual(t[0][0], t[1][1])

    def test_disc(self):
        # 0.02 kg, D = 0.3048: r^2 = 0.02322576; Izz = 0.02 * 0.02322576 / 2 = 2.322576e-4; Ixx = half of that
        t = M.disc_inertia(0.02, 0.3048)
        self.assertAlmostEqual(t[2][2], 2.322576e-4, places=12)
        self.assertAlmostEqual(t[0][0], 1.161288e-4, places=12)
        # perpendicular axis theorem for a flat body: Izz = Ixx + Iyy
        self.assertAlmostEqual(t[2][2], t[0][0] + t[1][1], places=14)

    def test_rod_along_x_and_diagonal(self):
        # 0.1 kg, L = 0.3 along x: transverse I = 0.1 * 0.09 / 12 = 7.5e-4, axial 0
        t = M.rod_inertia(0.1, 0.3, (1, 0, 0))
        self.assertAlmostEqual(t[0][0], 0.0, places=15)
        self.assertAlmostEqual(t[1][1], 7.5e-4, places=14)
        self.assertAlmostEqual(t[2][2], 7.5e-4, places=14)
        # along the (1,1,0)/sqrt2 diagonal: k = 7.5e-4; Ixx = k (1 - 1/2) = 3.75e-4; Ixy = -k * (-1/2)... = +k/2 entry (tensor entry = -k d_x d_y)
        d = M.rod_inertia(0.1, 0.3, (1, 1, 0))
        self.assertAlmostEqual(d[0][0], 3.75e-4, places=14)
        self.assertAlmostEqual(d[0][1], -3.75e-4, places=14)
        self.assertAlmostEqual(d[2][2], 7.5e-4, places=14)

    def test_bad_inputs_rejected(self):
        for call in (lambda: M.box_inertia(0.0, (1, 1, 1)), lambda: M.box_inertia(1.0, (1, -1, 1)),
                     lambda: M.box_inertia(float("nan"), (1, 1, 1)), lambda: M.cylinder_inertia(1, -1, 1),
                     lambda: M.disc_inertia(-1, 1), lambda: M.rod_inertia(1, 1, (0, 0, 0))):
            with self.assertRaises(ValueError):
                call()


class ParallelAxisTests(unittest.TestCase):
    def test_point_mass_offset(self):
        # 0.1 kg at (0.2, 0.2, 0): Ixx = m (y^2 + z^2) = 0.004, Iyy = 0.004, Izz = m (x^2 + y^2) = 0.008, Ixy entry = -m x y = -0.004
        t = M.parallel_axis(M.zeros(), 0.1, (0.2, 0.2, 0.0))
        self.assertAlmostEqual(t[0][0], 0.004, places=14)
        self.assertAlmostEqual(t[1][1], 0.004, places=14)
        self.assertAlmostEqual(t[2][2], 0.008, places=14)
        self.assertAlmostEqual(t[0][1], -0.004, places=14)
        self.assertEqual(t[0][1], t[1][0])

    def test_adds_to_the_part_tensor(self):
        base = M.box_inertia(1.0, (0.2, 0.2, 0.1))
        t = M.parallel_axis(base, 1.0, (0.0, 0.0, 0.1))  # shift along z: Ixx += 1 * 0.01
        self.assertAlmostEqual(t[0][0], 0.05 / 12 + 0.01, places=14)
        self.assertAlmostEqual(t[2][2], base[2][2], places=14)


class CombineTests(unittest.TestCase):
    def test_two_masses_cg_and_inertia(self):
        # 3 kg at x = 0 and 1 kg at x = 0.4 -> CG x = 0.4 / 4 = 0.1. Point masses: Iyy = 3 * 0.1^2 + 1 * 0.3^2 = 0.03 + 0.09 = 0.12
        t = lambda: M.zeros()  # noqa: E731
        total, cg, inertia = M.combine([M.Body("a", 3.0, (0, 0, 0), t()), M.Body("b", 1.0, (0.4, 0, 0), t())])
        self.assertAlmostEqual(total, 4.0, places=14)
        self.assertAlmostEqual(cg[0], 0.1, places=14)
        self.assertAlmostEqual(inertia[1][1], 0.12, places=14)
        self.assertAlmostEqual(inertia[2][2], 0.12, places=14)
        self.assertAlmostEqual(inertia[0][0], 0.0, places=14)

    def test_symmetric_set_has_cg_at_origin_and_no_products(self):
        pts = [(0.2, 0.2, 0.0), (0.2, -0.2, 0.0), (-0.2, 0.2, 0.0), (-0.2, -0.2, 0.0)]
        total, cg, inertia = M.combine([M.Body(str(i), 0.149, p, M.zeros()) for i, p in enumerate(pts)])
        # reference hand values (old catalog): Ixx = 4 * 0.149 * 0.04 = 0.02384, Izz = 0.04768
        self.assertAlmostEqual(total, 0.596, places=14)
        for k in range(3):
            self.assertAlmostEqual(cg[k], 0.0, places=15)
        self.assertAlmostEqual(inertia[0][0], 0.02384, places=14)
        self.assertAlmostEqual(inertia[2][2], 0.04768, places=14)
        for i, j in ((0, 1), (0, 2), (1, 2)):
            self.assertAlmostEqual(inertia[i][j], 0.0, places=15)

    def test_payload_offset_moves_cg_and_adds_parallel_axis_term(self):
        # body 1.5 kg at origin + payload 0.5 kg at x = 0.1: CG x = 0.5 * 0.1 / 2.0 = 0.025
        # Iyy about the new CG (point masses): 1.5 * 0.025^2 + 0.5 * 0.075^2 = 0.0009375 + 0.0028125 = 0.00375
        total, cg, inertia = M.combine([M.Body("b", 1.5, (0, 0, 0), M.zeros()), M.Body("p", 0.5, (0.1, 0, 0), M.zeros())])
        self.assertAlmostEqual(cg[0], 0.025, places=14)
        self.assertAlmostEqual(inertia[1][1], 0.00375, places=14)

    def test_combined_inertia_is_symmetric(self):
        _, _, t = M.combine([M.Body("a", 1.0, (0.1, 0.2, 0.3), M.box_inertia(1, (0.1, 0.1, 0.1))),
                             M.Body("b", 2.0, (-0.3, 0.1, -0.2), M.disc_inertia(2, 0.2))])
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(t[i][j], t[j][i], places=15)

    def test_empty_rejected(self):
        with self.assertRaises(ValueError):
            M.combine([])

    def test_nonpositive_mass_rejected(self):
        with self.assertRaises(ValueError):
            M.Body("x", 0.0, (0, 0, 0), M.zeros())


class SectionTests(unittest.TestCase):
    def test_tube(self):
        # od 0.016, id 0.013: od^4 = 6.5536e-8, id^4 = 2.8561e-8, diff = 3.6975e-8; I = pi/64 * diff = 0.0490874 * 3.6975e-8 = 1.815006e-9
        # A = pi/4 (2.56e-4 - 1.69e-4) = pi/4 * 8.7e-5 = 6.83296e-5; Z = I / 0.008 = 2.268758e-7
        s = M.tube_section(0.016, 0.013)
        self.assertAlmostEqual(s["second_moment_m4"], math.pi / 64 * 3.6975e-8, places=18)
        self.assertAlmostEqual(s["second_moment_m4"], 1.815006e-9, places=14)
        self.assertAlmostEqual(s["area_m2"], math.pi / 4 * 8.7e-5, places=15)
        self.assertAlmostEqual(s["section_modulus_m3"], 2.268758e-7, places=12)

    def test_solid_tube_limit(self):
        s = M.tube_section(0.01, 0.0)  # solid rod: I = pi d^4 / 64
        self.assertAlmostEqual(s["second_moment_m4"], math.pi * 1e-8 / 64, places=20)

    def test_plate(self):
        # w 0.02, t 0.005: I = 0.02 * 1.25e-7 / 12 = 2.083333e-10; Z = I / 0.0025 = 8.33333e-8
        s = M.plate_section(0.02, 0.005)
        self.assertAlmostEqual(s["second_moment_m4"], 2.0833333e-10, places=17)
        self.assertAlmostEqual(s["section_modulus_m3"], 8.333333e-8, places=14)

    def test_bad_sections(self):
        for call in (lambda: M.tube_section(0.01, 0.01), lambda: M.tube_section(0.01, -0.001),
                     lambda: M.plate_section(0.0, 0.005), lambda: M.plate_section(0.02, float("inf"))):
            with self.assertRaises(ValueError):
                call()

    def test_cantilever_stress(self):
        # F = 10 N at L = 0.2 m on Z = 2.268758e-7: M = 2.0 N m, sigma = 2.0 / 2.268758e-7 = 8.8154e6 Pa
        self.assertAlmostEqual(M.cantilever_bending_stress_pa(10.0, 0.2, 2.268758e-7), 8.8154e6, delta=50.0)
        with self.assertRaises(ValueError):
            M.cantilever_bending_stress_pa(10.0, 0.2, 0.0)

    def test_projected_areas(self):
        # boxes (0.2, 0.1, 0.05) and (0.1, 0.1, 0.1): top = 0.02 + 0.01, front (y*z) = 0.005 + 0.01, side (x*z) = 0.01 + 0.01
        a = M.projected_areas_m2([(0.2, 0.1, 0.05), (0.1, 0.1, 0.1)])
        self.assertAlmostEqual(a["top"], 0.03, places=15)
        self.assertAlmostEqual(a["front"], 0.015, places=15)
        self.assertAlmostEqual(a["side"], 0.02, places=15)


if __name__ == "__main__":
    unittest.main()
