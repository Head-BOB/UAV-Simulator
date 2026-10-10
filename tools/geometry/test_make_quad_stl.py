# Tier: D
import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_quad_stl as g  # noqa: E402

CAT = json.loads(g.CATALOG.read_text())
CFG = json.loads(g.CONFIG.read_text())


class ShapeTests(unittest.TestCase):
    def test_box_volume_and_watertight(self):
        m = g.Mesh()
        m.add_box((0.0, 0.0, 0.0), (0.12, 0.12, 0.10))
        g.check_watertight(m)
        # 0.12 * 0.12 * 0.10 = 0.00144 m^3 (hand)
        self.assertAlmostEqual(g.signed_volume(m), 0.00144, places=12)
        self.assertEqual(len(m.t), 12)

    def test_prism_volume_hand_calc(self):
        # Regular n-gon area = n/2 r^2 sin(2 pi / n); n = 8, r = 0.008: 4 * 6.4e-5 * 0.70710678 = 1.8101934e-4 m^2
        # times length 0.1 = 1.8101934e-5 m^3. Direction along +Z and along a diagonal give the same volume.
        for d in ((0.0, 0.0, 1.0), (1.0, 1.0, 0.0), (0.3, -0.5, 0.2)):
            m = g.Mesh()
            m.add_prism((0.1, 0.2, 0.3), d, 0.1, 0.008, 8)
            g.check_watertight(m)
            self.assertAlmostEqual(g.signed_volume(m), 4 * 6.4e-5 * math.sqrt(0.5) * 0.1, places=15)

    def test_open_mesh_detected(self):
        m = g.Mesh()
        m.add_box((0, 0, 0), (1, 1, 1))
        m.t.pop()
        with self.assertRaises(ValueError):
            g.check_watertight(m)

    def test_inward_normals_detected(self):
        m = g.Mesh()
        m.add_box((0, 0, 0), (1, 1, 1))
        m.t = [(a, c, b) for a, b, c in m.t]  # flip every triangle
        with self.assertRaises(ValueError):
            g.check_watertight(m)

    def test_zero_direction_rejected(self):
        with self.assertRaises(ValueError):
            g.Mesh().add_prism((0, 0, 0), (0, 0, 0), 1.0, 0.1, 8)


class QuadTests(unittest.TestCase):
    def test_quad_is_watertight_with_expected_volume(self):
        m = g.build_quad(CAT, CFG)  # raises if not watertight
        body = 0.12 * 0.12 * 0.10
        can_area = 16 / 2 * (0.0209 ** 2) * math.sin(2 * math.pi / 16)
        cans = 4 * can_area * 0.0265
        # arm: radius 0.008, 8 sides; start r = 0.06*sqrt2 + 0.008 + 0.002, end r = sqrt(0.08) - 0.0209 - 0.002
        r_start = 0.06 * math.sqrt(2) + 0.008 + 0.002
        r_end = math.sqrt(0.08) - 0.0209 - 0.002
        arms = 4 * (8 / 2 * 0.008 ** 2 * math.sin(2 * math.pi / 8)) * (r_end - r_start)
        self.assertAlmostEqual(g.signed_volume(m), body + cans + arms, places=9)

    def test_bounding_box_matches_dimensions(self):
        lo, hi = g.bounding_box(g.build_quad(CAT, CFG))
        # x and y: motor at 0.2 plus can radius 0.0209 = 0.2209 each side. z: max(body +-0.05, can +-0.01325).
        for i in (0, 1):
            self.assertAlmostEqual(hi[i], 0.2209, places=9)
            self.assertAlmostEqual(lo[i], -0.2209, places=9)
        self.assertAlmostEqual(hi[2], 0.05, places=12)
        self.assertAlmostEqual(lo[2], -0.05, places=12)

    def test_no_room_for_arm_rejected(self):
        cat = json.loads(json.dumps(CAT))
        cat["frame"]["body_half_width_m"]["value"] = 0.2
        with self.assertRaises(ValueError):
            g.build_quad(cat, CFG)

    def test_ascii_stl_counts(self):
        m = g.build_quad(CAT, CFG)
        text = g.to_ascii_stl(m)
        self.assertEqual(text.count("facet normal"), len(m.t))
        self.assertEqual(text.count("vertex"), 3 * len(m.t))
        self.assertTrue(text.startswith("solid ") and text.rstrip().endswith("endsolid quad_ref_build"))


if __name__ == "__main__":
    unittest.main()
