# Tier: D
"""Tests for vehicle_stl: hand-calculated shell volumes, watertightness, omitted parts, refusals, determinism, CLI."""
import copy
import hashlib
import json
import math
import tempfile
import unittest
from pathlib import Path

import vehicle_stl as V

ROOT = Path(__file__).resolve().parents[2]
BUILDS = ROOT / "tools" / "parts" / "builds"
REF = json.loads((BUILDS / "reference_quad" / "assembly.json").read_text(encoding="utf-8"))
FPV = json.loads((BUILDS / "fpv_5in" / "assembly.json").read_text(encoding="utf-8"))


def shell(geo, name):
    return next(s for s in geo["shells"] if s["instance"] == name)


class VolumeTests(unittest.TestCase):
    """Every shell is shrunk by GAP/2 on each side (0.001 m); expected values are written out by hand."""

    @classmethod
    def setUpClass(cls):
        cls.ref = V.build_geometry(REF)
        cls.fpv = V.build_geometry(FPV)

    def test_box_volumes_are_the_shrunk_dimension_products(self):
        # body 0.2 x 0.2 x 0.1 -> 0.198 x 0.198 x 0.098 = 3.841992e-3 m^3; pack 0.138 x 0.043 x 0.034 -> 0.136 x 0.041 x 0.032 = 1.78432e-4
        self.assertAlmostEqual(shell(self.ref, "body")["volume_m3"], 0.198 * 0.198 * 0.098, places=12)
        self.assertAlmostEqual(shell(self.ref, "body")["volume_m3"], 3.841992e-3, places=12)
        self.assertAlmostEqual(shell(self.ref, "pack")["volume_m3"], 1.78432e-4, places=12)
        self.assertAlmostEqual(shell(self.fpv, "body")["volume_m3"], 0.058 * 0.118 * 0.028, places=12)
        self.assertAlmostEqual(shell(self.fpv, "body")["volume_m3"], 1.91632e-4, places=11)

    def test_octagonal_tube_arm_volume(self):
        # radius 0.008 - 0.001 = 0.007; regular octagon area = 2 sqrt(2) R^2 = 1.385929e-4 m^2 (= 2 (1 + sqrt 2) a^2 with side a = 2 R sin(pi/8))
        # arm along the diagonal: start 0.1 sqrt2 + 0.008 sqrt2 + 0.002 = 0.1547351, end 0.2 sqrt2 - 0.0209 - 0.002 = 0.2599427, length 0.1052076
        s = shell(self.ref, "arm_0")
        self.assertEqual(s["kind"], "tube")
        self.assertAlmostEqual(s["volume_m3"], 1.4581036065e-5, places=14)
        a = 2 * 0.007 * math.sin(math.pi / 8)
        self.assertAlmostEqual(2 * (1 + math.sqrt(2)) * a * a * 0.10520764773832478, s["volume_m3"], places=14)

    def test_motor_can_is_a_16_gon_cylinder(self):
        # R = (0.0418 - 0.002) / 2 = 0.0199, h = 0.0265 - 0.002 = 0.0245: (16 / 2) R^2 sin(2 pi / 16) h = 2.97031e-5
        self.assertAlmostEqual(shell(self.ref, "motor_0")["volume_m3"], 2.9703107346e-5, places=14)
        # FPV can: R = (0.029 - 0.002) / 2, h = 0.033 - 0.002 -> 1.72965e-5
        self.assertAlmostEqual(shell(self.fpv, "motor_0")["volume_m3"], 1.7296525776e-5, places=14)

    def test_plate_arm_is_a_yawed_box(self):
        # FPV arm: start 0.03 c + 0.06 c + 0.0125 (2 c) + 0.002 = 0.0833173 (c = 1 / sqrt 2), end 0.11 - 0.0145 - 0.002 = 0.0935,
        # length 0.0101824; shrunk box (L - 0.002) x (0.025 - 0.002) x (0.005 - 0.002)
        s = shell(self.fpv, "arm_0")
        self.assertEqual(s["kind"], "plate")
        self.assertAlmostEqual(s["volume_m3"], 5.645836932e-7, places=15)

    def test_every_shell_is_watertight_with_the_expected_volume(self):
        for geo in (self.ref, self.fpv):
            for s in geo["shells"]:
                self.assertAlmostEqual(s["volume_m3"], s["expected_volume_m3"], places=12, msg=s["instance"])
                self.assertGreater(s["volume_m3"], 0.0)

    def test_total_volume_is_the_sum_of_the_shells(self):
        for geo in (self.ref, self.fpv):
            self.assertAlmostEqual(V.G.signed_volume(geo["mesh"]), sum(s["volume_m3"] for s in geo["shells"]), places=12)

    def test_reference_quad_shell_list_and_omissions(self):
        names = [s["instance"] for s in self.ref["shells"]]
        self.assertEqual(names, ["body", "arm_0", "arm_1", "arm_2", "arm_3", "motor_0", "motor_1", "motor_2", "motor_3", "pack"])
        omitted = {o["instance"]: o["reason"] for o in self.ref["omitted"]}
        self.assertEqual(sorted(omitted), ["esc_0", "esc_1", "esc_2", "esc_3", "lump", "prop_0", "prop_1", "prop_2", "prop_3"])
        self.assertIn("rotor wash", omitted["prop_0"])
        self.assertIn("inside the motor can", omitted["esc_0"])
        self.assertIn("none", omitted["lump"])

    def test_fpv_shell_list(self):
        names = [s["instance"] for s in self.fpv["shells"]]
        self.assertEqual(names, ["body", "arm_0", "arm_1", "arm_2", "arm_3", "motor_0", "motor_1", "motor_2", "motor_3", "pack", "4in1", "unit"])
        self.assertEqual(sorted(o["instance"] for o in self.fpv["omitted"]), ["prop_0", "prop_1", "prop_2", "prop_3"])

    def test_bounding_box_matches_the_assembly(self):
        # reference: motor 0.2 + R 0.0199 = 0.2199 in x and y; top of the body -0.05 + 0.001; bottom of the battery 0.067 + 0.017 - 0.001
        lo, hi = V.G.bounding_box(self.ref["mesh"])
        self.assertAlmostEqual(hi[0], 0.2199, places=9)
        self.assertAlmostEqual(lo[0], -0.2199, places=9)
        self.assertAlmostEqual(lo[2], -0.049, places=9)
        self.assertAlmostEqual(hi[2], 0.083, places=9)
        # FPV: motor 0.0777815 + 0.0135 = 0.0912815
        lo, hi = V.G.bounding_box(self.fpv["mesh"])
        self.assertAlmostEqual(hi[0], 0.0912815, places=9)

    def test_disc_option_adds_four_thin_discs_where_they_fit(self):
        geo = V.build_geometry(FPV, props="disc")
        discs = [s for s in geo["shells"] if s["kind"] == "disc"]
        self.assertEqual(len(discs), 4)
        # FPV 5 in props: R = (0.12954 - 0.002) / 2, height 0.004 - 0.002 = 0.002: 32-gon area times height
        r = (0.12954 - 0.002) / 2
        self.assertTrue(all(abs(d["volume_m3"] - 0.5 * 32 * r * r * math.sin(2 * math.pi / 32) * 0.002) < 1e-12 for d in discs))
        self.assertEqual(len(geo["omitted"]), 0)

    def test_disc_option_is_refused_where_the_props_overlap_the_frame(self):
        # reference quad: a 12 in prop disc at a 0.28 m arm end reaches inside the 0.2 m body footprint at the body's height
        with self.assertRaises(V.GeometryError):
            V.build_geometry(REF, props="disc")


class RefusalTests(unittest.TestCase):
    def test_overlapping_boxes_are_refused(self):
        a = copy.deepcopy(REF)
        battery = next(p for p in a["parts"] if p["instance"] == "pack")
        battery["position_m"] = [0.0, 0.0, 0.0]            # inside the body box
        with self.assertRaises(V.GeometryError):
            V.build_geometry(a)

    def test_arm_without_room_is_refused(self):
        a = copy.deepcopy(FPV)
        for p in a["parts"]:
            if p["role"] == "motor":
                p["position_m"] = [p["position_m"][0] * 0.8, p["position_m"][1] * 0.8, p["position_m"][2]]
        with self.assertRaises(V.GeometryError):
            V.build_geometry(a)

    def test_arm_off_the_motor_ray_is_refused(self):
        a = copy.deepcopy(REF)
        next(p for p in a["parts"] if p["instance"] == "arm_0")["position_m"] = [0.1, 0.03, 0.0]
        with self.assertRaises(V.GeometryError):
            V.build_geometry(a)

    def test_unknown_visual_kind_missing_body_and_bad_props(self):
        a = copy.deepcopy(REF)
        next(p for p in a["parts"] if p["instance"] == "pack")["visual"] = {"kind": "blob"}
        with self.assertRaises(V.GeometryError):
            V.build_geometry(a)
        b = copy.deepcopy(REF)
        b["parts"] = [p for p in b["parts"] if p["instance"] != "body"]
        with self.assertRaises(V.GeometryError):
            V.build_geometry(b)
        with self.assertRaises(V.GeometryError):
            V.build_geometry(REF, props="solid")


class AreaAndFileTests(unittest.TestCase):
    def test_projected_area_of_a_box(self):
        m = V.oriented_box((0.0, 0.0, 0.0), (0.1, 0.2, 0.3), 0.0)
        self.assertAlmostEqual(V.projected_area_m2(m, 0), 0.2 * 0.3, delta=0.0006)
        self.assertAlmostEqual(V.projected_area_m2(m, 1), 0.1 * 0.3, delta=0.0006)
        self.assertAlmostEqual(V.projected_area_m2(m, 2), 0.1 * 0.2, delta=0.0006)

    def test_projected_area_of_a_yawed_box_is_its_footprint(self):
        m = V.oriented_box((0.0, 0.0, 0.0), (0.2, 0.1, 0.05), math.radians(30))
        self.assertAlmostEqual(V.projected_area_m2(m, 2), 0.2 * 0.1, delta=0.0006)

    def test_oriented_box_volume_is_independent_of_yaw(self):
        for yaw in (0.0, 0.5, math.pi / 2):
            m = V.oriented_box((1.0, 2.0, 3.0), (0.1, 0.2, 0.3), yaw)
            V.G.check_watertight(m)
            self.assertAlmostEqual(V.G.signed_volume(m), 0.006, places=12)

    def test_geometry_document_and_cli_are_deterministic(self):
        with tempfile.TemporaryDirectory() as t:
            out1, out2 = Path(t) / "a", Path(t) / "b"
            for out in (out1, out2):
                self.assertEqual(V.main(["--build", str(BUILDS / "reference_quad"), "--out", str(out)]), 0)
            self.assertEqual((out1 / "vehicle.stl").read_bytes(), (out2 / "vehicle.stl").read_bytes())
            doc = json.loads((out1 / "geometry.json").read_text(encoding="utf-8"))
            self.assertEqual(doc["stl_sha256"], hashlib.sha256((out1 / "vehicle.stl").read_bytes()).hexdigest())
            self.assertEqual(doc["triangles"], (out1 / "vehicle.stl").read_text().count("facet normal"))
            self.assertEqual(doc["propellers"], "omit")
            self.assertEqual(doc["vehicle_class"], "quad_standard")
            self.assertIn("PRELIMINARY", doc["provenance"])
            # projected area along z of the reference quad: bigger than the body alone (0.198^2 = 0.0392) and below the bbox 0.44 x 0.44
            self.assertTrue(0.0392 < doc["projected_area_m2"]["along_z"] < 0.19)

    def test_cli_reports_a_geometry_error(self):
        with tempfile.TemporaryDirectory() as t:
            bad = Path(t) / "build"
            bad.mkdir()
            a = copy.deepcopy(REF)
            next(p for p in a["parts"] if p["instance"] == "pack")["position_m"] = [0.0, 0.0, 0.0]
            (bad / "assembly.json").write_text(json.dumps(a), encoding="utf-8")
            self.assertEqual(V.main(["--build", str(bad), "--out", str(Path(t) / "o")]), 1)
            self.assertFalse((Path(t) / "o" / "vehicle.stl").exists())


if __name__ == "__main__":
    unittest.main()
