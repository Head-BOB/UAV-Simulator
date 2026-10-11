# Tier: D
"""Hand-calculated tests for prop_table.py (merge, anchors, evaluation, flags) on a small synthetic data set and on the real UIUC 12x6E file."""
import json
import unittest
from pathlib import Path

import partsdb as P
import prop_table as T

MINI = {
    "static": [{"rpm": 1000.0, "ct": 0.10, "cp": 0.03}, {"rpm": 5000.0, "ct": 0.12, "cp": 0.04}],
    "wind_tunnel": {
        "r_2000.txt": [{"j": 0.1, "ct": 0.09, "cp": 0.031, "eta": 0}, {"j": 0.3, "ct": 0.05, "cp": 0.025, "eta": 0}],
        "r_4000.txt": [{"j": 0.1, "ct": 0.10, "cp": 0.033, "eta": 0}, {"j": 0.5, "ct": 0.02, "cp": 0.015, "eta": 0}],
    },
}
REAL = json.loads((P.REPO_ROOT / "offline_pipeline/props/apce_12x6.json").read_text(encoding="utf-8"))


class DuplicateRowTests(unittest.TestCase):
    def test_exact_duplicate_rows_are_dropped(self):
        data = json.loads(json.dumps(MINI))
        rows = data["wind_tunnel"]["r_2000.txt"]
        rows.extend([dict(rows[-1])] * 3)
        run = T.parse_runs(data)[0]
        self.assertEqual(len(run["points"]), 2)

    def test_same_j_with_different_values_is_refused(self):
        data = json.loads(json.dumps(MINI))
        data["wind_tunnel"]["r_2000.txt"].append({"j": 0.3, "ct": 0.06, "cp": 0.025, "eta": 0})
        with self.assertRaises(T.PropTableError):
            T.parse_runs(data)

    def test_the_real_file_has_duplicates_that_are_dropped(self):
        raw = REAL["wind_tunnel"]["apce_12x6_0633od_5042.txt"]
        run = next(r for r in T.parse_runs(REAL) if r["name"] == "apce_12x6_0633od_5042.txt")
        self.assertEqual((len(raw), len(run["points"])), (24, 11))   # 14 identical rows at J = 0.617992 collapse to 1


class StaticTests(unittest.TestCase):
    def test_static_interpolation(self):
        # rpm 3000 is the midpoint of 1000 and 5000: Ct = 0.11, Cp = 0.035
        ct, cp = T.static_at(MINI["static"], 3000.0)
        self.assertAlmostEqual(ct, 0.11, places=15)
        self.assertAlmostEqual(cp, 0.035, places=15)
        self.assertEqual(T.static_at(MINI["static"], 1000.0), (0.10, 0.03))

    def test_static_outside_its_data_is_refused(self):
        for rpm in (999.0, 5001.0):
            with self.assertRaises(T.PropTableError):
                T.static_at(MINI["static"], rpm)


class BlendTests(unittest.TestCase):
    LOW = [(0.1, 0.10, 0.030), (0.2, 0.09, 0.028), (0.3, 0.08, 0.026), (0.4, 0.07, 0.024)]
    HIGH = [(0.3, 0.082, 0.0262), (0.35, 0.079, 0.0252), (0.4, 0.068, 0.0238), (0.5, 0.054, 0.020), (0.6, 0.04, 0.016)]

    def test_blend_weights_at_the_ends_and_the_middle_of_the_overlap(self):
        # overlap [0.3, 0.4]: w = 0 at 0.3 -> low value; w = 0.5 at 0.35 -> average of low(0.35) = 0.075 and high = 0.079 -> 0.077; w = 1 at 0.4 -> high value
        out = {round(p[0], 6): p for p in T.blend(self.LOW, self.HIGH)}
        self.assertAlmostEqual(out[0.3][1], 0.08, places=15)
        self.assertAlmostEqual(out[0.35][1], 0.077, places=15)
        self.assertAlmostEqual(out[0.4][1], 0.068, places=15)
        # Cp the same way: 0.3 -> low 0.026; 0.35 -> (0.025 + 0.0252) / 2 = 0.0251; 0.4 -> high 0.0238
        self.assertAlmostEqual(out[0.3][2], 0.026, places=15)
        self.assertAlmostEqual(out[0.35][2], 0.0251, places=15)
        self.assertAlmostEqual(out[0.4][2], 0.0238, places=15)

    def test_outside_the_overlap_each_run_is_unchanged(self):
        out = {round(p[0], 6): p for p in T.blend(self.LOW, self.HIGH)}
        for p in self.LOW:
            if p[0] < 0.3:
                self.assertEqual(tuple(out[round(p[0], 6)]), p)
        for p in self.HIGH:
            if p[0] > 0.4:
                self.assertEqual(tuple(out[round(p[0], 6)]), p)

    def test_blended_points_are_strictly_increasing_in_j(self):
        js = [p[0] for p in T.blend(self.LOW, self.HIGH)]
        self.assertEqual(js, sorted(set(js)))
        self.assertEqual(js, [0.1, 0.2, 0.3, 0.35, 0.4, 0.5, 0.6])

    def test_runs_that_do_not_overlap_are_joined(self):
        out = T.blend([(0.1, 0.1, 0.03), (0.2, 0.09, 0.028)], [(0.3, 0.08, 0.026), (0.4, 0.07, 0.024)])
        self.assertEqual([p[0] for p in out], [0.1, 0.2, 0.3, 0.4])


class BuildTests(unittest.TestCase):
    def setUp(self):
        self.curves = T.build_curves(MINI)

    def test_two_speeds_two_curves_with_static_anchors(self):
        self.assertEqual([c["rpm"] for c in self.curves], [2000.0, 4000.0])
        # static at 2000: Ct = 0.10 + 0.02 x 0.25 = 0.105, Cp = 0.03 + 0.01 x 0.25 = 0.0325; at 4000: 0.115 and 0.0375
        for got, want in zip(self.curves[0]["points"][0] + self.curves[1]["points"][0], [0.0, 0.105, 0.0325, 0.0, 0.115, 0.0375]):
            self.assertAlmostEqual(got, want, places=15)
        self.assertEqual(len(self.curves[0]["points"]), 3)

    def test_grouping_tolerance_boundary(self):
        def groups(rpms):
            runs = [{"rpm": r, "points": [(0.1, 0.1, 0.03), (0.2, 0.09, 0.028)]} for r in rpms]
            return [len(g) for g in T.group_runs(runs)]
        self.assertEqual(groups([5000.0, 5100.0]), [2])        # exactly 2 % above the lowest: same speed
        self.assertEqual(groups([5000.0, 5100.5]), [1, 1])     # just over: different speeds
        self.assertEqual(groups([5000.0, 5050.0, 5120.0]), [2, 1])   # tolerance counts from the group's lowest rpm

    def test_a_curve_that_already_starts_at_j_zero_gets_no_second_anchor(self):
        data = json.loads(json.dumps(MINI))
        data["wind_tunnel"]["r_2000.txt"].insert(0, {"j": 0.0, "ct": 0.1, "cp": 0.032, "eta": 0})
        self.assertEqual(T.build_curves(data)[0]["points"][0], [0.0, 0.1, 0.032])

    def test_run_name_without_rpm_is_refused(self):
        data = json.loads(json.dumps(MINI))
        data["wind_tunnel"]["badname.txt"] = data["wind_tunnel"].pop("r_2000.txt")
        with self.assertRaises(T.PropTableError):
            T.build_curves(data)


class EvaluateTests(unittest.TestCase):
    def setUp(self):
        self.c = T.build_curves(MINI)

    def test_nodes_are_exact(self):
        self.assertEqual(T.evaluate(self.c, 0.1, 2000.0), (0.09, 0.031, False))
        ct, cp, flag = T.evaluate(self.c, 0.0, 4000.0)
        self.assertAlmostEqual(ct, 0.115, places=15)
        self.assertAlmostEqual(cp, 0.0375, places=15)
        self.assertFalse(flag)

    def test_j_midpoint_between_two_nodes(self):
        # curve 2000, J = 0.2 between (0.1, 0.09) and (0.3, 0.05): Ct = 0.07, Cp = 0.028
        ct, cp, flag = T.evaluate(self.c, 0.2, 2000.0)
        self.assertAlmostEqual(ct, 0.07, places=15)
        self.assertAlmostEqual(cp, 0.028, places=15)
        self.assertFalse(flag)

    def test_rpm_midpoint_between_two_curves(self):
        # J = 0.1, rpm 3000: Ct = (0.09 + 0.10) / 2 = 0.095, Cp = (0.031 + 0.033) / 2 = 0.032
        ct, cp, flag = T.evaluate(self.c, 0.1, 3000.0)
        self.assertAlmostEqual(ct, 0.095, places=15)
        self.assertAlmostEqual(cp, 0.032, places=15)
        self.assertFalse(flag)

    def test_planar_table_is_reproduced_exactly(self):
        # Ct = 0.2 - 0.1 J + 0.00001 rpm, Cp = 0.05 - 0.02 J + 0.000002 rpm are planes: bilinear interpolation reproduces them everywhere inside
        def plane(rpm):
            return {"rpm": rpm, "points": [[j, 0.2 - 0.1 * j + 1e-5 * rpm, 0.05 - 0.02 * j + 2e-6 * rpm] for j in (0.0, 0.2, 0.7)]}
        curves = [plane(1000.0), plane(3000.0), plane(8000.0)]
        for j, rpm in ((0.05, 1500.0), (0.45, 5200.0), (0.6, 7900.0), (0.0, 1000.0)):
            ct, cp, flag = T.evaluate(curves, j, rpm)
            self.assertAlmostEqual(ct, 0.2 - 0.1 * j + 1e-5 * rpm, places=14)
            self.assertAlmostEqual(cp, 0.05 - 0.02 * j + 2e-6 * rpm, places=14)
            self.assertFalse(flag)

    def test_flag_truth_table(self):
        c = self.c      # curves at 2000 (J_end 0.3) and 4000 (J_end 0.5)
        flags = lambda j, rpm: T.evaluate(c, j, rpm)[2]  # noqa: E731
        self.assertFalse(flags(0.0, 2000.0))
        self.assertTrue(flags(-1e-12, 2000.0))              # descent
        self.assertFalse(flags(0.3, 2500.0))                # exactly the end of the lower curve
        self.assertTrue(flags(0.3 + 1e-12, 2500.0))         # beyond the lower curve although the upper one still has data
        self.assertTrue(flags(0.4, 3000.0))                 # lower curve ends at 0.3
        self.assertFalse(flags(0.3, 3999.0))
        self.assertTrue(flags(0.1, 1999.999))               # below the first curve
        self.assertFalse(flags(0.1, 4000.0))
        self.assertTrue(flags(0.1, 4000.001))               # above the last curve

    def test_clamped_values_equal_the_edge_values(self):
        c = self.c
        self.assertEqual(T.evaluate(c, 5.0, 2000.0)[:2], (0.05, 0.025))        # J beyond the end holds the last point
        for got, want in zip(T.evaluate(c, -0.5, 2000.0)[:2], (0.105, 0.0325)):   # descent holds the J = 0 point
            self.assertAlmostEqual(got, want, places=15)
        self.assertEqual(T.evaluate(c, 0.1, 100.0)[:2], T.evaluate(c, 0.1, 2000.0)[:2])
        self.assertEqual(T.evaluate(c, 0.1, 9000.0)[:2], T.evaluate(c, 0.1, 4000.0)[:2])

    def test_single_curve_table(self):
        one = [self.c[0]]
        self.assertEqual(T.evaluate(one, 0.1, 2000.0), (0.09, 0.031, False))
        self.assertTrue(T.evaluate(one, 0.1, 2001.0)[2])    # any other rpm is outside the single measured speed


class ValidationTests(unittest.TestCase):
    def test_limits_and_shape_rules(self):
        good = T.build_curves(MINI)
        T.validate_curves(good)
        def bad(edit):
            c = json.loads(json.dumps(good))
            edit(c)
            with self.assertRaises(T.PropTableError):
                T.validate_curves(c)
        bad(lambda c: c.clear())
        bad(lambda c: c.extend([c[0]] * 8))                              # 10 curves
        bad(lambda c: c[1].update(rpm=c[0]["rpm"]))                      # rpm not increasing
        bad(lambda c: c[0]["points"].__setitem__(0, [0.01, 0.1, 0.03]))  # J does not start at 0
        bad(lambda c: c[0]["points"].__setitem__(1, [0.0, 0.1, 0.03]))   # J not increasing
        bad(lambda c: c[0]["points"].__setitem__(1, [0.1, 0.1, 0.0]))    # Cp = 0
        bad(lambda c: c[0]["points"].__setitem__(1, [0.1, float("nan"), 0.03]))
        bad(lambda c: c[0].update(points=[[0.0, 0.1, 0.03]]))            # one point
        bad(lambda c: c[0].update(points=[[i / 100.0, 0.1, 0.03] for i in range(49)]))   # 49 points


class RealDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.curves = T.build_curves(REAL)
        cls.runs = {r["name"]: r for r in T.parse_runs(REAL)}

    def test_four_curves_at_the_expected_speeds(self):
        # 3040, 4030, mean(5007, 5042) = 5024.5, mean(5993, 6044) = 6018.5
        self.assertEqual([c["rpm"] for c in self.curves], [3040.0, 4030.0, 5024.5, 6018.5])

    def test_every_curve_starts_at_the_static_value_for_its_speed(self):
        for c in self.curves:
            ct, cp = T.static_at(REAL["static"], c["rpm"])
            self.assertEqual(c["points"][0], [0.0, ct, cp])

    def test_unmerged_runs_are_reproduced_exactly(self):
        for name in ("apce_12x6_0630od_3040.txt", "apce_12x6_0631od_4030.txt"):
            run = self.runs[name]
            curve = next(c for c in self.curves if c["rpm"] == run["rpm"])
            for j, ct, cp in run["points"]:
                got = T.evaluate([curve], j, run["rpm"])[:2]
                self.assertAlmostEqual(got[0], ct, places=14, msg=(name, j))
                self.assertAlmostEqual(got[1], cp, places=14, msg=(name, j))

    def test_merged_runs_are_reproduced_outside_their_overlap(self):
        # overlap of the ~5000 pair is [0.400771, 0.473]; of the ~6000 pair [0.333913, 0.394285] (J ranges of the raw files)
        pairs = (("apce_12x6_0632od_5007.txt", "apce_12x6_0633od_5042.txt", 5024.5), ("apce_12x6_0634od_5993.txt", "apce_12x6_0635od_6044.txt", 6018.5))
        for low_name, high_name, rpm in pairs:
            low, high = self.runs[low_name], self.runs[high_name]
            curve = next(c for c in self.curves if c["rpm"] == rpm)
            b0, a1 = high["points"][0][0], low["points"][-1][0]
            self.assertLess(b0, a1)
            for j, ct, cp in low["points"]:
                if j < b0:
                    got = T.evaluate([curve], j, rpm)[:2]
                    self.assertAlmostEqual(got[0], ct, places=14)
                    self.assertAlmostEqual(got[1], cp, places=14)
            for j, ct, cp in high["points"]:
                if j > a1:
                    got = T.evaluate([curve], j, rpm)[:2]
                    self.assertAlmostEqual(got[0], ct, places=14)
                    self.assertAlmostEqual(got[1], cp, places=14)

    def test_merged_curves_are_continuous_across_the_overlap(self):
        for c in self.curves:
            pts = c["points"]
            self.assertTrue(all(b[0] > a[0] for a, b in zip(pts, pts[1:])))
            self.assertLessEqual(len(pts), T.MAX_POINTS)
            # no jump bigger than the data's own step: neighbouring Ct values differ by less than 0.02
            self.assertTrue(all(abs(b[1] - a[1]) < 0.02 for a, b in zip(pts, pts[1:])), c["rpm"])

    def test_hover_value_against_the_static_fit(self):
        # Hand: static rows give Ct(3040) = 0.09845 and Ct(4030) = 0.10003; at 3553 rpm (weight 0.518) Ct = 0.09927.
        # The static fit used so far (mean over 3000-4500 rpm) is 0.100256, i.e. 1.0 % higher.
        ct, cp, flag = T.evaluate(self.curves, 0.0, 3553.0)
        self.assertAlmostEqual(ct, 0.09927, delta=5e-4)
        self.assertAlmostEqual((ct - 0.100256) / 0.100256, -0.0097, delta=0.006)
        self.assertFalse(flag)

    def test_measured_range(self):
        self.assertTrue(T.evaluate(self.curves, 0.0, 3039.0)[2])
        self.assertFalse(T.evaluate(self.curves, 0.0, 3040.0)[2])
        self.assertFalse(T.evaluate(self.curves, 0.0, 6018.5)[2])
        self.assertTrue(T.evaluate(self.curves, 0.0, 6019.0)[2])

    def test_provenance(self):
        prov = T.provenance(P.REPO_ROOT / "offline_pipeline/props/apce_12x6.json", self.curves)
        self.assertEqual(prov["curves"], 4)
        self.assertEqual(prov["measured_rpm_range"], [3040.0, 6018.5])
        self.assertEqual(len(prov["source_sha256"]), 64)


class MutationGapTests(unittest.TestCase):
    def test_runs_that_only_touch_are_joined_without_a_blend(self):
        # low ends exactly where high starts: a zero-width overlap must be a plain join (a blend would divide by zero)
        low = [(0.0, 0.10, 0.04), (0.3, 0.06, 0.03)]
        high = [(0.3, 0.06, 0.03), (0.6, 0.0, 0.02)]
        self.assertEqual(T.blend(low, high), [(0.0, 0.10, 0.04), (0.3, 0.06, 0.03), (0.6, 0.0, 0.02)])

    def test_flag_comes_from_either_bracketing_curve(self):
        # curve at 1000 reaches J 0.6, the one at 2000 only 0.3; hand check at J 0.45, rpm 1500 (inside the first, beyond the second)
        curves = [{"rpm": 1000.0, "points": [[0.0, 0.1, 0.03], [0.6, 0.0, 0.02]]},
                  {"rpm": 2000.0, "points": [[0.0, 0.12, 0.04], [0.3, 0.06, 0.03]]}]
        ct, cp, flag = T.evaluate(curves, 0.45, 1500.0)
        self.assertTrue(flag)
        # first curve at 0.45: Ct 0.025; second held at its end: 0.06; mean 0.0425
        self.assertAlmostEqual(ct, 0.0425, places=12)
        self.assertFalse(T.evaluate(curves, 0.3, 1500.0)[2])

    def test_nine_curves_are_refused(self):
        curves = [{"rpm": 1000.0 * (k + 1), "points": [[0.0, 0.1, 0.03], [0.5, 0.05, 0.02]]} for k in range(9)]
        with self.assertRaises(T.PropTableError):
            T.validate_curves(curves)
        T.validate_curves(curves[:8])


if __name__ == "__main__":
    unittest.main()
