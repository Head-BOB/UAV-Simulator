# Tier: D
import contextlib
import copy
import io
import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import run_case  # noqa: E402
from run_case import CaseError, load_case, main, run_set, vram_check  # noqa: E402

GEOM = b"prop"


def good_case(case_id: str = "c1") -> dict:
    # 100*100*100 = 1e6 cells x 55 B = 5.5e7 B <= 6e9 x (1 - 0.2) = 4.8e9 B. Numbers are
    # arbitrary test values, not FluidX3D data.
    return {
        "case_id": case_id,
        "kind": "cfd",
        "geometry_file": "prop.stl",
        "geometry_hash": hashlib.sha256(GEOM).hexdigest(),
        "rpm": 5000,
        "air": {"density_kg_m3": 1.225, "kinematic_viscosity_m2_s": 1.5e-5},
        "solver": {"name": "fake", "version": "0.0"},
        "grid": {"nx": 100, "ny": 100, "nz": 100},
        "memory": {"bytes_per_cell": 55, "vram_bytes": 6_000_000_000,
                   "headroom_fraction": 0.2},
        "vram_formula": run_case.VRAM_FORMULA,
    }


def write_case(root: Path, set_name: str, case: dict) -> Path:
    case_dir = root / set_name / case["case_id"]
    case_dir.mkdir(parents=True)
    (case_dir / "prop.stl").write_bytes(GEOM)
    (case_dir / "case.json").write_text(json.dumps(case), encoding="utf-8")
    return case_dir


class FakeSolver:
    def __init__(self, result=(1.5, 0.02)):
        self.calls = 0
        self.result = result

    def __call__(self, case, case_dir):
        self.calls += 1
        return self.result


class FakeClock:
    def __init__(self, values):
        self.values = list(values)

    def __call__(self):
        return self.values.pop(0)


class TmpTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)


class VramTest(unittest.TestCase):
    def test_fits_example(self):
        req, limit, fits = vram_check(good_case()["grid"], good_case()["memory"])
        self.assertEqual(req, 55_000_000)
        self.assertEqual(limit, 4.8e9)
        self.assertTrue(fits)

    def test_boundary_exact_and_one_over(self):
        # limit = 1000 x (1 - 0.5) = 500.0 (exact in binary). 10x10x5 x 1 B = 500 -> fits (<=).
        mem = {"bytes_per_cell": 1, "vram_bytes": 1000, "headroom_fraction": 0.5}
        self.assertEqual(vram_check({"nx": 10, "ny": 10, "nz": 5}, mem), (500, 500.0, True))
        # 501 cells x 1 B = 501 > 500 -> refused.
        self.assertFalse(vram_check({"nx": 501, "ny": 1, "nz": 1}, mem)[2])

    def test_headroom_is_applied(self):
        # Without headroom 600 <= 1000; with h = 0.5 the limit is 500 -> refused.
        mem = {"bytes_per_cell": 1, "vram_bytes": 1000, "headroom_fraction": 0.5}
        self.assertFalse(vram_check({"nx": 600, "ny": 1, "nz": 1}, mem)[2])


class LoadCaseTest(TmpTest):
    def load(self, mutate):
        case = good_case()
        mutate(case)
        case_dir = write_case(self.root, "s", case)
        return load_case(case_dir)

    def assertRejects(self, mutate):
        with self.assertRaises(CaseError):
            self.load(mutate)

    def test_good_case_loads(self):
        self.assertEqual(self.load(lambda c: None)["case_id"], "c1")

    def test_missing_key(self):
        self.assertRejects(lambda c: c.pop("rpm"))

    def test_unknown_key(self):
        self.assertRejects(lambda c: c.update(extra=1))

    def test_nested_unknown_key(self):
        self.assertRejects(lambda c: c["grid"].update(nw=1))

    def test_bool_grid(self):
        self.assertRejects(lambda c: c["grid"].update(nx=True))

    def test_zero_grid(self):
        self.assertRejects(lambda c: c["grid"].update(ny=0))

    def test_rpm_nan_zero_negative_bool(self):
        for bad in (float("nan"), 0, -1, True, float("inf")):
            with self.subTest(bad=bad):
                case = good_case()
                case["rpm"] = bad
                case_dir = self.root / f"s{abs(hash(repr(bad)))}" / "c1"
                case_dir.mkdir(parents=True)
                (case_dir / "prop.stl").write_bytes(GEOM)
                (case_dir / "case.json").write_text(json.dumps(case), encoding="utf-8")
                with self.assertRaises(CaseError):
                    load_case(case_dir)

    def test_headroom_zero_and_one(self):
        self.assertRejects(lambda c: c["memory"].update(headroom_fraction=0))
        # second call needs a fresh set dir
        case = good_case()
        case["memory"]["headroom_fraction"] = 1
        case_dir = write_case(self.root, "s2", case)
        with self.assertRaises(CaseError) as ctx:
            load_case(case_dir)
        # h = 1 would also fail the VRAM check (limit 0); assert the headroom rule itself fired.
        self.assertIn("headroom_fraction must be < 1", str(ctx.exception))

    def test_case_id_must_match_folder(self):
        case = good_case("other")
        case_dir = write_case(self.root, "s", case)
        (case_dir.parent / "c1").mkdir()
        (case_dir.parent / "c1" / "case.json").write_text(json.dumps(case), encoding="utf-8")
        with self.assertRaises(CaseError):
            load_case(case_dir.parent / "c1")

    def test_kind_unknown_and_fea(self):
        self.assertRejects(lambda c: c.update(kind="bogus"))
        case = good_case()
        case["kind"] = "fea"
        case_dir = write_case(self.root, "s3", case)
        with self.assertRaises(CaseError):
            load_case(case_dir)

    def test_formula_changed(self):
        self.assertRejects(lambda c: c.update(vram_formula="always fits"))

    def test_geometry_hash_mismatch(self):
        self.assertRejects(lambda c: c.update(geometry_hash="0" * 64))

    def test_geometry_path_traversal(self):
        self.assertRejects(lambda c: c.update(geometry_file="../prop.stl"))

    def test_does_not_fit(self):
        self.assertRejects(lambda c: c["grid"].update(nx=100_000))


class RunSetTest(TmpTest):
    def test_happy_path_results_and_summary(self):
        write_case(self.root, "s", good_case())
        summary = self.root / "summary.md"
        solver = FakeSolver()
        code = run_set(self.root / "s", {"fake": solver}, summary, FakeClock([10.0, 12.5]))
        self.assertEqual(code, 0)
        self.assertEqual(solver.calls, 1)
        got = json.loads((self.root / "s" / "c1" / "results" / "results.json").read_text())
        case = good_case()
        self.assertEqual(got, {
            "case_id": "c1", "kind": "cfd",
            "geometry_hash": hashlib.sha256(GEOM).hexdigest(),
            "rpm": 5000.0, "grid": case["grid"], "cells": 1_000_000,
            "vram_required_bytes": 55_000_000, "vram_limit_bytes": 4.8e9,
            "vram_formula": run_case.VRAM_FORMULA, "solver": case["solver"],
            "air": case["air"], "thrust_n": 1.5, "torque_nm": 0.02,
            "elapsed_s": 2.5, "status": "ok",
        })
        self.assertEqual(
            summary.read_text(),
            "case=c1 rpm=5000.0 thrust_n=1.5 torque_nm=0.02 elapsed_s=2.5\n",
        )

    def test_refusal_runs_nothing(self):
        write_case(self.root, "s", good_case("c1"))
        bad = good_case("c2")
        bad["grid"]["nx"] = 100_000
        write_case(self.root, "s", bad)
        solver = FakeSolver()
        code = run_set(self.root / "s", {"fake": solver}, None, FakeClock([]))
        self.assertEqual(code, 1)
        self.assertEqual(solver.calls, 0)
        self.assertFalse((self.root / "s" / "c1" / "results").exists())

    def test_nan_output_fails_without_results(self):
        write_case(self.root, "s", good_case())
        solver = FakeSolver((float("nan"), 0.02))
        code = run_set(self.root / "s", {"fake": solver}, None, FakeClock([0.0, 1.0]))
        self.assertEqual(code, 1)
        self.assertFalse((self.root / "s" / "c1" / "results" / "results.json").exists())

    def test_unknown_solver_fails(self):
        write_case(self.root, "s", good_case())
        self.assertEqual(run_set(self.root / "s", {}, None, FakeClock([])), 1)

    def test_missing_or_empty_set(self):
        self.assertEqual(run_set(self.root / "nope", {}, None), 1)
        (self.root / "empty").mkdir()
        self.assertEqual(run_set(self.root / "empty", {}, None), 1)

    def test_real_solver_not_configured(self):
        write_case(self.root, "s", good_case())
        case = copy.deepcopy(good_case())
        case["solver"]["name"] = "fluidx3d"
        (self.root / "s" / "c1" / "case.json").write_text(json.dumps(case), encoding="utf-8")
        self.assertEqual(run_set(self.root / "s", run_case.SOLVERS, None), 1)


class MainTest(TmpTest):
    def test_bad_set_names(self):
        for bad in ("../x", "a/b", "", "a b"):
            with self.subTest(bad=bad):
                err = io.StringIO()
                with contextlib.redirect_stderr(err):
                    code = main(["--set", bad, "--cases-root", str(self.root)])
                self.assertEqual(code, 1)
                # Message check: a missing directory would also exit 1, so prove the name was rejected.
                self.assertIn("bad set name", err.getvalue())


if __name__ == "__main__":
    unittest.main()
