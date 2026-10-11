# Tier: D
"""Tests for controller_export (controller_config.json): class defaults, range rules, provenance, place in the build outputs."""
import copy
import json
import math
import unittest
from pathlib import Path

import build_vehicle as B
import controller_export as X
import partsdb as P

HERE = Path(__file__).resolve().parent
REF = json.loads((HERE / "builds" / "reference_quad.build.json").read_text(encoding="utf-8"))
FPV = json.loads((HERE / "builds" / "fpv_5in.build.json").read_text(encoding="utf-8"))
DOC = json.loads((P.DEFAULT_DB / X.CLASSES_FILE).read_text(encoding="utf-8"))


class ClassFileTests(unittest.TestCase):
    def test_both_classes_are_valid(self):
        for name, entry in DOC["classes"].items():
            X.validate_class(name, entry)

    def test_class_values_are_the_documented_engineering_choices(self):
        std, fpv = DOC["classes"]["quad_standard"], DOC["classes"]["quad_fpv_5in"]
        self.assertAlmostEqual(std["max_tilt_rad"], math.radians(35.0), places=12)
        self.assertAlmostEqual(std["max_rate_rps"], math.radians(220.0), places=12)
        self.assertAlmostEqual(fpv["max_tilt_rad"], math.radians(60.0), places=12)
        self.assertAlmostEqual(fpv["acro_max_rate_rps"], math.radians(670.0), places=12)
        self.assertEqual(std["mixer_axis_authority"], 0.2)         # mirrors the 0.2 in physics_core/src/mixer.rs
        self.assertEqual(fpv["mixer_axis_authority"], 0.2)
        self.assertGreater(fpv["max_tilt_rad"], std["max_tilt_rad"])  # the master prompt: the FPV speed test needs more tilt

    def test_every_value_is_declared_assumed(self):
        self.assertEqual(DOC["provenance"]["source_type"], "ASSUMED")
        self.assertIn("TODO(HUMAN-VERIFY)", DOC["provenance"]["note"])

    def test_range_rules_reject(self):
        base = DOC["classes"]["quad_standard"]
        bad = [("tilt 0", {"max_tilt_rad": 0.0}), ("tilt 81 deg", {"max_tilt_rad": math.radians(81)}), ("rate 0", {"max_rate_rps": 0.0}),
               ("down accel 0.9 g", {"max_accel_down_mps2": 0.9 * 9.80665}), ("expo 1.1", {"acro_expo": 1.1}), ("expo -0.1", {"acro_expo": -0.1}),
               ("td negative", {"rate_derivative_time_s": -0.1}), ("authority 0.6", {"mixer_axis_authority": 0.6}),
               ("nan", {"velocity_kp_per_s": float("nan")}), ("string", {"velocity_kp_per_s": "3"}), ("bool", {"velocity_kp_per_s": True}),
               ("ki 0", {"velocity_ki_per_s2": 0.0}), ("corner 0", {"rate_integral_corner_rps": 0.0})]
        for name, change in bad:
            with self.assertRaises(X.ControllerExportError, msg=name):
                X.validate_class("t", {**base, **change})
        X.validate_class("t", {**base, "max_tilt_rad": math.radians(80), "acro_expo": 1.0, "rate_derivative_time_s": 0.0})
        missing = dict(base)
        del missing["acro_expo"]
        with self.assertRaises(X.ControllerExportError):
            X.validate_class("t", missing)
        with self.assertRaises(X.ControllerExportError):
            X.validate_class("t", {**base, "extra": 1.0})


class BuildTests(unittest.TestCase):
    def test_output_has_class_version_values_and_provenance(self):
        out = X.build_controller(P.DEFAULT_DB, "quad_fpv_5in", 0.3)
        self.assertEqual((out["config_version"], out["vehicle_class"]), (1, "quad_fpv_5in"))
        for k, v in DOC["classes"]["quad_fpv_5in"].items():
            self.assertEqual(out[k], v)
        self.assertEqual(out["hover_duty"], 0.3)
        self.assertEqual(out["provenance"]["source_file"], "parts_db/controller_classes.json")
        self.assertEqual(len(out["provenance"]["source_sha256"]), 64)
        self.assertEqual(out["provenance"]["source_type"], "ASSUMED")

    def test_unknown_class_missing_file_and_bad_version_fail(self):
        with self.assertRaises(X.ControllerExportError):
            X.build_controller(P.DEFAULT_DB, "hexacopter", 0.3)
        with self.assertRaises(X.ControllerExportError):
            X.build_controller(HERE, "quad_standard", 0.3)        # no classes file there
        import tempfile
        with tempfile.TemporaryDirectory() as t:
            doc = copy.deepcopy(DOC)
            doc["version"] = 2
            (Path(t) / X.CLASSES_FILE).write_text(json.dumps(doc), encoding="utf-8")
            with self.assertRaises(X.ControllerExportError):
                X.build_controller(Path(t), "quad_standard", 0.3)

    def test_hover_duty_must_be_a_duty(self):
        for bad in (0.0, -0.1, 1.01, float("nan"), "0.3", None):
            with self.assertRaises(X.ControllerExportError, msg=repr(bad)):
                X.build_controller(P.DEFAULT_DB, "quad_standard", bad)
        self.assertEqual(X.build_controller(P.DEFAULT_DB, "quad_standard", 1.0)["hover_duty"], 1.0)

    def test_builder_uses_the_sidecar_hover_duty(self):
        for spec in (REF, FPV):
            res = B.build(spec)
            self.assertEqual(res.controller["hover_duty"], res.sidecar["hover_duty_estimate"])

    def test_builder_writes_the_file_for_both_classes(self):
        for spec, cls in ((REF, "quad_standard"), (FPV, "quad_fpv_5in")):
            res = B.build(spec)
            self.assertTrue(res.ok, res.findings)
            self.assertEqual(res.controller["vehicle_class"], cls)
            self.assertEqual(json.loads(B.render(res)["controller_config.json"]), res.controller)

    def test_a_bad_class_file_stops_the_build_with_a_finding(self):
        res = B.build({**REF, "vehicle_class": "quad_standard"}, P.DEFAULT_DB)
        self.assertTrue(res.ok)
        import shutil, tempfile
        with tempfile.TemporaryDirectory() as t:
            repo = Path(t)
            shutil.copytree(P.DEFAULT_DB, repo / "parts_db", ignore=shutil.ignore_patterns("source_cache", "parts_index.sqlite"))
            (repo / "offline_pipeline" / "props").mkdir(parents=True)
            shutil.copy(P.REPO_ROOT / "offline_pipeline/props/apce_12x6.json", repo / "offline_pipeline/props/apce_12x6.json")
            path = repo / "parts_db" / X.CLASSES_FILE
            doc = json.loads(path.read_text(encoding="utf-8"))
            doc["classes"]["quad_standard"]["max_tilt_rad"] = 0.0
            path.write_text(json.dumps(doc), encoding="utf-8")
            res = B.build(REF, repo / "parts_db")
            self.assertFalse(res.ok)
            self.assertTrue(any(f.rule == "E-CONTROLLER" for f in res.findings))


if __name__ == "__main__":
    unittest.main()
