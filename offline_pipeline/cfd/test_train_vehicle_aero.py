# Tier: D
"""Tests for train_vehicle_aero: exact fit of a formula CSV, held-out error, kind rules, binding hash recomputed independently, verify catches changes."""
import copy
import csv
import hashlib
import io
import json
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np
import onnx

import train_vehicle_aero as TV
import vehicle_sweep as S

ROOT = Path(__file__).resolve().parents[2]
REF = ROOT / "tools" / "parts" / "builds" / "reference_quad"
FPV = ROOT / "tools" / "parts" / "builds" / "fpv_5in"
NOW = "2026-10-11T00:00:00+00:00"


def copy_build(src: Path, dst: Path) -> Path:
    dst.mkdir(parents=True)
    for name in ("assembly.json", "vehicle_config.json"):
        shutil.copy(src / name, dst / name)
    return dst


def trained_csv(text: str) -> str:
    """The synthetic text with its provenance column changed to what the real sweep writes (test only: the numbers stay synthetic)."""
    return text.replace("SYNTHETIC formula, not CFD", "preliminary CFD, unvalidated")


class FitTests(unittest.TestCase):
    def test_weights_recover_the_formula_and_zero_speed_gives_zero_force(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "c.csv").write_text(S.synthetic_csv(REF), encoding="utf-8")
            r = TV.train(REF, d / "c.csv", d / "m.onnx", now=NOW)
            # F_axis = scale V^2 (c0 + c1 a + c2 b + c3 a^2 + c4 b^2) with the formula of synthetic_force (scale = the CSV's ref area)
            scale = float(next(csv.DictReader(io.StringIO(S.synthetic_csv(REF))))["ref_area_m2"])
            want = scale * np.array([[-0.05, 0.0, 0.0], [0.02, 0.02, -1.50], [0.01, -0.60, 0.02], [-0.30, 0.0, -0.20], [-0.25, 0.02, 0.0]])
            np.testing.assert_allclose(r["weights"], want, rtol=1e-6, atol=1e-9)
            self.assertLess(r["rms"], 1e-6)
            # V = 0: the design row is all zero, so the force is exactly zero whatever the angles
            self.assertTrue(np.all(TV.T.design(np.array([0.0]), np.array([0.3]), np.array([-0.2])) @ r["weights"] == 0.0))

    def test_heldout_error_by_hand_on_exact_data(self):
        # a model that is exactly quadratic in the angles: leaving a speed out changes nothing, so the held-out error is ~0
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "c.csv").write_text(S.synthetic_csv(REF), encoding="utf-8")
            r = TV.train(REF, d / "c.csv", d / "m.onnx", now=NOW)
            self.assertLess(r["holdout"], 1e-6)
            self.assertEqual(r["envelope"]["speed_mps"], [0.0, 20.0])
            self.assertAlmostEqual(r["envelope"]["alpha_rad"], 20.0 * 3.141592653589793 / 180.0, places=12)


class ModelEvaluationTests(unittest.TestCase):
    """Runs the ONNX graph itself (onnx reference evaluator, pure numpy) and compares with the formula, by hand."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        (d / "c.csv").write_text(S.synthetic_csv(REF), encoding="utf-8")
        TV.train(REF, d / "c.csv", d / "m.onnx", now=NOW)
        from onnx.reference import ReferenceEvaluator
        cls.ev = ReferenceEvaluator(str(d / "m.onnx"))
        cls.area = float(next(csv.DictReader(io.StringIO(S.synthetic_csv(REF))))["ref_area_m2"])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_model(self, v, a, b):
        pred, unc, inside = self.ev.run(None, {"inputs": np.array([[v, a, b]], dtype=np.float32)})
        return pred[0], float(unc[0, 0]), float(inside[0, 0])

    def test_force_at_hand_points(self):
        # F = A V^2 (c0 + c1 a + c2 b + c3 a^2 + c4 b^2) with x: (-0.05, 0.02, 0.01, -0.30, -0.25), y: (0, 0.02, -0.60, 0, 0.02), z: (0, -1.5, 0.02, -0.2, 0)
        a = self.area
        f, _, inside = self.run_model(10.0, 0.0, 0.0)
        np.testing.assert_allclose(f, [-5.0 * a, 0.0, 0.0], atol=1e-6 * a)
        self.assertEqual(inside, 1.0)
        f, _, _ = self.run_model(10.0, 0.1, 0.0)             # x: -0.05 + 0.002 - 0.003 = -0.051 ; z: -0.15 - 0.002 = -0.152
        np.testing.assert_allclose(f, [-5.1 * a, 0.002 * 100 * a, -15.2 * a], rtol=2e-4, atol=1e-6 * a)
        f, _, _ = self.run_model(20.0, 0.0, 0.2)             # y: -0.60 x 0.2 + 0.02 x 0.04 = -0.1192 x 400 ; x: -0.05 + 0.002 - 0.25 x 0.04 = -0.058
        np.testing.assert_allclose(f, [-0.058 * 400 * a, -0.1192 * 400 * a, (0.02 * 0.2) * 400 * a], rtol=2e-4, atol=1e-6 * a)

    def test_zero_speed_gives_zero_force_whatever_the_angles(self):
        f, _, _ = self.run_model(0.0, 0.3, -0.2)
        np.testing.assert_array_equal(f, [0.0, 0.0, 0.0])

    def test_force_scales_with_speed_squared(self):
        f10, _, _ = self.run_model(10.0, 0.1, 0.05)
        f20, _, _ = self.run_model(20.0, 0.1, 0.05)
        np.testing.assert_allclose(f20, 4.0 * f10, rtol=1e-5)

    def test_envelope_flag_follows_the_sweep_box(self):
        # the sweep box is V in [0, 20] m/s, |alpha| and |beta| <= 20 deg = 0.349066 rad
        self.assertEqual(self.run_model(20.0, 0.3, -0.3)[2], 1.0)
        self.assertEqual(self.run_model(25.0, 0.0, 0.0)[2], 0.0)
        self.assertEqual(self.run_model(10.0, 0.40, 0.0)[2], 0.0)
        self.assertEqual(self.run_model(10.0, 0.0, -0.40)[2], 0.0)

    def test_uncertainty_is_the_fit_rms(self):
        _, unc, _ = self.run_model(10.0, 0.0, 0.0)
        self.assertLess(unc, 1e-6)


class KindTests(unittest.TestCase):
    def run_kind(self, text):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "c.csv").write_text(text, encoding="utf-8")
            return TV.train(REF, d / "c.csv", d / "m.onnx", now=NOW)["kind"]

    def test_synthetic_is_a_stub_and_cfd_provenance_is_trained(self):
        text = S.synthetic_csv(REF)
        self.assertEqual(self.run_kind(text), "stub")
        self.assertEqual(self.run_kind(trained_csv(text)), "trained")

    def test_a_mix_of_provenances_is_a_stub(self):
        lines = trained_csv(S.synthetic_csv(REF)).splitlines()
        lines[1] = lines[1].replace("preliminary CFD, unvalidated", "SYNTHETIC formula, not CFD")
        self.assertEqual(self.run_kind("\n".join(lines) + "\n"), "stub")

    def test_trained_needs_three_speeds(self):
        rows = list(csv.reader(io.StringIO(trained_csv(S.synthetic_csv(REF)))))
        keep = [rows[0]] + [r for r in rows[1:] if r[1] in ("5.0", "10.0")]
        buf = io.StringIO()
        csv.writer(buf, lineterminator="\n").writerows(keep)
        with self.assertRaises(TV.TrainError):
            self.run_kind(buf.getvalue())


class MetadataTests(unittest.TestCase):
    def test_metadata_sidecar_and_independent_binding_hash(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "c.csv").write_text(S.synthetic_csv(REF), encoding="utf-8")
            TV.train(REF, d / "c.csv", d / "m.onnx", now=NOW)
            meta = {p.key: p.value for p in onnx.load(str(d / "m.onnx")).metadata_props}
            side = json.loads((d / "m.onnx.model.json").read_text(encoding="utf-8"))
            cfg = hashlib.sha256((REF / "vehicle_config.json").read_bytes()).hexdigest()
            data = hashlib.sha256((d / "c.csv").read_bytes()).hexdigest()
            geo = json.loads((d / "c.csv").read_text().splitlines()[1].split(",")[-4:-3][0:1] and "{}") if False else None
            self.assertEqual(meta["config_hash"], cfg)
            self.assertEqual(meta["training_data_sha256"], data)
            self.assertEqual(meta["binding_hash"], hashlib.sha256(f"{cfg}|{meta['geometry_hash']}|{data}|1".encode()).hexdigest())
            self.assertEqual(side["binding_hash"], meta["binding_hash"])
            self.assertEqual(side["model_sha256"], hashlib.sha256((d / "m.onnx").read_bytes()).hexdigest())
            self.assertEqual((meta["kind"], meta["cfd_sanity_status"], meta["propellers"], meta["vehicle_class"], meta["build_name"]),
                             ("stub", "NOT_RUN", "omit", "quad_standard", "reference_quad"))
            self.assertIn("NOT VALIDATED", meta["provenance"])
            self.assertIn("force only", side["model_form"])
            self.assertIn("preliminary CFD, unvalidated", side["limitations"])
            self.assertEqual(side["cfd_sanity_statement"], TV.NOT_RUN_STATEMENT)

    def test_sanity_report_status_is_carried(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "c.csv").write_text(S.synthetic_csv(REF), encoding="utf-8")
            (d / "san.json").write_text(json.dumps({"status": "PARTIAL", "statement": "Only one of the checks has been run."}), encoding="utf-8")
            TV.train(REF, d / "c.csv", d / "m.onnx", d / "san.json", now=NOW)
            side = json.loads((d / "m.onnx.model.json").read_text(encoding="utf-8"))
            self.assertEqual((side["cfd_sanity_status"], side["cfd_sanity_statement"]), ("PARTIAL", "Only one of the checks has been run."))

    def test_deterministic_with_a_fixed_stamp(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "c.csv").write_text(S.synthetic_csv(REF), encoding="utf-8")
            TV.train(REF, d / "c.csv", d / "a.onnx", now=NOW)
            TV.train(REF, d / "c.csv", d / "b.onnx", now=NOW)
            self.assertEqual((d / "a.onnx").read_bytes(), (d / "b.onnx").read_bytes())

    def test_other_class_and_other_build_give_different_bindings(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "r.csv").write_text(S.synthetic_csv(REF), encoding="utf-8")
            (d / "f.csv").write_text(S.synthetic_csv(FPV), encoding="utf-8")
            a = TV.train(REF, d / "r.csv", d / "r.onnx", now=NOW)["binding_hash"]
            b = TV.train(FPV, d / "f.csv", d / "f.onnx", now=NOW)["binding_hash"]
            self.assertNotEqual(a, b)


class RefusalTests(unittest.TestCase):
    def test_csv_for_another_geometry_is_refused(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "c.csv").write_text(S.synthetic_csv(FPV), encoding="utf-8")          # FPV rows against the reference build
            with self.assertRaises(TV.TrainError) as e:
                TV.train(REF, d / "c.csv", d / "m.onnx", now=NOW)
            self.assertIn("different geometry", str(e.exception))

    def test_csv_without_the_vehicle_columns_or_with_mixed_geometry(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "old.csv").write_text("case_id,speed_mps\n1,5\n", encoding="utf-8")
            with self.assertRaises(TV.TrainError):
                TV.train(REF, d / "old.csv", d / "m.onnx", now=NOW)
            rows = S.synthetic_csv(REF).splitlines()
            fields = rows[1].split(",")
            fields[-4] = "0" * 64
            rows[1] = ",".join(fields)
            (d / "mix.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")
            with self.assertRaises(TV.TrainError):
                TV.train(REF, d / "mix.csv", d / "m.onnx", now=NOW)

    def test_sweep_that_cannot_determine_the_terms_is_refused(self):
        rows = list(csv.reader(io.StringIO(S.synthetic_csv(REF))))
        keep = [rows[0]] + [r for r in rows[1:] if r[3] == "0.0"]       # no sideslip variation
        buf = io.StringIO()
        csv.writer(buf, lineterminator="\n").writerows(keep)
        with tempfile.TemporaryDirectory() as t:
            (Path(t) / "c.csv").write_text(buf.getvalue(), encoding="utf-8")
            with self.assertRaises(TV.TrainError):
                TV.train(REF, Path(t) / "c.csv", Path(t) / "m.onnx", now=NOW)


class VerifyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = Path(self.tmp.name)
        self.build = copy_build(REF, self.d / "build")
        (self.d / "c.csv").write_text(S.synthetic_csv(self.build), encoding="utf-8")
        TV.train(self.build, self.d / "c.csv", self.d / "m.onnx", now=NOW)

    def tearDown(self):
        self.tmp.cleanup()

    def test_clean_model_verifies(self):
        self.assertEqual(TV.verify(self.build, self.d / "m.onnx", self.d / "c.csv"), [])

    def test_changed_model_byte(self):
        b = bytearray((self.d / "m.onnx").read_bytes())
        b[-5] ^= 0x01
        (self.d / "m.onnx").write_bytes(bytes(b))
        self.assertTrue(any("model_sha256" in p for p in TV.verify(self.build, self.d / "m.onnx")))

    def test_changed_csv_config_and_geometry(self):
        (self.d / "c.csv").write_text((self.d / "c.csv").read_text() + " ", encoding="utf-8")
        self.assertTrue(any("CSV differs" in p for p in TV.verify(self.build, self.d / "m.onnx", self.d / "c.csv")))
        cfg = json.loads((self.build / "vehicle_config.json").read_text())
        cfg["mass_kg"] = cfg["mass_kg"] + 0.01
        (self.build / "vehicle_config.json").write_text(json.dumps(cfg), encoding="utf-8")
        self.assertTrue(any("vehicle_config.json changed" in p for p in TV.verify(self.build, self.d / "m.onnx")))
        a = json.loads((self.build / "assembly.json").read_text())
        next(p for p in a["parts"] if p["instance"] == "pack")["visual"]["dims_m"][0] += 0.01
        (self.build / "assembly.json").write_text(json.dumps(a), encoding="utf-8")
        self.assertTrue(any("geometry changed" in p for p in TV.verify(self.build, self.d / "m.onnx")))

    def test_tampered_sidecar(self):
        side = json.loads((self.d / "m.onnx.model.json").read_text())
        side["binding_hash"] = "0" * 64
        (self.d / "m.onnx.model.json").write_text(json.dumps(side), encoding="utf-8")
        problems = TV.verify(self.build, self.d / "m.onnx")
        self.assertTrue(any("metadata binding_hash" in p for p in problems) and any("not the hash of its parts" in p for p in problems))

    def test_missing_files(self):
        self.assertEqual(len(TV.verify(self.build, self.d / "nope.onnx")), 1)


class CliTests(unittest.TestCase):
    def test_cli_round_trip_and_errors(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            (d / "c.csv").write_text(S.synthetic_csv(REF), encoding="utf-8")
            self.assertEqual(TV.main(["train", "--build", str(REF), "--csv", str(d / "c.csv"), "--out", str(d / "m.onnx"), "--now", NOW]), 0)
            self.assertEqual(TV.main(["verify", "--build", str(REF), "--model", str(d / "m.onnx"), "--csv", str(d / "c.csv")]), 0)
            self.assertEqual(TV.main(["verify", "--build", str(FPV), "--model", str(d / "m.onnx")]), 1)          # wrong build
            self.assertEqual(TV.main(["train", "--build", str(FPV), "--csv", str(d / "c.csv"), "--out", str(d / "x.onnx")]), 1)


if __name__ == "__main__":
    unittest.main()
