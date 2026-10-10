# Tier: D
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import onnx
from onnx.reference import ReferenceEvaluator

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_aero_from_csv as t  # noqa: E402

SAMPLE = Path(__file__).resolve().parent / "sample" / "synthetic_quad_cfd.csv"
# True synthetic coefficients used to write the sample (N per (m/s)^2), columns [1, a, b, a^2, b^2], axes fx fy fz.
W_TRUE = np.array([[-0.020, 0.002, -0.001], [0.004, 0.0, 0.0005], [0.0, 0.003, 0.0], [-0.06, 0.0, 0.0], [-0.01, 0.0, 0.0]])


def run(path: Path, x):
    ref = ReferenceEvaluator(onnx.load(str(path)))
    return ref.run(None, {"inputs": np.array([x], dtype=np.float32)})


class TrainTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.out = self.dir / "m.onnx"

    def tearDown(self):
        self.tmp.cleanup()

    def test_fit_recovers_known_coefficients(self):
        w = t.fit(t.load_csv(SAMPLE))
        np.testing.assert_allclose(w, W_TRUE, atol=2e-8, rtol=0)  # CSV has 9 significant digits

    def test_onnx_matches_hand_calc(self):
        t.train(SAMPLE, self.out)
        pred, unc, env = run(self.out, [10.0, 0.1, 0.0])
        # F = V^2 (c0 + c1 a + c3 a^2): fx = 100 (-0.020 + 0.1*0.004 + 0.01*-0.06) = 100 * -0.0202 = -2.02
        # fy = 100 (0.002 + 0.1*0 + 0.01*0) = 0.2 ; fz = 100 (-0.001 + 0.1*0.0005) = -0.095
        np.testing.assert_allclose(pred[0], [-2.02, 0.2, -0.095], atol=1e-5)
        self.assertEqual(env[0, 0], 1.0)
        self.assertLess(float(unc[0, 0]), 1e-6)  # exact data: residual only from the 9-digit rounding

    def test_beta_terms(self):
        t.train(SAMPLE, self.out)
        pred, _, _ = run(self.out, [20.0, 0.0, 0.2])
        # V^2 = 400; b = 0.2: fx = 400 (-0.020 + 0 + -0.01*0.04) = 400 * -0.0204 = -8.16
        # fy = 400 (0.002 + 0.2*0.003) = 400 * 0.0026 = 1.04 ; fz = 400 (-0.001) = -0.4
        np.testing.assert_allclose(pred[0], [-8.16, 1.04, -0.4], atol=1e-4)

    def test_zero_speed_zero_force(self):
        t.train(SAMPLE, self.out)
        pred, _, env = run(self.out, [0.0, 0.2, 0.1])
        np.testing.assert_allclose(pred[0], [0.0, 0.0, 0.0], atol=0.0)
        self.assertEqual(env[0, 0], 1.0)

    def test_envelope_edges(self):
        t.train(SAMPLE, self.out)
        a20 = float(np.float32(math.radians(20.0)))
        self.assertEqual(run(self.out, [20.0, a20, 0.0])[2][0, 0], 1.0)   # exactly on the box edge: inside
        self.assertEqual(run(self.out, [20.5, 0.0, 0.0])[2][0, 0], 0.0)   # faster than any case
        self.assertEqual(run(self.out, [10.0, 0.4, 0.0])[2][0, 0], 0.0)   # alpha 0.4 rad > 20 deg
        self.assertEqual(run(self.out, [10.0, 0.0, -0.4])[2][0, 0], 0.0)  # beta
        self.assertEqual(run(self.out, [-1.0, 0.0, 0.0])[2][0, 0], 0.0)   # negative airspeed
        # Negative angles: -0.3 rad (17.2 deg) is inside the +-20 deg box, -0.4 rad is outside (regression for a lower bound).
        self.assertEqual(run(self.out, [10.0, -0.3, -0.3])[2][0, 0], 1.0)
        self.assertEqual(run(self.out, [10.0, -0.4, 0.0])[2][0, 0], 0.0)

    def test_synthetic_is_stub_with_provenance(self):
        r = t.train(SAMPLE, self.out)
        self.assertEqual(r["kind"], "stub")
        meta = {p.key: p.value for p in onnx.load(str(self.out)).metadata_props}
        self.assertEqual(meta["kind"], "stub")
        self.assertIn("SYNTHETIC", meta["provenance"])
        self.assertIn("NOT VALIDATED", meta["provenance"])
        self.assertEqual(meta["geometry_hash"], t.sha256_file(t.GEOMETRY))
        self.assertEqual(meta["config_hash"], t.sha256_file(t.CONFIG))
        self.assertEqual(meta["training_data_sha256"], t.sha256_file(SAMPLE))
        self.assertEqual(meta["training_rows"], "27")

    def test_cfd_provenance_is_trained(self):
        csv = self.dir / "cfd.csv"
        csv.write_text(SAMPLE.read_text().replace("SYNTHETIC sample - not CFD", '"' + t.TRAINED_PROVENANCE + '"'))
        r = t.train(csv, self.out)
        self.assertEqual(r["kind"], "trained")
        self.assertIsNotNone(r["holdout"])
        self.assertLess(r["holdout"], 1e-5)  # data is exactly in the model family
        meta = {p.key: p.value for p in onnx.load(str(self.out)).metadata_props}
        self.assertIn("preliminary CFD, unvalidated", meta["provenance"])

    def test_mixed_provenance_is_stub(self):
        lines = SAMPLE.read_text().splitlines()
        lines[1] = lines[1].replace("SYNTHETIC sample - not CFD", '"' + t.TRAINED_PROVENANCE + '"')
        csv = self.dir / "mixed.csv"
        csv.write_text("\n".join(lines) + "\n")
        self.assertEqual(t.train(csv, self.out)["kind"], "stub")

    def test_trained_needs_three_speeds(self):
        csv = self.dir / "two.csv"
        rows = [ln for ln in SAMPLE.read_text().splitlines() if ",20.0," not in ln]
        csv.write_text("\n".join(rows).replace("SYNTHETIC sample - not CFD", '"' + t.TRAINED_PROVENANCE + '"') + "\n")
        with self.assertRaises(t.TrainError):
            t.train(csv, self.out)

    def test_rank_deficient_sweep_rejected(self):
        csv = self.dir / "alpha_only.csv"
        # keep only rows with beta == 0
        rows = [SAMPLE.read_text().splitlines()[0]] + [ln for ln in SAMPLE.read_text().splitlines()[1:] if ln.split(",")[3] == "0.0"]
        csv.write_text("\n".join(rows) + "\n")
        with self.assertRaises(t.TrainError):
            t.fit(t.load_csv(csv))

    def test_bad_csv_rejected(self):
        header = SAMPLE.read_text().splitlines()[0]
        good = SAMPLE.read_text().splitlines()[1].split(",")
        cases = {
            "zero_speed": ",".join([good[0], "0.0"] + good[2:]),
            "nan": ",".join(good[:4] + ["nan"] + good[5:]),
            "text": ",".join(good[:4] + ["abc"] + good[5:]),
        }
        for name, row in cases.items():
            p = self.dir / f"{name}.csv"
            p.write_text(header + "\n" + row + "\n")
            with self.assertRaises(t.TrainError, msg=name):
                t.load_csv(p)
        p = self.dir / "nocol.csv"
        p.write_text("a,b\n1,2\n")
        with self.assertRaises(t.TrainError):
            t.load_csv(p)

    def test_deterministic_bytes(self):
        a, b = self.dir / "a.onnx", self.dir / "b.onnx"
        t.train(SAMPLE, a, now="2026-01-01T00:00:00+00:00")
        t.train(SAMPLE, b, now="2026-01-01T00:00:00+00:00")
        self.assertEqual(a.read_bytes(), b.read_bytes())

    def test_io_contract_matches_stub(self):
        t.train(SAMPLE, self.out)
        m = onnx.load(str(self.out))
        self.assertEqual([i.name for i in m.graph.input], ["inputs"])
        self.assertEqual([o.name for o in m.graph.output], ["predicted_values", "uncertainty", "in_validated_envelope"])
        dims = [d.dim_value for d in m.graph.input[0].type.tensor_type.shape.dim]
        self.assertEqual(dims, [1, 3])

    def test_holdout_detects_wrong_scaling(self):
        # Data that is NOT proportional to V^2 (force linear in V): leave-one-speed-out error must be clearly > 0.
        d = t.load_csv(SAMPLE)
        d["force"] = d["force"] / (d["speed"] ** 2)[:, None] * d["speed"][:, None] * 5.0
        self.assertGreater(t.leave_one_speed_out_rms(d), 0.05)


if __name__ == "__main__":
    unittest.main()
