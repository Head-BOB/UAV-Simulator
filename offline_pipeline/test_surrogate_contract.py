# Tier: D
"""Contract test between the Rust input builders and the committed stub ONNX models (review E).

Runs aero_stub.onnx and fea_stub.onnx with onnx.reference.ReferenceEvaluator (pure Python, no ONNX
Runtime, no new dependency) on hand-calculated inputs laid out exactly as physics_core sends them:

* aero (`aero_surrogate::aero_inputs`): [airspeed_mps, alpha_rad, beta_rad], width 3.
* FEA (`fea_surrogate::fea_inputs`): [arm_load_n x4, load_dir_body x3, motor_temp_K], width 8.

The stubs carry weights only where noted in train_aero.py / train_fea.py, so this test can prove
the SLOT of airspeed and of the four arm loads and the widths, but NOT the order of alpha vs beta
or of direction vs temperature (their weights are 0). Those orders are covered on the Rust side
(`aero_surrogate::tests`, `fea_surrogate::tests::test_fea_to_model_inputs_order`); a trained model
must be contract-tested again with non-zero weights there. Widths and the Kelvin offset are read
from the Rust source text so a one-sided change fails here.
"""
import re
import unittest
from pathlib import Path

import numpy as np
import onnx
from onnx.reference import ReferenceEvaluator

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "offline_pipeline" / "trained_models"
RUST_AERO = ROOT / "physics_core" / "src" / "surrogate" / "aero_surrogate.rs"
RUST_FEA = ROOT / "physics_core" / "src" / "surrogate" / "fea_surrogate.rs"


def run(model_name: str, values: list[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Runs a stub on one input row; returns (predicted_values, uncertainty, envelope)."""
    model = onnx.load(str(MODELS / model_name))
    ref = ReferenceEvaluator(model)
    x = np.array([values], dtype=np.float32)
    pred, unc, env = ref.run(None, {"inputs": x})
    return pred, unc, env


def rust_const(path: Path, name: str) -> str:
    """Value text of `pub const NAME: type = VALUE;` or `const NAME: type = VALUE;` in a Rust file."""
    text = path.read_text(encoding="utf-8")
    m = re.search(rf"\bconst {name}: [a-z0-9]+ = ([0-9._]+);", text)
    if not m:
        raise AssertionError(f"{name} not found in {path}")
    return m.group(1).replace("_", "")


def input_width(model_name: str) -> list[int]:
    model = onnx.load(str(MODELS / model_name))
    dims = model.graph.input[0].type.tensor_type.shape.dim
    return [d.dim_value for d in dims]


class AeroContract(unittest.TestCase):
    def test_width_matches_rust(self) -> None:
        self.assertEqual(input_width("aero_stub.onnx"), [1, int(rust_const(RUST_AERO, "AERO_INPUT_WIDTH"))])
        self.assertEqual(input_width("aero_stub.onnx"), [1, 3])

    # Hand calc, stub W rows: airspeed -> [-0.5, 0, -1.2]; alpha and beta rows are 0.
    # [10, 0.3, 0.2] -> [-5.0, 0.0, -12.0]. Airspeed in slot 0 is the only way to get -5 / -12.
    def test_airspeed_is_slot_0(self) -> None:
        pred, _, _ = run("aero_stub.onnx", [10.0, 0.3, 0.2])
        np.testing.assert_allclose(pred, [[-5.0, 0.0, -12.0]], atol=1e-5)

    # 3-4-5 triangle as Rust sends it for v_body = (3, 0, 4): V = 5, alpha = atan2(4, 3), beta = 0.
    # -0.5 * 5 = -2.5; -1.2 * 5 = -6.0. A model fed [alpha, V, beta] would return 0 here.
    def test_rust_style_row_3_4_5(self) -> None:
        row = [5.0, 0.9272952180016122, 0.0]
        pred, _, _ = run("aero_stub.onnx", row)
        np.testing.assert_allclose(pred, [[-2.5, 0.0, -6.0]], atol=1e-5)

    def test_zero_row_gives_zero_force(self) -> None:
        pred, _, _ = run("aero_stub.onnx", [0.0, 0.0, 0.0])
        np.testing.assert_allclose(pred, [[0.0, 0.0, 0.0]], atol=0.0)

    def test_stub_metadata_and_raw_envelope(self) -> None:
        meta = {p.key: p.value for p in onnx.load(str(MODELS / "aero_stub.onnx")).metadata_props}
        self.assertEqual(meta["kind"], "stub")
        _, _, env = run("aero_stub.onnx", [1.0, 0.0, 0.0])
        # The file says 1.0; Rust forces 0 for a stub at query time (review A).
        self.assertEqual(float(env[0][0]), 1.0)


class FeaContract(unittest.TestCase):
    def test_width_matches_rust(self) -> None:
        self.assertEqual(input_width("fea_stub.onnx"), [1, int(rust_const(RUST_FEA, "FEA_INPUT_WIDTH"))])
        self.assertEqual(input_width("fea_stub.onnx"), [1, 8])

    def test_rust_temperature_is_kelvin(self) -> None:
        # Rust adds this to deg C before the model sees it: 20 C -> 293.15 K.
        self.assertEqual(float(rust_const(RUST_FEA, "KELVIN_AT_0_C")), 273.15)

    # Hand calc, stub: margin = 15 - 0.005 * (l0 + l1 + l2 + l3); direction and temperature
    # weights are 0. Rust row for thrusts 2+3+4+5 N, direction (0, 0, -1), hottest motor
    # 90.5 C = 363.65 K: 15 - 0.005 * 14 = 14.93.
    def test_rust_style_row(self) -> None:
        pred, _, _ = run("fea_stub.onnx", [2.0, 3.0, 4.0, 5.0, 0.0, 0.0, -1.0, 363.65])
        np.testing.assert_allclose(pred, [[14.93]], atol=1e-5)

    # Each of the first four slots is an arm load worth -0.005 per N: +1 N in slot i gives 14.925
    # for the 4 x 2.5 N = 10 N base (15 - 0.05 = 14.95, minus 0.005).
    def test_each_arm_load_slot(self) -> None:
        base = [2.5, 2.5, 2.5, 2.5, 0.0, 0.0, -1.0, 293.15]
        pred0, _, _ = run("fea_stub.onnx", base)
        np.testing.assert_allclose(pred0, [[14.95]], atol=1e-5)
        for i in range(4):
            row = list(base)
            row[i] += 1.0
            pred, _, _ = run("fea_stub.onnx", row)
            np.testing.assert_allclose(pred, [[14.945]], atol=1e-5, err_msg=f"slot {i}")

    # Stub limitation, stated on purpose: direction and temperature slots have weight 0, so
    # the stub cannot tell K from deg C or direction from temperature.
    def test_stub_ignores_direction_and_temperature(self) -> None:
        a, _, _ = run("fea_stub.onnx", [1.0, 1.0, 1.0, 1.0, 0.0, 0.0, -1.0, 293.15])
        b, _, _ = run("fea_stub.onnx", [1.0, 1.0, 1.0, 1.0, 0.5, -0.5, 0.7, 20.0])
        np.testing.assert_allclose(a, b, atol=0.0)

    def test_stub_metadata_and_raw_envelope(self) -> None:
        meta = {p.key: p.value for p in onnx.load(str(MODELS / "fea_stub.onnx")).metadata_props}
        self.assertEqual(meta["kind"], "stub")
        _, _, env = run("fea_stub.onnx", [1.0] * 8)
        self.assertEqual(float(env[0][0]), 1.0)


if __name__ == "__main__":
    unittest.main()
