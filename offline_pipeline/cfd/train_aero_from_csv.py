# Tier: D
"""Fit the whole-vehicle aero surrogate from a quad CFD CSV and write an ONNX model.

Run from the repo root, inside .venv:
  python offline_pipeline/cfd/train_aero_from_csv.py --csv PATH [--out PATH]

Input CSV: the `quad_cfd.csv` written by `fluidx3d_quad_sweep.py collect` (columns in OUT_HEADER). Body-frame force in N,
X forward, Y right, Z down. Inputs of the ONNX model (same contract as `aero_stub.onnx` and the Rust
`aero_surrogate::aero_inputs`): `inputs` [1,3] = [airspeed_mps, alpha_rad, beta_rad].  Output
`predicted_values` [1,3] = body-frame force, N.

Model: F = V^2 * (c0 + c1 a + c2 b + c3 a^2 + c4 b^2)  per axis, a = alpha, b = beta (rad). F ~ V^2 is the
dynamic-pressure scaling at fixed Reynolds-insensitive shape (ASSUMED; the CFD speeds are 5 to 20 m/s). There is NO alpha*beta term
because the sweep is pruned to the two axes (alpha at beta = 0, beta at alpha = 0): that column would be all zero.
V = 0 rows are not in the CSV; F(V=0) = 0 follows from the V^2 factor. `uncertainty` = RMS fit residual (N).
`in_validated_envelope` = 1 only inside the CFD sweep box: 0 <= V <= max V, |alpha| <= max |alpha|, |beta| <= max |beta|
(low-speed Reynolds effects are not simulated: ASSUMED).

Provenance: `kind` = `trained` ONLY if every CSV row says "preliminary CFD, unvalidated" (what the sweep tool writes). Any
other provenance (for example the synthetic sample) makes `kind` = `stub`. Neither is VALIDATED; the metadata says so.
Held-out error: leave-one-speed-out RMS in N (needs 3 or more speeds); with one geometry that is the strongest honest split here.
Deterministic: no random numbers (seed recorded as "none").
"""
import argparse
import csv
import hashlib
import json
import math
import sys
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
GEOMETRY = ROOT / "offline_pipeline" / "geometry" / "current_geometry.json"
CONFIG = ROOT / "offline_pipeline" / "geometry" / "quad_ref_build_config.json"
DEFAULT_OUT = ROOT / "offline_pipeline" / "trained_models" / "aero_quad_cfd.onnx"
TRAINED_PROVENANCE = "preliminary CFD, unvalidated"
N_FEATURES = 5
COLUMNS = ("speed_mps", "alpha_deg", "beta_deg", "fx_body_n", "fy_body_n", "fz_body_n")


class TrainError(Exception):
    pass


def features(alpha_rad, beta_rad) -> np.ndarray:
    """Design columns without the V^2 factor: [1, a, b, a^2, b^2]."""
    a, b = np.asarray(alpha_rad, dtype=np.float64), np.asarray(beta_rad, dtype=np.float64)
    return np.stack([np.ones_like(a), a, b, a * a, b * b], axis=-1)


def load_csv(path: Path) -> dict:
    with path.open(newline="") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise TrainError(f"{path}: no data rows")
    missing = [c for c in COLUMNS + ("provenance",) if c not in rows[0]]
    if missing:
        raise TrainError(f"{path}: missing columns {missing}")
    try:
        data = np.array([[float(r[c]) for c in COLUMNS] for r in rows], dtype=np.float64)
    except ValueError as exc:
        raise TrainError(f"{path}: non-numeric value: {exc}") from exc
    if not np.all(np.isfinite(data)):
        raise TrainError(f"{path}: non-finite value")
    if np.any(data[:, 0] <= 0.0):
        raise TrainError(f"{path}: every speed must be > 0 (V = 0 is exact zero, not a data row)")
    return {"speed": data[:, 0], "alpha_rad": np.radians(data[:, 1]), "beta_rad": np.radians(data[:, 2]),
            "force": data[:, 3:6], "provenance": {r["provenance"] for r in rows}}


def design(speed, alpha_rad, beta_rad) -> np.ndarray:
    return (np.asarray(speed, dtype=np.float64) ** 2)[:, None] * features(alpha_rad, beta_rad)


def fit(d: dict) -> np.ndarray:
    """Least squares W [5,3] with design(V, a, b) @ W = F. Raises if the sweep cannot determine all 5 terms."""
    x = design(d["speed"], d["alpha_rad"], d["beta_rad"])
    if np.linalg.matrix_rank(x) < N_FEATURES:
        raise TrainError("the sweep cannot determine all terms (rank deficient): need alpha and beta both varied")
    w, *_ = np.linalg.lstsq(x, d["force"], rcond=None)
    return w


def residual_rms(w: np.ndarray, d: dict) -> float:
    r = design(d["speed"], d["alpha_rad"], d["beta_rad"]) @ w - d["force"]
    return float(math.sqrt(np.mean(np.sum(r * r, axis=1))))


def subset(d: dict, mask) -> dict:
    return {k: (v[mask] if isinstance(v, np.ndarray) else v) for k, v in d.items()}


def leave_one_speed_out_rms(d: dict):
    """Mean over speeds of the RMS |force error| (N) on a speed left out of the fit; None if < 3 speeds."""
    speeds = sorted(set(d["speed"].tolist()))
    if len(speeds) < 3:
        return None
    errs = []
    for s in speeds:
        test = d["speed"] == s
        errs.append(residual_rms(fit(subset(d, ~test)), subset(d, test)))
    return float(np.mean(errs))


def envelope(d: dict) -> dict:
    return {"speed_mps": [0.0, float(d["speed"].max())], "alpha_rad": float(np.abs(d["alpha_rad"]).max()),
            "beta_rad": float(np.abs(d["beta_rad"]).max())}


def _const(name, arr):
    arr = np.asarray(arr, dtype=np.float32)
    return helper.make_node("Constant", [], [name], value=helper.make_tensor(name + "_v", TensorProto.FLOAT, list(arr.shape),
                                                                         arr.flatten().tolist()))


def build_onnx(w: np.ndarray, rms: float, env: dict):
    """Graph: inputs[1,3] -> V,a,b -> V^2 * [1,a,b,a^2,b^2] @ W ; envelope from range checks."""
    sel = lambda i: _const(f"sel{i}", np.eye(3)[:, i:i + 1])  # noqa: E731 (3x1 selector, MatMul picks column i)
    nodes = [sel(0), sel(1), sel(2),
             helper.make_node("MatMul", ["inputs", "sel0"], ["v"]),
             helper.make_node("MatMul", ["inputs", "sel1"], ["a"]),
             helper.make_node("MatMul", ["inputs", "sel2"], ["b"]),
             helper.make_node("Mul", ["v", "v"], ["v2"]),
             helper.make_node("Mul", ["a", "a"], ["a2"]),
             helper.make_node("Mul", ["b", "b"], ["b2"]),
             _const("one", [[1.0]]),
             helper.make_node("Concat", ["one", "a", "b", "a2", "b2"], ["phi0"], axis=1),
             helper.make_node("Mul", ["phi0", "v2"], ["phi"]),  # [1,5] * [1,1]
             _const("W", w),
             helper.make_node("MatMul", ["phi", "W"], ["predicted_values"]),
             _const("unc", [[rms]]),
             _const("lo", [[0.0, -env["alpha_rad"], -env["beta_rad"]]]),
             _const("hi", [[env["speed_mps"][1], env["alpha_rad"], env["beta_rad"]]]),
             helper.make_node("GreaterOrEqual", ["inputs", "lo"], ["ge"]),
             helper.make_node("LessOrEqual", ["inputs", "hi"], ["le"]),
             helper.make_node("And", ["ge", "le"], ["inside"]),
             helper.make_node("Cast", ["inside"], ["inside_f"], to=TensorProto.FLOAT),
             helper.make_node("ReduceMin", ["inside_f"], ["in_validated_envelope"], axes=[1], keepdims=1),
             helper.make_node("Identity", ["unc"], ["uncertainty"])]
    graph = helper.make_graph(
        nodes, "AeroSurrogateQuadCfd",
        [helper.make_tensor_value_info("inputs", TensorProto.FLOAT, [1, 3])],
        [helper.make_tensor_value_info("predicted_values", TensorProto.FLOAT, [1, 3]),
         helper.make_tensor_value_info("uncertainty", TensorProto.FLOAT, [1, 1]),
         helper.make_tensor_value_info("in_validated_envelope", TensorProto.FLOAT, [1, 1])])
    model = helper.make_model(graph, producer_name="uav-simulator-cfd", opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 8
    return model


def sha256_file(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def train(csv_path: Path, out_path: Path, config_path: Path = CONFIG, now: str | None = None) -> dict:
    d = load_csv(csv_path)
    w = fit(d)
    rms = residual_rms(w, d)
    holdout = leave_one_speed_out_rms(d)
    kind = "trained" if d["provenance"] == {TRAINED_PROVENANCE} else "stub"
    if kind == "trained" and holdout is None:
        raise TrainError("a trained model needs 3 or more distinct speeds for the held-out error")
    env = envelope(d)
    model = build_onnx(w, rms, env)
    meta = {
        "kind": kind,
        "provenance": ("; ".join(sorted(d["provenance"])) + (" | NOT VALIDATED against any measurement")),
        "geometry_hash": sha256_file(GEOMETRY), "config_hash": sha256_file(config_path),
        "config_file": str(config_path.relative_to(ROOT)),
        "training_data_sha256": sha256_file(csv_path), "training_rows": str(len(d["speed"])),
        "holdout_rms_n": "none" if holdout is None else repr(holdout), "holdout_kind": "leave-one-speed-out RMS |F| error, N",
        "fit_rms_n": repr(rms), "valid_range": json.dumps(env), "seed": "none (deterministic least squares)",
        "numpy": np.__version__, "onnx": onnx.__version__,
        "trained_at_utc": now or datetime.now(UTC).isoformat(),
    }
    for k, v in meta.items():
        model.metadata_props.add(key=k, value=str(v))
    onnx.checker.check_model(model)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(out_path))
    return {"weights": w, "rms": rms, "holdout": holdout, "kind": kind, "envelope": env}


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--config", type=Path, default=CONFIG)
    a = ap.parse_args(argv)
    try:
        r = train(a.csv, a.out, a.config)
    except TrainError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {a.out}: kind={r['kind']} fit RMS {r['rms']:.4g} N, holdout {r['holdout']}, envelope {r['envelope']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
