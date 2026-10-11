# Tier: D
"""Per-build aero surrogate from a vehicle CFD CSV, bound to the build it came from.

  train   --build DIR --csv vehicle_cfd.csv --out model.onnx [--sanity cfd_sanity_report.json] [--now ISO]
  verify  --build DIR --model model.onnx [--csv vehicle_cfd.csv]

Model form and ONNX contract are those of train_aero_from_csv.py (imported, not edited): F = V^2 (c0 + c1 a + c2 b + c3 a^2 + c4 b^2) per
body axis, input [airspeed_mps, alpha_rad, beta_rad], outputs predicted_values (body force, N), uncertainty (fit RMS, N),
in_validated_envelope. The core's aero format carries FORCE only: moments are in the CSV and the report, not in the model.

Binding (the hash decision): ONNX metadata and the `<model>.model.json` sidecar carry
  config_hash        = sha256 of the build's vehicle_config.json bytes          (same definition as the core's compile-time hash)
  geometry_hash      = sha256 of the build's CFD geometry STL bytes              (same definition as vehicle_sweep.py / vehicle_stl.py)
  training_data_sha256 = sha256 of the CSV bytes
  trainer_version, binding_hash = sha256("config_hash|geometry_hash|training_data_sha256|trainer_version")
and the sidecar adds model_sha256 (the model file's own hash). The Rust loader recomputes binding_hash and checks all three against what the caller expects.
`kind` is `trained` ONLY if every CSV row says "preliminary CFD, unvalidated" and there are >= 3 speeds for the held-out error; anything else
(the synthetic sample) gives `stub`. Neither is validated; the sidecar carries the sanity-check status (NOT_RUN unless a report is given).
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import onnx

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import train_aero_from_csv as T  # noqa: E402
import vehicle_stl as VS  # noqa: E402

TRAINER_VERSION = "1"
MODEL_TEXT = "SIDECAR"
SIDECAR_SUFFIX = ".model.json"
NOT_RUN_STATEMENT = "The sphere and grid-refinement checks have NOT been run."


class TrainError(Exception):
    pass


def sha256_bytes(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def binding_hash(config_hash: str, geometry_hash: str, data_hash: str, trainer_version: str = TRAINER_VERSION) -> str:
    return sha256_bytes("|".join([config_hash, geometry_hash, data_hash, trainer_version]).encode("utf-8"))


def build_geometry_hash(build: Path, props: str) -> str:
    assembly = json.loads((build / "assembly.json").read_text(encoding="utf-8"))
    try:
        geo = VS.build_geometry(assembly, props)
    except VS.GeometryError as e:
        raise TrainError(f"geometry: {e}") from e
    return sha256_bytes(VS.G.to_ascii_stl(geo["mesh"], assembly["build_name"]).encode("utf-8"))


def csv_columns(path: Path) -> dict:
    with path.open(newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise TrainError(f"{path}: no data rows")
    for col in ("geometry_sha256", "propellers"):
        if col not in rows[0]:
            raise TrainError(f"{path}: missing column {col} (not a vehicle_sweep CSV)")
    hashes, props = {r["geometry_sha256"] for r in rows}, {r["propellers"] for r in rows}
    if len(hashes) != 1 or len(props) != 1:
        raise TrainError(f"{path}: rows come from more than one geometry or propeller setting")
    return {"geometry_sha256": hashes.pop(), "propellers": props.pop()}


def train(build: Path, csv_path: Path, out_path: Path, sanity_path: Path | None = None, now: str | None = None) -> dict:
    cols = csv_columns(csv_path)
    geometry_hash = build_geometry_hash(build, cols["propellers"])
    if cols["geometry_sha256"] != geometry_hash:
        raise TrainError("the CSV was made for a different geometry than this build's current one (the build changed since the CFD was run)")
    try:
        d = T.load_csv(csv_path)
    except T.TrainError as e:
        raise TrainError(str(e)) from e
    try:
        w = T.fit(d)
        rms = T.residual_rms(w, d)
        holdout = T.leave_one_speed_out_rms(d)
    except T.TrainError as e:
        raise TrainError(str(e)) from e
    kind = "trained" if d["provenance"] == {T.TRAINED_PROVENANCE} else "stub"
    if kind == "trained" and holdout is None:
        raise TrainError("a trained model needs 3 or more distinct speeds for the held-out error")
    env = T.envelope(d)
    model = T.build_onnx(w, rms, env)
    config_hash = sha256_bytes((build / "vehicle_config.json").read_bytes())
    data_hash = sha256_bytes(csv_path.read_bytes())
    bind = binding_hash(config_hash, geometry_hash, data_hash)
    assembly = json.loads((build / "assembly.json").read_text(encoding="utf-8"))
    sanity = json.loads(sanity_path.read_text(encoding="utf-8")) if sanity_path else {"status": "NOT_RUN", "statement": NOT_RUN_STATEMENT}
    stamp = now or datetime.now(UTC).isoformat()
    meta = {
        "kind": kind, "provenance": "; ".join(sorted(d["provenance"])) + " | NOT VALIDATED against any measurement",
        "geometry_hash": geometry_hash, "config_hash": config_hash, "training_data_sha256": data_hash, "trainer_version": TRAINER_VERSION, "binding_hash": bind,
        "vehicle_class": assembly["vehicle_class"], "build_name": assembly["build_name"], "propellers": cols["propellers"],
        "cfd_sanity_status": sanity["status"], "training_rows": str(len(d["speed"])), "holdout_rms_n": "none" if holdout is None else repr(holdout),
        "holdout_kind": "leave-one-speed-out RMS |F| error, N", "fit_rms_n": repr(rms), "valid_range": json.dumps(env),
        "seed": "none (deterministic least squares)", "numpy": np.__version__, "onnx": onnx.__version__, "trained_at_utc": stamp,
    }
    for k, v in meta.items():
        model.metadata_props.add(key=k, value=str(v))
    onnx.checker.check_model(model)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(out_path))
    side = {"schema_version": 1, "model_file": out_path.name, "model_sha256": sha256_bytes(out_path.read_bytes()), **{k: meta[k] for k in (
        "kind", "provenance", "geometry_hash", "config_hash", "training_data_sha256", "trainer_version", "binding_hash", "vehicle_class", "build_name",
        "propellers", "cfd_sanity_status", "training_rows", "holdout_rms_n", "fit_rms_n", "trained_at_utc")},
        "cfd_sanity_statement": sanity.get("statement", NOT_RUN_STATEMENT), "valid_range": env,
        "model_form": "F = V^2 (c0 + c1 alpha + c2 beta + c3 alpha^2 + c4 beta^2) per body axis; force only (no moments)",
        "limitations": ["preliminary CFD, unvalidated", "propellers omitted from the CFD geometry unless the sidecar says otherwise", "force only, no moments"]}
    sidecar = out_path.with_name(out_path.name + SIDECAR_SUFFIX)
    sidecar.write_text(json.dumps(side, indent=2) + "\n", encoding="utf-8", newline="\n")
    return {"weights": w, "rms": rms, "holdout": holdout, "kind": kind, "envelope": env, "sidecar": sidecar, "binding_hash": bind}


def verify(build: Path, model_path: Path, csv_path: Path | None = None) -> list:
    """Problems found (empty list = the model, its sidecar, the build and the CSV agree)."""
    problems = []
    sidecar_path = model_path.with_name(model_path.name + SIDECAR_SUFFIX)
    if not model_path.exists() or not sidecar_path.exists():
        return [f"{model_path} or its {SIDECAR_SUFFIX} is missing"]
    side = json.loads(sidecar_path.read_text(encoding="utf-8"))
    if sha256_bytes(model_path.read_bytes()) != side["model_sha256"]:
        problems.append("model file differs from the sidecar's model_sha256")
    meta = {p.key: p.value for p in onnx.load(str(model_path)).metadata_props}
    for k in ("kind", "geometry_hash", "config_hash", "training_data_sha256", "trainer_version", "binding_hash"):
        if meta.get(k) != side.get(k):
            problems.append(f"metadata {k} differs from the sidecar")
    if binding_hash(side["config_hash"], side["geometry_hash"], side["training_data_sha256"], side["trainer_version"]) != side["binding_hash"]:
        problems.append("binding_hash is not the hash of its parts")
    if sha256_bytes((build / "vehicle_config.json").read_bytes()) != side["config_hash"]:
        problems.append("the build's vehicle_config.json changed since training")
    try:
        if build_geometry_hash(build, side["propellers"]) != side["geometry_hash"]:
            problems.append("the build's geometry changed since training")
    except TrainError as e:
        problems.append(str(e))
    if csv_path is not None and sha256_bytes(csv_path.read_bytes()) != side["training_data_sha256"]:
        problems.append("the CSV differs from the training data")
    return problems


def main(argv: list) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("train")
    t.add_argument("--build", type=Path, required=True)
    t.add_argument("--csv", type=Path, required=True)
    t.add_argument("--out", type=Path, required=True)
    t.add_argument("--sanity", type=Path)
    t.add_argument("--now")
    v = sub.add_parser("verify")
    v.add_argument("--build", type=Path, required=True)
    v.add_argument("--model", type=Path, required=True)
    v.add_argument("--csv", type=Path)
    a = ap.parse_args(argv)
    try:
        if a.cmd == "train":
            r = train(a.build, a.csv, a.out, a.sanity, a.now)
            print(f"wrote {a.out}: kind={r['kind']} fit RMS {r['rms']:.4g} N, holdout {r['holdout']}, binding {r['binding_hash']}")
            return 0
        problems = verify(a.build, a.model, a.csv)
    except (TrainError, OSError, KeyError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    for p in problems:
        print(f"MISMATCH: {p}", file=sys.stderr)
    print("OK: model, sidecar, build and data agree" if not problems else f"{len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
