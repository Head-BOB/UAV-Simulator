# Tier: D
"""Manual end-to-end check that a `kind: trained` per-build CFD model loads in BOTH helper builds and is labelled correctly.

  ORT_DYLIB_PATH=<libonnxruntime.so.1.27.x | onnxruntime.dll> python tools/check_cfd_trained_path.py \
      --run-campaign-strict <run_campaign built WITHOUT allow_stub_models> --run-campaign-stub <run_campaign built WITH it>

THE MODEL USED HERE IS A TEST DOUBLE, NOT CFD. It is trained from the committed SYNTHETIC sample CSV whose provenance column is rewritten to
"preliminary CFD, unvalidated" in a temporary folder, only so that the trainer writes `kind: trained` and the code path of a real model is
exercised. It is never written into the repository, never staged, and every file name says TEST_DOUBLE. Its numbers mean nothing.

Checks (PASS / FAIL each, exit 1 on any FAIL):
  1. the trainer writes kind=trained for the relabelled rows
  2. strict helper: exit 0, campaign_result.json cfd_surrogate = "CFD surrogate: PRELIMINARY, UNVALIDATED", level warn, kind trained,
     a numeric held-out error, sanity NOT_RUN, and the drag model label says cfd_surrogate:trained:<sha of the model file>
  3. stub-enabled helper: the same
  4. the report built from that campaign says the same and lists no flow images
  5. a model whose provenance was NOT relabelled (the synthetic fixture) is refused by the strict helper with exit code 3
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SAMPLE = ROOT / "offline_pipeline" / "cfd" / "sample"
BUILD = ROOT / "tools" / "parts" / "builds" / "reference_quad"
TRAINED_PROVENANCE = "preliminary CFD, unvalidated"
EXPECTED_STATUS = "CFD surrogate: PRELIMINARY, UNVALIDATED"


def run(cmd: list[str], env: dict | None = None) -> subprocess.CompletedProcess:
    return subprocess.run([str(c) for c in cmd], capture_output=True, text=True, env=env, timeout=1800)


def relabelled_csv(src: Path, dst: Path) -> None:
    with src.open(newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    header = rows[0]
    col = header.index("provenance")
    solver = header.index("solver")
    for r in rows[1:]:
        r[col] = TRAINED_PROVENANCE
        r[solver] = "TEST DOUBLE (relabelled synthetic, not CFD)"
    with dst.open("w", newline="", encoding="utf-8") as f:
        csv.writer(f, lineterminator="\n").writerows(rows)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run-campaign-strict", type=Path, required=True)
    ap.add_argument("--run-campaign-stub", type=Path, required=True)
    ap.add_argument("--python", default=sys.executable, help="interpreter with numpy and onnx (the project venv)")
    a = ap.parse_args(argv)
    if not os.environ.get("ORT_DYLIB_PATH"):
        print("FAIL ORT_DYLIB_PATH is not set (an ONNX Runtime 1.27.x library is needed)")
        return 1
    failed = False

    def check(name: str, ok: bool, detail: str = "") -> None:
        nonlocal failed
        print(("PASS " if ok else "FAIL ") + name + ("" if ok or not detail else f"\n     {detail[-600:]}"))
        failed |= not ok

    with tempfile.TemporaryDirectory(prefix="TEST_DOUBLE_cfd_") as t:
        tmp = Path(t)
        csv_path = tmp / "TEST_DOUBLE_not_cfd.csv"
        relabelled_csv(SAMPLE / "synthetic_quad_standard_cfd.csv", csv_path)
        cfd = tmp / "cfd"
        cfd.mkdir()
        model = cfd / "aero_TEST_DOUBLE.onnx"
        r = run([a.python, ROOT / "offline_pipeline/cfd/train_vehicle_aero.py", "train", "--build", BUILD, "--csv", csv_path, "--out", model,
                 "--now", "2026-10-11T00:00:00+00:00"])
        side_path = Path(str(model) + ".model.json")
        side = json.loads(side_path.read_text(encoding="utf-8")) if side_path.is_file() else {}
        check("the trainer writes kind=trained for the relabelled rows", r.returncode == 0 and side.get("kind") == "trained", r.stdout + r.stderr)
        if side.get("kind") != "trained":
            return 1
        (cfd / "geometry.json").write_text((SAMPLE / "geometry.json").read_text(encoding="utf-8"), encoding="utf-8")
        model_sha = hashlib.sha256(model.read_bytes()).hexdigest()
        base = [BUILD, ROOT / "campaign/conditions_smoke.json", ROOT / "campaign/smoke_definitions.json"]
        for label, exe in (("strict helper (no allow_stub_models)", a.run_campaign_strict), ("stub-enabled helper", a.run_campaign_stub)):
            out = tmp / ("out_" + label.split()[0])
            r = run([exe, *base, out, "--tests", "climb", "--aero-dir", cfd])
            top = {}
            if (out / "campaign_result.json").is_file():
                top = json.loads((out / "campaign_result.json").read_text(encoding="utf-8"))
            n = top.get("cfd_surrogate") or {}
            check(f"{label}: exit 0", r.returncode == 0, r.stdout + r.stderr)
            check(f"{label}: status is '{EXPECTED_STATUS}' (warn)", n.get("status") == EXPECTED_STATUS and n.get("level") == "warn", json.dumps(n))
            check(f"{label}: kind trained, sanity NOT_RUN, numeric held-out error",
                  n.get("kind") == "trained" and n.get("cfd_sanity_status") == "NOT_RUN" and _is_number(n.get("holdout_rms_n")), json.dumps(n))
            check(f"{label}: drag model label carries the model file hash", top.get("drag_model") == f"cfd_surrogate:trained:{model_sha}", str(top.get("drag_model")))
            if label.startswith("strict"):
                strict_out = out
        rep = tmp / "report"
        r = run([a.python, ROOT / "tools/report/make_report.py", "--build", BUILD, "--campaign", strict_out, "--cfd-dir", cfd, "--out", rep])
        doc = json.loads((rep / "report.json").read_text(encoding="utf-8")) if (rep / "report.json").is_file() else {"cfd": {}}
        c = doc["cfd"]
        check("report: status, level, held-out error, sanity NOT_RUN, no flow images",
              c.get("status") == EXPECTED_STATUS and c.get("level") == "warn" and _is_number(c.get("holdout_rms_n")) and c.get("sanity", {}).get("status") == "NOT_RUN"
              and c.get("images_statement") == "no flow images", r.stdout + r.stderr + json.dumps({k: v for k, v in c.items() if k != "images"})[:300])
        # the synthetic fixture must still be refused by the strict build, with its reason on disk
        syn = tmp / "syn"
        syn.mkdir()
        for n_ in ("synthetic_quad_standard_aero.onnx", "synthetic_quad_standard_aero.onnx.model.json", "geometry.json"):
            (syn / n_).write_bytes((SAMPLE / n_).read_bytes())
        out = tmp / "out_syn"
        r = run([a.run_campaign_strict, *base, out, "--tests", "climb", "--aero-dir", syn])
        reason = json.loads((out / "aero_model_status.json").read_text(encoding="utf-8")) if (out / "aero_model_status.json").is_file() else {}
        check("strict helper refuses the synthetic fixture: exit 3 and a reason on disk", r.returncode == 3 and reason.get("status") == "load_failed" and bool(reason.get("reason")),
              r.stdout + r.stderr)
    print("FAILED" if failed else "ALL PASS (the model was a TEST DOUBLE; nothing was written to the repository)")
    return 1 if failed else 0


def _is_number(v) -> bool:
    try:
        float(v)
        return True
    except (TypeError, ValueError):
        return False


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
