# Tier: D
"""CFD sanity checks to run BEFORE trusting a vehicle sweep: a sphere drag case with a known answer and a grid-refinement pair.

  generate-sphere      one FluidX3D case: a sphere at the configured Reynolds number (class_sweeps.json -> sanity.sphere)
  generate-refinement  the sweep's middle-speed, zero-angle case at the coarse and the fine resolution (sanity.refinement)
  report               read the result files of both, write cfd_sanity_report.json (status NOT_RUN / PARTIAL / RUN, deviations in percent)

Sphere reference: Schiller-Naumann drag correlation Cd = 24/Re (1 + 0.15 Re^0.687), valid for Re below about 800 and used here as a
rough guide at Re = 1000 [UNVERIFIED: a literature value has to be checked by a human; the standard drag curve is near 0.44-0.47 there].
The report gives the DEVIATION and nothing more: no pass / fail, because no tolerance has a source. A report with status NOT_RUN says so in
every model sidecar and CFD result that uses it.
No FluidX3D code is copied; setup.cpp (same template and TODO(HUMAN-VERIFY) marks as vehicle_sweep.py) only calls its public API.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vehicle_sweep as S  # noqa: E402

G = S.G
STATUS_NOT_RUN = "NOT_RUN"
STATUS_PARTIAL = "PARTIAL"
STATUS_RUN = "RUN"
SPHERE_REFERENCE_LABEL = "Schiller-Naumann correlation [UNVERIFIED]"


def schiller_naumann_cd(reynolds: float) -> float:
    """Cd = 24/Re (1 + 0.15 Re^0.687)."""
    if not (math.isfinite(reynolds) and reynolds > 0.0):
        raise S.SweepError("Reynolds number must be finite and > 0")
    return 24.0 / reynolds * (1.0 + 0.15 * reynolds ** 0.687)


def icosphere(radius: float, subdivisions: int) -> "G.Mesh":
    """Closed icosphere (20 x 4^s triangles), outward normals, vertices on the sphere of the given radius."""
    t = (1.0 + math.sqrt(5.0)) / 2.0
    base = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    verts = [tuple(c / math.sqrt(sum(x * x for x in v)) for c in v) for v in base]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    for _ in range(subdivisions):
        cache, new_faces = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = tuple((verts[a][i] + verts[b][i]) / 2.0 for i in range(3))
                n = math.sqrt(sum(x * x for x in m))
                verts.append(tuple(x / n for x in m))
                cache[key] = len(verts) - 1
            return cache[key]
        for a, b, c in faces:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    m = G.Mesh()
    m.v = [tuple(radius * c for c in v) for v in verts]
    m.t = faces
    G.check_watertight(m)
    return m


def sphere_case(cfg: dict, memory: dict) -> dict:
    sp, air, lbm, dom = cfg["sanity"]["sphere"], cfg["air"], cfg["lbm"], cfg["domain"]
    d_m, d_cells = sp["diameter_m"], sp["diameter_cells"]
    dx = d_m / d_cells
    speed = sp["reynolds"] * air["nu_m2ps"] / d_m
    nx, ny, nz = (int(math.ceil(round(s * d_cells, 9)) + (math.ceil(round(s * d_cells, 9)) % 2)) for s in sp["domain_diameters_xyz"])
    if nx * ny * nz > dom["max_cells"]:
        raise S.SweepError(f"sphere grid {nx}x{ny}x{nz} exceeds the {dom['max_cells']} cell limit")
    required, limit, fits = S.vram_check({"nx": nx, "ny": ny, "nz": nz}, memory)
    if not fits:
        raise S.SweepError(f"sphere grid needs {required} B but only {limit:.0f} B are allowed")
    nu_lb = lbm["velocity"] * d_cells / sp["reynolds"]
    steps = S.total_steps(nx, lbm)
    mesh = icosphere(d_cells / 2.0, sp["icosphere_subdivisions"])
    stl = G.to_ascii_stl(mesh, "sphere_lattice")
    return {"case_id": f"sphere_re{int(round(sp['reynolds']))}", "speed_mps": speed, "alpha_deg": 0.0, "beta_deg": 0.0, "dx_m": dx,
            "grid": {"nx": nx, "ny": ny, "nz": nz}, "memory": memory, "vram_required_bytes": required, "vram_limit_bytes": limit,
            "total_steps": steps, "avg_steps": int(steps * lbm["avg_fraction"]), "lattice_updates": nx * ny * nz * steps, "span_m": d_m,
            "cg_cells": [sp["inlet_diameters"] * d_cells, 0.5 * ny, 0.5 * nz], "bbox_centre_from_cg_cells": [0.0, 0.0, 0.0],
            "ref_area_m2": math.pi * d_m * d_m / 4.0, "ref_length_m": d_m, "reynolds": sp["reynolds"], "nu_lb": nu_lb, "tau": 3.0 * nu_lb + 0.5,
            "rotation_rows": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], "stl_sha256": S.hashlib.sha256(stl.encode()).hexdigest(),
            "solver": S.SolverName, "provenance": S.PROVENANCE, "_stl_text": stl, "expected_cd": schiller_naumann_cd(sp["reynolds"]),
            "expected_cd_source": SPHERE_REFERENCE_LABEL, "icosphere_subdivisions": sp["icosphere_subdivisions"]}


def generate_sphere(out: Path, bytes_per_cell: int, vram_gb: float) -> dict:
    cfg = S.load_class_config()
    memory = {"bytes_per_cell": int(bytes_per_cell), "vram_bytes": int(vram_gb * 1024 ** 3), "headroom_fraction": cfg["domain"]["vram_headroom_fraction"]}
    case = sphere_case(cfg, memory)
    d = out / case["case_id"]
    d.mkdir(parents=True, exist_ok=True)
    (d / "vehicle.stl").write_text(case.pop("_stl_text"), encoding="utf-8", newline="\n")
    (d / "setup.cpp").write_text(S.render_setup(case, cfg), encoding="utf-8", newline="\n")
    (d / "case.json").write_text(json.dumps(case, indent=2) + "\n", encoding="utf-8", newline="\n")
    manifest = {"provenance": S.PROVENANCE, "kind": "sphere", "geometry_sha256": case["stl_sha256"], "propellers": "n/a", "ref_area_m2": case["ref_area_m2"],
                "ref_length_m": case["ref_length_m"], "cases": [case]}
    (out / "sweep_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n")
    out_abs = out.resolve()
    rel = Path(*([".."] * len(out_abs.relative_to(S.ROOT).parts))) if out_abs.is_relative_to(S.ROOT) else Path(".")
    (out / "run_sweep.ps1").write_text(S.RUN_SWEEP_PS1.replace("@@REPO_REL@@", str(rel).replace("/", "\\")), encoding="utf-8", newline="\n")
    return case


def sphere_cd(case: dict, raw_line: str, rho: float) -> float:
    """Drag coefficient from one raw result line: Cd = F_x / (q A) with q = 0.5 rho U^2 and A = pi D^2 / 4."""
    vals = [float(x) for x in raw_line.strip().split(",")]
    if len(vals) != len(S.RAW_HEADER.split(",")) or not all(math.isfinite(x) for x in vals):
        raise S.SweepError("sphere raw result is malformed or non-finite")
    if vals[0] <= 0.0:
        raise S.SweepError(f"sphere drag {vals[0]} N must be > 0: force sign or setup is wrong")
    q = 0.5 * rho * case["speed_mps"] ** 2
    return vals[0] / (q * case["ref_area_m2"])


def generate_refinement(build: Path, out: Path, bytes_per_cell: int, vram_gb: float) -> list:
    cfg = S.load_class_config()
    assembly = json.loads((build / "assembly.json").read_text(encoding="utf-8"))
    ref, cd = cfg["sanity"]["refinement"], cfg["classes"][assembly["vehicle_class"]]
    speed = cd["speeds_mps"][ref["speed_index"]]
    cases = []
    for label, cells in (("coarse", ref["coarse_cells_across_span"]), ("fine", ref["fine_cells_across_span"])):
        sub = out / label
        m = S.generate(build, sub, bytes_per_cell, vram_gb, cells, [speed], [ref["alpha_deg"]], 1)
        cases.append((label, cells, m[0]))
    return cases


def _raw(case_dir: Path) -> str | None:
    p = case_dir / "raw_result.csv"
    if not p.exists():
        return None
    data = p.read_text(encoding="utf-8").strip().splitlines()
    if len(data) != 2 or data[0] != S.RAW_HEADER:
        raise S.SweepError(f"{p}: unexpected layout")
    return data[1]


def build_report(sphere_dir: Path | None, refinement_dir: Path | None) -> dict:
    cfg = S.load_class_config()
    rho = cfg["air"]["rho_kgpm3"]
    sphere = {"status": STATUS_NOT_RUN, "reference": SPHERE_REFERENCE_LABEL}
    if sphere_dir is not None and (sphere_dir / "sweep_manifest.json").exists():
        case = json.loads((sphere_dir / "sweep_manifest.json").read_text(encoding="utf-8"))["cases"][0]
        line = _raw(sphere_dir / case["case_id"])
        sphere.update({"reynolds": case["reynolds"], "expected_cd": case["expected_cd"], "diameter_cells": cfg["sanity"]["sphere"]["diameter_cells"]})
        if line is not None:
            cd = sphere_cd(case, line, rho)
            sphere.update({"status": STATUS_RUN, "measured_cd": cd, "deviation_percent": 100.0 * (cd - case["expected_cd"]) / case["expected_cd"]})
    refinement = {"status": STATUS_NOT_RUN}
    if refinement_dir is not None and (refinement_dir / "coarse" / "sweep_manifest.json").exists() and (refinement_dir / "fine" / "sweep_manifest.json").exists():
        values = {}
        for label in ("coarse", "fine"):
            doc = json.loads((refinement_dir / label / "sweep_manifest.json").read_text(encoding="utf-8"))
            case = doc["cases"][0]
            line = _raw(refinement_dir / label / case["case_id"])
            if line is not None:
                row = S.collect_row(case, line, rho)
                values[label] = {"cells_across_span": round(case["span_m"] / case["dx_m"]), "case_id": case["case_id"], "cx": row["c"][0], "cz": row["c"][2]}
        refinement["cases"] = values
        if len(values) == 2:
            fine, coarse = values["fine"], values["coarse"]
            refinement.update({"status": STATUS_RUN, "drag_coefficient_change_percent": 100.0 * (fine["cx"] - coarse["cx"]) / abs(fine["cx"]),
                               "note": "change of the body-frame x force coefficient from the coarse to the fine grid; not a convergence proof"})
    states = {sphere["status"], refinement["status"]}
    overall = STATUS_RUN if states == {STATUS_RUN} else (STATUS_NOT_RUN if states == {STATUS_NOT_RUN} else STATUS_PARTIAL)
    return {"schema_version": 1, "status": overall, "sphere": sphere, "refinement": refinement,
            "provenance": "preliminary CFD, unvalidated: deviations are reported, no tolerance is claimed",
            "statement": {"NOT_RUN": "The sphere and grid-refinement checks have NOT been run.", "PARTIAL": "Only one of the sphere and grid-refinement checks has been run.",
                          "RUN": "Both checks have been run; read the deviations."}[overall]}


def main(argv: list) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("generate-sphere")
    s.add_argument("--out", type=Path, required=True)
    s.add_argument("--bytes-per-cell", type=int, required=True)
    s.add_argument("--vram-gb", type=float, default=6.0)
    r = sub.add_parser("generate-refinement")
    r.add_argument("--build", type=Path, required=True)
    r.add_argument("--out", type=Path, required=True)
    r.add_argument("--bytes-per-cell", type=int, required=True)
    r.add_argument("--vram-gb", type=float, default=6.0)
    p = sub.add_parser("report")
    p.add_argument("--sphere", type=Path)
    p.add_argument("--refinement", type=Path)
    p.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    try:
        if a.cmd == "generate-sphere":
            c = generate_sphere(a.out, a.bytes_per_cell, a.vram_gb)
            print(f"wrote sphere case {c['case_id']}: grid {c['grid']}, U {c['speed_mps']:.5f} m/s, expected Cd {c['expected_cd']:.4f} ({SPHERE_REFERENCE_LABEL})")
        elif a.cmd == "generate-refinement":
            for label, cells, case in generate_refinement(a.build, a.out, a.bytes_per_cell, a.vram_gb):
                print(f"wrote {label} case {case['case_id']}: {cells} cells across the span, grid {case['grid']}")
        else:
            doc = build_report(a.sphere, a.refinement)
            a.out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8", newline="\n")
            print(f"wrote {a.out}: {doc['status']} - {doc['statement']}")
    except S.SweepError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
