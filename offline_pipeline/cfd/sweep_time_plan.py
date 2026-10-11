# Tier: D
"""Total time of each sweep profile from ONE measured "seconds per case", and the exact commands, in order, for the GPU laptop.

  python offline_pipeline/cfd/sweep_time_plan.py --seconds-per-case 25 --measured-profile demo [--measured-class quad_standard]
                                                 [--profile demo] [--build tools/parts/builds/reference_quad] [--bytes-per-cell 55]

Method (all of it follows from class_sweeps.json; nothing here is measured except the one number you give):
  lattice updates of one case = nx * ny * nz * steps;  nx, ny, nz = the domain spans (in vehicle spans) x cells across the span, each rounded up
  to an even number (the vehicle's own span cancels out: dx = span / cells);  steps = ceil(flow_throughs * nx / lbm velocity).
  seconds per update = the measured seconds per case / the lattice updates of the case you measured.
  Every OTHER case's time is that rate times its lattice updates. This assumes the GPU's cells-per-second rate does not depend on the grid
  size: ASSUMED, labelled as such in the output (a small grid may run at a lower rate, so treat the scaled numbers as rough).
Nothing is run by this tool and no FluidX3D speed is assumed: with no measurement there is no time.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import fluidx3d_quad_sweep as Q  # noqa: E402
import sphere_validation as SV  # noqa: E402
import vehicle_sweep as S  # noqa: E402

DEFAULT_PROFILE = "default"
HIGH_BYTES_PER_CELL = 100   # ASSUMED upper bound, used ONLY for the VRAM refusal check of the very first case (see commands)


class PlanError(ValueError):
    pass


def even_up(x: float) -> int:
    n = math.ceil(round(x, 9))
    return n + (n % 2)


def grid_cells(spans_xyz, cells_across: int) -> tuple[int, int, int]:
    """(nx, ny, nz) of a vehicle case: the same rounding as vehicle_sweep.grid_for with dx = span / cells_across."""
    return tuple(even_up(s * cells_across) for s in spans_xyz)  # type: ignore[return-value]


def case_updates(grid: tuple[int, int, int], lbm: dict) -> int:
    """Lattice updates of one case: cells x steps (steps from the x length: flow_throughs * nx / velocity, rounded up)."""
    return grid[0] * grid[1] * grid[2] * S.total_steps(grid[0], lbm)


def profile_cfg(profile: str) -> dict:
    return S.load_class_config() if profile == DEFAULT_PROFILE else S.load_class_config(profile=profile)


def class_row(cfg: dict, cls: str) -> dict:
    c = cfg["classes"][cls]
    cases = Q.sweep_cases(c["speeds_mps"], c["angles_deg"])
    grid = grid_cells(cfg["domain"]["spans_xyz"], c["cells_across_span"])
    return {"class": cls, "cases": len(cases), "speeds": len(c["speeds_mps"]), "angles": len(c["angles_deg"]), "cells_across_span": c["cells_across_span"],
            "grid": grid, "cells": grid[0] * grid[1] * grid[2], "steps": S.total_steps(grid[0], cfg["lbm"]), "updates_per_case": case_updates(grid, cfg["lbm"])}


def sanity_rows(cfg: dict) -> dict:
    """Lattice updates of the sphere case and of the coarse / fine refinement cases (both of the refinement pair run for the class's mid speed)."""
    sp, lbm = cfg["sanity"]["sphere"], cfg["lbm"]
    sphere_grid = tuple(even_up(s * sp["diameter_cells"]) for s in sp["domain_diameters_xyz"])
    ref = cfg["sanity"]["refinement"]
    pair = {label: case_updates(grid_cells(cfg["domain"]["spans_xyz"], ref[f"{label}_cells_across_span"]), lbm) for label in ("coarse", "fine")}
    return {"sphere": case_updates(sphere_grid, lbm), "refine_coarse": pair["coarse"], "refine_fine": pair["fine"]}


def fmt_time(seconds: float) -> str:
    if seconds < 90:
        return f"{seconds:.0f} s"
    if seconds < 5400:
        return f"{seconds / 60:.1f} min"
    return f"{seconds / 3600:.1f} h"


def build_plan(seconds_per_case: float, measured_profile: str, measured_class: str, profiles: list[str]) -> dict:
    if not (math.isfinite(seconds_per_case) and seconds_per_case > 0.0):
        raise PlanError("--seconds-per-case must be a finite number > 0 (measure one case first)")
    cfgs = {p: profile_cfg(p) for p in profiles}
    if measured_profile not in cfgs:
        raise PlanError(f"the measured profile {measured_profile!r} is not among the profiles")
    if measured_class not in cfgs[measured_profile]["classes"]:
        raise PlanError(f"unknown vehicle class {measured_class!r}")
    base = class_row(cfgs[measured_profile], measured_class)["updates_per_case"]
    rate = seconds_per_case / base      # seconds per lattice update, from the one measurement
    rows = []
    for p in profiles:
        for cls in sorted(cfgs[p]["classes"]):
            r = class_row(cfgs[p], cls)
            measured = p == measured_profile and cls == measured_class
            r.update({"profile": p, "seconds_per_case": r["updates_per_case"] * rate, "basis": "MEASURED" if measured else "ASSUMED (scaled by lattice updates)"})
            r["total_s"] = r["cases"] * r["seconds_per_case"]
            rows.append(r)
    san = {}
    for p in profiles:
        s = sanity_rows(cfgs[p])
        san[p] = {"sphere_s": s["sphere"] * rate, "refine_s": (s["refine_coarse"] + s["refine_fine"]) * rate, **s}
    return {"seconds_per_case_measured": seconds_per_case, "measured_profile": measured_profile, "measured_class": measured_class, "rate_s_per_update": rate,
            "rows": rows, "sanity": san}


def unknown_note(rows: list[dict]) -> str:
    """Why the case count of each profile is (or is not) enough: 5 unknowns per body axis, equations = cases (one force component per case and axis)."""
    out = ["Parameters of the fit: 5 per body axis (c0 + c1 alpha + c2 beta + c3 alpha^2 + c4 beta^2, times V^2). One speed needs 5 attitudes",
           "(alpha = -a, 0, +a at beta = 0 and beta = -a, +a at alpha = 0) to determine them; the held-out error needs >= 3 speeds."]
    seen = set()
    for r in rows:
        key = (r["profile"], r["speeds"], r["cases"])
        if key in seen:
            continue
        seen.add(key)
        verdict = "enough" if r["speeds"] >= 3 and r["cases"] // r["speeds"] >= 5 else "NOT enough"
        out.append(f"  profile {r['profile']}: {r['speeds']} speeds x {r['cases'] // r['speeds']} attitudes = {r['cases']} equations per axis for 5 unknowns: {verdict}")
    return "\n".join(out)


def render(plan: dict) -> str:
    out = ["SWEEP TIME PLAN (nothing here was run; one measured number, the rest scaled)",
           f"measured: {plan['seconds_per_case_measured']:.4g} s for one case of profile {plan['measured_profile']!r}, class {plan['measured_class']!r}",
           f"rate: {plan['rate_s_per_update']:.4g} s per lattice update (ASSUMED the same for every grid size)", ""]
    out.append(f"{'profile':<9}{'class':<15}{'cases':>6}  {'grid':<16}{'steps':>7}{'updates/case':>14}{'time/case':>11}{'total':>9}  basis")
    for r in plan["rows"]:
        g = "x".join(str(n) for n in r["grid"])
        out.append(f"{r['profile']:<9}{r['class']:<15}{r['cases']:>6}  {g:<16}{r['steps']:>7}{r['updates_per_case']:>14.3e}{fmt_time(r['seconds_per_case']):>11}"
                   f"{fmt_time(r['total_s']):>9}  {r['basis']}")
    out.append("")
    out.append("Checks before the sweep (ASSUMED time, scaled):")
    for p, s in plan["sanity"].items():
        out.append(f"  {p:<8} sphere Re=1000: {fmt_time(s['sphere_s'])}   grid-refinement pair (coarse + fine): {fmt_time(s['refine_s'])}")
    out.append("")
    out.append(unknown_note(plan["rows"]))
    return "\n".join(out)


def commands(build: str, profile: str, bytes_per_cell: int | None, cfg: dict, vehicle_class: str) -> list[tuple[str, list[str]]]:
    """Ordered (title, lines). `bytes_per_cell` None prints the placeholder <BYTES_PER_CELL> (you measure it in step 1)."""
    b = str(bytes_per_cell) if bytes_per_cell else "<BYTES_PER_CELL>"
    flag = "" if profile == DEFAULT_PROFILE else f" --profile {profile}"
    gen = "offline_pipeline/cfd/generated"
    c = cfg["classes"][vehicle_class]
    mid = Q.case_id(c["speeds_mps"][min(1, len(c["speeds_mps"]) - 1)], 0.0, 0.0)
    ps = "powershell -File offline_pipeline\\cfd\\generated\\{}\\run_sweep.ps1 -FluidX3D C:\\tools\\FluidX3D"
    return [
        ("1. bytes_per_cell: measure it once (the tools have no default on purpose)",
         ["In C:\\tools\\FluidX3D\\src\\defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, FORCE_FIELD, SUBGRID active; GRAPHICS commented out.",
          f"python offline_pipeline/cfd/vehicle_sweep.py generate --build {build} --out {gen}/first --bytes-per-cell {HIGH_BYTES_PER_CELL} --max-cases 1{flag}",
          "   (100 is an ASSUMED upper bound used ONLY by the VRAM refusal check of this first case; a lower real value makes the check looser.)",
          ps.format("first"),
          "Read the memory (MB) and the cell count FluidX3D prints at start-up.  BYTES_PER_CELL = memory in bytes / cells (round UP)."]),
        ("2. one timed case (this is the number the time plan needs)",
         [f"python offline_pipeline/cfd/vehicle_sweep.py generate --build {build} --out {gen}/timed --bytes-per-cell {b} --max-cases 1{flag}",
          ps.format("timed"),
          "Note the wall-clock seconds of the case, then:",
          f"python offline_pipeline/cfd/sweep_time_plan.py --seconds-per-case <SECONDS> --measured-profile {profile} --profile {profile} --build {build}"]),
        ("3. sphere check, Re = 1000 (known answer, Cd about 0.438 [UNVERIFIED])",
         [f"python offline_pipeline/cfd/sphere_validation.py generate-sphere --out {gen}/sphere --bytes-per-cell {b}", ps.format("sphere")]),
        ("4. grid-refinement pair (same case at two resolutions)",
         [f"python offline_pipeline/cfd/sphere_validation.py generate-refinement --build {build} --out {gen}/refine --bytes-per-cell {b}{flag}",
          ps.format("refine\\coarse"), ps.format("refine\\fine")]),
        ("5. the sweep (the number of cases is in the table above)",
         [f"python offline_pipeline/cfd/vehicle_sweep.py generate --build {build} --out {gen}/sweep --bytes-per-cell {b}{flag}", ps.format("sweep"),
          f"python offline_pipeline/cfd/vehicle_sweep.py collect --sweep {gen}/sweep",
          f"python offline_pipeline/cfd/sphere_validation.py report --sphere {gen}/sphere --refinement {gen}/refine --out {gen}/cfd_sanity_report.json",
          f"python offline_pipeline/cfd/flow_image.py --case-dir {gen}/sweep/{mid} --out {gen}/cfd_demo/velocity_slice_{mid}.png --scale 2   (the file slice_y_mid.f32 must be in that case folder)"]),
        ("6. the trainer (writes <model>.onnx and <model>.onnx.model.json; kind is trained only if every row says preliminary CFD, unvalidated)",
         [f"python offline_pipeline/cfd/train_vehicle_aero.py train --build {build} --csv {gen}/sweep/vehicle_cfd.csv --out {gen}/cfd_demo/aero_model.onnx "
          f"--sanity {gen}/cfd_sanity_report.json"]),
        ("7. model validation (hash binding, then a real campaign that must load it)",
         [f"python offline_pipeline/cfd/train_vehicle_aero.py verify --build {build} --model {gen}/cfd_demo/aero_model.onnx --csv {gen}/sweep/vehicle_cfd.csv",
          f"copy {gen}/sweep/geometry.json {gen}/cfd_demo/geometry.json",
          "$env:ORT_DYLIB_PATH = \"<folder of onnxruntime-win-x64-1.27.1>\\lib\\onnxruntime.dll\"",
          f"cargo run --release --manifest-path physics_core/Cargo.toml --example run_campaign -- {build} campaign/conditions_standard.json "
          f"campaign/test_definitions.json <out dir> --tests climb --aero-dir {gen}/cfd_demo",
          "   it must print 'CFD surrogate: PRELIMINARY, UNVALIDATED' and write the same text into <out dir>/campaign_result.json; anything else (exit code 3: the",
          "   reason is in <out dir>/aero_model_status.json) is a failure to read, not to ignore."]),
        ("8. into the package",
         [f"python tools/package/stage_package.py --package <packaged folder> --python-zip <python embed zip> --wheels-dir <wheels> "
          f"--onnxruntime-zip onnxruntime-win-x64-1.27.1.zip --exe-dir physics_core/target/release/examples --cfd-dir {gen}/cfd_demo",
          f"python tools/verify_package.py --package <packaged folder> --require-helpers"]),
    ]


def render_commands(cmds: list[tuple[str, list[str]]]) -> str:
    out = ["", "COMMANDS, in this order (run from the repository root; PowerShell on the GPU laptop):"]
    for title, lines in cmds:
        out.append("")
        out.append(title)
        out += [f"    {line}" for line in lines]
    return "\n".join(out)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seconds-per-case", type=float, required=True, help="wall-clock seconds of ONE measured case")
    ap.add_argument("--measured-profile", default="demo", help="profile the measured case belongs to (default or a profile of class_sweeps.json)")
    ap.add_argument("--measured-class", default="quad_standard")
    ap.add_argument("--profile", default="demo", help="profile used in the printed commands")
    ap.add_argument("--build", default="tools/parts/builds/reference_quad")
    ap.add_argument("--bytes-per-cell", type=int)
    ap.add_argument("--no-commands", action="store_true")
    a = ap.parse_args(argv)
    try:
        cfg_all = S.load_class_config()
        profiles = [DEFAULT_PROFILE] + sorted(cfg_all.get("profiles", {}))
        plan = build_plan(a.seconds_per_case, a.measured_profile, a.measured_class, profiles)
        print(render(plan))
        if not a.no_commands:
            cfg = profile_cfg(a.profile)
            print(render_commands(commands(a.build, a.profile, a.bytes_per_cell, cfg, a.measured_class)))
    except (PlanError, S.SweepError, KeyError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
