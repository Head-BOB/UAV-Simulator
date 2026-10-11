# Tier: D
"""FluidX3D sweep generator and collector for a BUILT vehicle (any class), PRELIMINARY / UNVALIDATED.

Sub-commands, run from the repo root inside .venv:
  generate   from a build directory: watertight geometry (vehicle_stl.py), one folder per case (rotated, lattice-scaled STL +
             setup.cpp + case.json), sweep_manifest.json and run_sweep.ps1 for the GPU laptop. Cases that do not fit are NOT written.
  check      one case: re-check the grid against the VRAM formula and (optionally) the GPU's free VRAM.
  collect    read each case's raw_result.csv (SI, simulation frame), rotate to the body frame, add coefficients, write
             vehicle_cfd.csv with provenance.
  synthetic  write a clearly labelled SYNTHETIC sample CSV (formula, not CFD) for the build's class, to test everything without a GPU.

Conventions are those of fluidx3d_quad_sweep.py (imported, not edited): body X forward / Y right / Z down; V_b = V (cos a cos b,
sin b, sin a cos b); the simulation blows along +x_sim and the STL is rotated by R so the air direction maps to +x_sim.
Differences from the quad tool: the geometry comes from the build, the mesh is translated so the CENTRE OF GRAVITY is the rotation
origin (forces are collected as torques about the CG), and the domain / resolution follow the vehicle's span:
  dx = span / cells_across_span, domain = domain.spans_xyz x span, CG placed inlet_spans x span downstream of the inlet.
Coefficients: C = F / (q A_ref) with A_ref = frontal projected area of the geometry (along X) and Cm = M / (q A_ref L_ref) with
L_ref = span (larger of the x and y extents). ASSUMED conventions, stated in every output row.
Lattice units: nu_lb = nu u_lb / (V dx) (so Re = V span / nu is preserved), tau = 3 nu_lb + 1/2. A case is refused if nu_lb is below
the configured minimum (stability of the lattice scheme at that Reynolds number is not known: TODO(HUMAN-VERIFY)).

FluidX3D is an external, separately licensed program (non-commercial licence, college project). No FluidX3D code is copied here:
setup.cpp only CALLS its public setup API, and every API use is TODO(HUMAN-VERIFY: matches FluidX3D 3.8).
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "tools" / "geometry"))
sys.path.insert(0, str(ROOT / "offline_pipeline"))
import fluidx3d_quad_sweep as Q  # noqa: E402  (rotation and sweep helpers, not edited)
import make_quad_stl as G  # noqa: E402
import vehicle_stl as VS  # noqa: E402
from run_case import vram_check  # noqa: E402

CLASS_FILE = HERE / "class_sweeps.json"
PROVENANCE = "preliminary CFD, unvalidated"
SYNTHETIC_PROVENANCE = "SYNTHETIC formula, not CFD"
RAW_HEADER = Q.RAW_HEADER
OUT_HEADER = Q.OUT_HEADER + ",geometry_sha256,propellers,ref_area_m2,ref_length_m"
SolverName = "FluidX3D 3.8"


class SweepError(Exception):
    pass


def load_class_config(path: Path = CLASS_FILE) -> dict:
    cfg = json.loads(path.read_text(encoding="utf-8"))
    if cfg.get("schema_version") != 1:
        raise SweepError("class_sweeps.json: schema_version must be 1")
    if cfg.get("provenance", {}).get("source_type") != "ASSUMED":
        raise SweepError("class_sweeps.json: provenance must say ASSUMED")
    lbm, dom, air = cfg["lbm"], cfg["domain"], cfg["air"]
    ok = (0.0 < lbm["velocity"] < 0.2 and lbm["min_lattice_viscosity"] > 0 and lbm["flow_throughs"] > 0 and 0 < lbm["avg_fraction"] <= 0.5
          and len(dom["spans_xyz"]) == 3 and all(s > 0 for s in dom["spans_xyz"]) and dom["inlet_spans"] > 0 and dom["max_cells"] > 0
          and 0 <= dom["vram_headroom_fraction"] < 1 and dom["wall_margin_cells"] >= 1 and air["rho_kgpm3"] > 0 and air["nu_m2ps"] > 0)
    if not ok:
        raise SweepError("class_sweeps.json: a value is out of range")
    for name, c in cfg["classes"].items():
        if not (c["speeds_mps"] and all(v > 0 for v in c["speeds_mps"]) and c["angles_deg"] and all(abs(a) < 90 for a in c["angles_deg"]) and c["cells_across_span"] >= 20):
            raise SweepError(f"class {name}: speeds, angles or cells_across_span out of range")
    return cfg


def vehicle_dims(mesh) -> dict:
    lo, hi = G.bounding_box(mesh)
    ext = [hi[i] - lo[i] for i in range(3)]
    return {"extent_m": ext, "span_m": max(ext[0], ext[1]), "bbox_lo": lo, "bbox_hi": hi}


def grid_for(span_m: float, dx_m: float, spans_xyz) -> tuple:
    """Cells (nx, ny, nz) for the domain; each dimension is rounded up to a multiple of 2."""
    dims = []
    for s in spans_xyz:
        n = math.ceil(round(s * span_m / dx_m, 9))
        dims.append(n + (n % 2))
    return tuple(dims)


def total_steps(nx: int, lbm: dict) -> int:
    return int(math.ceil(round(lbm["flow_throughs"] * nx / lbm["velocity"], 6)))


def lattice_viscosity(nu_si: float, lbm_u: float, speed: float, dx_m: float) -> float:
    return nu_si * lbm_u / (speed * dx_m)


def place_mesh(mesh, rows, cg, dx_m: float):
    """Translate so the CG is the origin, rotate by R, divide by dx (lattice units). Returns (mesh, bbox centre offset from the CG in cells)."""
    out = G.Mesh()
    out.v = [tuple(c / dx_m for c in Q.mat_vec(rows, tuple(p[i] - cg[i] for i in range(3)))) for p in mesh.v]
    out.t = list(mesh.t)
    lo, hi = G.bounding_box(out)
    return out, tuple(0.5 * (lo[i] + hi[i]) for i in range(3)), lo, hi


def make_case(speed, alpha, beta, ctx: dict) -> dict:
    cfg, dims, dx = ctx["cfg"], ctx["dims"], ctx["dx_m"]
    dom, lbm, air = cfg["domain"], cfg["lbm"], cfg["air"]
    nx, ny, nz = grid_for(dims["span_m"], dx, dom["spans_xyz"])
    if nx * ny * nz > dom["max_cells"]:
        raise SweepError(f"grid {nx}x{ny}x{nz} = {nx * ny * nz} cells exceeds the {dom['max_cells']} limit")
    required, limit, fits = vram_check({"nx": nx, "ny": ny, "nz": nz}, ctx["memory"])
    if not fits:
        raise SweepError(f"grid {nx}x{ny}x{nz} needs {required} B but only {limit:.0f} B are allowed")
    nu_lb = lattice_viscosity(air["nu_m2ps"], lbm["velocity"], speed, dx)
    if nu_lb < lbm["min_lattice_viscosity"]:
        raise SweepError(f"speed {speed} m/s at dx {dx:.5f} m gives lattice viscosity {nu_lb:.3e} < {lbm['min_lattice_viscosity']:.1e}: refused")
    rows = Q.body_to_sim_rotation(alpha, beta)
    placed, bb_centre, lo, hi = place_mesh(ctx["mesh"], rows, ctx["cg"], dx)
    cg_cells = (dom["inlet_spans"] * dims["span_m"] / dx, 0.5 * ny, 0.5 * nz)
    margin = dom["wall_margin_cells"]
    for axis, n in enumerate((nx, ny, nz)):
        if cg_cells[axis] + lo[axis] < margin or cg_cells[axis] + hi[axis] > n - margin:
            raise SweepError(f"case a={alpha} b={beta}: the vehicle does not fit the domain along axis {axis} with {margin} cells of margin")
    steps = total_steps(nx, lbm)
    stl_text = G.to_ascii_stl(placed, "vehicle_lattice")
    return {
        "case_id": Q.case_id(speed, alpha, beta), "speed_mps": speed, "alpha_deg": alpha, "beta_deg": beta, "dx_m": dx,
        "grid": {"nx": nx, "ny": ny, "nz": nz}, "memory": ctx["memory"], "vram_required_bytes": required, "vram_limit_bytes": limit,
        "total_steps": steps, "avg_steps": int(steps * lbm["avg_fraction"]), "lattice_updates": nx * ny * nz * steps,
        "rotation_rows": [list(r) for r in rows], "span_m": dims["span_m"], "cg_cells": list(cg_cells),
        "bbox_centre_from_cg_cells": list(bb_centre), "ref_area_m2": ctx["ref_area_m2"], "ref_length_m": dims["span_m"],
        "reynolds": speed * dims["span_m"] / air["nu_m2ps"], "nu_lb": nu_lb, "tau": 3.0 * nu_lb + 0.5, "mach_lb": lbm["velocity"] * math.sqrt(3.0),
        "stl_sha256": hashlib.sha256(stl_text.encode()).hexdigest(), "geometry_sha256": ctx["geometry_sha256"],
        "propellers": ctx["propellers"], "vehicle_class": ctx["vehicle_class"], "sweep_config_version": cfg["version"],
        "solver": SolverName, "provenance": PROVENANCE, "_stl_text": stl_text,
    }


SETUP_TEMPLATE = r'''// GENERATED by vehicle_sweep.py for case @@CASE_ID@@. PRELIMINARY / UNVALIDATED. Do not edit.
// Copy to FluidX3D src/setup.cpp (run_sweep.ps1 does this). The GPU runner needs these in defines.hpp
// (run_sweep.ps1 checks them): #define FP16S, #define EQUILIBRIUM_BOUNDARIES, #define FORCE_FIELD, #define SUBGRID,
// and GRAPHICS off for a headless run. TODO(HUMAN-VERIFY: matches FluidX3D 3.8) every API name below.
#include "setup.hpp"

void main_setup() { // vehicle in a wind tunnel, flow along +x; the STL is pre-rotated about the CG and pre-scaled to lattice units
	const uint Nx=@@NX@@u, Ny=@@NY@@u, Nz=@@NZ@@u;
	const float si_dx = @@DX_M@@f;                 // m per lattice cell
	const float si_u = @@SPEED@@f;                 // m/s freestream
	const float si_rho = @@RHO@@f;                 // kg/m^3
	const float si_nu = @@NU@@f;                   // m^2/s
	const float lbm_u = @@LBM_U@@f;
	const float lbm_ref_len = @@SPAN_M@@f/si_dx;   // reference span in cells
	units.set_m_kg_s(lbm_ref_len, lbm_u, 1.0f, @@SPAN_M@@f, si_u, si_rho); // TODO(HUMAN-VERIFY): signature
	LBM lbm(Nx, Ny, Nz, units.nu(si_nu));          // TODO(HUMAN-VERIFY): constructor and units.nu
	const float3 cg = float3(@@CGX@@f, @@CGY@@f, @@CGZ@@f);                       // centre of gravity in cells: forces and torques refer to it
	const float3 center = cg + float3(@@BBX@@f, @@BBY@@f, @@BBZ@@f);               // voxelize_stl centres the BOUNDING BOX here
	lbm.voxelize_stl(get_exe_path()+"vehicle.stl", center, float3x3(1.0f), 0.0f); // TODO(HUMAN-VERIFY): overload and that the bbox centre is what is placed
	const ulong N = lbm.get_N();
	parallel_for(N, [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.x[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // far field, fixed inflow velocity
	}); // TODO(HUMAN-VERIFY): flag name TYPE_E and the use with EQUILIBRIUM_BOUNDARIES
	const ulong total = @@TOTAL_STEPS@@ull, avg_steps = @@AVG_STEPS@@ull, chunk = 50ull;
	lbm.run(0u); // initialise
	double s[6]={0,0,0,0,0,0}, s2=0.0; ulong k=0ull;
	while(lbm.get_t()<total) {
		lbm.run(chunk, total);
		if(lbm.get_t()+avg_steps>=total) { // last quarter of the run
			lbm.update_force_field(); // TODO(HUMAN-VERIFY)
			const float3 f = units.si_F(lbm.object_force(TYPE_S)); // TODO(HUMAN-VERIFY): force ON the object, SI
			const float3 t = units.si_T(lbm.object_torque(cg, TYPE_S)); // TODO(HUMAN-VERIFY): torque about the CG
			s[0]+=f.x; s[1]+=f.y; s[2]+=f.z; s[3]+=t.x; s[4]+=t.y; s[5]+=t.z; s2+=(double)f.x*(double)f.x; k++;
		}
	}
	const double m0 = k ? s[0]/(double)k : 0.0, var = k ? s2/(double)k-m0*m0 : 0.0;
	write_file(get_exe_path()+"raw_result.csv", string("@@RAW_HEADER@@\n")+to_string(m0)+","+to_string(k?s[1]/(double)k:0.0)+","+to_string(k?s[2]/(double)k:0.0)+","
		+to_string(k?s[3]/(double)k:0.0)+","+to_string(k?s[4]/(double)k:0.0)+","+to_string(k?s[5]/(double)k:0.0)+","+to_string(sqrt(var>0.0?var:0.0))+","+to_string(total)+","+to_string(avg_steps)+"\n");
	// velocity-magnitude slice through the mid plane y = Ny/2 (x fastest, then z), float32 SI m/s, for flow_image.py
	// TODO(HUMAN-VERIFY): lbm.u component access and units.si_u; the file must hold Nx*Nz floats
	vector<float> slice((ulong)Nx*(ulong)Nz);
	for(uint z=0u; z<Nz; z++) for(uint x=0u; x<Nx; x++) { const ulong n = lbm.index(x, Ny/2u, z); // TODO(HUMAN-VERIFY): index helper
		slice[(ulong)z*(ulong)Nx+(ulong)x] = units.si_u(sqrt(sq(lbm.u.x[n])+sq(lbm.u.y[n])+sq(lbm.u.z[n]))); }
	write_file(get_exe_path()+"slice_y_mid.f32", slice); // TODO(HUMAN-VERIFY): binary write overload
	exit(0);
}
'''

RUN_SWEEP_PS1 = r'''# GENERATED. Run on the Windows GPU laptop (RTX 4050, 6 GB). PRELIMINARY / UNVALIDATED results.
# Usage:  powershell -File run_sweep.ps1 -FluidX3D C:\tools\FluidX3D [-BuildCmd "..."] [-Python python]
# For each case: log the VRAM estimate, REFUSE if it does not fit in the GPU's free memory, copy the case files, build, run,
# keep raw_result.csv and slice_y_mid.f32. A refused or failed case is skipped and listed at the end.
param(
  [Parameter(Mandatory=$true)][string]$FluidX3D,
  [string]$BuildCmd = 'msbuild FluidX3D.sln /p:Configuration=Release /p:Platform=x64 /v:m',  # TODO(HUMAN-VERIFY): your build command
  [string]$ExeRel = 'bin\FluidX3D.exe',                                                   # TODO(HUMAN-VERIFY): where the exe lands
  [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $here '@@REPO_REL@@')).Path
$tool = Join-Path $repo 'offline_pipeline\cfd\vehicle_sweep.py'
$defines = Get-Content (Join-Path $FluidX3D 'src\defines.hpp') -Raw
foreach ($d in @('FP16S','EQUILIBRIUM_BOUNDARIES','FORCE_FIELD','SUBGRID')) {
  if ($defines -notmatch "(?m)^\s*#define\s+$d\b") { throw "defines.hpp must contain an active '#define $d' (see README)" }
}
if ($defines -match '(?m)^\s*#define\s+GRAPHICS\b') { throw "turn GRAPHICS off in defines.hpp for a headless run" }
$failed = @()
foreach ($case in Get-ChildItem -Directory $here | Where-Object { Test-Path (Join-Path $_.FullName 'case.json') }) {
  $free = [int](& nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | Select-Object -First 1)
  Write-Host "== $($case.Name): GPU free VRAM $free MiB"
  & $Python -I $tool check --case (Join-Path $case.FullName 'case.json') --free-vram-mib $free
  if ($LASTEXITCODE -ne 0) { Write-Warning "REFUSED $($case.Name): does not fit"; $failed += $case.Name; continue }
  try {
    Copy-Item (Join-Path $case.FullName 'setup.cpp') (Join-Path $FluidX3D 'src\setup.cpp') -Force
    Push-Location $FluidX3D
    Invoke-Expression $BuildCmd
    if ($LASTEXITCODE -ne 0) { throw "build failed" }
    $exe = Join-Path $FluidX3D $ExeRel
    Copy-Item (Join-Path $case.FullName 'vehicle.stl') (Join-Path (Split-Path $exe) 'vehicle.stl') -Force
    & $exe
    Copy-Item (Join-Path (Split-Path $exe) 'raw_result.csv') (Join-Path $case.FullName 'raw_result.csv') -Force
    if (Test-Path (Join-Path (Split-Path $exe) 'slice_y_mid.f32')) { Copy-Item (Join-Path (Split-Path $exe) 'slice_y_mid.f32') (Join-Path $case.FullName 'slice_y_mid.f32') -Force }
  } catch { Write-Warning "FAILED $($case.Name): $_"; $failed += $case.Name } finally { Pop-Location }
}
& $Python -I $tool collect --sweep $here
if ($failed.Count -gt 0) { Write-Warning "Skipped or failed: $($failed -join ', ')" }
Write-Host "Send back: vehicle_cfd.csv, sweep_manifest.json and the slice_y_mid.f32 files from $here"
'''


def render_setup(case: dict, cfg: dict) -> str:
    g, air, lbm = case["grid"], cfg["air"], cfg["lbm"]
    fills = {
        "CASE_ID": case["case_id"], "NX": g["nx"], "NY": g["ny"], "NZ": g["nz"], "DX_M": repr(case["dx_m"]), "SPEED": repr(case["speed_mps"]),
        "RHO": repr(air["rho_kgpm3"]), "NU": repr(air["nu_m2ps"]), "LBM_U": repr(lbm["velocity"]), "SPAN_M": repr(case["span_m"]),
        "CGX": repr(case["cg_cells"][0]), "CGY": repr(case["cg_cells"][1]), "CGZ": repr(case["cg_cells"][2]),
        "BBX": repr(case["bbox_centre_from_cg_cells"][0]), "BBY": repr(case["bbox_centre_from_cg_cells"][1]), "BBZ": repr(case["bbox_centre_from_cg_cells"][2]),
        "TOTAL_STEPS": case["total_steps"], "AVG_STEPS": case["avg_steps"], "RAW_HEADER": RAW_HEADER,
    }
    text = SETUP_TEMPLATE
    for k, v in fills.items():
        text = text.replace(f"@@{k}@@", str(v))
    if "@@" in text:
        raise SweepError("unreplaced placeholder in setup.cpp")
    return text


def prepare(build_dir: Path, bytes_per_cell: int, vram_gb: float, cells_across: int | None, props: str, cfg: dict) -> dict:
    assembly = json.loads((build_dir / "assembly.json").read_text(encoding="utf-8"))
    cls = assembly["vehicle_class"]
    if cls not in cfg["classes"]:
        raise SweepError(f"no sweep definition for vehicle class {cls!r} in class_sweeps.json")
    try:
        geo = VS.build_geometry(assembly, props)
    except VS.GeometryError as e:
        raise SweepError(f"geometry: {e}") from e
    stl_text = G.to_ascii_stl(geo["mesh"], assembly["build_name"])
    gdoc = VS.geometry_document(geo, assembly, stl_text, props)
    dims = vehicle_dims(geo["mesh"])
    cells = cells_across or cfg["classes"][cls]["cells_across_span"]
    memory = {"bytes_per_cell": int(bytes_per_cell), "vram_bytes": int(vram_gb * 1024 ** 3), "headroom_fraction": cfg["domain"]["vram_headroom_fraction"]}
    return {"cfg": cfg, "dims": dims, "dx_m": dims["span_m"] / cells, "memory": memory, "mesh": geo["mesh"], "cg": tuple(assembly["totals"]["cg_from_frame_centre_m"]),
            "ref_area_m2": gdoc["projected_area_m2"]["along_x"], "geometry_sha256": gdoc["stl_sha256"], "propellers": props, "vehicle_class": cls,
            "class_def": cfg["classes"][cls], "stl_text": stl_text, "geometry_doc": gdoc, "assembly": assembly}


def generate(build_dir: Path, out: Path, bytes_per_cell: int, vram_gb: float, cells_across, speeds, angles, max_cases, props: str = "omit") -> list:
    cfg = load_class_config()
    ctx = prepare(build_dir, bytes_per_cell, vram_gb, cells_across, props, cfg)
    cd = ctx["class_def"]
    cases = Q.sweep_cases(speeds or cd["speeds_mps"], angles or cd["angles_deg"], max_cases)
    planned = [make_case(v, a, b, ctx) for v, a, b in cases]    # every case is checked before anything is written
    out.mkdir(parents=True, exist_ok=True)
    (out / "vehicle_unrotated.stl").write_text(ctx["stl_text"], encoding="utf-8", newline="\n")
    (out / "geometry.json").write_text(json.dumps(ctx["geometry_doc"], indent=2) + "\n", encoding="utf-8", newline="\n")
    manifest = []
    for case in planned:
        d = out / case["case_id"]
        d.mkdir(exist_ok=True)
        (d / "vehicle.stl").write_text(case.pop("_stl_text"), encoding="utf-8", newline="\n")
        (d / "setup.cpp").write_text(render_setup(case, cfg), encoding="utf-8", newline="\n")
        (d / "case.json").write_text(json.dumps(case, indent=2) + "\n", encoding="utf-8", newline="\n")
        manifest.append(case)
    out_abs = out.resolve()
    rel = Path(*([".."] * len(out_abs.relative_to(ROOT).parts))) if out_abs.is_relative_to(ROOT) else Path(".")
    (out / "run_sweep.ps1").write_text(RUN_SWEEP_PS1.replace("@@REPO_REL@@", str(rel).replace("/", "\\")), encoding="utf-8", newline="\n")
    doc = {"provenance": PROVENANCE, "build": ctx["assembly"]["build_name"], "vehicle_class": ctx["vehicle_class"], "geometry_sha256": ctx["geometry_sha256"],
           "sweep_config_version": cfg["version"], "propellers": props, "ref_area_m2": ctx["ref_area_m2"], "ref_length_m": ctx["dims"]["span_m"], "cases": manifest}
    (out / "sweep_manifest.json").write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8", newline="\n")
    return manifest


def check_case(case_path: Path, free_vram_mib=None) -> str:
    case = json.loads(case_path.read_text(encoding="utf-8"))
    required, limit, fits = vram_check(case["grid"], case["memory"])
    msg = f"{case['case_id']}: estimate {required / 2**20:.0f} MiB, allowed {limit / 2**20:.0f} MiB"
    if not fits:
        raise SweepError(msg + " -> does not fit the configured VRAM")
    if free_vram_mib is not None and required > free_vram_mib * 2**20 * (1.0 - case["memory"]["headroom_fraction"]):
        raise SweepError(msg + f" -> does not fit the {free_vram_mib} MiB currently free")
    return msg + " -> fits"


def collect_row(case: dict, raw_line: str, rho: float) -> dict:
    """Body-frame force / torque (about the CG) and coefficients from one raw_result.csv data line (SI, simulation frame)."""
    vals = [float(x) for x in raw_line.strip().split(",")]
    if len(vals) != len(RAW_HEADER.split(",")) or not all(math.isfinite(x) for x in vals):
        raise SweepError(f"{case['case_id']}: raw result is malformed or non-finite")
    f_sim, t_sim = tuple(vals[0:3]), tuple(vals[3:6])
    if f_sim[0] <= 0.0:
        raise SweepError(f"{case['case_id']}: drag along the flow is {f_sim[0]} N (must be > 0): force sign convention or setup is wrong, TODO(HUMAN-VERIFY)")
    rows = case["rotation_rows"]
    f_b, t_b = Q.mat_t_vec(rows, f_sim), Q.mat_t_vec(rows, t_sim)
    q = 0.5 * rho * case["speed_mps"] ** 2
    a_ref, l_ref = case["ref_area_m2"], case["ref_length_m"]
    return {"case_id": case["case_id"], "speed_mps": case["speed_mps"], "alpha_deg": case["alpha_deg"], "beta_deg": case["beta_deg"], "f": f_b, "m": t_b,
            "c": tuple(x / (q * a_ref) for x in f_b), "cm": tuple(x / (q * a_ref * l_ref) for x in t_b)}


def _row_cells(r: dict, case: dict, doc: dict, provenance: str, solver: str) -> list:
    return [r["case_id"], f"{r['speed_mps']}", f"{r['alpha_deg']}", f"{r['beta_deg']}", *(f"{x:.9g}" for x in r["f"]), *(f"{x:.9g}" for x in r["m"]),
            *(f"{x:.9g}" for x in r["c"]), *(f"{x:.9g}" for x in r["cm"]), solver, provenance, doc["geometry_sha256"], doc["propellers"],
            f"{doc['ref_area_m2']:.9g}", f"{doc['ref_length_m']:.9g}"]


def collect(sweep: Path) -> Path:
    doc = json.loads((sweep / "sweep_manifest.json").read_text(encoding="utf-8"))
    cfg = load_class_config()
    buf = io.StringIO()
    writer = csv.writer(buf, lineterminator="\n")
    writer.writerow(OUT_HEADER.split(","))
    for case in doc["cases"]:
        raw = sweep / case["case_id"] / "raw_result.csv"
        if not raw.exists():
            print(f"missing {raw}: case skipped", file=sys.stderr)
            continue
        data = raw.read_text(encoding="utf-8").strip().splitlines()
        if len(data) != 2 or data[0] != RAW_HEADER:
            raise SweepError(f"{raw}: unexpected layout")
        r = collect_row(case, data[1], cfg["air"]["rho_kgpm3"])
        writer.writerow(_row_cells(r, case, doc, PROVENANCE, case["solver"]))
    out = sweep / "vehicle_cfd.csv"
    out.write_text(buf.getvalue(), encoding="utf-8", newline="\n")
    return out


def synthetic_force(speed: float, alpha: float, beta: float, scale: float) -> tuple:
    """Formula behind the SYNTHETIC sample: F = V^2 (c0 + c1 a + c2 b + c3 a^2 + c4 b^2) per axis (a, b in rad), scaled by the frontal area."""
    c = {"x": (-0.05, 0.02, 0.01, -0.30, -0.25), "y": (0.0, 0.02, -0.60, 0.0, 0.02), "z": (0.0, -1.50, 0.02, -0.20, 0.0)}
    return tuple(scale * speed * speed * (k[0] + k[1] * alpha + k[2] * beta + k[3] * alpha * alpha + k[4] * beta * beta) for k in (c["x"], c["y"], c["z"]))


def synthetic_csv(build_dir: Path) -> str:
    """Clearly labelled SYNTHETIC sample rows for the build's class (no GPU, no CFD)."""
    cfg = load_class_config()
    assembly = json.loads((build_dir / "assembly.json").read_text(encoding="utf-8"))
    cd = cfg["classes"][assembly["vehicle_class"]]
    gdoc_geo = VS.build_geometry(assembly)
    stl = G.to_ascii_stl(gdoc_geo["mesh"], assembly["build_name"])
    gdoc = VS.geometry_document(gdoc_geo, assembly, stl, "omit")
    doc = {"geometry_sha256": gdoc["stl_sha256"], "propellers": "omit", "ref_area_m2": gdoc["projected_area_m2"]["along_x"], "ref_length_m": vehicle_dims(gdoc_geo["mesh"])["span_m"]}
    buf = io.StringIO()
    writer = csv.writer(buf, lineterminator="\n")
    writer.writerow(OUT_HEADER.split(","))
    rho = cfg["air"]["rho_kgpm3"]
    for v, a, b in Q.sweep_cases(cd["speeds_mps"], cd["angles_deg"]):
        f = synthetic_force(v, math.radians(a), math.radians(b), doc["ref_area_m2"])
        q = 0.5 * rho * v * v
        r = {"case_id": Q.case_id(v, a, b), "speed_mps": v, "alpha_deg": a, "beta_deg": b, "f": f, "m": (0.0, 0.0, 0.0),
             "c": tuple(x / (q * doc["ref_area_m2"]) for x in f), "cm": (0.0, 0.0, 0.0)}
        writer.writerow(_row_cells(r, {}, doc, SYNTHETIC_PROVENANCE, "SYNTHETIC formula"))
    return buf.getvalue()


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("generate")
    g.add_argument("--build", type=Path, required=True)
    g.add_argument("--out", type=Path, required=True)
    g.add_argument("--bytes-per-cell", type=int, required=True, help="FluidX3D bytes per cell for your defines.hpp: MEASURE it (TODO(HUMAN-VERIFY: B-7)); no default on purpose")
    g.add_argument("--vram-gb", type=float, default=6.0)
    g.add_argument("--cells-across", type=int)
    g.add_argument("--speeds", type=float, nargs="+")
    g.add_argument("--angles", type=float, nargs="+")
    g.add_argument("--max-cases", type=int)
    g.add_argument("--props", choices=("omit", "disc"), default="omit")
    c = sub.add_parser("check")
    c.add_argument("--case", type=Path, required=True)
    c.add_argument("--free-vram-mib", type=int)
    k = sub.add_parser("collect")
    k.add_argument("--sweep", type=Path, required=True)
    s = sub.add_parser("synthetic")
    s.add_argument("--build", type=Path, required=True)
    s.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    try:
        if a.cmd == "generate":
            m = generate(a.build, a.out, a.bytes_per_cell, a.vram_gb, a.cells_across, a.speeds, a.angles, a.max_cases, a.props)
            print(f"wrote {len(m)} cases to {a.out}; grid {m[0]['grid']}, {m[0]['total_steps']} steps each")
        elif a.cmd == "check":
            print(check_case(a.case, a.free_vram_mib))
        elif a.cmd == "synthetic":
            a.out.write_text(synthetic_csv(a.build), encoding="utf-8", newline="\n")
            print(f"wrote {a.out} (SYNTHETIC)")
        else:
            print(f"wrote {collect(a.sweep)}")
    except SweepError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
