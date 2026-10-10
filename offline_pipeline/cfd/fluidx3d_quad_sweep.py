# Tier: D
"""FluidX3D sweep generator and collector for the quad (whole-vehicle aero), PRELIMINARY / UNVALIDATED.

Three sub-commands, run from the repo root inside .venv:
  generate  write one folder per case (rotated, lattice-scaled STL + setup.cpp + case.json), a
            sweep_manifest.json and run_sweep.ps1 for the GPU laptop. Cases that do not fit are NOT written.
  check     one case: re-check the grid against the VRAM formula and (optionally) the GPU's free VRAM.
  collect   read each case's raw_result.csv (SI, simulation frame), rotate to the body frame, add
            coefficients, write quad_cfd.csv with provenance.

Conventions (match physics_core aero_surrogate::aero_inputs, TODO(HUMAN-VERIFY) there):
  body frame X forward, Y right, Z down. Vehicle air-relative velocity in the body frame is
  V (cos a cos b, sin b, sin a cos b), a = alpha = atan2(w, u), b = beta = asin(v/|V|).
  The simulation always blows along +x_sim. The STL is rotated by R (rows e1 e2 e3) so that the
  body direction of the AIR (-V_b) maps to +x_sim: x_sim = R x_body, e1 = -V_b/|V|,
  e3 = unit(Z_b - (Z_b . e1) e1), e2 = e3 x e1.  Body-frame force = R^T force_sim.
  V = 0 is not simulated: aero force is exactly zero with no relative flow (F ~ V^2); the trainer adds it.

FluidX3D is an external, separately licensed program (non-commercial licence, college project, recorded
in HUMAN-TODO). No FluidX3D code is copied here: setup.cpp only CALLS its public setup API, and every
API use is TODO(HUMAN-VERIFY: matches FluidX3D 3.8).
"""
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
sys.path.insert(0, str(ROOT / "tools" / "geometry"))
sys.path.insert(0, str(ROOT / "offline_pipeline"))
import make_quad_stl as geo  # noqa: E402
from run_case import vram_check  # noqa: E402  (same VRAM formula as the propeller pipeline)

AIR_RHO_KG_M3 = 1.225          # ISA sea level, 15 C
AIR_NU_M2_S = 1.4607e-5        # mu 1.7894e-5 Pa s / rho 1.225 (ISA sea level), ASSUMED still air at 15 C
LBM_VELOCITY = 0.075           # lattice inflow speed, a usual low-Mach choice (ASSUMED)
DOMAIN_SPANS = (4.0, 2.0, 2.0)  # domain size in units of REF_SPAN_M (x downstream, y, z)
INLET_SPANS = 1.0              # object centre this many spans downstream of the inlet
REF_SPAN_M = 0.45              # reference span, just above the 0.4418 m bounding box (ASSUMED)
FLOW_THROUGHS = 5.0            # simulated flow-through times (ASSUMED, check convergence on the real run)
AVG_FRACTION = 0.25            # average forces over the last quarter of the run
MAX_CELLS = 300 ** 3           # the user's limit for the RTX 4050 6 GB laptop
DEFAULT_SPEEDS_MPS = (5.0, 10.0, 15.0, 20.0)
DEFAULT_ANGLES_DEG = (-20.0, -10.0, 0.0, 10.0, 20.0)
A_REF_M2 = 0.4418 * 0.10       # bbox width x height: coefficient convention only (ASSUMED)
L_REF_M = 0.4                  # motor spacing: moment coefficient convention only
HEADROOM = 0.15                # keep 15 % of VRAM free (ASSUMED)
RAW_HEADER = "fx_sim_n,fy_sim_n,fz_sim_n,tx_sim_nm,ty_sim_nm,tz_sim_nm,force_std_n,total_steps,avg_steps"
OUT_HEADER = ("case_id,speed_mps,alpha_deg,beta_deg,fx_body_n,fy_body_n,fz_body_n,mx_body_nm,my_body_nm,mz_body_nm,"
              "cx,cy,cz,cl_x,cl_y,cl_z,solver,provenance")
PROVENANCE = "preliminary CFD, unvalidated"


class SweepError(Exception):
    pass


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def body_to_sim_rotation(alpha_deg: float, beta_deg: float):
    """Rows (e1, e2, e3): x_sim = R x_body, with the air direction (-V_b) mapped to +x_sim."""
    if not all(math.isfinite(v) for v in (alpha_deg, beta_deg)) or abs(alpha_deg) >= 90.0 or abs(beta_deg) >= 90.0:
        raise SweepError("alpha and beta must be finite and within (-90, 90) degrees")
    a, b = math.radians(alpha_deg), math.radians(beta_deg)
    v_b = (math.cos(a) * math.cos(b), math.sin(b), math.sin(a) * math.cos(b))
    e1 = tuple(-x for x in v_b)
    zb = (0.0, 0.0, 1.0)
    d = sum(zb[i] * e1[i] for i in range(3))
    e3 = _unit(tuple(zb[i] - d * e1[i] for i in range(3)))
    e2 = _cross(e3, e1)
    return (e1, e2, e3)


def mat_vec(rows, v):
    return tuple(sum(r[i] * v[i] for i in range(3)) for r in rows)


def mat_t_vec(rows, v):
    return tuple(sum(rows[k][i] * v[k] for k in range(3)) for i in range(3))


def grid_for(dx_m: float):
    """Cells (nx, ny, nz) for the domain; each dimension is rounded up to a multiple of 2."""
    dims = []
    for spans in DOMAIN_SPANS:
        n = math.ceil(spans * REF_SPAN_M / dx_m)
        dims.append(n + (n % 2))
    return tuple(dims)


def rotated_scaled_mesh(mesh, rows, dx_m: float):
    """Rotate every vertex by R and divide by dx (lattice units). R is a proper rotation, so the
    winding (outward normals) is unchanged."""
    out = geo.Mesh()
    out.v = [tuple(c / dx_m for c in mat_vec(rows, p)) for p in mesh.v]
    out.t = list(mesh.t)
    return out


def case_id(speed: float, alpha: float, beta: float) -> str:
    def f(x):
        return f"{'m' if x < 0 else 'p'}{abs(int(round(x))):02d}"
    return f"v{int(round(speed)):02d}_a{f(alpha)}_b{f(beta)}"


def sweep_cases(speeds, angles, max_cases=None):
    """alpha sweep at beta = 0, beta sweep at alpha = 0 (pruned: no alpha x beta grid), speeds > 0 only."""
    cases = []
    for v in speeds:
        if not (math.isfinite(v) and v > 0.0):
            raise SweepError("speeds must be finite and > 0 (V = 0 is analytic zero, not simulated)")
        for a in angles:
            cases.append((v, a, 0.0))
        for b in angles:
            if b != 0.0:
                cases.append((v, 0.0, b))
    return cases[:max_cases] if max_cases else cases


def total_steps(nx: int) -> int:
    return int(math.ceil(round(FLOW_THROUGHS * nx / LBM_VELOCITY, 6)))


SETUP_TEMPLATE = r'''// GENERATED by fluidx3d_quad_sweep.py for case @@CASE_ID@@. PRELIMINARY / UNVALIDATED. Do not edit.
// Copy to FluidX3D src/setup.cpp (run_sweep.ps1 does this). The GPU runner needs these in defines.hpp
// (run_sweep.ps1 checks them): #define FP16S, #define EQUILIBRIUM_BOUNDARIES, #define FORCE_FIELD, #define SUBGRID,
// and GRAPHICS off for a headless run. TODO(HUMAN-VERIFY: matches FluidX3D 3.8) every API name below.
#include "setup.hpp"

void main_setup() { // vehicle in a wind tunnel, flow along +x; the STL is pre-rotated and pre-scaled to lattice units
	const uint Nx=@@NX@@u, Ny=@@NY@@u, Nz=@@NZ@@u;
	const float si_dx = @@DX_M@@f;                 // m per lattice cell
	const float si_u = @@SPEED@@f;                 // m/s freestream
	const float si_rho = @@RHO@@f;                 // kg/m^3
	const float si_nu = @@NU@@f;                   // m^2/s
	const float lbm_u = @@LBM_U@@f;
	const float lbm_ref_len = @@REF_SPAN_M@@f/si_dx; // reference span in cells
	units.set_m_kg_s(lbm_ref_len, lbm_u, 1.0f, @@REF_SPAN_M@@f, si_u, si_rho); // TODO(HUMAN-VERIFY): signature
	LBM lbm(Nx, Ny, Nz, units.nu(si_nu));          // TODO(HUMAN-VERIFY): constructor and units.nu
	const float3 center = float3(@@CX@@f, 0.5f*(float)Ny, 0.5f*(float)Nz);
	lbm.voxelize_stl(get_exe_path()+"quad.stl", center, float3x3(1.0f), 0.0f); // TODO(HUMAN-VERIFY): overload; the STL is already in cells, bbox centre = body origin
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
			const float3 t = units.si_T(lbm.object_torque(center, TYPE_S)); // TODO(HUMAN-VERIFY): torque about the body origin
			s[0]+=f.x; s[1]+=f.y; s[2]+=f.z; s[3]+=t.x; s[4]+=t.y; s[5]+=t.z; s2+=(double)f.x*(double)f.x; k++;
		}
	}
	const double m0 = k ? s[0]/(double)k : 0.0, var = k ? s2/(double)k-m0*m0 : 0.0;
	write_file(get_exe_path()+"raw_result.csv", string("@@RAW_HEADER@@\n")+to_string(m0)+","+to_string(k?s[1]/(double)k:0.0)+","+to_string(k?s[2]/(double)k:0.0)+","
		+to_string(k?s[3]/(double)k:0.0)+","+to_string(k?s[4]/(double)k:0.0)+","+to_string(k?s[5]/(double)k:0.0)+","+to_string(sqrt(var>0.0?var:0.0))+","+to_string(total)+","+to_string(avg_steps)+"\n");
	exit(0);
}
'''

RUN_SWEEP_PS1 = r'''# GENERATED. Run on the Windows GPU laptop (RTX 4050, 6 GB). PRELIMINARY / UNVALIDATED results.
# Usage:  powershell -File run_sweep.ps1 -FluidX3D C:\tools\FluidX3D [-BuildCmd "..."] [-Python python]
# For each case: log the VRAM estimate, REFUSE if it does not fit in the GPU's free memory, copy the
# case files, build, run, keep raw_result.csv. A refused or failed case is skipped and listed at the end.
param(
  [Parameter(Mandatory=$true)][string]$FluidX3D,
  [string]$BuildCmd = 'msbuild FluidX3D.sln /p:Configuration=Release /p:Platform=x64 /v:m',  # TODO(HUMAN-VERIFY): your build command
  [string]$ExeRel = 'bin\FluidX3D.exe',                                                   # TODO(HUMAN-VERIFY): where the exe lands
  [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $here '@@REPO_REL@@')).Path
$checker = Join-Path $repo 'offline_pipeline\cfd\fluidx3d_quad_sweep.py'
$defines = Get-Content (Join-Path $FluidX3D 'src\defines.hpp') -Raw
foreach ($d in @('FP16S','EQUILIBRIUM_BOUNDARIES','FORCE_FIELD','SUBGRID')) {
  if ($defines -notmatch "(?m)^\s*#define\s+$d\b") { throw "defines.hpp must contain an active '#define $d' (see README)" }
}
if ($defines -match '(?m)^\s*#define\s+GRAPHICS\b') { throw "turn GRAPHICS off in defines.hpp for a headless run" }
$failed = @()
foreach ($case in Get-ChildItem -Directory $here | Where-Object { Test-Path (Join-Path $_.FullName 'case.json') }) {
  $free = [int](& nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | Select-Object -First 1)
  Write-Host "== $($case.Name): GPU free VRAM $free MiB"
  & $Python -I $checker check --case (Join-Path $case.FullName 'case.json') --free-vram-mib $free
  if ($LASTEXITCODE -ne 0) { Write-Warning "REFUSED $($case.Name): does not fit"; $failed += $case.Name; continue }
  try {
    Copy-Item (Join-Path $case.FullName 'setup.cpp') (Join-Path $FluidX3D 'src\setup.cpp') -Force
    Push-Location $FluidX3D
    Invoke-Expression $BuildCmd
    if ($LASTEXITCODE -ne 0) { throw "build failed" }
    $exe = Join-Path $FluidX3D $ExeRel
    Copy-Item (Join-Path $case.FullName 'quad.stl') (Join-Path (Split-Path $exe) 'quad.stl') -Force
    & $exe
    Copy-Item (Join-Path (Split-Path $exe) 'raw_result.csv') (Join-Path $case.FullName 'raw_result.csv') -Force
  } catch { Write-Warning "FAILED $($case.Name): $_"; $failed += $case.Name } finally { Pop-Location }
}
& $Python -I $checker collect --sweep $here
if ($failed.Count -gt 0) { Write-Warning "Skipped or failed: $($failed -join ', ')" }
Write-Host "Send back: quad_cfd.csv and sweep_manifest.json from $here"
'''


def render_setup(case: dict) -> str:
    g = case["grid"]
    fills = {
        "CASE_ID": case["case_id"], "NX": g["nx"], "NY": g["ny"], "NZ": g["nz"],
        "DX_M": repr(case["dx_m"]), "SPEED": repr(case["speed_mps"]), "RHO": repr(AIR_RHO_KG_M3),
        "NU": repr(AIR_NU_M2_S), "LBM_U": repr(LBM_VELOCITY), "REF_SPAN_M": repr(REF_SPAN_M),
        "CX": repr(INLET_SPANS * REF_SPAN_M / case["dx_m"]), "TOTAL_STEPS": case["total_steps"],
        "AVG_STEPS": case["avg_steps"], "RAW_HEADER": RAW_HEADER,
    }
    text = SETUP_TEMPLATE
    for k, v in fills.items():
        text = text.replace(f"@@{k}@@", str(v))
    if "@@" in text:
        raise SweepError("unreplaced placeholder in setup.cpp")
    return text


def make_case(speed, alpha, beta, dx_m, memory, base_mesh) -> dict:
    nx, ny, nz = grid_for(dx_m)
    if nx * ny * nz > MAX_CELLS:
        raise SweepError(f"grid {nx}x{ny}x{nz} = {nx * ny * nz} cells exceeds the {MAX_CELLS} limit")
    required, limit, fits = vram_check({"nx": nx, "ny": ny, "nz": nz}, memory)
    if not fits:
        raise SweepError(f"grid {nx}x{ny}x{nz} needs {required} B but only {limit:.0f} B are allowed")
    rows = body_to_sim_rotation(alpha, beta)
    stl_text = geo.to_ascii_stl(rotated_scaled_mesh(base_mesh, rows, dx_m), "quad_lattice")
    steps = total_steps(nx)
    return {
        "case_id": case_id(speed, alpha, beta), "speed_mps": speed, "alpha_deg": alpha, "beta_deg": beta,
        "dx_m": dx_m, "grid": {"nx": nx, "ny": ny, "nz": nz}, "memory": memory,
        "vram_required_bytes": required, "vram_limit_bytes": limit,
        "total_steps": steps, "avg_steps": int(steps * AVG_FRACTION), "rotation_rows": [list(r) for r in rows],
        "stl_sha256": hashlib.sha256(stl_text.encode()).hexdigest(), "solver": "FluidX3D 3.8",
        "provenance": PROVENANCE, "_stl_text": stl_text,
    }


def generate(out: Path, bytes_per_cell: int, vram_gb: float, dx_m: float, speeds, angles, max_cases) -> list[dict]:
    memory = {"bytes_per_cell": int(bytes_per_cell), "vram_bytes": int(vram_gb * 1024 ** 3), "headroom_fraction": HEADROOM}
    cat = json.loads(geo.CATALOG.read_text())
    cfg = json.loads(geo.CONFIG.read_text())
    base = geo.build_quad(cat, cfg)
    planned = [make_case(v, a, b, dx_m, memory, base) for v, a, b in sweep_cases(speeds, angles, max_cases)]  # all checked before any write
    out.mkdir(parents=True, exist_ok=True)
    manifest = []
    for case in planned:
        d = out / case["case_id"]
        d.mkdir(exist_ok=True)
        (d / "quad.stl").write_text(case.pop("_stl_text"))
        (d / "setup.cpp").write_text(render_setup(case))
        (d / "case.json").write_text(json.dumps(case, indent=2) + "\n")
        manifest.append(case)
    rel = Path(*([".."] * len(out.resolve().relative_to(ROOT).parts))) if out.resolve().is_relative_to(ROOT) else Path(".")
    (out / "run_sweep.ps1").write_text(RUN_SWEEP_PS1.replace("@@REPO_REL@@", str(rel).replace("/", "\\")))
    (out / "sweep_manifest.json").write_text(json.dumps({"provenance": PROVENANCE, "cases": manifest}, indent=2) + "\n")
    return manifest


def check_case(case_path: Path, free_vram_mib=None) -> str:
    case = json.loads(case_path.read_text())
    required, limit, fits = vram_check(case["grid"], case["memory"])
    msg = f"{case['case_id']}: estimate {required / 2**20:.0f} MiB, allowed {limit / 2**20:.0f} MiB"
    if not fits:
        raise SweepError(msg + " -> does not fit the configured VRAM")
    if free_vram_mib is not None and required > free_vram_mib * 2**20 * (1.0 - case["memory"]["headroom_fraction"]):
        raise SweepError(msg + f" -> does not fit the {free_vram_mib} MiB currently free")
    return msg + " -> fits"


def collect_row(case: dict, raw_line: str) -> dict:
    """Body-frame force/torque and coefficients from one raw_result.csv data line (SI, simulation frame)."""
    vals = [float(x) for x in raw_line.strip().split(",")]
    if len(vals) != len(RAW_HEADER.split(",")) or not all(math.isfinite(x) for x in vals):
        raise SweepError(f"{case['case_id']}: raw result is malformed or non-finite")
    f_sim, t_sim = tuple(vals[0:3]), tuple(vals[3:6])
    if f_sim[0] <= 0.0:
        raise SweepError(f"{case['case_id']}: drag along the flow is {f_sim[0]} N (must be > 0): "
                         "force sign convention or setup is wrong, TODO(HUMAN-VERIFY)")
    rows = case["rotation_rows"]
    f_b, t_b = mat_t_vec(rows, f_sim), mat_t_vec(rows, t_sim)
    q = 0.5 * AIR_RHO_KG_M3 * case["speed_mps"] ** 2
    return {"case_id": case["case_id"], "speed_mps": case["speed_mps"], "alpha_deg": case["alpha_deg"],
            "beta_deg": case["beta_deg"], "f": f_b, "m": t_b,
            "c": tuple(x / (q * A_REF_M2) for x in f_b), "cm": tuple(x / (q * A_REF_M2 * L_REF_M) for x in t_b)}


def collect(sweep: Path) -> Path:
    manifest = json.loads((sweep / "sweep_manifest.json").read_text())
    buf = io.StringIO()
    writer = csv.writer(buf, lineterminator="\n")  # quotes fields that contain a comma (the provenance text does)
    writer.writerow(OUT_HEADER.split(","))
    for case in manifest["cases"]:
        raw = sweep / case["case_id"] / "raw_result.csv"
        if not raw.exists():
            print(f"missing {raw}: case skipped", file=sys.stderr)
            continue
        data = raw.read_text().strip().splitlines()
        if len(data) != 2 or data[0] != RAW_HEADER:
            raise SweepError(f"{raw}: unexpected layout")
        r = collect_row(case, data[1])
        writer.writerow([r["case_id"], f"{r['speed_mps']}", f"{r['alpha_deg']}", f"{r['beta_deg']}",
                         *(f"{x:.9g}" for x in r["f"]), *(f"{x:.9g}" for x in r["m"]),
                         *(f"{x:.9g}" for x in r["c"]), *(f"{x:.9g}" for x in r["cm"]),
                         case["solver"], PROVENANCE])
    out = sweep / "quad_cfd.csv"
    out.write_text(buf.getvalue())
    return out


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("generate")
    g.add_argument("--out", type=Path, default=HERE / "generated" / "quad_sweep")
    g.add_argument("--bytes-per-cell", type=int, required=True,
                   help="FluidX3D bytes per cell for your defines.hpp: MEASURE it (TODO(HUMAN-VERIFY: B-7)); no default on purpose")
    g.add_argument("--vram-gb", type=float, default=6.0)
    g.add_argument("--dx-m", type=float, default=0.004)
    g.add_argument("--speeds", type=float, nargs="+", default=list(DEFAULT_SPEEDS_MPS))
    g.add_argument("--angles", type=float, nargs="+", default=list(DEFAULT_ANGLES_DEG))
    g.add_argument("--max-cases", type=int)
    c = sub.add_parser("check")
    c.add_argument("--case", type=Path, required=True)
    c.add_argument("--free-vram-mib", type=int)
    k = sub.add_parser("collect")
    k.add_argument("--sweep", type=Path, required=True)
    a = ap.parse_args(argv)
    try:
        if a.cmd == "generate":
            m = generate(a.out, a.bytes_per_cell, a.vram_gb, a.dx_m, a.speeds, a.angles, a.max_cases)
            print(f"wrote {len(m)} cases to {a.out}; grid {m[0]['grid']}, {m[0]['total_steps']} steps each")
        elif a.cmd == "check":
            print(check_case(a.case, a.free_vram_mib))
        else:
            print(f"wrote {collect(a.sweep)}")
    except SweepError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
