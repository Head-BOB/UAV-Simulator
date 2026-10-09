# Tier: D
"""Case-set runner for the offline CFD pipeline (Completion Plan WP-6.1 to WP-6.3).

Usage: python offline_pipeline/run_case.py --set <name> [--cases-root DIR]

Layout: <cases-root>/<set>/<case_id>/case.json, the geometry file named in case.json,
and results/results.json (written here). Every case in the set is validated, and its
grid is checked against VRAM, BEFORE any case runs, so a bad case 5 does not waste the
GPU hours of cases 1 to 4.

Units: SI (rpm in revolutions per minute, thrust in N, torque in N*m, time in s).

TODO(HUMAN-VERIFY): bytes_per_cell for FluidX3D and the real solver launcher and its
output format are not known here (B-7). The real solver entry below raises on purpose.
TODO(HUMAN-VERIFY): the case.json schema is this script's own design, not from a standard.
"""
import argparse
import hashlib
import json
import math
import os
import re
import sys
import time
from pathlib import Path
from typing import Callable

# Written into every case.json so the rule that was applied is on record and cannot drift.
VRAM_FORMULA = "nx*ny*nz*bytes_per_cell <= vram_bytes*(1-headroom_fraction)"

_TOP_KEYS = {
    "case_id", "kind", "geometry_file", "geometry_hash", "rpm", "air", "solver",
    "grid", "memory", "vram_formula",
}
_AIR_KEYS = {"density_kg_m3", "kinematic_viscosity_m2_s"}
_SOLVER_KEYS = {"name", "version"}
_GRID_KEYS = {"nx", "ny", "nz"}
_MEMORY_KEYS = {"bytes_per_cell", "vram_bytes", "headroom_fraction"}
_SET_NAME = re.compile(r"[A-Za-z0-9_-]+")


class CaseError(Exception):
    """A case.json is missing, malformed, or does not fit the GPU."""


def _keys(obj: object, expected: set[str], where: str) -> dict:
    if not isinstance(obj, dict):
        raise CaseError(f"{where}: must be an object")
    missing = expected - obj.keys()
    extra = obj.keys() - expected
    if missing or extra:
        raise CaseError(f"{where}: missing {sorted(missing)}, unknown {sorted(extra)}")
    return obj


def _pos_float(value: object, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CaseError(f"{where}: must be a number")
    if not math.isfinite(value) or value <= 0:
        raise CaseError(f"{where}: must be finite and > 0, got {value}")
    return float(value)


def _pos_int(value: object, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise CaseError(f"{where}: must be an int >= 1, got {value!r}")
    return value


def _nonempty_str(value: object, where: str) -> str:
    if not isinstance(value, str) or not value:
        raise CaseError(f"{where}: must be a non-empty string")
    return value


def vram_check(grid: dict, memory: dict) -> tuple[int, float, bool]:
    """Return (required_bytes, limit_bytes, fits) per VRAM_FORMULA.

    Exact integer arithmetic for the requirement; the limit is vram*(1-h) in float.
    """
    required = grid["nx"] * grid["ny"] * grid["nz"] * memory["bytes_per_cell"]
    limit = memory["vram_bytes"] * (1.0 - memory["headroom_fraction"])
    return required, limit, required <= limit


def load_case(case_dir: Path) -> dict:
    """Read and fully validate <case_dir>/case.json. Raises CaseError."""
    path = case_dir / "case.json"
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise CaseError(f"{path}: cannot read: {exc}") from exc
    case = _keys(raw, _TOP_KEYS, "case.json")
    if case["case_id"] != case_dir.name:
        raise CaseError(f"case_id {case['case_id']!r} != folder {case_dir.name!r}")
    if case["kind"] != "cfd":
        raise CaseError(f"kind {case['kind']!r} not implemented (only 'cfd')")
    _pos_float(case["rpm"], "rpm")
    air = _keys(case["air"], _AIR_KEYS, "air")
    for key in _AIR_KEYS:
        _pos_float(air[key], f"air.{key}")
    solver = _keys(case["solver"], _SOLVER_KEYS, "solver")
    for key in _SOLVER_KEYS:
        _nonempty_str(solver[key], f"solver.{key}")
    grid = _keys(case["grid"], _GRID_KEYS, "grid")
    for key in _GRID_KEYS:
        _pos_int(grid[key], f"grid.{key}")
    mem = _keys(case["memory"], _MEMORY_KEYS, "memory")
    _pos_int(mem["bytes_per_cell"], "memory.bytes_per_cell")
    _pos_int(mem["vram_bytes"], "memory.vram_bytes")
    headroom = _pos_float(mem["headroom_fraction"], "memory.headroom_fraction")
    if headroom >= 1.0:
        raise CaseError(f"memory.headroom_fraction must be < 1, got {headroom}")
    if case["vram_formula"] != VRAM_FORMULA:
        raise CaseError("vram_formula differs from the formula this script applies")
    _check_geometry(case_dir, case)
    _, _, fits = vram_check(grid, mem)
    if not fits:
        raise CaseError(f"case {case['case_id']}: grid does not fit in VRAM with headroom")
    return case


def _check_geometry(case_dir: Path, case: dict) -> None:
    name = _nonempty_str(case["geometry_file"], "geometry_file")
    if Path(name).name != name:
        raise CaseError(f"geometry_file must be a plain file name, got {name!r}")
    try:
        data = (case_dir / name).read_bytes()
    except OSError as exc:
        raise CaseError(f"cannot read geometry {name}: {exc}") from exc
    actual = hashlib.sha256(data).hexdigest()
    if actual != case["geometry_hash"]:
        raise CaseError(f"geometry_hash mismatch for {name}: file is {actual}")


def _fluidx3d_not_configured(case: dict, case_dir: Path) -> tuple[float, float]:
    raise CaseError("FluidX3D launcher not configured (TODO(HUMAN-VERIFY), B-7)")


SOLVERS: dict[str, Callable[[dict, Path], tuple[float, float]]] = {
    "fluidx3d": _fluidx3d_not_configured,
}


def _write_results(case_dir: Path, case: dict, thrust_n: float, torque_nm: float,
                   elapsed_s: float) -> None:
    required, limit, _ = vram_check(case["grid"], case["memory"])
    grid = case["grid"]
    result = {
        "case_id": case["case_id"],
        "kind": case["kind"],
        "geometry_hash": case["geometry_hash"],
        "rpm": float(case["rpm"]),
        "grid": grid,
        "cells": grid["nx"] * grid["ny"] * grid["nz"],
        "vram_required_bytes": required,
        "vram_limit_bytes": limit,
        "vram_formula": case["vram_formula"],
        "solver": case["solver"],
        "air": case["air"],
        "thrust_n": thrust_n,
        "torque_nm": torque_nm,
        "elapsed_s": elapsed_s,
        "status": "ok",
    }
    out_dir = case_dir / "results"
    out_dir.mkdir(exist_ok=True)
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    (out_dir / "results.json").write_bytes(text.encode("utf-8"))


def run_set(set_dir: Path, solvers: dict, summary_path: Path | None,
            clock: Callable[[], float] = time.monotonic) -> int:
    """Validate every case first, then run them. Returns a process exit code."""
    if not set_dir.is_dir():
        print(f"error: no such case set: {set_dir}", file=sys.stderr)
        return 1
    case_dirs = sorted(p for p in set_dir.iterdir() if p.is_dir())
    if not case_dirs:
        print(f"error: case set {set_dir} is empty", file=sys.stderr)
        return 1
    cases = []
    try:
        for case_dir in case_dirs:
            cases.append((case_dir, load_case(case_dir)))
    except CaseError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    for case_dir, case in cases:
        if run_one(case_dir, case, solvers, summary_path, clock) != 0:
            return 1
    return 0


def run_one(case_dir: Path, case: dict, solvers: dict, summary_path: Path | None,
            clock: Callable[[], float]) -> int:
    solver = solvers.get(case["solver"]["name"])
    if solver is None:
        print(f"error: unknown solver {case['solver']['name']!r}", file=sys.stderr)
        return 1
    start = clock()
    try:
        thrust_n, torque_nm = solver(case, case_dir)
    except CaseError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    elapsed_s = clock() - start
    if not all(isinstance(v, (int, float)) and math.isfinite(v)
               for v in (thrust_n, torque_nm)):
        print(f"error: case {case['case_id']}: solver returned non-finite output",
              file=sys.stderr)
        return 1
    _write_results(case_dir, case, float(thrust_n), float(torque_nm), elapsed_s)
    line = (f"case={case['case_id']} rpm={float(case['rpm'])} thrust_n={float(thrust_n)} "
            f"torque_nm={float(torque_nm)} elapsed_s={elapsed_s}")
    print(line)
    if summary_path is not None:
        with open(summary_path, "a", encoding="utf-8", newline="\n") as fh:
            fh.write(line + "\n")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--set", dest="set_name", required=True)
    default_root = Path(__file__).resolve().parent / "cases"
    parser.add_argument("--cases-root", type=Path, default=default_root)
    args = parser.parse_args(argv)
    if not _SET_NAME.fullmatch(args.set_name):
        print(f"error: bad set name {args.set_name!r}", file=sys.stderr)
        return 1
    env_summary = os.environ.get("GITHUB_STEP_SUMMARY")
    summary = Path(env_summary) if env_summary else None
    return run_set(args.cases_root / args.set_name, SOLVERS, summary)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
