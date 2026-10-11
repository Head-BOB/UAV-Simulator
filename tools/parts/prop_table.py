# Tier: D
"""Propeller Ct/Cp(J, rpm) table for the core, built from the UIUC data file of a propeller (`offline_pipeline/props/*.json`).

Build (approved 2026-10-10):
  1. every wind-tunnel run has a nominal rpm (last number of its file name, e.g. `..._0630od_3040.txt` -> 3040 rpm);
  2. runs whose nominal rpm lie within `GROUP_TOLERANCE` of the group's lowest rpm are the same speed measured over different J ranges;
     they are merged into one curve at the mean rpm, blended linearly over the J overlap (weight 0 -> lower-J run, 1 -> higher-J run);
  3. the static Ct/Cp at the curve's rpm (linear in rpm between the static rows) is added as the J = 0 point.
Evaluation (`evaluate`, the Python reference of `physics_core::powertrain::coefficients`): linear in J inside a curve, linear in rpm between the two
bracketing curves; outside the data the nearest edge value is HELD and the point is flagged extrapolated (J < 0, J beyond the last point of either
bracketing curve, rpm outside the first/last curve). Nothing is extrapolated along a slope.

Limits shared with the core: at most 8 curves, 48 points per curve, J strictly increasing from exactly 0, Cp > 0 everywhere.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path

MAX_CURVES = 8                  # must equal physics_core types::MAX_PROP_CURVES
MAX_POINTS = 48                 # must equal physics_core types::MAX_PROP_POINTS
GROUP_TOLERANCE = 0.02          # runs within 2 % of the group's lowest rpm are one speed. ASSUMED, TODO(HUMAN-VERIFY)


class PropTableError(ValueError):
    """The data file cannot be turned into a valid table."""


def parse_runs(prop_json: dict) -> list[dict]:
    """Wind-tunnel runs as {name, rpm, points[(J, Ct, Cp)]} sorted by rpm then by first J."""
    runs = []
    for name, rows in prop_json["wind_tunnel"].items():
        m = re.search(r"_(\d+)\.txt$", name)
        if not m:
            raise PropTableError(f"cannot read the nominal rpm from the run name {name!r}")
        # The UIUC files repeat their last row several times (identical J, Ct, Cp): exact duplicates carry no information and are
        # dropped. Rows with the same J but different values would be ambiguous and are refused.
        pts = sorted({(float(r["j"]), float(r["ct"]), float(r["cp"])) for r in rows})
        if len(pts) < 2:
            raise PropTableError(f"{name}: fewer than 2 distinct points")
        if any(b[0] <= a[0] for a, b in zip(pts, pts[1:])):
            raise PropTableError(f"{name}: the same J appears with different Ct or Cp")
        runs.append({"name": name, "rpm": float(m.group(1)), "points": pts})
    return sorted(runs, key=lambda r: (r["rpm"], r["points"][0][0]))


def static_at(static_rows: list[dict], rpm: float) -> tuple[float, float]:
    """Static (J = 0) Ct and Cp at `rpm`, linear between the static rows. Raises outside the measured static range."""
    rows = sorted(static_rows, key=lambda r: r["rpm"])
    if not rows[0]["rpm"] <= rpm <= rows[-1]["rpm"]:
        raise PropTableError(f"static data does not cover {rpm} rpm")
    for a, b in zip(rows, rows[1:]):
        if rpm <= b["rpm"]:
            w = (rpm - a["rpm"]) / (b["rpm"] - a["rpm"])
            return a["ct"] + w * (b["ct"] - a["ct"]), a["cp"] + w * (b["cp"] - a["cp"])
    raise PropTableError("unreachable")


def _interp(points: list[tuple[float, float, float]], j: float) -> tuple[float, float]:
    """Piecewise-linear (Ct, Cp) at j inside the points' range (j must lie within it)."""
    for a, b in zip(points, points[1:]):
        if j <= b[0]:
            w = (j - a[0]) / (b[0] - a[0])
            return a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2])
    return points[-1][1], points[-1][2]


def blend(low: list[tuple], high: list[tuple]) -> list[tuple]:
    """Merge two runs of the same speed. `low` covers the lower J range. Over the overlap [high.J0, low.J_end] the value is
    (1 - w) low(J) + w high(J) with w = (J - high.J0) / (low.J_end - high.J0); outside it each run is used unchanged."""
    b0, a1 = high[0][0], low[-1][0]
    if b0 >= a1:                        # no overlap: just join them
        return sorted(set(low) | set(high))
    out = [p for p in low if p[0] < b0]
    grid = sorted({p[0] for p in low if b0 <= p[0] <= a1} | {p[0] for p in high if b0 <= p[0] <= a1})
    for j in grid:
        w = (j - b0) / (a1 - b0)
        ct_a, cp_a = _interp(low, j)
        ct_b, cp_b = _interp(high, j)
        out.append((j, (1 - w) * ct_a + w * ct_b, (1 - w) * cp_a + w * cp_b))
    out += [p for p in high if p[0] > a1]
    return out


def group_runs(runs: list[dict]) -> list[list[dict]]:
    groups: list[list[dict]] = []
    for r in runs:
        if groups and r["rpm"] <= groups[-1][0]["rpm"] * (1.0 + GROUP_TOLERANCE):
            groups[-1].append(r)
        else:
            groups.append([r])
    return groups


def build_curves(prop_json: dict) -> list[dict]:
    """The core's table: [{rpm, points: [[J, Ct, Cp], ...]}], J strictly increasing from 0."""
    curves = []
    for group in group_runs(parse_runs(prop_json)):
        group = sorted(group, key=lambda r: r["points"][0][0])
        pts = group[0]["points"]
        for nxt in group[1:]:
            pts = blend(pts, nxt["points"])
        rpm = sum(r["rpm"] for r in group) / len(group)
        ct0, cp0 = static_at(prop_json["static"], rpm)
        if pts[0][0] > 0.0:
            pts = [(0.0, ct0, cp0)] + pts
        curves.append({"rpm": rpm, "points": [list(p) for p in pts]})
    validate_curves(curves)
    return curves


def validate_curves(curves: list[dict]) -> None:
    if not 1 <= len(curves) <= MAX_CURVES:
        raise PropTableError(f"{len(curves)} curves; the core holds 1..{MAX_CURVES}")
    rpms = [c["rpm"] for c in curves]
    if any(b <= a for a, b in zip(rpms, rpms[1:])) or rpms[0] <= 0:
        raise PropTableError("curve rpm must be positive and strictly increasing")
    for c in curves:
        pts = c["points"]
        if not 2 <= len(pts) <= MAX_POINTS:
            raise PropTableError(f"curve {c['rpm']}: {len(pts)} points; the core holds 2..{MAX_POINTS}")
        js = [p[0] for p in pts]
        if js[0] != 0.0 or any(b <= a for a, b in zip(js, js[1:])):
            raise PropTableError(f"curve {c['rpm']}: J must start at exactly 0 and strictly increase")
        if not all(math.isfinite(v) for p in pts for v in p) or any(p[2] <= 0.0 for p in pts):
            raise PropTableError(f"curve {c['rpm']}: non-finite value or Cp <= 0")


def _eval_curve(points: list, j: float) -> tuple[float, float, bool]:
    """(Ct, Cp, beyond_end) at j with j clamped to [0, J_end]. beyond_end is True if j exceeded the curve's last point."""
    j_end = points[-1][0]
    jc = min(max(j, 0.0), j_end)
    for a, b in zip(points, points[1:]):
        if jc <= b[0]:
            w = (jc - a[0]) / (b[0] - a[0])
            return a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), j > j_end
    return points[-1][1], points[-1][2], j > j_end


def evaluate(curves: list[dict], j: float, rpm: float) -> tuple[float, float, bool]:
    """Python reference of the core: (Ct, Cp, extrapolated). Same clamps, same flags."""
    rpms = [c["rpm"] for c in curves]
    flagged = j < 0.0 or rpm < rpms[0] or rpm > rpms[-1]
    r = min(max(rpm, rpms[0]), rpms[-1])
    if len(curves) == 1:
        ct, cp, beyond = _eval_curve(curves[0]["points"], j)
        return ct, cp, flagged or beyond
    i = max(k for k in range(len(curves)) if rpms[k] <= r) if r < rpms[-1] else len(curves) - 2
    i = min(i, len(curves) - 2)
    ct_a, cp_a, beyond_a = _eval_curve(curves[i]["points"], j)
    ct_b, cp_b, beyond_b = _eval_curve(curves[i + 1]["points"], j)
    t = (r - rpms[i]) / (rpms[i + 1] - rpms[i])
    return ct_a + t * (ct_b - ct_a), cp_a + t * (cp_b - cp_a), flagged or beyond_a or beyond_b


def provenance(prop_json_path: Path, curves: list[dict], repo_root: Path | None = None) -> dict:
    """Where the table came from and what range it measures. The path is stored relative to `repo_root` (goldens must not hold machine paths)."""
    data = prop_json_path.read_bytes()
    return {
        "source_file": str(prop_json_path.relative_to(repo_root) if repo_root else prop_json_path), "source_sha256": hashlib.sha256(data).hexdigest(),
        "curves": len(curves), "points_per_curve": [len(c["points"]) for c in curves],
        "measured_rpm_range": [curves[0]["rpm"], curves[-1]["rpm"]],
        "measured_j_end_per_curve": [c["points"][-1][0] for c in curves],
        "grouping_tolerance": GROUP_TOLERANCE,
        "method": "runs within 2 % of rpm merged with a linear blend over the J overlap; static Ct/Cp added as the J = 0 point; nominal rpm from the file names",
    }
