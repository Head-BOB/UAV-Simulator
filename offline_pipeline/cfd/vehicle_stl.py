# Tier: D
"""Watertight simplified STL of a BUILT vehicle for CFD, from its `assembly.json` (any vehicle class).

Run from the repo root inside .venv:
  python offline_pipeline/cfd/vehicle_stl.py --build tools/parts/builds/reference_quad --out OUTDIR [--props omit|disc]

Writes `vehicle.stl` (ASCII, metres, body frame X forward / Y right / Z down, origin at the frame's centre) and `geometry.json`
(sha256 of the STL bytes, bounding box, the shells that were written, the parts that were left out and why, projected areas).

Shells (one closed triangle mesh each, none overlapping another):
  * box primitives (frame body, battery, flight controller, ESC stack, payload, camera, ...): axis-aligned boxes;
  * `tube` arms: regular octagonal prisms whose CIRCUMSCRIBED diameter is the stated outer diameter, along the arm's diagonal;
  * `plate` arms: yawed rectangular boxes (length x width x thickness) along the arm's diagonal;
  * motors: 16-sided cylinders, axis Z, centred on the part position.
  Arms are trimmed so they start outside the frame body and end before the motor can (overlapping shells would break a voxeliser
  that uses ray-crossing parity). Every shell is then shrunk by GAP_M / 2 on all sides, so two parts that touch end up GAP_M apart.
LEFT OUT by default and listed in `geometry.json`: propellers (a solid disc would act as a plate; rotor wash and propeller drag are
NOT in this CFD), wiring and anything whose visual kind is `none`, and ESC boxes that sit inside a motor can in the stand-in assembly.
`--props disc` writes a 4 mm thick disc per propeller: an experiment, never the default.
Every size comes from the build's assembly (which carries the part records' provenance); the CFD result inherits it.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools" / "geometry"))
import make_quad_stl as G  # noqa: E402  (Mesh, check_watertight, signed_volume, to_ascii_stl are reused, not edited)

GAP_M = 0.002             # clearance between touching shells (ASSUMED)
CAN_SIDES = 16
ARM_SIDES = 8             # octagonal tube
PROP_DISC_THICKNESS_M = 0.004   # only with --props disc (ASSUMED)
PROP_DISC_SIDES = 32
AREA_CELL_M = 0.001       # raster cell for projected areas
BOX_KINDS = ("box",)
OMIT_NONE = "visual kind is none (no geometry)"


class GeometryError(ValueError):
    """The assembly cannot be turned into a valid, non-overlapping watertight geometry."""


def _bbox_of(mesh: G.Mesh):
    return G.bounding_box(mesh)


def oriented_box(center, size, yaw_rad: float) -> G.Mesh:
    """Closed box of `size` (along its own x, y, z) centred at `center`, rotated by `yaw_rad` about Z (right-handed, z down)."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    ex, ey = (c, s, 0.0), (-s, c, 0.0)
    ez = (0.0, 0.0, 1.0)
    hx, hy, hz = (v / 2.0 for v in size)
    m = G.Mesh()
    ids = []
    for sz in (-1, 1):
        for sy in (-1, 1):
            for sx in (-1, 1):
                p = tuple(center[i] + sx * hx * ex[i] + sy * hy * ey[i] + sz * hz * ez[i] for i in range(3))
                ids.append(m._add_v(p))
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]   # same table as Mesh.add_box
    for a, b, c2, d in quads:
        m.t.append((ids[a], ids[b], ids[c2]))
        m.t.append((ids[a], ids[c2], ids[d]))
    return m


def cylinder(center, diameter: float, height: float, sides: int) -> G.Mesh:
    """Closed `sides`-gon prism with axis Z, centred at `center`; the ring vertices lie on the circle of the stated diameter."""
    m = G.Mesh()
    m.add_prism((center[0], center[1], center[2] - height / 2.0), (0.0, 0.0, 1.0), height, diameter / 2.0, sides)
    return m


def merge(meshes: list) -> G.Mesh:
    out = G.Mesh()
    for m in meshes:
        base = len(out.v)
        out.v.extend(m.v)
        out.t.extend((a + base, b + base, c + base) for a, b, c in m.t)
    return out


def box_volume(size) -> float:
    return size[0] * size[1] * size[2]


def prism_volume(radius: float, sides: int, length: float) -> float:
    """Regular n-gon prism with circumradius `radius`: area (n/2) R^2 sin(2 pi / n) times the length."""
    return 0.5 * sides * radius * radius * math.sin(2.0 * math.pi / sides) * length


def arm_interval(direction, body_half_xy, arm_half_width: float, motor_radius: float, can_radius: float):
    """Radial start / end of an arm along its unit horizontal `direction`: outside the body box, before the motor can."""
    dx, dy = abs(direction[0]), abs(direction[1])
    start = body_half_xy[0] * dx + body_half_xy[1] * dy + arm_half_width * (dx + dy) + GAP_M
    end = motor_radius - can_radius - GAP_M
    if end <= start:
        raise GeometryError(f"no room for an arm between the body and the motor can (start {start:.4f} m, end {end:.4f} m)")
    return start, end


def _shrunk_box(center, size, yaw=0.0) -> G.Mesh:
    return oriented_box(center, tuple(max(s - GAP_M, 1e-6) for s in size), yaw)


def _aabb_overlap(a, b) -> bool:
    (alo, ahi), (blo, bhi) = a, b
    return all(alo[i] < bhi[i] and blo[i] < ahi[i] for i in range(3))


def build_geometry(assembly: dict, props: str = "omit") -> dict:
    """Returns {'mesh', 'shells': [...], 'omitted': [...]}. Raises GeometryError on an unusable or overlapping assembly."""
    if props not in ("omit", "disc"):
        raise GeometryError("props must be 'omit' or 'disc'")
    parts = {p["instance"]: p for p in assembly["parts"]}
    body = next((p for p in assembly["parts"] if p["instance"] == "body" and p["visual"]["kind"] == "box"), None)
    if body is None:
        raise GeometryError("the assembly has no frame body box")
    body_half = (body["visual"]["dims_m"][0] / 2.0, body["visual"]["dims_m"][1] / 2.0)
    motors = [p for p in assembly["parts"] if p["role"] == "motor"]
    shells, omitted, solid = [], [], []

    def add(part, kind, mesh, volume, arm=False):
        G.check_watertight(mesh)
        solid.append((part["instance"], mesh, arm))
        shells.append({"instance": part["instance"], "role": part["role"], "part_id": part["part_id"], "kind": kind,
                       "triangles": len(mesh.t), "volume_m3": G.signed_volume(mesh), "expected_volume_m3": volume,
                       "bbox_m": [list(b) for b in _bbox_of(mesh)]})

    for p in assembly["parts"]:
        v, pos, role = p["visual"], p["position_m"], p["role"]
        if v["kind"] == "none":
            omitted.append({"instance": p["instance"], "role": role, "reason": OMIT_NONE})
        elif role == "propeller":
            if props == "omit":
                omitted.append({"instance": p["instance"], "role": role, "reason": "propeller omitted: a solid disc would act as a plate; rotor wash and propeller drag are not in this CFD"})
            else:
                d, t = v["diameter_m"], PROP_DISC_THICKNESS_M
                m = cylinder(pos, d - GAP_M, t - GAP_M, PROP_DISC_SIDES)
                add(p, "disc", m, prism_volume((d - GAP_M) / 2.0, PROP_DISC_SIDES, t - GAP_M))
        elif role == "motor":
            d, h = v["diameter_m"], v["height_m"]
            m = cylinder(pos, d - GAP_M, h - GAP_M, CAN_SIDES)
            add(p, "cylinder", m, prism_volume((d - GAP_M) / 2.0, CAN_SIDES, h - GAP_M))
        elif role == "esc" and v["kind"] == "box" and any(math.hypot(pos[0] - mo["position_m"][0], pos[1] - mo["position_m"][1]) < mo["visual"]["diameter_m"] / 2.0 for mo in motors):
            omitted.append({"instance": p["instance"], "role": role, "reason": "ESC sits inside the motor can in the stand-in assembly"})
        elif v["kind"] == "box":
            size = tuple(v["dims_m"])
            add(p, "box", _shrunk_box(pos, size), box_volume(tuple(max(s - GAP_M, 1e-6) for s in size)))
        elif v["kind"] in ("tube", "plate"):
            _add_arm(p, v, motors, body_half, add)
        else:
            raise GeometryError(f"part {p['instance']}: unknown visual kind {v['kind']!r}")
    mesh = merge([m for _, m, _ in solid])
    _check_no_overlap(solid)
    G.check_watertight(mesh)
    return {"mesh": mesh, "shells": shells, "omitted": omitted}


def _add_arm(p, v, motors, body_half, add) -> None:
    pos = p["position_m"]
    r_arm = math.hypot(pos[0], pos[1])
    if r_arm == 0.0:
        raise GeometryError(f"arm {p['instance']} sits at the centre: its direction is undefined")
    direction = (pos[0] / r_arm, pos[1] / r_arm)
    motor = min(motors, key=lambda mo: math.hypot(mo["position_m"][0] - pos[0], mo["position_m"][1] - pos[1]))
    mp = motor["position_m"]
    r_motor = math.hypot(mp[0], mp[1])
    cross = abs(mp[0] * direction[1] - mp[1] * direction[0])
    if cross > 1e-3:
        raise GeometryError(f"arm {p['instance']} is not on the ray through its motor")
    can_r = motor["visual"]["diameter_m"] / 2.0
    width = v["od_m"] if v["kind"] == "tube" else v["width_m"]
    start, end = arm_interval(direction, body_half, width / 2.0, r_motor, can_r)
    length = end - start
    if v["kind"] == "tube":
        radius = v["od_m"] / 2.0 - GAP_M / 2.0
        m = G.Mesh()
        m.add_prism((direction[0] * start, direction[1] * start, pos[2]), (direction[0], direction[1], 0.0), length, radius, ARM_SIDES)
        add(p, "tube", m, prism_volume(radius, ARM_SIDES, length), arm=True)
    else:
        centre = (direction[0] * (start + end) / 2.0, direction[1] * (start + end) / 2.0, pos[2])
        size = (length - GAP_M, v["width_m"] - GAP_M, v["thickness_m"] - GAP_M)
        yaw = math.atan2(direction[1], direction[0])
        add(p, "plate", oriented_box(centre, size, yaw), box_volume(size), arm=True)


def _check_no_overlap(solid: list) -> None:
    """Non-arm shells must not overlap each other (AABB test); arms are trimmed by construction."""
    rigid = [(n, _bbox_of(m)) for n, m, arm in solid if not arm]
    for i in range(len(rigid)):
        for j in range(i + 1, len(rigid)):
            if _aabb_overlap(rigid[i][1], rigid[j][1]):
                raise GeometryError(f"shells {rigid[i][0]} and {rigid[j][0]} overlap")


def projected_area_m2(mesh: G.Mesh, axis: int, cell: float = AREA_CELL_M) -> float:
    """Area of the union of the mesh's triangles projected along `axis` (0 = X, 1 = Y, 2 = Z), by raster sampling at `cell`."""
    import numpy as np
    a, b = [i for i in range(3) if i != axis]
    pts = np.array(mesh.v)[:, [a, b]]
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    nx, ny = int(math.ceil((hi[0] - lo[0]) / cell)), int(math.ceil((hi[1] - lo[1]) / cell))
    covered = np.zeros((nx, ny), dtype=bool)
    xs = lo[0] + (np.arange(nx) + 0.5) * cell
    ys = lo[1] + (np.arange(ny) + 0.5) * cell
    for t in mesh.t:
        p0, p1, p2 = (pts[i] for i in t)
        tri_min, tri_max = np.minimum(np.minimum(p0, p1), p2), np.maximum(np.maximum(p0, p1), p2)
        i0, i1 = max(int((tri_min[0] - lo[0]) / cell) - 1, 0), min(int((tri_max[0] - lo[0]) / cell) + 2, nx)
        j0, j1 = max(int((tri_min[1] - lo[1]) / cell) - 1, 0), min(int((tri_max[1] - lo[1]) / cell) + 2, ny)
        if i0 >= i1 or j0 >= j1:
            continue
        gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing="ij")
        den = (p1[1] - p2[1]) * (p0[0] - p2[0]) + (p2[0] - p1[0]) * (p0[1] - p2[1])
        if abs(den) < 1e-18:
            continue
        w0 = ((p1[1] - p2[1]) * (gx - p2[0]) + (p2[0] - p1[0]) * (gy - p2[1])) / den
        w1 = ((p2[1] - p0[1]) * (gx - p2[0]) + (p0[0] - p2[0]) * (gy - p2[1])) / den
        inside = (w0 >= 0) & (w1 >= 0) & (w0 + w1 <= 1)
        covered[i0:i1, j0:j1] |= inside
    return float(covered.sum()) * cell * cell


def geometry_document(geo: dict, assembly: dict, stl_text: str, props: str) -> dict:
    mesh = geo["mesh"]
    lo, hi = _bbox_of(mesh)
    return {
        "schema_version": 1,
        "generated_by": "offline_pipeline/cfd/vehicle_stl.py",
        "build_name": assembly["build_name"],
        "vehicle_class": assembly["vehicle_class"],
        "frame_convention": assembly["frame_convention"],
        "stl_sha256": hashlib.sha256(stl_text.encode("utf-8")).hexdigest(),
        "triangles": len(mesh.t),
        "volume_m3": G.signed_volume(mesh),
        "bbox_min_m": list(lo),
        "bbox_max_m": list(hi),
        "projected_area_m2": {"along_x": projected_area_m2(mesh, 0), "along_y": projected_area_m2(mesh, 1), "along_z": projected_area_m2(mesh, 2)},
        "cg_from_frame_centre_m": assembly["totals"]["cg_from_frame_centre_m"],
        "propellers": props,
        "gap_m": GAP_M,
        "shells": geo["shells"],
        "omitted": geo["omitted"],
        "provenance": "simplified dimension-true stand-in geometry from the build's assembly.json; propellers omitted unless stated; PRELIMINARY",
    }


def main(argv: list) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--build", type=Path, required=True, help="build directory holding assembly.json")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--props", choices=("omit", "disc"), default="omit")
    a = ap.parse_args(argv)
    assembly = json.loads((a.build / "assembly.json").read_text(encoding="utf-8"))
    try:
        geo = build_geometry(assembly, a.props)
    except GeometryError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    text = G.to_ascii_stl(geo["mesh"], assembly["build_name"])
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "vehicle.stl").write_text(text, encoding="utf-8", newline="\n")
    doc = geometry_document(geo, assembly, text, a.props)
    (a.out / "geometry.json").write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {a.out}/vehicle.stl ({doc['triangles']} triangles, {len(geo['shells'])} shells, {len(geo['omitted'])} parts left out), sha256 {doc['stl_sha256']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
