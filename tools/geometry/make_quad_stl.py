# Tier: D
"""Simplified WATERTIGHT quad STL for CFD, from the component catalog and the reference config.

Run from the repo root:  python tools/geometry/make_quad_stl.py [--out PATH]

Parts (body frame X forward, Y right, Z down, metres, as in ADR-001; centred on the body):
 - body: square plate box, 2 * frame.body_half_width_m wide, central_body_box_m[2] tall (ASSUMED sizes);
 - 4 arms: octagonal prisms (outer diameter = frame.arm_tube_outer_diameter_m) along the diagonals;
 - 4 motor cans: 16-sided cylinders, diameter 0.0418 m, height 0.0265 m (T-Motor MN3508 page, CITED), axis Z.
 Propellers are OMITTED (the discs are not solid; their effect is not in this CFD). Arms and cans are
 separate closed shells with a gap GAP_M between them and the body: no shell overlaps another, so a
 voxeliser that uses ray-crossing parity cannot be confused by overlapping volumes.
 Every shell is an index-shared closed triangle mesh; `check_watertight` verifies each edge is used
 exactly twice, once in each direction, and the signed volume is > 0 (outward normals).
All sizes are PRELIMINARY / ASSUMED where the catalog says so.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "offline_pipeline" / "components" / "catalog.json"
CONFIG = ROOT / "offline_pipeline" / "geometry" / "quad_ref_build_config.json"
DEFAULT_OUT = ROOT / "offline_pipeline" / "cfd" / "generated" / "quad_ref_build.stl"
CAN_DIAMETER_M = 0.0418   # T-Motor MN3508 page, CITED
CAN_HEIGHT_M = 0.0265     # T-Motor MN3508 page, CITED
CAN_SIDES = 16
ARM_SIDES = 8
GAP_M = 0.002             # clearance between shells (ASSUMED, smaller than any planned CFD cell is not required)


class Mesh:
    """Shared-vertex triangle mesh; every shell added is closed with CCW-from-outside triangles."""

    def __init__(self) -> None:
        self.v: list[tuple[float, float, float]] = []
        self.t: list[tuple[int, int, int]] = []

    def _add_v(self, p) -> int:
        self.v.append((float(p[0]), float(p[1]), float(p[2])))
        return len(self.v) - 1

    def add_box(self, center, size) -> None:
        cx, cy, cz = center
        hx, hy, hz = (s / 2.0 for s in size)
        ids = [self._add_v((cx + sx * hx, cy + sy * hy, cz + sz * hz))
               for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]  # index = sx + 2 sy + 4 sz (bits)
        # ids[i]: bit0 = +x, bit1 = +y, bit2 = +z
        quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
        # Each quad listed CCW seen from outside (checked by the signed-volume test).
        for a, b, c, d in quads:
            self.t.append((ids[a], ids[b], ids[c]))
            self.t.append((ids[a], ids[c], ids[d]))

    def add_prism(self, start, direction, length, radius, sides) -> None:
        """Closed n-gon prism from `start` along unit `direction` for `length` (caps are fans)."""
        d = _unit(direction)
        u, w = _basis(d)
        ring0, ring1 = [], []
        for k in range(sides):
            a = 2.0 * math.pi * k / sides
            off = tuple(radius * (math.cos(a) * u[i] + math.sin(a) * w[i]) for i in range(3))
            ring0.append(self._add_v(tuple(start[i] + off[i] for i in range(3))))
            ring1.append(self._add_v(tuple(start[i] + length * d[i] + off[i] for i in range(3))))
        c0 = self._add_v(start)
        c1 = self._add_v(tuple(start[i] + length * d[i] for i in range(3)))
        for k in range(sides):
            n = (k + 1) % sides
            # (u, w, d) is right-handed, so angle increases CCW seen from +d: ring order is CCW from +d.
            self.t.append((c0, ring0[n], ring0[k]))            # start cap, outward normal -d
            self.t.append((c1, ring1[k], ring1[n]))            # end cap, outward normal +d
            self.t.append((ring0[k], ring0[n], ring1[n]))      # side
            self.t.append((ring0[k], ring1[n], ring1[k]))


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    if n <= 0.0 or not math.isfinite(n):
        raise ValueError("zero or non-finite direction")
    return tuple(x / n for x in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _basis(d):
    """Unit u, w with u x w = d (right-handed), both perpendicular to unit d."""
    ref = (1.0, 0.0, 0.0) if abs(d[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = _unit(_cross(d, ref))
    w = _cross(d, u)
    return u, w


def signed_volume(m: Mesh) -> float:
    vol = 0.0
    for a, b, c in m.t:
        p, q, r = m.v[a], m.v[b], m.v[c]
        vol += (p[0] * (q[1] * r[2] - q[2] * r[1]) - p[1] * (q[0] * r[2] - q[2] * r[0])
                + p[2] * (q[0] * r[1] - q[1] * r[0])) / 6.0
    return vol


def check_watertight(m: Mesh) -> None:
    """Raises ValueError unless every directed edge has exactly one opposite partner and volume > 0."""
    edges: dict[tuple[int, int], int] = {}
    for a, b, c in m.t:
        for e in ((a, b), (b, c), (c, a)):
            edges[e] = edges.get(e, 0) + 1
    for (a, b), n in edges.items():
        if n != 1 or edges.get((b, a)) != 1:
            raise ValueError(f"edge {a}->{b} used {n} times, reverse {edges.get((b, a), 0)}: not watertight")
    if signed_volume(m) <= 0.0:
        raise ValueError("signed volume <= 0: normals point inward")


def bounding_box(m: Mesh):
    xs, ys, zs = zip(*m.v)
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def build_quad(cat: dict, cfg: dict) -> Mesh:
    half = cat["frame"]["body_half_width_m"]["value"]
    body_h = cat["frame"]["central_body_box_m"]["value"][2]
    arm_r = cat["frame"]["arm_tube_outer_diameter_m"]["value"] / 2.0
    m = Mesh()
    m.add_box((0.0, 0.0, 0.0), (2 * half, 2 * half, body_h))
    # The arm axis lies on the diagonal through each motor. Start beyond the body corner plus a gap,
    # stop before the motor can (radius CAN_DIAMETER_M / 2) minus a gap.
    for pos in cfg["motor_pos_m"]:
        r_motor = math.hypot(pos[0], pos[1])
        direction = (pos[0] / r_motor, pos[1] / r_motor, 0.0)
        r_start = half * math.sqrt(2.0) + arm_r + GAP_M
        r_end = r_motor - CAN_DIAMETER_M / 2.0 - GAP_M
        if r_end <= r_start:
            raise ValueError("no room for an arm between the body and the motor can")
        start = (direction[0] * r_start, direction[1] * r_start, 0.0)
        m.add_prism(start, direction, r_end - r_start, arm_r, ARM_SIDES)
        m.add_prism((pos[0], pos[1], -CAN_HEIGHT_M / 2.0), (0.0, 0.0, 1.0), CAN_HEIGHT_M, CAN_DIAMETER_M / 2.0, CAN_SIDES)
    check_watertight(m)
    return m


def to_ascii_stl(m: Mesh, name: str = "quad_ref_build") -> str:
    lines = [f"solid {name}"]
    for a, b, c in m.t:
        p, q, r = m.v[a], m.v[b], m.v[c]
        n = _cross(tuple(q[i] - p[i] for i in range(3)), tuple(r[i] - p[i] for i in range(3)))
        ln = math.sqrt(sum(x * x for x in n))
        n = tuple(x / ln for x in n) if ln > 0.0 else (0.0, 0.0, 0.0)
        lines.append(f"  facet normal {n[0]:.9e} {n[1]:.9e} {n[2]:.9e}\n    outer loop")
        for pt in (p, q, r):
            lines.append(f"      vertex {pt[0]:.9e} {pt[1]:.9e} {pt[2]:.9e}")
        lines.append("    endloop\n  endfacet")
    lines.append(f"endsolid {name}")
    return "\n".join(lines) + "\n"


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args(argv)
    m = build_quad(json.loads(CATALOG.read_text()), json.loads(CONFIG.read_text()))
    text = to_ascii_stl(m)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text)
    lo, hi = bounding_box(m)
    print(f"wrote {args.out}: {len(m.t)} triangles, volume {signed_volume(m):.6e} m^3, "
          f"bbox {lo} .. {hi}, sha256 {hashlib.sha256(text.encode()).hexdigest()}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
