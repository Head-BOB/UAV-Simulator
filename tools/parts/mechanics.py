# Tier: D
"""Rigid-body mass properties for the vehicle builder: inertia tensors, parallel axis, CG, beam sections.

Frame: body X forward, Y right, Z DOWN (ADR-001). Units SI. Tensors are 3x3 lists of lists,
I = [[Ixx, -Ixy, -Ixz], [-Ixy, Iyy, -Iyz], [-Ixz, -Iyz, Izz]] (products of inertia enter with a minus sign).
Pure functions, no I/O, no randomness. Values must be finite; bad input raises ValueError.

Simplifications (documented, flagged in the build sidecar):
 - every part is a solid of uniform density in the shape named by the layout (box, cylinder, thin disc, thin rod);
 - a part's bounding box is used as its envelope, no rotation.
"""
from __future__ import annotations

import math

Vec = tuple[float, float, float]
Tensor = list[list[float]]


def _finite(*values: float) -> None:
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite value in mass properties")


def zeros() -> Tensor:
    return [[0.0] * 3 for _ in range(3)]


def add(a: Tensor, b: Tensor) -> Tensor:
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def diag(ixx: float, iyy: float, izz: float) -> Tensor:
    t = zeros()
    t[0][0], t[1][1], t[2][2] = ixx, iyy, izz
    return t


def box_inertia(m_kg: float, dims_m: Vec) -> Tensor:
    """Solid box about its own CG: Ixx = m (b^2 + c^2) / 12 etc. for edge lengths (a, b, c) along (x, y, z)."""
    _finite(m_kg, *dims_m)
    if m_kg <= 0 or min(dims_m) < 0:
        raise ValueError("box needs m > 0 and non-negative edges")
    a, b, c = dims_m
    return diag(m_kg * (b * b + c * c) / 12.0, m_kg * (a * a + c * c) / 12.0, m_kg * (a * a + b * b) / 12.0)


def cylinder_inertia(m_kg: float, diameter_m: float, height_m: float) -> Tensor:
    """Solid cylinder, axis along z: Izz = m r^2 / 2; Ixx = Iyy = m (3 r^2 + h^2) / 12."""
    _finite(m_kg, diameter_m, height_m)
    if m_kg <= 0 or diameter_m < 0 or height_m < 0:
        raise ValueError("cylinder needs m > 0 and non-negative size")
    r2 = (diameter_m / 2.0) ** 2
    return diag(m_kg * (3.0 * r2 + height_m ** 2) / 12.0, m_kg * (3.0 * r2 + height_m ** 2) / 12.0, m_kg * r2 / 2.0)


def disc_inertia(m_kg: float, diameter_m: float) -> Tensor:
    """Thin disc in the x-y plane (a propeller): Izz = m r^2 / 2; Ixx = Iyy = m r^2 / 4 (= m D^2 / 16)."""
    _finite(m_kg, diameter_m)
    if m_kg <= 0 or diameter_m < 0:
        raise ValueError("disc needs m > 0 and non-negative diameter")
    r2 = (diameter_m / 2.0) ** 2
    return diag(m_kg * r2 / 4.0, m_kg * r2 / 4.0, m_kg * r2 / 2.0)


def rod_inertia(m_kg: float, length_m: float, direction: Vec) -> Tensor:
    """Thin rod about its CG with unit-length `direction`: I = m L^2 / 12 (E - d d^T)."""
    _finite(m_kg, length_m, *direction)
    n = math.sqrt(sum(c * c for c in direction))
    if m_kg <= 0 or length_m < 0 or n < 1e-12:
        raise ValueError("rod needs m > 0, non-negative length and a non-zero direction")
    d = [c / n for c in direction]
    k = m_kg * length_m * length_m / 12.0
    return [[k * ((1.0 if i == j else 0.0) - d[i] * d[j]) for j in range(3)] for i in range(3)]


def parallel_axis(i_cm: Tensor, m_kg: float, r_m: Vec) -> Tensor:
    """Tensor about a point offset by `r` from the part CG: I = I_cm + m (|r|^2 E - r r^T)."""
    _finite(m_kg, *r_m)
    r2 = sum(c * c for c in r_m)
    return [[i_cm[i][j] + m_kg * ((r2 if i == j else 0.0) - r_m[i] * r_m[j]) for j in range(3)] for i in range(3)]


class Body:
    """One mass element: mass, CG position in the body frame, tensor about its own CG."""

    __slots__ = ("name", "mass_kg", "position_m", "tensor")

    def __init__(self, name: str, mass_kg: float, position_m: Vec, tensor: Tensor) -> None:
        _finite(mass_kg, *position_m)
        if mass_kg <= 0:
            raise ValueError(f"{name}: mass must be > 0")
        self.name, self.mass_kg, self.position_m, self.tensor = name, mass_kg, tuple(position_m), tensor


def combine(bodies: list[Body]) -> tuple[float, Vec, Tensor]:
    """Total mass, CG, and the tensor about the combined CG (parallel axis for every body)."""
    if not bodies:
        raise ValueError("no bodies")
    total = sum(b.mass_kg for b in bodies)
    cg = tuple(sum(b.mass_kg * b.position_m[k] for b in bodies) / total for k in range(3))
    tensor = zeros()
    for b in bodies:
        r = tuple(b.position_m[k] - cg[k] for k in range(3))
        tensor = add(tensor, parallel_axis(b.tensor, b.mass_kg, r))
    return total, cg, tensor  # type: ignore[return-value]


def tube_section(od_m: float, id_m: float) -> dict:
    """Hollow round tube: A = pi/4 (od^2 - id^2), I = pi/64 (od^4 - id^4), Z = I / (od/2)."""
    _finite(od_m, id_m)
    if not (0 <= id_m < od_m):
        raise ValueError("tube needs 0 <= id < od")
    a = math.pi / 4.0 * (od_m ** 2 - id_m ** 2)
    i = math.pi / 64.0 * (od_m ** 4 - id_m ** 4)
    return {"area_m2": a, "second_moment_m4": i, "section_modulus_m3": i / (od_m / 2.0)}


def plate_section(width_m: float, thickness_m: float) -> dict:
    """Flat arm bending in its thickness direction (thrust loads the plate flat-wise): I = w t^3 / 12, Z = I / (t/2)."""
    _finite(width_m, thickness_m)
    if width_m <= 0 or thickness_m <= 0:
        raise ValueError("plate needs positive width and thickness")
    i = width_m * thickness_m ** 3 / 12.0
    return {"area_m2": width_m * thickness_m, "second_moment_m4": i, "section_modulus_m3": i / (thickness_m / 2.0)}


def cantilever_bending_stress_pa(force_n: float, arm_length_m: float, section_modulus_m3: float) -> float:
    """Tip load on a cantilever arm: M = F L, sigma = M / Z."""
    _finite(force_n, arm_length_m, section_modulus_m3)
    if section_modulus_m3 <= 0 or arm_length_m < 0 or force_n < 0:
        raise ValueError("bad cantilever input")
    return force_n * arm_length_m / section_modulus_m3


def projected_areas_m2(boxes_m: list[Vec]) -> dict:
    """Conservative sum of bounding-box faces (overlap ignored): top = a*b, front = b*c (looking along x), side = a*c."""
    for b in boxes_m:
        _finite(*b)
    return {"top": sum(b[0] * b[1] for b in boxes_m), "front": sum(b[1] * b[2] for b in boxes_m),
            "side": sum(b[0] * b[2] for b in boxes_m)}
