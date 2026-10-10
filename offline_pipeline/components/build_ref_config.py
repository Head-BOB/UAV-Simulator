# Tier: D
"""Builds offline_pipeline/geometry/quad_ref_build_config.json from the component catalog and the
UIUC prop fit. Run from the repo root:  python offline_pipeline/components/build_ref_config.py

Every derived number is ASSUMED where its inputs are (see catalog.json). Nothing here is validated.
 - mass_kg = frame + payload + 4 x (motor + ESC + prop) + battery.
 - inertia: 4 point masses (motor + ESC + prop) at the motor positions, plus the remaining mass as
   a solid box. Point mass at (x, y, 0): Ixx += m y^2, Iyy += m x^2, Izz += m (x^2 + y^2).
   Box a x b x h: Ixx = m (b^2 + h^2)/12, Iyy = m (a^2 + h^2)/12, Izz = m (a^2 + b^2)/12.
 - thrust_coeff, torque_coeff: static fit of the UIUC file (tools/props/fetch_uiuc.py).
 - max_rpm: motor.loaded_max_rpm_proxy (ASSUMED proxy).
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "offline_pipeline" / "components" / "catalog.json"
PROP_JSON = ROOT / "offline_pipeline" / "props" / "apce_12x6.json"
OUT = ROOT / "offline_pipeline" / "geometry" / "quad_ref_build_config.json"
MOTOR_SPIN = [1.0, -1.0, -1.0, 1.0]  # config order FR, FL, RR, RL, same as the shipped config
G_MPS2 = 9.80665
RHO_SL = 1.225  # ISA sea level, kg/m^3 (hover check only; the core uses its own ISA)


def val(cat: dict, group: str, key: str):
    return cat[group][key]["value"]


def motor_positions(spacing_m: float) -> list[list[float]]:
    h = spacing_m / 2.0
    return [[h, h, 0.0], [h, -h, 0.0], [-h, h, 0.0], [-h, -h, 0.0]]


def total_mass_kg(cat: dict) -> float:
    corner = val(cat, "motor", "mass_kg") + val(cat, "esc", "mass_kg") + val(cat, "prop", "mass_kg")
    return (val(cat, "frame", "frame_mass_kg") + val(cat, "frame", "avionics_payload_mass_kg")
            + 4.0 * corner + val(cat, "battery", "mass_kg"))


def inertia_kgm2(cat: dict, pos: list[list[float]]) -> list[float]:
    m_corner = val(cat, "motor", "mass_kg") + val(cat, "esc", "mass_kg") + val(cat, "prop", "mass_kg")
    ixx = sum(m_corner * p[1] ** 2 for p in pos)
    iyy = sum(m_corner * p[0] ** 2 for p in pos)
    izz = sum(m_corner * (p[0] ** 2 + p[1] ** 2) for p in pos)
    m_box = total_mass_kg(cat) - 4.0 * m_corner
    a, b, h = val(cat, "frame", "central_body_box_m")
    return [ixx + m_box * (b * b + h * h) / 12.0, iyy + m_box * (a * a + h * h) / 12.0,
            izz + m_box * (a * a + b * b) / 12.0]


def check_clearance(cat: dict, pos: list[list[float]]) -> dict:
    """Prop tip to prop tip (adjacent) and prop tip to the body edge. Raises if a prop does not clear."""
    d = val(cat, "prop", "diameter_m")
    spacing = val(cat, "frame", "motor_spacing_m")
    tip_gap = spacing - d
    inner_tip = math.hypot(pos[0][0], pos[0][1]) - d / 2.0
    body = val(cat, "frame", "body_half_width_m") * math.sqrt(2.0)  # corner of the square plate
    result = {"adjacent_tip_gap_m": tip_gap, "inner_tip_radius_m": inner_tip, "body_corner_radius_m": body}
    if tip_gap <= 0.0 or inner_tip <= body:
        raise ValueError(f"props do not clear: {result}")
    return result


def hover_throttle(mass_kg: float, ct: float, d_m: float, max_rpm: float, rho: float = RHO_SL) -> float:
    """Same formula as physics_core aero::hover_throttle: sqrt(m g / (4 Ct rho (rpm/60)^2 D^4))."""
    return math.sqrt(mass_kg * G_MPS2 / (4.0 * ct * rho * (max_rpm / 60.0) ** 2 * d_m ** 4))


def build(cat: dict, prop: dict) -> dict:
    pos = motor_positions(val(cat, "frame", "motor_spacing_m"))
    check_clearance(cat, pos)
    fit = prop["static_fit"]
    if abs(prop["diameter_m"] - val(cat, "prop", "diameter_m")) > 1e-9:
        raise ValueError("prop json diameter differs from the catalog")
    return {
        "mass_kg": round(total_mass_kg(cat), 6),
        "inertia_kgm2": [round(x, 6) for x in inertia_kgm2(cat, pos)],
        "motor_pos_m": pos,
        "motor_spin": MOTOR_SPIN,
        "prop_diameter_m": val(cat, "prop", "diameter_m"),
        "thrust_coeff": round(fit["ct"], 6),
        "torque_coeff": round(fit["kappa_m"], 6),
        "max_rpm": float(val(cat, "motor", "loaded_max_rpm_proxy")),
    }


def main() -> int:
    cat = json.loads(CATALOG.read_text())
    prop = json.loads(PROP_JSON.read_text())
    cfg = build(cat, prop)
    OUT.write_text(json.dumps(cfg, indent=2) + "\n")
    ht = hover_throttle(cfg["mass_kg"], cfg["thrust_coeff"], cfg["prop_diameter_m"], cfg["max_rpm"])
    print(f"wrote {OUT}\n  mass {cfg['mass_kg']} kg, inertia {cfg['inertia_kgm2']}, hover throttle {ht:.4f} "
          f"(RPM {ht * cfg['max_rpm']:.0f}), clearance {check_clearance(cat, cfg['motor_pos_m'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
