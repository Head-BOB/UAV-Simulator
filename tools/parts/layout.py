# Tier: D
"""Where each part sits in the body frame, and the mass bodies used for CG and inertia.

Body frame (ADR-001): X forward, Y right, Z DOWN; origin = geometric centre of the frame's central body box.
Motors in config order FR, FL, RR, RL at (+-s/2, +-s/2) with spin [+1, -1, -1, +1], the same convention as the
shipped vehicle_config.json (open point B-3: confirm on the real airframe).

Every offset and rule here is a LAYOUT ASSUMPTION, `TODO(HUMAN-VERIFY)`: real positions come from the real build.
A build file can override any part's centre with `position_override_m`.
"""
from __future__ import annotations

import math

import mechanics as M

MOTOR_SIGNS = [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]   # FR, FL, RR, RL (x forward, y right)
MOTOR_SPIN = [1.0, -1.0, -1.0, 1.0]                                  # +1 = reaction torque about body +Z (CCW from above)
MAST_HEIGHT_M = 0.05            # GPS mast height above the top plate. ASSUMED, TODO(HUMAN-VERIFY)
CFRP_DENSITY_KGPM3 = 1600.0     # carbon-fibre laminate density for the arm mass split. ASSUMED, TODO(HUMAN-VERIFY): same value as offline_pipeline/fea units test
ARM_MASS_MAX_FRACTION = 0.9     # the arm estimate may not exceed this share of the frame mass. ASSUMED sanity limit

SLOTS = ("frame_plate", "stack_30x30", "stack_20x20", "mast", "front_mount", "belly_mount", "power_input")
ROLE_ORDER = ["wiring", "flight_controller", "receiver", "pdb", "gps", "camera", "gimbal", "payload"]


class LayoutError(ValueError):
    """The build cannot be laid out (reported as rule E-LAYOUT)."""


def motor_xy(spacing_m: float) -> list[tuple[float, float]]:
    h = spacing_m / 2.0
    return [(sx * h, sy * h) for sx, sy in MOTOR_SIGNS]


def arm_section(frame: dict) -> dict:
    """Section of one arm from the frame record: tube if given, else plate. Returns kind, area, I, Z and the half thickness."""
    def v(name):
        q = frame.get(name)
        return q["value"] if q else None
    if v("tube_od_m") is not None:
        s = M.tube_section(v("tube_od_m"), v("tube_id_m"))
        return {"kind": "tube", "od_m": v("tube_od_m"), "id_m": v("tube_id_m"), "half_thickness_m": v("tube_od_m") / 2.0, **s}
    s = M.plate_section(v("arm_plate_width_m"), v("arm_plate_thickness_m"))
    return {"kind": "plate", "width_m": v("arm_plate_width_m"), "thickness_m": v("arm_plate_thickness_m"),
            "half_thickness_m": v("arm_plate_thickness_m") / 2.0, **s}


def frame_bodies(frame: dict, section: dict) -> tuple[list[M.Body], dict]:
    """Frame mass split into a central body box and four arm rods (centre to motor), arms from section area x CFRP density."""
    spacing = frame["motor_spacing_adjacent_m"]["value"]
    arm_len = frame["arm_length_m"]["value"]
    box = frame.get("central_body_box_m")
    if box is None:
        raise LayoutError("frame needs central_body_box_m for the inertia estimate")
    m_frame = frame["mass_kg"]["value"]
    m_arm = CFRP_DENSITY_KGPM3 * section["area_m2"] * arm_len
    if 4.0 * m_arm >= ARM_MASS_MAX_FRACTION * m_frame:
        raise LayoutError(f"arm mass estimate {4.0 * m_arm:.4f} kg is >= {ARM_MASS_MAX_FRACTION:.0%} of the frame mass {m_frame} kg: "
                          "arm section or frame mass is inconsistent")
    bodies = [M.Body("frame_body", m_frame - 4.0 * m_arm, (0.0, 0.0, 0.0), M.box_inertia(m_frame - 4.0 * m_arm, tuple(box["value"])))]
    for i, (x, y) in enumerate(motor_xy(spacing)):
        norm = math.hypot(x, y)
        d = (x / norm, y / norm, 0.0)
        centre = (0.5 * arm_len * d[0], 0.5 * arm_len * d[1], 0.0)
        bodies.append(M.Body(f"arm_{i}", m_arm, centre, M.rod_inertia(m_arm, arm_len, d)))
    return bodies, {"arm_mass_each_kg": m_arm, "body_mass_kg": m_frame - 4.0 * m_arm, "density_kgpm3": CFRP_DENSITY_KGPM3}


def _slot_center(slot: str, bbox: tuple, ctx: dict, stack_offset_m: float) -> tuple[float, float, float]:
    a, b, c = bbox
    hb, bx = ctx["body_h_m"], ctx["body_x_m"]
    if slot in ("frame_plate", "stack_30x30", "stack_20x20"):
        return (stack_offset_m, 0.0, -hb / 2.0 - c / 2.0)
    if slot == "mast":
        return (stack_offset_m, 0.0, -hb / 2.0 - MAST_HEIGHT_M - c / 2.0)
    if slot == "front_mount":
        return (bx / 2.0 + a / 2.0, stack_offset_m, 0.0)
    if slot == "belly_mount":
        return (stack_offset_m, 0.0, hb / 2.0 + ctx["battery_h_m"] + c / 2.0)
    if slot == "power_input":
        return (stack_offset_m, 0.0, hb / 2.0 - c / 2.0)
    raise LayoutError(f"unknown mount_slot {slot!r} (known: {', '.join(SLOTS)})")


def place_slotted(items: list[dict], ctx: dict) -> dict[str, tuple]:
    """Centre of every slotted part. Parts sharing a slot are laid side by side (x, or y for the front mount), centred."""
    by_slot: dict[str, list[dict]] = {}
    for it in items:
        by_slot.setdefault(it["slot"], []).append(it)
    out: dict[str, tuple] = {}
    for slot, group in by_slot.items():
        group.sort(key=lambda it: (ROLE_ORDER.index(it["role"]) if it["role"] in ROLE_ORDER else 99, it["id"]))
        axis = 1 if slot == "front_mount" else 0
        total = sum(it["bbox"][axis] for it in group)
        cursor = -total / 2.0
        for it in group:
            width = it["bbox"][axis]
            out[it["id"]] = _slot_center(slot, it["bbox"], ctx, cursor + width / 2.0)
            cursor += width
    return out


def corner_bodies(motor: dict, esc: dict, prop: dict, spacing_m: float, arm_half_m: float, esc_each: bool) -> tuple[list[M.Body], float, list[dict]]:
    """Motor (cylinder), propeller (thin disc above it) and, for 1-channel ESCs, an ESC box at each motor.
    Returns bodies, the propeller-plane z (thrust point) and placement notes for the assembly."""
    bb = motor["bounding_box_m"]["value"]
    d_motor, h_motor = bb[0], bb[2]
    z_motor = -(arm_half_m + h_motor / 2.0)
    pbb = prop["bounding_box_m"]["value"]
    z_prop = -(arm_half_m + h_motor + pbb[2] / 2.0)
    m_motor, m_prop = motor["mass_kg"]["value"], prop["mass_kg"]["value"]
    d_prop = prop["diameter_m"]["value"]
    bodies: list[M.Body] = []
    notes: list[dict] = []
    for i, (x, y) in enumerate(motor_xy(spacing_m)):
        bodies.append(M.Body(f"motor_{i}", m_motor, (x, y, z_motor), M.cylinder_inertia(m_motor, d_motor, h_motor)))
        bodies.append(M.Body(f"prop_{i}", m_prop, (x, y, z_prop), M.disc_inertia(m_prop, d_prop)))
        notes.append({"index": i, "xy": (x, y), "motor_z": z_motor, "prop_z": z_prop})
        if esc_each:
            m_esc = esc["mass_kg"]["value"]
            bodies.append(M.Body(f"esc_{i}", m_esc, (x, y, z_motor), M.box_inertia(m_esc, tuple(esc["bounding_box_m"]["value"]))))
    return bodies, z_prop, notes
