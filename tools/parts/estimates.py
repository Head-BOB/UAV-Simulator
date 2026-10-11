# Tier: D
"""Design-time electrical and aerodynamic ESTIMATES for the vehicle builder.

These are quick, hand-checkable numbers used to set `max_rpm` in the exported config and to feed the
compatibility rules. Phase P3's electrical model in the Rust core supersedes them. Known simplifications:
nominal pack voltage (no sag), no ESC voltage drop, static propeller coefficients (no forward-flight loss),
sea-level ISA density, constant no-load current.

Model (all SI, n in rev/s):
  prop:   T = Ct rho n^2 D^4,   Q = Cp rho n^2 D^5 / (2 pi)           (UIUC static fit, see tools/props/fetch_uiuc.py)
  motor:  Kt = 60 / (2 pi Kv) [N m/A],  I = Q / Kt + I0,  rpm = Kv (V - I Rm)
  full throttle: solve 60 n = Kv (V - I(n) Rm) for n by bisection (bounded iterations, deterministic).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

G_MPS2 = 9.80665                                   # same constant as physics_core aero.rs GRAVITY
RHO_SL_KGPM3 = 101325.0 / (287.05 * 288.15)         # ISA sea level, identical to physics_core aero::air_density(0, standard)
BISECTION_STEPS = 200                               # bound on the loop; 2^-200 is far below f64 resolution
MAX_ELECTRICAL_ITERATIONS = BISECTION_STEPS


def kt_nm_per_a(kv_rpm_per_v: float) -> float:
    """Torque constant Kt = 60 / (2 pi Kv), N*m per A (SI Kv in rad/s/V is 2 pi Kv / 60)."""
    if not (math.isfinite(kv_rpm_per_v) and kv_rpm_per_v > 0):
        raise ValueError("Kv must be positive and finite")
    return 60.0 / (2.0 * math.pi * kv_rpm_per_v)


def kappa_m(cp: float, d_m: float, ct: float) -> float:
    """Yaw torque per thrust, kappa = Q / T = Cp D / (2 pi Ct) [N m per N]; the core's `torque_coeff`."""
    if not (ct > 0 and cp > 0 and d_m > 0):
        raise ValueError("Ct, Cp and D must be positive")
    return cp * d_m / (2.0 * math.pi * ct)


def prop_torque_nm(cp: float, rho: float, n_rps: float, d_m: float) -> float:
    return cp * rho * n_rps ** 2 * d_m ** 5 / (2.0 * math.pi)


def prop_thrust_n(ct: float, rho: float, n_rps: float, d_m: float) -> float:
    return ct * rho * n_rps ** 2 * d_m ** 4


def motor_current_a(kv: float, i0_a: float, torque_nm: float) -> float:
    return torque_nm / kt_nm_per_a(kv) + i0_a


def solve_full_throttle(kv: float, rm_ohm: float, i0_a: float, v_v: float, ct: float, cp: float, d_m: float,
                        rho: float = RHO_SL_KGPM3) -> dict:
    """Operating point where the motor at terminal voltage `v_v` drives the propeller (torque balance).

    g(n) = 60 n - Kv (V - I(n) Rm) is increasing in n, g(0) < 0 (needs V > I0 Rm) and g(Kv V / 60) > 0, so bisection converges.
    Returns rpm, current_a, torque_nm, thrust_n (per motor), shaft_w, electrical_w (V I) and the iteration count.
    """
    for name, val in (("Kv", kv), ("Rm", rm_ohm), ("V", v_v), ("Ct", ct), ("Cp", cp), ("D", d_m), ("rho", rho)):
        if not (math.isfinite(val) and val > 0):
            raise ValueError(f"{name} must be positive and finite")
    if not (math.isfinite(i0_a) and i0_a >= 0):
        raise ValueError("I0 must be non-negative and finite")
    if v_v <= i0_a * rm_ohm:
        raise ValueError("voltage does not even overcome the no-load voltage drop")

    def g(n: float) -> float:
        cur = motor_current_a(kv, i0_a, prop_torque_nm(cp, rho, n, d_m))
        return 60.0 * n - kv * (v_v - cur * rm_ohm)

    lo, hi = 0.0, kv * v_v / 60.0
    steps = 0
    while steps < BISECTION_STEPS:
        mid = 0.5 * (lo + hi)
        if g(mid) > 0.0:
            hi = mid
        else:
            lo = mid
        steps += 1
    n = 0.5 * (lo + hi)
    q = prop_torque_nm(cp, rho, n, d_m)
    cur = motor_current_a(kv, i0_a, q)
    return {"rpm": n * 60.0, "current_a": cur, "torque_nm": q, "thrust_n": prop_thrust_n(ct, rho, n, d_m),
            "shaft_w": q * 2.0 * math.pi * n, "electrical_w": v_v * cur, "iterations": steps}


def hover_throttle(mass_kg: float, ct: float, d_m: float, max_rpm: float, rho: float = RHO_SL_KGPM3) -> float:
    """Same formula as physics_core aero::hover_throttle: sqrt(m g / (4 Ct rho (rpm/60)^2 D^4)). Not clamped."""
    if not (mass_kg > 0 and ct > 0 and d_m > 0 and max_rpm > 0):
        raise ValueError("mass, Ct, D and max RPM must be positive")
    return math.sqrt(mass_kg * G_MPS2 / (4.0 * ct * rho * (max_rpm / 60.0) ** 2 * d_m ** 4))


def hover_point(kv: float, rm_ohm: float, i0_a: float, ct: float, cp: float, d_m: float, mass_kg: float,
                rho: float = RHO_SL_KGPM3) -> dict:
    """Per-motor hover state: thrust m g / 4 -> n -> torque -> current, and the terminal voltage needed (V = rpm/Kv + I Rm)."""
    t_each = mass_kg * G_MPS2 / 4.0
    n = math.sqrt(t_each / (ct * rho * d_m ** 4))
    q = prop_torque_nm(cp, rho, n, d_m)
    cur = motor_current_a(kv, i0_a, q)
    v_need = n * 60.0 / kv + cur * rm_ohm
    return {"rpm": n * 60.0, "thrust_n": t_each, "torque_nm": q, "current_a": cur, "terminal_voltage_v": v_need,
            "electrical_w": v_need * cur}


def pack_current_a(motor_electrical_w_each: float, n_motors: int, esc_efficiency: float, v_pack_v: float,
                   avionics_w: float) -> float:
    """Battery current: (motors' electrical power / ESC efficiency + avionics power) / pack voltage."""
    if not (0 < esc_efficiency <= 1 and v_pack_v > 0):
        raise ValueError("efficiency must be in (0, 1] and V > 0")
    return (n_motors * motor_electrical_w_each / esc_efficiency + avionics_w) / v_pack_v


def load_static_fit(prop_record: dict, records: dict, repo_root: Path) -> dict:
    """Ct, Cp from the propeller's UIUC static fit; a SCALED prop uses its parent's (ASSUMED stand-in)."""
    rec = prop_record
    basis = "UIUC_MEASURED"
    if rec["data_basis"] == "SCALED":
        parent_id = rec["scaled_from"]
        if parent_id not in records or records[parent_id].get("data_basis") != "UIUC_MEASURED":
            raise ValueError(f"{rec['id']}: scaled_from {parent_id!r} is not a UIUC_MEASURED propeller")
        basis = f"SCALED_FROM:{parent_id}"
        rec = records[parent_id]
    data = json.loads((repo_root / rec["data_file"]).read_text(encoding="utf-8"))
    fit = data["static_fit"]
    ct, cp = float(fit["ct"]), float(fit["cp"])
    if not (math.isfinite(ct) and math.isfinite(cp) and ct > 0 and cp > 0):
        raise ValueError(f"{rec['id']}: static fit Ct/Cp invalid")
    return {"ct": ct, "cp": cp, "band_rpm": fit["band_rpm"], "basis": basis, "data_file": rec["data_file"]}
