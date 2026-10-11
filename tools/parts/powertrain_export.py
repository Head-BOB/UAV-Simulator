# Tier: D
"""Builds `powertrain_config.json` (the core's `PowertrainConfig`) from the part records of a build.

Electrical values come straight from the records. The thermal network is DERIVED because the parts database stores one thermal
resistance per part and a motor time constant, while the core wants a two-node motor network:
  * heat capacity of a part = mass x specific heat (specific heats are named ASSUMED constants below, TODO(HUMAN-VERIFY));
  * motor: total resistance from the record split winding-to-case / case-to-ambient by a named ASSUMED fraction, capacity split
    winding / case by a named ASSUMED fraction;
  * the record's thermal_time_constant_s is NOT used: it is compared with R_total x C_total and a mismatch is reported.
`parameters_verified` is 1 only if no value feeding the file is ASSUMED (never true today).
"""
from __future__ import annotations

import json
import math

MAX_OCV_POINTS = 16                      # must equal physics_core types::MAX_OCV_POINTS
MOTOR_R_WINDING_CASE_FRACTION = 0.3      # share of the motor's total thermal resistance between winding and case. ASSUMED, TODO(HUMAN-VERIFY)
MOTOR_C_WINDING_FRACTION = 0.25          # share of the motor's heat capacity in the winding (copper). ASSUMED, TODO(HUMAN-VERIFY)
SPECIFIC_HEAT_MOTOR_J_PER_KG_K = 450.0   # steel/copper/aluminium mix. ASSUMED, TODO(HUMAN-VERIFY)
SPECIFIC_HEAT_ESC_J_PER_KG_K = 900.0     # aluminium/PCB mix. ASSUMED, TODO(HUMAN-VERIFY)
SPECIFIC_HEAT_BATTERY_J_PER_KG_K = 1000.0  # LiPo pack. ASSUMED, TODO(HUMAN-VERIFY)
COOLING_REF_AIRSPEED_MPS = 5.0           # air speed that doubles the cooling. ASSUMED, TODO(HUMAN-VERIFY)
COOLING_EXPONENT = 0.8                   # must equal physics_core powertrain::COOLING_EXPONENT
THERMAL_STEP_FRACTION = 0.2              # must equal physics_core powertrain::THERMAL_STEP_FRACTION
DT_MAX_S = 0.01                          # largest dt the core accepts
WORST_CASE_AIRSPEED_MPS = 100.0          # for the build-time stability check only
TAU_MISMATCH_FACTOR = 2.0                # warn if the record's tau and R x C differ by more than this factor


class PowertrainExportError(ValueError):
    """The build cannot be exported (rule E-POWERTRAIN)."""


def _v(rec: dict, field: str):
    return rec[field]["value"]


def _t(rec: dict, field: str) -> str:
    return rec[field]["provenance"]["source_type"]


def _tidy(x):
    """Drop float conversion noise (0.75 x 46.8 = 35.099999999999994) by keeping 12 significant digits."""
    if isinstance(x, float):
        return float(f"{x:.12g}")
    if isinstance(x, list):
        return [_tidy(v) for v in x]
    return x


def cooling_factor(v_mps: float) -> float:
    return 1.0 + (v_mps / COOLING_REF_AIRSPEED_MPS) ** COOLING_EXPONENT


def build_powertrain(motor: dict, esc: dict, prop: dict, battery: dict, vehicle_config: dict, cp: float,
                     prop_curves: list | None = None) -> tuple[dict, dict, list]:
    """Returns (config, provenance, warnings). `vehicle_config` is the dict going to vehicle_config.json (its Ct and diameter are reused).
    `prop_curves` ([{rpm, points: [[J, Ct, Cp], ...]}], see prop_table.py) switches the core to the Ct/Cp(J, rpm) table; None keeps the static constants."""
    curve = _v(battery, "ocv_curve")
    if len(curve) > MAX_OCV_POINTS:
        raise PowertrainExportError(f"{battery['id']}: OCV curve has {len(curve)} points, the core holds {MAX_OCV_POINTS}")
    r_total = _v(motor, "thermal_resistance_k_per_w")
    c_motor = _v(motor, "mass_kg") * SPECIFIC_HEAT_MOTOR_J_PER_KG_K
    c_esc = _v(esc, "mass_kg") * SPECIFIC_HEAT_ESC_J_PER_KG_K
    c_batt = _v(battery, "mass_kg") * SPECIFIC_HEAT_BATTERY_J_PER_KG_K
    cfg = {
        "motor_kv_rpm_per_v": _v(motor, "kv_rpm_per_v"),
        "motor_resistance_ohm": _v(motor, "winding_resistance_ohm"),
        "motor_no_load_current_a": _v(motor, "no_load_current_a"),
        "motor_max_current_a": _v(motor, "max_current_a"),
        "motor_winding_temp_limit_c": _v(motor, "winding_temp_limit_c"),
        "prop_diameter_m": vehicle_config["prop_diameter_m"],
        "prop_ct": vehicle_config["thrust_coeff"],
        "prop_cp": cp,
        "esc_efficiency": _v(esc, "efficiency"),
        "esc_max_current_a": _v(esc, "max_continuous_current_a"),
        "esc_temp_limit_c": _v(esc, "temp_limit_c"),
        "battery_capacity_ah": _v(battery, "capacity_ah"),
        "battery_internal_resistance_ohm": _v(battery, "internal_resistance_ohm"),
        "battery_cutoff_v_per_cell": _v(battery, "voltage_per_cell_cutoff_v"),
        "battery_max_continuous_a": _v(battery, "max_continuous_c") * _v(battery, "capacity_ah"),
        "battery_temp_limit_c": _v(battery, "temp_limit_c"),
        "motor_r_winding_case_k_per_w": MOTOR_R_WINDING_CASE_FRACTION * r_total,
        "motor_r_case_ambient_k_per_w": (1.0 - MOTOR_R_WINDING_CASE_FRACTION) * r_total,
        "motor_c_winding_j_per_k": MOTOR_C_WINDING_FRACTION * c_motor,
        "motor_c_case_j_per_k": (1.0 - MOTOR_C_WINDING_FRACTION) * c_motor,
        "esc_r_ambient_k_per_w": _v(esc, "thermal_resistance_k_per_w"),
        "esc_c_j_per_k": c_esc,
        "battery_r_ambient_k_per_w": _v(battery, "thermal_resistance_k_per_w"),
        "battery_c_j_per_k": c_batt,
        "cooling_ref_airspeed_mps": COOLING_REF_AIRSPEED_MPS,
        "battery_cells": _v(battery, "cells"),
        "parameters_verified": 0,
        "ocv_curve": [list(p) for p in curve],
    }
    inputs = [(motor, f) for f in ("kv_rpm_per_v", "winding_resistance_ohm", "no_load_current_a", "max_current_a", "winding_temp_limit_c",
                                   "thermal_resistance_k_per_w", "mass_kg")]
    inputs += [(esc, f) for f in ("efficiency", "max_continuous_current_a", "temp_limit_c", "thermal_resistance_k_per_w", "mass_kg")]
    inputs += [(battery, f) for f in ("capacity_ah", "internal_resistance_ohm", "voltage_per_cell_cutoff_v", "max_continuous_c",
                                      "temp_limit_c", "thermal_resistance_k_per_w", "mass_kg", "ocv_curve", "cells")]
    assumed = [{"part_id": r["id"], "field": f} for r, f in inputs if _t(r, f) == "ASSUMED"]
    builder_assumed = ["motor R split", "motor C split", "specific heats", "cooling reference air speed", "cooling exponent"]
    cfg["parameters_verified"] = 1 if not assumed and not builder_assumed else 0   # the builder constants are always ASSUMED today
    cfg = {k: _tidy(v) for k, v in cfg.items()}
    if prop_curves is not None:
        cfg["prop_curves"] = prop_curves          # computed table values keep full precision (no tidying)
    warnings = _checks(cfg, motor, r_total, c_motor)
    prov = {
        "source": "tools/parts/powertrain_export.py",
        "assumed_part_inputs": assumed,
        "builder_assumptions": builder_assumed,
        "derived": {
            "motor_c_total_j_per_k": {"value": c_motor, "formula": f"motor mass x {SPECIFIC_HEAT_MOTOR_J_PER_KG_K} J/kg/K"},
            "motor_tau_from_rc_s": r_total * c_motor,
            "motor_tau_record_s": _v(motor, "thermal_time_constant_s"),
            "esc_c_j_per_k": {"value": c_esc, "formula": f"ESC mass x {SPECIFIC_HEAT_ESC_J_PER_KG_K} J/kg/K"},
            "battery_c_j_per_k": {"value": c_batt, "formula": f"pack mass x {SPECIFIC_HEAT_BATTERY_J_PER_KG_K} J/kg/K"},
            "battery_max_continuous_a": {"value": cfg["battery_max_continuous_a"], "formula": "max_continuous_c x capacity_ah"},
            "motor_r_winding_case": f"{MOTOR_R_WINDING_CASE_FRACTION} x the record's thermal_resistance_k_per_w",
            "motor_c_winding": f"{MOTOR_C_WINDING_FRACTION} x motor heat capacity",
        },
        "propeller_model": "table" if prop_curves is not None else "static",
        "parameters_verified_rule": "1 only if no part input and no builder constant is ASSUMED",
    }
    return cfg, prov, warnings


def smallest_time_constant_s(cfg: dict) -> float:
    """tau_min used by the core's explicit-step guard, at the worst-case cooling (100 m/s plus a 20 m/s prop wash)."""
    f_worst = cooling_factor(WORST_CASE_AIRSPEED_MPS + 20.0)
    f_free = cooling_factor(WORST_CASE_AIRSPEED_MPS)
    return min(cfg["motor_r_winding_case_k_per_w"] * cfg["motor_c_winding_j_per_k"],
               cfg["motor_r_winding_case_k_per_w"] * cfg["motor_c_case_j_per_k"],
               cfg["motor_r_case_ambient_k_per_w"] / f_worst * cfg["motor_c_case_j_per_k"],
               cfg["esc_r_ambient_k_per_w"] / f_free * cfg["esc_c_j_per_k"],
               cfg["battery_r_ambient_k_per_w"] / f_free * cfg["battery_c_j_per_k"])


def _checks(cfg: dict, motor: dict, r_total: float, c_motor: float) -> list:
    """Returns warning strings; raises for a config the core would refuse at every step."""
    tau_min = smallest_time_constant_s(cfg)
    if DT_MAX_S > THERMAL_STEP_FRACTION * tau_min:
        raise PowertrainExportError(
            f"thermal network too fast for the core's step: tau_min {tau_min:.4f} s needs dt <= {THERMAL_STEP_FRACTION * tau_min:.4f} s, "
            f"the core accepts up to {DT_MAX_S} s")
    warns = []
    tau_rc, tau_rec = r_total * c_motor, _v(motor, "thermal_time_constant_s")
    ratio = max(tau_rc, tau_rec) / min(tau_rc, tau_rec)
    if ratio > TAU_MISMATCH_FACTOR:
        warns.append(f"{motor['id']}: thermal_time_constant_s {tau_rec:.1f} s (record) vs R x C {tau_rc:.1f} s (mass-based) differ by a factor {ratio:.1f}; "
                     "the mass-based capacity is used")
    if not all(math.isfinite(v) for v in cfg.values() if isinstance(v, float)):
        raise PowertrainExportError("non-finite value in the powertrain config")
    return warns


def format_powertrain(cfg: dict) -> str:
    """JSON text of the config: scalars one per line, tables one point per line (a 4-curve table is about 100 lines, not 1000)."""
    lines = ["{"]
    items = list(cfg.items())
    for n, (key, value) in enumerate(items):
        comma = "," if n < len(items) - 1 else ""
        if key == "ocv_curve":
            body = ",\n".join("    " + json.dumps(p) for p in value)
            lines.append(f'  "{key}": [\n{body}\n  ]{comma}')
        elif key == "prop_curves":
            curves = []
            for c in value:
                pts = ",\n".join("      " + json.dumps(p) for p in c["points"])
                curves.append(f'    {{"rpm": {json.dumps(c["rpm"])}, "points": [\n{pts}\n    ]}}')
            lines.append(f'  "{key}": [\n' + ",\n".join(curves) + f"\n  ]{comma}")
        else:
            lines.append(f'  "{key}": {json.dumps(value)}{comma}')
    lines.append("}")
    return "\n".join(lines) + "\n"
