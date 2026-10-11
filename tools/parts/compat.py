# Tier: D
"""Compatibility and feasibility rules for a build. Every finding carries a rule id.

Severity: "error" blocks the build, "warning" is shown and saved, "info" is bookkeeping.
Every threshold below is an ASSUMED engineering rule of thumb, `TODO(HUMAN-VERIFY)`: a named constant, not a standard.
Part-to-part hard limits come from `partsdb.incompatibilities` (P1) and are mapped to rule ids here.
"""
from __future__ import annotations

import math
from typing import NamedTuple

import partsdb as P

TIP_GAP_WARN_FRACTION = 0.05      # adjacent tip gap below this share of D is a warning. ASSUMED
TW_ERROR_BELOW = 1.0              # static thrust/weight below this cannot lift off. Physics, not a rule of thumb
TW_WARN_BELOW = 2.0               # control margin rule of thumb. ASSUMED
HOVER_THROTTLE_WARN_ABOVE = 0.7   # little headroom left. ASSUMED
HOVER_CURRENT_WARN_FRACTION = 0.7  # of the pack's continuous rating. ASSUMED
ARM_STRESS_WARN_MARGIN = 1.5      # allowable / applied below this warns. ASSUMED
ARM_STRESS_ERROR_MARGIN = 1.0     # allowable / applied below this is an error


class Finding(NamedTuple):
    rule: str
    severity: str
    message: str
    parts: tuple

    def as_dict(self) -> dict:
        return {"rule": self.rule, "severity": self.severity, "message": self.message, "parts": list(self.parts)}


def _map_pair(a: dict, b: dict, rule_for) -> list[Finding]:
    out = []
    for msg in P.incompatibilities(a, b):
        out.append(Finding(rule_for(msg), "error", msg, (a["id"], b["id"])))
    return out


def pair_findings(parts: dict) -> list[Finding]:
    """The P1 hard rules between the main parts, mapped to rule ids."""
    motor, esc, prop, bat, frame = (parts[k] for k in ("motor", "esc", "propeller", "battery", "frame"))
    out: list[Finding] = []
    out += _map_pair(motor, bat, lambda m: "E-CELLS-MOTOR")
    out += _map_pair(esc, bat, lambda m: "E-CELLS-ESC")
    out += _map_pair(motor, esc, lambda m: "E-ESC-I")
    out += _map_pair(motor, prop, lambda m: "E-SHAFT" if "bore" in m else "E-PROP-RANGE")
    out += _map_pair(motor, frame, lambda m: "E-MOUNT")
    out += _map_pair(prop, frame, lambda m: "E-FRAME-PROP")
    return out


def arms_finding(frame: dict) -> list[Finding]:
    n = frame["arm_count"]["value"]
    if n != 4:
        return [Finding("E-ARMS", "error", f"{frame['id']} has {n} arms; the core simulates exactly 4 motors", (frame["id"],))]
    return []


def clearance_findings(frame_id: str, prop_id: str, spacing_m: float, d_m: float, body_corner_radius_m: float) -> list[Finding]:
    """Adjacent tip gap = s - D (props on a square of side s). Inner tip radius = s/sqrt(2) - D/2 vs the body corner."""
    out: list[Finding] = []
    gap = spacing_m - d_m
    if gap <= 0.0:
        out.append(Finding("E-CLEAR-TIP", "error", f"adjacent props overlap: spacing {spacing_m:.4f} m - D {d_m:.4f} m = {gap:.4f} m", (frame_id, prop_id)))
    elif gap < TIP_GAP_WARN_FRACTION * d_m:
        out.append(Finding("E-CLEAR-TIP", "warning", f"adjacent tip gap {gap:.4f} m is below {TIP_GAP_WARN_FRACTION:.0%} of D", (frame_id, prop_id)))
    inner = spacing_m / math.sqrt(2.0) - d_m / 2.0
    if inner <= body_corner_radius_m:
        out.append(Finding("E-CLEAR-BODY", "error", f"inner prop tip radius {inner:.4f} m does not clear the body corner radius {body_corner_radius_m:.4f} m", (frame_id, prop_id)))
    return out


def performance_findings(p: dict) -> list[Finding]:
    """`p`: computed numbers and the used parts (see build_vehicle). Keys documented where they are built."""
    out: list[Finding] = []
    motor, esc, bat = p["motor_id"], p["esc_id"], p["battery_id"]
    if not p["hover_throttle"] <= 1.0:
        out.append(Finding("E-HOVER", "error", f"hover throttle {p['hover_throttle']:.3f} > 1: the vehicle cannot hover (core requires <= 1)", (motor,)))
    elif p["hover_throttle"] > HOVER_THROTTLE_WARN_ABOVE:
        out.append(Finding("W-HOVER-THR", "warning", f"hover throttle {p['hover_throttle']:.3f} > {HOVER_THROTTLE_WARN_ABOVE}: little headroom", (motor,)))
    tw = p["thrust_to_weight"]
    if tw < TW_ERROR_BELOW:
        out.append(Finding("E-TW", "error", f"static thrust/weight {tw:.2f} < {TW_ERROR_BELOW}", (motor,)))
    elif tw < TW_WARN_BELOW:
        out.append(Finding("W-TW", "warning", f"static thrust/weight {tw:.2f} < {TW_WARN_BELOW}", (motor,)))
    if p["full_current_a"] > p["motor_max_current_a"]:
        out.append(Finding("W-FULL-I-MOTOR", "warning", f"estimated full-throttle current {p['full_current_a']:.1f} A exceeds the motor's {p['motor_max_current_a']:.1f} A rating", (motor,)))
    if p["full_current_a"] > p["esc_max_current_a"]:
        out.append(Finding("W-FULL-I-ESC", "warning", f"estimated full-throttle current {p['full_current_a']:.1f} A exceeds the ESC's {p['esc_max_current_a']:.1f} A rating", (esc,)))
    if p["hover_current_a"] > p["motor_max_current_a"]:
        out.append(Finding("E-HOVER-I-MOTOR", "error", f"hover current {p['hover_current_a']:.1f} A exceeds the motor rating {p['motor_max_current_a']:.1f} A", (motor,)))
    limit = p["pack_limit_a"]
    if p["hover_pack_a"] > limit:
        out.append(Finding("E-HOVER-I", "error", f"hover pack current {p['hover_pack_a']:.1f} A exceeds the battery's {limit:.1f} A continuous rating", (bat,)))
    elif p["hover_pack_a"] > HOVER_CURRENT_WARN_FRACTION * limit:
        out.append(Finding("W-HOVER-I", "warning", f"hover pack current {p['hover_pack_a']:.1f} A is above {HOVER_CURRENT_WARN_FRACTION:.0%} of the battery's {limit:.1f} A", (bat,)))
    if p["full_pack_a"] > limit:
        out.append(Finding("W-FULL-I-PACK", "warning", f"full-throttle pack current {p['full_pack_a']:.1f} A exceeds the battery's {limit:.1f} A continuous rating", (bat,)))
    margin = p.get("arm_margin")
    if margin is not None:
        if margin < ARM_STRESS_ERROR_MARGIN:
            out.append(Finding("E-ARM-STRESS", "error", f"arm bending margin {margin:.2f} < {ARM_STRESS_ERROR_MARGIN} at full-throttle thrust", (p["frame_id"],)))
        elif margin < ARM_STRESS_WARN_MARGIN:
            out.append(Finding("W-ARM-STRESS", "warning", f"arm bending margin {margin:.2f} < {ARM_STRESS_WARN_MARGIN}", (p["frame_id"],)))
    return out


def inertia_finding(diag: list) -> list[Finding]:
    """Physical consistency: principal-like moments must satisfy the triangle inequalities (internal check)."""
    a, b, c = diag
    if a > b + c or b > a + c or c > a + b:
        return [Finding("E-INERTIA", "error", f"inertia diagonal {diag} violates the triangle inequality", ())]
    return []
