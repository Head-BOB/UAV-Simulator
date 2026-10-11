# Tier: D
"""Weakest-component ranking: margin = limit / peak load, from the peaks the campaign actually recorded.

Currents: margin = rated current / peak current. Temperatures: a RISE margin, (limit - ambient) / (peak - ambient), because a
ratio of absolute deg C is meaningless. Thrust reserve is already a ratio (maximum thrust / weight); its limit is 1.0.
A margin below 1.0 means the limit was exceeded in a run. Every limit comes from `powertrain_config.json` (part data, some ASSUMED).
"""
from __future__ import annotations

NO_HEATING_RISE_K = 0.1   # a peak less than this above ambient is "no heating" (and the margin is capped). ASSUMED display rule
MARGIN_CAP = 99.0
# slot id, label, peak name in events.json, limit key in powertrain_config.json, kind
SLOTS = [
    ("motor_current", "Motor current", "motor_current_a", "motor_max_current_a", "current"),
    ("esc_current", "ESC current", "motor_current_a", "esc_max_current_a", "current"),
    ("pack_current", "Battery pack current", "pack_current_a", "battery_max_continuous_a", "current"),
    ("winding_temp", "Motor winding temperature", "winding_c", "motor_winding_temp_limit_c", "temperature"),
    ("esc_temp", "ESC temperature", "esc_c", "esc_temp_limit_c", "temperature"),
    ("battery_temp", "Battery temperature", "battery_c", "battery_temp_limit_c", "temperature"),
]
RESERVE_SLOT = ("thrust_reserve", "Thrust reserve (max thrust / weight)")


class MarginError(ValueError):
    """A limit or a peak is missing or not usable."""


def _max_over_tests(campaign: dict, peak: str):
    best = None
    for name in sorted(campaign["tests"]):
        p = campaign["tests"][name]["peaks"].get(peak)
        if p is not None and (best is None or p["value"] > best[0]):
            best = (p["value"], name, p["t_s"])
    return best


def _min_over_tests(campaign: dict, peak: str):
    best = None
    for name in sorted(campaign["tests"]):
        p = campaign["tests"][name]["peaks"].get(peak)
        if p is not None and (best is None or p["value"] < best[0]):
            best = (p["value"], name, p["t_s"])
    return best


def current_margin(limit_a: float, peak_a: float) -> float:
    """limit / peak, capped. A zero peak is "never loaded" (cap)."""
    if limit_a <= 0.0:
        raise MarginError("current limit must be positive")
    if peak_a <= 0.0:
        return MARGIN_CAP
    return min(MARGIN_CAP, limit_a / peak_a)


def rise_margin(limit_c: float, peak_c: float, ambient_c: float) -> tuple[float, bool]:
    """(margin, no_heating). Rise margin (limit - ambient) / (peak - ambient)."""
    if limit_c <= ambient_c:
        raise MarginError("temperature limit must be above the ambient temperature")
    rise = peak_c - ambient_c
    if rise < NO_HEATING_RISE_K:
        return MARGIN_CAP, True
    return min(MARGIN_CAP, (limit_c - ambient_c) / rise), False


def compute_margins(campaign: dict, powertrain: dict) -> list[dict]:
    """All margin rows sorted ascending (the first row is the limiting metric); ties by slot id."""
    ambient = float(campaign["common"]["conditions"]["ambient_c"])
    rows = []
    for slot, label, peak_name, limit_key, kind in SLOTS:
        got = _max_over_tests(campaign, peak_name)
        if got is None:
            raise MarginError(f"no recorded peak {peak_name!r} in any test")
        if limit_key not in powertrain:
            raise MarginError(f"powertrain config has no {limit_key!r}")
        peak, test, t_s = got
        limit = float(powertrain[limit_key])
        if kind == "current":
            margin, no_heat, unit, basis = current_margin(limit, peak), False, "A", "limit / peak"
        else:
            margin, no_heat = rise_margin(limit, peak, ambient)
            unit, basis = "deg C", f"(limit - ambient) / (peak - ambient), ambient {ambient:g} deg C"
        rows.append({"slot": slot, "label": label, "limit": limit, "peak": peak, "unit": unit, "margin": margin,
                     "basis": basis, "no_heating": no_heat, "peak_test": test, "peak_t_s": t_s})
    got = _min_over_tests(campaign, "min_thrust_reserve_ratio")
    if got is None:
        raise MarginError("no recorded peak 'min_thrust_reserve_ratio' in any test")
    rows.append({"slot": RESERVE_SLOT[0], "label": RESERVE_SLOT[1], "limit": 1.0, "peak": got[0], "unit": "ratio",
                 "margin": got[0], "basis": "lowest recorded ratio / 1.0", "no_heating": False, "peak_test": got[1], "peak_t_s": got[2]})
    rows.sort(key=lambda r: (r["margin"], r["slot"]))
    return rows
