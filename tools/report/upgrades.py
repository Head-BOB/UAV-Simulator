# Tier: D
"""Upgrade candidates (single part swaps) and their comparison with the baseline, from MEASURED campaign results only.

A candidate is a copy of the baseline build spec with ONE part of ONE role replaced by another database part. It must pass the
builder's compatibility checker (errors refuse it, warnings are kept). Nothing here estimates a result: a candidate whose campaign
did not run is "not tested" with the reason. The ranking key is the change of the BASELINE's limiting margin slot.
"""
from __future__ import annotations

import copy

import _paths  # noqa: F401  (puts tools/parts on sys.path)
import build_vehicle as B
import partsdb as P

ROLES = ("battery", "esc", "motor", "propeller")
MOTOR_COUNT = 4   # both vehicle classes are quads
SCALED_BASIS_OK = "UIUC_MEASURED"
# metric id -> (label, unit, higher_is_better). None = no preference shown.
METRICS = {
    "limiting_margin": ("Margin of the baseline's limiting component", "ratio", True),
    "max_payload_kg": ("Max hover payload", "kg", True),
    "max_wind_mps": ("Max wind held", "m/s", True),
    "endurance_s": ("Endurance", "s", True),
    "energy_wh": ("Energy used in the endurance run", "Wh", None),
    "min_thrust_reserve": ("Lowest thrust reserve", "ratio", True),
    "climb_time_s": ("Time to climb target", "s", False),
    "peak_winding_c": ("Peak winding temperature", "deg C", False),
    "peak_esc_c": ("Peak ESC temperature", "deg C", False),
    "peak_battery_c": ("Peak battery temperature", "deg C", False),
    "mass_kg": ("Vehicle mass", "kg", False),
    "price_usd": ("Parts price", "USD", False),
    "max_speed_mps": ("Max speed", "m/s", True),
}


def candidate_id(role: str, part_id: str) -> str:
    return f"{role}__{part_id}"


def swap_specs(spec: dict, records: dict) -> list[dict]:
    """All single swaps: [{id, role, part_id, spec}] sorted by id. Parts of other categories than the role are skipped."""
    out = []
    for role in ROLES:
        current = spec["parts"].get(role)
        for pid in sorted(records):
            rec = records[pid]
            if rec.get("category") != role or pid == current:
                continue
            new = copy.deepcopy(spec)
            new["parts"][role] = pid
            new["name"] = candidate_id(role, pid)
            out.append({"id": candidate_id(role, pid), "role": role, "part_id": pid, "spec": new})
    return out


def check_candidate(cand: dict, db_root=P.DEFAULT_DB) -> dict:
    """Runs the builder. status 'buildable' (result kept) or 'rejected' (error rule ids and messages)."""
    res = B.build(cand["spec"], db_root)
    errors = [f for f in res.findings if f.severity == "error"]
    warnings = [f for f in res.findings if f.severity == "warning"]
    if not res.ok or errors:
        return {**cand, "status": "rejected", "errors": [{"rule": f.rule, "message": f.message} for f in errors], "warnings": [], "build": None}
    return {**cand, "status": "buildable", "errors": [], "warnings": [{"rule": f.rule, "message": f.message} for f in warnings], "build": res}


def parts_price_usd(spec: dict, records: dict) -> float | None:
    """Sum of unit price x count over the spec's parts; None if any part has no price."""
    total = 0.0
    for role, pid in sorted(spec["parts"].items()):
        rec = records[pid]
        price = rec.get("price_usd")
        if price is None:
            return None
        if role in ("motor", "propeller"):
            n = MOTOR_COUNT
        elif role == "esc":
            n = MOTOR_COUNT if P._val(rec, "channels") == 1 else 1
        else:
            n = 1
        total += float(price["value"]) * n
    return total


def peak_over_tests(campaign: dict, name: str, pick=max):
    vals = [t["peaks"][name]["value"] for t in campaign["tests"].values() if name in t["peaks"]]
    return pick(vals) if vals else None


def headline(campaign: dict, margin_rows: list[dict], mass_kg: float | None, price_usd: float | None, limiting_slot: str) -> dict:
    """Comparable numbers of one campaign. None = not tested / not available."""
    import campaign_data as D
    by = {r["slot"]: r["margin"] for r in margin_rows}
    return {
        "limiting_margin": by.get(limiting_slot),
        "max_payload_kg": D.metric(campaign, "hover_payload", "passing_payload_kg"),
        "max_wind_mps": D.metric(campaign, "wind_gust", "passing_wind_mps"),
        "endurance_s": D.metric(campaign, "endurance", "duration_s"),
        "energy_wh": D.metric(campaign, "endurance", "energy_used_wh"),
        "min_thrust_reserve": peak_over_tests(campaign, "min_thrust_reserve_ratio", min),
        "climb_time_s": D.metric(campaign, "climb", "time_to_target_s"),
        "peak_winding_c": peak_over_tests(campaign, "winding_c"),
        "peak_esc_c": peak_over_tests(campaign, "esc_c"),
        "peak_battery_c": peak_over_tests(campaign, "battery_c"),
        "mass_kg": mass_kg,
        "price_usd": price_usd,
        "max_speed_mps": D.metric(campaign, "max_speed", "max_passing_speed_mps"),   # None when the campaign did not run it
        "margins": by,
    }


def deltas(base: dict, cand: dict) -> dict:
    """metric -> {base, cand, delta, better} for every metric; delta None when either side is not available."""
    out = {}
    for key, (_, _, higher) in METRICS.items():
        b, c = base.get(key), cand.get(key)
        if b is None or c is None:
            out[key] = {"base": b, "cand": c, "delta": None, "better": None}
            continue
        d = c - b
        better = None if (higher is None or abs(d) < 1e-12) else (d > 0) == higher
        out[key] = {"base": b, "cand": c, "delta": d, "better": better}
    return out


def rank(entries: list[dict]) -> list[dict]:
    """Sets 'rank' on tested, measured-basis entries by the change of the limiting margin (largest gain first, id breaks ties).

    Entries: {id, status, scaled_basis, deltas}. Candidates that are not tested, have no limiting-margin delta, or use a scaled
    stand-in propeller table get rank None and a reason.
    """
    ranked = []
    for e in entries:
        d = (e.get("deltas") or {}).get("limiting_margin", {}).get("delta")
        if e["status"] != "tested":
            e["rank"], e["rank_note"] = None, "not tested"
        elif e.get("scaled_basis"):
            e["rank"], e["rank_note"] = None, "propeller table is a scaled stand-in (ASSUMED): not ranked as measured"
        elif d is None:
            e["rank"], e["rank_note"] = None, "limiting margin not available"
        else:
            ranked.append(e)
    ranked.sort(key=lambda e: (-e["deltas"]["limiting_margin"]["delta"], e["id"]))
    for i, e in enumerate(ranked, start=1):
        e["rank"], e["rank_note"] = i, ""
    entries.sort(key=lambda e: (e["rank"] is None, e["rank"] or 0, e["id"]))
    return entries
