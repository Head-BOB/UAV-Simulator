# Tier: D
"""Assumptions and limitations, generated from what actually influenced THIS result (never a fixed boilerplate list).

Each row: {id, category, item, where, source_type, effect}. Rows are built only from the inputs given, so a part that is not in the
build or a test that did not run contributes nothing.
"""
from __future__ import annotations

VALIDATION_FLAG_PERCENT = 10.0   # the master prompt asks for every validation error above 10 % to be listed
RESERVE_ALERT_RATIO = 1.5        # the report marks when the thrust reserve first falls below this. ASSUMED display threshold


def _row(category: str, item: str, where: str, source_type: str, effect: str) -> dict:
    return {"category": category, "item": item, "where": where, "source_type": source_type, "effect": effect}


def validation_flags(validation: dict | None) -> list[dict]:
    """Every validation row whose |percent error| is above the flag level, with section name; sorted by section then id."""
    out = []
    for sec in (validation or {}).get("sections", []):
        for r in sec.get("rows", []):
            pe = r.get("percent_error")
            if pe is not None and abs(pe) > VALIDATION_FLAG_PERCENT:
                out.append({"section": sec.get("name"), "id": r["id"], "quantity": r["quantity"], "percent_error": pe,
                            "band_percent": r.get("band_percent"), "outside_band": bool(r.get("outside_band"))})
    out.sort(key=lambda r: (str(r["section"]), r["id"]))
    return out


def turbulence_influenced(common: dict, tests: list[str]) -> bool:
    """Dryden shaped the result when it is selected and there is wind to shape, or the wind test ran."""
    c = common["conditions"]
    return c["turbulence"] == "dryden" and (c["wind_speed_mps"] > 0 or "wind_gust" in tests)


def drag_kind(label: str) -> str:
    """'analytic' | 'cfd_stub' | 'cfd'."""
    if label.startswith("cfd_surrogate:"):
        return "cfd_stub" if label.split(":")[1] == "stub" else "cfd"
    return "analytic"


def collect(sidecar: dict, powertrain: dict, controller: dict, campaign: dict, validation: dict | None, part_status: dict) -> list[dict]:
    rows: list[dict] = []
    seen: set = set()

    def add(r: dict, key=None) -> None:
        k = key or (r["category"], r["item"])
        if k not in seen:
            seen.add(k)
            rows.append(r)

    for a in sidecar.get("powertrain", {}).get("assumed_part_inputs", []):
        add(_row("part input", f"{a['part_id']}.{a['field']}", "powertrain_config.json", "ASSUMED",
                 "feeds the electrical / thermal model; replace with a datasheet or measured value"), ("part", a["part_id"], a["field"]))
    for a in sidecar.get("assumed_inputs", []):
        add(_row("part input", f"{a['part_id']}.{a['field']}", "vehicle_build.json assumed_inputs", "ASSUMED", a.get("note", "")),
            ("part", a["part_id"], a["field"]))
    for text in sidecar.get("builder_assumptions", []):
        add(_row("builder", text, "tools/parts/build_vehicle.py", "ASSUMED", "builder constant or layout rule used to derive the vehicle config"))
    if powertrain.get("parameters_verified") == 0:
        add(_row("powertrain", "powertrain parameters are not all verified (parameters_verified = 0)", "powertrain_config.json",
                 "ASSUMED", "electrical, thermal and battery numbers behind every result are partly assumed"))
    prov = controller.get("provenance", {})
    if prov.get("source_type") == "ASSUMED":
        add(_row("controller", f"controller class values ({controller.get('vehicle_class')}) and derived gains", prov.get("source_file", "controller_config.json"),
                 "ASSUMED", "limits, bandwidths and gains shape every closed-loop result (tracking errors, break points)"))
    common = campaign["common"]
    tests = sorted(campaign["tests"])
    for name in tests:
        m = campaign["tests"][name]["manifest"]
        add(_row("test definition", f"{name} v{m['definition_version']} (set {m['definitions_set_version']})", "campaign/test_definitions.json",
                 "ASSUMED", "limits, durations, tolerances and criteria are engineering starting points, not requirements"))
    c = common["conditions"]
    label = c.get("condition_set") or "unnamed"
    add(_row("conditions", f"condition set {label}: ambient {c['ambient_c']:g} C, site {c['site_altitude_m']:g} m, wind {c['wind_speed_mps']:g} m/s, "
                           f"SOC {c['battery_soc']:g}, payload {c['payload_kg']:g} kg", "conditions file", "USER_INPUT",
             "chosen by the person running the study; results hold for these conditions only"))
    if turbulence_influenced(common, tests):
        add(_row("model", "turbulence: Dryden (MIL-HDBK-1797 low-altitude form), UNVERIFIED", "physics_core/src/turbulence.rs", "UNVERIFIED",
                 "every equation awaits a human check against the handbook (HUMAN-TODO); wind-test break points depend on it"))
    kind = drag_kind(common["drag_model"])
    if kind == "analytic":
        add(_row("model", "body drag: legacy analytic model (no CFD)", "physics_core/src/aero.rs", "ASSUMED",
                 "drag of the built vehicle is not computed from its shape"))
    else:
        add(_row("model", f"body drag: {common['drag_model']}", "per-build CFD surrogate", "PRELIMINARY",
                 "preliminary CFD surrogate, force only, propellers omitted from the geometry" + ("; STUB model, not trained on CFD" if kind == "cfd_stub" else "")))
    basis = sidecar.get("estimates", {}).get("ct_cp_basis")
    if basis and basis != "UIUC_MEASURED":
        add(_row("model", f"propeller coefficients basis: {basis}", "vehicle_build.json estimates", "ASSUMED",
                 "no measured table for this propeller: thrust and power are scaled from another size"))
    for name in tests:
        for note in campaign["tests"][name]["result"].get("notes", []):
            if "ASSUMED" in note:
                add(_row("test note", note, f"{name}/result.json", "ASSUMED", "stated by the test itself"))
    add(_row("report", f"thrust reserve alert threshold {RESERVE_ALERT_RATIO:g} (max thrust / weight)", "tools/report/assumptions.py", "ASSUMED",
             "only decides which time the event timeline marks as 'reserve fell below'; no result changes"))
    for pid, status in sorted(part_status.items()):
        if status != "Verified":
            add(_row("part verification", f"{pid}: {status}", "parts_db/human_verification.json", "UNVERIFIED",
                     "key fields have not been ticked by a human against the datasheet"))
    for v in validation_flags(validation):
        add(_row("validation", f"{v['section']} {v['id']}: error {v['percent_error']:+.2f} %", "validation_report.json",
                 "MEASURED_DEVIATION", f"model differs from the reference by more than {VALIDATION_FLAG_PERCENT:g} %"))
    for i, r in enumerate(rows, start=1):
        r["id"] = f"A{i:03d}"
    return rows
