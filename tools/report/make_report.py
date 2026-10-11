# Tier: D
"""Builds the engineer's report (data first, then HTML / JSON / CSV) from campaign result folders and the build's files.

Usage (one campaign):  python tools/report/make_report.py --build <build dir> --campaign <result dir> [--campaign <dir2> ...]
                         [--validation <validation_report.json>] [--cfd-dir <dir>] --out <report dir>
Usage (a study):       python tools/report/make_report.py --study <study dir> --out <report dir>
Everything in the report is read from files written by the other tools; nothing is estimated here. No clock and no machine path
is written, so the same inputs give byte-identical output.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import sys
from pathlib import Path

import _paths
import assumptions as A
import campaign_data as D
import margins as M
import partsdb as P
import power as PW
import svg_plot as S
import upgrades as U
import verification as V

SCHEMA_VERSION = 1
ROUND_SIG = 9
BUILD_FILES = ("vehicle_config.json", "powertrain_config.json", "controller_config.json", "vehicle_build.json", "assembly.json")
EXPLAIN_NOT_BUILT: list[dict] = []   # every planned test exists now; a test that was not part of a campaign is simply absent from its "tests" list
SPEED_TESTS = ("max_speed", "fpv_sprint", "fpv_punchout")


class ReportError(ValueError):
    """The inputs cannot make a report."""


def sha256_file(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def round_sig(v, n: int = ROUND_SIG):
    """Floats to n significant digits (recursively), so the JSON does not carry float noise."""
    if isinstance(v, bool) or v is None or isinstance(v, (int, str)):
        return v
    if isinstance(v, float):
        return float(f"{v:.{n}g}") if v == v and abs(v) != float("inf") else None
    if isinstance(v, dict):
        return {k: round_sig(x, n) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [round_sig(x, n) for x in v]
    return v


def read_json(p: Path) -> dict:
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise ReportError(f"cannot read {p.name}: {e}") from e


def load_build_dir(d: Path) -> dict:
    return {"dir": d, "sidecar": read_json(d / "vehicle_build.json"), "powertrain": read_json(d / "powertrain_config.json"),
            "controller": read_json(d / "controller_config.json"), "config": read_json(d / "vehicle_config.json")}


# ----------------------------------------------------------------------------------------------- sections

def bom_rows(sidecar: dict, records: dict, ticks: list) -> list[dict]:
    rows = []
    for p in sorted(sidecar["parts"], key=lambda x: x["role"]):
        rec = records.get(p["part_id"])
        counts: dict = {}
        mass = mass_src = price = None
        if rec is not None:
            for k, v in sorted(rec.items()):
                if isinstance(v, dict) and "provenance" in v:
                    st = v["provenance"].get("source_type", "UNKNOWN")
                    counts[st] = counts.get(st, 0) + 1
            if "mass_kg" in rec:
                mass, mass_src = rec["mass_kg"]["value"], rec["mass_kg"]["provenance"].get("source_type")
            price = rec["price_usd"]["value"] if "price_usd" in rec else None
        rows.append({"role": p["role"], "part_id": p["part_id"], "manufacturer": p.get("manufacturer"), "model": p.get("model"),
                     "mass_kg": mass, "mass_source": mass_src, "price_usd": price, "provenance_counts": counts,
                     "verification": V.part_status(rec, ticks) if rec is not None else "not in database"})
    return rows


def test_rows(campaign: dict) -> list[dict]:
    out = []
    for name in sorted(campaign["tests"]):
        t = campaign["tests"][name]
        r = t["result"]
        out.append({"test": name, "outcome": r["outcome"], "definition_version": r["definition_version"], "breaking_point": r.get("breaking_point"),
                    "metrics": r.get("metrics", []), "notes": r.get("notes", []), "steps_simulated": r.get("steps_simulated"),
                    "evaluations_note": "searches run many simulations; the series and events are of the reported run"})
    return out


def event_rows(campaign: dict) -> tuple[list[dict], list[dict]]:
    """(events that occurred, key moments from series and peaks)."""
    ev, key = [], []
    for name in sorted(campaign["tests"]):
        t = campaign["tests"][name]
        for e in t["events"]:
            if e["first_t_s"] is not None:
                ev.append({"test": name, "event": e["name"], "first_t_s": e["first_t_s"], "last_t_s": e["last_t_s"],
                           "activations": e["activations"], "active_steps": e["active_steps"]})
        for pk, label in (("winding_c", "winding temperature peak"), ("esc_c", "ESC temperature peak"), ("battery_c", "battery temperature peak"),
                          ("min_pack_v", "lowest pack voltage (sag)"), ("pack_current_a", "pack current peak"), ("motor_current_a", "motor current peak")):
            p = t["peaks"].get(pk)
            if p is not None:
                key.append({"test": name, "moment": label, "value": p["value"], "t_s": p["t_s"]})
        for fname in sorted(t["series"]):
            s = t["series"][fname]
            for tt, r in zip(s["t_s"], s["thrust_reserve_ratio"]):
                if r < A.RESERVE_ALERT_RATIO:
                    key.append({"test": name, "moment": f"thrust reserve first below {A.RESERVE_ALERT_RATIO:g}", "value": r, "t_s": tt})
                    break
    return ev, key


def pick_series(campaign: dict, prefer=("endurance", "thermal_soak", "climb", "hover_payload", "wind_gust")):
    for name in prefer:
        t = campaign["tests"].get(name)
        if t and t["series"]:
            fname = sorted(t["series"])[0]
            return name, fname, t["series"][fname]
    name = sorted(campaign["tests"])[0]
    fname = sorted(campaign["tests"][name]["series"])[0]
    return name, fname, campaign["tests"][name]["series"][fname]


def build_plots(campaign: dict) -> dict:
    name, fname, s = pick_series(campaign)
    t = s["t_s"]
    marks = [(e["first_t_s"], e["name"]) for e in campaign["tests"][name]["events"]
             if e["first_t_s"] is not None and e["name"] in ("battery_cutoff", "motor_over_current", "motor_over_temp", "esc_over_temp", "battery_over_temp")]
    rpm = [sum(s[f"rpm_{m}"][i] for m in range(1, 5)) / 4.0 for i in range(len(t))]
    return {"source": f"{name}/{fname}",
            "temperature": S.line_chart([("winding", t, s["winding_c_max"]), ("ESC", t, s["esc_c_max"]), ("battery", t, s["battery_c"])],
                                        "Temperatures", "time (s)", "deg C", marks),
            "voltage": S.line_chart([("pack voltage", t, s["battery_v"])], "Pack voltage", "time (s)", "V", marks),
            "current": S.line_chart([("pack current", t, s["battery_a"])], "Pack current", "time (s)", "A", marks),
            "rpm": S.line_chart([("mean rpm", t, rpm)], "Motor speed (mean of four)", "time (s)", "rpm", marks),
            "reserve": S.line_chart([("thrust reserve", t, s["thrust_reserve_ratio"])], "Thrust reserve (max thrust / weight)", "time (s)", "ratio", marks)}


def validation_section(path: Path | None) -> dict:
    if path is None:
        return {"status": "not provided", "sections": [], "flagged": []}
    v = read_json(path)
    secs = [{"name": s["name"], "status": s["status"], "note": s.get("note"), "summary": s["summary"], "rows": s["rows"]} for s in v["sections"]]
    return {"status": "provided", "sections": secs, "flagged": A.validation_flags(v), "flag_percent": A.VALIDATION_FLAG_PERCENT,
            "reference_files": v.get("reference_files"), "core_interface_version": v.get("core_interface_version")}


def cfd_section(cfd_dir: Path | None, drag_model: str) -> dict:
    if cfd_dir is None or not (cfd_dir / "geometry.json").exists():
        return {"status": "no CFD run", "drag_model": drag_model,
                "statement": "No CFD result is used. Body drag comes from the legacy analytic model, which does not see the vehicle's shape."}
    out = {"status": "preliminary CFD, unvalidated", "drag_model": drag_model, "geometry": read_json(cfd_dir / "geometry.json"), "images": []}
    sm = cfd_dir / "sweep_manifest.json"
    if sm.exists():
        m = read_json(sm)
        cases = m.get("cases", [])
        out["sweep"] = {"cases": len(cases), "build": m.get("build"), "propellers": m.get("propellers"), "ref_area_m2": m.get("ref_area_m2"),
                        "ref_length_m": m.get("ref_length_m"), "sweep_config_version": m.get("sweep_config_version"),
                        "grid": cases[0]["grid"] if cases else None, "dx_m": cases[0]["dx_m"] if cases else None}
    sr = cfd_dir / "cfd_sanity_report.json"
    out["sanity"] = read_json(sr) if sr.exists() else {"status": "NOT_RUN", "statement": "The sphere and grid-refinement checks have NOT been run."}
    models = sorted(cfd_dir.glob("*.model.json"))
    if models:
        out["model"] = read_json(models[0])
    for png in sorted(cfd_dir.glob("*.png")):
        out["images"].append({"name": png.name, "data_uri": "data:image/png;base64," + base64.b64encode(png.read_bytes()).decode("ascii")})
    return out


def condition_envelope(name: str, campaign: dict, margin_rows: list[dict], build: dict) -> dict:
    h = U.headline(campaign, margin_rows, build["config"].get("mass_kg"), None, margin_rows[0]["slot"])
    return {"condition_set": name, "conditions": campaign["common"]["conditions"], "limiting": {"slot": margin_rows[0]["slot"], "label": margin_rows[0]["label"],
            "margin": margin_rows[0]["margin"]}, "headline": {k: v for k, v in h.items() if k != "margins"},
            "outcomes": {t: campaign["tests"][t]["result"]["outcome"] for t in sorted(campaign["tests"])}}


def summary_section(campaign: dict, margin_rows: list[dict], power: dict | None) -> list[dict]:
    out = []
    lim = margin_rows[0]
    out.append({"label": "Limiting component", "value": lim["label"], "detail": f"margin {lim['margin']:.3g} ({lim['basis']})"})
    for test, key, label, unit in (("hover_payload", "passing_payload_kg", "Max hover payload", "kg"), ("wind_gust", "passing_wind_mps", "Max wind held", "m/s"),
                                   ("climb", "time_to_target_s", "Climb time to target", "s"), ("endurance", "duration_s", "Endurance", "s"),
                                   ("endurance", "energy_used_wh", "Energy used (endurance run)", "Wh"),
                                   ("max_speed", "max_passing_speed_mps", "Max speed (commanded, passing)", "m/s"),
                                   ("fpv_sprint", "terminal_speed_mps", "FPV sprint terminal speed", "m/s"),
                                   ("fpv_punchout", "final_climb_rate_mps", "FPV punch-out climb rate", "m/s")):
        v = D.metric(campaign, test, key)
        bp = (campaign["tests"].get(test) or {}).get("result", {}).get("breaking_point")
        detail = ""
        if bp:
            detail = f"between {bp['last_pass']} and {bp['first_fail']} {bp['unit']} (criterion {bp['criterion']}, resolution {bp['resolution']})"
        if test == "max_speed" and (campaign["tests"].get(test) or {}).get("result", {}).get("outcome") == "NOT_REACHED_WITHIN_BOUND":
            detail = "LIMIT-BOUND: the class speed limit ended the search, not the vehicle"
        out.append({"label": label, "value": v, "unit": unit, "detail": detail or ("" if v is not None else "not run in this campaign")})
    if power:
        share = power["percent_of_pack"]["shaft"]
        out.append({"label": f"Mean pack power ({power['source'].split('/')[0]} run)", "value": power["pack_w"], "unit": "W",
                    "detail": "no pack power" if share is None else f"shaft {share:.1f} % of pack power"})
    return out


def banners(common: dict, tests: list[str], bom: list[dict], flags: list[dict], powertrain: dict) -> list[dict]:
    out = []
    if "smoke" in common["definitions_set_version"]:
        out.append({"id": "smoke", "level": "danger", "text": f"SMOKE test definitions ({common['definitions_set_version']}): short durations and a nearly empty battery. NOT FOR ENGINEERING USE."})
    if common["definitions_set_version"].endswith("-edited"):
        out.append({"id": "edited_limits", "level": "warn", "text": f"Test limits were EDITED by the user ({common['definitions_set_version']}): pass / fail results are not the project's reviewed definitions."})
    kind = A.drag_kind(common["drag_model"])
    if kind == "cfd_stub":
        out.append({"id": "stub_model", "level": "danger", "text": "Drag comes from a STUB / SYNTHETIC surrogate model, not from CFD."})
    elif kind == "cfd":
        out.append({"id": "prelim_cfd", "level": "warn", "text": "Drag comes from a preliminary CFD surrogate: unvalidated, force only, propellers omitted."})
    else:
        out.append({"id": "no_cfd", "level": "warn", "text": "No CFD: body drag uses the legacy analytic model."})
    if A.turbulence_influenced(common, tests):
        out.append({"id": "turbulence", "level": "warn", "text": "Turbulence: Dryden, unverified."})
    if any(t in SPEED_TESTS for t in tests) and A.drag_kind(common["drag_model"]) == "analytic":
        out.append({"id": "speed_drag", "level": "warn", "text": "Speed results (max speed, FPV sprint, FPV punch-out) use the legacy analytic drag, which under-predicts forward speed "
                                                                   "and sinks when pitched: they are not vehicle limits."})
    unv = [r["part_id"] for r in bom if r["verification"] != "Verified"]
    if unv:
        out.append({"id": "unverified_parts", "level": "warn", "text": f"{len(unv)} of {len(bom)} parts are not human-verified: " + ", ".join(unv)})
    if flags:
        out.append({"id": "validation", "level": "warn", "text": f"{len(flags)} validation rows differ from their reference by more than {A.VALIDATION_FLAG_PERCENT:g} %."})
    if powertrain.get("parameters_verified") == 0:
        out.append({"id": "assumed_inputs", "level": "warn", "text": "The powertrain model contains ASSUMED inputs (see Assumptions)."})
    return out


def reproducibility(build: dict, campaigns: dict, validation_path: Path | None, commands: list[str], out_hint: Path | None) -> dict:
    files = {}
    for f in BUILD_FILES:
        files[f"build/{f}"] = sha256_file(build["dir"] / f)
    for cname in sorted(campaigns):
        root = campaigns[cname]["dir"]
        for p in sorted(root.rglob("*")):
            if p.is_file():
                files[f"campaign/{cname}/{p.relative_to(root).as_posix()}"] = sha256_file(p)
    if validation_path is not None:
        files["validation_report.json"] = sha256_file(validation_path)
    first = campaigns[sorted(campaigns)[0]]["common"]
    return {"git_hash": first["git_hash"], "core_interface_version": first["core_interface_version"], "controller_config_version": first["controller_config_version"],
            "definitions_set_version": first["definitions_set_version"], "parts_db_version": first["parts_db_version"], "step_s": first["step_s"],
            "seeds": {c: campaigns[c]["common"]["conditions"]["seed"] for c in sorted(campaigns)}, "drag_model": first["drag_model"], "turbulence": first["turbulence"],
            "conditions": {c: campaigns[c]["common"]["conditions"] for c in sorted(campaigns)}, "input_sha256": files, "commands": commands}


# ----------------------------------------------------------------------------------------------- upgrades

def upgrade_section(study_dir: Path | None, base_build: dict, base_head: dict, limiting_slot: str, db_records: dict, smoke: bool,
                    base_weakest: dict | None = None) -> dict:
    if study_dir is None:
        return {"status": "not run", "statement": "No upgrade study was run: give --study to compare single-part swaps by campaign results."}
    study = read_json(study_dir / "study.json")
    sel = study["selected_condition_set"]
    entries = []
    for c in study["candidates"]:
        e = {"id": c["id"], "role": c["role"], "part_id": c["part_id"], "status": c["status"], "errors": c["errors"], "warnings": c["warnings"],
             "scaled_basis": False, "deltas": None, "headline": None, "weakest": None, "reason": c.get("reason", "")}
        if c["status"] in ("tested", "not_tested"):
            d = study_dir / c["dir"].removeprefix("<study>/")
            cfg = read_json(d / "vehicle_config.json")
            side = read_json(d / "vehicle_build.json")
            e["scaled_basis"] = side["estimates"].get("ct_cp_basis") != U.SCALED_BASIS_OK
        if c["status"] == "tested":
            cdir = d / "campaign" / sel
            try:
                camp = D.load_campaign(cdir)
                pt = read_json(d / "powertrain_config.json")
                rows = M.compute_margins(camp, pt)
            except (D.CampaignDataError, M.MarginError, ReportError) as err:
                e["status"], e["reason"] = "not_tested", f"campaign result unusable: {err}"
            else:
                e["headline"] = U.headline(camp, rows, cfg.get("mass_kg"), c.get("price_usd"), limiting_slot)
                e["weakest"] = {"slot": rows[0]["slot"], "label": rows[0]["label"], "margin": rows[0]["margin"]}
                e["deltas"] = U.deltas(base_head, e["headline"])
        elif c["status"] == "rejected":
            e["reason"] = "not compatible: " + ", ".join(x["rule"] for x in c["errors"])
        entries.append(e)
    tested_status = {"tested": "tested", "not_tested": "not_tested", "rejected": "rejected"}
    for e in entries:
        e["status"] = tested_status[e["status"]]
    ranked = U.rank([e for e in entries if e["status"] == "tested"])
    rest = sorted([e for e in entries if e["status"] != "tested"], key=lambda e: (e["status"], e["id"]))
    for e in rest:
        e["rank"], e["rank_note"] = None, e["reason"] or e["status"]
    return {"status": "run", "condition_set": sel, "limiting_slot": limiting_slot, "metrics": {k: {"label": v[0], "unit": v[1], "higher_is_better": v[2]} for k, v in U.METRICS.items()},
            "baseline": {k: v for k, v in base_head.items() if k != "margins"}, "baseline_weakest": base_weakest, "entries": ranked + rest,
            "notice": ("SMOKE definitions: this comparison is not for engineering use. " if smoke else "")
                      + "Each change is a measured difference between two campaigns under the same conditions; every assumed input of the model applies to both."}


# ----------------------------------------------------------------------------------------------- assembly

def build_report(build_dir: Path, campaign_dirs: dict[str, Path], validation_path: Path | None = None, cfd_dir: Path | None = None,
                 study_dir: Path | None = None, commands: list[str] | None = None, db_root: Path = P.DEFAULT_DB, selected: str | None = None) -> tuple[dict, dict]:
    """(report dict, plots dict). `campaign_dirs`: condition-set name -> result folder; `selected` (default: the first name) drives the detail sections."""
    if not campaign_dirs:
        raise ReportError("at least one campaign folder is needed")
    build = load_build_dir(Path(build_dir))
    campaigns = {n: D.load_campaign(Path(p)) for n, p in campaign_dirs.items()}
    sel = selected or sorted(campaigns, key=lambda n: n)[0]
    if sel not in campaigns:
        raise ReportError(f"selected condition set {sel!r} has no campaign folder")
    camp = campaigns[sel]
    margin_rows = M.compute_margins(camp, build["powertrain"])
    records, _ = P.load_records(db_root)
    ticks = V.load_ticks(db_root)
    bom = bom_rows(build["sidecar"], records, ticks)
    status = {r["part_id"]: r["verification"] for r in bom}
    validation = validation_section(validation_path)
    val_doc = read_json(validation_path) if validation_path else None
    tests = sorted(camp["tests"])
    assumptions = A.collect(build["sidecar"], build["powertrain"], build["controller"], camp, val_doc, status)
    pw_name, pw_file, pw_series = pick_series(camp, prefer=("endurance", "thermal_soak", "climb"))
    power = PW.breakdown(pw_series)
    power["source"] = f"{pw_name}/{pw_file}"
    events, key_moments = event_rows(camp)
    base_head = U.headline(camp, margin_rows, build["config"].get("mass_kg"), U.parts_price_usd(
        {"parts": {r["role"]: r["part_id"] for r in build["sidecar"]["parts"] if r["role"] in ("frame", "motor", "esc", "propeller", "battery")}}, records), margin_rows[0]["slot"])
    common = camp["common"]
    smoke = "smoke" in common["definitions_set_version"]
    flags = validation["flagged"]
    report = {
        "schema_version": SCHEMA_VERSION, "title": f"Engineer's report: {common['build']}", "build": common["build"],
        "banners": banners(common, tests, bom, flags, build["powertrain"]),
        "summary": summary_section(camp, margin_rows, power),
        "vehicle": {"class": build["controller"].get("vehicle_class"), "bom": bom, "mass_kg": build["config"].get("mass_kg"),
                    "inertia_kgm2": build["config"].get("inertia_kgm2"), "mass_properties": build["sidecar"].get("mass_properties"),
                    "findings": build["sidecar"].get("findings", [])},
        "conditions": {"selected": sel, "values": common["conditions"], "all": {n: campaigns[n]["common"]["conditions"] for n in sorted(campaigns)}},
        "tests": test_rows(camp), "not_built": EXPLAIN_NOT_BUILT,
        "events": events, "key_moments": key_moments, "plot_source": None,
        "margins": margin_rows, "limiting": {"slot": margin_rows[0]["slot"], "label": margin_rows[0]["label"], "margin": margin_rows[0]["margin"]},
        "power": power,
        "endurance": {"duration_s": D.metric(camp, "endurance", "duration_s"), "energy_used_wh": D.metric(camp, "endurance", "energy_used_wh"),
                      "mean_power_w": D.metric(camp, "endurance", "mean_power_w"), "final_soc": D.metric(camp, "endurance", "final_soc"),
                      "notes": (camp["tests"].get("endurance") or {"result": {}})["result"].get("notes", [])},
        "cfd": cfd_section(cfd_dir, common["drag_model"]), "validation": validation,
        "envelope": [condition_envelope(n, campaigns[n], M.compute_margins(campaigns[n], build["powertrain"]), build) for n in sorted(campaigns)],
        "upgrades": upgrade_section(study_dir, build, base_head, margin_rows[0]["slot"], records, smoke,
                                    {"slot": margin_rows[0]["slot"], "label": margin_rows[0]["label"], "margin": margin_rows[0]["margin"]}),
        "assumptions": assumptions,
        "reproducibility": reproducibility(build, campaigns, validation_path, commands or [], None),
    }
    plots = build_plots(camp)
    report["plot_source"] = plots["source"]
    return round_sig(report), plots


# ----------------------------------------------------------------------------------------------- command line

def main(argv: list[str]) -> int:
    import render_html as R   # imported late: it needs nothing from here but the report dict
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--build", type=Path)
    ap.add_argument("--campaign", type=Path, action="append", default=[], help="result folder; the folder's condition-set name is read from its manifest")
    ap.add_argument("--validation", type=Path)
    ap.add_argument("--cfd-dir", type=Path)
    ap.add_argument("--study", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    try:
        if a.study:
            study = read_json(a.study / "study.json")
            base = a.study / study["baseline"]["dir"].removeprefix("<study>/")
            camps = {n: base / "campaign" / n for n in study["baseline_condition_sets"]}
            build_dir, val = base, base / "validation_report.json"
            cmds = [" ".join(r["command"]) for r in study["baseline"]["runs"]] + [" ".join(r["command"]) for e in study["candidates"] for r in e.get("runs", [])]
            report, plots = build_report(build_dir, camps, val if val.exists() else None, a.cfd_dir, a.study, cmds, selected=study["selected_condition_set"])
        else:
            if not a.build or not a.campaign:
                ap.error("give --study, or --build and at least one --campaign")
            camps = {}
            for c in a.campaign:
                name = D.load_campaign(c)["common"]["conditions"].get("condition_set") or c.name
                camps[name] = c
            report, plots = build_report(a.build, camps, a.validation, a.cfd_dir, None, [" ".join(["python", "tools/report/make_report.py", *argv_shown(argv)])],
                                         selected=next(iter(camps)))
    except (ReportError, D.CampaignDataError, M.MarginError, OSError) as e:
        print(f"REPORT STOPPED: {e}", file=sys.stderr)
        return 1
    R.write_all(report, plots, a.out)
    print(f"report written to {a.out}")
    return 0


def argv_shown(argv: list[str]) -> list[str]:
    """Command line without machine prefixes and without --out (where the report goes is not an input of it)."""
    out = []
    drop = {i for i, x in enumerate(argv) if x == "--out"}
    drop |= {i + 1 for i in drop}
    for i, x in enumerate(argv):
        if i in drop:
            continue
        p = Path(x)
        out.append(p.relative_to(_paths.REPO).as_posix() if p.is_absolute() and _paths.REPO in p.parents else (p.name if p.is_absolute() else x))
    return out


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
