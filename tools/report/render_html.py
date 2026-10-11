# Tier: D
"""Writes report.html (self-contained, print-ready), report.json and the CSV files from the report dict. Deterministic: no clock."""
from __future__ import annotations

import csv
import io
import json
from html import escape
from pathlib import Path

SECTIONS = [("summary", "Summary"), ("vehicle", "Vehicle and bill of materials"), ("conditions", "Conditions"),
            ("tests", "Test results and breaking points"), ("events", "Event timeline"), ("limiting", "Limiting components"),
            ("power", "Power breakdown"), ("endurance", "Endurance and energy"), ("cfd", "CFD summary"), ("validation", "Validation"),
            ("envelope", "Capability envelope"), ("upgrades", "Upgrade comparison"), ("assumptions", "Assumptions and limitations"),
            ("reproducibility", "Reproducibility")]
NOT_AVAILABLE = "n/a"

CSS = """
:root{--bg:#fff;--fg:#1b1f24;--muted:#57606a;--line:#d0d7de;--warn:#9a6700;--danger:#cf222e;--good:#1a7f37;--bad:#cf222e;--code:#f6f8fa}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#0d1117;--fg:#e6edf3;--muted:#9da7b3;--line:#30363d;--warn:#d29922;--danger:#ff7b72;--good:#3fb950;--bad:#ff7b72;--code:#161b22}}
:root[data-theme="dark"]{--bg:#0d1117;--fg:#e6edf3;--muted:#9da7b3;--line:#30363d;--warn:#d29922;--danger:#ff7b72;--good:#3fb950;--bad:#ff7b72;--code:#161b22}
body{background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif;margin:0 auto;padding:16px;max-width:1100px}
h1{font-size:1.6rem}h2{font-size:1.25rem;border-bottom:1px solid var(--line);padding-bottom:4px;margin-top:2rem}h3{font-size:1.05rem}
table{border-collapse:collapse;width:100%;font-size:13px;margin:8px 0}th,td{border:1px solid var(--line);padding:3px 6px;text-align:left;vertical-align:top}
th{background:var(--code)}td.num{text-align:right;font-variant-numeric:tabular-nums}.tablewrap{overflow-x:auto}
.banner{border-left:5px solid var(--warn);padding:6px 10px;margin:6px 0;background:var(--code)}.banner.danger{border-color:var(--danger)}
.good{color:var(--good)}.bad{color:var(--bad)}.muted{color:var(--muted)}.flag{font-weight:bold;color:var(--bad)}
code,pre{background:var(--code);font-size:12px}pre{padding:8px;overflow-x:auto}svg{max-width:100%;height:auto;background:var(--bg)}svg text{fill:var(--fg)}
img{max-width:100%}
@media print{body{max-width:none;font-size:11px}h2{break-after:avoid}section{break-inside:auto}table,svg,img{break-inside:avoid}.tablewrap{overflow:visible}}
"""


def fmt(v, unit: str = "") -> str:
    """Number for display (4 significant digits unless it is a whole number); None is shown as n/a."""
    if v is None:
        return NOT_AVAILABLE
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, int):
        s = str(v)
    elif isinstance(v, float):
        s = f"{v:.0f}" if v == int(v) and abs(v) < 1e6 else f"{v:.4g}"
    else:
        s = str(v)
    return f"{s} {unit}".strip()


def td(v, unit: str = "", cls: str = "") -> str:
    num = isinstance(v, (int, float)) and not isinstance(v, bool)
    c = " ".join(x for x in (("num" if num else ""), cls) if x)
    return f'<td{f" class={chr(34)}{c}{chr(34)}" if c else ""}>{escape(fmt(v, unit))}</td>'


def table(headers: list[str], rows: list[list[str]]) -> str:
    """rows hold ready-made <td> strings."""
    head = "".join(f"<th>{escape(h)}</th>" for h in headers)
    body = "".join("<tr>" + "".join(r) + "</tr>" for r in rows)
    return f'<div class="tablewrap"><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'


def text_td(s) -> str:
    return f"<td>{escape('' if s is None else str(s))}</td>"


def sec(sid: str, title: str, inner: str) -> str:
    return f'<section id="{sid}"><h2>{escape(title)}</h2>{inner}</section>'


# ----------------------------------------------------------------------------------------------- sections

def s_summary(r: dict) -> str:
    banners = "".join(f'<div class="banner {escape(b["level"])}" data-banner="{escape(b["id"])}">{escape(b["text"])}</div>' for b in r["banners"])
    rows = [[text_td(x["label"]), td(x["value"], x.get("unit", "")), text_td(x.get("detail", ""))] for x in r["summary"]]
    return banners + table(["Result", "Value", "Detail"], rows)


def s_vehicle(r: dict) -> str:
    v = r["vehicle"]
    rows = [[text_td(b["role"]), text_td(b["part_id"]), text_td(f"{b['manufacturer']} {b['model']}"), td(b["mass_kg"], "kg"), text_td(b["mass_source"]),
             text_td(", ".join(f"{k} {n}" for k, n in sorted(b["provenance_counts"].items()))), text_td(b["verification"]),
             td(b["price_usd"], "USD")] for b in v["bom"]]
    inertia = v.get("inertia_kgm2")
    extra = f'<p>Vehicle class {escape(str(v["class"]))}; mass {escape(fmt(v["mass_kg"], "kg"))}; inertia (diagonal about the CG) {escape(", ".join(fmt(x) for x in inertia) if inertia else NOT_AVAILABLE)} kg m<sup>2</sup>.</p>'
    findings = "".join(f"<li><code>{escape(f['rule'])}</code> ({escape(f['severity'])}): {escape(f['message'])}</li>" for f in v["findings"])
    return extra + table(["Role", "Part", "Name", "Mass", "Mass source", "Field provenance (count)", "Human verification", "Unit price"], rows) + \
        (f"<h3>Build findings</h3><ul>{findings}</ul>" if findings else "")


def s_conditions(r: dict) -> str:
    c = r["conditions"]
    rows = [[text_td(k), td(v)] for k, v in c["values"].items()]
    other = ", ".join(sorted(c["all"]))
    return f"<p>Selected condition set: <b>{escape(str(c['selected']))}</b>. Sets with results in this report: {escape(other)}.</p>" + table(["Condition", "Value"], rows)


def s_tests(r: dict) -> str:
    out = ""
    for t in r["tests"]:
        bp = t["breaking_point"]
        head = f"<h3>{escape(t['test'])}: {escape(t['outcome'])} <span class='muted'>(definition v{escape(str(t['definition_version']))}, {escape(fmt(t['steps_simulated']))} steps)</span></h3>"
        if bp:
            head += (f"<p>Breaking point in <b>{escape(bp['quantity'])}</b> ({escape(bp['unit'])}): last pass {escape(fmt(bp['last_pass']))}, first fail {escape(fmt(bp['first_fail']))}; "
                     f"criterion <b>{escape(bp['criterion'])}</b>; resolution {escape(fmt(bp['resolution']))}; {escape(fmt(bp['evaluations']))} evaluations.</p>")
        rows = [[text_td(m["name"]), td(m["value"])] for m in t["metrics"]]
        notes = "".join(f"<li>{escape(n)}</li>" for n in t["notes"])
        out += head + (table(["Metric", "Value"], rows) if rows else "") + (f"<ul>{notes}</ul>" if notes else "")
    nb = "".join(f"<li>{escape(x['test'])}: {escape(x['status'])} ({escape(x['reason'])})</li>" for x in r["not_built"])
    return out + (f"<h3>Not tested</h3><ul>{nb}</ul>" if nb else "")


def s_events(r: dict, plots: dict) -> str:
    ev = [[text_td(e["test"]), text_td(e["event"]), td(e["first_t_s"], "s"), td(e["last_t_s"], "s"), td(e["activations"]), td(e["active_steps"])] for e in r["events"]]
    key = [[text_td(k["test"]), text_td(k["moment"]), td(k["value"]), td(k["t_s"], "s")] for k in r["key_moments"]]
    figs = "".join(f'<figure>{plots[k]}</figure>' for k in ("temperature", "voltage", "current", "rpm", "reserve"))
    return (f"<p>Plots show <code>{escape(r['plot_source'])}</code>.</p>{figs}<h3>Key moments</h3>" + table(["Test", "Moment", "Value", "Time"], key)
            + "<h3>Events that occurred</h3>" + (table(["Test", "Event", "First", "Last", "Activations", "Active steps"], ev) if ev else "<p>No event flag was raised in any test.</p>"))


def s_limiting(r: dict) -> str:
    rows = [[td(i + 1), text_td(m["label"]), td(m["limit"], m["unit"]), td(m["peak"], m["unit"]), td(m["margin"], cls="" if m["margin"] >= 1.0 else "bad"),
             text_td(m["peak_test"]), td(m["peak_t_s"], "s"), text_td(m["basis"] + ("; no heating" if m["no_heating"] else ""))] for i, m in enumerate(r["margins"])]
    return ("<p>Margin = limit / peak load from the recorded runs; below 1.0 the limit was exceeded. Temperatures use the rise above ambient. "
            f"The limiting component is <b>{escape(r['limiting']['label'])}</b>.</p>") + table(["Rank", "Component", "Limit", "Peak", "Margin", "In test", "At", "Basis"], rows)


def s_power(r: dict) -> str:
    p = r["power"]
    pc = p["percent_of_pack"]
    rows = [[text_td(n), td(p[k], "W"), td(100.0 if key is None else pc[key], "%")] for n, k, key in (
        ("Pack electrical power", "pack_w", None), ("Shaft (mechanical) power", "shaft_w", "shaft"), ("Motor copper loss", "motor_copper_w", "motor_copper"),
        ("Motor no-load loss", "motor_noload_w", "motor_noload"), ("ESC loss", "esc_loss_w", "esc_loss"), ("Other (battery internal, not split)", "other_w", "other"),
        ("Total loss", "loss_total_w", "loss_total"))]
    return f"<p>Time-averaged over <code>{escape(p['source'])}</code> (rows {escape(fmt(p['rows']))}, {escape(fmt(p['window_s'][0]))} to {escape(fmt(p['window_s'][1]))} s).</p>" + \
        table(["Quantity", "Average", "Share of pack power"], rows)


def s_endurance(r: dict) -> str:
    e = r["endurance"]
    if e["duration_s"] is None:
        return "<p>The endurance test was not part of this campaign.</p>"
    rows = [[text_td("Duration"), td(e["duration_s"], "s")], [text_td("Energy used"), td(e["energy_used_wh"], "Wh")],
            [text_td("Mean power"), td(e["mean_power_w"], "W")], [text_td("Final state of charge"), td(e["final_soc"])]]
    notes = "".join(f"<li>{escape(n)}</li>" for n in e["notes"])
    return table(["Quantity", "Value"], rows) + (f"<ul>{notes}</ul>" if notes else "")


def s_cfd(r: dict) -> str:
    c = r["cfd"]
    if c["status"] == "no CFD run":
        return f"<p><b>{escape(c['status'])}.</b> {escape(c['statement'])}</p>"
    out = f"<p><b>{escape(c['status'])}</b>. Drag model in the campaign: {escape(c['drag_model'])}.</p>"
    g = c["geometry"]
    out += f"<p>Geometry: {escape(fmt(g.get('triangles')))} triangles, STL sha256 <code>{escape(str(g.get('stl_sha256')))}</code>.</p>"
    if "sweep" in c and c["sweep"]["grid"]:
        s = c["sweep"]
        out += (f"<p>Sweep: {escape(fmt(s['cases']))} cases; first grid {escape(str(s['grid']))}, cell size {escape(fmt(s['dx_m'], 'm'))}; propellers: {escape(str(s['propellers']))}; "
                f"reference area {escape(fmt(s['ref_area_m2'], 'm2'))}, reference length {escape(fmt(s['ref_length_m'], 'm'))}.</p>")
    out += f"<p>Sanity checks: <b>{escape(str(c['sanity'].get('status')))}</b>. {escape(str(c['sanity'].get('statement', '')))}</p>"
    if "model" in c:
        m = c["model"]
        out += (f"<p>Surrogate: kind <b>{escape(str(m.get('kind')))}</b>, {escape(str(m.get('provenance')))}; held-out RMS error {escape(str(m.get('holdout_rms_n')))} N; "
                f"limitations: {escape('; '.join(m.get('limitations', [])))}.</p>")
    for im in c["images"]:
        out += f'<figure><img alt="{escape(im["name"])}" src="{im["data_uri"]}"><figcaption>{escape(im["name"])} (preliminary CFD, unvalidated)</figcaption></figure>'
    return out


def s_validation(r: dict) -> str:
    v = r["validation"]
    if v["status"] != "provided":
        return "<p>No validation report was given.</p>"
    out = f"<p>Rows whose error is above {escape(fmt(v['flag_percent']))} % are marked and listed under Assumptions.</p>"
    for s in v["sections"]:
        sm = s["summary"]
        out += (f"<h3>{escape(s['name'])} <span class='muted'>({escape(s['status'])})</span></h3><p>{escape(str(s.get('note') or ''))} Points {escape(fmt(sm['n']))}; "
                f"mean |error| {escape(fmt(sm['mean_abs_percent'], '%'))}; largest |error| {escape(fmt(sm['max_abs_percent'], '%'))} ({escape(str(sm['worst_row_id']))}); "
                f"outside band {escape(fmt(sm['n_outside_band']))}.</p>")
        rows = []
        for x in s["rows"]:
            pe = x["percent_error"]
            flag = pe is not None and abs(pe) > v["flag_percent"]
            rows.append([text_td(x["id"]), text_td(x["quantity"]), td(x["reference"], x["unit"]), td(x["model"], x["unit"]),
                         td(pe, "%", "flag" if flag else ""), td(x["band_percent"], "%"), text_td("outside" if x["outside_band"] else ""), text_td("above 10 %" if flag else "")])
        if rows:
            out += table(["Point", "Quantity", "Reference", "Model", "Error", "Band", "Band check", "Flag"], rows)
    return out


def s_envelope(r: dict) -> str:
    rows = []
    for e in r["envelope"]:
        h = e["headline"]
        rows.append([text_td(e["condition_set"]), td(h["max_payload_kg"], "kg"), td(h["max_wind_mps"], "m/s"), td(h["endurance_s"], "s"), td(h["climb_time_s"], "s"),
                     td(h["min_thrust_reserve"]), td(h["peak_winding_c"], "C"), text_td(e["limiting"]["label"]), td(e["limiting"]["margin"]), td(h["max_speed_mps"], "m/s")])
    return table(["Condition set", "Max payload", "Max wind", "Endurance", "Climb time", "Lowest reserve", "Peak winding", "Limiting component", "Margin", "Max speed"], rows)


def s_upgrades(r: dict) -> str:
    u = r["upgrades"]
    if u["status"] != "run":
        return f"<p>{escape(u['statement'])}</p>"
    show = ["limiting_margin", "max_payload_kg", "max_wind_mps", "endurance_s", "peak_winding_c", "peak_esc_c", "peak_battery_c", "mass_kg", "price_usd"]
    head = ["Rank", "Candidate", "Status"] + [u["metrics"][k]["label"] for k in show] + ["Weakest component after the swap", "Notes"]
    rows = []
    for e in u["entries"]:
        cells = [td(e["rank"]), text_td(e["id"]), text_td(e["status"])]
        for k in show:
            d = (e["deltas"] or {}).get(k)
            if d is None or d["delta"] is None:
                cells.append(td(None))
            else:
                cls = "good" if d["better"] else ("bad" if d["better"] is False else "")
                sign = "+" if d["delta"] > 0 else ""
                cells.append(f'<td class="num {cls}">{escape(sign + fmt(d["delta"]))}</td>')
        w = e.get("weakest")
        cells.append(text_td(NOT_AVAILABLE if w is None else f"{w['label']} (margin {fmt(w['margin'])})"))
        notes = [e["rank_note"]] + [x["rule"] for x in e["warnings"]]
        cells.append(text_td("; ".join(x for x in notes if x)))
        rows.append(cells)
    base = u["baseline"]
    return (f"<p>{escape(u['notice'])}</p><p>Condition set: <b>{escape(u['condition_set'])}</b>. Ranked by the change of the baseline's limiting margin (<b>{escape(u['limiting_slot'])}</b>). "
            f"Baseline: weakest component {escape(u['baseline_weakest']['label'])}, margin {escape(fmt(base['limiting_margin']))}, payload {escape(fmt(base['max_payload_kg'], 'kg'))}, endurance {escape(fmt(base['endurance_s'], 's'))}. "
            "Entries show the difference to the baseline; green is better, red is worse.</p>") + table(head, rows)


def s_assumptions(r: dict) -> str:
    rows = [[text_td(a["id"]), text_td(a["category"]), text_td(a["item"]), text_td(a["source_type"]), text_td(a["where"]), text_td(a["effect"])] for a in r["assumptions"]]
    return table(["Id", "Category", "Item", "Type", "Where", "Effect"], rows)


def s_repro(r: dict) -> str:
    p = r["reproducibility"]
    rows = [[text_td(k), td(p[k])] for k in ("git_hash", "core_interface_version", "controller_config_version", "definitions_set_version", "parts_db_version", "step_s", "drag_model", "turbulence")]
    seeds = [[text_td(k), td(v)] for k, v in p["seeds"].items()]
    files = [[text_td(k), text_td(v)] for k, v in p["input_sha256"].items()]
    cmds = "\n".join(p["commands"])
    return (table(["Item", "Value"], rows) + "<h3>Seeds</h3>" + table(["Condition set", "Seed"], seeds) + "<h3>Commands</h3><pre>" + escape(cmds) + "</pre>"
            + "<h3>Input files (sha256)</h3>" + table(["File", "sha256"], files))


BUILDERS = {"summary": s_summary, "vehicle": s_vehicle, "conditions": s_conditions, "tests": s_tests, "limiting": s_limiting, "power": s_power,
            "endurance": s_endurance, "cfd": s_cfd, "validation": s_validation, "envelope": s_envelope, "upgrades": s_upgrades,
            "assumptions": s_assumptions, "reproducibility": s_repro}


def render_html(report: dict, plots: dict) -> str:
    parts = []
    for sid, title in SECTIONS:
        inner = s_events(report, plots) if sid == "events" else BUILDERS[sid](report)
        parts.append(sec(sid, title, inner))
    toc = "".join(f'<a href="#{sid}">{escape(t)}</a> ' for sid, t in SECTIONS)
    return ("<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
            f"<title>{escape(report['title'])}</title><style>{CSS}</style></head><body><h1>{escape(report['title'])}</h1><nav>{toc}</nav>" + "".join(parts) + "</body></html>\n")


# ----------------------------------------------------------------------------------------------- csv / json

def csv_text(headers: list[str], rows: list[list]) -> str:
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(headers)
    for r in rows:
        w.writerow(["" if v is None else (json.dumps(v) if isinstance(v, float) else v) for v in r])
    return buf.getvalue()


def csv_files(r: dict) -> dict[str, str]:
    out = {}
    out["summary.csv"] = csv_text(["label", "value", "unit", "detail"], [[x["label"], x["value"], x.get("unit", ""), x.get("detail", "")] for x in r["summary"]])
    out["margins.csv"] = csv_text(["rank", "slot", "component", "limit", "peak", "unit", "margin", "peak_test", "peak_t_s", "basis"],
                                  [[i + 1, m["slot"], m["label"], m["limit"], m["peak"], m["unit"], m["margin"], m["peak_test"], m["peak_t_s"], m["basis"]] for i, m in enumerate(r["margins"])])
    out["events.csv"] = csv_text(["test", "event", "first_t_s", "last_t_s", "activations", "active_steps"],
                                 [[e["test"], e["event"], e["first_t_s"], e["last_t_s"], e["activations"], e["active_steps"]] for e in r["events"]])
    u = r["upgrades"]
    metrics = list(u["metrics"]) if u["status"] == "run" else []
    out["upgrades.csv"] = csv_text(["rank", "candidate", "role", "part_id", "status", "note", "weakest_slot", "weakest_margin"] + [f"delta_{k}" for k in metrics],
                                   [[e["rank"], e["id"], e["role"], e["part_id"], e["status"], e["rank_note"], (e.get("weakest") or {}).get("slot"), (e.get("weakest") or {}).get("margin")]
                                    + [((e["deltas"] or {}).get(k) or {}).get("delta") for k in metrics]
                                    for e in u.get("entries", [])])
    out["assumptions.csv"] = csv_text(["id", "category", "item", "type", "where", "effect"],
                                      [[a["id"], a["category"], a["item"], a["source_type"], a["where"], a["effect"]] for a in r["assumptions"]])
    rows = [[s["name"], x["id"], x["quantity"], x["unit"], x["reference"], x["model"], x["percent_error"], x["band_percent"], x["outside_band"],
             x["percent_error"] is not None and abs(x["percent_error"]) > r["validation"]["flag_percent"]] for s in r["validation"]["sections"] for x in s["rows"]]
    out["validation.csv"] = csv_text(["section", "id", "quantity", "unit", "reference", "model", "percent_error", "band_percent", "outside_band", "above_flag_level"], rows)
    return out


def write_all(report: dict, plots: dict, out: Path) -> list[Path]:
    out.mkdir(parents=True, exist_ok=True)
    files = {"report.html": render_html(report, plots), "report.json": json.dumps(report, indent=2, sort_keys=True) + "\n", **csv_files(report)}
    paths = []
    for name in sorted(files):
        p = out / name
        p.write_text(files[name], encoding="utf-8", newline="\n")
        paths.append(p)
    return paths
