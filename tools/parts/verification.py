# Tier: D
"""Human verification ticks and the generated PARTS-VERIFY.md checklist.

A tick records {part_id, field, value, verified_by, date}. It counts only while the record's current
value equals the ticked value: change a number and the tick goes stale (shown as STALE, counted as
unverified). A part is "Verified" only when every one of its key fields has a valid tick; otherwise
"Unverified" (none) or "Partly verified". The UI shows this status next to the provenance tags.
"""
from __future__ import annotations

import json
from pathlib import Path

import fields as F
import partsdb as P

HEADER = """# Parts verification checklist (GENERATED)

Do not edit by hand. Regenerate with `python tools/parts/parts_cli.py verify-sheet`.
For each line, open the source, compare the value, then record the tick:

    python tools/parts/parts_cli.py tick <part_id> <field> --by "<your name>"

A tick is tied to the value shown. If the value changes later the tick shows STALE and the part
goes back to Unverified. AI-transcribed values are Unverified until a human ticks them.
"""


def load_ticks(db_root: Path) -> list[dict]:
    p = db_root / "human_verification.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else []


def tick_state(rec: dict, field: str, ticks: list[dict]) -> str:
    """'ok', 'stale' or 'none' for one field of one record."""
    mine = [t for t in ticks if t["part_id"] == rec["id"] and t["field"] == field]
    if not mine:
        return "none"
    return "ok" if mine[-1]["value"] == P._val(rec, field) else "stale"


def key_fields(rec: dict) -> list[str]:
    """Key fields (max 8) present in this record, in the category's declared order."""
    return [f for f in F.CATEGORIES[rec["category"]]["key"] if f in rec][:8]


def part_status(rec: dict, ticks: list[dict]) -> str:
    states = [tick_state(rec, f, ticks) for f in key_fields(rec)]
    if states and all(s == "ok" for s in states):
        return "Verified"
    return "Partly verified" if any(s == "ok" for s in states) else "Unverified"


def add_tick(db_root: Path, rec: dict, field: str, by: str, today: str) -> dict:
    if field not in key_fields(rec):
        raise ValueError(f"{field} is not a key field of {rec['id']} (key fields: {key_fields(rec)})")
    if not by.strip():
        raise ValueError("--by must name the person who checked the datasheet")
    ticks = load_ticks(db_root)
    entry = {"part_id": rec["id"], "field": field, "value": P._val(rec, field),
             "verified_by": by.strip(), "date": today}
    ticks.append(entry)
    (db_root / "human_verification.json").write_text(P.canonical_json(ticks), encoding="utf-8", newline="\n")
    return entry


def _fmt(value) -> str:
    return json.dumps(value) if isinstance(value, list) else str(value)


def render_sheet(db_root: Path) -> str:
    recs, _ = P.load_records(db_root)
    ticks = load_ticks(db_root)
    lines = [HEADER]
    for rid in sorted(recs):
        rec = recs[rid]
        lines.append(f"\n## {rid} ({rec['category']}): {rec['manufacturer']} {rec['model']}  [{part_status(rec, ticks)}]\n")
        for f in key_fields(rec):
            prov = rec[f]["provenance"]
            st = tick_state(rec, f, ticks)
            box = {"ok": "[x]", "stale": "[ ] STALE", "none": "[ ]"}[st]
            src = prov.get("source_url") or "(no URL)"
            lines.append(f"- {box} `{f}` = {_fmt(rec[f]['value'])}  ({prov['source_type']}) {src}")
    return "\n".join(lines) + "\n"
