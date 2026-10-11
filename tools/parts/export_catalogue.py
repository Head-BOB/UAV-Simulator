# Tier: D
"""Writes parts_db/parts_catalogue.json: the compact, UI-ready view of the parts database (the Unreal app reads this file).

Per part: id, category, manufacturer, model, price, human-verification status, and the category's KEY fields, each with value,
unit and source type (CITED / ASSUMED / CALCULATED / MEASURED). Nothing is computed or converted here except the unit text, which
is read from the field-name suffix (units are part of every field name by project rule). Deterministic: sorted, no clock.

Usage: python tools/parts/export_catalogue.py [--db <parts_db>] [--out <file>] [--check]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import fields as F
import partsdb as P
import verification as V

SCHEMA_VERSION = 1
# longest suffix first. A name with no known suffix gets the unit "" (a count or a ratio).
UNIT_BY_SUFFIX = [("_rpm_per_v", "rpm/V"), ("_k_per_w", "K/W"), ("_m2", "m^2"), ("_kgm2", "kg m^2"), ("_pa", "Pa"), ("_ohm", "ohm"),
                  ("_kg", "kg"), ("_ah", "Ah"), ("_m", "m"), ("_a", "A"), ("_v", "V"), ("_w", "W"), ("_s", "s"), ("_c", "C")]


# names whose suffix would give a wrong unit: a battery C rating ends in _c but is not a temperature
UNIT_EXACT = {"max_continuous_c": "C rate", "measured_rpm_min": "rpm", "measured_rpm_max": "rpm"}


def unit_of(name: str) -> str:
    if name in UNIT_EXACT:
        return UNIT_EXACT[name]
    for suffix, unit in UNIT_BY_SUFFIX:
        if name.endswith(suffix):
            return unit
    return ""


def entry(rec: dict, ticks: list) -> dict:
    cat = rec["category"]
    key_fields = []
    for name in V.key_fields(rec):
        q = rec[name]
        key_fields.append({"name": name, "value": q["value"], "unit": unit_of(name), "source_type": q["provenance"]["source_type"]})
    price = rec.get("price_usd")
    return {"id": rec["id"], "category": cat, "manufacturer": rec.get("manufacturer", ""), "model": rec.get("model", ""),
            "frame_class": rec.get("frame_class"), "price_usd": None if price is None else price["value"],
            "price_source_type": None if price is None else price["provenance"]["source_type"], "verification": V.part_status(rec, ticks),
            "key_fields": key_fields, "notes": rec.get("notes", "")}


def build_catalogue(db_root: Path = P.DEFAULT_DB) -> dict:
    records, errors = P.load_records(db_root)
    if errors:
        raise ValueError("parts database has load errors: " + "; ".join(errors))
    ticks = V.load_ticks(db_root)
    parts = [entry(records[i], ticks) for i in sorted(records, key=lambda i: (records[i]["category"], i))]
    cats = sorted({p["category"] for p in parts})
    return {"schema_version": SCHEMA_VERSION, "db_version": _db_version(db_root), "categories": cats, "parts": parts}


def _db_version(db_root: Path) -> str:
    import json
    return json.loads((db_root / "VERSION.json").read_text(encoding="utf-8")).get("db_version", "unknown")


def render(db_root: Path = P.DEFAULT_DB) -> str:
    return P.canonical_json(build_catalogue(db_root))


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--db", type=Path, default=P.DEFAULT_DB)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--check", action="store_true", help="fail if the output file differs from a fresh export")
    a = ap.parse_args(argv)
    out = a.out or (a.db / "parts_catalogue.json")
    try:
        text = render(a.db)
    except (ValueError, OSError) as e:
        print(f"EXPORT STOPPED: {e}", file=sys.stderr)
        return 1
    if a.check:
        if not out.exists() or out.read_text(encoding="utf-8") != text:
            print(f"{out} is stale or missing: run tools/parts/export_catalogue.py", file=sys.stderr)
            return 2
        print("catalogue is up to date")
        return 0
    out.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
