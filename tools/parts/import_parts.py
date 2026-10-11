# Tier: D
"""Bulk import of parts from a CSV (route b): validates EVERY row against the schema, writes nothing
unless all rows pass. One CSV per category (CSV has no sheets). Templates are generated from
`fields.py`: `python tools/parts/import_parts.py --write-templates`.

Columns: the plain string fields, then for every quantity field F:
  F (value), F__source_type, F__source_url, F__retrieved_on, F__sha256, F__note.
Vector and curve values are JSON text in the cell, e.g. [0.2,0.2,0.1] or [[0,3.3],[1,4.2]].
`category` and `schema_version` are filled in automatically. Row numbers in messages are CSV line
numbers (the header is line 1).

Usage (repo root, inside .venv):
  python tools/parts/import_parts.py --category motor my_motors.csv [--db parts_db]
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import fields as F
import partsdb as P

PROV_COLS = {"source_type": "source_type", "source_url": "source_url", "retrieved_on": "retrieved_on",
             "sha256": "sha256_of_file", "note": "note"}
AUTO_STRINGS = {"category", "schema_version"}


def template_columns(category: str) -> list[str]:
    cols = [s.name for s in F.string_fields(category) if s.name not in AUTO_STRINGS]
    for q in F.quantity_fields(category):
        cols.append(q.name)
        cols += [f"{q.name}__{suffix}" for suffix in PROV_COLS]
    return cols


def template_text(category: str) -> str:
    # Header only: no example row, so nobody imports a made-up part by accident.
    return ",".join(template_columns(category)) + "\r\n"


def template_path(tools_dir: Path, category: str) -> Path:
    return tools_dir / f"parts_template_{category}.csv"


def write_templates(tools_dir: Path) -> list[Path]:
    out = []
    for c in F.CATEGORIES:
        p = template_path(tools_dir, c)
        p.write_text(template_text(c), encoding="utf-8", newline="")
        out.append(p)
    return out


def _parse_value(q: F.Q, text: str):
    if q.vtype == "integer":
        return int(text)
    if q.vtype == "number":
        return float(text)
    return json.loads(text)


def row_to_record(category: str, row: dict[str, str], line: int) -> tuple[dict, list[str]]:
    """Build one record from a CSV row. Returns (record, errors); errors carry 'row N, field F: ...'."""
    errs: list[str] = []
    rec: dict = {"category": category, "schema_version": F.SCHEMA_VERSION}
    for s in F.string_fields(category):
        if s.name in AUTO_STRINGS:
            continue
        v = (row.get(s.name) or "").strip()
        if v:
            rec[s.name] = v
        elif s.required:
            errs.append(f"row {line}, field {s.name}: required value is empty")
    for q in F.quantity_fields(category):
        text = (row.get(q.name) or "").strip()
        prov = {}
        for col, key in PROV_COLS.items():
            cell = (row.get(f"{q.name}__{col}") or "").strip()
            if cell:
                prov[key] = cell
        if not text:
            if q.required:
                errs.append(f"row {line}, field {q.name}: required value is empty")
            elif prov:
                errs.append(f"row {line}, field {q.name}: provenance given but value is empty")
            continue
        if "source_type" not in prov:
            errs.append(f"row {line}, field {q.name}: provenance source_type is missing (every number needs provenance)")
            continue
        try:
            value = _parse_value(q, text)
        except ValueError as e:
            errs.append(f"row {line}, field {q.name}: cannot read {text!r} as {q.vtype} ({e})")
            continue
        rec[q.name] = {"value": value, "provenance": prov}
    return rec, errs


def import_csv(category: str, csv_path: Path, db_root: Path, write: bool = True) -> tuple[list[Path], list[str]]:
    """Validate all rows; write all records only if there are no errors at all."""
    if category not in F.CATEGORIES:
        return [], [f"unknown category {category!r}"]
    errors: list[str] = []
    records: list[dict] = []
    with csv_path.open(encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        missing = [c for c in template_columns(category) if c not in (reader.fieldnames or [])]
        required_cols = {s.name for s in F.string_fields(category) if s.required} - AUTO_STRINGS
        required_cols |= set(F.required_quantity_names(category))
        hard_missing = [c for c in missing if c in required_cols]
        if hard_missing:
            return [], [f"header: missing required column(s) {hard_missing}"]
        for line, row in enumerate(reader, start=2):
            rec, errs = row_to_record(category, row, line)
            if not errs:
                verr, _ = P.validate_record(rec, db_root)
                errs = [f"row {line}: {e}" for e in verr]
            errors += errs
            records.append(rec)
    ids = [r.get("id") for r in records]
    for i in sorted({x for x in ids if x and ids.count(x) > 1}):
        errors.append(f"id {i!r} appears more than once in the file")
    existing, _ = P.load_records(db_root)
    for i in ids:
        if i in existing:
            errors.append(f"id {i!r} already exists in the database")
    if errors or not write:
        return [], errors
    return [P.write_record(r, db_root) for r in records], []


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv", nargs="?", type=Path)
    ap.add_argument("--category")
    ap.add_argument("--db", type=Path, default=P.DEFAULT_DB)
    ap.add_argument("--write-templates", action="store_true")
    a = ap.parse_args(argv)
    if a.write_templates:
        for p in write_templates(Path(__file__).resolve().parent):
            print("wrote", p)
        return 0
    if not a.csv or not a.category:
        ap.error("need a CSV file and --category (or --write-templates)")
    paths, errs = import_csv(a.category, a.csv, a.db)
    if errs:
        print("IMPORT REFUSED, nothing written:", *errs, sep="\n  ", file=sys.stderr)
        return 1
    print(f"imported {len(paths)} part(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
