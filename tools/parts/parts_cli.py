# Tier: D
"""Parts database command line (run from the repo root, inside .venv).

  parts_cli.py validate                       validate every record + cross-record rules
  parts_cli.py search [--category C] [--text T] [--max-mass KG] [--max-price USD]
                      [--compatible-with category=id,category=id]
  parts_cli.py show ID                        print one record
  parts_cli.py index                          rebuild the SQLite index
  parts_cli.py add --from FILE                add one validated record (refuses incomplete ones)
  parts_cli.py schema [--check]               write / check parts_db/schema/part.schema.json
  parts_cli.py verify-sheet [--check]         write / check dev-docs/PARTS-VERIFY.md
  parts_cli.py tick ID FIELD --by NAME        record a human check of one key value
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
from pathlib import Path

import partsdb as P
import verification as V

VERIFY_SHEET = P.REPO_ROOT / "dev-docs" / "PARTS-VERIFY.md"


def _schema_path(db: Path) -> Path:
    return db / "schema" / "part.schema.json"


def cmd_validate(db: Path) -> int:
    errors, warnings = P.validate_db(db)
    recs, _ = P.load_records(db)
    for rid in sorted(warnings):
        for w in warnings[rid]:
            print(f"warning {rid}: {w}")
    for rid in sorted(errors):
        for e in errors[rid]:
            print(f"ERROR {rid}: {e}", file=sys.stderr)
    print(f"{len(recs)} records, {sum(map(len, errors.values()))} errors, {sum(map(len, warnings.values()))} warnings")
    return 1 if errors else 0


def cmd_search(db: Path, a: argparse.Namespace) -> int:
    compat = dict(kv.split("=", 1) for kv in a.compatible_with.split(",")) if a.compatible_with else None
    recs, _ = P.load_records(db)
    ticks = V.load_ticks(db)
    for rid in P.search(db, a.category, a.text, a.max_mass, a.max_price, compat):
        r = recs[rid]
        print(f"{rid}\t{r['category']}\t{r['manufacturer']} {r['model']}\t{P._val(r, 'mass_kg')} kg\t{V.part_status(r, ticks)}")
    return 0


def cmd_add(db: Path, src: Path) -> int:
    try:
        rec = json.loads(src.read_text(encoding="utf-8"))
        path = P.write_record(rec, db)
    except (OSError, ValueError, KeyError, TypeError) as e:
        print(f"add refused: {e}", file=sys.stderr)
        return 1
    print("added", path)
    return 0


def _write_or_check(path: Path, text: str, check: bool, what: str) -> int:
    if check:
        if not path.exists() or path.read_text(encoding="utf-8") != text:
            print(f"{what} is stale: {path}. Regenerate it.", file=sys.stderr)
            return 1
        print(f"{what} is up to date")
        return 0
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")
    print("wrote", path)
    return 0


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", type=Path, default=P.DEFAULT_DB)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("validate")
    sp = sub.add_parser("search")
    sp.add_argument("--category")
    sp.add_argument("--text")
    sp.add_argument("--max-mass", type=float)
    sp.add_argument("--max-price", type=float)
    sp.add_argument("--compatible-with")
    sub.add_parser("show").add_argument("id")
    sub.add_parser("index")
    sub.add_parser("add").add_argument("--from", dest="src", type=Path, required=True)
    for name in ("schema", "verify-sheet"):
        sub.add_parser(name).add_argument("--check", action="store_true")
    tp = sub.add_parser("tick")
    tp.add_argument("id")
    tp.add_argument("field")
    tp.add_argument("--by", required=True)
    a = ap.parse_args(argv)
    db: Path = a.db

    if a.cmd == "validate":
        return cmd_validate(db)
    if a.cmd == "search":
        return cmd_search(db, a)
    if a.cmd == "show":
        recs, _ = P.load_records(db)
        if a.id not in recs:
            print(f"no part {a.id!r}", file=sys.stderr)
            return 1
        print(P.canonical_json(recs[a.id]), end="")
        return 0
    if a.cmd == "index":
        print("wrote", P.build_index(db))
        return 0
    if a.cmd == "add":
        return cmd_add(db, a.src)
    if a.cmd == "schema":
        return _write_or_check(_schema_path(db), P.canonical_json(P.build_schema()), a.check, "schema")
    if a.cmd == "verify-sheet":
        sheet = VERIFY_SHEET if db == P.DEFAULT_DB else db.parent / "PARTS-VERIFY.md"
        return _write_or_check(sheet, V.render_sheet(db), a.check, "verification sheet")
    if a.cmd == "tick":
        recs, _ = P.load_records(db)
        if a.id not in recs:
            print(f"no part {a.id!r}", file=sys.stderr)
            return 1
        try:
            e = V.add_tick(db, recs[a.id], a.field, a.by, datetime.date.today().isoformat())
        except ValueError as ex:
            print(f"tick refused: {ex}", file=sys.stderr)
            return 1
        print("ticked", e)
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
