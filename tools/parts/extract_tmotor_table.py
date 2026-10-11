# Tier: D
"""Extracts the MN3508 KV700 bench-test table from the SAVED T-Motor store page into `parts_db/validation/tmotor_mn3508_kv700_table.json`.

The page text is read from the pinned cache file (sha256 checked against `parts_db/source_pins.json`); nothing is fetched. The table is
printed as a flat run of tokens: supply voltage, `T-MOTOR <prop>`, then rows `<throttle>% <A> <W> <g> <rpm> <g/W> [<temp C>]`.
Every row is checked against itself (power = V x I, efficiency = thrust / power) before it is written.

Usage: python tools/parts/extract_tmotor_table.py [--check]
  --check  exit 1 if the committed file differs from a fresh extraction. Without the (git-ignored) cache it only verifies the committed
           file's own checks and that its recorded sha256 equals the pin, and says so.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import partsdb as P
import validation_files as VF

PAGE_URL = "https://store.tmotor.com/goods-354-MN3508.html"
START_MARKER = "MN3508 KV700 11.1 T-MOTOR"       # the KV700 test table starts at its first supply voltage
END_MARKER = "Notes:"
EXPECTED_ROWS = 20
OUT_NAME = VF.MOTOR_TABLE_FILE


class ExtractError(ValueError):
    """The page text does not hold the table in the expected form."""


def page_text(html: str) -> str:
    """Visible text of the page: scripts / styles dropped, tags replaced by spaces, entities of the degree sign kept as text, whitespace folded."""
    no_code = re.sub(r"<script.*?</script>|<style.*?</style>", "", html, flags=re.S)
    text = re.sub(r"<[^>]+>", " ", no_code)
    return re.sub(r"\s+", " ", text)


def _is_number(tok: str) -> bool:
    return re.fullmatch(r"\d+(\.\d+)?", tok) is not None


def parse_rows(text: str) -> list:
    """Rows of the KV700 table from the page text. Raises ExtractError when the markers or the token pattern are missing."""
    start = text.find(START_MARKER)
    if start < 0:
        raise ExtractError(f"marker {START_MARKER!r} not found")
    end = text.find(END_MARKER, start)          # the KV700 table (both supply voltages) ends at the next 'Notes:'
    if end < 0:
        raise ExtractError("end of table not found")
    tokens = text[start + len("MN3508 KV700 "):end].split()
    rows, voltage, prop, i = [], None, None, 0
    while i < len(tokens):
        tok = tokens[i]
        if _is_number(tok) and i + 1 < len(tokens) and tokens[i + 1] == "T-MOTOR":
            voltage = float(tok)
            i += 1
        elif tok == "T-MOTOR":
            prop = tokens[i + 1]
            i += 2
        elif tok.endswith("%") and _is_number(tok[:-1]):
            vals = tokens[i + 1:i + 6]
            if voltage is None or prop is None or len(vals) < 5 or not all(_is_number(v) for v in vals):
                raise ExtractError(f"cannot read the row at token {i}: {tokens[i:i + 7]}")
            i += 6
            temp = None
            nxt = tokens[i] if i < len(tokens) else None
            after = tokens[i + 1] if i + 1 < len(tokens) else None
            if nxt is not None and _is_number(nxt) and after is not None and after.endswith("%"):
                temp = float(nxt)                    # only the first row of each prop carries the operating temperature
                i += 1
            rows.append((voltage, prop, float(tok[:-1]), *[float(v) for v in vals], temp))
        else:
            raise ExtractError(f"unexpected token {tok!r} at {i}")
    return rows


def to_document(rows: list, pin: dict) -> dict:
    out = []
    for v, prop, thr, cur, pwr, thr_g, rpm, eff, temp in rows:
        m = re.fullmatch(r"(\d+(?:\.\d+)?)\*(\d+(?:\.\d+)?)CF", prop)
        if not m:
            raise ExtractError(f"cannot read the propeller size from {prop!r}")
        out.append({"id": f"kv700_{v:g}v_{prop.lower().replace('*', 'x')}_{thr:g}pct", "supply_voltage_v": v, "prop": f"T-MOTOR {prop}",
                    "prop_diameter_in": float(m.group(1)), "prop_pitch_in": float(m.group(2)), "throttle_percent": thr, "current_a": cur,
                    "power_w": pwr, "thrust_g": thr_g, "rpm": rpm, "efficiency_g_per_w": eff, "temperature_c": temp})
    doc = {"schema_version": VF.SCHEMA_VERSION, "motor_id": "tmotor_mn3508_kv700",
           "description": "Bench test data of the T-Motor MN3508 KV700 as printed on the manufacturer's store page (Test Data table).",
           "source": {"url": PAGE_URL, "cache_file": pin["cache_file"], "sha256": pin["sha256"], "retrieved_on": pin["retrieved_on"], "bytes": pin["bytes"]},
           "provenance": {"source_type": "CITED",
                          "note": "Typed by tools/parts/extract_tmotor_table.py from the saved page, not read from the live site. TODO(HUMAN-VERIFY): compare a few rows with the live page. The propellers are T-MOTOR carbon props with no Ct/Cp data: thrust and efficiency are not predictable by the model."},
           "units": {"supply_voltage_v": "V", "current_a": "A", "power_w": "W", "thrust_g": "gram-force", "rpm": "rev/min", "efficiency_g_per_w": "gf/W",
                     "temperature_c": "deg C at 100 % throttle after 10 min (first row of each prop only, else null)"},
           "rows": out}
    VF.check_motor_table(doc)
    return doc


def extract(db_root: Path = P.DEFAULT_DB) -> dict:
    """Fresh extraction from the pinned cache file. Raises ExtractError on a missing cache, hash mismatch or wrong row count."""
    pins = json.loads((db_root / "source_pins.json").read_text(encoding="utf-8"))
    pin = pins.get(PAGE_URL)
    if pin is None:
        raise ExtractError(f"{PAGE_URL} is not in source_pins.json")
    cache = db_root / "source_cache" / pin["cache_file"]
    if not cache.exists():
        raise ExtractError(f"cache file {cache} missing")
    raw = cache.read_bytes()
    if hashlib.sha256(raw).hexdigest() != pin["sha256"]:
        raise ExtractError("cache file does not match the pinned sha256")
    rows = parse_rows(page_text(raw.decode("utf-8", errors="ignore")))
    if len(rows) != EXPECTED_ROWS:
        raise ExtractError(f"{len(rows)} rows found, expected {EXPECTED_ROWS}")
    return to_document(rows, pin)


def render(doc: dict) -> str:
    return P.canonical_json(doc)


def check(db_root: Path = P.DEFAULT_DB) -> tuple:
    """(ok, message). Fresh comparison if the cache exists, else the committed file's own checks and its pin."""
    path = db_root / "validation" / OUT_NAME
    if not path.exists():
        return False, f"{path} is missing"
    committed = path.read_text(encoding="utf-8")
    cache_present = any((db_root / "source_cache").glob("*")) if (db_root / "source_cache").exists() else False
    if cache_present:
        try:
            fresh = render(extract(db_root))
        except ExtractError as e:
            return False, f"extraction failed: {e}"
        return (committed == fresh), ("up to date (fresh extraction compared)" if committed == fresh else "stale: re-run the extractor")
    try:
        doc = json.loads(committed)
        VF.check_motor_table(doc)
        pins = json.loads((db_root / "source_pins.json").read_text(encoding="utf-8"))
        if pins.get(PAGE_URL, {}).get("sha256") != doc["source"]["sha256"]:
            return False, "recorded sha256 differs from source_pins.json"
        if len(doc["rows"]) != EXPECTED_ROWS:
            return False, f"{len(doc['rows'])} rows, expected {EXPECTED_ROWS}"
    except (ValueError, KeyError) as e:
        return False, f"committed file invalid: {e}"
    return True, "no source cache here: only the committed file's own checks and its pin were verified"


def main(argv: list) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--db", type=Path, default=P.DEFAULT_DB)
    a = ap.parse_args(argv)
    if a.check:
        ok, msg = check(a.db)
        print(msg)
        return 0 if ok else 1
    try:
        doc = extract(a.db)
    except ExtractError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    out = a.db / "validation" / OUT_NAME
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(render(doc), encoding="utf-8", newline="\n")
    print(f"wrote {out} ({len(doc['rows'])} rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
