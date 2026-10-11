# Tier: D
"""Parts database library: schema generation, validation, SQLite index, search, compatibility.

The JSON records under `parts_db/parts/<category>/<id>.json` are the source of truth. The SQLite index
is generated and gitignored. Validation is two stages: (1) JSON Schema generated from `fields.py`
(provenance on every number), (2) semantic checks the schema cannot express (ranges between fields,
OCV curve shape, prop data file hash, source pins).

Nothing here fetches anything from the network. See fetch_source.py for that.
"""
from __future__ import annotations

import datetime
import hashlib
import json
import math
import re
import sqlite3
from pathlib import Path
from typing import Any

import jsonschema

import fields as F

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DB = REPO_ROOT / "parts_db"
SHA_RE = r"^[0-9a-f]{64}$"
DATE_RE = r"^[0-9]{4}-[0-9]{2}-[0-9]{2}$"
URL_RE = r"^https?://\S+$"
TODO_RE = r"TODO\(HUMAN-VERIFY"


# --------------------------------------------------------------------------------------------
# Schema generation
# --------------------------------------------------------------------------------------------
def _value_schema(q: F.Q) -> dict:
    if q.vtype in ("number", "integer"):
        s: dict[str, Any] = {"type": q.vtype}
        if q.lo is not None:
            s["exclusiveMinimum" if q.lo_excl else "minimum"] = q.lo
        if q.hi is not None:
            s["maximum"] = q.hi
        return s
    if q.vtype == "vec3":
        item: dict[str, Any] = {"type": "number"}
        if q.lo is not None:
            item["exclusiveMinimum" if q.lo_excl else "minimum"] = q.lo
        return {"type": "array", "items": item, "minItems": 3, "maxItems": 3}
    if q.vtype == "curve":
        pair = {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2}
        return {"type": "array", "items": pair, "minItems": 2}
    raise ValueError(f"unknown vtype {q.vtype!r} for {q.name}")


def _provenance_schema() -> dict:
    return {
        "type": "object",
        "required": ["source_type"],
        "additionalProperties": False,
        "properties": {
            "source_type": {"enum": list(F.SOURCE_TYPES)},
            "source_url": {"type": "string", "pattern": URL_RE},
            "retrieved_on": {"type": "string", "pattern": DATE_RE},
            "sha256_of_file": {"type": "string", "pattern": SHA_RE},
            "note": {"type": "string"},
        },
        "allOf": [
            {"if": {"properties": {"source_type": {"const": "CITED"}}},
             "then": {"required": ["source_url", "retrieved_on"]}},
            {"if": {"properties": {"source_type": {"const": "ASSUMED"}}},
             "then": {"required": ["note"], "properties": {"note": {"pattern": TODO_RE}}}},
            {"if": {"properties": {"source_type": {"const": "CALCULATED"}}},
             "then": {"required": ["note"], "properties": {"note": {"minLength": 10}}}},
            {"if": {"properties": {"source_type": {"const": "MEASURED"}}},
             "then": {"required": ["note", "retrieved_on"], "properties": {"note": {"minLength": 10}}}},
        ],
    }


def _category_schema(category: str) -> dict:
    props: dict[str, Any] = {}
    required: list[str] = []
    for s in F.string_fields(category):
        sch: dict[str, Any] = {"type": "string", "minLength": 1}
        if s.enum:
            sch = {"enum": list(s.enum)}
        if s.name == "category":
            sch = {"const": category}
        if s.name == "schema_version":
            sch = {"const": F.SCHEMA_VERSION}
        props[s.name] = sch
        if s.required:
            required.append(s.name)
    for q in F.quantity_fields(category):
        props[q.name] = {
            "type": "object",
            "required": ["value", "provenance"],
            "additionalProperties": False,
            "properties": {"value": _value_schema(q), "provenance": {"$ref": "#/$defs/provenance"}},
        }
        if q.required:
            required.append(q.name)
    return {"type": "object", "required": required, "additionalProperties": False, "properties": props}


def build_schema() -> dict:
    defs: dict[str, Any] = {"provenance": _provenance_schema()}
    for c in F.CATEGORIES:
        defs[c] = _category_schema(c)
    dispatch = [
        {"if": {"properties": {"category": {"const": c}}, "required": ["category"]},
         "then": {"$ref": f"#/$defs/{c}"}}
        for c in F.CATEGORIES
    ]
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "urn:uav-simulator:parts-db:part:" + F.SCHEMA_VERSION,
        "title": "UAV parts database record",
        "description": "GENERATED from tools/parts/fields.py. Do not edit by hand.",
        "type": "object",
        "required": ["category"],
        "properties": {"category": {"enum": list(F.CATEGORIES)}},
        "allOf": dispatch,
        "$defs": defs,
    }


def canonical_json(obj: Any) -> str:
    """Stable, human-readable text: sorted keys, 2-space indent, LF, trailing newline."""
    return json.dumps(obj, indent=2, sort_keys=True, ensure_ascii=False) + "\n"


# --------------------------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------------------------
def load_pins(db_root: Path) -> dict:
    p = db_root / "source_pins.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}


def load_records(db_root: Path = DEFAULT_DB) -> tuple[dict[str, dict], list[str]]:
    """Return ({id: record}, errors). Errors: unreadable JSON, id != file name, wrong folder, duplicate id."""
    recs: dict[str, dict] = {}
    errors: list[str] = []
    parts_dir = db_root / "parts"
    if not parts_dir.is_dir():
        return recs, [f"{parts_dir}: no parts directory"]
    for folder in sorted(p for p in parts_dir.iterdir() if p.is_dir()):
        if folder.name not in F.CATEGORIES:
            errors.append(f"{folder.name}: unknown category folder")
            continue
        for path in sorted(folder.glob("*.json")):
            try:
                rec = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as e:
                errors.append(f"{path.name}: unreadable JSON ({e})")
                continue
            rid = rec.get("id") if isinstance(rec, dict) else None
            if rid != path.stem:
                errors.append(f"{path.name}: id {rid!r} does not equal the file name")
                continue
            if rec.get("category") != folder.name:
                errors.append(f"{path.name}: category {rec.get('category')!r} is not its folder {folder.name!r}")
                continue
            if rid in recs:
                errors.append(f"{rid}: duplicate id")
                continue
            recs[rid] = rec
    return recs, errors


# --------------------------------------------------------------------------------------------
# Validation
# --------------------------------------------------------------------------------------------
def _path_text(err: jsonschema.ValidationError) -> str:
    return ".".join(str(p) for p in err.absolute_path) or "(record)"


def schema_errors(rec: dict) -> list[str]:
    """JSON Schema errors, each prefixed with the field path."""
    cat = rec.get("category") if isinstance(rec, dict) else None
    if cat not in F.CATEGORIES:
        return [f"(record): category {cat!r} is not one of {sorted(F.CATEGORIES)}"]
    schema = build_schema()
    wrapper = {"$schema": schema["$schema"], "$ref": f"#/$defs/{cat}", "$defs": schema["$defs"]}
    v = jsonschema.Draft202012Validator(wrapper)
    out = []
    for e in sorted(v.iter_errors(rec), key=lambda e: [str(p) for p in e.absolute_path]):
        # Descend into the deepest context so "provenance.note" is named, not the whole if/then.
        out.append(f"{_path_text(e)}: {e.message}")
    return out


def _val(rec: dict, name: str) -> Any:
    q = rec.get(name)
    return q["value"] if isinstance(q, dict) and "value" in q else None


def _is_date(text: str) -> bool:
    try:
        datetime.date.fromisoformat(text)
    except ValueError:
        return False
    return True


def _provenance_semantics(rec: dict, pins: dict) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []
    for q in F.quantity_fields(rec["category"]):
        item = rec.get(q.name)
        if not isinstance(item, dict) or not isinstance(item.get("provenance"), dict):
            continue
        p = item["provenance"]
        ro = p.get("retrieved_on")
        if isinstance(ro, str) and re.match(DATE_RE, ro) and not _is_date(ro):
            errors.append(f"{q.name}.provenance.retrieved_on: {ro!r} is not a calendar date")
        url, sha = p.get("source_url"), p.get("sha256_of_file")
        pin = pins.get(url) if isinstance(url, str) else None
        if pin is not None:
            if p.get("source_type") == "CITED" and sha is None:
                errors.append(f"{q.name}.provenance: source {url} is pinned, so sha256_of_file is required")
            elif sha is not None and sha != pin["sha256"]:
                errors.append(f"{q.name}.provenance.sha256_of_file does not equal the pin for {url}")
        elif p.get("source_type") == "CITED" and isinstance(url, str):
            warnings.append(f"{q.name}: CITED source {url} has no fetched pin (value read but file not hashed)")
        if sha is not None and url is None:
            errors.append(f"{q.name}.provenance: sha256_of_file given without source_url")
    return errors, warnings


def _check_range_pair(rec: dict, lo: str, hi: str, errors: list[str]) -> None:
    a, b = _val(rec, lo), _val(rec, hi)
    if a is not None and b is not None and a > b:
        errors.append(f"{lo} ({a}) must not exceed {hi} ({b})")


def _curve_errors(rec: dict) -> list[str]:
    curve = _val(rec, "ocv_curve")
    if not isinstance(curve, list) or len(curve) < 2:
        return []
    errs = []
    socs = [pt[0] for pt in curve]
    ocvs = [pt[1] for pt in curve]
    if socs[0] != 0.0 or socs[-1] != 1.0:
        errs.append(f"ocv_curve must start at soc 0 and end at soc 1 (got {socs[0]} .. {socs[-1]})")
    if any(b <= a for a, b in zip(socs, socs[1:])):
        errs.append("ocv_curve soc values must be strictly increasing")
    if any(b < a for a, b in zip(ocvs, ocvs[1:])):
        errs.append("ocv_curve voltage must be non-decreasing with soc")
    vmax = _val(rec, "voltage_per_cell_max_v")
    if any(v <= 0 or (vmax is not None and v > vmax + 1e-9) for v in ocvs):
        errs.append(f"ocv_curve voltages must lie in (0, voltage_per_cell_max_v={vmax}]")
    return errs


def _prop_errors(rec: dict, repo_root: Path) -> list[str]:
    errs = []
    basis = rec.get("data_basis")
    if basis == "UIUC_MEASURED":
        df, sha = rec.get("data_file"), rec.get("data_file_sha256")
        if not df or not sha:
            return ["data_basis UIUC_MEASURED requires data_file and data_file_sha256"]
        if not re.match(SHA_RE, sha):
            return ["data_file_sha256 is not 64 lowercase hex characters"]
        path = repo_root / df
        if not path.is_file():
            errs.append(f"data_file {df} does not exist")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != sha:
            errs.append(f"data_file {df} sha256 does not match data_file_sha256")
    elif basis == "SCALED":
        if not rec.get("scaled_from"):
            errs.append("data_basis SCALED requires scaled_from (id of a UIUC_MEASURED propeller)")
        sc = rec.get("ct_cp_scaling_assumed")
        if not isinstance(sc, dict) or sc.get("provenance", {}).get("source_type") != "ASSUMED":
            errs.append("data_basis SCALED requires ct_cp_scaling_assumed with source_type ASSUMED")
    return errs


def _frame_errors(rec: dict) -> list[str]:
    errs = []
    od, idd = _val(rec, "tube_od_m"), _val(rec, "tube_id_m")
    w, t = _val(rec, "arm_plate_width_m"), _val(rec, "arm_plate_thickness_m")
    tube = od is not None and idd is not None
    plate = w is not None and t is not None
    if (od is None) != (idd is None):
        errs.append("tube_od_m and tube_id_m must be given together")
    if (w is None) != (t is None):
        errs.append("arm_plate_width_m and arm_plate_thickness_m must be given together")
    if not tube and not plate:
        errs.append("frame needs an arm section: tube_od_m + tube_id_m, or arm_plate_width_m + arm_plate_thickness_m")
    if tube and idd >= od:
        errs.append(f"tube_id_m ({idd}) must be smaller than tube_od_m ({od})")
    return errs


def semantic_errors(rec: dict, repo_root: Path) -> list[str]:
    cat = rec["category"]
    errs: list[str] = []
    if cat == "frame":
        errs += _frame_errors(rec)
    elif cat == "motor":
        _check_range_pair(rec, "cells_min", "cells_max", errs)
        _check_range_pair(rec, "prop_diameter_min_m", "prop_diameter_max_m", errs)
    elif cat == "esc":
        _check_range_pair(rec, "cells_min", "cells_max", errs)
        _check_range_pair(rec, "max_continuous_current_a", "burst_current_a", errs)
    elif cat == "propeller":
        errs += _prop_errors(rec, repo_root)
        _check_range_pair(rec, "measured_rpm_min", "measured_rpm_max", errs)
    elif cat == "battery":
        c, n, m = (_val(rec, k) for k in ("voltage_per_cell_cutoff_v", "voltage_per_cell_nominal_v", "voltage_per_cell_max_v"))
        if None not in (c, n, m) and not (c < n < m):
            errs.append(f"need cutoff ({c}) < nominal ({n}) < max ({m}) voltage per cell")
        errs += _curve_errors(rec)
    mass, mass_nc = _val(rec, "mass_kg"), _val(rec, "mass_without_cables_kg")
    if mass is not None and mass_nc is not None and mass_nc > mass:
        errs.append(f"mass_without_cables_kg ({mass_nc}) must not exceed mass_kg ({mass})")
    for name in [q.name for q in F.quantity_fields(cat)]:
        v = _val(rec, name)
        if isinstance(v, (int, float)) and not math.isfinite(v):
            errs.append(f"{name}: value is not finite")
    return errs


def validate_record(rec: dict, db_root: Path = DEFAULT_DB, repo_root: Path | None = None) -> tuple[list[str], list[str]]:
    """Return (errors, warnings). Errors block the record; warnings are shown."""
    repo_root = repo_root or db_root.parent
    errs = schema_errors(rec)
    if errs:
        return errs, []
    sem = semantic_errors(rec, repo_root)
    perr, warn = _provenance_semantics(rec, load_pins(db_root))
    return sem + perr, warn


def validate_db(db_root: Path = DEFAULT_DB) -> tuple[dict[str, list[str]], dict[str, list[str]]]:
    """Validate every record plus cross-record rules. Returns ({id: errors}, {id: warnings}); '(db)' = global."""
    recs, load_errs = load_records(db_root)
    errors: dict[str, list[str]] = {}
    warnings: dict[str, list[str]] = {}
    if load_errs:
        errors["(db)"] = load_errs
    for rid, rec in recs.items():
        e, w = validate_record(rec, db_root)
        if rec.get("category") == "propeller" and rec.get("data_basis") == "SCALED":
            src = recs.get(rec.get("scaled_from"))
            if src is None or src.get("data_basis") != "UIUC_MEASURED":
                e = e + [f"scaled_from {rec.get('scaled_from')!r} is not a UIUC_MEASURED propeller in the database"]
        if e:
            errors[rid] = e
        if w:
            warnings[rid] = w
    return errors, warnings


# --------------------------------------------------------------------------------------------
# Index and search
# --------------------------------------------------------------------------------------------
def db_fingerprint(db_root: Path) -> str:
    h = hashlib.sha256()
    for p in sorted((db_root / "parts").rglob("*.json")):
        h.update(p.relative_to(db_root).as_posix().encode())
        h.update(p.read_bytes())
    return h.hexdigest()


def build_index(db_root: Path = DEFAULT_DB, index_path: Path | None = None) -> Path:
    """(Re)build the SQLite index from the JSON files. Refuses an invalid database."""
    errors, _ = validate_db(db_root)
    if errors:
        raise ValueError("database does not validate; not indexing: " + "; ".join(sorted(errors)))
    recs, _ = load_records(db_root)
    index_path = index_path or (db_root / "parts_index.sqlite")
    if index_path.exists():
        index_path.unlink()
    con = sqlite3.connect(index_path)
    try:
        con.execute("CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT)")
        con.execute("CREATE TABLE parts (id TEXT PRIMARY KEY, category TEXT, manufacturer TEXT, model TEXT,"
                    " mass_kg REAL, price_usd REAL, text TEXT)")
        con.execute("INSERT INTO meta VALUES ('fingerprint', ?)", (db_fingerprint(db_root),))
        for rid, rec in sorted(recs.items()):
            text = " ".join(str(x) for x in (rid, rec["category"], rec["manufacturer"], rec["model"], rec.get("notes", ""))).lower()
            con.execute("INSERT INTO parts VALUES (?,?,?,?,?,?,?)",
                        (rid, rec["category"], rec["manufacturer"], rec["model"],
                         _val(rec, "mass_kg"), _val(rec, "price_usd"), text))
        con.commit()
    finally:
        con.close()
    return index_path


def ensure_index(db_root: Path = DEFAULT_DB) -> Path:
    path = db_root / "parts_index.sqlite"
    if path.exists():
        con = sqlite3.connect(path)
        try:
            row = con.execute("SELECT value FROM meta WHERE key='fingerprint'").fetchone()
        except sqlite3.DatabaseError:
            row = None
        finally:
            con.close()
        if row and row[0] == db_fingerprint(db_root):
            return path
    return build_index(db_root, path)


def incompatibilities(a: dict, b: dict) -> list[str]:
    """Hard part-to-part violations between two records (empty list = compatible or no rule applies).
    Only physical/electrical limits from the records; the full build checker (T/W, hover current) is P2."""
    pair = {a["category"]: a, b["category"]: b}
    out: list[str] = []
    motor, esc, prop, bat, frame = (pair.get(k) for k in ("motor", "esc", "propeller", "battery", "frame"))
    if motor and bat:
        n = _val(bat, "cells")
        if not _val(motor, "cells_min") <= n <= _val(motor, "cells_max"):
            out.append(f"{bat['id']} is {n}S; {motor['id']} accepts {_val(motor, 'cells_min')}-{_val(motor, 'cells_max')}S")
    if esc and bat:
        n = _val(bat, "cells")
        if not _val(esc, "cells_min") <= n <= _val(esc, "cells_max"):
            out.append(f"{bat['id']} is {n}S; {esc['id']} accepts {_val(esc, 'cells_min')}-{_val(esc, 'cells_max')}S")
    if motor and esc and _val(esc, "max_continuous_current_a") < _val(motor, "max_current_a"):
        out.append(f"{esc['id']} continuous {_val(esc, 'max_continuous_current_a')} A < {motor['id']} max {_val(motor, 'max_current_a')} A")
    if motor and prop:
        d = _val(prop, "diameter_m")
        if not _val(motor, "prop_diameter_min_m") <= d <= _val(motor, "prop_diameter_max_m"):
            out.append(f"{prop['id']} {d} m outside {motor['id']} prop range")
        if _val(prop, "bore_m") < _val(motor, "shaft_diameter_m"):
            out.append(f"{prop['id']} bore {_val(prop, 'bore_m')} m < {motor['id']} shaft {_val(motor, 'shaft_diameter_m')} m")
    if motor and frame and motor["mount_pattern"] != frame["motor_mount_pattern"]:
        out.append(f"mount pattern {motor['mount_pattern']!r} ({motor['id']}) != {frame['motor_mount_pattern']!r} ({frame['id']})")
    if prop and frame and _val(prop, "diameter_m") > _val(frame, "max_prop_diameter_m"):
        out.append(f"{prop['id']} {_val(prop, 'diameter_m')} m > {frame['id']} max {_val(frame, 'max_prop_diameter_m')} m")
    return out


def search(db_root: Path = DEFAULT_DB, category: str | None = None, text: str | None = None,
           mass_max_kg: float | None = None, price_max_usd: float | None = None,
           compatible_with: dict[str, str] | None = None) -> list[str]:
    """Sorted part ids. `compatible_with` maps category -> part id of the current build; a candidate
    is dropped if it violates a hard rule against any of them (a part of the same category is skipped)."""
    path = ensure_index(db_root)
    sql, args = "SELECT id FROM parts WHERE 1=1", []
    if category:
        sql += " AND category = ?"
        args.append(category)
    for word in (text or "").lower().split():
        sql += " AND text LIKE ?"
        args.append(f"%{word}%")
    if mass_max_kg is not None:
        sql += " AND mass_kg <= ?"
        args.append(mass_max_kg)
    if price_max_usd is not None:
        sql += " AND price_usd IS NOT NULL AND price_usd <= ?"
        args.append(price_max_usd)
    con = sqlite3.connect(path)
    try:
        ids = [r[0] for r in con.execute(sql + " ORDER BY id", args)]
    finally:
        con.close()
    if compatible_with:
        recs, _ = load_records(db_root)
        build = [recs[i] for i in compatible_with.values()]
        ids = [i for i in ids if all(not incompatibilities(recs[i], b) for b in build
                                     if b["category"] != recs[i]["category"])]
    return ids


def write_record(rec: dict, db_root: Path = DEFAULT_DB) -> Path:
    """Validate then write a new record. Refuses invalid records and existing ids."""
    errs, _ = validate_record(rec, db_root)
    if errs:
        raise ValueError("record refused: " + "; ".join(errs))
    recs, _ = load_records(db_root)
    if rec["id"] in recs:
        raise ValueError(f"record refused: id {rec['id']!r} already exists")
    if not re.match(r"^[a-z0-9][a-z0-9_]*$", rec["id"]):
        raise ValueError("record refused: id must be lowercase letters, digits and underscores")
    path = db_root / "parts" / rec["category"] / f"{rec['id']}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(canonical_json(rec), encoding="utf-8", newline="\n")
    return path
