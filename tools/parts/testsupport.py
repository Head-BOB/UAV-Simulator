# Tier: D
"""Test helpers: build valid synthetic records for any category from the field spec, and temp databases.
Synthetic records are for TESTS ONLY (never written to parts_db/)."""
from __future__ import annotations

import copy
import json
import shutil
import tempfile
from pathlib import Path

import fields as F
import partsdb as P

TODO = "TODO(HUMAN-VERIFY): synthetic test value"

# Coherent overrides so the semantic rules pass. Everything else is generated from the spec.
OVERRIDES: dict[str, dict] = {
    "frame": {"tube_od_m": 0.016, "tube_id_m": 0.013, "arm_count": 4, "motor_spacing_adjacent_m": 0.4,
              "arm_length_m": 0.28, "max_prop_diameter_m": 0.4},
    "motor": {"kv_rpm_per_v": 700.0, "cells_min": 3, "cells_max": 4, "prop_diameter_min_m": 0.25,
              "prop_diameter_max_m": 0.4, "shaft_diameter_m": 0.004, "max_current_a": 20.0},
    "esc": {"cells_min": 3, "cells_max": 6, "max_continuous_current_a": 30.0, "efficiency": 0.95, "channels": 1},
    "propeller": {"diameter_m": 0.3, "bore_m": 0.006, "blades": 2},
    "battery": {"cells": 4, "voltage_per_cell_nominal_v": 3.7, "voltage_per_cell_max_v": 4.2,
                "voltage_per_cell_cutoff_v": 3.3,
                "ocv_curve": [[0.0, 3.3], [0.5, 3.8], [1.0, 4.2]]},
}


def _value(q: F.Q, category: str):
    ov = OVERRIDES.get(category, {})
    if q.name in ov:
        return ov[q.name]
    if q.vtype == "vec3":
        return [0.1, 0.1, 0.1] if q.lo is not None else [0.0, 0.0, 0.0]
    if q.vtype == "curve":
        return [[0.0, 1.0], [1.0, 2.0]]
    if q.vtype == "integer":
        return int(q.lo) if q.lo is not None else 1
    if q.hi is not None and q.hi <= 1.0:
        return 0.5
    base = (q.lo or 0.0)
    return base + 1.0


def quantity(value, kind: str = "ASSUMED") -> dict:
    if kind == "ASSUMED":
        return {"value": value, "provenance": {"source_type": "ASSUMED", "note": TODO}}
    if kind == "CALCULATED":
        return {"value": value, "provenance": {"source_type": "CALCULATED", "note": "test formula a * b from inputs"}}
    raise ValueError(kind)


def synth(category: str, rid: str | None = None, **over) -> dict:
    """A valid record of `category`. `over` replaces plain values (wrapped as ASSUMED) or whole quantities (dict)."""
    rec: dict = {"id": rid or f"test_{category}", "category": category, "schema_version": F.SCHEMA_VERSION,
                 "manufacturer": "TestCo", "model": f"{category} model"}
    for s in F.CATEGORIES[category]["s"]:
        if s.required:
            rec[s.name] = s.enum[0] if s.enum else f"{s.name}_x"
    for q in F.quantity_fields(category):
        if q.required:
            rec[q.name] = quantity(_value(q, category))
    if category == "frame":
        rec["tube_od_m"] = quantity(OVERRIDES["frame"]["tube_od_m"])
        rec["tube_id_m"] = quantity(OVERRIDES["frame"]["tube_id_m"])
    if category == "propeller":
        rec["data_basis"] = "SCALED"
        rec["scaled_from"] = "test_ref_prop"
        rec["ct_cp_scaling_assumed"] = quantity(1.0)
    for k, v in over.items():
        if v is None:
            rec.pop(k, None)
        elif isinstance(v, dict) and "provenance" in v:
            rec[k] = v
        elif k in {q.name for q in F.quantity_fields(category)}:
            rec[k] = quantity(v)
        else:
            rec[k] = v
    return rec


class TempDb:
    """A throw-away database directory (db_root = <tmp>/parts_db, repo_root = <tmp>)."""

    def __init__(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self._tmp.name)
        self.root = self.repo / "parts_db"
        (self.root / "parts").mkdir(parents=True)
        (self.root / "allowed_sources.json").write_text(json.dumps({"hosts": ["example.com"]}), encoding="utf-8")
        (self.root / "source_pins.json").write_text("{}\n", encoding="utf-8")

    def add(self, rec: dict) -> Path:
        p = self.root / "parts" / rec["category"] / f"{rec['id']}.json"
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(P.canonical_json(rec), encoding="utf-8", newline="\n")
        return p

    def add_ref_prop(self, rid: str = "test_ref_prop") -> None:
        """A UIUC_MEASURED propeller (with a real data file + hash) that synthetic SCALED props can point at."""
        data = b'{"test": "prop table"}'
        f = self.repo / "data" / "ref_prop.json"
        f.parent.mkdir(exist_ok=True)
        f.write_bytes(data)
        import hashlib
        self.add(synth("propeller", rid, data_basis="UIUC_MEASURED", data_file="data/ref_prop.json",
                       data_file_sha256=hashlib.sha256(data).hexdigest(), scaled_from=None,
                       ct_cp_scaling_assumed=None))

    def pins(self, pins: dict) -> None:
        (self.root / "source_pins.json").write_text(json.dumps(pins), encoding="utf-8")

    def close(self) -> None:
        self._tmp.cleanup()


def deepcopy(x):
    return copy.deepcopy(x)
