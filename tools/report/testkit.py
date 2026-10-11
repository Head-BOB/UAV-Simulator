# Tier: D
"""Test helper: writes a small synthetic campaign folder with chosen numbers (not a real run)."""
from __future__ import annotations

import json
from pathlib import Path

import campaign_data as D

CONDITIONS = {"condition_set": "unit", "ambient_c": 20, "site_altitude_m": 0, "temp_offset_c": 5, "wind_speed_mps": 0, "wind_from_deg": 270,
              "turbulence": "dryden", "gust_level": 0, "seed": 1, "payload_kg": 0, "payload_position_m": [0, 0, 0], "battery_soc": 0.9,
              "battery_temp_c": None}
PARTS = [{"role": "battery", "part_id": "bat_a"}, {"role": "motor", "part_id": "mot_a"}]
PEAK_NAMES = ["winding_c", "case_c", "esc_c", "battery_c", "motor_current_a", "pack_current_a", "min_pack_v", "tilt_rad", "min_thrust_reserve_ratio"]


def series_text(rows: list[dict]) -> str:
    """CSV with every REQUIRED column; `rows` gives overrides per row, other columns are 0."""
    lines = [",".join(D.REQUIRED_COLUMNS)]
    for r in rows:
        lines.append(",".join(repr(float(r.get(c, 0.0))) for c in D.REQUIRED_COLUMNS))
    return "\n".join(lines) + "\n"


def default_peaks() -> dict:
    return {"winding_c": 70.0, "case_c": 60.0, "esc_c": 50.0, "battery_c": 30.0, "motor_current_a": 20.0, "pack_current_a": 60.0,
            "min_pack_v": 13.0, "tilt_rad": 0.1, "min_thrust_reserve_ratio": 2.0}


def write_campaign(root: Path, tests: dict[str, dict], conditions: dict | None = None, drag: str = "analytic_legacy",
                   definitions_set: str = "1.0.0", build: str = "unit_build") -> Path:
    """tests: name -> {"outcome", "metrics": {name: value}, "peaks": {...}, "rows": [...], "notes": [...], "breaking_point": {...}|None}."""
    root.mkdir(parents=True, exist_ok=True)
    cond = dict(CONDITIONS if conditions is None else conditions)
    entries = []
    for name, t in tests.items():
        d = root / name
        d.mkdir()
        rows = t.get("rows") or [{"t_s": 0.0}, {"t_s": 1.0}]
        (d / f"timeseries_{name}.csv").write_text(series_text(rows), encoding="utf-8")
        manifest = {"schema_version": 1, "test": name, "definition_version": "1", "definitions_set_version": definitions_set,
                    "definition": {}, "build": build, "parts": PARTS, "parts_db_version": "0.1.0", "git_hash": "abc", "core_interface_version": 14,
                    "controller_config_version": 1, "step_s": 0.002, "drag_model": drag, "turbulence": "Dryden, UNVERIFIED", "conditions": cond}
        result = {"schema_version": 1, "test": name, "definition_version": "1", "outcome": t.get("outcome", "COMPLETED"),
                  "breaking_point": t.get("breaking_point"), "metrics": [{"name": k, "value": v} for k, v in t.get("metrics", {}).items()],
                  "notes": t.get("notes", []), "series": [{"file": f"timeseries_{name}.csv", "rows": len(rows)}], "steps_simulated": 1,
                  "drag_model": drag}
        peaks = {**default_peaks(), **t.get("peaks", {})}
        events = {"events": [{"name": "motor_over_current", "first_t_s": None, "last_t_s": None, "active_steps": 0, "activations": 0}],
                  "peaks": [{"name": k, "value": peaks[k], "t_s": 1.0} for k in PEAK_NAMES]}
        for fname, doc in (("manifest.json", manifest), ("result.json", result), ("events.json", events)):
            (d / fname).write_text(json.dumps(doc), encoding="utf-8")
        entries.append({"test": name, "outcome": result["outcome"], "result": f"{name}/result.json"})
    top = {"schema_version": 1, "build": build, "definitions_set_version": definitions_set, "git_hash": "abc", "drag_model": drag,
           "turbulence": "Dryden, UNVERIFIED", "tests": entries}
    (root / "campaign_result.json").write_text(json.dumps(top), encoding="utf-8")
    return root


POWERTRAIN = {"motor_max_current_a": 25.74, "esc_max_current_a": 40.0, "battery_max_continuous_a": 125.0, "motor_winding_temp_limit_c": 120.0,
              "esc_temp_limit_c": 100.0, "battery_temp_limit_c": 60.0, "parameters_verified": 0}
