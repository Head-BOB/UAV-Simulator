# Tier: D
"""Shape and range checks of the validation reference files in `parts_db/validation/` (mirrors what `physics_core::validation` loads).

Files: `tmotor_mn3508_kv700_table.json` (published bench rows) and `acceptance_bands.json` (percent bands, ASSUMED).
"""
from __future__ import annotations

import math

SCHEMA_VERSION = 1
MOTOR_TABLE_FILE = "tmotor_mn3508_kv700_table.json"
BANDS_FILE = "acceptance_bands.json"
ROW_KEYS = {"id", "supply_voltage_v", "prop", "prop_diameter_in", "prop_pitch_in", "throttle_percent", "current_a", "power_w",
            "thrust_g", "rpm", "efficiency_g_per_w", "temperature_c"}
POWER_TOLERANCE = 0.005        # published power vs V x I (relative)
EFFICIENCY_TOLERANCE = 0.01    # published efficiency vs thrust / power (relative)
BAND_QUANTITIES = ("motor_voltage_percent", "motor_power_self_consistency_percent", "uiuc_ct_percent", "uiuc_cp_percent",
                   "battery_ocv_percent", "battery_sag_percent", "battery_soc_percent", "battery_curve_percent")


class ValidationFileError(ValueError):
    """A reference file is malformed or out of range."""


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def check_row(row: dict) -> None:
    """Raises unless the row has exactly the documented keys, positive values, and its own numbers agree."""
    if set(row) != ROW_KEYS:
        raise ValidationFileError(f"row {row.get('id')}: keys differ: {sorted(set(row) ^ ROW_KEYS)}")
    for k in ("supply_voltage_v", "prop_diameter_in", "prop_pitch_in", "throttle_percent", "current_a", "power_w", "thrust_g", "rpm", "efficiency_g_per_w"):
        if not _num(row[k]) or row[k] <= 0:
            raise ValidationFileError(f"row {row['id']}: {k} must be a positive number")
    if row["temperature_c"] is not None and not _num(row["temperature_c"]):
        raise ValidationFileError(f"row {row['id']}: temperature_c must be a number or null")
    expect_p = row["supply_voltage_v"] * row["current_a"]
    if abs(row["power_w"] - expect_p) > POWER_TOLERANCE * expect_p:
        raise ValidationFileError(f"row {row['id']}: power {row['power_w']} W differs from V x I = {expect_p:.3f} W")
    expect_e = row["thrust_g"] / row["power_w"]
    if abs(row["efficiency_g_per_w"] - expect_e) > EFFICIENCY_TOLERANCE * expect_e:
        raise ValidationFileError(f"row {row['id']}: efficiency {row['efficiency_g_per_w']} differs from thrust / power = {expect_e:.3f}")


def check_motor_table(doc: dict) -> None:
    if doc.get("schema_version") != SCHEMA_VERSION:
        raise ValidationFileError(f"motor table: schema_version {doc.get('schema_version')!r}")
    for key in ("motor_id", "source", "provenance", "rows"):
        if key not in doc:
            raise ValidationFileError(f"motor table: missing {key}")
    src = doc["source"]
    if not isinstance(src.get("sha256"), str) or len(src["sha256"]) != 64:
        raise ValidationFileError("motor table: source.sha256 must be 64 hex characters")
    if doc["provenance"].get("source_type") != "CITED":
        raise ValidationFileError("motor table: provenance.source_type must be CITED")
    rows = doc["rows"]
    if not rows:
        raise ValidationFileError("motor table: no rows")
    ids = [r.get("id") for r in rows]
    if len(set(ids)) != len(ids):
        raise ValidationFileError("motor table: duplicate row ids")
    for r in rows:
        check_row(r)


def check_bands(doc: dict) -> None:
    if doc.get("schema_version") != SCHEMA_VERSION:
        raise ValidationFileError(f"bands: schema_version {doc.get('schema_version')!r}")
    if doc.get("provenance", {}).get("source_type") != "ASSUMED":
        raise ValidationFileError("bands: provenance.source_type must be ASSUMED (no source exists)")
    bands = doc.get("bands_percent")
    if not isinstance(bands, dict):
        raise ValidationFileError("bands: bands_percent missing")
    for k, v in bands.items():
        if k not in BAND_QUANTITIES:
            raise ValidationFileError(f"bands: unknown quantity {k!r}")
        if not _num(v) or v <= 0:
            raise ValidationFileError(f"bands: {k} must be a positive number")
