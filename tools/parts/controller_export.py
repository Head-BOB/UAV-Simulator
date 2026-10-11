# Tier: D
"""Builds `controller_config.json` (the Rust controller's `ControllerConfig`) from `parts_db/controller_classes.json`.

Every value in that file is ASSUMED (no source exists); the build copies the entry of the vehicle class and adds the class name and
the provenance. The ranges below mirror `physics_core::controller::validate_config` and are checked here so a bad class file fails
at build time, not when the core refuses to load it.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

CONFIG_VERSION = 1                       # must equal physics_core controller::CONTROLLER_CONFIG_VERSION
GRAVITY_MPS2 = 9.80665
MAX_TILT_LIMIT_RAD = 1.3962634015954637  # 80 deg, same as the core
MAX_DOWN_ACCEL_FRACTION_G = 0.9
MAX_AUTHORITY = 0.5
CLASSES_FILE = "controller_classes.json"

POSITIVE = ("max_rate_rps", "max_yaw_rate_rps", "max_speed_h_mps", "max_accel_h_mps2", "max_climb_mps", "max_descent_mps",
            "max_accel_up_mps2", "max_accel_down_mps2", "position_kp_per_s", "velocity_kp_per_s", "velocity_ki_per_s2",
            "accel_integral_limit_mps2", "attitude_kp_per_s", "yaw_kp_per_s", "rate_bandwidth_rps", "yaw_rate_bandwidth_rps",
            "rate_integral_corner_rps", "rate_integral_limit", "acro_max_rate_rps", "acro_max_yaw_rate_rps", "mixer_axis_authority")
FIELDS = ("max_tilt_rad", "rate_derivative_time_s", "acro_expo") + POSITIVE


class ControllerExportError(ValueError):
    """The class file is missing, malformed or out of range (rule E-CONTROLLER)."""


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def validate_class(name: str, cfg: dict) -> None:
    """Raises ControllerExportError unless `cfg` has exactly the fields of the core's config and every range holds."""
    if set(cfg) != set(FIELDS):
        raise ControllerExportError(f"class {name}: fields differ from the core's list: {sorted(set(cfg) ^ set(FIELDS))}")
    bad = [k for k in FIELDS if not _finite(cfg[k])]
    if bad:
        raise ControllerExportError(f"class {name}: non-numeric or non-finite: {bad}")
    bad = [k for k in POSITIVE if cfg[k] <= 0.0]
    if bad:
        raise ControllerExportError(f"class {name}: must be > 0: {bad}")
    if not 0.0 < cfg["max_tilt_rad"] <= MAX_TILT_LIMIT_RAD:
        raise ControllerExportError(f"class {name}: max_tilt_rad must be in (0, 80 deg]")
    if cfg["max_accel_down_mps2"] >= MAX_DOWN_ACCEL_FRACTION_G * GRAVITY_MPS2:
        raise ControllerExportError(f"class {name}: max_accel_down_mps2 must stay below 0.9 g")
    if cfg["rate_derivative_time_s"] < 0.0 or not 0.0 <= cfg["acro_expo"] <= 1.0 or cfg["mixer_axis_authority"] > MAX_AUTHORITY:
        raise ControllerExportError(f"class {name}: derivative time, expo or authority out of range")


def build_controller(db_root: Path, vehicle_class: str, hover_duty: float) -> dict:
    """The `controller_config.json` content for a vehicle class. `hover_duty` is the build's own hover duty estimate (CALCULATED,
    static-propeller hover point of tools/parts/estimates.py), in (0, 1]."""
    if not (_finite(hover_duty) and 0.0 < hover_duty <= 1.0):
        raise ControllerExportError(f"hover duty {hover_duty!r} must be in (0, 1]")
    path = db_root / CLASSES_FILE
    try:
        raw = path.read_bytes()
        doc = json.loads(raw.decode("utf-8"))
    except (OSError, ValueError) as e:
        raise ControllerExportError(f"cannot read {path}: {e}") from e
    if doc.get("version") != CONFIG_VERSION:
        raise ControllerExportError(f"{CLASSES_FILE}: version {doc.get('version')!r}, expected {CONFIG_VERSION}")
    classes = doc.get("classes")
    if not isinstance(classes, dict) or vehicle_class not in classes:
        raise ControllerExportError(f"{CLASSES_FILE}: no entry for vehicle class {vehicle_class!r}")
    entry = classes[vehicle_class]
    validate_class(vehicle_class, entry)
    out = {"config_version": CONFIG_VERSION, "vehicle_class": vehicle_class, "hover_duty": hover_duty}
    out.update({k: entry[k] for k in FIELDS})
    out["provenance"] = {"source_file": f"parts_db/{CLASSES_FILE}", "source_sha256": hashlib.sha256(raw).hexdigest(),
                         **doc["provenance"]}
    return out
