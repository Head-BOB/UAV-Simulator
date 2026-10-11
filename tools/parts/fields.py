# Tier: D
"""Field specification for the parts database: the single source for the JSON Schema, the CSV
templates and the verification sheet. `python tools/parts/parts_cli.py schema` regenerates
`parts_db/schema/part.schema.json` from this file; a test fails if the two differ.

Rules (see dev-docs/PARTS-DB.md):
 - A numeric field is a QUANTITY: {"value": ..., "provenance": {...}}. A bare number fails validation.
 - Units are SI and the unit is part of the field name (`mass_kg`, `kv_rpm_per_v`).
 - Plain strings (ids, labels, patterns, URLs) carry no provenance.
 - vtype: "number", "integer", "vec3" (3 numbers), "curve" (list of [x, y] pairs).
"""
from __future__ import annotations

from typing import NamedTuple

SCHEMA_VERSION = "1.0.0"
SOURCE_TYPES = ("MEASURED", "CITED", "CALCULATED", "ASSUMED")


class Q(NamedTuple):
    """One quantity field. lo/hi are inclusive unless lo_excl is set (then lo is exclusive)."""

    name: str
    vtype: str
    required: bool = True
    lo: float | None = None
    lo_excl: bool = False
    hi: float | None = None


class S(NamedTuple):
    """One plain string field."""

    name: str
    required: bool = True
    enum: tuple[str, ...] | None = None


COMMON_STRINGS = [
    S("id"), S("category"), S("manufacturer"), S("model"), S("schema_version"),
    S("notes", False), S("price_url", False), S("availability_url", False),
]


def _common_q(bbox_required: bool) -> list[Q]:
    return [
        Q("mass_kg", "number", True, 0.0, True),
        Q("mass_without_cables_kg", "number", False, 0.0, True),
        Q("bounding_box_m", "vec3", bbox_required, 0.0, True),
        Q("price_usd", "number", False, 0.0),
    ]


_AVIONICS_Q = [
    Q("current_draw_a", "number", True, 0.0),
    Q("supply_voltage_v", "number", True, 0.0),  # 0 = passive part, no supply
]
_AVIONICS_S = [S("mount_slot")]

CATEGORIES: dict[str, dict] = {
    "frame": {
        "q": _common_q(True) + [
            Q("motor_spacing_adjacent_m", "number", True, 0.0, True),
            Q("arm_count", "integer", True, 3),
            Q("arm_length_m", "number", True, 0.0, True),
            Q("tube_od_m", "number", False, 0.0, True),
            Q("tube_id_m", "number", False, 0.0),
            Q("arm_plate_width_m", "number", False, 0.0, True),
            Q("arm_plate_thickness_m", "number", False, 0.0, True),
            Q("arm_allowable_bending_stress_pa", "number", False, 0.0, True),
            Q("body_half_width_m", "number", False, 0.0, True),
            Q("central_body_box_m", "vec3", False, 0.0, True),
            Q("cg_offset_m", "vec3", True),
            Q("projected_area_top_m2", "number", True, 0.0, True),
            Q("projected_area_front_m2", "number", True, 0.0, True),
            Q("projected_area_side_m2", "number", True, 0.0, True),
            Q("max_prop_diameter_m", "number", True, 0.0, True),
        ],
        "s": [S("frame_class"), S("motor_mount_pattern"), S("tube_material", False)],
        "key": ["mass_kg", "motor_spacing_adjacent_m", "arm_length_m", "max_prop_diameter_m",
                "bounding_box_m", "tube_od_m", "tube_id_m", "arm_count"],
    },
    "motor": {
        "q": _common_q(True) + [
            Q("kv_rpm_per_v", "number", True, 0.0, True),
            Q("winding_resistance_ohm", "number", True, 0.0, True),
            Q("no_load_current_a", "number", True, 0.0),
            Q("no_load_test_voltage_v", "number", True, 0.0, True),
            Q("max_current_a", "number", True, 0.0, True),
            Q("rating_duration_s", "number", True, 0.0, True),
            Q("max_power_w", "number", True, 0.0, True),
            Q("cells_min", "integer", True, 1),
            Q("cells_max", "integer", True, 1),
            Q("shaft_diameter_m", "number", True, 0.0, True),
            Q("prop_diameter_min_m", "number", True, 0.0, True),
            Q("prop_diameter_max_m", "number", True, 0.0, True),
            Q("thermal_resistance_k_per_w", "number", True, 0.0, True),
            Q("thermal_time_constant_s", "number", True, 0.0, True),
            Q("winding_temp_limit_c", "number", True, -50.0),
        ],
        "s": [S("mount_pattern")],
        "key": ["mass_kg", "kv_rpm_per_v", "winding_resistance_ohm", "max_current_a",
                "max_power_w", "cells_max", "shaft_diameter_m", "bounding_box_m"],
    },
    "esc": {
        "q": _common_q(True) + [
            Q("max_continuous_current_a", "number", True, 0.0, True),
            Q("burst_current_a", "number", False, 0.0, True),
            Q("burst_duration_s", "number", False, 0.0, True),
            Q("channels", "integer", True, 1),
            Q("cells_min", "integer", True, 1),
            Q("cells_max", "integer", True, 1),
            Q("efficiency", "number", True, 0.0, True, 1.0),
            Q("thermal_resistance_k_per_w", "number", True, 0.0, True),
            Q("temp_limit_c", "number", True, -50.0),
        ],
        "s": [],
        "key": ["mass_kg", "max_continuous_current_a", "cells_max", "cells_min", "channels",
                "efficiency", "bounding_box_m", "temp_limit_c"],
    },
    "propeller": {
        "q": _common_q(True) + [
            Q("diameter_m", "number", True, 0.0, True),
            Q("pitch_m", "number", True, 0.0, True),
            Q("blades", "integer", True, 2),
            Q("bore_m", "number", True, 0.0, True),
            Q("measured_rpm_min", "number", False, 0.0),
            Q("measured_rpm_max", "number", False, 0.0, True),
            Q("ct_cp_scaling_assumed", "number", False, 0.0, True),
        ],
        "s": [S("data_basis", True, ("UIUC_MEASURED", "SCALED")), S("data_file", False),
              S("data_file_sha256", False), S("scaled_from", False)],
        "key": ["mass_kg", "diameter_m", "pitch_m", "bore_m", "blades", "bounding_box_m"],
    },
    "battery": {
        "q": _common_q(True) + [
            Q("cells", "integer", True, 1),
            Q("capacity_ah", "number", True, 0.0, True),
            Q("voltage_per_cell_nominal_v", "number", True, 0.0, True),
            Q("voltage_per_cell_max_v", "number", True, 0.0, True),
            Q("voltage_per_cell_cutoff_v", "number", True, 0.0, True),
            Q("ocv_curve", "curve", True),
            Q("internal_resistance_ohm", "number", True, 0.0, True),
            Q("max_continuous_c", "number", True, 0.0, True),
            Q("usable_fraction", "number", False, 0.0, True, 1.0),
            Q("thermal_resistance_k_per_w", "number", True, 0.0, True),
            Q("temp_limit_c", "number", True, -50.0),
        ],
        "s": [S("connector")],
        "key": ["mass_kg", "cells", "capacity_ah", "max_continuous_c", "internal_resistance_ohm",
                "voltage_per_cell_nominal_v", "voltage_per_cell_cutoff_v", "bounding_box_m"],
    },
    "wiring": {
        "q": _common_q(False),
        "s": [],
        "key": ["mass_kg"],
    },
}
for _cat in ("flight_controller", "pdb", "receiver", "camera", "gimbal", "gps", "payload"):
    CATEGORIES[_cat] = {
        "q": _common_q(True) + _AVIONICS_Q,
        "s": list(_AVIONICS_S),
        "key": ["mass_kg", "current_draw_a", "supply_voltage_v", "bounding_box_m"],
    }

# Fields every record has regardless of category. Used by the CSV template.
def quantity_fields(category: str) -> list[Q]:
    return CATEGORIES[category]["q"]


def string_fields(category: str) -> list[S]:
    return COMMON_STRINGS + CATEGORIES[category]["s"]


def required_quantity_names(category: str) -> list[str]:
    return [q.name for q in quantity_fields(category) if q.required]
