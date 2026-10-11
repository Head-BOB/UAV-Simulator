# Tier: D
"""Vehicle builder: part ids -> vehicle_config.json (core input), vehicle_build.json (sidecar with provenance and rule
results) and assembly.json (3D assembly spec). Pure Python, deterministic (no clock, no randomness).

  python tools/parts/build_vehicle.py tools/parts/builds/reference_quad.build.json --out tools/parts/builds/reference_quad [--check]

Exit codes: 0 built (warnings allowed); 1 refused (errors listed, nothing written); 2 `--check` found stale/missing files.
Scope: 4-motor classes only (the core's VehicleConfig has fixed 4-element arrays). The exported config has exactly the core's
fields (`deny_unknown_fields`); everything else goes in the sidecar. The numbers are DESIGN-TIME ESTIMATES (see estimates.py).
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import NamedTuple

import compat as C
import estimates as E
import layout as L
import mechanics as M
import partsdb as P
import powertrain_export as PX
import controller_export as CX
import prop_table as PT

REQUIRED_ROLES = ["frame", "motor", "esc", "propeller", "battery"]
OPTIONAL_ROLES = ["wiring", "flight_controller", "receiver", "pdb", "gps", "camera", "gimbal"]
VEHICLE_CLASSES = ("quad_standard", "quad_fpv_5in")
SPEC_KEYS = {"name", "vehicle_class", "parts", "payload", "position_override_m", "notes"}
SEVERITY_ORDER = {"error": 0, "warning": 1, "info": 2}
ROUND_DP = 6
# Fields that feed the results, per role (used to list ASSUMED inputs in the sidecar and, later, in the report).
USED_FIELDS = {
    "frame": ["mass_kg", "motor_spacing_adjacent_m", "arm_length_m", "arm_count", "central_body_box_m", "body_half_width_m",
              "tube_od_m", "tube_id_m", "arm_plate_width_m", "arm_plate_thickness_m", "arm_allowable_bending_stress_pa",
              "projected_area_top_m2", "projected_area_front_m2", "projected_area_side_m2", "max_prop_diameter_m", "cg_offset_m"],
    "motor": ["mass_kg", "bounding_box_m", "kv_rpm_per_v", "winding_resistance_ohm", "no_load_current_a", "max_current_a",
              "shaft_diameter_m", "cells_min", "cells_max", "prop_diameter_min_m", "prop_diameter_max_m"],
    "esc": ["mass_kg", "bounding_box_m", "max_continuous_current_a", "channels", "cells_min", "cells_max", "efficiency"],
    "propeller": ["mass_kg", "bounding_box_m", "diameter_m", "bore_m"],
    "battery": ["mass_kg", "bounding_box_m", "cells", "capacity_ah", "voltage_per_cell_nominal_v", "max_continuous_c"],
}
AVIONICS_FIELDS = ["mass_kg", "bounding_box_m", "current_draw_a", "supply_voltage_v"]


class Result(NamedTuple):
    ok: bool
    findings: list
    config: dict | None
    sidecar: dict | None
    assembly: dict | None
    powertrain: dict | None
    controller: dict | None = None


def _err(rule: str, msg: str, parts: tuple = ()) -> C.Finding:
    return C.Finding(rule, "error", msg, parts)


def _vec3_ok(v) -> bool:
    return isinstance(v, list) and len(v) == 3 and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)


def validate_spec(spec: dict, records: dict) -> list:
    """Structure of the build file and the part ids. Returns error findings (empty = fine)."""
    out: list = []
    if not isinstance(spec, dict):
        return [_err("E-BUILD", "build file must be a JSON object")]
    for k in sorted(set(spec) - SPEC_KEYS):
        out.append(_err("E-BUILD", f"unknown key {k!r} in the build file"))
    name = spec.get("name")
    if not (isinstance(name, str) and re.match(r"^[a-z0-9][a-z0-9_]*$", name)):
        out.append(_err("E-BUILD", "name must be lowercase letters, digits and underscores"))
    if spec.get("vehicle_class") not in VEHICLE_CLASSES:
        out.append(_err("E-BUILD", f"vehicle_class must be one of {VEHICLE_CLASSES}"))
    parts = spec.get("parts")
    if not isinstance(parts, dict):
        return out + [_err("E-BUILD", "parts must be an object role -> part id")]
    for role in REQUIRED_ROLES:
        if role not in parts:
            out.append(_err("E-BUILD", f"required part {role!r} is missing"))
    for role, pid in sorted(parts.items()):
        if role == "payload":
            out.append(_err("E-PAYLOAD", "payload is a test condition, not a vehicle part: set the payload mass and position on the "
                            "Conditions screen (a build file may carry the top-level 'payload' key instead of a 'payload' part role)", (pid,) if isinstance(pid, str) else ()))
        elif role not in REQUIRED_ROLES + OPTIONAL_ROLES:
            out.append(_err("E-BUILD", f"unknown role {role!r}"))
        elif pid not in records:
            out.append(_err("E-BUILD", f"{role}: unknown part id {pid!r}"))
        elif records[pid]["category"] != role:
            out.append(_err("E-BUILD", f"{role}: {pid!r} is a {records[pid]['category']}, not a {role}", (pid,)))
    payload = spec.get("payload")
    if payload is not None:
        if not isinstance(payload, dict) or set(payload) - {"id", "position_m"}:
            out.append(_err("E-BUILD", "payload must be {id, position_m?}"))
        else:
            pid = payload.get("id")
            if pid not in records or records[pid]["category"] != "payload":
                out.append(_err("E-BUILD", f"payload: {pid!r} is not a payload part"))
            if "position_m" in payload and not _vec3_ok(payload["position_m"]):
                out.append(_err("E-BUILD", "payload.position_m must be 3 finite numbers"))
    ids = set(parts.values()) | ({payload["id"]} if isinstance(payload, dict) and "id" in payload else set())
    for pid, pos in sorted((spec.get("position_override_m") or {}).items()):
        if pid not in ids:
            out.append(_err("E-BUILD", f"position_override_m: {pid!r} is not in this build"))
        elif not _vec3_ok(pos):
            out.append(_err("E-BUILD", f"position_override_m[{pid}] must be 3 finite numbers"))
    return out


def _val(rec: dict, field: str):
    return rec[field]["value"]


def _used_rows(recs_by_role: dict, prop_basis: str) -> list:
    rows = []
    for role, rec in recs_by_role.items():
        for f in (USED_FIELDS.get(role) or AVIONICS_FIELDS):
            item = rec.get(f)
            if isinstance(item, dict):
                rows.append((rec["id"], f, item["provenance"]["source_type"], item["provenance"].get("note", "")))
    if prop_basis.startswith("SCALED"):
        prop = recs_by_role["propeller"]
        item = prop["ct_cp_scaling_assumed"]
        rows.append((prop["id"], "ct_cp_scaling_assumed", item["provenance"]["source_type"], item["provenance"]["note"]))
    return rows


def _lumped_old_estimate(m_total: float, central_box: list, corner_mass: float, spacing: float) -> list:
    """The old build_ref_config.py method (point masses at the motors + the rest as one box), kept as a cross-check only."""
    h = spacing / 2.0
    pts = [(h, h), (h, -h), (-h, h), (-h, -h)]
    ixx = sum(corner_mass * y * y for _, y in pts)
    iyy = sum(corner_mass * x * x for x, _ in pts)
    izz = sum(corner_mass * (x * x + y * y) for x, y in pts)
    rest = m_total - 4.0 * corner_mass
    a, b, c = central_box
    return [ixx + rest * (b * b + c * c) / 12.0, iyy + rest * (a * a + c * c) / 12.0, izz + rest * (a * a + b * b) / 12.0]


def _sorted_findings(fs: list) -> list:
    return sorted(fs, key=lambda f: (SEVERITY_ORDER[f.severity], f.rule, f.message))


def build(spec: dict, db_root: Path = P.DEFAULT_DB) -> Result:
    records, load_errs = P.load_records(db_root)
    findings = validate_spec(spec, records)
    if load_errs:
        findings.append(_err("E-DB", "; ".join(load_errs)))
    if findings:
        return Result(False, _sorted_findings(findings), None, None, None, None)
    db_errors, _ = P.validate_db(db_root)
    used_ids = sorted(set(spec["parts"].values()) | ({spec["payload"]["id"]} if spec.get("payload") else set()))
    bad = [i for i in used_ids if i in db_errors] + (["(db)"] if "(db)" in db_errors else [])
    if bad:
        return Result(False, [_err("E-DB", f"parts database has errors in: {bad}", tuple(bad))], None, None, None, None)
    try:
        return _build_checked(spec, records, db_root)
    except L.LayoutError as e:
        return Result(False, [_err("E-LAYOUT", str(e))], None, None, None, None)
    except ValueError as e:
        return Result(False, [_err("E-COMPUTE", str(e))], None, None, None, None)


def _build_checked(spec: dict, records: dict, db_root: Path) -> Result:
    by_role = {role: records[pid] for role, pid in spec["parts"].items()}
    frame, motor, esc, prop, bat = (by_role[r] for r in REQUIRED_ROLES)
    findings: list = []
    if frame["frame_class"] != spec["vehicle_class"]:
        findings.append(_err("E-CLASS", f"frame class {frame['frame_class']!r} does not match vehicle_class {spec['vehicle_class']!r}", (frame["id"],)))
    findings += C.arms_finding(frame)
    if _val(esc, "channels") not in (1, 4):
        findings.append(_err("E-BUILD", f"{esc['id']}: ESC channels must be 1 or 4 (got {_val(esc, 'channels')})", (esc["id"],)))
    findings += C.pair_findings(by_role)
    spacing, d_m = _val(frame, "motor_spacing_adjacent_m"), _val(prop, "diameter_m")
    # Tip and body clearance depend only on the frame and the propeller, so they are checked BEFORE the early refusal: a propeller that
    # does not fit the frame is reported together with the part-pair errors, not only after them (they used to hide it).
    if "central_body_box_m" in frame:   # a missing box is reported by the layout step (E-LAYOUT) below
        box = _val(frame, "central_body_box_m")
        body_corner_m = _val(frame, "body_half_width_m") * math.sqrt(2.0) if frame.get("body_half_width_m") else math.hypot(box[0], box[1]) / 2.0
        findings += C.clearance_findings(frame["id"], prop["id"], spacing, d_m, body_corner_m)
    if any(f.severity == "error" for f in findings):
        return Result(False, _sorted_findings(findings), None, None, None, None)

    section = L.arm_section(frame)
    frame_b, arm_info = L.frame_bodies(frame, section)
    esc_each = _val(esc, "channels") == 1
    corner_b, z_prop, _ = L.corner_bodies(motor, esc, prop, spacing, section["half_thickness_m"], esc_each)
    box = _val(frame, "central_body_box_m")
    bat_bbox = tuple(_val(bat, "bounding_box_m"))
    ctx = {"body_h_m": box[2], "body_x_m": box[0], "battery_h_m": bat_bbox[2]}
    overrides = spec.get("position_override_m") or {}

    slot_items = []
    if not esc_each:
        slot_items.append({"id": esc["id"], "role": "esc", "slot": "stack_30x30", "bbox": tuple(_val(esc, "bounding_box_m"))})
    for role in OPTIONAL_ROLES:
        rec = by_role.get(role)
        if rec is None or role == "wiring":
            continue
        slot_items.append({"id": rec["id"], "role": role, "slot": rec["mount_slot"], "bbox": tuple(_val(rec, "bounding_box_m"))})
    payload_rec = records[spec["payload"]["id"]] if spec.get("payload") else None
    if payload_rec is not None and "position_m" not in spec["payload"]:
        slot_items.append({"id": payload_rec["id"], "role": "payload", "slot": payload_rec["mount_slot"], "bbox": tuple(_val(payload_rec, "bounding_box_m"))})
    centres = L.place_slotted(slot_items, ctx)
    centres[bat["id"]] = (0.0, 0.0, box[2] / 2.0 + bat_bbox[2] / 2.0)
    if payload_rec is not None and "position_m" in spec["payload"]:
        centres[payload_rec["id"]] = tuple(spec["payload"]["position_m"])
    for pid, pos in overrides.items():
        centres[pid] = tuple(pos)

    bodies = list(frame_b) + list(corner_b)
    bodies.append(M.Body("battery", _val(bat, "mass_kg"), centres[bat["id"]], M.box_inertia(_val(bat, "mass_kg"), bat_bbox)))
    if not esc_each:
        e_m = _val(esc, "mass_kg")
        bodies.append(M.Body("esc_4in1", e_m, centres[esc["id"]], M.box_inertia(e_m, tuple(_val(esc, "bounding_box_m")))))
    extra = [(r, by_role[r]) for r in OPTIONAL_ROLES if r in by_role and r != "wiring"] + ([("payload", payload_rec)] if payload_rec else [])
    for role, rec in extra:
        bodies.append(M.Body(rec["id"], _val(rec, "mass_kg"), centres[rec["id"]], M.box_inertia(_val(rec, "mass_kg"), tuple(_val(rec, "bounding_box_m")))))
    wiring = by_role.get("wiring")
    if wiring is not None:
        w_pos = tuple(overrides.get(wiring["id"], (0.0, 0.0, 0.0)))
        bodies.append(M.Body("wiring", _val(wiring, "mass_kg"), w_pos, M.zeros()))
    total_kg, cg, tensor = M.combine(bodies)

    fit = E.load_static_fit(prop, records, db_root.parent)
    ct, cp = fit["ct"], fit["cp"]
    cells = _val(bat, "cells")
    v_pack = cells * _val(bat, "voltage_per_cell_nominal_v")
    kv, rm, i0 = _val(motor, "kv_rpm_per_v"), _val(motor, "winding_resistance_ohm"), _val(motor, "no_load_current_a")
    full = E.solve_full_throttle(kv, rm, i0, v_pack, ct, cp, d_m)

    r = lambda x: round(x, ROUND_DP)  # noqa: E731
    config = {
        "mass_kg": r(total_kg),
        "inertia_kgm2": [r(tensor[0][0]), r(tensor[1][1]), r(tensor[2][2])],
        "motor_pos_m": [[r(x - cg[0]), r(y - cg[1]), r(z_prop - cg[2])] for x, y in L.motor_xy(spacing)],
        "motor_spin": list(L.MOTOR_SPIN),
        "prop_diameter_m": d_m,
        "thrust_coeff": r(ct),
        "torque_coeff": r(E.kappa_m(cp, d_m, ct)),
        "max_rpm": round(full["rpm"], 3),
    }
    findings += C.inertia_finding(config["inertia_kgm2"])

    hover_thr = E.hover_throttle(config["mass_kg"], config["thrust_coeff"], d_m, config["max_rpm"])
    hover = E.hover_point(kv, rm, i0, config["thrust_coeff"], cp, d_m, config["mass_kg"])
    avionics_w = sum(_val(rec, "current_draw_a") * _val(rec, "supply_voltage_v") for _, rec in extra)
    eff = _val(esc, "efficiency")
    cap_a = _val(bat, "max_continuous_c") * _val(bat, "capacity_ah")
    arm_margin = None
    sigma = M.cantilever_bending_stress_pa(full["thrust_n"], _val(frame, "arm_length_m"), section["section_modulus_m3"])
    if frame.get("arm_allowable_bending_stress_pa"):
        arm_margin = _val(frame, "arm_allowable_bending_stress_pa") / sigma
    perf = {
        "motor_id": motor["id"], "esc_id": esc["id"], "battery_id": bat["id"], "frame_id": frame["id"],
        "hover_throttle": hover_thr, "thrust_to_weight": 4.0 * full["thrust_n"] / (config["mass_kg"] * E.G_MPS2),
        "full_current_a": full["current_a"], "hover_current_a": hover["current_a"],
        "motor_max_current_a": _val(motor, "max_current_a"), "esc_max_current_a": _val(esc, "max_continuous_current_a"),
        "pack_limit_a": cap_a,
        "hover_pack_a": E.pack_current_a(hover["electrical_w"], 4, eff, v_pack, avionics_w),
        "full_pack_a": E.pack_current_a(full["electrical_w"], 4, eff, v_pack, avionics_w),
        "arm_margin": arm_margin,
    }
    findings += C.performance_findings(perf)
    try:
        prop_json = json.loads((db_root.parent / fit["data_file"]).read_text(encoding="utf-8"))
        curves = PT.build_curves(prop_json)
        powertrain, pt_prov, pt_warnings = PX.build_powertrain(motor, esc, prop, bat, config, cp, curves)
        pt_prov["propeller_table"] = _propeller_model(PT.provenance(db_root.parent / fit["data_file"], curves, db_root.parent), curves, fit, config, hover)
    except (PX.PowertrainExportError, PT.PropTableError) as e:
        findings.append(_err("E-POWERTRAIN", str(e)))
        powertrain, pt_prov, pt_warnings = None, None, []
    try:
        controller = CX.build_controller(db_root, spec["vehicle_class"], hover["terminal_voltage_v"] / v_pack)
    except CX.ControllerExportError as e:
        findings.append(_err("E-CONTROLLER", str(e)))
        controller = None
    findings += [C.Finding("W-THERMAL-TAU", "warning", w, (motor["id"],)) for w in pt_warnings]
    used = _used_rows({**by_role, **({"payload": payload_rec} if payload_rec else {})}, fit["basis"])
    assumed = [{"part_id": p, "field": f, "note": n} for p, f, t, n in used if t == "ASSUMED"]
    findings.append(C.Finding("I-ASSUMED-COUNT", "info", f"{len(assumed)} ASSUMED input values influence this result (listed in the sidecar)", ()))
    if any(f.severity == "error" for f in findings):
        return Result(False, _sorted_findings(findings), None, None, None, None)

    old = _lumped_old_estimate(config["mass_kg"], box, _val(motor, "mass_kg") + _val(prop, "mass_kg") + (_val(esc, "mass_kg") if esc_each else 0.0), spacing)
    boxes = [tuple(_val(rec, "bounding_box_m")) for rec in [motor] * 4 + [bat] + [r_ for _, r_ in extra]] + ([tuple(_val(esc, "bounding_box_m"))] * (4 if esc_each else 1))
    areas = M.projected_areas_m2(boxes)
    fa = {k: _val(frame, f"projected_area_{k}_m2") for k in ("top", "front", "side")}
    sidecar = _sidecar(spec, config, records, db_root, by_role, perf, full, hover, fit, tensor, cg, old, section, sigma, arm_info,
                       {k: fa[k] + areas[k] for k in fa}, assumed, findings, v_pack, payload_rec, pt_prov)
    assembly = _assembly(spec, by_role, payload_rec, frame_b, corner_b, centres, overrides, section, spacing, box, bat, esc, esc_each, cg, total_kg, wiring, extra, z_prop)
    return Result(True, _sorted_findings(findings), config, sidecar, assembly, powertrain, controller)


def _propeller_model(prov: dict, curves: list, fit: dict, config: dict, hover: dict) -> dict:
    """Sidecar section about the Ct/Cp(J, rpm) table: its source and range, and how it differs from the static fit at hover."""
    ct, cp, flagged = PT.evaluate(curves, 0.0, hover["rpm"])
    return {**prov, "basis": fit["basis"], "stand_in": fit["basis"] != "UIUC_MEASURED",
            "hover_rpm_estimate": hover["rpm"], "hover_ct_table": ct, "hover_cp_table": cp, "hover_outside_data": flagged,
            "static_ct": config["thrust_coeff"], "static_cp": fit["cp"],
            "hover_ct_table_minus_static_percent": 100.0 * (ct - config["thrust_coeff"]) / config["thrust_coeff"],
            "note": "the core uses the table; estimates.py (max_rpm, hover point) still use the static constants, so hover thrust and duty in the core differ slightly"}


def _sidecar(spec, config, records, db_root, by_role, perf, full, hover, fit, tensor, cg, old, section, sigma, arm_info, areas, assumed, findings, v_pack, payload_rec, pt_prov) -> dict:
    version = json.loads((db_root / "VERSION.json").read_text(encoding="utf-8"))
    old_pct = [100.0 * (config["inertia_kgm2"][i] - old[i]) / old[i] for i in range(3)]
    parts = [{"role": role, "part_id": rec["id"], "manufacturer": rec["manufacturer"], "model": rec["model"]} for role, rec in sorted(by_role.items())]
    if payload_rec:
        parts.append({"role": "payload", "part_id": payload_rec["id"], "manufacturer": payload_rec["manufacturer"], "model": payload_rec["model"]})
    est = "design-time estimate (tools/parts/estimates.py); P3's electrical model supersedes it"
    return {
        "generated_by": "tools/parts/build_vehicle.py", "build_name": spec["name"], "vehicle_class": spec["vehicle_class"],
        "database": {"db_version": version["db_version"], "schema_version": version["schema_version"]},
        "parts": parts,
        "config_provenance": {
            "mass_kg": {"source_type": "CALCULATED", "formula": "sum of part mass_kg (with cables) x counts"},
            "inertia_kgm2": {"source_type": "CALCULATED", "formula": "diagonal of the combined tensor about the CG; parallel axis; shapes: box, cylinder (motor), thin disc (prop), thin rod (arm)"},
            "motor_pos_m": {"source_type": "CALCULATED", "formula": "(+-s/2, +-s/2) from motor_spacing_adjacent_m; z = propeller plane z minus CG z; order FR, FL, RR, RL"},
            "motor_spin": {"source_type": "ASSUMED", "formula": "same convention as the shipped vehicle_config.json (open point B-3 TODO(HUMAN-VERIFY))"},
            "prop_diameter_m": {"source_type": "CITED/ASSUMED", "formula": "propeller record diameter_m"},
            "thrust_coeff": {"source_type": "CALCULATED", "formula": f"static Ct fit, {fit['basis']}, file {fit['data_file']}, band {fit['band_rpm']} rpm"},
            "torque_coeff": {"source_type": "CALCULATED", "formula": "kappa = Cp D / (2 pi Ct)"},
            "max_rpm": {"source_type": "CALCULATED", "formula": f"full-throttle motor/prop torque balance at nominal pack voltage {v_pack:.3f} V; {est}"},
        },
        "estimates": {"note": est, "rho_kgpm3": E.RHO_SL_KGPM3, "pack_voltage_nominal_v": v_pack, "full_throttle": full,
                      "hover_point": hover, "hover_throttle": perf["hover_throttle"], "thrust_to_weight": perf["thrust_to_weight"],
                      "pack_hover_a": perf["hover_pack_a"], "pack_full_a": perf["full_pack_a"], "pack_limit_a": perf["pack_limit_a"],
                      "ct_cp_basis": fit["basis"]},
        "mass_properties": {"cg_from_frame_centre_m": list(cg), "inertia_tensor_about_cg_kgm2": tensor,
                            "core_limitation": "the core takes only the diagonal; the off-diagonal products above are dropped",
                            "frame_split": arm_info},
        "inertia_cross_check_old_lumped": {"old_method": "point masses at motors + remaining mass as one central box (build_ref_config.py)",
                                           "old_kgm2": old, "new_minus_old_percent": old_pct, "asserted_equal": False},
        "arm": {"section": section, "max_bending_stress_pa_at_full_throttle": sigma},
        "projected_areas_m2": {"values": areas, "note": "frame record areas + bounding-box faces of motors, battery and other parts; props excluded; overlap ignored (conservative)"},
        "powertrain": pt_prov,
        "hover_duty_estimate": hover["terminal_voltage_v"] / v_pack,
        "assumed_inputs": assumed,
        "builder_assumptions": ["layout slots and offsets (layout.py)", f"arm mass from CFRP density {L.CFRP_DENSITY_KGPM3} kg/m3", "nominal pack voltage, no sag, no ESC drop",
                                "static propeller coefficients, sea-level ISA density", "rule thresholds in compat.py"],
        "findings": [f.as_dict() for f in _sorted_findings(findings)],
    }


def _assembly(spec, by_role, payload_rec, frame_b, corner_b, centres, overrides, section, spacing, box, bat, esc, esc_each, cg, total_kg, wiring, extra, z_prop) -> dict:
    frame, motor, prop = by_role["frame"], by_role["motor"], by_role["propeller"]
    parts: list[dict] = []

    def add(role, rec, inst, pos, bbox, mass, visual):
        parts.append({"part_id": rec["id"], "role": role, "instance": inst, "position_m": [round(x, 9) for x in pos],
                      "bbox_m": [round(x, 9) for x in bbox], "mass_kg": mass, "visual": visual, "mesh": "stand_in"})

    fb = next(b for b in frame_b if b.name == "frame_body")
    add("frame", frame, "body", fb.position_m, box, fb.mass_kg, {"kind": "box", "dims_m": list(box)})
    arm_len = _val(frame, "arm_length_m")
    for b in frame_b:
        if b.name.startswith("arm_"):
            vis = {"kind": "tube" if section["kind"] == "tube" else "plate", "length_m": arm_len}
            vis.update({k: section[k] for k in ("od_m", "id_m", "width_m", "thickness_m") if k in section})
            add("frame", frame, b.name, b.position_m, (arm_len, section["half_thickness_m"] * 2.0, section["half_thickness_m"] * 2.0), b.mass_kg, vis)
    for b in corner_b:
        kind, i = b.name.rsplit("_", 1)
        if kind == "motor":
            bb = _val(motor, "bounding_box_m")
            add("motor", motor, b.name, b.position_m, bb, b.mass_kg, {"kind": "cylinder", "diameter_m": bb[0], "height_m": bb[2], "spin": L.MOTOR_SPIN[int(i)]})
        elif kind == "prop":
            add("propeller", prop, b.name, b.position_m, _val(prop, "bounding_box_m"), b.mass_kg, {"kind": "disc", "diameter_m": _val(prop, "diameter_m"), "spin": L.MOTOR_SPIN[int(i)]})
        else:
            add("esc", esc, b.name, b.position_m, _val(esc, "bounding_box_m"), b.mass_kg, {"kind": "box", "dims_m": _val(esc, "bounding_box_m")})
    add("battery", bat, "pack", centres[bat["id"]], _val(bat, "bounding_box_m"), _val(bat, "mass_kg"), {"kind": "box", "dims_m": _val(bat, "bounding_box_m")})
    if not esc_each:
        add("esc", esc, "4in1", centres[esc["id"]], _val(esc, "bounding_box_m"), _val(esc, "mass_kg"), {"kind": "box", "dims_m": _val(esc, "bounding_box_m")})
    for role, rec in extra:
        add(role, rec, "unit", centres[rec["id"]], _val(rec, "bounding_box_m"), _val(rec, "mass_kg"), {"kind": "box", "dims_m": _val(rec, "bounding_box_m")})
    if wiring is not None:
        add("wiring", wiring, "lump", overrides.get(wiring["id"], (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0), _val(wiring, "mass_kg"), {"kind": "none"})
    return {
        "generated_by": "tools/parts/build_vehicle.py", "build_name": spec["name"], "vehicle_class": spec["vehicle_class"],
        "frame_convention": "NED world; body X forward, Y right, Z down; origin = geometric centre of the frame's central body box; metres",
        "mesh_policy": "every part is a dimension-true stand-in; the Unreal phase (P7) decides which parts have real meshes",
        "motor_order": ["FR", "FL", "RR", "RL"], "propeller_plane_z_m": z_prop,
        "totals": {"mass_kg": total_kg, "cg_from_frame_centre_m": list(cg)},
        "parts": parts,
    }


def render(result: Result) -> dict:
    """File name -> text, exactly as written (also used by --check)."""
    return {"vehicle_config.json": json.dumps(result.config, indent=2) + "\n",
            "vehicle_build.json": P.canonical_json(result.sidecar),
            "assembly.json": P.canonical_json(result.assembly),
            "powertrain_config.json": PX.format_powertrain(result.powertrain),
            "controller_config.json": json.dumps(result.controller, indent=2) + "\n"}


def write_outputs(result: Result, out_dir: Path) -> list:
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for name, text in render(result).items():
        p = out_dir / name
        p.write_text(text, encoding="utf-8", newline="\n")
        paths.append(p)
    return paths


def check_outputs(result: Result, out_dir: Path) -> list:
    """Names of files that are missing or differ from a fresh build."""
    stale = []
    for name, text in render(result).items():
        p = out_dir / name
        if not p.exists() or p.read_bytes().decode("utf-8") != text:
            stale.append(name)
    return stale


def main(argv: list) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("build_file", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--check", action="store_true", help="do not write; fail if the output files are stale or missing")
    ap.add_argument("--db", type=Path, default=P.DEFAULT_DB)
    a = ap.parse_args(argv)
    try:
        spec = json.loads(a.build_file.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        print(f"cannot read build file: {e}", file=sys.stderr)
        return 1
    res = build(spec, a.db)
    for f in res.findings:
        print(f"{f.severity.upper():7} {f.rule}: {f.message}", file=sys.stderr if f.severity == "error" else sys.stdout)
    if not res.ok:
        print("BUILD REFUSED, nothing written", file=sys.stderr)
        return 1
    if a.check:
        stale = check_outputs(res, a.out)
        if stale:
            print(f"stale or missing in {a.out}: {stale}. Rebuild without --check.", file=sys.stderr)
            return 2
        print("build outputs are up to date")
        return 0
    write_outputs(res, a.out)
    c = res.config
    print(f"built {spec['name']}: mass {c['mass_kg']} kg, inertia {c['inertia_kgm2']}, max_rpm {c['max_rpm']}, "
          f"hover throttle {res.sidecar['estimates']['hover_throttle']:.4f}, T/W {res.sidecar['estimates']['thrust_to_weight']:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
