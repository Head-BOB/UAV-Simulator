# Tier: D
"""Load generated build files through the REAL core (the release library) and compare with the builder's own numbers:
  * vehicle_config.json: `ffi_load_vehicle_config`, every field and the derived hover_throttle;
  * powertrain_config.json (next to it, if present): `ffi_load_powertrain_config`, `ffi_init_powertrain_state`, a bench point
    against the independent Python solver (tools/parts/estimates.py), and 200 real `ffi_step_powertrain` steps with the generated
    pair (code 14 would mean the two files contradict each other).

  (cd physics_core && cargo build --release)
  python tools/parts/check_core_loads.py tools/parts/builds/reference_quad/vehicle_config.json [--lib PATH]

Exit 0: core accepted the files, every field equals the JSON, hover throttle equals the Python estimate (rel 1e-12), the bench point
equals the Python solve (rel 1e-9), and the steps return 0 with valid telemetry.
Exit 1: mismatch or core refused (the FFI error code is printed). Exit 2: library or file missing (not a pass).
This is a script, not a unit test, because it needs a built library.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import math
import sys
from pathlib import Path

import estimates as E
import prop_table as PT

REPO = Path(__file__).resolve().parents[2]
DEFAULT_LIB = REPO / "physics_core" / "target" / "release" / "libphysics_core.so"
EXPECTED_INTERFACE_VERSION = 14


class VehicleConfig(ctypes.Structure):
    """Mirror of `types::VehicleConfig` (repr(C), 200 bytes)."""

    _fields_ = [("mass_kg", ctypes.c_double), ("inertia_kgm2", ctypes.c_double * 3),
                ("motor_pos_m", (ctypes.c_double * 3) * 4), ("motor_spin", ctypes.c_double * 4),
                ("prop_diameter_m", ctypes.c_double), ("thrust_coeff", ctypes.c_double),
                ("torque_coeff", ctypes.c_double), ("max_rpm", ctypes.c_double), ("hover_throttle", ctypes.c_double)]


class DroneState(ctypes.Structure):
    """Mirror of `types::DroneState` (64 bytes)."""

    _fields_ = [("position", ctypes.c_double * 3), ("velocity", ctypes.c_float * 3),
                ("orientation", ctypes.c_float * 4), ("angular_velocity", ctypes.c_float * 3)]


class ControlInputs(ctypes.Structure):
    _fields_ = [("throttle", ctypes.c_float), ("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw", ctypes.c_float)]


class Vec3d(ctypes.Structure):
    """`WindInput` (24 bytes)."""

    _fields_ = [("wind_ned_mps", ctypes.c_double * 3)]


class EnvironmentInput(ctypes.Structure):
    _fields_ = [("site_altitude_m", ctypes.c_double), ("temp_offset_c", ctypes.c_double)]


class DebugTelemetry(ctypes.Structure):
    """Mirror of `types::DebugTelemetry` (136 bytes)."""

    _fields_ = [("net_thrust", ctypes.c_float * 3), ("aero_drag", ctypes.c_float * 3), ("gravity", ctypes.c_float * 3),
                ("net_force", ctypes.c_float * 3), ("motor_thrusts", ctypes.c_float * 4), ("motor_rpms", ctypes.c_float * 4),
                ("motor_temperatures_c", ctypes.c_float * 4), ("structural_safety_margin", ctypes.c_float),
                ("is_validated_envelope", ctypes.c_int32), ("stub_loaded", ctypes.c_int32), ("aero_source", ctypes.c_int32),
                ("fea_source", ctypes.c_int32), ("physics_fault", ctypes.c_int32), ("dropped_time_events", ctypes.c_int32),
                ("uncalibrated_flags", ctypes.c_int32), ("thermal_valid", ctypes.c_int32), ("thermal_over_temp", ctypes.c_int32)]


class PowertrainState(ctypes.Structure):
    """Mirror of `types::PowertrainState` (144 bytes)."""

    _fields_ = [("soc", ctypes.c_double), ("charge_used_ah", ctypes.c_double), ("battery_current_a", ctypes.c_double),
                ("battery_temp_c", ctypes.c_double), ("time_below_cutoff_s", ctypes.c_double),
                ("motor_winding_c", ctypes.c_double * 4), ("motor_case_c", ctypes.c_double * 4), ("esc_temp_c", ctypes.c_double * 4),
                ("prop_extrapolated_time_s", ctypes.c_double)]


class PowertrainTelemetry(ctypes.Structure):
    """Mirror of `types::PowertrainTelemetry` (576 bytes)."""

    _fields_ = [(n, ctypes.c_double * 4) for n in ("motor_current_a", "motor_voltage_v", "motor_rpm", "motor_torque_nm", "motor_thrust_n",
                                                    "motor_copper_loss_w", "motor_noload_loss_w", "esc_loss_w", "motor_winding_c",
                                                    "motor_case_c", "esc_temp_c")] + \
              [(n, ctypes.c_double) for n in ("battery_voltage_v", "battery_ocv_v", "battery_current_a", "battery_power_w", "battery_soc",
                                              "battery_temp_c", "battery_heat_w", "charge_used_ah", "time_below_cutoff_s")] + \
              [(n, ctypes.c_int32) for n in ("step_valid", "battery_cutoff", "battery_over_current", "battery_over_temp", "battery_soc_clamped",
                                             "motor_over_current_mask", "motor_over_temp_mask", "esc_over_current_mask", "esc_over_temp_mask",
                                             "thermal_valid", "prop_extrapolated", "reserved0", "reserved1", "reserved2")] + \
              [(n, ctypes.c_double * 4) for n in ("motor_advance_ratio", "motor_ct", "motor_cp")]


class PowertrainBenchOut(ctypes.Structure):
    _fields_ = [(n, ctypes.c_double) for n in ("rpm", "motor_current_a", "torque_nm", "thrust_n", "shaft_w", "motor_electrical_w",
                                              "copper_loss_w", "noload_loss_w", "esc_loss_w", "battery_current_a", "battery_power_w",
                                              "system_efficiency", "advance_ratio", "ct", "cp")] + \
              [("extrapolated", ctypes.c_int32), ("reserved", ctypes.c_int32)]


class PropCoefficients(ctypes.Structure):
    _fields_ = [("ct", ctypes.c_double), ("cp", ctypes.c_double), ("extrapolated", ctypes.c_int32), ("reserved", ctypes.c_int32)]


def check_powertrain(lib: ctypes.CDLL, pt_path: Path, vehicle: VehicleConfig) -> list:
    """Loads the powertrain config, compares a bench point with the Python solver and steps the real core. Returns problems."""
    bad: list = []
    for name, fn_args in (("ffi_load_powertrain_config", [ctypes.c_char_p, ctypes.c_void_p]),
                          ("ffi_init_powertrain_state", [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.POINTER(PowertrainState)]),
                          ("ffi_powertrain_bench_point", [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.POINTER(PowertrainBenchOut)]),
                          ("ffi_propeller_coefficients", [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.POINTER(PropCoefficients)])):
        getattr(lib, name).argtypes = fn_args
        getattr(lib, name).restype = ctypes.c_int32
    cfg_buf = ctypes.create_string_buffer(9792)
    code = lib.ffi_load_powertrain_config(str(pt_path).encode(), cfg_buf)
    if code != 0:
        return [f"core REFUSED the powertrain config, code {code}"]
    pt = json.loads(pt_path.read_text(encoding="utf-8"))
    state = PowertrainState()
    if lib.ffi_init_powertrain_state(cfg_buf, 20.0, 1.0, ctypes.byref(state)) != 0:
        return ["ffi_init_powertrain_state failed"]
    v_pack = pt["battery_cells"] * 3.7
    bench = PowertrainBenchOut()
    if lib.ffi_powertrain_bench_point(cfg_buf, 1.0, v_pack, E.RHO_SL_KGPM3, 0.0, ctypes.byref(bench)) != 0:
        return ["ffi_powertrain_bench_point failed"]
    curves = pt.get("prop_curves")
    if curves is None:   # static model: the independent constant-coefficient solver applies
        ref = E.solve_full_throttle(pt["motor_kv_rpm_per_v"], pt["motor_resistance_ohm"], pt["motor_no_load_current_a"], v_pack,
                                    pt["prop_ct"], pt["prop_cp"], pt["prop_diameter_m"])
        for core_val, py_val, name in ((bench.rpm, ref["rpm"], "rpm"), (bench.motor_current_a, ref["current_a"], "current"),
                                       (bench.thrust_n, ref["thrust_n"], "thrust")):
            if abs(core_val - py_val) > 1e-9 * abs(py_val):
                bad.append(f"bench {name}: core {core_val!r} != python {py_val!r}")
    else:
        bad += _compare_table(lib, cfg_buf, curves)
        bad += _bench_table_consistency(bench, curves)
    bad += _step_real_core(lib, cfg_buf, vehicle, state)
    return bad


def _compare_table(lib: ctypes.CDLL, cfg_buf, curves: list) -> list:
    """Core `ffi_propeller_coefficients` against the Python reference `prop_table.evaluate` on a (J, rpm) grid inside and outside the data."""
    bad: list = []
    rpm_lo, rpm_hi = curves[0]["rpm"], curves[-1]["rpm"]
    j_grid = [-0.1, 0.0, 0.013, 0.1, 0.2, 0.3333, 0.45, 0.6, 0.66, 0.9, 2.0]
    rpm_grid = [rpm_lo - 500.0, rpm_lo, rpm_lo + 1.0, 3553.0, 4030.0, 4500.0, 5024.5, 5500.0, rpm_hi - 1.0, rpm_hi, rpm_hi + 700.0, 9000.0]
    out = PropCoefficients()
    for j in j_grid:
        for rpm in rpm_grid:
            if lib.ffi_propeller_coefficients(cfg_buf, j, rpm, ctypes.byref(out)) != 0:
                return [f"ffi_propeller_coefficients refused ({j}, {rpm})"]
            ct, cp, flag = PT.evaluate(curves, j, rpm)
            if abs(out.ct - ct) > 1e-12 or abs(out.cp - cp) > 1e-12 or bool(out.extrapolated) != flag:
                bad.append(f"table ({j}, {rpm}): core ({out.ct!r}, {out.cp!r}, {out.extrapolated}) != python ({ct!r}, {cp!r}, {flag})")
    return bad


def _bench_table_consistency(bench: PowertrainBenchOut, curves: list) -> list:
    """The bench point must have used the table: its reported Ct/Cp are what the Python table gives at the bench's own J and rpm."""
    ct, cp, flag = PT.evaluate(curves, bench.advance_ratio, bench.rpm)
    if abs(bench.ct - ct) > 1e-12 or abs(bench.cp - cp) > 1e-12 or bool(bench.extrapolated) != flag:
        return [f"bench coefficients ({bench.ct!r}, {bench.cp!r}, {bench.extrapolated}) != table ({ct!r}, {cp!r}, {flag})"]
    return []


def _step_real_core(lib: ctypes.CDLL, cfg_buf, vehicle: VehicleConfig, state: PowertrainState) -> list:
    lib.ffi_create_default_drone_state.restype = DroneState
    step = lib.ffi_step_powertrain
    step.restype = ctypes.c_int32
    step.argtypes = [ctypes.POINTER(DroneState), ctypes.POINTER(PowertrainState), ctypes.POINTER(ControlInputs), ctypes.POINTER(Vec3d),
                     ctypes.POINTER(EnvironmentInput), ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(VehicleConfig), ctypes.c_void_p,
                     ctypes.POINTER(DebugTelemetry), ctypes.POINTER(PowertrainTelemetry), ctypes.POINTER(ctypes.c_int32), ctypes.c_double]
    drone = lib.ffi_create_default_drone_state()
    drone.position[2] = -10.0
    controls, wind, env = ControlInputs(0.4, 0.0, 0.0, 0.0), Vec3d(), EnvironmentInput()
    tel, ptel, latch = DebugTelemetry(), PowertrainTelemetry(), ctypes.c_int32(0)
    for i in range(200):
        code = step(ctypes.byref(drone), ctypes.byref(state), ctypes.byref(controls), ctypes.byref(wind), ctypes.byref(env), None, None,
                    ctypes.byref(vehicle), cfg_buf, ctypes.byref(tel), ctypes.byref(ptel), ctypes.byref(latch), 0.002)
        if code != 0:
            return [f"ffi_step_powertrain returned {code} at step {i} (14 = the generated vehicle/powertrain pair contradict each other)"]
    if ptel.step_valid != 1 or not all(math.isfinite(v) for v in ptel.motor_rpm) or latch.value != 0:
        return ["powertrain telemetry invalid after 200 steps"]
    if not 0.0 < state.soc < 1.0:
        return [f"state of charge {state.soc} did not move"]
    return []


def main(argv: list) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("config", type=Path)
    ap.add_argument("--lib", type=Path, default=DEFAULT_LIB)
    a = ap.parse_args(argv)
    if not a.lib.exists() or not a.config.exists():
        print(f"missing: {[str(p) for p in (a.lib, a.config) if not p.exists()]}", file=sys.stderr)
        return 2
    lib = ctypes.CDLL(str(a.lib))
    lib.ffi_get_interface_version.restype = ctypes.c_int32
    lib.ffi_load_vehicle_config.argtypes = [ctypes.c_char_p, ctypes.POINTER(VehicleConfig)]
    lib.ffi_load_vehicle_config.restype = ctypes.c_int32
    version = lib.ffi_get_interface_version()
    if version != EXPECTED_INTERFACE_VERSION:
        print(f"core interface version {version}, this script expects {EXPECTED_INTERFACE_VERSION}", file=sys.stderr)
        return 1
    if ctypes.sizeof(VehicleConfig) != 200:
        print("ctypes mirror has the wrong size", file=sys.stderr)
        return 1
    out = VehicleConfig()
    code = lib.ffi_load_vehicle_config(str(a.config).encode(), ctypes.byref(out))
    if code != 0:
        print(f"core REFUSED the config, code {code}", file=sys.stderr)
        return 1
    cfg = json.loads(a.config.read_text(encoding="utf-8"))
    bad = []
    for name in ("mass_kg", "prop_diameter_m", "thrust_coeff", "torque_coeff", "max_rpm"):
        if getattr(out, name) != cfg[name]:
            bad.append(f"{name}: core {getattr(out, name)!r} != json {cfg[name]!r}")
    if list(out.inertia_kgm2) != cfg["inertia_kgm2"] or list(out.motor_spin) != cfg["motor_spin"]:
        bad.append("inertia or spin differ")
    if [list(r) for r in out.motor_pos_m] != cfg["motor_pos_m"]:
        bad.append("motor_pos_m differs")
    expect = E.hover_throttle(cfg["mass_kg"], cfg["thrust_coeff"], cfg["prop_diameter_m"], cfg["max_rpm"])
    if abs(out.hover_throttle - expect) > 1e-12 * max(1.0, abs(expect)):
        bad.append(f"hover_throttle: core {out.hover_throttle!r} != python {expect!r}")
    pt_path = a.config.with_name("powertrain_config.json")
    if pt_path.exists():
        bad += check_powertrain(lib, pt_path, out)
    if bad:
        print("MISMATCH:", *bad, sep="\n  ", file=sys.stderr)
        return 1
    extra = " and the powertrain pair (load, init, bench point, 200 steps)" if pt_path.exists() else ""
    print(f"core accepted {a.config.name}{extra}: hover_throttle core {out.hover_throttle:.12f} == python {expect:.12f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
