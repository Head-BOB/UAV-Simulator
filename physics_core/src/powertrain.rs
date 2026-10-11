// Tier: F
//! Powertrain model (interface v14): motor + propeller torque balance, ESC, battery and lumped thermal networks.
//!
//! Pure, deterministic, allocation-free in the step. Units are SI; `n` is shaft speed in rev/s unless a name says rpm.
//!
//! Model (every constant is a field of `PowertrainConfig` or a named constant with a source or `TODO(HUMAN-VERIFY)`):
//! * Propeller: `T = Ct rho n^2 D^4`, `Q = Cp rho n^2 D^5 / (2 pi)`, with `Ct, Cp` either constants (`prop_model` 0, the static fit) or
//!   read from the UIUC table at the advance ratio `J = V_axial / (n D)` and the speed `rpm` (`prop_model` 1): linear in `J` inside
//!   a curve, linear in rpm between the two bracketing curves, nearest edge value HELD and a flag raised outside the measured data.
//! * Motor: `Kt = 60 / (2 pi Kv)`, `I = Q / Kt + I0`, `V = 60 n / Kv + I Rm`. Copper loss `I^2 Rm` heats the winding,
//!   the no-load loss `I0 x back-EMF` (iron + friction) heats the case.
//! * Torque balance `60 n = Kv (V - Rm (Q(n) / Kt + I0))` is solved by bounded bisection (`BISECTION_STEPS`).
//! * ESC: terminal voltage `V = duty x pack voltage`, efficiency `eta`: pack power = motor electrical power / `eta`.
//! * Battery: `V_pack = cells x OCV(SOC) - I R_int` (the sag uses the PREVIOUS step's current: one-step lag, no algebraic
//!   loop), SOC by coulomb counting.
//! * Thermal: forward-Euler lumped RC networks (motor winding - case - ambient, ESC, battery) with forced-convection cooling.
//!
//! Not modelled: rotor inertia (speed follows the torque balance instantly), regeneration and braking, battery temperature
//! and SOC dependence of the internal resistance, ESC switching detail, humidity, edgewise inflow of the propeller and rotor-by-rotor
//! inflow differences from body rates (the axial inflow is the same for the four rotors). Absolute temperatures are only as good as the thermal inputs, which are ASSUMED placeholders today
//! (`parameters_verified` stays 0).

use crate::types::{
    MAX_OCV_POINTS, MAX_PROP_CURVES, MAX_PROP_POINTS, PowertrainBenchOut, PowertrainConfig,
    PowertrainState, PowertrainTelemetry, VehicleConfig,
};
use glam::{DQuat, DVec3};
use serde::Deserialize;
use std::f64::consts::PI;
use std::fs;

/// Bisection steps of the torque balance. The bracket is `[0, Kv V / 60]` (at most a few hundred rev/s), so 60 halvings
/// resolve it far below f64 noise. A fixed count keeps the loop bounded and the result bit-reproducible.
const BISECTION_STEPS: u32 = 60;

/// Exponent of the forced-convection law `h ~ v^0.8`. Turbulent flow over a body gives an exponent near 0.8
/// (Nusselt-number correlations), but no source was checked for this geometry. TODO(HUMAN-VERIFY): cite or replace.
const COOLING_EXPONENT: f64 = 0.8;

/// The explicit thermal step must satisfy `dt <= THERMAL_STEP_FRACTION x tau_min`. For the two-node motor network the
/// largest eigenvalue magnitude is at most `3 / tau_min` (Gershgorin, with `tau` the products R x C of the network), and
/// forward Euler is stable for `dt < 2 / |lambda_max|` = `0.67 tau_min`; 0.2 leaves a margin of more than 3.
const THERMAL_STEP_FRACTION: f64 = 0.2;

const MOTOR_COUNT: usize = 4;
const SECONDS_PER_HOUR: f64 = 3600.0;
/// Largest cell count accepted (a 14S pack is the largest the parts database models).
const MAX_CELLS: i32 = 14;
/// Tolerance for the vehicle/powertrain consistency check of the shaft constants (relative).
const KAPPA_RELATIVE_TOLERANCE: f64 = 1.0e-3;
/// Tolerance for `prop_diameter_m` and `prop_ct` equality with the vehicle config (relative).
const SHARED_FIELD_RELATIVE_TOLERANCE: f64 = 1.0e-9;
/// Plausible temperature window for a state value, deg C (rejects garbage, not a physical limit).
const STATE_TEMP_MIN_C: f64 = -100.0;
const STATE_TEMP_MAX_C: f64 = 2000.0;

/// Why a powertrain call was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowertrainError {
    /// A config field is non-finite or out of range, the OCV table is malformed, or the config contradicts the vehicle config.
    InvalidConfig,
    /// A state field is non-finite or out of range.
    InvalidState,
    /// A step input (duty, density, airspeed, ambient, dt) is non-finite or out of range.
    InvalidInput,
    /// The thermal time step is too large for the network (`dt > THERMAL_STEP_FRACTION x tau_min`).
    ThermalStep,
    /// A result came out non-finite.
    NonFinite,
}

fn positive_finite(x: f64) -> bool {
    x.is_finite() && x > 0.0
}

fn nonnegative_finite(x: f64) -> bool {
    x.is_finite() && x >= 0.0
}

fn rel_diff(a: f64, b: f64) -> f64 {
    (a - b).abs() / a.abs().max(b.abs()).max(f64::MIN_POSITIVE)
}

/// Checks every field of a `PowertrainConfig`.
///
/// # Errors
/// `InvalidConfig` on the first violated rule: motor and propeller constants finite and > 0 (`I0` >= 0); `eta` in (0, 1];
/// 1 to 14 cells; 2 to 16 OCV points with SOC strictly increasing from exactly 0 to exactly 1 and a positive,
/// non-decreasing voltage; resistances, capacities and capacitances finite and > 0 (battery resistance >= 0);
/// limits finite; `parameters_verified` 0 or 1; `reserved` 0.
pub fn validate_powertrain_config(cfg: &PowertrainConfig) -> Result<(), PowertrainError> {
    let positives = [
        cfg.motor_kv_rpm_per_v,
        cfg.motor_resistance_ohm,
        cfg.motor_max_current_a,
        cfg.prop_diameter_m,
        cfg.prop_ct,
        cfg.prop_cp,
        cfg.esc_max_current_a,
        cfg.battery_capacity_ah,
        cfg.battery_cutoff_v_per_cell,
        cfg.battery_max_continuous_a,
        cfg.motor_r_winding_case_k_per_w,
        cfg.motor_r_case_ambient_k_per_w,
        cfg.motor_c_winding_j_per_k,
        cfg.motor_c_case_j_per_k,
        cfg.esc_r_ambient_k_per_w,
        cfg.esc_c_j_per_k,
        cfg.battery_r_ambient_k_per_w,
        cfg.battery_c_j_per_k,
        cfg.cooling_ref_airspeed_mps,
    ];
    let limits = [
        cfg.motor_winding_temp_limit_c,
        cfg.esc_temp_limit_c,
        cfg.battery_temp_limit_c,
    ];
    let ok = positives.iter().all(|&v| positive_finite(v))
        && nonnegative_finite(cfg.motor_no_load_current_a)
        && nonnegative_finite(cfg.battery_internal_resistance_ohm)
        && limits.iter().all(|v| v.is_finite())
        && cfg.esc_efficiency.is_finite()
        && cfg.esc_efficiency > 0.0
        && cfg.esc_efficiency <= 1.0
        && (1..=MAX_CELLS).contains(&cfg.battery_cells)
        && (cfg.parameters_verified == 0 || cfg.parameters_verified == 1)
        && cfg.reserved == 0;
    if !ok {
        return Err(PowertrainError::InvalidConfig);
    }
    validate_ocv_table(cfg)?;
    validate_prop_table(cfg)
}

fn validate_ocv_table(cfg: &PowertrainConfig) -> Result<(), PowertrainError> {
    let n = usize::try_from(cfg.ocv_points).map_err(|_| PowertrainError::InvalidConfig)?;
    if !(2..=MAX_OCV_POINTS).contains(&n) {
        return Err(PowertrainError::InvalidConfig);
    }
    if !cfg
        .ocv_soc
        .iter()
        .chain(cfg.ocv_v_per_cell.iter())
        .all(|v| v.is_finite())
    {
        return Err(PowertrainError::InvalidConfig);
    }
    let socs = cfg.ocv_soc.get(..n).ok_or(PowertrainError::InvalidConfig)?;
    let volts = cfg
        .ocv_v_per_cell
        .get(..n)
        .ok_or(PowertrainError::InvalidConfig)?;
    // Exact comparison on purpose: 0 and 1 are the sentinels that make the table cover the whole SOC range.
    let spans = socs.first() == Some(&0.0) && socs.last() == Some(&1.0);
    let increasing = socs.windows(2).all(|w| matches!(w, [a, b] if b > a));
    let nondecreasing = volts.windows(2).all(|w| matches!(w, [a, b] if b >= a));
    if spans && increasing && nondecreasing && volts.iter().all(|&v| v > 0.0) {
        Ok(())
    } else {
        Err(PowertrainError::InvalidConfig)
    }
}

fn validate_prop_table(cfg: &PowertrainConfig) -> Result<(), PowertrainError> {
    let finite = cfg.prop_curve_rpm.iter().all(|v| v.is_finite())
        && cfg
            .prop_j
            .iter()
            .chain(cfg.prop_ct_table.iter())
            .chain(cfg.prop_cp_table.iter())
            .flatten()
            .all(|v| v.is_finite());
    match (finite, cfg.prop_model) {
        (true, 0) if cfg.prop_curve_count == 0 => Ok(()),
        (true, 1) => validate_curves(cfg),
        _ => Err(PowertrainError::InvalidConfig),
    }
}

/// Table rules: 1..=8 curves with strictly increasing positive rpm; each curve 2..=48 points, `J` strictly increasing from exactly 0,
/// `Cp > 0` (the torque must stay positive for the bisection bracket), `Ct` finite (it may be negative at high `J`).
fn validate_curves(cfg: &PowertrainConfig) -> Result<(), PowertrainError> {
    let count =
        usize::try_from(cfg.prop_curve_count).map_err(|_| PowertrainError::InvalidConfig)?;
    if !(1..=MAX_PROP_CURVES).contains(&count) {
        return Err(PowertrainError::InvalidConfig);
    }
    let rpms = cfg
        .prop_curve_rpm
        .get(..count)
        .ok_or(PowertrainError::InvalidConfig)?;
    let rpm_ok = rpms.first().is_some_and(|&r| r > 0.0)
        && rpms.windows(2).all(|w| matches!(w, [a, b] if b > a));
    if !rpm_ok {
        return Err(PowertrainError::InvalidConfig);
    }
    let rows = cfg
        .prop_j
        .iter()
        .zip(cfg.prop_ct_table.iter())
        .zip(cfg.prop_cp_table.iter())
        .zip(cfg.prop_curve_points.iter())
        .take(count);
    for (((js, _cts), cps), &m) in rows {
        let m = usize::try_from(m).map_err(|_| PowertrainError::InvalidConfig)?;
        if !(2..=MAX_PROP_POINTS).contains(&m) {
            return Err(PowertrainError::InvalidConfig);
        }
        let (Some(j), Some(cp)) = (js.get(..m), cps.get(..m)) else {
            return Err(PowertrainError::InvalidConfig);
        };
        // Exact comparison on purpose: J = 0 is the sentinel that anchors the curve at the static point.
        let ok = j.first() == Some(&0.0)
            && j.windows(2).all(|w| matches!(w, [a, b] if b > a))
            && cp.iter().all(|&v| v > 0.0);
        if !ok {
            return Err(PowertrainError::InvalidConfig);
        }
    }
    Ok(())
}

/// The powertrain config must describe the same propeller as the vehicle config the core integrates with.
///
/// # Errors
/// `InvalidConfig` if the diameters or the thrust coefficients differ, or if `kappa = Cp D / (2 pi Ct)` differs from the
/// vehicle's `torque_coeff` by more than `KAPPA_RELATIVE_TOLERANCE` (the builder rounds kappa to 6 decimals).
pub fn check_consistency(
    vehicle: &VehicleConfig,
    cfg: &PowertrainConfig,
) -> Result<(), PowertrainError> {
    let kappa = cfg.prop_cp * cfg.prop_diameter_m / (2.0 * PI * cfg.prop_ct);
    let ok = rel_diff(vehicle.prop_diameter_m, cfg.prop_diameter_m)
        <= SHARED_FIELD_RELATIVE_TOLERANCE
        && rel_diff(vehicle.thrust_coeff, cfg.prop_ct) <= SHARED_FIELD_RELATIVE_TOLERANCE
        && rel_diff(vehicle.torque_coeff, kappa) <= KAPPA_RELATIVE_TOLERANCE;
    if ok {
        Ok(())
    } else {
        Err(PowertrainError::InvalidConfig)
    }
}

/// Checks a `PowertrainState`.
///
/// # Errors
/// `InvalidState` if SOC is outside [0, 1], a temperature is outside the plausibility window, or the current, charge or
/// time below cutoff is negative or non-finite.
pub fn validate_state(state: &PowertrainState) -> Result<(), PowertrainError> {
    let temps_ok = state
        .motor_winding_c
        .iter()
        .chain(state.motor_case_c.iter())
        .chain(state.esc_temp_c.iter())
        .chain(std::iter::once(&state.battery_temp_c))
        .all(|t| t.is_finite() && (STATE_TEMP_MIN_C..=STATE_TEMP_MAX_C).contains(t));
    let ok = temps_ok
        && state.soc.is_finite()
        && (0.0..=1.0).contains(&state.soc)
        && nonnegative_finite(state.charge_used_ah)
        && nonnegative_finite(state.battery_current_a)
        && nonnegative_finite(state.time_below_cutoff_s)
        && nonnegative_finite(state.prop_extrapolated_time_s);
    if ok {
        Ok(())
    } else {
        Err(PowertrainError::InvalidState)
    }
}

/// A fresh state: full of `soc`, every node at `ambient_c`, no current history.
///
/// # Errors
/// `InvalidState` if `soc` is outside [0, 1] or `ambient_c` is outside the plausibility window.
pub fn initial_state(soc: f64, ambient_c: f64) -> Result<PowertrainState, PowertrainError> {
    let state = PowertrainState {
        soc,
        charge_used_ah: 0.0,
        battery_current_a: 0.0,
        battery_temp_c: ambient_c,
        time_below_cutoff_s: 0.0,
        motor_winding_c: [ambient_c; MOTOR_COUNT],
        motor_case_c: [ambient_c; MOTOR_COUNT],
        esc_temp_c: [ambient_c; MOTOR_COUNT],
        prop_extrapolated_time_s: 0.0,
    };
    validate_state(&state)?;
    Ok(state)
}

/// Open-circuit voltage per cell at `soc` by linear interpolation of the table. `soc` is clamped to [0, 1]; the flag says
/// whether it had to be.
///
/// # Errors
/// `InvalidConfig` if the table length is out of range or `soc` is NaN.
pub fn ocv_per_cell(cfg: &PowertrainConfig, soc: f64) -> Result<(f64, bool), PowertrainError> {
    if soc.is_nan() {
        return Err(PowertrainError::InvalidInput);
    }
    let n = usize::try_from(cfg.ocv_points).map_err(|_| PowertrainError::InvalidConfig)?;
    let socs = cfg.ocv_soc.get(..n).ok_or(PowertrainError::InvalidConfig)?;
    let volts = cfg
        .ocv_v_per_cell
        .get(..n)
        .ok_or(PowertrainError::InvalidConfig)?;
    let clamped = !(0.0..=1.0).contains(&soc);
    let x = soc.clamp(0.0, 1.0);
    for (sw, vw) in socs.windows(2).zip(volts.windows(2)) {
        if let ([s0, s1], [v0, v1]) = (sw, vw)
            && x <= *s1
        {
            return Ok((v0 + (v1 - v0) * (x - s0) / (s1 - s0), clamped));
        }
    }
    volts
        .last()
        .map(|&v| (v, clamped))
        .ok_or(PowertrainError::InvalidConfig)
}

/// Rotor speed below which the advance ratio is not computed from `V / (n D)` (rev/s). Numerical guard against 0/0; far below any
/// operating speed.
const N_FLOOR_RPS: f64 = 1.0e-9;

/// Propeller coefficients at one operating point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropPoint {
    /// Thrust coefficient.
    pub ct: f64,
    /// Power coefficient.
    pub cp: f64,
    /// True if (J, rpm) lies outside the measured data and the nearest edge value was used.
    pub extrapolated: bool,
}

/// One curve of the table at `j`: `(Ct, Cp, beyond_end)`. `j` is clamped to `[0, J_end]`; `beyond_end` says it exceeded the last point.
fn eval_curve(cfg: &PowertrainConfig, k: usize, j: f64) -> Option<(f64, f64, bool)> {
    let m = usize::try_from(*cfg.prop_curve_points.get(k)?).ok()?;
    let js = cfg.prop_j.get(k)?.get(..m)?;
    let cts = cfg.prop_ct_table.get(k)?.get(..m)?;
    let cps = cfg.prop_cp_table.get(k)?.get(..m)?;
    let j_end = *js.last()?;
    let jc = j.clamp(0.0, j_end);
    for ((jw, cw), pw) in js.windows(2).zip(cts.windows(2)).zip(cps.windows(2)) {
        if let ([j0, j1], [c0, c1], [p0, p1]) = (jw, cw, pw)
            && jc <= *j1
        {
            let w = (jc - j0) / (j1 - j0);
            return Some((c0 + w * (c1 - c0), p0 + w * (p1 - p0), j > j_end));
        }
    }
    Some((*cts.last()?, *cps.last()?, j > j_end))
}

/// `Ct` and `Cp` at advance ratio `j` and shaft speed `rpm`.
///
/// `prop_model` 0 returns the constants `prop_ct`/`prop_cp` (never extrapolated). Model 1 interpolates the table: linear in `j` inside
/// each curve, then linear in rpm between the two curves that bracket `rpm`. Outside the data the nearest edge value is HELD (no slope
/// is extrapolated) and `extrapolated` is set: `j < 0`, `j` beyond the last point of EITHER bracketing curve, or `rpm` below the first /
/// above the last curve. The config must have passed `validate_powertrain_config`.
///
/// # Errors
/// `InvalidInput` if `j` or `rpm` is NaN; `InvalidConfig` if the table does not fit its own counts.
pub fn coefficients(
    cfg: &PowertrainConfig,
    j: f64,
    rpm: f64,
) -> Result<PropPoint, PowertrainError> {
    if j.is_nan() || rpm.is_nan() {
        return Err(PowertrainError::InvalidInput);
    }
    if cfg.prop_model != 1 {
        return Ok(PropPoint {
            ct: cfg.prop_ct,
            cp: cfg.prop_cp,
            extrapolated: false,
        });
    }
    let count =
        usize::try_from(cfg.prop_curve_count).map_err(|_| PowertrainError::InvalidConfig)?;
    let rpms = cfg
        .prop_curve_rpm
        .get(..count)
        .ok_or(PowertrainError::InvalidConfig)?;
    let (Some(&first), Some(&last)) = (rpms.first(), rpms.last()) else {
        return Err(PowertrainError::InvalidConfig);
    };
    let r = rpm.clamp(first, last);
    let lower = rpms
        .iter()
        .take(count.saturating_sub(1))
        .rposition(|&x| x <= r)
        .unwrap_or(0);
    let upper = (lower + 1).min(count - 1);
    let (thrust_lo, power_lo, outside_lo) =
        eval_curve(cfg, lower, j).ok_or(PowertrainError::InvalidConfig)?;
    let (thrust_hi, power_hi, outside_hi) =
        eval_curve(cfg, upper, j).ok_or(PowertrainError::InvalidConfig)?;
    let (Some(&rpm_lo), Some(&rpm_hi)) = (rpms.get(lower), rpms.get(upper)) else {
        return Err(PowertrainError::InvalidConfig);
    };
    let t = if upper == lower {
        0.0
    } else {
        (r - rpm_lo) / (rpm_hi - rpm_lo)
    };
    Ok(PropPoint {
        ct: thrust_lo + t * (thrust_hi - thrust_lo),
        cp: power_lo + t * (power_hi - power_lo),
        extrapolated: j < 0.0 || rpm < first || rpm > last || outside_lo || outside_hi,
    })
}

/// Advance ratio `J = V_axial / (n D)`. At a rotor speed below `N_FLOOR_RPS` the ratio is infinite in the direction of the inflow
/// (the table then reads its end value) and 0 without inflow.
#[must_use]
pub fn advance_ratio(axial_inflow_mps: f64, n_rps: f64, d_m: f64) -> f64 {
    if n_rps > N_FLOOR_RPS {
        axial_inflow_mps / (n_rps * d_m)
    } else if axial_inflow_mps > 0.0 {
        f64::INFINITY
    } else if axial_inflow_mps < 0.0 {
        f64::NEG_INFINITY
    } else {
        0.0
    }
}

/// Axial inflow into the rotors, m/s: the air-relative velocity of the vehicle (NED, `v_ground - wind`) projected on the rotor thrust
/// direction (body -Z rotated by the attitude, the convention `mixer` uses for thrust). Positive when the vehicle moves along its
/// thrust direction (climb, or forward flight with the rotors tilted forward). The same value is used for all four rotors; edgewise
/// flow is not modelled. `orientation_xyzw` is the validated body -> NED quaternion in (x, y, z, w) order.
#[must_use]
pub fn axial_inflow_mps(v_air_ned_mps: [f32; 3], orientation_xyzw: [f32; 4]) -> f64 {
    let q = DQuat::from_xyzw(
        f64::from(orientation_xyzw[0]),
        f64::from(orientation_xyzw[1]),
        f64::from(orientation_xyzw[2]),
        f64::from(orientation_xyzw[3]),
    );
    let thrust_dir = q * DVec3::new(0.0, 0.0, -1.0);
    let v = DVec3::new(
        f64::from(v_air_ned_mps[0]),
        f64::from(v_air_ned_mps[1]),
        f64::from(v_air_ned_mps[2]),
    );
    v.dot(thrust_dir)
}

/// Torque constant `Kt = 60 / (2 pi Kv)`, N*m per A.
#[must_use]
pub fn kt_nm_per_a(kv_rpm_per_v: f64) -> f64 {
    60.0 / (2.0 * PI * kv_rpm_per_v)
}

/// Loaded pack voltage: open-circuit voltage minus the resistive drop, floored at 0 (a pack cannot go negative here).
#[must_use]
pub fn loaded_voltage_v(ocv_pack_v: f64, current_a: f64, resistance_ohm: f64) -> f64 {
    (ocv_pack_v - current_a * resistance_ohm).max(0.0)
}

/// One motor + propeller operating point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorPoint {
    /// Shaft speed, rev/s.
    pub n_rps: f64,
    /// Shaft speed, rpm.
    pub rpm: f64,
    /// Motor current, A.
    pub current_a: f64,
    /// Propeller torque, N*m.
    pub torque_nm: f64,
    /// Propeller thrust, N.
    pub thrust_n: f64,
    /// Shaft power `Q omega`, W.
    pub shaft_w: f64,
    /// Terminal power `V I`, W.
    pub electrical_w: f64,
    /// Copper loss `I^2 Rm`, W.
    pub copper_w: f64,
    /// No-load loss `I0 x back-EMF`, W.
    pub noload_w: f64,
    /// Advance ratio of the propeller (0 for a rotor at rest).
    pub advance_ratio: f64,
    /// Thrust coefficient used (0 for a rotor at rest).
    pub ct: f64,
    /// Power coefficient used (0 for a rotor at rest).
    pub cp: f64,
    /// True if the propeller was outside the measured data (always false at rest).
    pub extrapolated: bool,
}

impl MotorPoint {
    const fn at_rest(current_a: f64, electrical_w: f64, copper_w: f64) -> Self {
        Self {
            n_rps: 0.0,
            rpm: 0.0,
            current_a,
            torque_nm: 0.0,
            thrust_n: 0.0,
            shaft_w: 0.0,
            electrical_w,
            copper_w,
            noload_w: 0.0,
            advance_ratio: 0.0,
            ct: 0.0,
            cp: 0.0,
            extrapolated: false,
        }
    }
}

/// Solves the motor + propeller torque balance at terminal voltage `v_terminal_v` and air density `rho_kgpm3`, with the vehicle
/// moving `axial_inflow_mps` along the rotor axis (0 in hover).
///
/// `g(n) = 60 n - Kv (V - Rm (Q(n)/Kt + I0))` is increasing in `n` (`Cp` falls with `J` and `J` falls with `n`, so `Q = Cp(J(n)) n^2 ...`
/// grows with `n`); bisection on `[0, Kv V / 60]`. A zero or negative voltage gives an idle motor (0 current: no regeneration). If
/// `V <= I0 Rm` the motor cannot overcome the no-load drop and stays locked, drawing `V / Rm`.
///
/// # Errors
/// `InvalidInput` for a non-finite voltage or inflow, or a non-positive density; `NonFinite` if a result is not finite.
pub fn solve_motor(
    cfg: &PowertrainConfig,
    v_terminal_v: f64,
    rho_kgpm3: f64,
    axial_inflow_mps: f64,
) -> Result<MotorPoint, PowertrainError> {
    if !v_terminal_v.is_finite() || !axial_inflow_mps.is_finite() || !positive_finite(rho_kgpm3) {
        return Err(PowertrainError::InvalidInput);
    }
    let (kv, rm, i0) = (
        cfg.motor_kv_rpm_per_v,
        cfg.motor_resistance_ohm,
        cfg.motor_no_load_current_a,
    );
    if v_terminal_v <= 0.0 {
        return Ok(MotorPoint::at_rest(0.0, 0.0, 0.0));
    }
    if v_terminal_v <= i0 * rm {
        let locked = v_terminal_v / rm;
        return Ok(MotorPoint::at_rest(
            locked,
            v_terminal_v * locked,
            locked * locked * rm,
        ));
    }
    let kt = kt_nm_per_a(kv);
    let d = cfg.prop_diameter_m;
    let torque_per_cp = rho_kgpm3 * d.powi(5) / (2.0 * PI);
    let (mut lo, mut hi) = (0.0_f64, kv * v_terminal_v / 60.0);
    for _ in 0..BISECTION_STEPS {
        let mid = 0.5 * (lo + hi);
        let cp = coefficients(cfg, advance_ratio(axial_inflow_mps, mid, d), 60.0 * mid)?.cp;
        let q_coef = cp * torque_per_cp;
        let balance = 60.0 * mid - kv * (v_terminal_v - rm * (q_coef * mid * mid / kt + i0));
        if balance > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let n = 0.5 * (lo + hi);
    let j = advance_ratio(axial_inflow_mps, n, d);
    let prop = coefficients(cfg, j, 60.0 * n)?;
    let torque = prop.cp * torque_per_cp * n * n;
    let current = torque / kt + i0;
    let back_emf = 60.0 * n / kv;
    let point = MotorPoint {
        n_rps: n,
        rpm: 60.0 * n,
        current_a: current,
        torque_nm: torque,
        thrust_n: prop.ct * rho_kgpm3 * n * n * d.powi(4),
        shaft_w: torque * 2.0 * PI * n,
        electrical_w: v_terminal_v * current,
        copper_w: current * current * rm,
        noload_w: i0 * back_emf,
        advance_ratio: j,
        ct: prop.ct,
        cp: prop.cp,
        extrapolated: prop.extrapolated,
    };
    let finite = [
        point.rpm,
        point.current_a,
        point.torque_nm,
        point.thrust_n,
        point.shaft_w,
        point.electrical_w,
        point.advance_ratio,
    ]
    .iter()
    .all(|v| v.is_finite());
    if finite {
        Ok(point)
    } else {
        Err(PowertrainError::NonFinite)
    }
}

/// Stateless bench point: one motor + propeller + ESC at a fixed pack voltage and duty (the manufacturer's thrust-table scenario),
/// with the vehicle moving `axial_inflow_mps` along the rotor axis (0 for a static bench).
///
/// # Errors
/// `InvalidInput` if `duty` is outside [0, 1] or not finite, `v_pack_v` is negative or not finite, or `rho_kgpm3` is not
/// positive; otherwise the errors of `solve_motor`.
pub fn bench_point(
    cfg: &PowertrainConfig,
    duty: f64,
    v_pack_v: f64,
    rho_kgpm3: f64,
    axial_inflow_mps: f64,
) -> Result<PowertrainBenchOut, PowertrainError> {
    if !(0.0..=1.0).contains(&duty) || !nonnegative_finite(v_pack_v) {
        return Err(PowertrainError::InvalidInput);
    }
    let m = solve_motor(cfg, duty * v_pack_v, rho_kgpm3, axial_inflow_mps)?;
    let battery_w = m.electrical_w / cfg.esc_efficiency;
    let battery_a = if v_pack_v > 0.0 {
        battery_w / v_pack_v
    } else {
        0.0
    };
    Ok(PowertrainBenchOut {
        rpm: m.rpm,
        motor_current_a: m.current_a,
        torque_nm: m.torque_nm,
        thrust_n: m.thrust_n,
        shaft_w: m.shaft_w,
        motor_electrical_w: m.electrical_w,
        copper_loss_w: m.copper_w,
        noload_loss_w: m.noload_w,
        esc_loss_w: battery_w - m.electrical_w,
        battery_current_a: battery_a,
        battery_power_w: battery_w,
        system_efficiency: if battery_w > 0.0 {
            m.shaft_w / battery_w
        } else {
            0.0
        },
        advance_ratio: m.advance_ratio,
        ct: m.ct,
        cp: m.cp,
        extrapolated: i32::from(m.extrapolated),
        reserved: 0,
    })
}

/// Forced-convection factor `1 + (v/v_ref)^0.8` (>= 1); the thermal resistance to ambient is divided by it.
#[must_use]
pub fn cooling_factor(v_eff_mps: f64, v_ref_mps: f64) -> f64 {
    1.0 + (v_eff_mps.max(0.0) / v_ref_mps).powf(COOLING_EXPONENT)
}

/// Induced velocity of a rotor in hover, `sqrt(T / (2 rho A))` with `A = pi D^2 / 4` (momentum theory): the prop wash
/// that cools the motor even when the vehicle is not moving.
#[must_use]
pub fn induced_velocity_mps(thrust_n: f64, rho_kgpm3: f64, d_m: f64) -> f64 {
    let area = PI * d_m * d_m / 4.0;
    (thrust_n.max(0.0) / (2.0 * rho_kgpm3 * area)).sqrt()
}

/// One forward-Euler step of a single thermal node: `C dT/dt = P - (T - T_amb) / R`.
#[must_use]
pub fn euler_node(
    temp_c: f64,
    heat_w: f64,
    r_k_per_w: f64,
    c_j_per_k: f64,
    ambient_c: f64,
    dt_s: f64,
) -> f64 {
    temp_c + dt_s * (heat_w - (temp_c - ambient_c) / r_k_per_w) / c_j_per_k
}

/// Inputs of one powertrain step.
#[derive(Debug, Clone, Copy)]
pub struct StepInput {
    /// Motor duty (mixer throttle), each in [0, 1].
    pub duties: [f64; MOTOR_COUNT],
    /// Air density, kg/m^3.
    pub rho_kgpm3: f64,
    /// Air-relative speed of the vehicle, m/s (cooling).
    pub airspeed_mps: f64,
    /// Axial inflow into the rotors, m/s (see `axial_inflow_mps`); positive when the vehicle moves along its thrust direction.
    pub axial_inflow_mps: f64,
    /// Ambient air temperature, deg C.
    pub ambient_c: f64,
    /// Time step, s.
    pub dt_s: f64,
}

fn validate_step_input(input: &StepInput) -> Result<(), PowertrainError> {
    let ok = input.duties.iter().all(|d| (0.0..=1.0).contains(d))
        && positive_finite(input.rho_kgpm3)
        && nonnegative_finite(input.airspeed_mps)
        && input.axial_inflow_mps.is_finite()
        && input.ambient_c.is_finite()
        && positive_finite(input.dt_s);
    if ok {
        Ok(())
    } else {
        Err(PowertrainError::InvalidInput)
    }
}

/// Electrical stage results for the four motors and the pack.
struct Electrical {
    points: [MotorPoint; MOTOR_COUNT],
    terminal_v: [f64; MOTOR_COUNT],
    esc_loss_w: [f64; MOTOR_COUNT],
    esc_input_a: [f64; MOTOR_COUNT],
    ocv_pack_v: f64,
    loaded_v: f64,
    pack_a: f64,
    pack_w: f64,
    soc_clamped: bool,
}

fn electrical_stage(
    cfg: &PowertrainConfig,
    state: &PowertrainState,
    input: &StepInput,
) -> Result<Electrical, PowertrainError> {
    let (ocv_cell, soc_clamped) = ocv_per_cell(cfg, state.soc)?;
    let ocv_pack_v = ocv_cell * f64::from(cfg.battery_cells);
    let supply_v = loaded_voltage_v(
        ocv_pack_v,
        state.battery_current_a,
        cfg.battery_internal_resistance_ohm,
    );
    let mut points = [MotorPoint::at_rest(0.0, 0.0, 0.0); MOTOR_COUNT];
    let mut terminal_v = [0.0; MOTOR_COUNT];
    for ((point, v_m), duty) in points
        .iter_mut()
        .zip(terminal_v.iter_mut())
        .zip(input.duties.iter())
    {
        *v_m = duty * supply_v;
        *point = solve_motor(cfg, *v_m, input.rho_kgpm3, input.axial_inflow_mps)?;
    }
    let esc_loss_w = points.map(|p| p.electrical_w * (1.0 / cfg.esc_efficiency - 1.0));
    let pack_w: f64 = points.iter().map(|p| p.electrical_w).sum::<f64>() / cfg.esc_efficiency;
    let pack_a = if supply_v > 0.0 {
        pack_w / supply_v
    } else {
        0.0
    };
    let esc_input_a = points.map(|p| {
        if supply_v > 0.0 {
            p.electrical_w / (cfg.esc_efficiency * supply_v)
        } else {
            0.0
        }
    });
    let loaded_v = loaded_voltage_v(ocv_pack_v, pack_a, cfg.battery_internal_resistance_ohm);
    Ok(Electrical {
        points,
        terminal_v,
        esc_loss_w,
        esc_input_a,
        ocv_pack_v,
        loaded_v,
        pack_a,
        pack_w,
        soc_clamped,
    })
}

/// Thermal resistances to ambient with the cooling factor applied, for the stability guard and the update.
struct Cooling {
    motor_case: [f64; MOTOR_COUNT],
    esc: f64,
    battery: f64,
}

fn cooling(
    cfg: &PowertrainConfig,
    points: &[MotorPoint; MOTOR_COUNT],
    input: &StepInput,
) -> Cooling {
    let case_r = points.map(|p| {
        let wash = induced_velocity_mps(p.thrust_n, input.rho_kgpm3, cfg.prop_diameter_m);
        cfg.motor_r_case_ambient_k_per_w
            / cooling_factor(input.airspeed_mps + wash, cfg.cooling_ref_airspeed_mps)
    });
    // ESC and battery are not assumed to sit in the prop wash (their position is unknown): vehicle airspeed only.
    let free = cooling_factor(input.airspeed_mps, cfg.cooling_ref_airspeed_mps);
    Cooling {
        motor_case: case_r,
        esc: cfg.esc_r_ambient_k_per_w / free,
        battery: cfg.battery_r_ambient_k_per_w / free,
    }
}

fn check_thermal_step(
    cfg: &PowertrainConfig,
    cool: &Cooling,
    dt_s: f64,
) -> Result<(), PowertrainError> {
    let case_r_min = cool
        .motor_case
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    let tau_min = [
        cfg.motor_r_winding_case_k_per_w * cfg.motor_c_winding_j_per_k,
        cfg.motor_r_winding_case_k_per_w * cfg.motor_c_case_j_per_k,
        case_r_min * cfg.motor_c_case_j_per_k,
        cool.esc * cfg.esc_c_j_per_k,
        cool.battery * cfg.battery_c_j_per_k,
    ]
    .into_iter()
    .fold(f64::INFINITY, f64::min);
    if dt_s <= THERMAL_STEP_FRACTION * tau_min {
        Ok(())
    } else {
        Err(PowertrainError::ThermalStep)
    }
}

fn thermal_stage(
    cfg: &PowertrainConfig,
    next: &mut PowertrainState,
    elec: &Electrical,
    cool: &Cooling,
    input: &StepInput,
    battery_heat_w: f64,
) {
    let (ta, dt) = (input.ambient_c, input.dt_s);
    let old = *next;
    for (i, point) in elec.points.iter().enumerate() {
        let (Some(tw), Some(tc), Some(te)) = (
            old.motor_winding_c.get(i),
            old.motor_case_c.get(i),
            old.esc_temp_c.get(i),
        ) else {
            continue;
        };
        let (Some(r_case), Some(esc_loss)) = (cool.motor_case.get(i), elec.esc_loss_w.get(i))
        else {
            continue;
        };
        let flow = (tw - tc) / cfg.motor_r_winding_case_k_per_w;
        let new_w = tw + dt * (point.copper_w - flow) / cfg.motor_c_winding_j_per_k;
        let new_c =
            tc + dt * (point.noload_w + flow - (tc - ta) / r_case) / cfg.motor_c_case_j_per_k;
        let new_e = euler_node(*te, *esc_loss, cool.esc, cfg.esc_c_j_per_k, ta, dt);
        if let (Some(w), Some(c), Some(e)) = (
            next.motor_winding_c.get_mut(i),
            next.motor_case_c.get_mut(i),
            next.esc_temp_c.get_mut(i),
        ) {
            (*w, *c, *e) = (new_w, new_c, new_e);
        }
    }
    next.battery_temp_c = euler_node(
        old.battery_temp_c,
        battery_heat_w,
        cool.battery,
        cfg.battery_c_j_per_k,
        ta,
        dt,
    );
}

/// Bit i set if propeller i was outside the measured data this step.
fn prop_mask(points: &[MotorPoint; MOTOR_COUNT]) -> i32 {
    points.iter().enumerate().fold(0, |mask, (i, p)| {
        if p.extrapolated {
            mask | (1 << i)
        } else {
            mask
        }
    })
}

fn mask_where(values: &[f64; MOTOR_COUNT], limit: f64) -> i32 {
    values.iter().enumerate().fold(
        0,
        |mask, (i, &v)| if v > limit { mask | (1 << i) } else { mask },
    )
}

/// Advances the powertrain by one step and returns the telemetry. `state` is only written on success.
///
/// Order: battery supply voltage (previous current) -> motor/ESC solve -> pack current, SOC, cutoff -> thermal update.
///
/// # Errors
/// `InvalidConfig`, `InvalidState`, `InvalidInput` for rejected inputs; `ThermalStep` if `dt` is too large for the thermal
/// network; `NonFinite` if any result is not finite. On any error `state` is unchanged.
pub fn step(
    cfg: &PowertrainConfig,
    state: &mut PowertrainState,
    input: &StepInput,
) -> Result<PowertrainTelemetry, PowertrainError> {
    validate_powertrain_config(cfg)?;
    validate_state(state)?;
    validate_step_input(input)?;
    let elec = electrical_stage(cfg, state, input)?;
    let cool = cooling(cfg, &elec.points, input);
    check_thermal_step(cfg, &cool, input.dt_s)?;

    let mut next = *state;
    let charge_ah = elec.pack_a * input.dt_s / SECONDS_PER_HOUR;
    let soc_raw = state.soc - charge_ah / cfg.battery_capacity_ah;
    next.soc = soc_raw.max(0.0);
    next.charge_used_ah = state.charge_used_ah + charge_ah;
    next.battery_current_a = elec.pack_a;
    let cutoff = elec.loaded_v < f64::from(cfg.battery_cells) * cfg.battery_cutoff_v_per_cell;
    next.time_below_cutoff_s = if cutoff {
        state.time_below_cutoff_s + input.dt_s
    } else {
        0.0
    };
    let battery_heat_w = elec.pack_a * elec.pack_a * cfg.battery_internal_resistance_ohm;
    thermal_stage(cfg, &mut next, &elec, &cool, input, battery_heat_w);
    next.prop_extrapolated_time_s = state.prop_extrapolated_time_s
        + if prop_mask(&elec.points) == 0 {
            0.0
        } else {
            input.dt_s
        };

    let t = build_telemetry(cfg, &next, &elec, battery_heat_w, cutoff, soc_raw < 0.0);
    let finite = next.soc.is_finite()
        && elec
            .points
            .iter()
            .all(|p| p.rpm.is_finite() && p.current_a.is_finite() && p.thrust_n.is_finite())
        && validate_state(&next).is_ok();
    if !finite {
        return Err(PowertrainError::NonFinite);
    }
    *state = next;
    Ok(t)
}

fn build_telemetry(
    cfg: &PowertrainConfig,
    next: &PowertrainState,
    elec: &Electrical,
    battery_heat_w: f64,
    cutoff: bool,
    soc_depleted: bool,
) -> PowertrainTelemetry {
    let p = &elec.points;
    PowertrainTelemetry {
        motor_current_a: p.map(|m| m.current_a),
        motor_voltage_v: elec.terminal_v,
        motor_rpm: p.map(|m| m.rpm),
        motor_torque_nm: p.map(|m| m.torque_nm),
        motor_thrust_n: p.map(|m| m.thrust_n),
        motor_copper_loss_w: p.map(|m| m.copper_w),
        motor_noload_loss_w: p.map(|m| m.noload_w),
        esc_loss_w: elec.esc_loss_w,
        motor_winding_c: next.motor_winding_c,
        motor_case_c: next.motor_case_c,
        esc_temp_c: next.esc_temp_c,
        battery_voltage_v: elec.loaded_v,
        battery_ocv_v: elec.ocv_pack_v,
        battery_current_a: elec.pack_a,
        battery_power_w: elec.pack_w,
        battery_soc: next.soc,
        battery_temp_c: next.battery_temp_c,
        battery_heat_w,
        charge_used_ah: next.charge_used_ah,
        time_below_cutoff_s: next.time_below_cutoff_s,
        step_valid: 1,
        battery_cutoff: i32::from(cutoff),
        battery_over_current: i32::from(elec.pack_a > cfg.battery_max_continuous_a),
        battery_over_temp: i32::from(next.battery_temp_c > cfg.battery_temp_limit_c),
        battery_soc_clamped: i32::from(elec.soc_clamped || soc_depleted),
        motor_over_current_mask: mask_where(&p.map(|m| m.current_a), cfg.motor_max_current_a),
        motor_over_temp_mask: mask_where(&next.motor_winding_c, cfg.motor_winding_temp_limit_c),
        esc_over_current_mask: mask_where(&elec.esc_input_a, cfg.esc_max_current_a),
        esc_over_temp_mask: mask_where(&next.esc_temp_c, cfg.esc_temp_limit_c),
        thermal_valid: cfg.parameters_verified,
        prop_extrapolated: prop_mask(p),
        reserved0: 0,
        reserved1: 0,
        reserved2: 0,
        motor_advance_ratio: p.map(|m| m.advance_ratio),
        motor_ct: p.map(|m| m.ct),
        motor_cp: p.map(|m| m.cp),
    }
}

/// True if any temperature or electrical flag of the telemetry is set (the legacy `thermal_over_temp` meaning).
#[must_use]
pub fn any_over_temp(t: &PowertrainTelemetry) -> bool {
    t.motor_over_temp_mask != 0 || t.esc_over_temp_mask != 0 || t.battery_over_temp != 0
}

/// JSON form of `PowertrainConfig` (`powertrain_config.json`): flat keys with the struct's names, and the OCV table as
/// `ocv_curve = [[soc, volts_per_cell], ...]`. Unknown keys are rejected.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PowertrainConfigJson {
    motor_kv_rpm_per_v: f64,
    motor_resistance_ohm: f64,
    motor_no_load_current_a: f64,
    motor_max_current_a: f64,
    motor_winding_temp_limit_c: f64,
    prop_diameter_m: f64,
    prop_ct: f64,
    prop_cp: f64,
    esc_efficiency: f64,
    esc_max_current_a: f64,
    esc_temp_limit_c: f64,
    battery_capacity_ah: f64,
    battery_internal_resistance_ohm: f64,
    battery_cutoff_v_per_cell: f64,
    battery_max_continuous_a: f64,
    battery_temp_limit_c: f64,
    motor_r_winding_case_k_per_w: f64,
    motor_r_case_ambient_k_per_w: f64,
    motor_c_winding_j_per_k: f64,
    motor_c_case_j_per_k: f64,
    esc_r_ambient_k_per_w: f64,
    esc_c_j_per_k: f64,
    battery_r_ambient_k_per_w: f64,
    battery_c_j_per_k: f64,
    cooling_ref_airspeed_mps: f64,
    battery_cells: i32,
    parameters_verified: i32,
    ocv_curve: Vec<[f64; 2]>,
    /// Optional propeller table: `[{"rpm": r, "points": [[J, Ct, Cp], ...]}, ...]`. Absent: the static `prop_ct`/`prop_cp`.
    prop_curves: Option<Vec<CurveJson>>,
}

/// One propeller speed curve of the JSON form.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CurveJson {
    rpm: f64,
    points: Vec<[f64; 3]>,
}

/// Why a powertrain config file was not loaded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadError {
    /// The file could not be read.
    Io,
    /// Malformed JSON, a missing or unknown key, or an OCV table that does not fit.
    Parse,
    /// Parsed, but `validate_powertrain_config` rejected the values.
    Invalid,
}

/// Parses and validates `powertrain_config.json` text.
///
/// # Errors
/// `Parse` for bad JSON or a table longer than `MAX_OCV_POINTS`; `Invalid` for out-of-range values.
pub fn parse_powertrain_config(json: &str) -> Result<PowertrainConfig, LoadError> {
    let raw: PowertrainConfigJson = serde_json::from_str(json).map_err(|_| LoadError::Parse)?;
    if raw.ocv_curve.len() > MAX_OCV_POINTS {
        return Err(LoadError::Parse);
    }
    let mut ocv_soc = [0.0; MAX_OCV_POINTS];
    let mut ocv_v = [0.0; MAX_OCV_POINTS];
    for ((point, soc), volts) in raw
        .ocv_curve
        .iter()
        .zip(ocv_soc.iter_mut())
        .zip(ocv_v.iter_mut())
    {
        (*soc, *volts) = (point[0], point[1]);
    }
    let table = prop_table_fields(raw.prop_curves.as_deref())?;
    let cfg = PowertrainConfig {
        motor_kv_rpm_per_v: raw.motor_kv_rpm_per_v,
        motor_resistance_ohm: raw.motor_resistance_ohm,
        motor_no_load_current_a: raw.motor_no_load_current_a,
        motor_max_current_a: raw.motor_max_current_a,
        motor_winding_temp_limit_c: raw.motor_winding_temp_limit_c,
        prop_diameter_m: raw.prop_diameter_m,
        prop_ct: raw.prop_ct,
        prop_cp: raw.prop_cp,
        esc_efficiency: raw.esc_efficiency,
        esc_max_current_a: raw.esc_max_current_a,
        esc_temp_limit_c: raw.esc_temp_limit_c,
        battery_capacity_ah: raw.battery_capacity_ah,
        battery_internal_resistance_ohm: raw.battery_internal_resistance_ohm,
        battery_cutoff_v_per_cell: raw.battery_cutoff_v_per_cell,
        battery_max_continuous_a: raw.battery_max_continuous_a,
        battery_temp_limit_c: raw.battery_temp_limit_c,
        motor_r_winding_case_k_per_w: raw.motor_r_winding_case_k_per_w,
        motor_r_case_ambient_k_per_w: raw.motor_r_case_ambient_k_per_w,
        motor_c_winding_j_per_k: raw.motor_c_winding_j_per_k,
        motor_c_case_j_per_k: raw.motor_c_case_j_per_k,
        esc_r_ambient_k_per_w: raw.esc_r_ambient_k_per_w,
        esc_c_j_per_k: raw.esc_c_j_per_k,
        battery_r_ambient_k_per_w: raw.battery_r_ambient_k_per_w,
        battery_c_j_per_k: raw.battery_c_j_per_k,
        cooling_ref_airspeed_mps: raw.cooling_ref_airspeed_mps,
        ocv_soc,
        ocv_v_per_cell: ocv_v,
        battery_cells: raw.battery_cells,
        ocv_points: i32::try_from(raw.ocv_curve.len()).map_err(|_| LoadError::Parse)?,
        parameters_verified: raw.parameters_verified,
        reserved: 0,
        prop_curve_rpm: table.rpm,
        prop_j: table.j,
        prop_ct_table: table.ct,
        prop_cp_table: table.cp,
        prop_curve_points: table.points,
        prop_curve_count: table.count,
        prop_model: table.model,
    };
    validate_powertrain_config(&cfg).map_err(|_| LoadError::Invalid)?;
    Ok(cfg)
}

/// The fixed-size propeller table fields built from the JSON curves.
struct PropTableFields {
    rpm: [f64; MAX_PROP_CURVES],
    j: [[f64; MAX_PROP_POINTS]; MAX_PROP_CURVES],
    ct: [[f64; MAX_PROP_POINTS]; MAX_PROP_CURVES],
    cp: [[f64; MAX_PROP_POINTS]; MAX_PROP_CURVES],
    points: [i32; MAX_PROP_CURVES],
    count: i32,
    model: i32,
}

fn prop_table_fields(curves: Option<&[CurveJson]>) -> Result<PropTableFields, LoadError> {
    let mut t = PropTableFields {
        rpm: [0.0; MAX_PROP_CURVES],
        j: [[0.0; MAX_PROP_POINTS]; MAX_PROP_CURVES],
        ct: [[0.0; MAX_PROP_POINTS]; MAX_PROP_CURVES],
        cp: [[0.0; MAX_PROP_POINTS]; MAX_PROP_CURVES],
        points: [0; MAX_PROP_CURVES],
        count: 0,
        model: 0,
    };
    let Some(curves) = curves else { return Ok(t) };
    if curves.len() > MAX_PROP_CURVES || curves.iter().any(|c| c.points.len() > MAX_PROP_POINTS) {
        return Err(LoadError::Parse);
    }
    t.count = i32::try_from(curves.len()).map_err(|_| LoadError::Parse)?;
    t.model = 1;
    let rows = t
        .rpm
        .iter_mut()
        .zip(t.j.iter_mut())
        .zip(t.ct.iter_mut())
        .zip(t.cp.iter_mut())
        .zip(t.points.iter_mut())
        .zip(curves.iter());
    for (((((rpm, js), cts), cps), count), curve) in rows {
        *rpm = curve.rpm;
        *count = i32::try_from(curve.points.len()).map_err(|_| LoadError::Parse)?;
        let cells = js
            .iter_mut()
            .zip(cts.iter_mut())
            .zip(cps.iter_mut())
            .zip(curve.points.iter());
        for (((j, ct), cp), point) in cells {
            (*j, *ct, *cp) = (point[0], point[1], point[2]);
        }
    }
    Ok(t)
}

/// Reads and parses a `powertrain_config.json` file.
///
/// # Errors
/// `Io` if the file cannot be read, otherwise see `parse_powertrain_config`.
pub fn load_powertrain_config(path: &str) -> Result<PowertrainConfig, LoadError> {
    let text = fs::read_to_string(path).map_err(|_| LoadError::Io)?;
    parse_powertrain_config(&text)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    type CfgCase = (&'static str, Box<dyn Fn(&mut PowertrainConfig)>);
    type StateMutator = Box<dyn Fn(&mut PowertrainState)>;

    const RHO: f64 = 1.2250122659906946; // ISA sea level: 101325 / (287.05 * 288.15)
    const CP: f64 = 0.031_874_666_666_666_66; // UIUC APC 12x6E static fit (tools/props/fetch_uiuc.py)

    /// MN3508 KV700 + APC 12x6E + 4S pack. Thermal numbers are round TEST values chosen for hand calculation.
    pub(crate) fn ref_cfg() -> PowertrainConfig {
        let mut ocv_soc = [0.0; MAX_OCV_POINTS];
        let mut ocv_v = [0.0; MAX_OCV_POINTS];
        let curve = [
            (0.0, 3.30),
            (0.1, 3.62),
            (0.2, 3.70),
            (0.3, 3.75),
            (0.5, 3.82),
            (0.7, 3.92),
            (0.9, 4.08),
            (1.0, 4.20),
        ];
        for (i, (s, v)) in curve.iter().enumerate() {
            ocv_soc[i] = *s;
            ocv_v[i] = *v;
        }
        PowertrainConfig {
            motor_kv_rpm_per_v: 700.0,
            motor_resistance_ohm: 0.072,
            motor_no_load_current_a: 0.5,
            motor_max_current_a: 23.0,
            motor_winding_temp_limit_c: 120.0,
            prop_diameter_m: 0.3048,
            prop_ct: 0.100_256,
            prop_cp: CP,
            esc_efficiency: 0.95,
            esc_max_current_a: 40.0,
            esc_temp_limit_c: 100.0,
            battery_capacity_ah: 5.0,
            battery_internal_resistance_ohm: 0.02,
            battery_cutoff_v_per_cell: 3.3,
            battery_max_continuous_a: 125.0,
            battery_temp_limit_c: 60.0,
            motor_r_winding_case_k_per_w: 0.2,
            motor_r_case_ambient_k_per_w: 0.4,
            motor_c_winding_j_per_k: 20.0,
            motor_c_case_j_per_k: 60.0,
            esc_r_ambient_k_per_w: 2.0,
            esc_c_j_per_k: 10.0,
            battery_r_ambient_k_per_w: 3.0,
            battery_c_j_per_k: 300.0,
            cooling_ref_airspeed_mps: 5.0,
            ocv_soc,
            ocv_v_per_cell: ocv_v,
            battery_cells: 4,
            ocv_points: 8,
            parameters_verified: 0,
            reserved: 0,
            prop_curve_rpm: [0.0; MAX_PROP_CURVES],
            prop_j: [[0.0; MAX_PROP_POINTS]; MAX_PROP_CURVES],
            prop_ct_table: [[0.0; MAX_PROP_POINTS]; MAX_PROP_CURVES],
            prop_cp_table: [[0.0; MAX_PROP_POINTS]; MAX_PROP_CURVES],
            prop_curve_points: [0; MAX_PROP_CURVES],
            prop_curve_count: 0,
            prop_model: 0,
        }
    }

    /// Independent closed-form solution of the static torque balance (a quadratic in n), rev/s.
    fn quadratic_rps(cfg: &PowertrainConfig, v: f64) -> f64 {
        let kt = 60.0 / (2.0 * PI * cfg.motor_kv_rpm_per_v);
        let q = cfg.prop_cp * RHO * cfg.prop_diameter_m.powi(5) / (2.0 * PI);
        let a = cfg.motor_kv_rpm_per_v * cfg.motor_resistance_ohm * q / kt;
        let c =
            cfg.motor_kv_rpm_per_v * (v - cfg.motor_resistance_ohm * cfg.motor_no_load_current_a);
        (-60.0 + (3600.0 + 4.0 * a * c).sqrt()) / (2.0 * a)
    }

    fn input(duty: f64) -> StepInput {
        StepInput {
            duties: [duty; 4],
            rho_kgpm3: RHO,
            airspeed_mps: 0.0,
            axial_inflow_mps: 0.0,
            ambient_c: 20.0,
            dt_s: 0.01,
        }
    }

    // ---------------------------------------------------------------- validation
    #[test]
    fn test_pt_reference_config_is_valid() {
        assert_eq!(validate_powertrain_config(&ref_cfg()), Ok(()));
    }

    #[test]
    fn test_pt_config_rejects_each_bad_field() {
        let bad: Vec<CfgCase> = vec![
            ("kv zero", Box::new(|c| c.motor_kv_rpm_per_v = 0.0)),
            ("rm negative", Box::new(|c| c.motor_resistance_ohm = -0.1)),
            (
                "i0 negative",
                Box::new(|c| c.motor_no_load_current_a = -0.1),
            ),
            ("i0 nan", Box::new(|c| c.motor_no_load_current_a = f64::NAN)),
            (
                "max current inf",
                Box::new(|c| c.motor_max_current_a = f64::INFINITY),
            ),
            ("ct zero", Box::new(|c| c.prop_ct = 0.0)),
            ("eta zero", Box::new(|c| c.esc_efficiency = 0.0)),
            ("eta above one", Box::new(|c| c.esc_efficiency = 1.0001)),
            ("cells zero", Box::new(|c| c.battery_cells = 0)),
            ("cells 15", Box::new(|c| c.battery_cells = 15)),
            ("capacity zero", Box::new(|c| c.battery_capacity_ah = 0.0)),
            (
                "r_int negative",
                Box::new(|c| c.battery_internal_resistance_ohm = -0.01),
            ),
            (
                "cutoff zero",
                Box::new(|c| c.battery_cutoff_v_per_cell = 0.0),
            ),
            (
                "thermal r zero",
                Box::new(|c| c.motor_r_winding_case_k_per_w = 0.0),
            ),
            ("thermal c nan", Box::new(|c| c.esc_c_j_per_k = f64::NAN)),
            ("v_ref zero", Box::new(|c| c.cooling_ref_airspeed_mps = 0.0)),
            ("limit nan", Box::new(|c| c.battery_temp_limit_c = f64::NAN)),
            ("verified 2", Box::new(|c| c.parameters_verified = 2)),
            ("reserved set", Box::new(|c| c.reserved = 1)),
            ("ocv one point", Box::new(|c| c.ocv_points = 1)),
            ("ocv 17 points", Box::new(|c| c.ocv_points = 17)),
            ("ocv negative count", Box::new(|c| c.ocv_points = -1)),
            ("ocv soc starts at 0.1", Box::new(|c| c.ocv_soc[0] = 0.1)),
            ("ocv soc ends at 0.99", Box::new(|c| c.ocv_soc[7] = 0.99)),
            (
                "ocv soc not increasing",
                Box::new(|c| c.ocv_soc[3] = c.ocv_soc[2]),
            ),
            (
                "ocv voltage decreasing",
                Box::new(|c| c.ocv_v_per_cell[4] = 3.0),
            ),
            ("ocv voltage zero", Box::new(|c| c.ocv_v_per_cell[0] = 0.0)),
            (
                "ocv unused entry nan",
                Box::new(|c| c.ocv_soc[15] = f64::NAN),
            ),
        ];
        for (name, mutate) in bad {
            let mut cfg = ref_cfg();
            mutate(&mut cfg);
            assert_eq!(
                validate_powertrain_config(&cfg),
                Err(PowertrainError::InvalidConfig),
                "{name}"
            );
        }
    }

    #[test]
    fn test_pt_state_validation() {
        let good = initial_state(1.0, 20.0).unwrap();
        assert_eq!(validate_state(&good), Ok(()));
        let bad: Vec<StateMutator> = vec![
            Box::new(|s| s.soc = -0.01),
            Box::new(|s| s.soc = 1.01),
            Box::new(|s| s.soc = f64::NAN),
            Box::new(|s| s.motor_winding_c[2] = f64::NAN),
            Box::new(|s| s.motor_case_c[0] = 3000.0),
            Box::new(|s| s.esc_temp_c[3] = -200.0),
            Box::new(|s| s.battery_temp_c = f64::INFINITY),
            Box::new(|s| s.charge_used_ah = -1.0),
            Box::new(|s| s.battery_current_a = -1.0),
            Box::new(|s| s.time_below_cutoff_s = f64::NAN),
        ];
        for (i, mutate) in bad.iter().enumerate() {
            let mut s = good;
            mutate(&mut s);
            assert_eq!(
                validate_state(&s),
                Err(PowertrainError::InvalidState),
                "case {i}"
            );
        }
        assert_eq!(
            initial_state(1.5, 20.0).err(),
            Some(PowertrainError::InvalidState)
        );
        assert_eq!(
            initial_state(0.5, f64::NAN).err(),
            Some(PowertrainError::InvalidState)
        );
    }

    #[test]
    fn test_pt_consistency_with_vehicle_config() {
        let cfg = ref_cfg();
        // kappa = Cp D / (2 pi Ct) = 0.0318747 * 0.3048 / (2 pi * 0.100256) = 0.015423
        let mut v = VehicleConfig {
            mass_kg: 1.5,
            inertia_kgm2: [0.03; 3],
            motor_pos_m: [[0.2, 0.2, 0.0]; 4],
            motor_spin: [1.0, -1.0, -1.0, 1.0],
            prop_diameter_m: 0.3048,
            thrust_coeff: 0.100_256,
            torque_coeff: 0.015_423,
            max_rpm: 8000.0,
            hover_throttle: 0.4,
        };
        assert_eq!(check_consistency(&v, &cfg), Ok(()));
        v.prop_diameter_m = 0.3;
        assert_eq!(
            check_consistency(&v, &cfg),
            Err(PowertrainError::InvalidConfig)
        );
        v.prop_diameter_m = 0.3048;
        v.thrust_coeff = 0.11;
        assert_eq!(
            check_consistency(&v, &cfg),
            Err(PowertrainError::InvalidConfig)
        );
        v.thrust_coeff = 0.100_256;
        v.torque_coeff = 0.02;
        assert_eq!(
            check_consistency(&v, &cfg),
            Err(PowertrainError::InvalidConfig)
        );
    }

    // ---------------------------------------------------------------- battery
    #[test]
    fn test_pt_ocv_interpolation_hand_values() {
        let cfg = ref_cfg();
        // between (0.5, 3.82) and (0.7, 3.92): at 0.6 -> 3.82 + 0.1 * (0.1 / 0.2) = 3.87
        assert!((ocv_per_cell(&cfg, 0.6).unwrap().0 - 3.87).abs() < 1e-12);
        // knots are exact; ends of the table
        assert_eq!(ocv_per_cell(&cfg, 0.0).unwrap(), (3.30, false));
        assert_eq!(ocv_per_cell(&cfg, 1.0).unwrap(), (4.20, false));
        assert!((ocv_per_cell(&cfg, 0.5).unwrap().0 - 3.82).abs() < 1e-12);
        // between (0.0, 3.30) and (0.1, 3.62) at 0.05 -> 3.46
        assert!((ocv_per_cell(&cfg, 0.05).unwrap().0 - 3.46).abs() < 1e-12);
    }

    #[test]
    fn test_pt_ocv_clamps_and_flags() {
        let cfg = ref_cfg();
        assert_eq!(ocv_per_cell(&cfg, -0.2).unwrap(), (3.30, true));
        assert_eq!(ocv_per_cell(&cfg, 1.2).unwrap(), (4.20, true));
        assert_eq!(
            ocv_per_cell(&cfg, f64::NAN).err(),
            Some(PowertrainError::InvalidInput)
        );
    }

    #[test]
    fn test_pt_sag_is_current_times_resistance() {
        // 20 A x 0.02 ohm = 0.4 V; 16.8 V open circuit -> 16.4 V
        assert!((loaded_voltage_v(16.8, 20.0, 0.02) - 16.4).abs() < 1e-12);
        assert_eq!(loaded_voltage_v(1.0, 1000.0, 1.0), 0.0); // floored, never negative
    }

    #[test]
    fn test_pt_soc_coulomb_counting() {
        // 10 A for 360 s = 1 Ah out of 5 Ah -> SOC 1.0 - 0.2 = 0.8 (3600 steps of 0.1 s)
        let cfg = ref_cfg();
        let mut state = initial_state(1.0, 20.0).unwrap();
        let mut charge = 0.0;
        for _ in 0..3600 {
            let dq = 10.0 * 0.1 / SECONDS_PER_HOUR;
            charge += dq;
            state.soc -= dq / cfg.battery_capacity_ah;
        }
        assert!((state.soc - 0.8).abs() < 1e-12 && (charge - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_pt_step_soc_matches_the_current_it_reports() {
        let cfg = ref_cfg();
        let mut state = initial_state(1.0, 20.0).unwrap();
        let t = step(&cfg, &mut state, &input(0.4)).unwrap();
        // charge = I dt / 3600; SOC = 1 - charge / 5 Ah
        let dq = t.battery_current_a * 0.01 / 3600.0;
        assert!((state.charge_used_ah - dq).abs() < 1e-15);
        assert!((state.soc - (1.0 - dq / 5.0)).abs() < 1e-15);
        assert_eq!(state.battery_current_a, t.battery_current_a);
    }

    #[test]
    fn test_pt_first_step_has_no_sag_second_step_does() {
        let cfg = ref_cfg();
        let mut state = initial_state(1.0, 20.0).unwrap();
        let t1 = step(&cfg, &mut state, &input(0.6)).unwrap();
        // step 1 supply = open circuit 4 x 4.20 = 16.8 V (previous current is 0): motor volts = 0.6 x 16.8
        assert!((t1.motor_voltage_v[0] - 0.6 * 16.8).abs() < 1e-12);
        let i1 = t1.battery_current_a;
        let t2 = step(&cfg, &mut state, &input(0.6)).unwrap();
        // step 2 supply = OCV(SOC after step 1) - I1 R; SOC barely moved, so check against the same formula
        let ocv2 = ocv_per_cell(&cfg, 1.0 - i1 * 0.01 / 3600.0 / 5.0)
            .unwrap()
            .0
            * 4.0;
        assert!((t2.motor_voltage_v[0] - 0.6 * (ocv2 - i1 * 0.02)).abs() < 1e-9);
        assert!(t2.motor_voltage_v[0] < t1.motor_voltage_v[0]);
    }

    #[test]
    fn test_pt_cutoff_boundary_and_hold_time() {
        // Settle the sag first (the loaded voltage of the very first step is lower than the settled one because step 1 has
        // no sag in its supply voltage and so draws more current), then probe the NEXT step's loaded voltage on clones.
        let mut cfg = ref_cfg();
        let mut settled = initial_state(0.9, 20.0).unwrap();
        for _ in 0..20 {
            step(&cfg, &mut settled, &input(0.5)).unwrap();
        }
        let v_next = step(&cfg, &mut settled.clone(), &input(0.5))
            .unwrap()
            .battery_voltage_v;
        // threshold exactly at the loaded voltage: the rule is strictly below -> no flag
        cfg.battery_cutoff_v_per_cell = v_next / 4.0;
        let mut at = settled;
        assert_eq!(step(&cfg, &mut at, &input(0.5)).unwrap().battery_cutoff, 0);
        // a hair above -> flagged, and the hold timer counts dt per consecutive step
        cfg.battery_cutoff_v_per_cell = v_next / 4.0 + 1e-9;
        let mut s3 = settled;
        let a = step(&cfg, &mut s3, &input(0.5)).unwrap();
        assert_eq!(a.battery_cutoff, 1);
        assert!((a.time_below_cutoff_s - 0.01).abs() < 1e-15);
        let b = step(&cfg, &mut s3, &input(0.5)).unwrap();
        assert_eq!(b.battery_cutoff, 1);
        assert!((b.time_below_cutoff_s - 0.02).abs() < 1e-15);
        // recovery above the threshold resets the timer
        cfg.battery_cutoff_v_per_cell = 3.0;
        let c = step(&cfg, &mut s3, &input(0.5)).unwrap();
        assert_eq!((c.battery_cutoff, c.time_below_cutoff_s), (0, 0.0));
    }

    #[test]
    fn test_pt_soc_never_goes_negative_and_flags_depletion() {
        let cfg = ref_cfg();
        let mut state = initial_state(1.0e-9, 20.0).unwrap();
        state.battery_current_a = 0.0;
        let t = step(&cfg, &mut state, &input(1.0)).unwrap();
        assert_eq!(state.soc, 0.0);
        assert_eq!(t.battery_soc_clamped, 1);
        assert!(t.battery_voltage_v.is_finite());
    }

    #[test]
    fn test_pt_battery_over_current_flag() {
        // full duty on four motors from a fresh 16.8 V pack draws about 4 x 33 A / 0.95 = 140 A
        let mut cfg = ref_cfg();
        cfg.battery_max_continuous_a = 200.0;
        let mut state = initial_state(1.0, 20.0).unwrap();
        let t = step(&cfg, &mut state, &input(1.0)).unwrap();
        assert!(t.battery_current_a > 100.0 && t.battery_current_a < 200.0);
        assert_eq!(t.battery_over_current, 0);
        cfg.battery_max_continuous_a = t.battery_current_a - 0.01;
        let mut s2 = initial_state(1.0, 20.0).unwrap();
        assert_eq!(
            step(&cfg, &mut s2, &input(1.0))
                .unwrap()
                .battery_over_current,
            1
        );
    }

    // ---------------------------------------------------------------- motor + propeller
    #[test]
    fn test_pt_kt_hand_value() {
        // 60 / (2 pi 700) = 0.0136419
        assert!((kt_nm_per_a(700.0) - 0.013_641_85).abs() < 1e-8);
    }

    #[test]
    fn test_pt_solve_matches_the_closed_form_quadratic() {
        let cfg = ref_cfg();
        for v in [0.5, 2.0, 5.4, 9.0, 14.8, 16.8, 22.2] {
            let p = solve_motor(&cfg, v, RHO, 0.0).unwrap();
            let n = quadratic_rps(&cfg, v);
            assert!(
                (p.rpm - 60.0 * n).abs() < 1e-6,
                "v = {v}: {} vs {}",
                p.rpm,
                60.0 * n
            );
        }
    }

    #[test]
    fn test_pt_full_throttle_reference_point() {
        // 14.8 V on the MN3508 KV700 with the 12x6E: 8981.4057 rpm, 27.353 A, 23.752 N (hand iteration in
        // tools/parts/estimates.py: n = 150 rps gives I = 27.47 A -> 8978 rpm; converged value below)
        let p = solve_motor(&ref_cfg(), 14.8, RHO, 0.0).unwrap();
        assert!((p.rpm - 8_981.405_667_8).abs() < 1e-4);
        assert!((p.current_a - 27.353_062).abs() < 1e-5);
        assert!((p.thrust_n - 23.751_814).abs() < 1e-5);
        assert!((p.torque_nm - 0.366_325_5).abs() < 1e-6);
    }

    #[test]
    fn test_pt_zero_and_negative_voltage_idle() {
        for v in [0.0, -3.0] {
            let p = solve_motor(&ref_cfg(), v, RHO, 0.0).unwrap();
            assert_eq!(
                (p.rpm, p.current_a, p.thrust_n, p.electrical_w),
                (0.0, 0.0, 0.0, 0.0)
            );
        }
    }

    #[test]
    fn test_pt_locked_rotor_below_the_no_load_drop() {
        // I0 Rm = 0.5 x 0.072 = 0.036 V. At 0.02 V the motor stays at rest and draws V / Rm = 0.27778 A.
        let p = solve_motor(&ref_cfg(), 0.02, RHO, 0.0).unwrap();
        assert_eq!(p.rpm, 0.0);
        assert!((p.current_a - 0.02 / 0.072).abs() < 1e-12);
        assert!((p.copper_w - p.electrical_w).abs() < 1e-15);
        // just above the drop it runs
        assert!(solve_motor(&ref_cfg(), 0.0361, RHO, 0.0).unwrap().rpm > 0.0);
    }

    #[test]
    fn test_pt_rpm_and_thrust_increase_with_voltage() {
        let cfg = ref_cfg();
        let mut last = solve_motor(&cfg, 1.0, RHO, 0.0).unwrap();
        for k in 2..=22 {
            let p = solve_motor(&cfg, f64::from(k), RHO, 0.0).unwrap();
            assert!(p.rpm > last.rpm && p.thrust_n > last.thrust_n && p.current_a > last.current_a);
            last = p;
        }
    }

    #[test]
    fn test_pt_density_lowers_thrust_and_current() {
        // a thinner atmosphere unloads the propeller: more rpm, less torque, less current, less thrust
        let cfg = ref_cfg();
        let sea = solve_motor(&cfg, 14.8, RHO, 0.0).unwrap();
        let high = solve_motor(&cfg, 14.8, 0.8781, 0.0).unwrap();
        assert!(
            high.rpm > sea.rpm && high.current_a < sea.current_a && high.thrust_n < sea.thrust_n
        );
    }

    #[test]
    fn test_pt_motor_energy_balance() {
        let cfg = ref_cfg();
        for v in [3.0, 8.0, 14.8, 22.2] {
            let p = solve_motor(&cfg, v, RHO, 0.0).unwrap();
            // V I = shaft + copper + no-load (exact identity of the model; the bisection residual is far below 1e-9)
            let sum = p.shaft_w + p.copper_w + p.noload_w;
            assert!(
                (p.electrical_w - sum).abs() < 1e-9 * p.electrical_w,
                "v = {v}"
            );
        }
    }

    #[test]
    fn test_pt_solver_is_deterministic_and_rejects_bad_input() {
        let cfg = ref_cfg();
        let a = solve_motor(&cfg, 14.8, RHO, 0.0).unwrap();
        let b = solve_motor(&cfg, 14.8, RHO, 0.0).unwrap();
        assert_eq!(a.rpm.to_bits(), b.rpm.to_bits());
        assert_eq!(a.current_a.to_bits(), b.current_a.to_bits());
        assert_eq!(
            solve_motor(&cfg, f64::NAN, RHO, 0.0).err(),
            Some(PowertrainError::InvalidInput)
        );
        assert_eq!(
            solve_motor(&cfg, 14.8, 0.0, 0.0).err(),
            Some(PowertrainError::InvalidInput)
        );
        assert_eq!(
            solve_motor(&cfg, 14.8, f64::NAN, 0.0).err(),
            Some(PowertrainError::InvalidInput)
        );
    }

    #[test]
    fn test_pt_bench_point_hand_values() {
        let b = bench_point(&ref_cfg(), 1.0, 14.8, RHO, 0.0).unwrap();
        // motor electrical power 14.8 x 27.353062 = 404.825 W; pack power / 0.95 = 426.132 W; pack current / 14.8 = 28.793 A
        // ESC loss = 426.132 - 404.825 = 21.307 W; efficiency = shaft 344.540 / 426.132 = 0.80853
        assert!((b.motor_electrical_w - 404.825_32).abs() < 1e-3);
        assert!((b.battery_power_w - 404.825_32 / 0.95).abs() < 1e-3);
        assert!((b.battery_current_a - 404.825_32 / 0.95 / 14.8).abs() < 1e-4);
        assert!((b.esc_loss_w - 21.306_6).abs() < 1e-3);
        assert!((b.system_efficiency - 344.540_35 / (404.825_32 / 0.95)).abs() < 1e-5);
        let idle = bench_point(&ref_cfg(), 0.0, 14.8, RHO, 0.0).unwrap();
        assert_eq!(
            (idle.rpm, idle.battery_current_a, idle.system_efficiency),
            (0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn test_pt_bench_point_rejects_bad_input() {
        let cfg = ref_cfg();
        for (d, v, r) in [
            (1.1, 14.8, RHO),
            (-0.1, 14.8, RHO),
            (f64::NAN, 14.8, RHO),
            (0.5, -1.0, RHO),
            (0.5, f64::NAN, RHO),
            (0.5, 14.8, 0.0),
        ] {
            assert_eq!(
                bench_point(&cfg, d, v, r, 0.0).err(),
                Some(PowertrainError::InvalidInput),
                "{d} {v} {r}"
            );
        }
    }

    // ---------------------------------------------------------------- step: electrical bookkeeping
    #[test]
    fn test_pt_step_power_bookkeeping() {
        let cfg = ref_cfg();
        let mut state = initial_state(0.8, 20.0).unwrap();
        let _ = step(&cfg, &mut state, &input(0.5)).unwrap();
        let t = step(&cfg, &mut state, &input(0.55)).unwrap();
        let motor_el: f64 = (0..4)
            .map(|i| t.motor_voltage_v[i] * t.motor_current_a[i])
            .sum();
        let esc_loss: f64 = t.esc_loss_w.iter().sum();
        // pack power = motor electrical + ESC loss
        assert!((t.battery_power_w - (motor_el + esc_loss)).abs() < 1e-9 * t.battery_power_w);
        // each motor: V I = shaft + copper + no-load
        for i in 0..4 {
            let shaft = t.motor_torque_nm[i] * t.motor_rpm[i] * 2.0 * PI / 60.0;
            let sum = shaft + t.motor_copper_loss_w[i] + t.motor_noload_loss_w[i];
            assert!((t.motor_voltage_v[i] * t.motor_current_a[i] - sum).abs() < 1e-9 * sum);
        }
        // the pack voltage reported is open circuit minus I R
        assert!(
            (t.battery_voltage_v - (t.battery_ocv_v - t.battery_current_a * 0.02)).abs() < 1e-12
        );
        // battery heat = I^2 R
        assert!((t.battery_heat_w - t.battery_current_a.powi(2) * 0.02).abs() < 1e-12);
    }

    #[test]
    fn test_pt_step_zero_duty_draws_nothing() {
        let cfg = ref_cfg();
        let mut state = initial_state(1.0, 20.0).unwrap();
        let t = step(&cfg, &mut state, &input(0.0)).unwrap();
        assert_eq!(
            (t.battery_current_a, t.battery_power_w, t.battery_heat_w),
            (0.0, 0.0, 0.0)
        );
        assert!(t.motor_rpm.iter().all(|&r| r == 0.0));
        assert_eq!(state.soc, 1.0);
        assert_eq!(t.step_valid, 1);
    }

    #[test]
    fn test_pt_step_unequal_duties_give_unequal_thrust() {
        let cfg = ref_cfg();
        let mut state = initial_state(1.0, 20.0).unwrap();
        let mut inp = input(0.4);
        inp.duties = [0.3, 0.4, 0.5, 0.6];
        let t = step(&cfg, &mut state, &inp).unwrap();
        assert!(t.motor_thrust_n.windows(2).all(|w| w[1] > w[0]));
    }

    #[test]
    fn test_pt_step_flags_for_current_and_temperature_limits() {
        let mut cfg = ref_cfg();
        cfg.motor_max_current_a = 20.0; // full throttle draws ~27 A on each motor
        cfg.esc_max_current_a = 20.0;
        let mut state = initial_state(1.0, 20.0).unwrap();
        let t = step(&cfg, &mut state, &input(1.0)).unwrap();
        assert_eq!(t.motor_over_current_mask, 0b1111);
        assert_eq!(t.esc_over_current_mask, 0b1111);
        let mut inp = input(1.0);
        inp.duties = [1.0, 0.0, 1.0, 0.0];
        let mut s2 = initial_state(1.0, 20.0).unwrap();
        let u = step(&cfg, &mut s2, &inp).unwrap();
        assert_eq!(u.motor_over_current_mask, 0b0101); // bit i = motor i
        let mut low = ref_cfg();
        low.motor_winding_temp_limit_c = 20.0; // any heating exceeds it after one step
        low.esc_temp_limit_c = 20.0;
        low.battery_temp_limit_c = 20.0;
        let mut s3 = initial_state(1.0, 20.0).unwrap();
        let w = step(&low, &mut s3, &input(1.0)).unwrap();
        assert_eq!(
            (
                w.motor_over_temp_mask,
                w.esc_over_temp_mask,
                w.battery_over_temp
            ),
            (0b1111, 0b1111, 1)
        );
        assert!(any_over_temp(&w));
        assert!(!any_over_temp(&PowertrainTelemetry::no_data()));
    }

    #[test]
    fn test_pt_motor_current_flag_is_strictly_above_the_rating() {
        // a rating exactly equal to the current is not exceeded; a hair below it is
        let mut cfg = ref_cfg();
        let t = step(&cfg, &mut initial_state(1.0, 20.0).unwrap(), &input(1.0)).unwrap();
        cfg.motor_max_current_a = t.motor_current_a[0];
        let at = step(&cfg, &mut initial_state(1.0, 20.0).unwrap(), &input(1.0)).unwrap();
        assert_eq!(at.motor_over_current_mask, 0);
        cfg.motor_max_current_a = t.motor_current_a[0] - 1e-9;
        let below = step(&cfg, &mut initial_state(1.0, 20.0).unwrap(), &input(1.0)).unwrap();
        assert_eq!(below.motor_over_current_mask, 0b1111);
    }

    #[test]
    fn test_pt_esc_input_current_includes_the_efficiency() {
        // First step, full duty, 16.8 V supply: ESC input current = V_m I_m / (eta x 16.8). Dropping eta would give 0.95 of that.
        let mut cfg = ref_cfg();
        let t = step(&cfg, &mut initial_state(1.0, 20.0).unwrap(), &input(1.0)).unwrap();
        let i_esc = t.motor_voltage_v[0] * t.motor_current_a[0] / (0.95 * 16.8);
        assert!((t.battery_current_a - 4.0 * i_esc).abs() < 1e-9); // the four ESC inputs add up to the pack current
        cfg.esc_max_current_a = 0.5 * (i_esc + 0.95 * i_esc);
        let flagged = step(&cfg, &mut initial_state(1.0, 20.0).unwrap(), &input(1.0)).unwrap();
        assert_eq!(flagged.esc_over_current_mask, 0b1111);
        cfg.esc_max_current_a = i_esc + 1e-6;
        let clear = step(&cfg, &mut initial_state(1.0, 20.0).unwrap(), &input(1.0)).unwrap();
        assert_eq!(clear.esc_over_current_mask, 0);
    }

    #[test]
    fn test_pt_pack_voltage_scales_with_the_cell_count() {
        for cells in [1_i32, 3, 4, 6] {
            let mut cfg = ref_cfg();
            cfg.battery_cells = cells;
            let t = step(&cfg, &mut initial_state(1.0, 20.0).unwrap(), &input(0.5)).unwrap();
            // full pack: 4.20 V per cell; first step has no sag
            assert!(
                (t.battery_ocv_v - 4.2 * f64::from(cells)).abs() < 1e-12,
                "{cells}"
            );
            assert!(
                (t.motor_voltage_v[0] - 0.5 * 4.2 * f64::from(cells)).abs() < 1e-12,
                "{cells}"
            );
        }
    }

    #[test]
    fn test_pt_json_accepts_exactly_sixteen_ocv_points() {
        let pts: Vec<String> = (0..16)
            .map(|i| format!("[{},{}]", f64::from(i) / 15.0, 3.3 + 0.06 * f64::from(i)))
            .collect();
        let json = ref_json().replace(
            "\"ocv_curve\":[[0.0,3.30],[0.1,3.62],[0.2,3.70],[0.3,3.75],[0.5,3.82],[0.7,3.92],[0.9,4.08],[1.0,4.20]]",
            &format!("\"ocv_curve\":[{}]", pts.join(",")),
        );
        let cfg = parse_powertrain_config(&json).unwrap();
        assert_eq!(cfg.ocv_points, 16);
        assert_eq!(cfg.ocv_soc[15], 1.0);
    }

    #[test]
    fn test_pt_thermal_valid_follows_the_config() {
        let mut cfg = ref_cfg();
        let mut s = initial_state(1.0, 20.0).unwrap();
        assert_eq!(step(&cfg, &mut s, &input(0.4)).unwrap().thermal_valid, 0);
        cfg.parameters_verified = 1;
        assert_eq!(step(&cfg, &mut s, &input(0.4)).unwrap().thermal_valid, 1);
    }

    // ---------------------------------------------------------------- thermal
    #[test]
    fn test_pt_cooling_factor_hand_values() {
        assert_eq!(cooling_factor(0.0, 5.0), 1.0);
        assert!((cooling_factor(5.0, 5.0) - 2.0).abs() < 1e-15); // v = v_ref halves the resistance
        assert!((cooling_factor(10.0, 5.0) - (1.0 + 2.0_f64.powf(0.8))).abs() < 1e-15); // 2.7411
        assert!((cooling_factor(10.0, 5.0) - 2.741_1).abs() < 1e-4);
        assert_eq!(cooling_factor(-3.0, 5.0), 1.0); // negative speed is treated as zero
    }

    #[test]
    fn test_pt_induced_velocity_hand_value() {
        // A = pi 0.3048^2 / 4 = 0.072966 m^2; v = sqrt(23.752 / (2 x 1.225 x 0.072966)) = sqrt(132.87) = 11.527 m/s
        let v = induced_velocity_mps(23.752, 1.225, 0.3048);
        assert!((v - 11.527).abs() < 2e-3);
        assert_eq!(induced_velocity_mps(0.0, 1.225, 0.3048), 0.0);
    }

    #[test]
    fn test_pt_single_node_steady_state_is_power_times_resistance() {
        // 20 W into R = 2 K/W, C = 10 J/K (tau = 20 s): rise -> 40 K. After 10 tau the rest is 40 e^-10 = 0.0018 K.
        let mut t = 20.0;
        for _ in 0..20_000 {
            t = euler_node(t, 20.0, 2.0, 10.0, 20.0, 0.01);
        }
        assert!((t - 20.0 - 40.0).abs() < 4.0e-3);
    }

    #[test]
    fn test_pt_adiabatic_rate_is_power_over_capacity() {
        // no cooling path (R huge): dT/dt = P / C exactly. 20 W / 10 J/K = 2 K/s -> 2.0 K after 1 s (100 steps of 0.01 s)
        let mut t = 20.0;
        for _ in 0..100 {
            t = euler_node(t, 20.0, 1.0e12, 10.0, 20.0, 0.01);
        }
        assert!((t - 22.0).abs() < 1e-9);
    }

    #[test]
    fn test_pt_motor_two_node_steady_state() {
        // P_cu = 10 W into the winding, P_nl = 5 W into the case, R_wc = 0.2, R_ca = 0.4 (still air):
        // case - ambient = (10 + 5) x 0.4 = 6 K; winding - case = 10 x 0.2 = 2 K; winding - ambient = 8 K.
        let cfg = ref_cfg();
        let (mut tw, mut tc) = (20.0_f64, 20.0_f64);
        for _ in 0..40_000 {
            let flow = (tw - tc) / cfg.motor_r_winding_case_k_per_w;
            let new_w = tw + 0.01 * (10.0 - flow) / cfg.motor_c_winding_j_per_k;
            let new_c = tc
                + 0.01 * (5.0 + flow - (tc - 20.0) / cfg.motor_r_case_ambient_k_per_w)
                    / cfg.motor_c_case_j_per_k;
            (tw, tc) = (new_w, new_c);
        }
        assert!(
            (tc - 26.0).abs() < 1e-2 && (tw - 28.0).abs() < 1e-2,
            "{tw} {tc}"
        );
    }

    #[test]
    fn test_pt_thermal_step_guard_boundary() {
        // tau_min of the reference network at still air: R_wc C_w = 0.2 x 20 = 4 s; R_ca C_c = 0.4 x 60 = 24; ESC 2 x 10 = 20;
        // battery 3 x 300 = 900 -> 4 s, so dt <= 0.2 x 4 = 0.8 s is allowed (the step input check caps dt separately).
        let cfg = ref_cfg();
        let mut s = initial_state(1.0, 20.0).unwrap();
        let mut inp = input(0.0);
        inp.dt_s = 0.8;
        assert!(step(&cfg, &mut s, &inp).is_ok());
        inp.dt_s = 0.8001;
        assert_eq!(
            step(&cfg, &mut s, &inp).err(),
            Some(PowertrainError::ThermalStep)
        );
        let mut fast = ref_cfg();
        fast.esc_c_j_per_k = 0.01; // ESC tau = 0.02 s -> dt must be <= 0.004 s
        assert_eq!(
            step(&fast, &mut s, &input(0.0)).err(),
            Some(PowertrainError::ThermalStep)
        );
    }

    #[test]
    fn test_pt_reference_hover_600_s_stays_bounded() {
        // Hover duty for the reference vehicle is ~0.37: use 0.37 for 600 s (60,000 steps of 0.01 s).
        // battery capacitance 15 J/K (tau 45 s) so that 600 s is 13 time constants for every node, and a huge pack so the
        // losses stay constant (otherwise the nodes trail the slowly falling losses by tau x dT/dt, about 0.1 K here)
        let mut cfg = ref_cfg();
        cfg.battery_c_j_per_k = 15.0;
        cfg.battery_capacity_ah = 1.0e6;
        let mut state = initial_state(1.0, 20.0).unwrap();
        let mut last = None;
        for _ in 0..60_000 {
            last = Some(step(&cfg, &mut state, &input(0.37)).unwrap());
        }
        let t = last.unwrap();
        // steady-state energy balance of each node, from the loss numbers the step itself reports:
        // winding - case = P_cu x R_wc ; ESC - ambient = P_esc x R_esc ; battery - ambient = P_bat x R_bat
        for i in 0..4 {
            assert!(
                (t.motor_winding_c[i] - t.motor_case_c[i] - t.motor_copper_loss_w[i] * 0.2).abs()
                    < 5e-3
            );
            assert!((t.esc_temp_c[i] - 20.0 - t.esc_loss_w[i] * 2.0).abs() < 5e-3);
        }
        assert!((t.battery_temp_c - 20.0 - t.battery_heat_w * 3.0).abs() < 5e-3);
        // bounded and plausible: a hover with ~1.6 W of copper loss cannot heat a motor by tens of kelvin
        assert!(
            t.motor_winding_c.iter().all(|&w| w > 20.0 && w < 30.0),
            "{:?}",
            t.motor_winding_c
        );
        assert!(t.motor_winding_c[0] < 669.0 / 20.0); // the legacy model reported about 669 C
        assert!(t.battery_soc < 1.0 && t.battery_soc > 0.999_99); // about 1.2 Ah drawn from a 1e6 Ah pack
    }

    #[test]
    fn test_pt_cooling_with_airspeed_lowers_temperature() {
        let cfg = ref_cfg();
        let run = |airspeed: f64| {
            let mut s = initial_state(1.0, 20.0).unwrap();
            let mut inp = input(0.8);
            inp.airspeed_mps = airspeed;
            let mut last = None;
            for _ in 0..30_000 {
                last = Some(step(&cfg, &mut s, &inp).unwrap());
            }
            last.unwrap()
        };
        let still = run(0.0);
        let moving = run(15.0);
        assert!(moving.motor_case_c[0] < still.motor_case_c[0]);
        assert!(moving.esc_temp_c[0] < still.esc_temp_c[0]);
        assert!(moving.battery_temp_c < still.battery_temp_c);
    }

    #[test]
    fn test_pt_ambient_offset_moves_the_steady_state() {
        let cfg = ref_cfg();
        let run = |ambient: f64| {
            let mut s = initial_state(1.0, ambient).unwrap();
            let mut inp = input(0.0);
            inp.ambient_c = ambient;
            for _ in 0..100 {
                step(&cfg, &mut s, &inp).unwrap();
            }
            s.motor_winding_c[0]
        };
        // no heat at zero duty: the nodes simply stay at the ambient they started at
        assert!((run(35.0) - 35.0).abs() < 1e-12 && (run(-5.0) + 5.0).abs() < 1e-12);
    }

    // ---------------------------------------------------------------- step safety
    #[test]
    fn test_pt_step_rejects_bad_inputs_and_leaves_state_alone() {
        let cfg = ref_cfg();
        let mut state = initial_state(0.7, 20.0).unwrap();
        let before = state;
        let cases: Vec<(&str, StepInput)> = vec![
            (
                "duty nan",
                StepInput {
                    duties: [f64::NAN, 0.0, 0.0, 0.0],
                    ..input(0.0)
                },
            ),
            (
                "duty above 1",
                StepInput {
                    duties: [0.0, 1.1, 0.0, 0.0],
                    ..input(0.0)
                },
            ),
            (
                "duty negative",
                StepInput {
                    duties: [0.0, 0.0, -0.1, 0.0],
                    ..input(0.0)
                },
            ),
            (
                "rho zero",
                StepInput {
                    rho_kgpm3: 0.0,
                    ..input(0.0)
                },
            ),
            (
                "airspeed negative",
                StepInput {
                    airspeed_mps: -1.0,
                    ..input(0.0)
                },
            ),
            (
                "ambient nan",
                StepInput {
                    ambient_c: f64::NAN,
                    ..input(0.0)
                },
            ),
            (
                "dt zero",
                StepInput {
                    dt_s: 0.0,
                    ..input(0.0)
                },
            ),
            (
                "dt nan",
                StepInput {
                    dt_s: f64::NAN,
                    ..input(0.0)
                },
            ),
        ];
        for (name, inp) in cases {
            assert_eq!(
                step(&cfg, &mut state, &inp).err(),
                Some(PowertrainError::InvalidInput),
                "{name}"
            );
            assert_eq!(state.soc.to_bits(), before.soc.to_bits(), "{name}");
        }
        let mut bad_state = before;
        bad_state.soc = f64::NAN;
        assert_eq!(
            step(&cfg, &mut bad_state, &input(0.1)).err(),
            Some(PowertrainError::InvalidState)
        );
        let mut bad_cfg = ref_cfg();
        bad_cfg.esc_efficiency = 0.0;
        assert_eq!(
            step(&bad_cfg, &mut state, &input(0.1)).err(),
            Some(PowertrainError::InvalidConfig)
        );
        assert_eq!(state.charge_used_ah, before.charge_used_ah);
    }

    #[test]
    fn test_pt_step_is_bit_identical_twice() {
        let cfg = ref_cfg();
        let run = || {
            let mut s = initial_state(0.9, 25.0).unwrap();
            let mut out = Vec::new();
            for k in 0..200 {
                let d = 0.2 + 0.003 * f64::from(k);
                let t = step(&cfg, &mut s, &input(d)).unwrap();
                out.push((
                    t.battery_current_a.to_bits(),
                    t.motor_winding_c[0].to_bits(),
                    t.motor_rpm[1].to_bits(),
                ));
            }
            (out, s.soc.to_bits(), s.battery_temp_c.to_bits())
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn test_pt_pathological_resistance_does_not_run_away() {
        // 1 ohm of pack resistance with the one-step-lag sag: the pack voltage may oscillate but stays finite and non-negative
        let mut cfg = ref_cfg();
        cfg.battery_internal_resistance_ohm = 1.0;
        let mut s = initial_state(1.0, 20.0).unwrap();
        for _ in 0..2000 {
            let t = step(&cfg, &mut s, &input(1.0)).unwrap();
            assert!(t.battery_voltage_v.is_finite() && t.battery_voltage_v >= 0.0);
            assert!(t.battery_current_a.is_finite() && t.battery_current_a >= 0.0);
        }
    }

    #[test]
    fn test_pt_normal_load_converges_to_a_steady_current() {
        // with the one-step lag the current settles: successive steps at constant duty agree to 1e-9 A
        // a huge pack keeps the SOC (and so the OCV) constant, so only the sag lag is being tested
        let mut cfg = ref_cfg();
        cfg.battery_capacity_ah = 1.0e6;
        let mut s = initial_state(1.0, 20.0).unwrap();
        let mut prev = 0.0;
        let mut cur = 0.0;
        for _ in 0..50 {
            prev = cur;
            cur = step(&cfg, &mut s, &input(0.5)).unwrap().battery_current_a;
        }
        assert!((cur - prev).abs() < 1e-9, "{prev} {cur}");
    }

    // ---------------------------------------------------------------- JSON
    pub(crate) fn ref_json() -> String {
        r#"{"motor_kv_rpm_per_v":700.0,"motor_resistance_ohm":0.072,"motor_no_load_current_a":0.5,"motor_max_current_a":23.0,
"motor_winding_temp_limit_c":120.0,"prop_diameter_m":0.3048,"prop_ct":0.100256,"prop_cp":0.031874666666666664,
"esc_efficiency":0.95,"esc_max_current_a":40.0,"esc_temp_limit_c":100.0,"battery_capacity_ah":5.0,
"battery_internal_resistance_ohm":0.02,"battery_cutoff_v_per_cell":3.3,"battery_max_continuous_a":125.0,
"battery_temp_limit_c":60.0,"motor_r_winding_case_k_per_w":0.2,"motor_r_case_ambient_k_per_w":0.4,
"motor_c_winding_j_per_k":20.0,"motor_c_case_j_per_k":60.0,"esc_r_ambient_k_per_w":2.0,"esc_c_j_per_k":10.0,
"battery_r_ambient_k_per_w":3.0,"battery_c_j_per_k":300.0,"cooling_ref_airspeed_mps":5.0,"battery_cells":4,
"parameters_verified":0,
"ocv_curve":[[0.0,3.30],[0.1,3.62],[0.2,3.70],[0.3,3.75],[0.5,3.82],[0.7,3.92],[0.9,4.08],[1.0,4.20]]}"#
            .to_string()
    }

    #[test]
    fn test_pt_json_parses_to_the_reference_config() {
        let cfg = parse_powertrain_config(&ref_json()).unwrap();
        let want = ref_cfg();
        assert_eq!(cfg.motor_kv_rpm_per_v, want.motor_kv_rpm_per_v);
        assert_eq!(cfg.ocv_points, 8);
        assert_eq!(cfg.ocv_soc[4], 0.5);
        assert_eq!(cfg.ocv_v_per_cell[7], 4.20);
        assert_eq!(cfg.ocv_soc[8], 0.0); // unused entries stay zero
        assert_eq!(cfg.battery_cells, 4);
    }

    #[test]
    fn test_pt_json_rejections() {
        assert_eq!(parse_powertrain_config("{").err(), Some(LoadError::Parse));
        assert_eq!(
            parse_powertrain_config(
                &ref_json().replace("\"battery_cells\":4", "\"battery_cells\":4,\"surprise\":1")
            )
            .err(),
            Some(LoadError::Parse)
        );
        assert_eq!(
            parse_powertrain_config(&ref_json().replace("\"motor_kv_rpm_per_v\":700.0,", "")).err(),
            Some(LoadError::Parse)
        );
        assert_eq!(
            parse_powertrain_config(
                &ref_json().replace("\"esc_efficiency\":0.95", "\"esc_efficiency\":1.5")
            )
            .err(),
            Some(LoadError::Invalid)
        );
        let many = vec!["[0.0,3.3]"; 17].join(",");
        let long = ref_json().replace("\"ocv_curve\":[[0.0,3.30],[0.1,3.62],[0.2,3.70],[0.3,3.75],[0.5,3.82],[0.7,3.92],[0.9,4.08],[1.0,4.20]]", &format!("\"ocv_curve\":[{many}]"));
        assert_eq!(parse_powertrain_config(&long).err(), Some(LoadError::Parse));
        assert_eq!(
            load_powertrain_config("/nonexistent/powertrain.json").err(),
            Some(LoadError::Io)
        );
    }

    #[test]
    fn test_pt_no_data_telemetry_is_nan_and_invalid() {
        let t = PowertrainTelemetry::no_data();
        assert!(t.battery_voltage_v.is_nan() && t.motor_rpm.iter().all(|v| v.is_nan()));
        assert_eq!(
            (
                t.step_valid,
                t.battery_cutoff,
                t.thermal_valid,
                t.motor_over_temp_mask
            ),
            (0, 0, 0, 0)
        );
    }

    // ---------------------------------------------------------------- P3b: propeller table
    type Curve<'a> = (f64, &'a [(f64, f64, f64)]);

    /// Fills the table fields of a config from `(rpm, [(J, Ct, Cp), ...])` curves and switches `prop_model` to 1.
    pub(crate) fn with_table(mut cfg: PowertrainConfig, curves: &[Curve<'_>]) -> PowertrainConfig {
        cfg.prop_model = 1;
        cfg.prop_curve_count = i32::try_from(curves.len()).unwrap();
        for (k, (rpm, points)) in curves.iter().enumerate() {
            cfg.prop_curve_rpm[k] = *rpm;
            cfg.prop_curve_points[k] = i32::try_from(points.len()).unwrap();
            for (m, (j, ct, cp)) in points.iter().enumerate() {
                cfg.prop_j[k][m] = *j;
                cfg.prop_ct_table[k][m] = *ct;
                cfg.prop_cp_table[k][m] = *cp;
            }
        }
        cfg
    }

    const CURVE_A: &[(f64, f64, f64)] =
        &[(0.0, 0.10, 0.040), (0.2, 0.08, 0.035), (0.6, 0.00, 0.020)];
    const CURVE_B: &[(f64, f64, f64)] =
        &[(0.0, 0.12, 0.044), (0.3, 0.09, 0.038), (0.5, 0.04, 0.026)];

    /// Two curves (2000 and 4000 rpm) with the hand values used in the tests below.
    pub(crate) fn hand_cfg() -> PowertrainConfig {
        with_table(ref_cfg(), &[(2000.0, CURVE_A), (4000.0, CURVE_B)])
    }

    /// J-dependent table that covers the whole operating range of the reference motor (rpm-independent, so it has two identical curves).
    pub(crate) fn wide_cfg() -> PowertrainConfig {
        const WIDE: &[(f64, f64, f64)] =
            &[(0.0, 0.10, 0.032), (0.3, 0.07, 0.030), (0.8, -0.01, 0.012)];
        with_table(ref_cfg(), &[(1000.0, WIDE), (9000.0, WIDE)])
    }

    /// Constant Ct/Cp (equal to the static fit) on a table that spans 3040 .. 6018.5 rpm and J up to 0.65.
    pub(crate) fn range_cfg() -> PowertrainConfig {
        let c = ref_cfg();
        let flat: &[(f64, f64, f64)] = &[(0.0, c.prop_ct, c.prop_cp), (0.65, c.prop_ct, c.prop_cp)];
        with_table(c, &[(3040.0, flat), (6018.5, flat)])
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-12
    }

    #[test]
    fn test_pt_table_nodes_are_exact() {
        let cfg = hand_cfg();
        let p = coefficients(&cfg, 0.2, 2000.0).unwrap();
        assert!(close(p.ct, 0.08) && close(p.cp, 0.035) && !p.extrapolated);
        let q = coefficients(&cfg, 0.0, 4000.0).unwrap();
        assert!(close(q.ct, 0.12) && close(q.cp, 0.044) && !q.extrapolated);
    }

    #[test]
    fn test_pt_table_j_interpolation_hand_values() {
        let cfg = hand_cfg();
        // curve A at J = 0.1: halfway between (0, 0.10) and (0.2, 0.08) -> Ct 0.09; Cp (0.040 + 0.035) / 2 = 0.0375
        let a = coefficients(&cfg, 0.1, 2000.0).unwrap();
        assert!(close(a.ct, 0.09) && close(a.cp, 0.0375));
        // curve A at J = 0.4: halfway between (0.2, 0.08) and (0.6, 0.0) -> Ct 0.04; Cp (0.035 + 0.020) / 2 = 0.0275
        let b = coefficients(&cfg, 0.4, 2000.0).unwrap();
        assert!(close(b.ct, 0.04) && close(b.cp, 0.0275) && !b.extrapolated);
    }

    #[test]
    fn test_pt_table_rpm_interpolation_hand_values() {
        let cfg = hand_cfg();
        // J = 0.1, rpm 3000 (halfway): curve A 0.09 / 0.0375; curve B at J = 0.1 (a third of the way to 0.3): Ct 0.12 - 0.03 / 3 = 0.11,
        // Cp 0.044 - 0.006 / 3 = 0.042. Mean: Ct 0.10, Cp 0.03975.
        let p = coefficients(&cfg, 0.1, 3000.0).unwrap();
        assert!(close(p.ct, 0.10) && close(p.cp, 0.03975) && !p.extrapolated);
        // weight 0.25 at rpm 2500: Ct = 0.09 + 0.25 (0.11 - 0.09) = 0.095
        assert!(close(coefficients(&cfg, 0.1, 2500.0).unwrap().ct, 0.095));
    }

    #[test]
    fn test_pt_table_planar_data_is_reproduced_exactly() {
        // Ct = 0.2 - 0.1 J + 1e-5 rpm and Cp = 0.05 - 0.02 J + 2e-6 rpm are planes: bilinear interpolation reproduces them anywhere inside.
        let pts = |rpm: f64| -> Vec<(f64, f64, f64)> {
            [0.0, 0.2, 0.7]
                .iter()
                .map(|&j| (j, 0.2 - 0.1 * j + 1e-5 * rpm, 0.05 - 0.02 * j + 2e-6 * rpm))
                .collect()
        };
        let (p1, p2, p3) = (pts(1000.0), pts(3000.0), pts(8000.0));
        let cfg = with_table(ref_cfg(), &[(1000.0, &p1), (3000.0, &p2), (8000.0, &p3)]);
        for (j, rpm) in [
            (0.05, 1500.0),
            (0.45, 5200.0),
            (0.6, 7900.0),
            (0.0, 1000.0),
            (0.7, 8000.0),
        ] {
            let p = coefficients(&cfg, j, rpm).unwrap();
            assert!(close(p.ct, 0.2 - 0.1 * j + 1e-5 * rpm), "{j} {rpm}");
            assert!(close(p.cp, 0.05 - 0.02 * j + 2e-6 * rpm), "{j} {rpm}");
            assert!(!p.extrapolated);
        }
    }

    #[test]
    fn test_pt_table_extrapolation_flag_truth_table() {
        let cfg = hand_cfg(); // curve A: J_end 0.6; curve B: J_end 0.5
        let flag = |j: f64, rpm: f64| coefficients(&cfg, j, rpm).unwrap().extrapolated;
        assert!(!flag(0.0, 2000.0));
        assert!(flag(-1e-12, 2000.0)); // descent
        assert!(!flag(0.5, 3000.0)); // exactly the end of the shorter bracketing curve
        assert!(flag(0.5 + 1e-12, 3000.0)); // beyond curve B although curve A still has data
        assert!(flag(0.55, 3000.0));
        // at rpm 2000 the shorter upper curve (J_end 0.5) still brackets, so 0.5 is the limit there too
        assert!(!flag(0.5, 2000.0) && flag(0.5 + 1e-12, 2000.0) && flag(0.55, 2000.0));
        // with the longer curve on top the limit is the shorter curve's end of the lower one: swap the roles
        let swapped = with_table(ref_cfg(), &[(2000.0, CURVE_B), (4000.0, CURVE_A)]);
        let flag2 = |j: f64, rpm: f64| coefficients(&swapped, j, rpm).unwrap().extrapolated;
        assert!(
            !flag2(0.5, 3000.0)
                && flag2(0.5 + 1e-12, 3000.0)
                && !flag2(0.5, 2000.0)
                && flag2(0.55, 4000.0)
        );
        assert!(flag(0.1, 1999.999) && !flag(0.1, 2000.0));
        assert!(!flag(0.1, 4000.0) && flag(0.1, 4000.001));
        // at rpm exactly equal to the lower curve the upper curve still counts (conservative): J = 0.55 is beyond curve B
        assert!(flag(0.55, 2000.0 + 1e-9));
    }

    #[test]
    fn test_pt_table_out_of_range_holds_the_edge_value() {
        let cfg = hand_cfg();
        // J above the end: hold the last point; rpm 3000 mixes A at 0.55 (Ct 0.01) and B held at its end (Ct 0.04): 0.025
        let p = coefficients(&cfg, 0.55, 3000.0).unwrap();
        assert!(close(p.ct, 0.025) && p.extrapolated);
        // J far beyond everything: both curves held at their last points, rpm 3000 -> Ct (0.0 + 0.04) / 2
        assert!(close(coefficients(&cfg, 50.0, 3000.0).unwrap().ct, 0.02));
        // descent holds the J = 0 point
        assert!(close(coefficients(&cfg, -5.0, 2000.0).unwrap().ct, 0.10));
        // rpm outside holds the edge curve
        assert!(close(coefficients(&cfg, 0.1, 100.0).unwrap().ct, 0.09));
        assert!(close(coefficients(&cfg, 0.1, 9000.0).unwrap().ct, 0.11));
        // infinities are legal inputs of the lookup (rotor at rest with inflow)
        assert!(
            coefficients(&cfg, f64::INFINITY, 2000.0)
                .unwrap()
                .extrapolated
        );
        assert!(
            coefficients(&cfg, f64::NEG_INFINITY, 2000.0)
                .unwrap()
                .extrapolated
        );
    }

    #[test]
    fn test_pt_table_single_curve_and_static_model() {
        let cfg = with_table(ref_cfg(), &[(2000.0, CURVE_A)]);
        assert!(close(coefficients(&cfg, 0.1, 2000.0).unwrap().ct, 0.09));
        assert!(coefficients(&cfg, 0.1, 2000.5).unwrap().extrapolated);
        // static model: the constants, never flagged, whatever (J, rpm)
        let st = coefficients(&ref_cfg(), 9.0, 99_999.0).unwrap();
        assert_eq!((st.ct, st.cp, st.extrapolated), (0.100_256, CP, false));
        assert_eq!(
            coefficients(&hand_cfg(), f64::NAN, 2000.0).err(),
            Some(PowertrainError::InvalidInput)
        );
        assert_eq!(
            coefficients(&hand_cfg(), 0.1, f64::NAN).err(),
            Some(PowertrainError::InvalidInput)
        );
    }

    #[test]
    fn test_pt_table_validation_rules() {
        assert_eq!(validate_powertrain_config(&hand_cfg()), Ok(()));
        let bad: Vec<CfgCase> = vec![
            ("model 2", Box::new(|c| c.prop_model = 2)),
            (
                "model 1 with no curves",
                Box::new(|c| c.prop_curve_count = 0),
            ),
            ("nine curves", Box::new(|c| c.prop_curve_count = 9)),
            ("negative count", Box::new(|c| c.prop_curve_count = -1)),
            (
                "rpm not increasing",
                Box::new(|c| c.prop_curve_rpm[1] = 2000.0),
            ),
            (
                "rpm zero",
                Box::new(|c| {
                    c.prop_curve_rpm[0] = 0.0;
                }),
            ),
            ("one point", Box::new(|c| c.prop_curve_points[0] = 1)),
            ("49 points", Box::new(|c| c.prop_curve_points[0] = 49)),
            ("J does not start at 0", Box::new(|c| c.prop_j[1][0] = 0.01)),
            ("J not increasing", Box::new(|c| c.prop_j[0][2] = 0.2)),
            ("Cp zero", Box::new(|c| c.prop_cp_table[1][1] = 0.0)),
            ("Cp negative", Box::new(|c| c.prop_cp_table[0][2] = -0.01)),
            ("Ct nan", Box::new(|c| c.prop_ct_table[0][0] = f64::NAN)),
            ("unused entry nan", Box::new(|c| c.prop_j[7][47] = f64::NAN)),
            (
                "rpm of unused curve nan",
                Box::new(|c| c.prop_curve_rpm[5] = f64::NAN),
            ),
        ];
        for (name, mutate) in bad {
            let mut cfg = hand_cfg();
            mutate(&mut cfg);
            assert_eq!(
                validate_powertrain_config(&cfg),
                Err(PowertrainError::InvalidConfig),
                "{name}"
            );
        }
        // model 0 must not carry curves
        let mut cfg = ref_cfg();
        cfg.prop_curve_count = 1;
        assert_eq!(
            validate_powertrain_config(&cfg),
            Err(PowertrainError::InvalidConfig)
        );
        // negative Ct is allowed (windmill), 8 curves and 48 points are allowed
        let mut cfg = hand_cfg();
        cfg.prop_ct_table[0][2] = -0.05;
        assert_eq!(validate_powertrain_config(&cfg), Ok(()));
        let long: Vec<(f64, f64, f64)> =
            (0..48).map(|i| (f64::from(i) * 0.01, 0.1, 0.03)).collect();
        let curves: Vec<Curve<'_>> = (0..8)
            .map(|k| (1000.0 * f64::from(k + 1), long.as_slice()))
            .collect();
        assert_eq!(
            validate_powertrain_config(&with_table(ref_cfg(), &curves)),
            Ok(())
        );
    }

    #[test]
    fn test_pt_flag_brackets_use_the_curve_at_the_node_and_the_next_one() {
        // curves at 2000 (J_end 0.6), 4000 (J_end 0.5), 8000 (J_end 0.3). At exactly 4000 rpm the bracket is (4000, 8000):
        // J = 0.4 is beyond the 8000 curve, so it is flagged. A hair below 4000 the bracket is (2000, 4000) and 0.4 is inside both.
        const CURVE_C: &[(f64, f64, f64)] = &[(0.0, 0.14, 0.050), (0.3, 0.10, 0.040)];
        let cfg = with_table(
            ref_cfg(),
            &[(2000.0, CURVE_A), (4000.0, CURVE_B), (8000.0, CURVE_C)],
        );
        let flag = |j: f64, rpm: f64| coefficients(&cfg, j, rpm).unwrap().extrapolated;
        assert!(flag(0.4, 4000.0));
        assert!(!flag(0.4, 3999.999));
        // the value at the node itself is the node curve's (weight 0 on the upper curve)
        assert!(close(coefficients(&cfg, 0.0, 4000.0).unwrap().ct, 0.12));
        // at the first node the bracket is (2000, 4000)
        assert!(flag(0.55, 2000.0) && !flag(0.5, 2000.0));
    }

    /// Ct = 0.10 + 1e-6 rpm - 0.05 J and Cp = 0.025 + 4e-6 rpm - 0.01 J as a (planar) table over 1000 .. 9000 rpm and J 0 .. 1.
    fn rpm_dependent_cfg() -> PowertrainConfig {
        let pts = |rpm: f64| -> Vec<(f64, f64, f64)> {
            [0.0, 1.0]
                .iter()
                .map(|&j| {
                    (
                        j,
                        0.10 + 1e-6 * rpm - 0.05 * j,
                        0.025 + 4e-6 * rpm - 0.01 * j,
                    )
                })
                .collect()
        };
        let (lo, hi) = (pts(1000.0), pts(9000.0));
        with_table(ref_cfg(), &[(1000.0, &lo), (9000.0, &hi)])
    }

    #[test]
    fn test_pt_solver_follows_the_rpm_dependence_of_the_table() {
        // Independent damped fixed point with the planar Cp(J, rpm) written out; the table depends on rpm, so passing the wrong
        // speed (rev/s instead of rpm) to the lookup would move the answer by several rpm.
        let cfg = rpm_dependent_cfg();
        let kt = 60.0 / (2.0 * PI * 700.0);
        for (v, inflow) in [(14.8, 0.0), (10.0, 6.0)] {
            let mut n = 80.0;
            for _ in 0..8000 {
                let j = inflow / (n * 0.3048);
                let cp = 0.025 + 4e-6 * (60.0 * n) - 0.01 * j;
                let q = cp * RHO * 0.3048_f64.powi(5) * n * n / (2.0 * PI);
                n = 0.5 * n + 0.5 * 700.0 * (v - 0.072 * (q / kt + 0.5)) / 60.0;
            }
            let p = solve_motor(&cfg, v, RHO, inflow).unwrap();
            assert!(
                (p.rpm - 60.0 * n).abs() < 1e-6,
                "{v} {inflow}: {} vs {}",
                p.rpm,
                60.0 * n
            );
            let ct = 0.10 + 1e-6 * (60.0 * n) - 0.05 * (inflow / (n * 0.3048));
            assert!((p.thrust_n - ct * RHO * n * n * 0.3048_f64.powi(4)).abs() < 1e-6);
        }
    }

    #[test]
    fn test_pt_solver_rejects_infinite_inflow_itself() {
        for inflow in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            assert_eq!(
                solve_motor(&wide_cfg(), 14.8, RHO, inflow).err(),
                Some(PowertrainError::InvalidInput)
            );
        }
    }

    #[test]
    fn test_pt_state_rejects_a_bad_extrapolated_time() {
        let mut s = initial_state(1.0, 20.0).unwrap();
        for bad in [-0.001, f64::NAN, f64::INFINITY] {
            s.prop_extrapolated_time_s = bad;
            assert_eq!(
                validate_state(&s),
                Err(PowertrainError::InvalidState),
                "{bad}"
            );
        }
        s.prop_extrapolated_time_s = 12.5;
        assert_eq!(validate_state(&s), Ok(()));
    }

    #[test]
    fn test_pt_advance_ratio() {
        // J = V / (n D): 5 m/s, 100 rev/s, D 0.3048 -> 0.164042
        assert!((advance_ratio(5.0, 100.0, 0.3048) - 5.0 / 30.48).abs() < 1e-15);
        assert!((advance_ratio(-5.0, 100.0, 0.3048) + 5.0 / 30.48).abs() < 1e-15);
        assert_eq!(advance_ratio(0.0, 100.0, 0.3048), 0.0);
        // at rest: infinite in the inflow direction, 0 without inflow
        assert_eq!(advance_ratio(1.0, 0.0, 0.3048), f64::INFINITY);
        assert_eq!(advance_ratio(-1.0, 0.0, 0.3048), f64::NEG_INFINITY);
        assert_eq!(advance_ratio(0.0, 0.0, 0.3048), 0.0);
    }

    #[test]
    fn test_pt_axial_inflow_geometry_hand_values() {
        let level = [0.0_f32, 0.0, 0.0, 1.0];
        // level: thrust points up (NED -Z); climbing 5 m/s is v_z = -5 -> +5; descending -> -5; hover -> 0
        assert!((axial_inflow_mps([0.0, 0.0, -5.0], level) - 5.0).abs() < 1e-12);
        assert!((axial_inflow_mps([0.0, 0.0, 5.0], level) + 5.0).abs() < 1e-12);
        assert_eq!(axial_inflow_mps([0.0, 0.0, 0.0], level), 0.0);
        assert!(axial_inflow_mps([8.0, 3.0, 0.0], level).abs() < 1e-12); // horizontal motion with a level rotor: no axial flow
        // roll +90 deg (q = (sin45, 0, 0, cos45)): thrust points along +Y (east): sideways motion counts, vertical motion does not
        let roll = [
            std::f32::consts::FRAC_1_SQRT_2,
            0.0,
            0.0,
            std::f32::consts::FRAC_1_SQRT_2,
        ];
        assert!((axial_inflow_mps([0.0, 3.0, 0.0], roll) - 3.0).abs() < 1e-6);
        assert!(axial_inflow_mps([0.0, 0.0, -5.0], roll).abs() < 1e-6);
        // pitch -30 deg (nose down): q = (0, -sin15, 0, cos15); thrust direction (sin30, 0, -cos30) = (0.5, 0, -0.866025)
        let pitch = [0.0_f32, -0.258_819_04, 0.0, 0.965_925_8];
        assert!((axial_inflow_mps([10.0, 0.0, 0.0], pitch) - 5.0).abs() < 1e-5); // forward flight at 10 m/s -> 5 m/s axial
        assert!((axial_inflow_mps([0.0, 0.0, -10.0], pitch) - 8.660_254).abs() < 1e-5); // climb 10 m/s -> 8.66 m/s axial
    }

    #[test]
    fn test_pt_flat_table_reproduces_the_static_solve_bit_for_bit() {
        // regression guard: a table with constant Ct/Cp equal to the static fit must give the P3a answer, with or without inflow
        let st = ref_cfg();
        let flat: &[(f64, f64, f64)] =
            &[(0.0, st.prop_ct, st.prop_cp), (0.7, st.prop_ct, st.prop_cp)];
        let tab = with_table(ref_cfg(), &[(500.0, flat), (9500.0, flat)]);
        for v in [3.0, 9.0, 14.8, 22.2] {
            for inflow in [0.0, 7.0, -4.0] {
                let a = solve_motor(&st, v, RHO, inflow).unwrap();
                let b = solve_motor(&tab, v, RHO, inflow).unwrap();
                assert_eq!(a.rpm.to_bits(), b.rpm.to_bits(), "{v} {inflow}");
                assert_eq!(a.thrust_n.to_bits(), b.thrust_n.to_bits(), "{v} {inflow}");
                assert_eq!(a.current_a.to_bits(), b.current_a.to_bits(), "{v} {inflow}");
            }
        }
    }

    /// Independent evaluation of `wide_cfg`'s table (the same J curve for any rpm), written without `coefficients`.
    fn wide_table(j: f64) -> (f64, f64) {
        let pts = [(0.0, 0.10, 0.032), (0.3, 0.07, 0.030), (0.8, -0.01, 0.012)];
        let jc = j.clamp(0.0, 0.8);
        let (a, b) = if jc <= 0.3 {
            (pts[0], pts[1])
        } else {
            (pts[1], pts[2])
        };
        let w = (jc - a.0) / (b.0 - a.0);
        (a.1 + w * (b.1 - a.1), a.2 + w * (b.2 - a.2))
    }

    #[test]
    fn test_pt_solver_with_inflow_matches_an_independent_fixed_point() {
        let cfg = wide_cfg();
        let (v, inflow) = (14.8, 8.0);
        // damped fixed point on n: n <- 0.5 n + 0.5 (Kv (V - Rm I(n)) / 60) with I(n) = Q(n) / Kt + I0, Q from the independent table
        let kt = 60.0 / (2.0 * PI * 700.0);
        let mut n = 80.0;
        for _ in 0..5000 {
            let (_, cp) = wide_table(inflow / (n * 0.3048));
            let q = cp * RHO * 0.3048_f64.powi(5) * n * n / (2.0 * PI);
            let i = q / kt + 0.5;
            n = 0.5 * n + 0.5 * 700.0 * (v - 0.072 * i) / 60.0;
        }
        let p = solve_motor(&cfg, v, RHO, inflow).unwrap();
        assert!((p.rpm - 60.0 * n).abs() < 1e-6, "{} {}", p.rpm, 60.0 * n);
        let (ct, _) = wide_table(inflow / (n * 0.3048));
        assert!((p.thrust_n - ct * RHO * n * n * 0.3048_f64.powi(4)).abs() < 1e-6);
        assert!((p.advance_ratio - inflow / (n * 0.3048)).abs() < 1e-9);
    }

    #[test]
    fn test_pt_inflow_unloads_the_propeller() {
        // forward flight / climb: thrust and current fall, rpm rises (the J-dependent table)
        let cfg = wide_cfg();
        let mut last = solve_motor(&cfg, 14.8, RHO, 0.0).unwrap();
        for inflow in [4.0, 8.0, 12.0, 16.0] {
            let p = solve_motor(&cfg, 14.8, RHO, inflow).unwrap();
            assert!(
                p.thrust_n < last.thrust_n && p.current_a < last.current_a && p.rpm > last.rpm,
                "{inflow}"
            );
            last = p;
        }
    }

    #[test]
    fn test_pt_windmill_region_has_negative_thrust_and_positive_torque() {
        // 3 V on the motor spins the propeller at about 20 rev/s; 25 m/s of inflow makes J far beyond the table (held at J = 0.8: Ct -0.01)
        let p = solve_motor(&wide_cfg(), 3.0, RHO, 25.0).unwrap();
        assert!(p.thrust_n < 0.0, "{}", p.thrust_n);
        assert!(p.torque_nm > 0.0 && p.current_a > 0.0);
        assert!(p.extrapolated);
    }

    #[test]
    fn test_pt_table_solve_energy_balance_and_determinism() {
        let cfg = wide_cfg();
        for (v, inflow) in [(5.0, 0.0), (14.8, 6.0), (22.2, -3.0)] {
            let p = solve_motor(&cfg, v, RHO, inflow).unwrap();
            let sum = p.shaft_w + p.copper_w + p.noload_w;
            assert!(
                (p.electrical_w - sum).abs() < 1e-9 * p.electrical_w,
                "{v} {inflow}"
            );
            let again = solve_motor(&cfg, v, RHO, inflow).unwrap();
            assert_eq!(p.rpm.to_bits(), again.rpm.to_bits());
        }
        // a rotor that barely turns with inflow stays finite
        let slow = solve_motor(&cfg, 0.0361, RHO, 30.0).unwrap();
        assert!(slow.rpm.is_finite() && slow.thrust_n.is_finite());
        assert_eq!(
            solve_motor(&cfg, 14.8, RHO, f64::NAN).err(),
            Some(PowertrainError::InvalidInput)
        );
    }

    #[test]
    fn test_pt_torque_balance_is_monotonic_on_the_real_shaped_table() {
        // the bisection argument needs Q(n) = Cp(J(n)) n^2 ... to grow with n: check it on a dense n grid for the wide table
        let cfg = wide_cfg();
        let mut last = 0.0;
        for k in 1..400 {
            let n = f64::from(k) * 0.5;
            let cp = coefficients(&cfg, advance_ratio(8.0, n, 0.3048), 60.0 * n)
                .unwrap()
                .cp;
            let q = cp * n * n;
            assert!(q > last, "{n}");
            last = q;
        }
    }

    #[test]
    fn test_pt_bench_point_reports_the_coefficients_and_flag() {
        let cfg = range_cfg();
        let b = bench_point(&cfg, 0.37, 14.8, RHO, 0.0).unwrap();
        assert!((b.ct - cfg.prop_ct).abs() < 1e-15 && (b.cp - cfg.prop_cp).abs() < 1e-15);
        assert_eq!(b.advance_ratio, 0.0);
        assert_eq!(b.extrapolated, 0); // about 3500 rpm, inside 3040 .. 6018.5
        let idle = bench_point(&cfg, 0.1, 14.8, RHO, 0.0).unwrap();
        assert_eq!(idle.extrapolated, 1); // about 700 rpm, below the first curve
        let fast = bench_point(&cfg, 1.0, 14.8, RHO, 0.0).unwrap(); // about 9000 rpm
        assert_eq!(fast.extrapolated, 1);
    }

    #[test]
    fn test_pt_step_flags_and_accumulates_time_outside_the_data() {
        let cfg = range_cfg();
        let mut state = initial_state(1.0, 20.0).unwrap();
        // hover duty: inside the data, no flag, no time
        let hover = step(&cfg, &mut state, &input(0.37)).unwrap();
        assert_eq!(
            (hover.prop_extrapolated, state.prop_extrapolated_time_s),
            (0, 0.0)
        );
        // idle duty: all four below 3040 rpm -> bits 0..3 and dt per step
        for k in 1..=5 {
            let t = step(&cfg, &mut state, &input(0.1)).unwrap();
            assert_eq!(t.prop_extrapolated, 0b1111);
            assert!(
                (state.prop_extrapolated_time_s - 0.01 * f64::from(k)).abs() < 1e-12,
                "{k}"
            );
        }
        // back inside: time stops growing
        let before = state.prop_extrapolated_time_s;
        step(&cfg, &mut state, &input(0.37)).unwrap();
        assert_eq!(state.prop_extrapolated_time_s, before);
        // a stopped motor is not "outside the data"
        let mut rest = initial_state(1.0, 20.0).unwrap();
        let t = step(&cfg, &mut rest, &input(0.0)).unwrap();
        assert_eq!(
            (t.prop_extrapolated, rest.prop_extrapolated_time_s),
            (0, 0.0)
        );
        // one motor idle, three in hover: only its bit
        let mut mixed = initial_state(1.0, 20.0).unwrap();
        let mut inp = input(0.37);
        inp.duties = [0.37, 0.1, 0.37, 0.37];
        assert_eq!(
            step(&cfg, &mut mixed, &inp).unwrap().prop_extrapolated,
            0b0010
        );
    }

    #[test]
    fn test_pt_step_inflow_changes_the_thrust_and_reports_j() {
        let cfg = wide_cfg();
        let run = |inflow: f64| {
            let mut s = initial_state(1.0, 20.0).unwrap();
            let mut inp = input(0.5);
            inp.axial_inflow_mps = inflow;
            step(&cfg, &mut s, &inp).unwrap()
        };
        let still = run(0.0);
        let climb = run(8.0);
        assert!(climb.motor_thrust_n[0] < still.motor_thrust_n[0]);
        assert_eq!(still.motor_advance_ratio, [0.0; 4]);
        // J = V / (n D) with the rpm the step reports
        let n = climb.motor_rpm[0] / 60.0;
        assert!((climb.motor_advance_ratio[0] - 8.0 / (n * 0.3048)).abs() < 1e-9);
        assert!(close(
            climb.motor_ct[0],
            wide_table(climb.motor_advance_ratio[0]).0
        ));
        assert!(close(
            climb.motor_cp[0],
            wide_table(climb.motor_advance_ratio[0]).1
        ));
        // a non-finite inflow is rejected before anything is computed
        let mut s = initial_state(1.0, 20.0).unwrap();
        let mut inp = input(0.5);
        inp.axial_inflow_mps = f64::INFINITY;
        let before = s;
        assert_eq!(
            step(&cfg, &mut s, &inp).err(),
            Some(PowertrainError::InvalidInput)
        );
        // a rejected step leaves the state untouched
        assert_eq!(s.soc.to_bits(), before.soc.to_bits());
        assert_eq!(
            s.motor_winding_c.map(f64::to_bits),
            before.motor_winding_c.map(f64::to_bits)
        );
        assert_eq!(
            s.prop_extrapolated_time_s.to_bits(),
            before.prop_extrapolated_time_s.to_bits()
        );
    }

    #[test]
    fn test_pt_json_accepts_eight_curves_of_48_points() {
        let points: Vec<String> = (0..48)
            .map(|i| format!("[{},0.1,0.03]", f64::from(i) * 0.01))
            .collect();
        let curves: Vec<String> = (1..=8)
            .map(|k| format!(r#"{{"rpm":{},"points":[{}]}}"#, 1000 * k, points.join(",")))
            .collect();
        let json = ref_json().trim_end().trim_end_matches('}').to_string()
            + &format!(r#","prop_curves":[{}]}}"#, curves.join(","));
        let cfg = parse_powertrain_config(&json).unwrap();
        assert_eq!(
            (
                cfg.prop_curve_count,
                cfg.prop_curve_points[7],
                cfg.prop_j[7][47]
            ),
            (8, 48, 0.47)
        );
        // 49 points in one curve: the file does not fit the fixed table
        let long: Vec<String> = (0..49)
            .map(|i| format!("[{},0.1,0.03]", f64::from(i) * 0.01))
            .collect();
        let too_long = ref_json().trim_end().trim_end_matches('}').to_string()
            + &format!(
                r#","prop_curves":[{{"rpm":1000,"points":[{}]}}]}}"#,
                long.join(",")
            );
        assert_eq!(
            parse_powertrain_config(&too_long).err(),
            Some(LoadError::Parse)
        );
    }

    #[test]
    fn test_pt_json_table_round_trip() {
        let curves = r#","prop_curves":[{"rpm":2000.0,"points":[[0.0,0.10,0.040],[0.2,0.08,0.035],[0.6,0.00,0.020]]},{"rpm":4000.0,"points":[[0.0,0.12,0.044],[0.3,0.09,0.038],[0.5,0.04,0.026]]}]}"#;
        let json = ref_json().trim_end().trim_end_matches('}').to_string() + curves;
        let cfg = parse_powertrain_config(&json).unwrap();
        let want = hand_cfg();
        assert_eq!((cfg.prop_model, cfg.prop_curve_count), (1, 2));
        assert_eq!(cfg.prop_curve_rpm, want.prop_curve_rpm);
        assert_eq!(cfg.prop_j, want.prop_j);
        assert_eq!(cfg.prop_ct_table, want.prop_ct_table);
        assert_eq!(cfg.prop_cp_table, want.prop_cp_table);
        assert_eq!(cfg.prop_curve_points[..2], [3, 3]);
        // without the key the model is static
        let st = parse_powertrain_config(&ref_json()).unwrap();
        assert_eq!((st.prop_model, st.prop_curve_count), (0, 0));
        // limits and errors
        let nine = (0..9)
            .map(|k| {
                format!(
                    r#"{{"rpm":{},"points":[[0.0,0.1,0.03],[0.5,0.05,0.02]]}}"#,
                    1000 * (k + 1)
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let too_many = ref_json().trim_end().trim_end_matches('}').to_string()
            + &format!(r#","prop_curves":[{nine}]}}"#);
        assert_eq!(
            parse_powertrain_config(&too_many).err(),
            Some(LoadError::Parse)
        );
        let bad_cp = json.replace("[0.2,0.08,0.035]", "[0.2,0.08,0.0]");
        assert_eq!(
            parse_powertrain_config(&bad_cp).err(),
            Some(LoadError::Invalid)
        );
        let extra_key = json.replace(r#""rpm":2000.0,"#, r#""rpm":2000.0,"note":"x","#);
        assert_eq!(
            parse_powertrain_config(&extra_key).err(),
            Some(LoadError::Parse)
        );
    }
}
