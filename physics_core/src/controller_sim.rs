// Tier: F
//! Closed-loop harness (test support, not on the flight path): the controller flies the real core (`ffi_step_powertrain`, analytic
//! drag, powertrain model) from a golden build directory. Used by the closed-loop tests and `examples/controller_tuning.rs`.
//! Allocates (sample vectors), so it is never part of a step loop.

use crate::config::load_vehicle_config;
use crate::controller::{
    ControllerConfig, ControllerState, Estimate, Output, Setpoint, Tuning, load_controller_config,
    step,
};
use crate::controller::{StepMetrics, step_metrics};
use crate::powertrain::{initial_state, load_powertrain_config};
use crate::types::{
    DebugTelemetry, DroneState, EnvironmentInput, FaultLatch, PowertrainConfig, PowertrainState,
    PowertrainTelemetry, VehicleConfig, WindInput,
};
use crate::{ffi_create_default_drone_state, ffi_step_powertrain};
use glam::{DQuat, DVec3};
use std::path::{Path, PathBuf};
use std::ptr;

/// Fixed step of every closed-loop run, s.
pub const DT_S: f64 = 0.002;

/// The three configs of a golden build directory.
#[derive(Copy, Clone)]
pub struct Build {
    /// `vehicle_config.json`.
    pub vehicle: VehicleConfig,
    /// `powertrain_config.json`.
    pub powertrain: PowertrainConfig,
    /// `controller_config.json`.
    pub controller: ControllerConfig,
}

/// `tools/parts/builds/<name>` next to the crate.
#[must_use]
pub fn golden_dir(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../tools/parts/builds")
        .join(name)
}

/// Loads the three configs of a build directory.
///
/// # Errors
/// A text naming the file that failed.
pub fn load_build(dir: &Path) -> Result<Build, String> {
    let p = |f: &str| dir.join(f).to_string_lossy().into_owned();
    Ok(Build {
        vehicle: load_vehicle_config(&p("vehicle_config.json"))
            .map_err(|e| format!("vehicle_config: {e:?}"))?,
        powertrain: load_powertrain_config(&p("powertrain_config.json"))
            .map_err(|e| format!("powertrain_config: {e:?}"))?,
        controller: load_controller_config(&p("controller_config.json"))
            .map_err(|e| format!("controller_config: {e:?}"))?,
    })
}

/// One recorded step.
#[derive(Copy, Clone, Debug)]
pub struct Sample {
    /// Time after the step, s.
    pub t_s: f64,
    /// NED position, m.
    pub position_ned_m: [f64; 3],
    /// NED velocity, m/s.
    pub velocity_ned_mps: [f64; 3],
    /// Body rates, rad/s.
    pub rates_rps: [f64; 3],
    /// Angle between the body -Z axis and vertical, rad.
    pub tilt_rad: f64,
    /// Heading from north, rad.
    pub yaw_rad: f64,
    /// Controller output of this step.
    pub out: Output,
    /// Sum of the four motor thrusts, N.
    pub thrust_n: f64,
    /// Per-motor bit mask: propeller outside the measured table.
    pub prop_extrapolated: i32,
}

/// Vehicle + controller + their states.
pub struct Rig {
    /// Physics state.
    pub state: DroneState,
    /// Wind for the next steps.
    pub wind: WindInput,
    /// Controller tuning derived from the build.
    pub tuning: Tuning,
    /// Controller state (counters live here).
    pub ctl: ControllerState,
    /// Time since construction, s.
    pub t_s: f64,
    pts: PowertrainState,
    vehicle: VehicleConfig,
    pcfg: PowertrainConfig,
    env: EnvironmentInput,
    latch: FaultLatch,
    tel: DebugTelemetry,
    pt_tel: PowertrainTelemetry,
}

fn angles(q: [f32; 4]) -> (f64, f64) {
    let quat = DQuat::from_xyzw(
        f64::from(q[0]),
        f64::from(q[1]),
        f64::from(q[2]),
        f64::from(q[3]),
    )
    .normalize();
    let z_b = quat * DVec3::Z;
    let yaw = (2.0 * (quat.w * quat.z + quat.x * quat.y))
        .atan2(1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
    (z_b.z.clamp(-1.0, 1.0).acos(), yaw)
}

impl Rig {
    /// Hovering at rest `altitude_m` above the origin, full battery, 20 C.
    ///
    /// # Errors
    /// A text if a config is refused.
    pub fn new(build: &Build, altitude_m: f64) -> Result<Self, String> {
        let mut state = ffi_create_default_drone_state();
        state.position = [0.0, 0.0, -altitude_m];
        Ok(Self {
            state,
            wind: WindInput::still(),
            tuning: Tuning::new(&build.controller, &build.vehicle)
                .map_err(|e| format!("tuning: {e:?}"))?,
            ctl: ControllerState::new(),
            t_s: 0.0,
            pts: initial_state(1.0, 20.0).map_err(|e| format!("powertrain state: {e:?}"))?,
            vehicle: build.vehicle,
            pcfg: build.powertrain,
            env: EnvironmentInput::standard(),
            latch: FaultLatch::new(),
            tel: DebugTelemetry::new(),
            pt_tel: PowertrainTelemetry::no_data(),
        })
    }

    /// Replaces the atmosphere used by the FFI step (campaign cross-check).
    pub fn set_environment(&mut self, env: EnvironmentInput) {
        self.env = env;
    }

    /// One controller step followed by one core step.
    ///
    /// # Errors
    /// A text if the controller or the core refuses.
    pub fn step(&mut self, sp: &Setpoint) -> Result<Sample, String> {
        let est = Estimate::from_state(&self.state);
        let out = step(&self.tuning, &mut self.ctl, &est, sp, DT_S)
            .map_err(|e| format!("controller: {e:?}"))?;
        let controls = out.to_control_inputs();
        // SAFETY: every pointer is valid, aligned and unaliased for the call; the surrogate handles are null (analytic drag).
        let code = unsafe {
            ffi_step_powertrain(
                ptr::from_mut(&mut self.state),
                ptr::from_mut(&mut self.pts),
                ptr::from_ref(&controls),
                ptr::from_ref(&self.wind),
                ptr::from_ref(&self.env),
                ptr::null_mut(),
                ptr::null_mut(),
                ptr::from_ref(&self.vehicle),
                ptr::from_ref(&self.pcfg),
                ptr::from_mut(&mut self.tel),
                ptr::from_mut(&mut self.pt_tel),
                ptr::from_mut(&mut self.latch),
                DT_S,
            )
        };
        if code != 0 {
            return Err(format!("core step code {code}"));
        }
        self.t_s += DT_S;
        let (tilt_rad, yaw_rad) = angles(self.state.orientation);
        let w = |v: f32| f64::from(v);
        Ok(Sample {
            t_s: self.t_s,
            position_ned_m: self.state.position,
            velocity_ned_mps: self.state.velocity.map(w),
            rates_rps: self.state.angular_velocity.map(w),
            tilt_rad,
            yaw_rad,
            out,
            thrust_n: self.pt_tel.motor_thrust_n.iter().sum(),
            prop_extrapolated: self.pt_tel.prop_extrapolated,
        })
    }

    /// Flies `seconds` with a setpoint that may depend on time.
    ///
    /// # Errors
    /// First error of `step`.
    pub fn fly_with(
        &mut self,
        seconds: f64,
        mut sp: impl FnMut(f64) -> Setpoint,
    ) -> Result<Vec<Sample>, String> {
        // steps = seconds / dt, a small non-negative whole number in every use here
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n = (seconds / DT_S).round().max(0.0) as usize;
        let mut v = Vec::with_capacity(n);
        for _ in 0..n {
            let s = sp(self.t_s);
            v.push(self.step(&s)?);
        }
        Ok(v)
    }

    /// Flies `seconds` with a constant setpoint.
    ///
    /// # Errors
    /// First error of `step`.
    pub fn fly(&mut self, sp: &Setpoint, seconds: f64) -> Result<Vec<Sample>, String> {
        self.fly_with(seconds, |_| *sp)
    }
}

/// Numbers measured in one scenario. Nothing here is a requirement: tests assert bands around them, the example prints them.
#[derive(Clone, Debug)]
pub struct Report {
    /// Scenario name.
    pub name: &'static str,
    /// Step-response numbers of the scenario's main signal, if it has a step.
    pub metrics: Option<StepMetrics>,
    /// Largest tilt, deg.
    pub max_tilt_deg: f64,
    /// Largest horizontal speed, m/s.
    pub max_speed_mps: f64,
    /// Altitude change between start and end, m (+ = higher).
    pub altitude_change_m: f64,
    /// Lowest altitude minus start altitude, m.
    pub min_altitude_change_m: f64,
    /// Largest horizontal distance from the start position, m.
    pub max_drift_m: f64,
    /// Controller counters at the end.
    pub steps: u64,
    /// Steps the headroom changed the collective.
    pub collective_limited_steps: u64,
    /// Steps an axis command was at +-1.
    pub axis_saturated_steps: u64,
    /// Steps the tilt limit cut the thrust.
    pub tilt_limited_steps: u64,
    /// Fraction of samples with any propeller outside the measured table.
    pub prop_outside_fraction: f64,
    /// Every sample finite.
    pub all_finite: bool,
}

fn horizontal_speed(s: &Sample) -> f64 {
    s.velocity_ned_mps[0].hypot(s.velocity_ned_mps[1])
}

fn report(
    name: &'static str,
    rig: &Rig,
    s: &[Sample],
    start: [f64; 3],
    metrics: Option<StepMetrics>,
) -> Report {
    let alt = |x: &Sample| -x.position_ned_m[2];
    let start_alt = -start[2];
    let last = s.last().copied();
    let flagged = s.iter().filter(|x| x.prop_extrapolated != 0).count();
    Report {
        name,
        metrics,
        max_tilt_deg: s
            .iter()
            .map(|x| x.tilt_rad)
            .fold(0.0, f64::max)
            .to_degrees(),
        max_speed_mps: s.iter().map(horizontal_speed).fold(0.0, f64::max),
        altitude_change_m: last.map_or(0.0, |x| alt(&x) - start_alt),
        min_altitude_change_m: s.iter().map(alt).fold(f64::INFINITY, f64::min) - start_alt,
        max_drift_m: s
            .iter()
            .map(|x| (x.position_ned_m[0] - start[0]).hypot(x.position_ned_m[1] - start[1]))
            .fold(0.0, f64::max),
        steps: rig.ctl.steps,
        collective_limited_steps: rig.ctl.collective_limited_steps,
        axis_saturated_steps: rig.ctl.axis_saturated_steps,
        tilt_limited_steps: rig.ctl.tilt_limited_steps,
        prop_outside_fraction: if s.is_empty() {
            0.0
        } else {
            f64::from(u32::try_from(flagged).unwrap_or(u32::MAX))
                / f64::from(u32::try_from(s.len()).unwrap_or(u32::MAX))
        },
        all_finite: s.iter().all(|x| {
            x.position_ned_m
                .iter()
                .chain(&x.velocity_ned_mps)
                .chain(&x.rates_rps)
                .all(|v| v.is_finite())
                && x.tilt_rad.is_finite()
                && x.thrust_n.is_finite()
        }),
    }
}

/// Altitude of the start of every scenario, m.
pub const START_ALTITUDE_M: f64 = 20.0;
/// Pre-roll hovering time, s: the battery is full, so the duty that hovers is lower than the build's estimate (nominal voltage);
/// the velocity integrator takes this time to absorb it. Every scenario starts after it.
pub const PREROLL_S: f64 = 8.0;

/// A rig that has hovered `PREROLL_S` at the start point.
///
/// # Errors
/// As `Rig::new` / `Rig::fly`.
pub fn settled_rig(build: &Build) -> Result<Rig, String> {
    let mut rig = Rig::new(build, START_ALTITUDE_M)?;
    let hold = Setpoint::Position {
        ned_m: [0.0, 0.0, -START_ALTITUDE_M],
        yaw_rad: 0.0,
    };
    rig.fly(&hold, PREROLL_S)?;
    rig.ctl.steps = 0;
    rig.ctl.collective_limited_steps = 0;
    rig.ctl.axis_saturated_steps = 0;
    rig.ctl.tilt_limited_steps = 0;
    Ok(rig)
}

/// A scenario's step signal: how to read it from a sample, its start and its target.
type MainSignal<'a> = (&'a dyn Fn(&Sample) -> f64, f64, f64);

fn run_one(
    build: &Build,
    name: &'static str,
    secs: f64,
    sp: impl FnMut(f64) -> Setpoint,
    wind: [f64; 3],
    main_signal: Option<MainSignal<'_>>,
) -> Result<Report, String> {
    let mut rig = settled_rig(build)?;
    rig.wind = WindInput { wind_ned_mps: wind };
    let start = rig.state.position;
    let s = rig.fly_with(secs, sp)?;
    let metrics = main_signal.and_then(|(f, a, b)| {
        let v: Vec<f64> = s.iter().map(f).collect();
        step_metrics(&v, DT_S, a, b, 0.02)
    });
    Ok(report(name, &rig, &s, start, metrics))
}

/// The scenario set of the P3c plan, flown on `build`. FPV-class extras (sprint, acro roll) run for every class; their numbers are
/// only meaningful where the class limits allow them.
///
/// # Errors
/// First error of any run.
pub fn run_scenarios(build: &Build) -> Result<Vec<Report>, String> {
    let c = &build.controller;
    let a0 = START_ALTITUDE_M;
    let v_step = (0.4 * c.max_speed_h_mps).min(10.0);
    let pos = |n: f64, e: f64, d: f64, yaw: f64| Setpoint::Position {
        ned_m: [n, e, d],
        yaw_rad: yaw,
    };
    let vel = |n: f64, yaw: f64| Setpoint::Velocity {
        ned_mps: [n, 0.0, 0.0],
        yaw_rad: yaw,
    };
    let alt = |x: &Sample| -x.position_ned_m[2];
    let yaw_deg_90 = std::f64::consts::FRAC_PI_2;
    let mut out = vec![
        run_one(
            build,
            "hover_hold",
            5.0,
            |_| pos(0.0, 0.0, -a0, 0.0),
            [0.0; 3],
            None,
        )?,
        run_one(
            build,
            "altitude_step_2m",
            8.0,
            |_| pos(0.0, 0.0, -a0 - 2.0, 0.0),
            [0.0; 3],
            Some((&alt, a0, a0 + 2.0)),
        )?,
        run_one(
            build,
            "velocity_step",
            10.0,
            |_| vel(v_step, 0.0),
            [0.0; 3],
            Some((&|x: &Sample| x.velocity_ned_mps[0], 0.0, v_step)),
        )?,
        run_one(
            build,
            "yaw_step_90deg",
            8.0,
            |_| pos(0.0, 0.0, -a0, yaw_deg_90),
            [0.0; 3],
            Some((&|x: &Sample| x.yaw_rad, 0.0, yaw_deg_90)),
        )?,
        run_one(
            build,
            "position_step_10m",
            20.0,
            |_| pos(10.0, 0.0, -a0, 0.0),
            [0.0; 3],
            Some((&|x: &Sample| x.position_ned_m[0], 0.0, 10.0)),
        )?,
        run_one(
            build,
            "gust_5mps_east",
            10.0,
            |_| pos(0.0, 0.0, -a0, 0.0),
            [0.0, 5.0, 0.0],
            None,
        )?,
        run_one(
            build,
            "sprint_max_speed",
            8.0,
            |_| vel(c.max_speed_h_mps, 0.0),
            [0.0; 3],
            None,
        )?,
    ];
    out.push(acro_roll(build)?);
    Ok(out)
}

/// Full roll stick from hover for 1 s; the main signal is the roll rate against the class acro rate.
fn acro_roll(build: &Build) -> Result<Report, String> {
    let mut rig = settled_rig(build)?;
    let start = rig.state.position;
    let sp = Setpoint::Acro {
        roll: 1.0,
        pitch: 0.0,
        yaw: 0.0,
        throttle: build.controller.hover_duty,
    };
    let s = rig.fly(&sp, 1.0)?;
    let v: Vec<f64> = s.iter().map(|x| x.rates_rps[0]).collect();
    let metrics = step_metrics(&v, DT_S, 0.0, build.controller.acro_max_rate_rps, 0.02);
    Ok(report("acro_full_roll_stick", &rig, &s, start, metrics))
}

#[cfg(test)]
mod tests {
    use super::*;

    // All bands below are ENGINEERING CHOICES (not requirements, no source): each is a margin around the numbers measured with
    // `cargo run --release --example controller_tuning -- <build>` on 2026-10-10, wide enough for another libm but tight enough
    // that a sign, clamp or gain error fails. The measured values are in `tools/parts/builds/<build>/controller_tuning.json`.

    /// Scenario reports of a golden build, computed once per build for the whole test binary (each set is about 30 s in debug).
    fn reports(name: &str) -> &'static [Report] {
        static REF: std::sync::OnceLock<Vec<Report>> = std::sync::OnceLock::new();
        static FPV: std::sync::OnceLock<Vec<Report>> = std::sync::OnceLock::new();
        let cell = if name == "reference_quad" { &REF } else { &FPV };
        cell.get_or_init(|| run_scenarios(&load_build(&golden_dir(name)).unwrap()).unwrap())
    }

    fn get<'a>(r: &'a [Report], name: &str) -> &'a Report {
        r.iter().find(|x| x.name == name).unwrap()
    }

    fn check_common(r: &[Report], tilt_limit_deg: f64) {
        for x in r {
            assert!(x.all_finite, "{}", x.name);
            // the tilt limit holds in level modes (acro has none)
            if x.name != "acro_full_roll_stick" {
                assert!(
                    x.max_tilt_deg <= tilt_limit_deg + 1.0,
                    "{} tilt {}",
                    x.name,
                    x.max_tilt_deg
                );
            }
        }
        let hover = get(r, "hover_hold");
        assert!(
            hover.altitude_change_m.abs() < 0.05 && hover.max_drift_m < 0.01,
            "hover drift"
        );
        assert_eq!(
            (hover.axis_saturated_steps, hover.collective_limited_steps),
            (0, 0)
        );
        let alt = get(r, "altitude_step_2m").metrics.unwrap();
        assert!(
            alt.overshoot_percent <= 10.0 && alt.final_error_fraction < 0.01,
            "{alt:?}"
        );
        let yaw = get(r, "yaw_step_90deg").metrics.unwrap();
        assert!(
            yaw.overshoot_percent <= 5.0
                && yaw.settling_time_s <= 4.0
                && yaw.final_error_fraction < 0.001,
            "{yaw:?}"
        );
        let pos = get(r, "position_step_10m");
        let m = pos.metrics.unwrap();
        assert!(
            m.overshoot_percent <= 5.0
                && m.final_error_fraction < 0.001
                && m.settling_time_s <= 10.0,
            "{m:?}"
        );
        assert!((pos.max_drift_m - 10.0).abs() < 0.2);
        let gust = get(r, "gust_5mps_east");
        assert!(
            gust.max_drift_m < 0.2 && gust.altitude_change_m.abs() < 0.05,
            "gust drift {}",
            gust.max_drift_m
        );
        let roll = get(r, "acro_full_roll_stick").metrics.unwrap();
        assert!(
            roll.overshoot_percent <= 10.0
                && roll.settling_time_s <= 0.5
                && roll.final_error_fraction < 0.01,
            "{roll:?}"
        );
    }

    #[test]
    fn test_ctl_sim_reference_quad_closed_loop_bands() {
        let r = reports("reference_quad");
        check_common(r, 35.0);
        let alt = get(r, "altitude_step_2m").metrics.unwrap();
        assert!(
            alt.settling_time_s <= 5.0 && alt.rise_time_s.unwrap() <= 3.0,
            "{alt:?}"
        );
        let v = get(r, "velocity_step");
        let m = v.metrics.unwrap();
        assert!(
            m.overshoot_percent <= 15.0
                && m.settling_time_s <= 4.0
                && m.final_error_fraction < 0.001,
            "{m:?}"
        );
        assert!(v.altitude_change_m.abs() < 0.3 && v.min_altitude_change_m > -0.5);
        // the sprint is held to the class speed limit (12 m/s) and keeps its altitude within a metre
        let s = get(r, "sprint_max_speed");
        assert!(
            s.max_speed_mps > 11.0 && s.max_speed_mps < 13.0 && s.min_altitude_change_m > -1.5,
            "{s:?}"
        );
        assert!(s.tilt_limited_steps > 0); // 35 degrees is the binding limit at 6 m/s^2
    }

    #[test]
    fn test_ctl_sim_fpv_closed_loop_bands() {
        let r = reports("fpv_5in");
        check_common(r, 60.0);
        let v = get(r, "velocity_step");
        let m = v.metrics.unwrap();
        assert!(
            m.overshoot_percent <= 15.0
                && m.settling_time_s <= 4.0
                && m.final_error_fraction < 0.001,
            "{m:?}"
        );
        assert!(
            v.max_tilt_deg > 50.0,
            "the 10 m/s step needs a big tilt on this class: {}",
            v.max_tilt_deg
        );
        // acro: full stick reaches 670 deg/s quickly
        let roll = get(r, "acro_full_roll_stick").metrics.unwrap();
        assert!(
            roll.rise_time_s.unwrap() <= 0.1 && roll.settling_time_s <= 0.3,
            "{roll:?}"
        );
    }

    #[test]
    fn test_ctl_sim_fpv_sprint_is_stable_but_does_not_reach_the_limit() {
        // KNOWN LIMITATION, recorded not hidden: the sprint demand is 35 m/s; the run stays finite and bounded, tilts to the 60 degree
        // limit, but flies at about 20 m/s and sinks about 12 m in 8 s because the legacy analytic drag (fixed body-axis areas and
        // coefficients of the demo airframe, not this vehicle) loads the body Z axis when the airframe is pitched. The CFD aero
        // model (P5) is meant to replace it. Bands here are wide on purpose.
        let r = reports("fpv_5in");
        let s = get(r, "sprint_max_speed");
        assert!(s.all_finite && s.max_tilt_deg <= 61.0);
        assert!(
            s.max_speed_mps > 15.0 && s.max_speed_mps < 36.0,
            "{}",
            s.max_speed_mps
        );
        assert!(
            s.min_altitude_change_m > -20.0,
            "{}",
            s.min_altitude_change_m
        );
        assert!(s.tilt_limited_steps > 0);
    }

    #[test]
    fn test_ctl_sim_propeller_outside_the_table_is_reported() {
        // the FPV propeller (12x6E table used as a stand-in) spins above the table's 6018 rpm in every scenario: flagged all the time
        let fpv = reports("fpv_5in");
        assert!(
            fpv.iter().all(|x| x.prop_outside_fraction > 0.99),
            "{:?}",
            fpv.iter()
                .map(|x| x.prop_outside_fraction)
                .collect::<Vec<_>>()
        );
        // the reference quad hovers below the first curve at full battery, so it is flagged there too, but not during a climb of 2 m
        let r = reports("reference_quad");
        assert!(get(r, "hover_hold").prop_outside_fraction > 0.99);
        assert!(get(r, "altitude_step_2m").prop_outside_fraction < 0.01);
    }

    #[test]
    fn test_ctl_sim_runs_are_bit_identical() {
        let b = load_build(&golden_dir("reference_quad")).unwrap();
        let run = || {
            let mut rig = settled_rig(&b).unwrap();
            let sp = Setpoint::Position {
                ned_m: [4.0, -2.0, -21.0],
                yaw_rad: 0.7,
            };
            let s = rig.fly(&sp, 3.0).unwrap();
            let l = s.last().copied().unwrap();
            (
                l.position_ned_m.map(f64::to_bits),
                l.out.throttle.to_bits(),
                l.yaw_rad.to_bits(),
            )
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn test_ctl_sim_golden_controller_configs_match_the_class_defaults() {
        let std = load_build(&golden_dir("reference_quad"))
            .unwrap()
            .controller;
        let fpv = load_build(&golden_dir("fpv_5in")).unwrap().controller;
        assert!((std.max_tilt_rad - 35.0_f64.to_radians()).abs() < 1e-12);
        assert!((fpv.max_tilt_rad - 60.0_f64.to_radians()).abs() < 1e-12);
        assert!((fpv.acro_max_rate_rps - 670.0_f64.to_radians()).abs() < 1e-12);
        // build-specific hover duty: CALCULATED by the builder (full battery needs less, nominal voltage 14.8 V -> 0.368)
        assert!((std.hover_duty - 0.368_337_428_093_147_6).abs() < 1e-9);
        assert!(fpv.hover_duty > 0.2 && fpv.hover_duty < 0.4);
    }

    #[test]
    fn test_ctl_sim_missing_build_is_an_error() {
        assert!(load_build(&golden_dir("no_such_build")).is_err());
    }
}
