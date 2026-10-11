// Tier: F
//! Headless test campaign engine (P4): runs a library of tests on a built vehicle under user-set conditions and reports breaking points.
//!
//! Tests (definitions in `campaign/test_definitions.json`, every value ASSUMED): `hover_payload` (bisection on payload mass),
//! `wind_gust` (wind ramp with refinement), `climb`, `endurance` (hover or square loop until battery cutoff) and `thermal_soak`.
//! Deferred: maximum speed and the two FPV tests (they need a better drag model than the legacy analytic one).
//!
//! The vehicle is stepped from the public pure functions (`aero`, `powertrain`, `mixer`, `integrator`) with the typed AMBIENT
//! temperature (`ffi_step_powertrain` ties the thermal ambient to 20 C + the ISA offset, which cannot represent a typed temperature;
//! a test proves bit equality with the FFI step when the ambient is 20 C + offset). Structural / FEA margins are not part of campaign
//! runs. Drag is the legacy analytic model and every manifest says so.
//!
//! Determinism: no wall clock anywhere, seeded turbulence, results collected by test index, so the files are identical for any thread count.

use crate::aero;
use crate::config;
use crate::controller::{self, ControllerConfig, ControllerState, Estimate, Setpoint, Tuning};
use crate::controller_sim::{Build, DT_S};
use crate::integrator;
use crate::mixer;
use crate::payload;
use crate::powertrain::{self, StepInput};
use crate::turbulence::{Turbulence, TurbulenceKind};
use crate::types::{
    DroneState, EnvironmentInput, PowertrainConfig, PowertrainState, PowertrainTelemetry,
    VehicleConfig, WindInput,
};
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};

/// Standard gravity, m/s^2 (CGPM 1901).
const GRAVITY_MPS2: f64 = 9.806_65;
/// ISA sea-level temperature, deg C (288.15 K, ICAO Doc 7488 [UNVERIFIED]); same value as `aero.rs`.
const ISA_SEA_LEVEL_C: f64 = 15.0;
/// ISA troposphere lapse rate, K/m (ICAO Doc 7488 [UNVERIFIED]); same value as `aero.rs`.
const ISA_LAPSE_K_PER_M: f64 = 0.006_5;
/// Largest mean wind accepted, m/s (the core accepts +-30 m/s per component including turbulence).
pub const MAX_MEAN_WIND_MPS: f64 = 20.0;
/// Margin kept below the core's per-component wind limit when turbulence is added, m/s.
const WIND_LIMIT_MPS: f64 = 29.9;
/// Version of the conditions / definitions / output layouts.
pub const SCHEMA_VERSION: u32 = 1;
/// The drag model every run uses today (named in every manifest).
pub const DRAG_MODEL: &str = "analytic_legacy";

/// Why a campaign call was refused or a run failed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CampaignError {
    /// A conditions / definitions value is out of range or inconsistent (text names it).
    Invalid(&'static str),
    /// A file could not be read.
    Io,
    /// A file is not valid JSON of the expected shape.
    Parse,
    /// The vehicle model refused a step (text names the part).
    Model(&'static str),
    /// A search or run could not be carried out (e.g. the controller refused its input).
    Run(&'static str),
}

// ----------------------------------------------------------------------------------------------- conditions

/// Which turbulence model a run uses.
pub use crate::turbulence::TurbulenceKind as Turb;

/// The conditions a campaign runs under.
#[derive(Clone, Debug, PartialEq)]
pub struct Conditions {
    /// Optional name of the condition set (e.g. `standard_day`); written to the manifest only when given.
    pub name: Option<String>,
    /// Ambient air temperature at the site, deg C (thermal ambient and, with the site altitude, the density).
    pub ambient_c: f64,
    /// Height of the take-off point above sea level, m.
    pub site_altitude_m: f64,
    /// Mean wind speed (also the wind at 20 ft for Dryden), m/s.
    pub wind_speed_mps: f64,
    /// Meteorological direction the wind comes FROM, deg (0 = from north, 90 = from east).
    pub wind_from_deg: f64,
    /// Turbulence model.
    pub turbulence: TurbulenceKind,
    /// Simple-gust RMS as a fraction of the mean wind (ignored by Dryden).
    pub gust_level: f64,
    /// Seed of the turbulence.
    pub seed: u64,
    /// Payload mass, kg.
    pub payload_kg: f64,
    /// Payload position, body frame from the vehicle CG, m.
    pub payload_position_m: [f64; 3],
    /// Battery state of charge at the start, 0..1.
    pub battery_soc: f64,
    /// Battery temperature at the start, deg C (default: ambient).
    pub battery_temp_c: Option<f64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ConditionsJson {
    schema_version: u32,
    #[serde(default)]
    name: Option<String>,
    ambient_c: f64,
    site_altitude_m: f64,
    wind_speed_mps: f64,
    wind_from_deg: f64,
    turbulence: String,
    gust_level: f64,
    seed: u64,
    payload_kg: f64,
    payload_position_m: [f64; 3],
    battery_soc: f64,
    battery_temp_c: Option<f64>,
}

/// ISA temperature at `height_m` above sea level, deg C (troposphere, 0..=11000 m).
#[must_use]
pub fn isa_temperature_c(height_m: f64) -> f64 {
    ISA_SEA_LEVEL_C - ISA_LAPSE_K_PER_M * height_m
}

impl Conditions {
    /// `temp_offset_c` for the core's `EnvironmentInput`: ambient minus the ISA temperature at the site.
    #[must_use]
    pub fn temp_offset_c(&self) -> f64 {
        self.ambient_c - isa_temperature_c(self.site_altitude_m)
    }

    /// The core's atmosphere for these conditions.
    #[must_use]
    pub fn environment(&self) -> EnvironmentInput {
        EnvironmentInput {
            site_altitude_m: self.site_altitude_m,
            temp_offset_c: self.temp_offset_c(),
        }
    }

    /// Mean wind as a NED vector (toward the direction the wind blows), m/s.
    #[must_use]
    pub fn mean_wind_ned(&self, speed_mps: f64) -> [f64; 3] {
        let from = self.wind_from_deg.to_radians();
        [-speed_mps * from.cos(), -speed_mps * from.sin(), 0.0]
    }

    /// Range checks.
    ///
    /// # Errors
    /// `Invalid` naming the first value out of range.
    pub fn validate(&self) -> Result<(), CampaignError> {
        let bad = |m: &'static str| Err(CampaignError::Invalid(m));
        // the name becomes a folder name in the report tools: lower-case letters, digits and underscore only
        if let Some(n) = &self.name
            && !(!n.is_empty()
                && n.len() <= 40
                && n.bytes()
                    .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_'))
        {
            return bad("condition set name must be 1..=40 of a-z 0-9 _");
        }
        if !(self.ambient_c.is_finite() && (-40.0..=55.0).contains(&self.ambient_c)) {
            return bad("ambient_c must be in -40..=55");
        }
        if !(self.site_altitude_m.is_finite() && (0.0..=4000.0).contains(&self.site_altitude_m)) {
            return bad("site_altitude_m must be in 0..=4000");
        }
        if !(self.wind_speed_mps.is_finite()
            && (0.0..=MAX_MEAN_WIND_MPS).contains(&self.wind_speed_mps))
        {
            return bad("wind_speed_mps must be in 0..=20");
        }
        if !(self.wind_from_deg.is_finite() && (0.0..360.0).contains(&self.wind_from_deg)) {
            return bad("wind_from_deg must be in 0..360");
        }
        if !(self.gust_level.is_finite() && (0.0..=1.0).contains(&self.gust_level)) {
            return bad("gust_level must be in 0..=1");
        }
        if !(self.battery_soc.is_finite() && (0.0..=1.0).contains(&self.battery_soc)) {
            return bad("battery_soc must be in 0..=1");
        }
        if !self
            .battery_temp_c
            .is_none_or(|t| t.is_finite() && (-40.0..=100.0).contains(&t))
        {
            return bad("battery_temp_c must be in -40..=100");
        }
        payload::validate_payload(self.payload_kg, self.payload_position_m)
            .map_err(|_| CampaignError::Invalid("payload_kg / payload_position_m out of range"))
    }
}

/// Parses and validates a conditions file.
///
/// # Errors
/// `Parse` for bad JSON / unknown key / unknown turbulence name; `Invalid` for a wrong version or an out-of-range value.
pub fn parse_conditions(json: &str) -> Result<Conditions, CampaignError> {
    let j: ConditionsJson = serde_json::from_str(json).map_err(|_| CampaignError::Parse)?;
    let turbulence = match j.turbulence.as_str() {
        "none" => TurbulenceKind::None,
        "dryden" => TurbulenceKind::Dryden,
        "simple" => TurbulenceKind::Simple,
        _ => return Err(CampaignError::Parse),
    };
    if j.schema_version != SCHEMA_VERSION {
        return Err(CampaignError::Invalid("conditions schema_version"));
    }
    let c = Conditions {
        name: j.name,
        ambient_c: j.ambient_c,
        site_altitude_m: j.site_altitude_m,
        wind_speed_mps: j.wind_speed_mps,
        wind_from_deg: j.wind_from_deg,
        turbulence,
        gust_level: j.gust_level,
        seed: j.seed,
        payload_kg: j.payload_kg,
        payload_position_m: j.payload_position_m,
        battery_soc: j.battery_soc,
        battery_temp_c: j.battery_temp_c,
    };
    c.validate()?;
    Ok(c)
}

// ----------------------------------------------------------------------------------------------- definitions

/// `hover_payload` definition.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HoverPayloadDef {
    /// Definition version.
    pub version: String,
    /// Start altitude, m above the site.
    pub start_altitude_m: f64,
    /// Pre-roll hover before the evaluation, s.
    pub settle_s: f64,
    /// Total hold time of one evaluation, s.
    pub hold_s: f64,
    /// Evaluation window at the end of the hold, s.
    pub eval_window_s: f64,
    /// Upper bound of the payload search, kg.
    pub payload_max_kg: f64,
    /// Bisection stops when the bracket is this narrow, kg.
    pub tolerance_kg: f64,
    /// Largest altitude error in the window, m.
    pub altitude_error_limit_m: f64,
    /// Smallest allowed `max thrust / weight` in the window.
    pub min_thrust_reserve_ratio: f64,
    /// Collective at or above this counts as capped.
    pub collective_cap: f64,
    /// Largest fraction of the window with the collective capped.
    pub collective_cap_fraction_limit: f64,
    /// Time-series output interval, s.
    pub output_interval_s: f64,
}

/// `wind_gust` definition.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct WindGustDef {
    /// Definition version.
    pub version: String,
    /// Start altitude, m above the site.
    pub start_altitude_m: f64,
    /// First wind speed of the ramp, m/s.
    pub start_wind_mps: f64,
    /// Ramp step, m/s.
    pub step_mps: f64,
    /// Last wind speed tried, m/s (<= 20).
    pub max_wind_mps: f64,
    /// Settle time before the evaluation, s.
    pub settle_s: f64,
    /// Hold time at each wind speed, s.
    pub hold_s: f64,
    /// Evaluation window at the end of the hold, s.
    pub eval_window_s: f64,
    /// Largest horizontal position error in the window, m.
    pub position_error_limit_m: f64,
    /// Largest fraction of the window with an axis command at its rail or the collective limited.
    pub saturation_fraction_limit: f64,
    /// Refinement stops when the bracket is this narrow, m/s.
    pub tolerance_mps: f64,
    /// Time-series output interval, s.
    pub output_interval_s: f64,
}

/// `climb` definition.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ClimbDef {
    /// Definition version.
    pub version: String,
    /// Start altitude, m above the site.
    pub start_altitude_m: f64,
    /// Hover before the climb starts, s.
    pub preroll_s: f64,
    /// Climb speed command, m/s (replaces the class limit for this test).
    pub command_climb_mps: f64,
    /// Upward acceleration limit for this test, m/s^2 (replaces the class limit).
    pub accel_limit_mps2: f64,
    /// Height to climb, m.
    pub target_height_m: f64,
    /// Time allowed, s.
    pub time_limit_s: f64,
    /// Time-series output interval, s.
    pub output_interval_s: f64,
}

/// `endurance` definition.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct EnduranceDef {
    /// Definition version.
    pub version: String,
    /// `hover` or `square_loop`.
    pub mission: String,
    /// Cruise altitude, m above the site.
    pub cruise_altitude_m: f64,
    /// Side of the square loop, m.
    pub loop_side_m: f64,
    /// Loop speed, m/s.
    pub loop_speed_mps: f64,
    /// Hover before the mission, s.
    pub settle_s: f64,
    /// Time limit, s.
    pub max_duration_s: f64,
    /// The battery must stay in cutoff this long to end the run, s.
    pub cutoff_confirm_s: f64,
    /// Altitude below which the run ends as a crash, m above the site.
    pub crash_altitude_m: f64,
    /// Time-series output interval, s.
    pub output_interval_s: f64,
}

/// `thermal_soak` definition.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ThermalSoakDef {
    /// Definition version.
    pub version: String,
    /// Hover altitude, m above the site.
    pub altitude_m: f64,
    /// Time limit, s.
    pub max_duration_s: f64,
    /// Window over which each temperature must stop changing, s.
    pub steady_window_s: f64,
    /// Steady when every node changes less than this, K/min.
    pub steady_rate_k_per_min: f64,
    /// End the run at the first component limit crossing.
    pub stop_on_first_limit: bool,
    /// Hold the state of charge constant to isolate the thermal behaviour.
    pub freeze_soc: bool,
    /// Time-series output interval, s.
    pub output_interval_s: f64,
}

/// `max_speed` definition (any class): a stepped ramp of the commanded forward speed in velocity mode.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MaxSpeedDef {
    /// Definition version.
    pub version: String,
    /// Flight altitude, m above the site.
    pub start_altitude_m: f64,
    /// First commanded speed, m/s.
    pub start_speed_mps: f64,
    /// Ramp step, m/s.
    pub step_mps: f64,
    /// Highest speed tried, m/s (the class speed limit caps it further).
    pub max_speed_mps: f64,
    /// Time from hover to the start of the evaluation window at each level, s (includes the acceleration).
    pub settle_s: f64,
    /// Time each level is flown after the settle time, s.
    pub hold_s: f64,
    /// Length of the evaluation window at the end of the hold, s.
    pub eval_window_s: f64,
    /// A level passes only if the mean horizontal speed in the window is at least this fraction of the command (0..=1).
    pub speed_fraction_min: f64,
    /// Largest |altitude error| in the window that still passes, m.
    pub altitude_error_limit_m: f64,
    /// Lowest thrust reserve ratio in the window that still passes.
    pub min_thrust_reserve_ratio: f64,
    /// Final width of the search bracket, m/s.
    pub tolerance_mps: f64,
    /// Time-series output interval, s.
    pub output_interval_s: f64,
}

/// `fpv_sprint` definition: a straight sprint from hover at the class's speed demand.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FpvSprintDef {
    /// Definition version.
    pub version: String,
    /// Start altitude, m above the site.
    pub start_altitude_m: f64,
    /// Hover before the sprint, s.
    pub preroll_s: f64,
    /// Sprint duration, s.
    pub sprint_s: f64,
    /// Time-series output interval, s.
    pub output_interval_s: f64,
}

/// `fpv_punchout` definition: full throttle straight up from hover (acro mode, sticks centred).
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FpvPunchoutDef {
    /// Definition version.
    pub version: String,
    /// Start altitude, m above the site.
    pub start_altitude_m: f64,
    /// Hover before the punch-out, s.
    pub preroll_s: f64,
    /// Punch-out duration, s.
    pub punchout_s: f64,
    /// Height gain whose time is reported, m.
    pub gain_target_m: f64,
    /// Time-series output interval, s.
    pub output_interval_s: f64,
}

/// A definitions file: one entry per test.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Definitions {
    /// Layout version.
    pub schema_version: u32,
    /// Version of this set of definitions.
    pub definitions_version: String,
    /// Free text: where the values come from.
    pub provenance: serde_json::Value,
    /// `hover_payload` entry.
    pub hover_payload: HoverPayloadDef,
    /// `wind_gust` entry.
    pub wind_gust: WindGustDef,
    /// `climb` entry.
    pub climb: ClimbDef,
    /// `endurance` entry.
    pub endurance: EnduranceDef,
    /// `thermal_soak` entry.
    pub thermal_soak: ThermalSoakDef,
    /// `max_speed` entry.
    pub max_speed: MaxSpeedDef,
    /// `fpv_sprint` entry.
    pub fpv_sprint: FpvSprintDef,
    /// `fpv_punchout` entry.
    pub fpv_punchout: FpvPunchoutDef,
}

fn pos(v: f64) -> bool {
    v.is_finite() && v > 0.0
}

fn frac(v: f64) -> bool {
    v.is_finite() && (0.0..=1.0).contains(&v)
}

impl Definitions {
    /// Range checks of every entry.
    ///
    /// # Errors
    /// `Invalid` naming the test.
    pub fn validate(&self) -> Result<(), CampaignError> {
        let hov = &self.hover_payload;
        let hover_ok = [
            hov.start_altitude_m,
            hov.settle_s,
            hov.hold_s,
            hov.eval_window_s,
            hov.payload_max_kg,
            hov.tolerance_kg,
            hov.altitude_error_limit_m,
            hov.output_interval_s,
        ]
        .iter()
        .all(|&v| pos(v))
            && hov.eval_window_s <= hov.hold_s
            && hov.tolerance_kg < hov.payload_max_kg
            && hov.min_thrust_reserve_ratio >= 1.0
            && hov.min_thrust_reserve_ratio.is_finite()
            && frac(hov.collective_cap)
            && frac(hov.collective_cap_fraction_limit);
        if !hover_ok {
            return Err(CampaignError::Invalid("hover_payload definition"));
        }
        let win = &self.wind_gust;
        let wind_ok = [
            win.start_altitude_m,
            win.step_mps,
            win.settle_s,
            win.hold_s,
            win.eval_window_s,
            win.position_error_limit_m,
            win.tolerance_mps,
            win.output_interval_s,
        ]
        .iter()
        .all(|&v| pos(v))
            && win.start_wind_mps.is_finite()
            && (0.0..=MAX_MEAN_WIND_MPS).contains(&win.max_wind_mps)
            && (0.0..win.max_wind_mps).contains(&win.start_wind_mps)
            && win.eval_window_s <= win.hold_s
            && win.tolerance_mps < win.step_mps
            && frac(win.saturation_fraction_limit);
        if !wind_ok {
            return Err(CampaignError::Invalid("wind_gust definition"));
        }
        let clb = &self.climb;
        let climb_ok = [
            clb.start_altitude_m,
            clb.command_climb_mps,
            clb.accel_limit_mps2,
            clb.target_height_m,
            clb.time_limit_s,
            clb.output_interval_s,
        ]
        .iter()
        .all(|&v| pos(v))
            && clb.preroll_s.is_finite()
            && clb.preroll_s >= 0.0;
        if !climb_ok {
            return Err(CampaignError::Invalid("climb definition"));
        }
        let end = &self.endurance;
        let mission_ok = end.mission == "hover" || end.mission == "square_loop";
        let endurance_ok = mission_ok
            && [
                end.cruise_altitude_m,
                end.loop_side_m,
                end.loop_speed_mps,
                end.max_duration_s,
                end.cutoff_confirm_s,
                end.output_interval_s,
            ]
            .iter()
            .all(|&v| pos(v))
            && end.settle_s.is_finite()
            && end.settle_s >= 0.0
            && end.crash_altitude_m.is_finite()
            && (0.0..end.cruise_altitude_m).contains(&end.crash_altitude_m);
        if !endurance_ok {
            return Err(CampaignError::Invalid("endurance definition"));
        }
        let thm = &self.thermal_soak;
        let thermal_ok = [
            thm.altitude_m,
            thm.max_duration_s,
            thm.steady_window_s,
            thm.steady_rate_k_per_min,
            thm.output_interval_s,
        ]
        .iter()
        .all(|&v| pos(v))
            && thm.steady_window_s < thm.max_duration_s;
        if !thermal_ok {
            return Err(CampaignError::Invalid("thermal_soak definition"));
        }
        self.validate_speed_tests()?;
        if self.schema_version != SCHEMA_VERSION {
            return Err(CampaignError::Invalid("definitions schema_version"));
        }
        Ok(())
    }

    fn validate_speed_tests(&self) -> Result<(), CampaignError> {
        let ms = &self.max_speed;
        let speed_ok = [
            ms.start_altitude_m,
            ms.start_speed_mps,
            ms.step_mps,
            ms.max_speed_mps,
            ms.settle_s,
            ms.hold_s,
            ms.eval_window_s,
            ms.altitude_error_limit_m,
            ms.tolerance_mps,
            ms.output_interval_s,
            ms.speed_fraction_min,
        ]
        .iter()
        .all(|&v| pos(v))
            && ms.start_speed_mps < ms.max_speed_mps
            && ms.eval_window_s <= ms.hold_s
            && ms.tolerance_mps < ms.step_mps
            && ms.speed_fraction_min <= 1.0 // (0, 1]: zero would make every speed pass
            && ms.min_thrust_reserve_ratio.is_finite()
            && ms.min_thrust_reserve_ratio >= 1.0;
        if !speed_ok {
            return Err(CampaignError::Invalid("max_speed definition"));
        }
        let sp = &self.fpv_sprint;
        let sprint_ok = [sp.start_altitude_m, sp.sprint_s, sp.output_interval_s]
            .iter()
            .all(|&v| pos(v))
            && sp.preroll_s.is_finite()
            && sp.preroll_s >= 0.0;
        if !sprint_ok {
            return Err(CampaignError::Invalid("fpv_sprint definition"));
        }
        let po = &self.fpv_punchout;
        let punch_ok = [
            po.start_altitude_m,
            po.punchout_s,
            po.gain_target_m,
            po.output_interval_s,
        ]
        .iter()
        .all(|&v| pos(v))
            && po.preroll_s.is_finite()
            && po.preroll_s >= 0.0;
        if !punch_ok {
            return Err(CampaignError::Invalid("fpv_punchout definition"));
        }
        Ok(())
    }
}

/// Parses and validates a definitions file.
///
/// # Errors
/// `Parse` for bad JSON / unknown key / missing test; `Invalid` for an out-of-range value.
pub fn parse_definitions(json: &str) -> Result<Definitions, CampaignError> {
    let d: Definitions = serde_json::from_str(json).map_err(|_| CampaignError::Parse)?;
    d.validate()?;
    Ok(d)
}

// ----------------------------------------------------------------------------------------------- the vehicle simulation

/// One simulated step as seen by the recorder and the test logic.
#[derive(Copy, Clone)]
pub struct Frame {
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
    /// Heading, rad.
    pub yaw_rad: f64,
    /// Roll about the body X axis, rad (aerospace ZYX Euler, from the quaternion (x, y, z, w)); for the replay viewer.
    pub roll_rad: f64,
    /// Pitch about the body Y axis, rad (same convention); for the replay viewer.
    pub pitch_rad: f64,
    /// Controller output of this step.
    pub out: controller::Output,
    /// Axis command at +-1 this step.
    pub axis_railed: bool,
    /// Wind including turbulence, NED, m/s.
    pub wind_ned_mps: [f64; 3],
    /// Turbulence part of the wind, NED, m/s.
    pub gust_ned_mps: [f64; 3],
    /// Powertrain telemetry of the step.
    pub tel: PowertrainTelemetry,
    /// Air-relative speed, m/s.
    pub air_speed_mps: f64,
    /// Axial inflow, m/s.
    pub axial_inflow_mps: f64,
    /// Air density, kg/m^3.
    pub rho_kgpm3: f64,
}

/// What a drag source answered for one step.
pub struct DragAnswer {
    /// Drag force, NED, N.
    pub force_ned_n: [f32; 3],
    /// The answer came from a CFD surrogate.
    pub from_surrogate: bool,
    /// For a surrogate answer: the query fell inside the model's validated envelope.
    pub inside_envelope: bool,
}

/// Where the aerodynamic drag of a run comes from.
pub trait DragSource {
    /// Text for the manifest (`analytic_legacy` or `cfd_surrogate:<kind>:<model sha256>`).
    fn label(&self) -> String;

    /// Drag for the air-relative state `air_state` (its velocity is the AIR-RELATIVE velocity).
    ///
    /// # Errors
    /// `Model("drag")` if the model cannot answer.
    fn force(
        &mut self,
        air_state: &DroneState,
        altitude_m: f64,
        env: &EnvironmentInput,
    ) -> Result<DragAnswer, CampaignError>;
}

/// The legacy analytic drag (fixed body-axis areas and coefficients of the demo airframe).
pub struct AnalyticDrag;

impl DragSource for AnalyticDrag {
    fn label(&self) -> String {
        DRAG_MODEL.to_string()
    }

    fn force(
        &mut self,
        air_state: &DroneState,
        altitude_m: f64,
        env: &EnvironmentInput,
    ) -> Result<DragAnswer, CampaignError> {
        let f = aero::get_drag_env(air_state, altitude_m, env)
            .map_err(|_| CampaignError::Model("drag"))?;
        Ok(DragAnswer {
            force_ned_n: f,
            from_surrogate: false,
            inside_envelope: false,
        })
    }
}

/// A per-build CFD surrogate (preliminary CFD, unvalidated), with the analytic model as the fallback when it has no answer.
pub struct SurrogateDrag {
    handle: crate::surrogate::SurrogateHandle,
    info: crate::surrogate::ModelInfo,
}

impl SurrogateDrag {
    /// Wraps a loaded model.
    #[must_use]
    pub fn new(
        handle: crate::surrogate::SurrogateHandle,
        info: crate::surrogate::ModelInfo,
    ) -> Self {
        Self { handle, info }
    }
}

impl DragSource for SurrogateDrag {
    fn label(&self) -> String {
        format!(
            "cfd_surrogate:{}:{}",
            self.info.kind, self.info.model_sha256
        )
    }

    fn force(
        &mut self,
        air_state: &DroneState,
        altitude_m: f64,
        env: &EnvironmentInput,
    ) -> Result<DragAnswer, CampaignError> {
        let (f, result) =
            aero::get_drag_with_fallback_env(air_state, altitude_m, env, Some(&mut self.handle))
                .map_err(|_| CampaignError::Model("drag"))?;
        Ok(DragAnswer {
            force_ned_n: f,
            from_surrogate: result.is_some(),
            inside_envelope: result.is_some_and(|r| r.in_validated_envelope == 1),
        })
    }
}

/// What the CFD surrogate did in a campaign, written into `campaign_result.json` and every test `manifest.json` ONLY when present
/// (so an analytic campaign keeps its exact bytes). The report and the app show `status` verbatim, coloured by `level`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CfdNote {
    /// Label for screens and reports, for example `CFD surrogate: SYNTHETIC (formula, not CFD)`.
    pub status: String,
    /// `danger` (stub or synthetic), `warn` (preliminary CFD, or a failed load that fell back to analytic drag) or `info`.
    pub level: &'static str,
    /// Model `kind` metadata (`trained` or `stub`), when a model was loaded.
    pub kind: Option<String>,
    /// SHA-256 of the model file, when a model was loaded or offered.
    pub model_sha256: Option<String>,
    /// Model `provenance` metadata.
    pub provenance: Option<String>,
    /// Held-out RMS force error in N as the trainer wrote it (text; `none` if it could not be computed).
    pub holdout_rms_n: Option<String>,
    /// CFD sanity-check status (`NOT_RUN`, `PARTIAL`, `RUN`) as the model carries it.
    pub cfd_sanity_status: Option<String>,
    /// Why a model that matched the build could not be used (the run then used analytic drag by explicit request).
    pub load_error: Option<String>,
}

impl CfdNote {
    /// JSON object text (keys in a fixed order; absent values are `null`).
    #[must_use]
    pub fn to_json(&self) -> String {
        use crate::textout::json_str;
        let opt = |v: &Option<String>| v.as_deref().map_or_else(|| "null".to_string(), json_str);
        format!(
            "{{\"status\": {}, \"level\": {}, \"kind\": {}, \"model_sha256\": {}, \"provenance\": {}, \"holdout_rms_n\": {}, \"cfd_sanity_status\": {}, \"load_error\": {}}}",
            json_str(&self.status),
            json_str(self.level),
            opt(&self.kind),
            opt(&self.model_sha256),
            opt(&self.provenance),
            opt(&self.holdout_rms_n),
            opt(&self.cfd_sanity_status),
            opt(&self.load_error),
        )
    }
}

/// `,\n  "cfd_surrogate": {...}` or nothing.
fn cfd_extra(note: Option<&CfdNote>) -> String {
    note.map_or_else(String::new, |n| {
        format!(",\n  \"cfd_surrogate\": {}", n.to_json())
    })
}

/// Creates a fresh drag source per simulation (a campaign runs many simulations, possibly on several threads).
pub type DragFactory = dyn Fn() -> Result<Box<dyn DragSource>, CampaignError> + Sync;

/// The factory for the analytic drag.
///
/// # Errors
/// Never.
pub fn analytic_drag() -> Result<Box<dyn DragSource>, CampaignError> {
    Ok(Box::new(AnalyticDrag))
}

/// Drag bookkeeping of a run, for the manifest and the result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DragStats {
    /// Model label.
    pub label: String,
    /// Steps answered by the surrogate.
    pub surrogate_steps: u64,
    /// Steps answered by the analytic model.
    pub analytic_steps: u64,
    /// Surrogate steps outside the validated envelope.
    pub outside_envelope_steps: u64,
}

/// Bisection steps of `sagged_full_duty_point` (the bracket `[0, OCV]` shrinks by 2^-60, far below f64 resolution of a voltage).
const SAG_BISECTION_STEPS: u32 = 60;

/// Thrust of ONE motor at full duty with the pack sagged by the current that full duty itself draws, and the pack voltage there.
///
/// With all four motors identical at duty 1 the motor terminal voltage equals the supply voltage `V`. The pack current is
/// `I(V) = 4 P_el(V) / (eta_esc V)` (`P_el` from `powertrain::solve_motor`) and the supply is `V = OCV - R I(V)`. The residual
/// `g(V) = V - (OCV - R I(V))` is increasing in `V` (the current grows with `V`, so the second term falls), `g(0+) < 0` and
/// `g(OCV) = R I >= 0`, so bisection on `[0, OCV]` finds the unique root. Same state of charge, resistance and air as the present step.
///
/// Returns `(thrust_n, supply_voltage_v)`.
///
/// # Errors
/// `Model("bench point")` for a non-positive or non-finite OCV or if the motor solve refuses.
fn sagged_full_duty_point(
    pcfg: &PowertrainConfig,
    ocv_pack_v: f64,
    rho_kgpm3: f64,
    axial_inflow_mps: f64,
) -> Result<(f64, f64), CampaignError> {
    if !(ocv_pack_v.is_finite() && ocv_pack_v > 0.0) {
        return Err(CampaignError::Model("bench point"));
    }
    let r_ohm = pcfg.battery_internal_resistance_ohm;
    let (mut lo, mut hi) = (0.0_f64, ocv_pack_v);
    for _ in 0..SAG_BISECTION_STEPS {
        let mid = 0.5 * (lo + hi);
        let m = powertrain::solve_motor(pcfg, mid, rho_kgpm3, axial_inflow_mps)
            .map_err(|_| CampaignError::Model("bench point"))?;
        let pack_a = 4.0 * m.electrical_w / (pcfg.esc_efficiency * mid);
        if mid - (ocv_pack_v - r_ohm * pack_a) > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let v = 0.5 * (lo + hi);
    let m = powertrain::solve_motor(pcfg, v, rho_kgpm3, axial_inflow_mps)
        .map_err(|_| CampaignError::Model("bench point"))?;
    Ok((m.thrust_n, v))
}

/// A vehicle with its controller, powertrain state and turbulence, ready to be stepped under given conditions.
pub struct Sim {
    /// Physics state.
    pub state: DroneState,
    /// Mean wind for the next steps, NED m/s (set by the test logic).
    pub mean_wind_ned_mps: [f64; 3],
    /// Controller state (counters live here).
    pub ctl: ControllerState,
    /// Time since construction, s.
    pub t_s: f64,
    /// Steps on which the gust had to be limited to keep the wind inside the core's range.
    pub wind_limited_steps: u64,
    /// The turbulence generator (its `clamped_steps` is reported).
    pub turbulence: Turbulence,
    tuning: Tuning,
    pts: PowertrainState,
    vehicle: VehicleConfig,
    pcfg: PowertrainConfig,
    env: EnvironmentInput,
    ambient_c: f64,
    freeze_soc: Option<f64>,
    drag: Box<dyn DragSource>,
    /// Steps whose drag came from the CFD surrogate.
    pub drag_surrogate_steps: u64,
    /// Steps whose drag came from the analytic model (no surrogate, or the surrogate had no answer).
    pub drag_analytic_steps: u64,
    /// Surrogate steps outside the model's validated envelope (always all of them for a stub).
    pub drag_outside_envelope_steps: u64,
    /// Progress slot of the test this simulation belongs to (`progress::no_test()` outside a campaign); only counted, never read here.
    prog: usize,
}

impl Sim {
    /// Builds a hovering-at-rest simulation `altitude_m` above the site.
    ///
    /// `controller_override` replaces the build's controller config (the climb test relaxes the class limits); the hover duty is
    /// scaled by `sqrt(M'/M)` for the payload and capped at 1.
    ///
    /// # Errors
    /// `Invalid` / `Model` if the conditions, payload or any config is refused.
    pub fn new(
        build: &Build,
        cond: &Conditions,
        altitude_m: f64,
        seed_offset: u64,
        controller_override: Option<ControllerConfig>,
    ) -> Result<Self, CampaignError> {
        Self::with_drag(
            build,
            cond,
            altitude_m,
            seed_offset,
            controller_override,
            Box::new(AnalyticDrag),
        )
    }

    /// Same as `new` with a chosen drag source (the analytic model or a CFD surrogate).
    ///
    /// # Errors
    /// As `new`.
    pub fn with_drag(
        build: &Build,
        cond: &Conditions,
        altitude_m: f64,
        seed_offset: u64,
        controller_override: Option<ControllerConfig>,
        drag: Box<dyn DragSource>,
    ) -> Result<Self, CampaignError> {
        cond.validate()?;
        let loaded =
            payload::apply_payload(&build.vehicle, cond.payload_kg, cond.payload_position_m)
                .map_err(|_| CampaignError::Invalid("payload"))?;
        let mut ccfg = controller_override.unwrap_or(build.controller);
        ccfg.hover_duty = (ccfg.hover_duty * loaded.hover_duty_scale).min(1.0);
        let tuning = Tuning::new(&ccfg, &loaded.vehicle)
            .map_err(|_| CampaignError::Model("controller tuning"))?;
        config::validate_runtime_config(&loaded.vehicle)
            .map_err(|_| CampaignError::Model("vehicle config"))?;
        powertrain::validate_powertrain_config(&build.powertrain)
            .map_err(|_| CampaignError::Model("powertrain config"))?;
        powertrain::check_consistency(&loaded.vehicle, &build.powertrain)
            .map_err(|_| CampaignError::Model("powertrain / vehicle consistency"))?;
        let env = cond.environment();
        aero::validate_environment(&env).map_err(|_| CampaignError::Invalid("atmosphere"))?;
        let mut pts = powertrain::initial_state(cond.battery_soc, cond.ambient_c)
            .map_err(|_| CampaignError::Invalid("battery start state"))?;
        if let Some(t) = cond.battery_temp_c {
            pts.battery_temp_c = t;
        }
        let mut state = crate::ffi_create_default_drone_state();
        state.position = [0.0, 0.0, -altitude_m];
        let turbulence = Turbulence::new(
            cond.turbulence,
            cond.seed.wrapping_add(seed_offset),
            cond.gust_level,
        )
        .map_err(|_| CampaignError::Invalid("gust_level"))?;
        Ok(Self {
            state,
            mean_wind_ned_mps: cond.mean_wind_ned(cond.wind_speed_mps),
            ctl: ControllerState::new(),
            t_s: 0.0,
            wind_limited_steps: 0,
            turbulence,
            tuning,
            pts,
            vehicle: loaded.vehicle,
            pcfg: build.powertrain,
            env,
            ambient_c: cond.ambient_c,
            freeze_soc: None,
            drag,
            drag_surrogate_steps: 0,
            drag_analytic_steps: 0,
            drag_outside_envelope_steps: 0,
            prog: crate::progress::current(),
        })
    }

    /// Drag bookkeeping so far.
    #[must_use]
    pub fn drag_stats(&self) -> DragStats {
        DragStats {
            label: self.drag.label(),
            surrogate_steps: self.drag_surrogate_steps,
            analytic_steps: self.drag_analytic_steps,
            outside_envelope_steps: self.drag_outside_envelope_steps,
        }
    }

    /// Holds the battery state of charge at its current value on every step (thermal soak).
    pub fn freeze_soc(&mut self) {
        self.freeze_soc = Some(self.pts.soc);
    }

    /// The vehicle after the payload (mass, inertia, motor positions).
    #[must_use]
    pub fn vehicle(&self) -> &VehicleConfig {
        &self.vehicle
    }

    /// The thermal limits of the powertrain config (winding, ESC, battery), deg C.
    #[must_use]
    pub fn temperature_limits_c(&self) -> [f64; 3] {
        [
            self.pcfg.motor_winding_temp_limit_c,
            self.pcfg.esc_temp_limit_c,
            self.pcfg.battery_temp_limit_c,
        ]
    }

    /// Weight of the loaded vehicle, N.
    #[must_use]
    pub fn weight_n(&self) -> f64 {
        self.vehicle.mass_kg * GRAVITY_MPS2
    }

    /// `4 x (thrust at full duty at the SAGGED pack voltage) / weight`, the thrust reserve ratio.
    ///
    /// The pack voltage at full duty is not the present (hover / cruise) voltage: four motors at full duty draw about 115 A
    /// on the reference build and the internal resistance drops the supply further. `sagged_full_duty_point` solves for that
    /// voltage with the open-circuit voltage and resistance held at their present values (state of charge unchanged).
    ///
    /// # Errors
    /// `Model` if the solve refuses.
    pub fn thrust_reserve_ratio(&self, f: &Frame) -> Result<f64, CampaignError> {
        let (thrust_n, _) = sagged_full_duty_point(
            &self.pcfg,
            f.tel.battery_ocv_v,
            f.rho_kgpm3,
            f.axial_inflow_mps,
        )?;
        Ok(4.0 * thrust_n / self.weight_n())
    }

    fn limited_wind(&mut self, gust: [f64; 3]) -> ([f64; 3], [f64; 3]) {
        let mut limited = false;
        let mut total = [0.0; 3];
        let mut g = [0.0; 3];
        for i in 0..3 {
            let mean = self.mean_wind_ned_mps.get(i).copied().unwrap_or(0.0);
            let raw = gust.get(i).copied().unwrap_or(0.0);
            let room = (WIND_LIMIT_MPS - mean.abs()).max(0.0);
            let clipped = raw.clamp(-room, room);
            limited |= clipped.to_bits() != raw.to_bits();
            if let (Some(t), Some(gg)) = (total.get_mut(i), g.get_mut(i)) {
                *t = mean + clipped;
                *gg = clipped;
            }
        }
        if limited {
            self.wind_limited_steps = self.wind_limited_steps.saturating_add(1);
        }
        (total, g)
    }

    /// One controller step followed by one vehicle step.
    ///
    /// # Errors
    /// `Run` if the controller refuses its input, `Model` if the physics refuses (divergence, density range).
    pub fn step(&mut self, sp: &Setpoint) -> Result<Frame, CampaignError> {
        crate::progress::add_step(self.prog);
        let est = Estimate::from_state(&self.state);
        let out = controller::step(&self.tuning, &mut self.ctl, &est, sp, DT_S)
            .map_err(|_| CampaignError::Run("controller input"))?;
        let railed = out.roll.abs().max(out.pitch.abs()).max(out.yaw.abs()) >= 1.0;
        let controls = out.to_control_inputs();
        let height = -self.state.position[2];
        let v_air_mean = mean_relative_speed(self.state.velocity, self.mean_wind_ned_mps);
        let gust = self
            .turbulence
            .step(DT_S, height.max(0.0), v_air_mean, self.mean_wind_ned_mps)
            .map_err(|_| CampaignError::Run("turbulence input"))?;
        let (wind_vec, gust_vec) = self.limited_wind(gust);
        let wind = WindInput {
            wind_ned_mps: wind_vec,
        };
        let (tel, new_state, speed, inflow, rho) = self.advance(&controls, &wind, height)?;
        // The core has no ground. The only other loss signal is the atmosphere model failing below `-site_altitude_m`, which at a high
        // site is far underground, so the ground (height 0 above the NED origin) is checked here for every test.
        if -new_state.position[2] <= 0.0 {
            return Err(CampaignError::Model("ground"));
        }
        self.state = new_state;
        self.t_s += DT_S;
        if let Some(soc) = self.freeze_soc {
            self.pts.soc = soc;
            self.pts.charge_used_ah = 0.0;
        }
        Ok(self.frame(out, railed, wind_vec, gust_vec, &tel, speed, inflow, rho))
    }

    /// The physics of one step (same order as `compute_powertrain_step`): drag, powertrain, forces, RK4.
    fn advance(
        &mut self,
        controls: &crate::types::ControlInputs,
        wind: &WindInput,
        altitude_m: f64,
    ) -> Result<(PowertrainTelemetry, DroneState, f64, f64, f64), CampaignError> {
        let orientation = crate::types::unit_quaternion_xyzw(self.state.orientation)
            .map_err(|_| CampaignError::Model("attitude"))?;
        aero::validate_wind(wind).map_err(|_| CampaignError::Model("wind"))?;
        let work = DroneState {
            orientation,
            ..self.state
        };
        let air_relative = aero::air_relative_velocity(work.velocity, wind);
        let speed = f64::from(air_relative[0])
            .hypot(f64::from(air_relative[1]))
            .hypot(f64::from(air_relative[2]));
        let inflow = powertrain::axial_inflow_mps(air_relative, orientation);
        let rho = aero::air_density(altitude_m, &self.env)
            .map_err(|_| CampaignError::Model("atmosphere"))?;
        let air_state = DroneState {
            velocity: air_relative,
            ..work
        };
        let answer = self.drag.force(&air_state, altitude_m, &self.env)?;
        if answer.from_surrogate {
            self.drag_surrogate_steps += 1;
            self.drag_outside_envelope_steps += u64::from(!answer.inside_envelope);
        } else {
            self.drag_analytic_steps += 1;
        }
        let drag = answer.force_ned_n;
        let input = StepInput {
            duties: mixer::mix_controls(controls),
            rho_kgpm3: rho,
            airspeed_mps: speed,
            axial_inflow_mps: inflow,
            ambient_c: self.ambient_c,
            dt_s: DT_S,
        };
        let mut pts = self.pts;
        let tel = powertrain::step(&self.pcfg, &mut pts, &input)
            .map_err(|_| CampaignError::Model("powertrain"))?;
        let (force, torque, _) = mixer::calculate_net_forces(
            tel.motor_thrust_n,
            drag.map(f64::from),
            orientation,
            &self.vehicle,
        );
        let next = integrator::step_rk4(&work, force, torque, &self.vehicle, DT_S);
        let finite = next.position.iter().all(|v| v.is_finite())
            && next.velocity.iter().all(|v| v.is_finite())
            && next.orientation.iter().all(|v| v.is_finite())
            && next.angular_velocity.iter().all(|v| v.is_finite());
        if !finite {
            return Err(CampaignError::Model("diverged"));
        }
        self.pts = pts;
        Ok((tel, next, speed, inflow, rho))
    }

    #[allow(clippy::too_many_arguments)]
    fn frame(
        &self,
        out: controller::Output,
        railed: bool,
        wind: [f64; 3],
        gust: [f64; 3],
        tel: &PowertrainTelemetry,
        speed: f64,
        inflow: f64,
        rho: f64,
    ) -> Frame {
        let w = |v: f32| f64::from(v);
        let q = DQuat::from_xyzw(
            w(self.state.orientation[0]),
            w(self.state.orientation[1]),
            w(self.state.orientation[2]),
            w(self.state.orientation[3]),
        )
        .normalize();
        let z_b = q * DVec3::Z;
        let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        let (roll, pitch) = roll_pitch_rad(&q);
        Frame {
            t_s: self.t_s,
            position_ned_m: self.state.position,
            velocity_ned_mps: self.state.velocity.map(w),
            rates_rps: self.state.angular_velocity.map(w),
            tilt_rad: z_b.z.clamp(-1.0, 1.0).acos(),
            yaw_rad: yaw,
            roll_rad: roll,
            pitch_rad: pitch,
            out,
            axis_railed: railed,
            wind_ned_mps: wind,
            gust_ned_mps: gust,
            tel: *tel,
            air_speed_mps: speed,
            axial_inflow_mps: inflow,
            rho_kgpm3: rho,
        }
    }
}

/// Roll and pitch, rad, of a unit quaternion (x, y, z, w) in the aerospace ZYX convention (yaw, then pitch, then roll).
/// roll = atan2(2(wx + yz), 1 - 2(x^2 + y^2)); pitch = asin(2(wy - zx)), the argument clamped against rounding past +-1.
fn roll_pitch_rad(q: &DQuat) -> (f64, f64) {
    let roll = (2.0 * (q.w * q.x + q.y * q.z)).atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    let pitch = (2.0 * (q.w * q.y - q.z * q.x)).clamp(-1.0, 1.0).asin();
    (roll, pitch)
}

fn mean_relative_speed(velocity: [f32; 3], wind: [f64; 3]) -> f64 {
    let d = |i: usize| {
        f64::from(velocity.get(i).copied().unwrap_or(0.0)) - wind.get(i).copied().unwrap_or(0.0)
    };
    d(0).hypot(d(1)).hypot(d(2))
}

// ----------------------------------------------------------------------------------------------- recording and events

/// Names of the detected events, in report order.
pub const EVENT_NAMES: [&str; 13] = [
    "motor_over_current",
    "esc_over_current",
    "battery_over_current",
    "motor_over_temp",
    "esc_over_temp",
    "battery_over_temp",
    "battery_cutoff",
    "prop_outside_table",
    "axis_saturated",
    "collective_limited",
    "tilt_limit_active",
    "altitude_hold_lost",
    "instability",
];

/// Tilt above which a run counts as unstable, rad (80 deg).
const INSTABILITY_TILT_RAD: f64 = 1.396_263_401_595_463_7;
/// Body rate above which a run counts as unstable, rad/s.
const INSTABILITY_RATE_RPS: f64 = 30.0;

/// First / last time and activity of one event.
#[derive(Clone, Debug, PartialEq)]
pub struct EventRecord {
    /// Event name (one of `EVENT_NAMES`).
    pub name: &'static str,
    /// First time the condition was true, s.
    pub first_t_s: Option<f64>,
    /// Last time the condition was true, s.
    pub last_t_s: f64,
    /// Steps on which the condition was true.
    pub active_steps: u64,
    /// Number of separate activations (rising edges).
    pub activations: u64,
    active: bool,
}

impl EventRecord {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            first_t_s: None,
            last_t_s: 0.0,
            active_steps: 0,
            activations: 0,
            active: false,
        }
    }

    fn update(&mut self, now: bool, t_s: f64) {
        if now {
            self.first_t_s.get_or_insert(t_s);
            self.last_t_s = t_s;
            self.active_steps = self.active_steps.saturating_add(1);
            if !self.active {
                self.activations = self.activations.saturating_add(1);
            }
        }
        self.active = now;
    }
}

/// A running maximum or minimum with the time it occurred.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Peak {
    /// Extreme value.
    pub value: f64,
    /// Time of the extreme, s.
    pub t_s: f64,
}

impl Peak {
    fn max(init: f64) -> Self {
        Self {
            value: init,
            t_s: 0.0,
        }
    }

    fn update_max(&mut self, v: f64, t_s: f64) {
        if v > self.value {
            *self = Self { value: v, t_s };
        }
    }

    fn update_min(&mut self, v: f64, t_s: f64) {
        if v < self.value {
            *self = Self { value: v, t_s };
        }
    }
}

/// Peaks over a run.
#[derive(Clone, Debug, PartialEq)]
pub struct Peaks {
    /// Hottest winding, deg C.
    pub winding_c: Peak,
    /// Hottest motor case, deg C.
    pub case_c: Peak,
    /// Hottest ESC, deg C.
    pub esc_c: Peak,
    /// Battery temperature, deg C.
    pub battery_c: Peak,
    /// Largest motor current, A.
    pub motor_current_a: Peak,
    /// Largest pack current, A.
    pub pack_current_a: Peak,
    /// Lowest loaded pack voltage, V.
    pub min_pack_v: Peak,
    /// Largest tilt, rad.
    pub tilt_rad: Peak,
    /// Lowest thrust reserve ratio (sampled at the output interval).
    pub min_reserve: Peak,
}

impl Peaks {
    fn new() -> Self {
        Self {
            winding_c: Peak::max(f64::NEG_INFINITY),
            case_c: Peak::max(f64::NEG_INFINITY),
            esc_c: Peak::max(f64::NEG_INFINITY),
            battery_c: Peak::max(f64::NEG_INFINITY),
            motor_current_a: Peak::max(f64::NEG_INFINITY),
            pack_current_a: Peak::max(f64::NEG_INFINITY),
            min_pack_v: Peak::max(f64::INFINITY),
            tilt_rad: Peak::max(f64::NEG_INFINITY),
            min_reserve: Peak::max(f64::INFINITY),
        }
    }
}

/// Statistics over an evaluation window.
#[derive(Clone, Debug, PartialEq)]
pub struct WindowStats {
    /// Steps in the window.
    pub steps: u64,
    /// Largest |altitude error|, m.
    pub max_alt_err_m: f64,
    /// Largest horizontal position error, m.
    pub max_h_err_m: f64,
    /// Steps with an axis command at its rail or the collective limited.
    pub saturated_steps: u64,
    /// Steps with the collective at or above the cap.
    pub capped_steps: u64,
    /// Lowest thrust reserve ratio sampled in the window.
    pub min_reserve: f64,
    /// Largest tilt, rad.
    pub max_tilt_rad: f64,
}

impl WindowStats {
    fn new() -> Self {
        Self {
            steps: 0,
            max_alt_err_m: 0.0,
            max_h_err_m: 0.0,
            saturated_steps: 0,
            capped_steps: 0,
            min_reserve: f64::INFINITY,
            max_tilt_rad: 0.0,
        }
    }

    /// Fraction of the window's steps that were saturated.
    #[must_use]
    pub fn saturated_fraction(&self) -> f64 {
        ratio(self.saturated_steps, self.steps)
    }

    /// Fraction of the window's steps with the collective capped.
    #[must_use]
    pub fn capped_fraction(&self) -> f64 {
        ratio(self.capped_steps, self.steps)
    }
}

fn ratio(a: u64, b: u64) -> f64 {
    f64::from(u32::try_from(a).unwrap_or(u32::MAX))
        / f64::from(u32::try_from(b.max(1)).unwrap_or(u32::MAX))
}

/// Column names of the time-series CSV.
pub const SERIES_COLUMNS: [&str; 57] = [
    "t_s",
    "pos_n_m",
    "pos_e_m",
    "pos_d_m",
    "vel_n_mps",
    "vel_e_mps",
    "vel_d_mps",
    "tilt_deg",
    "yaw_deg",
    "roll_deg",
    "pitch_deg",
    "wind_n_mps",
    "wind_e_mps",
    "wind_d_mps",
    "gust_n_mps",
    "gust_e_mps",
    "gust_d_mps",
    "air_speed_mps",
    "track_err_h_m",
    "track_err_alt_m",
    "rpm_1",
    "rpm_2",
    "rpm_3",
    "rpm_4",
    "current_a_1",
    "current_a_2",
    "current_a_3",
    "current_a_4",
    "torque_nm_1",
    "torque_nm_2",
    "torque_nm_3",
    "torque_nm_4",
    "thrust_n_1",
    "thrust_n_2",
    "thrust_n_3",
    "thrust_n_4",
    "thrust_reserve_ratio",
    "battery_v",
    "battery_a",
    "battery_soc",
    "battery_w",
    "motor_copper_w",
    "motor_noload_w",
    "esc_loss_w",
    "winding_c_max",
    "case_c_max",
    "esc_c_max",
    "battery_c",
    "cmd_throttle",
    "cmd_roll",
    "cmd_pitch",
    "cmd_yaw",
    "prop_outside_mask",
    "collective_limited",
    "axis_railed",
    "tilt_limit_active",
    "hold_error_active",
];

fn max4(v: &[f64; 4]) -> f64 {
    v.iter().copied().fold(f64::NEG_INFINITY, f64::max)
}

/// Records one run: decimated CSV rows, events, peaks and the window statistics.
pub struct Observer {
    interval_steps: u64,
    step: u64,
    csv: String,
    rows: usize,
    /// Events in `EVENT_NAMES` order.
    pub events: Vec<EventRecord>,
    /// Peaks over the run.
    pub peaks: Peaks,
    /// Statistics of the current evaluation window (reset by `start_window`).
    pub window: WindowStats,
    window_cap: f64,
    hold_limit_m: f64,
    /// Steps observed.
    pub steps: u64,
    /// Last observed frame.
    pub last: Option<Frame>,
}

impl Observer {
    /// New observer writing a row every `interval_s`; `hold_limit_m` is the altitude error that raises `altitude_hold_lost`.
    #[must_use]
    pub fn new(interval_s: f64, hold_limit_m: f64, collective_cap: f64) -> Self {
        // interval_s / DT_S is a small positive whole number in every definition
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let interval_steps = ((interval_s / DT_S).round().max(1.0)) as u64;
        let mut csv = SERIES_COLUMNS.join(",");
        csv.push('\n');
        Self {
            interval_steps,
            step: 0,
            csv,
            rows: 0,
            events: EVENT_NAMES.iter().map(|n| EventRecord::new(n)).collect(),
            peaks: Peaks::new(),
            window: WindowStats::new(),
            window_cap: collective_cap,
            hold_limit_m,
            steps: 0,
            last: None,
        }
    }

    /// Starts a fresh evaluation window.
    pub fn start_window(&mut self) {
        self.window = WindowStats::new();
    }

    /// The CSV text so far and its row count.
    #[must_use]
    pub fn csv(&self) -> (&str, usize) {
        (&self.csv, self.rows)
    }

    fn event_mut(&mut self, name: &str) -> Option<&mut EventRecord> {
        self.events.iter_mut().find(|e| e.name == name)
    }

    /// Feeds one step. `target` is the position the vehicle is meant to hold (NED), if it holds one.
    ///
    /// # Errors
    /// `Model` if the thrust reserve cannot be computed; `Run` if a value to be written is not finite.
    pub fn observe(
        &mut self,
        sim: &Sim,
        f: &Frame,
        target: Option<[f64; 3]>,
    ) -> Result<(), CampaignError> {
        self.steps += 1;
        let (alt_err, h_err) = target.map_or((0.0, 0.0), |t| {
            (
                (f.position_ned_m[2] - t[2]).abs(),
                (f.position_ned_m[0] - t[0]).hypot(f.position_ned_m[1] - t[1]),
            )
        });
        self.update_events(f, target.is_some() && alt_err > self.hold_limit_m);
        self.update_peaks_and_window(f, alt_err, h_err);
        if self.step.is_multiple_of(self.interval_steps) {
            let reserve = sim.thrust_reserve_ratio(f)?;
            self.peaks.min_reserve.update_min(reserve, f.t_s);
            self.window.min_reserve = self.window.min_reserve.min(reserve);
            self.write_row(f, reserve, h_err, alt_err)?;
        }
        self.step += 1;
        self.last = Some(*f);
        Ok(())
    }

    fn update_events(&mut self, f: &Frame, hold_lost: bool) {
        let t = &f.tel;
        let unstable = f.tilt_rad > INSTABILITY_TILT_RAD
            || f.rates_rps.iter().any(|r| r.abs() > INSTABILITY_RATE_RPS);
        let flags: [(&str, bool); 13] = [
            ("motor_over_current", t.motor_over_current_mask != 0),
            ("esc_over_current", t.esc_over_current_mask != 0),
            ("battery_over_current", t.battery_over_current != 0),
            ("motor_over_temp", t.motor_over_temp_mask != 0),
            ("esc_over_temp", t.esc_over_temp_mask != 0),
            ("battery_over_temp", t.battery_over_temp != 0),
            ("battery_cutoff", t.battery_cutoff != 0),
            ("prop_outside_table", t.prop_extrapolated != 0),
            ("axis_saturated", f.axis_railed),
            ("collective_limited", f.out.collective_limited),
            ("tilt_limit_active", f.out.tilt_limited),
            ("altitude_hold_lost", hold_lost),
            ("instability", unstable),
        ];
        for (name, now) in flags {
            if let Some(e) = self.event_mut(name) {
                e.update(now, f.t_s);
            }
        }
    }

    fn update_peaks_and_window(&mut self, f: &Frame, alt_err: f64, h_err: f64) {
        let t = &f.tel;
        let p = &mut self.peaks;
        p.winding_c.update_max(max4(&t.motor_winding_c), f.t_s);
        p.case_c.update_max(max4(&t.motor_case_c), f.t_s);
        p.esc_c.update_max(max4(&t.esc_temp_c), f.t_s);
        p.battery_c.update_max(t.battery_temp_c, f.t_s);
        p.motor_current_a
            .update_max(max4(&t.motor_current_a), f.t_s);
        p.pack_current_a.update_max(t.battery_current_a, f.t_s);
        p.min_pack_v.update_min(t.battery_voltage_v, f.t_s);
        p.tilt_rad.update_max(f.tilt_rad, f.t_s);
        let w = &mut self.window;
        w.steps += 1;
        w.max_alt_err_m = w.max_alt_err_m.max(alt_err);
        w.max_h_err_m = w.max_h_err_m.max(h_err);
        w.saturated_steps += u64::from(f.axis_railed || f.out.collective_limited);
        w.capped_steps += u64::from(f.out.throttle >= self.window_cap);
        w.max_tilt_rad = w.max_tilt_rad.max(f.tilt_rad);
    }

    fn write_row(
        &mut self,
        f: &Frame,
        reserve: f64,
        h_err: f64,
        alt_err: f64,
    ) -> Result<(), CampaignError> {
        use std::fmt::Write;
        let t = &f.tel;
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        let mut v: Vec<f64> = vec![f.t_s];
        v.extend(f.position_ned_m);
        v.extend(f.velocity_ned_mps);
        v.extend([
            f.tilt_rad.to_degrees(),
            f.yaw_rad.to_degrees(),
            f.roll_rad.to_degrees(),
            f.pitch_rad.to_degrees(),
        ]);
        v.extend(f.wind_ned_mps);
        v.extend(f.gust_ned_mps);
        v.extend([f.air_speed_mps, h_err, alt_err]);
        v.extend(t.motor_rpm);
        v.extend(t.motor_current_a);
        v.extend(t.motor_torque_nm);
        v.extend(t.motor_thrust_n);
        v.extend([
            reserve,
            t.battery_voltage_v,
            t.battery_current_a,
            t.battery_soc,
            t.battery_power_w,
            t.motor_copper_loss_w.iter().sum(),
            t.motor_noload_loss_w.iter().sum(),
            t.esc_loss_w.iter().sum(),
        ]);
        v.extend([
            max4(&t.motor_winding_c),
            max4(&t.motor_case_c),
            max4(&t.esc_temp_c),
            t.battery_temp_c,
        ]);
        v.extend([f.out.throttle, f.out.roll, f.out.pitch, f.out.yaw]);
        v.extend([
            f64::from(t.prop_extrapolated),
            b(f.out.collective_limited),
            b(f.axis_railed),
            b(f.out.tilt_limited),
            b(alt_err > self.hold_limit_m),
        ]);
        if v.len() != SERIES_COLUMNS.len() {
            return Err(CampaignError::Run("series column count"));
        }
        let cells: Option<Vec<String>> = v.iter().map(|x| crate::textout::num(*x)).collect();
        let cells = cells.ok_or(CampaignError::Run("non-finite value in the time series"))?;
        writeln!(self.csv, "{}", cells.join(",")).map_err(|_| CampaignError::Run("csv"))?;
        self.rows += 1;
        Ok(())
    }
}

// ----------------------------------------------------------------------------------------------- searches

/// Verdict of one evaluation inside a search.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Verdict {
    /// The evaluation passed.
    pub pass: bool,
    /// Criterion that failed (the first in priority order), if any.
    pub criterion: Option<&'static str>,
}

/// Result of a threshold search.
#[derive(Clone, Debug, PartialEq)]
pub enum Search {
    /// Even the upper bound passes: the limit is beyond the search range.
    AllPass,
    /// The lower bound already fails.
    FailsAtStart(&'static str),
    /// The largest passing value and the smallest failing value found, with the failing criterion.
    Found {
        /// Largest value that passed.
        pass: f64,
        /// Smallest value that failed.
        fail: f64,
        /// Criterion of the failing value.
        criterion: &'static str,
        /// Evaluations made.
        evaluations: u32,
    },
}

/// Hard bound on bisection iterations (the bracket halves each time).
const MAX_BISECTIONS: u32 = 60;

/// Bisection between a value known to pass (`lo`) and one known to fail (`hi`) until the bracket is `tol` wide.
/// Returns `(pass, fail, criterion, evaluations)`.
fn refine<E>(
    mut lo: f64,
    mut hi: f64,
    mut criterion: &'static str,
    tol: f64,
    eval: &mut impl FnMut(f64) -> Result<Verdict, E>,
) -> Result<(f64, f64, &'static str, u32), E> {
    let mut n = 0;
    while hi - lo > tol && n < MAX_BISECTIONS {
        let mid = 0.5 * (lo + hi);
        let v = eval(mid)?;
        n += 1;
        if v.pass {
            lo = mid;
        } else {
            hi = mid;
            criterion = v.criterion.unwrap_or(criterion);
        }
    }
    Ok((lo, hi, criterion, n))
}

/// Bisection search for the largest passing value in `[lo, hi]` of a function that passes at small values and fails at large ones.
///
/// # Errors
/// The first error of `eval`.
pub fn bisect<E>(
    lo: f64,
    hi: f64,
    tol: f64,
    mut eval: impl FnMut(f64) -> Result<Verdict, E>,
) -> Result<Search, E> {
    let first = eval(lo)?;
    if !first.pass {
        return Ok(Search::FailsAtStart(
            first.criterion.unwrap_or("unspecified"),
        ));
    }
    let last = eval(hi)?;
    if last.pass {
        return Ok(Search::AllPass);
    }
    let (pass, fail, criterion, n) = refine(
        lo,
        hi,
        last.criterion.unwrap_or("unspecified"),
        tol,
        &mut eval,
    )?;
    Ok(Search::Found {
        pass,
        fail,
        criterion,
        evaluations: n + 2,
    })
}

/// Ramp search: tries `start`, `start + step`, ... up to `max` and refines inside the first failing step down to `tol`.
///
/// # Errors
/// The first error of `eval`.
pub fn ramp_and_refine<E>(
    start: f64,
    step: f64,
    max: f64,
    tol: f64,
    mut eval: impl FnMut(f64) -> Result<Verdict, E>,
) -> Result<Search, E> {
    let mut previous: Option<f64> = None;
    let mut n = 0;
    let mut level = start;
    // number of ramp levels is bounded by (max - start) / step, a small whole number validated by the definitions
    while level <= max + 1e-9 && n < 10_000 {
        let v = eval(level)?;
        n += 1;
        if !v.pass {
            let criterion = v.criterion.unwrap_or("unspecified");
            return match previous {
                None => Ok(Search::FailsAtStart(criterion)),
                Some(lo) => {
                    let (pass, fail, crit, k) = refine(lo, level, criterion, tol, &mut eval)?;
                    Ok(Search::Found {
                        pass,
                        fail,
                        criterion: crit,
                        evaluations: n + k,
                    })
                }
            };
        }
        previous = Some(level);
        level += step;
    }
    Ok(Search::AllPass)
}

// ----------------------------------------------------------------------------------------------- test outputs

/// How a test ended.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// A breaking point was found.
    BreakingPointFound,
    /// The upper bound of the search passed: the limit is beyond the range.
    NotReachedWithinBound,
    /// The lowest value of the search already fails.
    FailsAtStart,
    /// The run reached its goal (climb target, battery cutoff, ...).
    Completed,
    /// The run ended at its time limit without reaching its goal.
    TimeLimit,
    /// Every temperature stopped changing.
    SteadyState,
    /// A component limit was crossed and the definition asked to stop there.
    LimitCrossed,
    /// The vehicle crashed or became unstable.
    Failed,
}

impl Outcome {
    /// Text used in the result files.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::BreakingPointFound => "BREAKING_POINT_FOUND",
            Self::NotReachedWithinBound => "NOT_REACHED_WITHIN_BOUND",
            Self::FailsAtStart => "FAILS_AT_START",
            Self::Completed => "COMPLETED",
            Self::TimeLimit => "TIME_LIMIT",
            Self::SteadyState => "STEADY_STATE",
            Self::LimitCrossed => "LIMIT_CROSSED",
            Self::Failed => "FAILED",
        }
    }
}

/// A breaking point with the criterion that triggered it and the resolution of the search.
#[derive(Clone, Debug, PartialEq)]
pub struct BreakingPoint {
    /// What was varied.
    pub quantity: &'static str,
    /// Unit of the values.
    pub unit: &'static str,
    /// Largest value that passed.
    pub last_pass: f64,
    /// Smallest value that failed.
    pub first_fail: f64,
    /// Criterion of the failure.
    pub criterion: String,
    /// Width of the final bracket (the resolution).
    pub resolution: f64,
    /// Evaluations made.
    pub evaluations: u32,
}

/// One time series file of a test.
#[derive(Clone, Debug, PartialEq)]
pub struct Series {
    /// Label in the file name (`timeseries_<label>.csv`).
    pub label: String,
    /// CSV text.
    pub csv: String,
    /// Data rows.
    pub rows: usize,
}

/// Everything one test produces.
#[derive(Clone, Debug)]
pub struct TestOutput {
    /// Test id.
    pub id: &'static str,
    /// Definition version used.
    pub version: String,
    /// How it ended.
    pub outcome: Outcome,
    /// Breaking point, for the search tests.
    pub breaking: Option<BreakingPoint>,
    /// Named numbers (units in the name).
    pub metrics: Vec<(String, f64)>,
    /// Remarks (limitations, assumptions) shown in the report.
    pub notes: Vec<String>,
    /// Events of the reported run.
    pub events: Vec<EventRecord>,
    /// Peaks of the reported run.
    pub peaks: Peaks,
    /// Time series.
    pub series: Vec<Series>,
    /// Steps simulated in all runs of the test.
    pub steps_total: u64,
    /// Turbulence height-clamp steps of the reported run (Dryden validity range).
    pub turbulence_clamped_steps: u64,
    /// Steps on which the gust was limited to keep the wind inside the core's range.
    pub wind_limited_steps: u64,
    /// Where the drag of the reported run came from.
    pub drag: DragStats,
}

fn series_of(label: &str, o: &Observer) -> Series {
    let (csv, rows) = o.csv();
    Series {
        label: label.to_string(),
        csv: csv.to_string(),
        rows,
    }
}

fn breaking(
    quantity: &'static str,
    unit: &'static str,
    s: &Search,
) -> (Outcome, Option<BreakingPoint>, Vec<String>) {
    match s {
        Search::AllPass => (
            Outcome::NotReachedWithinBound,
            None,
            vec![format!(
                "the largest {quantity} tried still passes: the limit is beyond the search range"
            )],
        ),
        Search::FailsAtStart(c) => (
            Outcome::FailsAtStart,
            None,
            vec![format!("the smallest {quantity} already fails ({c})")],
        ),
        Search::Found {
            pass,
            fail,
            criterion,
            evaluations,
        } => (
            Outcome::BreakingPointFound,
            Some(BreakingPoint {
                quantity,
                unit,
                last_pass: *pass,
                first_fail: *fail,
                criterion: (*criterion).to_string(),
                resolution: fail - pass,
                evaluations: *evaluations,
            }),
            Vec::new(),
        ),
    }
}

fn base_output(
    id: &'static str,
    version: &str,
    o: &Observer,
    clamped: u64,
    wind_limited: u64,
) -> TestOutput {
    TestOutput {
        id,
        version: version.to_string(),
        outcome: Outcome::Completed,
        breaking: None,
        metrics: Vec::new(),
        notes: Vec::new(),
        events: o.events.clone(),
        peaks: o.peaks.clone(),
        series: Vec::new(),
        steps_total: o.steps,
        turbulence_clamped_steps: clamped,
        wind_limited_steps: wind_limited,
        drag: DragStats {
            label: DRAG_MODEL.to_string(),
            surrogate_steps: 0,
            analytic_steps: 0,
            outside_envelope_steps: 0,
        },
    }
}

/// A step error that means the vehicle was lost (diverged, or reached the ground / fell out of the atmosphere model), not a bad input.
fn loss_criterion(e: &CampaignError) -> Option<&'static str> {
    match e {
        CampaignError::Model("diverged") => Some("instability"),
        CampaignError::Model("atmosphere" | "drag" | "ground") => Some("crash"),
        _ => None,
    }
}

/// Seconds to whole 2 ms steps (definitions are validated to be positive and modest).
fn steps_for(seconds: f64) -> u64 {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = (seconds / DT_S).round().max(0.0) as u64;
    n
}

fn hold_setpoint(altitude_m: f64) -> Setpoint {
    Setpoint::Position {
        ned_m: [0.0, 0.0, -altitude_m],
        yaw_rad: 0.0,
    }
}

// ----------------------------------------------------------------------------------------------- test 1: hover + payload

struct HoldRun {
    verdict: Verdict,
    obs: Observer,
    steps: u64,
    clamped: u64,
    wind_limited: u64,
    drag: DragStats,
}

fn hover_run(
    build: &Build,
    cond: &Conditions,
    def: &HoverPayloadDef,
    payload_kg: f64,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<HoldRun, CampaignError> {
    let mut c = cond.clone();
    c.payload_kg = payload_kg;
    let mut sim = Sim::with_drag(build, &c, def.start_altitude_m, seed_offset, None, drag()?)?;
    let mut obs = Observer::new(
        def.output_interval_s,
        def.altitude_error_limit_m,
        def.collective_cap,
    );
    let target = [0.0, 0.0, -def.start_altitude_m];
    let sp = hold_setpoint(def.start_altitude_m);
    let total = steps_for(def.settle_s + def.hold_s);
    let window_start = total.saturating_sub(steps_for(def.eval_window_s));
    for k in 0..total {
        if k == window_start {
            obs.start_window();
        }
        match sim.step(&sp) {
            Ok(f) => obs.observe(&sim, &f, Some(target))?,
            Err(e) => {
                let Some(criterion) = loss_criterion(&e) else {
                    return Err(e);
                };
                return Ok(HoldRun {
                    verdict: Verdict {
                        pass: false,
                        criterion: Some(criterion),
                    },
                    obs,
                    steps: k,
                    clamped: sim.turbulence.clamped_steps,
                    wind_limited: sim.wind_limited_steps,
                    drag: sim.drag_stats(),
                });
            }
        }
    }
    let w = &obs.window;
    let unstable = obs
        .events
        .iter()
        .any(|e| e.name == "instability" && e.first_t_s.is_some());
    let criterion = if unstable {
        Some("instability")
    } else if w.max_alt_err_m > def.altitude_error_limit_m {
        Some("altitude_hold")
    } else if w.min_reserve < def.min_thrust_reserve_ratio {
        Some("thrust_reserve")
    } else if w.capped_fraction() > def.collective_cap_fraction_limit {
        Some("collective_cap")
    } else {
        None
    };
    let clamped = sim.turbulence.clamped_steps;
    let wind_limited = sim.wind_limited_steps;
    Ok(HoldRun {
        verdict: Verdict {
            pass: criterion.is_none(),
            criterion,
        },
        obs,
        steps: total,
        clamped,
        wind_limited,
        drag: sim.drag_stats(),
    })
}

/// Test 1: the largest payload with which the vehicle still holds its altitude with enough thrust reserve.
///
/// # Errors
/// `Invalid` / `Model` / `Run` from the simulation.
pub fn run_hover_payload(
    build: &Build,
    cond: &Conditions,
    def: &HoverPayloadDef,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<TestOutput, CampaignError> {
    let mut steps_total = 0_u64;
    let mut last_pass: Option<(f64, HoldRun)> = None;
    let mut last_fail: Option<(f64, HoldRun)> = None;
    let search = bisect(0.0, def.payload_max_kg, def.tolerance_kg, |m| {
        let run = hover_run(build, cond, def, m, seed_offset, drag)?;
        steps_total += run.steps;
        let v = run.verdict;
        if v.pass {
            last_pass = Some((m, run));
        } else {
            last_fail = Some((m, run));
        }
        Ok::<_, CampaignError>(v)
    })?;
    let (outcome, bp, mut notes) = breaking("payload_mass", "kg", &search);
    let reported = last_pass
        .as_ref()
        .or(last_fail.as_ref())
        .ok_or(CampaignError::Run("no evaluation ran"))?;
    let mut out = base_output(
        "hover_payload",
        &def.version,
        &reported.1.obs,
        reported.1.clamped,
        reported.1.wind_limited,
    );
    out.drag = reported.1.drag.clone();
    out.outcome = outcome;
    out.breaking = bp;
    out.notes.append(&mut notes);
    out.steps_total = steps_total;
    if let Some((m, r)) = &last_pass {
        out.series
            .push(series_of(&format!("pass_{m:.3}kg"), &r.obs));
        let w = &r.obs.window;
        out.metrics.push(("passing_payload_kg".into(), *m));
        out.metrics
            .push(("window_max_alt_err_m".into(), w.max_alt_err_m));
        out.metrics
            .push(("window_min_thrust_reserve_ratio".into(), w.min_reserve));
        out.metrics.push((
            "window_collective_capped_percent".into(),
            100.0 * w.capped_fraction(),
        ));
    }
    if let Some((m, r)) = &last_fail {
        out.series
            .push(series_of(&format!("fail_{m:.3}kg"), &r.obs));
    }
    Ok(out)
}

// ----------------------------------------------------------------------------------------------- test 2: wind + gust

fn wind_run(
    build: &Build,
    cond: &Conditions,
    def: &WindGustDef,
    wind_mps: f64,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<HoldRun, CampaignError> {
    let mut c = cond.clone();
    c.wind_speed_mps = wind_mps;
    let mut sim = Sim::with_drag(build, &c, def.start_altitude_m, seed_offset, None, drag()?)?;
    let mut obs = Observer::new(def.output_interval_s, def.position_error_limit_m, 2.0);
    let target = [0.0, 0.0, -def.start_altitude_m];
    let sp = hold_setpoint(def.start_altitude_m);
    let total = steps_for(def.settle_s + def.hold_s);
    let window_start = total.saturating_sub(steps_for(def.eval_window_s));
    for k in 0..total {
        if k == window_start {
            obs.start_window();
        }
        match sim.step(&sp) {
            Ok(f) => obs.observe(&sim, &f, Some(target))?,
            Err(e) => {
                let Some(criterion) = loss_criterion(&e) else {
                    return Err(e);
                };
                return Ok(HoldRun {
                    verdict: Verdict {
                        pass: false,
                        criterion: Some(criterion),
                    },
                    obs,
                    steps: k,
                    clamped: sim.turbulence.clamped_steps,
                    wind_limited: sim.wind_limited_steps,
                    drag: sim.drag_stats(),
                });
            }
        }
    }
    let w = &obs.window;
    let unstable = obs
        .events
        .iter()
        .any(|e| e.name == "instability" && e.first_t_s.is_some());
    let criterion = if unstable {
        Some("instability")
    } else if w.max_h_err_m.max(w.max_alt_err_m) > def.position_error_limit_m {
        Some("position_error")
    } else if w.saturated_fraction() > def.saturation_fraction_limit {
        Some("saturation")
    } else {
        None
    };
    let (clamped, wind_limited) = (sim.turbulence.clamped_steps, sim.wind_limited_steps);
    Ok(HoldRun {
        verdict: Verdict {
            pass: criterion.is_none(),
            criterion,
        },
        obs,
        steps: total,
        clamped,
        wind_limited,
        drag: sim.drag_stats(),
    })
}

/// Test 2: the highest mean wind (with the typed turbulence) in which the vehicle holds its position.
///
/// # Errors
/// `Invalid` / `Model` / `Run` from the simulation.
pub fn run_wind_gust(
    build: &Build,
    cond: &Conditions,
    def: &WindGustDef,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<TestOutput, CampaignError> {
    let mut steps_total = 0_u64;
    let mut last_pass: Option<(f64, HoldRun)> = None;
    let mut last_fail: Option<(f64, HoldRun)> = None;
    let search = ramp_and_refine(
        def.start_wind_mps,
        def.step_mps,
        def.max_wind_mps,
        def.tolerance_mps,
        |w| {
            let run = wind_run(build, cond, def, w, seed_offset, drag)?;
            steps_total += run.steps;
            let v = run.verdict;
            if v.pass {
                last_pass = Some((w, run));
            } else {
                last_fail = Some((w, run));
            }
            Ok::<_, CampaignError>(v)
        },
    )?;
    let (outcome, bp, mut notes) = breaking("mean_wind_speed", "m/s", &search);
    let reported = last_pass
        .as_ref()
        .or(last_fail.as_ref())
        .ok_or(CampaignError::Run("no evaluation ran"))?;
    let mut out = base_output(
        "wind_gust",
        &def.version,
        &reported.1.obs,
        reported.1.clamped,
        reported.1.wind_limited,
    );
    out.drag = reported.1.drag.clone();
    out.outcome = outcome;
    out.breaking = bp;
    out.notes.append(&mut notes);
    out.steps_total = steps_total;
    if let Some((w, r)) = &last_pass {
        out.series
            .push(series_of(&format!("pass_{w:.2}mps"), &r.obs));
        out.metrics.push(("passing_wind_mps".into(), *w));
        out.metrics.push((
            "window_max_horizontal_error_m".into(),
            r.obs.window.max_h_err_m,
        ));
        out.metrics.push((
            "window_saturated_percent".into(),
            100.0 * r.obs.window.saturated_fraction(),
        ));
    }
    if let Some((w, r)) = &last_fail {
        out.series
            .push(series_of(&format!("fail_{w:.2}mps"), &r.obs));
    }
    out.notes.push(format!(
        "turbulence model: {}",
        turbulence_label(cond.turbulence)
    ));
    Ok(out)
}

/// The label that reports must show for a turbulence model.
#[must_use]
pub fn turbulence_label(kind: TurbulenceKind) -> &'static str {
    match kind {
        TurbulenceKind::None => "none",
        TurbulenceKind::Dryden => crate::turbulence::TURBULENCE_LABEL,
        TurbulenceKind::Simple => "simple filtered noise (fallback, ASSUMED)",
    }
}

// ----------------------------------------------------------------------------------------------- test 3: climb

/// Mean climb rate over the middle half of the climb time `t_total` (`speeds` are `(time, rate)` pairs).
#[must_use]
pub fn sustained_rate(speeds: &[(f64, f64)], t_total: f64) -> f64 {
    let mid: Vec<f64> = speeds
        .iter()
        .filter(|(ts, _)| *ts >= 0.25 * t_total && *ts <= 0.75 * t_total)
        .map(|(_, v)| *v)
        .collect();
    mid.iter().sum::<f64>() / f64::from(u32::try_from(mid.len().max(1)).unwrap_or(u32::MAX))
}

/// Test 3: climb performance from hover with the class limits replaced by the definition's.
///
/// # Errors
/// `Invalid` / `Model` / `Run` from the simulation.
pub fn run_climb(
    build: &Build,
    cond: &Conditions,
    def: &ClimbDef,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<TestOutput, CampaignError> {
    let mut ccfg = build.controller;
    ccfg.max_climb_mps = def.command_climb_mps;
    ccfg.max_accel_up_mps2 = def.accel_limit_mps2;
    let mut sim = Sim::with_drag(
        build,
        cond,
        def.start_altitude_m,
        seed_offset,
        Some(ccfg),
        drag()?,
    )?;
    let mut obs = Observer::new(def.output_interval_s, f64::INFINITY, 0.98);
    let hold = hold_setpoint(def.start_altitude_m);
    for _ in 0..steps_for(def.preroll_s) {
        let f = sim.step(&hold)?;
        obs.observe(&sim, &f, Some([0.0, 0.0, -def.start_altitude_m]))?;
    }
    obs.start_window();
    let climb_sp = Setpoint::Velocity {
        ned_mps: [0.0, 0.0, -def.command_climb_mps],
        yaw_rad: 0.0,
    };
    let t0 = sim.t_s;
    let goal = -def.start_altitude_m - def.target_height_m; // NED z of the target
    let mut speeds: Vec<(f64, f64)> = Vec::new();
    let mut reached: Option<f64> = None;
    let mut failed = false;
    while sim.t_s - t0 < def.time_limit_s {
        match sim.step(&climb_sp) {
            Ok(f) => {
                obs.observe(&sim, &f, None)?;
                speeds.push((f.t_s - t0, -f.velocity_ned_mps[2]));
                if f.position_ned_m[2] <= goal {
                    reached = Some(f.t_s - t0);
                    break;
                }
            }
            Err(e) if loss_criterion(&e).is_some() => {
                failed = true;
                break;
            }
            Err(e) => return Err(e),
        }
    }
    let mut out = base_output(
        "climb",
        &def.version,
        &obs,
        sim.turbulence.clamped_steps,
        sim.wind_limited_steps,
    );
    out.drag = sim.drag_stats();
    out.outcome = if failed {
        Outcome::Failed
    } else if reached.is_some() {
        Outcome::Completed
    } else {
        Outcome::TimeLimit
    };
    out.series.push(series_of("climb", &obs));
    if let Some(t) = reached {
        out.metrics.push(("time_to_target_s".into(), t));
        let mean = sustained_rate(&speeds, t);
        out.metrics.push(("sustained_climb_rate_mps".into(), mean));
    } else {
        out.notes
            .push("the target height was not reached within the time limit".into());
    }
    out.metrics.push((
        "max_climb_rate_mps".into(),
        speeds.iter().map(|(_, v)| *v).fold(0.0, f64::max),
    ));
    out.metrics.push((
        "climb_saturated_percent".into(),
        100.0 * obs.window.saturated_fraction(),
    ));
    out.metrics.push((
        "climb_min_thrust_reserve_ratio".into(),
        obs.window.min_reserve,
    ));
    out.notes.push(format!(
        "class climb / acceleration limits replaced for this test: {} m/s, {} m/s^2",
        def.command_climb_mps, def.accel_limit_mps2
    ));
    Ok(out)
}

// ----------------------------------------------------------------------------------------------- test 5: endurance

/// Distance to a waypoint closer than this counts as arrived, m.
const ARRIVAL_RADIUS_M: f64 = 2.0;
/// Speed command per metre of remaining distance, 1/s.
const APPROACH_GAIN_PER_S: f64 = 1.0;
/// Altitude hold gain of the loop guidance, 1/s.
const ALTITUDE_GAIN_PER_S: f64 = 0.8;
/// Largest vertical speed command of the loop guidance, m/s.
const ALTITUDE_SPEED_LIMIT_MPS: f64 = 1.0;

fn loop_setpoint(pos: [f64; 3], wp: [f64; 2], speed: f64, altitude_m: f64) -> (Setpoint, f64) {
    let (dn, de) = (wp[0] - pos[0], wp[1] - pos[1]);
    let dist = dn.hypot(de);
    let v = (APPROACH_GAIN_PER_S * dist).min(speed);
    let (vn, ve) = if dist > 1e-9 {
        (v * dn / dist, v * de / dist)
    } else {
        (0.0, 0.0)
    };
    let vd = (ALTITUDE_GAIN_PER_S * (-altitude_m - pos[2]))
        .clamp(-ALTITUDE_SPEED_LIMIT_MPS, ALTITUDE_SPEED_LIMIT_MPS);
    (
        Setpoint::Velocity {
            ned_mps: [vn, ve, vd],
            yaw_rad: 0.0,
        },
        dist,
    )
}

/// Test 5: flies the mission until the battery reaches its cutoff (or the time limit / a crash).
///
/// # Errors
/// `Invalid` / `Model` / `Run` from the simulation.
pub fn run_endurance(
    build: &Build,
    cond: &Conditions,
    def: &EnduranceDef,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<TestOutput, CampaignError> {
    let mut sim = Sim::with_drag(
        build,
        cond,
        def.cruise_altitude_m,
        seed_offset,
        None,
        drag()?,
    )?;
    let mut obs = Observer::new(def.output_interval_s, f64::INFINITY, 0.98);
    let hold = hold_setpoint(def.cruise_altitude_m);
    for _ in 0..steps_for(def.settle_s) {
        let f = sim.step(&hold)?;
        obs.observe(&sim, &f, None)?;
    }
    obs.start_window();
    let l = def.loop_side_m;
    let corners = [[l, 0.0], [l, l], [0.0, l], [0.0, 0.0]];
    let (mut idx, mut loops) = (0_usize, 0_u32);
    let (mut energy_ws, mut cutoff_s, mut cutoff_run_steps, mut distance_m) =
        (0.0, 0.0, 0_u64, 0.0);
    let t0 = sim.t_s;
    let mut end = Outcome::TimeLimit;
    while sim.t_s - t0 < def.max_duration_s {
        let pos = sim.state.position;
        let sp = if def.mission == "hover" {
            hold
        } else {
            let wp = corners.get(idx).copied().unwrap_or([0.0, 0.0]);
            let (sp, dist) = loop_setpoint(pos, wp, def.loop_speed_mps, def.cruise_altitude_m);
            if dist < ARRIVAL_RADIUS_M {
                idx = (idx + 1) % corners.len();
                loops += u32::from(idx == 0);
            }
            sp
        };
        match sim.step(&sp) {
            Ok(f) => {
                obs.observe(&sim, &f, None)?;
                energy_ws += f.tel.battery_power_w * DT_S;
                distance_m += f.velocity_ned_mps[0].hypot(f.velocity_ned_mps[1]) * DT_S;
                cutoff_run_steps = if f.tel.battery_cutoff != 0 {
                    cutoff_run_steps + 1
                } else {
                    0
                };
                cutoff_s = if f.tel.battery_cutoff != 0 {
                    cutoff_s + DT_S
                } else {
                    0.0
                };
                if cutoff_s >= def.cutoff_confirm_s {
                    end = Outcome::Completed;
                    break;
                }
                if -f.position_ned_m[2] < def.crash_altitude_m {
                    end = Outcome::Failed;
                    if let Some(e) = obs.events.iter_mut().find(|e| e.name == "instability") {
                        e.update(true, f.t_s);
                    }
                    break;
                }
            }
            Err(e) if loss_criterion(&e).is_some() => {
                end = Outcome::Failed;
                break;
            }
            Err(e) => return Err(e),
        }
    }
    let totals = EnduranceTotals {
        duration_s: sim.t_s - t0,
        energy_ws,
        distance_m,
        cutoff_run_steps,
        loops,
        end,
    };
    Ok(endurance_output(def, &obs, &sim, &totals))
}

/// What the endurance loop measured.
struct EnduranceTotals {
    duration_s: f64,
    energy_ws: f64,
    distance_m: f64,
    cutoff_run_steps: u64,
    loops: u32,
    end: Outcome,
}

fn endurance_output(
    def: &EnduranceDef,
    obs: &Observer,
    sim: &Sim,
    t: &EnduranceTotals,
) -> TestOutput {
    let mut out = base_output(
        "endurance",
        &def.version,
        obs,
        sim.turbulence.clamped_steps,
        sim.wind_limited_steps,
    );
    out.drag = sim.drag_stats();
    out.outcome = t.end;
    out.series.push(series_of("endurance", obs));
    let last_soc = obs.last.map_or(f64::NAN, |f| f.tel.battery_soc);
    let mean_power = if t.duration_s > 0.0 {
        t.energy_ws / t.duration_s
    } else {
        0.0
    };
    let cutoff_run_s = f64::from(u32::try_from(t.cutoff_run_steps).unwrap_or(u32::MAX)) * DT_S;
    out.metrics.push(("duration_s".into(), t.duration_s));
    out.metrics
        .push(("energy_used_wh".into(), t.energy_ws / 3600.0));
    out.metrics.push(("mean_power_w".into(), mean_power));
    out.metrics.push(("final_soc".into(), last_soc));
    out.metrics
        .push(("loops_completed".into(), f64::from(t.loops)));
    out.metrics.push(("distance_flown_m".into(), t.distance_m));
    out.metrics
        .push(("final_cutoff_run_s".into(), cutoff_run_s));
    let note = match t.end {
        Outcome::Completed => {
            "ended by battery cutoff (pack voltage below cells x cutoff, confirmed)"
        }
        Outcome::TimeLimit => "the battery did not reach cutoff within the time limit",
        _ => "the vehicle crashed or became unstable before the battery reached cutoff",
    };
    out.notes.push(note.into());
    out.notes.push("battery discharge curve and thermal inputs are ASSUMED: the endurance number is indicative".into());
    out
}

// ----------------------------------------------------------------------------------------------- test 6: thermal soak

/// Node temperatures tracked for the steady-state test: 4 windings, 4 cases, 4 ESCs, battery.
fn node_temps(f: &Frame) -> [f64; 13] {
    let t = &f.tel;
    let mut a = [0.0; 13];
    for (k, v) in t
        .motor_winding_c
        .iter()
        .chain(&t.motor_case_c)
        .chain(&t.esc_temp_c)
        .chain(std::iter::once(&t.battery_temp_c))
        .enumerate()
    {
        if let Some(x) = a.get_mut(k) {
            *x = *v;
        }
    }
    a
}

/// Test 6: hover at the set ambient until every temperature is steady, a limit is crossed, or the time limit.
///
/// # Errors
/// `Invalid` / `Model` / `Run` from the simulation.
pub fn run_thermal_soak(
    build: &Build,
    cond: &Conditions,
    def: &ThermalSoakDef,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<TestOutput, CampaignError> {
    let mut sim = Sim::with_drag(build, cond, def.altitude_m, seed_offset, None, drag()?)?;
    if def.freeze_soc {
        sim.freeze_soc();
    }
    let mut obs = Observer::new(def.output_interval_s, f64::INFINITY, 0.98);
    let hold = hold_setpoint(def.altitude_m);
    let per_second = steps_for(1.0);
    let window_s = steps_for(def.steady_window_s) / per_second.max(1);
    let mut history: Vec<[f64; 13]> = Vec::new();
    let mut end = Outcome::TimeLimit;
    let mut max_rate = f64::NAN;
    let total = steps_for(def.max_duration_s);
    for k in 0..total {
        match sim.step(&hold) {
            Ok(f) => {
                obs.observe(&sim, &f, None)?;
                if (k + 1) % per_second == 0 {
                    history.push(node_temps(&f));
                    let n = history.len();
                    if u64::try_from(n).unwrap_or(0) > window_s {
                        let now = history.last().copied().unwrap_or([0.0; 13]);
                        let then = history
                            .get(n - 1 - usize::try_from(window_s).unwrap_or(0))
                            .copied()
                            .unwrap_or([0.0; 13]);
                        let rate = now
                            .iter()
                            .zip(&then)
                            .map(|(a, b)| (a - b).abs())
                            .fold(0.0, f64::max)
                            / (def.steady_window_s / 60.0);
                        max_rate = rate;
                        if rate < def.steady_rate_k_per_min {
                            end = Outcome::SteadyState;
                            break;
                        }
                    }
                }
                let crossed = obs.events.iter().any(|e| {
                    e.first_t_s.is_some()
                        && matches!(
                            e.name,
                            "motor_over_temp" | "esc_over_temp" | "battery_over_temp"
                        )
                });
                if def.stop_on_first_limit && crossed {
                    end = Outcome::LimitCrossed;
                    break;
                }
            }
            Err(e) if loss_criterion(&e).is_some() => {
                end = Outcome::Failed;
                break;
            }
            Err(e) => return Err(e),
        }
    }
    Ok(thermal_output(def, &obs, &sim, end, max_rate))
}

/// Result of a thermal soak: limits and times (-1 = never crossed), peaks, final rate, duration.
fn thermal_output(
    def: &ThermalSoakDef,
    obs: &Observer,
    sim: &Sim,
    end: Outcome,
    max_rate: f64,
) -> TestOutput {
    let mut out = base_output(
        "thermal_soak",
        &def.version,
        obs,
        sim.turbulence.clamped_steps,
        sim.wind_limited_steps,
    );
    out.drag = sim.drag_stats();
    out.outcome = end;
    out.series.push(series_of("thermal_soak", obs));
    let limits = sim.temperature_limits_c();
    for (name, limit, ev) in [
        ("winding", limits[0], "motor_over_temp"),
        ("esc", limits[1], "esc_over_temp"),
        ("battery", limits[2], "battery_over_temp"),
    ] {
        out.metrics.push((format!("{name}_limit_c"), limit));
        let t = out
            .events
            .iter()
            .find(|e| e.name == ev)
            .and_then(|e| e.first_t_s);
        out.metrics
            .push((format!("time_to_{name}_limit_s"), t.unwrap_or(-1.0)));
    }
    out.metrics
        .push(("peak_winding_c".into(), obs.peaks.winding_c.value));
    out.metrics
        .push(("peak_winding_time_s".into(), obs.peaks.winding_c.t_s));
    out.metrics
        .push(("peak_esc_c".into(), obs.peaks.esc_c.value));
    out.metrics
        .push(("peak_battery_c".into(), obs.peaks.battery_c.value));
    out.metrics.push((
        "final_max_temperature_rate_k_per_min".into(),
        if max_rate.is_finite() { max_rate } else { -1.0 },
    ));
    out.metrics.push(("duration_s".into(), sim.t_s));
    out.notes
        .push("a time of -1 means the limit was never crossed".into());
    if def.freeze_soc {
        out.notes.push("state of charge held constant to isolate the thermal behaviour (see endurance for discharge)".into());
    }
    out
}

// ----------------------------------------------------------------------------------------------- the campaign

// ----------------------------------------------------------------------------------------------- tests 6 to 8: speed, FPV sprint, FPV punch-out

/// What every speed result says about its drag model. The legacy analytic drag loads the body Z axis when the airframe is pitched
/// (DEMO-PROGRESS, P3c), so forward speed and the altitude held in forward flight are under-predicted; a result is not a vehicle limit.
const SPEED_DRAG_NOTE: &str = "speed results use the legacy analytic drag, which is known to under-predict forward speed and to sink when pitched (the FPV sprint reaches about 20 of 35 m/s): not a vehicle limit";

fn drag_note(label: &str) -> String {
    if label == DRAG_MODEL {
        SPEED_DRAG_NOTE.to_string()
    } else {
        format!(
            "speed results use the drag model {label}: preliminary CFD, unvalidated, force only"
        )
    }
}

/// Verdict of one `max_speed` level from the window numbers. Criterion priority: instability, altitude error, thrust reserve, speed shortfall.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn speed_verdict(
    unstable: bool,
    mean_speed_mps: f64,
    command_mps: f64,
    speed_fraction_min: f64,
    max_alt_err_m: f64,
    alt_err_limit_m: f64,
    min_reserve: f64,
    reserve_limit: f64,
) -> Verdict {
    let criterion = if unstable {
        Some("instability")
    } else if max_alt_err_m > alt_err_limit_m {
        Some("altitude_error")
    } else if min_reserve < reserve_limit {
        Some("thrust_reserve")
    } else if mean_speed_mps < speed_fraction_min * command_mps {
        Some("speed_shortfall")
    } else {
        None
    };
    Verdict {
        pass: criterion.is_none(),
        criterion,
    }
}

/// First time at which the speed reaches `threshold`: the time of the first sample at or above it (no interpolation); -1 if it never does.
#[must_use]
pub fn time_to_speed(samples: &[(f64, f64)], threshold: f64) -> f64 {
    samples
        .iter()
        .find(|(_, v)| *v >= threshold)
        .map_or(-1.0, |(t, _)| *t)
}

/// Mean of the values whose time lies within `span_s` of the last sample's time (0 for no samples).
#[must_use]
pub fn terminal_mean(samples: &[(f64, f64)], span_s: f64) -> f64 {
    let Some(&(t_end, _)) = samples.last() else {
        return 0.0;
    };
    let tail: Vec<f64> = samples
        .iter()
        .filter(|(t, _)| *t >= t_end - span_s)
        .map(|(_, v)| *v)
        .collect();
    tail.iter().sum::<f64>() / f64::from(u32::try_from(tail.len().max(1)).unwrap_or(u32::MAX))
}

/// Ramp step that makes the LAST level land exactly on `upper`: `(upper - start) / n` with `n = ceil((upper - start) / step)`, at least one step.
/// Without it a ramp could stop short of the cap (levels 2, 5, 8, 11 for a cap of 12) and a "limit-bound" result would name a speed that was never flown.
#[must_use]
pub fn ramp_step(start: f64, step: f64, upper: f64) -> f64 {
    let span = upper - start;
    if !(span > 0.0 && step > 0.0) {
        return step;
    }
    let n = (span / step - 1e-9).ceil().max(1.0);
    span / n
}

struct SpeedRun {
    hold: HoldRun,
    mean_speed_mps: f64,
}

fn speed_run(
    build: &Build,
    cond: &Conditions,
    def: &MaxSpeedDef,
    command_mps: f64,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<SpeedRun, CampaignError> {
    let mut sim = Sim::with_drag(
        build,
        cond,
        def.start_altitude_m,
        seed_offset,
        None,
        drag()?,
    )?;
    let mut obs = Observer::new(def.output_interval_s, def.altitude_error_limit_m, 0.98);
    let sp = Setpoint::Velocity {
        ned_mps: [command_mps, 0.0, 0.0],
        yaw_rad: 0.0,
    };
    let total = steps_for(def.settle_s + def.hold_s);
    let window_start = total.saturating_sub(steps_for(def.eval_window_s));
    let (mut speed_sum, mut window_steps) = (0.0_f64, 0_u64);
    for k in 0..total {
        if k == window_start {
            obs.start_window();
        }
        match sim.step(&sp) {
            Ok(f) => {
                // only the altitude error is judged: the horizontal target follows the vehicle
                let target = [
                    f.position_ned_m[0],
                    f.position_ned_m[1],
                    -def.start_altitude_m,
                ];
                obs.observe(&sim, &f, Some(target))?;
                if k >= window_start {
                    speed_sum += f.velocity_ned_mps[0].hypot(f.velocity_ned_mps[1]);
                    window_steps += 1;
                }
            }
            Err(e) => {
                let Some(criterion) = loss_criterion(&e) else {
                    return Err(e);
                };
                return Ok(SpeedRun {
                    hold: HoldRun {
                        verdict: Verdict {
                            pass: false,
                            criterion: Some(criterion),
                        },
                        obs,
                        steps: k,
                        clamped: sim.turbulence.clamped_steps,
                        wind_limited: sim.wind_limited_steps,
                        drag: sim.drag_stats(),
                    },
                    mean_speed_mps: 0.0,
                });
            }
        }
    }
    let mean = speed_sum / f64::from(u32::try_from(window_steps.max(1)).unwrap_or(u32::MAX));
    let unstable = obs
        .events
        .iter()
        .any(|e| e.name == "instability" && e.first_t_s.is_some());
    let w = &obs.window;
    let verdict = speed_verdict(
        unstable,
        mean,
        command_mps,
        def.speed_fraction_min,
        w.max_alt_err_m,
        def.altitude_error_limit_m,
        w.min_reserve,
        def.min_thrust_reserve_ratio,
    );
    let (clamped, wind_limited, drag_stats) = (
        sim.turbulence.clamped_steps,
        sim.wind_limited_steps,
        sim.drag_stats(),
    );
    Ok(SpeedRun {
        hold: HoldRun {
            verdict,
            obs,
            steps: total,
            clamped,
            wind_limited,
            drag: drag_stats,
        },
        mean_speed_mps: mean,
    })
}

/// Test 6: maximum forward speed. The commanded speed is stepped up from `start_speed_mps` to `min(max_speed_mps, the class speed limit)`
/// (the step is adjusted so the last level is exactly that cap, see `ramp_step`);
/// the largest level that still passes is the result. If every level up to the class limit passes the result is LIMIT-BOUND (the class
/// limit, not the vehicle, ended the search) and says so.
///
/// # Errors
/// `Invalid` / `Model` / `Run` from the simulation.
pub fn run_max_speed(
    build: &Build,
    cond: &Conditions,
    def: &MaxSpeedDef,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<TestOutput, CampaignError> {
    let class_limit = build.controller.max_speed_h_mps;
    let upper = def.max_speed_mps.min(class_limit).max(def.start_speed_mps);
    // the last level must land exactly on `upper`, or a LIMIT-BOUND note would name a speed that was never flown
    let step_mps = ramp_step(def.start_speed_mps, def.step_mps, upper);
    let mut steps_total = 0_u64;
    let mut last_pass: Option<(f64, SpeedRun)> = None;
    let mut last_fail: Option<(f64, SpeedRun)> = None;
    let search = ramp_and_refine(
        def.start_speed_mps,
        step_mps,
        upper,
        def.tolerance_mps,
        |v| {
            let run = speed_run(build, cond, def, v, seed_offset, drag)?;
            steps_total += run.hold.steps;
            let verdict = run.hold.verdict;
            if verdict.pass {
                last_pass = Some((v, run));
            } else {
                last_fail = Some((v, run));
            }
            Ok::<_, CampaignError>(verdict)
        },
    )?;
    let (outcome, bp, mut notes) = breaking("commanded_speed", "m/s", &search);
    let reported = last_pass
        .as_ref()
        .or(last_fail.as_ref())
        .ok_or(CampaignError::Run("no evaluation ran"))?;
    let mut out = base_output(
        "max_speed",
        &def.version,
        &reported.1.hold.obs,
        reported.1.hold.clamped,
        reported.1.hold.wind_limited,
    );
    out.drag = reported.1.hold.drag.clone();
    out.outcome = outcome;
    out.breaking = bp;
    out.notes.append(&mut notes);
    out.steps_total = steps_total;
    if outcome == Outcome::NotReachedWithinBound {
        out.notes.push(format!("LIMIT-BOUND: every level up to {upper} m/s (the lower of the definition's maximum and the class speed limit {class_limit} m/s) passed; the vehicle was not the limit"));
    }
    if let Some((v, r)) = &last_pass {
        out.series
            .push(series_of(&format!("pass_{v:.2}mps"), &r.hold.obs));
        out.metrics.push(("max_passing_speed_mps".into(), *v));
        out.metrics
            .push(("window_mean_speed_mps".into(), r.mean_speed_mps));
        out.metrics.push((
            "window_max_altitude_error_m".into(),
            r.hold.obs.window.max_alt_err_m,
        ));
        out.metrics.push((
            "window_min_thrust_reserve_ratio".into(),
            r.hold.obs.window.min_reserve,
        ));
    }
    if let Some((v, r)) = &last_fail {
        out.series
            .push(series_of(&format!("fail_{v:.2}mps"), &r.hold.obs));
    }
    out.metrics
        .push(("class_speed_limit_mps".into(), class_limit));
    out.notes.push(drag_note(&out.drag.label));
    Ok(out)
}

/// One flown manoeuvre: the observer, the (time, horizontal speed), (time, climb rate) and (time, height above the start) samples taken
/// after the hover pre-roll, and whether the vehicle was lost.
struct Manoeuvre {
    obs: Observer,
    speeds: Vec<(f64, f64)>,
    climbs: Vec<(f64, f64)>,
    gains: Vec<(f64, f64)>,
    lost: bool,
    sim_clamped: u64,
    sim_wind_limited: u64,
    drag: DragStats,
}

#[allow(clippy::too_many_arguments)]
fn fly_manoeuvre(
    build: &Build,
    cond: &Conditions,
    start_altitude_m: f64,
    preroll_s: f64,
    duration_s: f64,
    interval_s: f64,
    setpoint: &Setpoint,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<Manoeuvre, CampaignError> {
    let mut sim = Sim::with_drag(build, cond, start_altitude_m, seed_offset, None, drag()?)?;
    let mut obs = Observer::new(interval_s, f64::INFINITY, 0.98);
    let hold = hold_setpoint(start_altitude_m);
    for _ in 0..steps_for(preroll_s) {
        let f = sim.step(&hold)?;
        obs.observe(&sim, &f, Some([0.0, 0.0, -start_altitude_m]))?;
    }
    obs.start_window();
    let t0 = sim.t_s;
    let (mut speeds, mut climbs, mut gains) = (Vec::new(), Vec::new(), Vec::new());
    let mut lost = false;
    while sim.t_s - t0 < duration_s {
        match sim.step(setpoint) {
            Ok(f) => {
                obs.observe(&sim, &f, None)?;
                let t = f.t_s - t0;
                speeds.push((t, f.velocity_ned_mps[0].hypot(f.velocity_ned_mps[1])));
                climbs.push((t, -f.velocity_ned_mps[2]));
                gains.push((t, -f.position_ned_m[2] - start_altitude_m));
            }
            Err(e) if loss_criterion(&e).is_some() => {
                lost = true;
                break;
            }
            Err(e) => return Err(e),
        }
    }
    Ok(Manoeuvre {
        obs,
        speeds,
        climbs,
        gains,
        lost,
        sim_clamped: sim.turbulence.clamped_steps,
        sim_wind_limited: sim.wind_limited_steps,
        drag: sim.drag_stats(),
    })
}

fn manoeuvre_output(
    id: &'static str,
    version: &str,
    label: &str,
    m: Manoeuvre,
) -> (TestOutput, Manoeuvre) {
    let mut out = base_output(id, version, &m.obs, m.sim_clamped, m.sim_wind_limited);
    out.drag = m.drag.clone();
    out.outcome = if m.lost {
        Outcome::Failed
    } else {
        Outcome::Completed
    };
    out.series.push(series_of(label, &m.obs));
    out.metrics.push((
        "peak_tilt_deg".into(),
        m.obs.peaks.tilt_rad.value.to_degrees(),
    ));
    if m.lost {
        out.notes.push(
            "the vehicle was lost (diverged or hit the ground) before the manoeuvre ended".into(),
        );
    }
    out.notes.push(drag_note(&out.drag.label));
    (out, m)
}

/// Test 7: a straight sprint from hover with the class's horizontal speed demand for `sprint_s`.
///
/// # Errors
/// `Invalid` / `Model` / `Run` from the simulation.
pub fn run_fpv_sprint(
    build: &Build,
    cond: &Conditions,
    def: &FpvSprintDef,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<TestOutput, CampaignError> {
    let demand = build.controller.max_speed_h_mps;
    let sp = Setpoint::Velocity {
        ned_mps: [demand, 0.0, 0.0],
        yaw_rad: 0.0,
    };
    let m = fly_manoeuvre(
        build,
        cond,
        def.start_altitude_m,
        def.preroll_s,
        def.sprint_s,
        def.output_interval_s,
        &sp,
        seed_offset,
        drag,
    )?;
    let (mut out, m) = manoeuvre_output("fpv_sprint", &def.version, "sprint", m);
    out.metrics.push(("speed_demand_mps".into(), demand));
    out.metrics
        .push(("terminal_speed_mps".into(), terminal_mean(&m.speeds, 1.0)));
    out.metrics.push((
        "peak_speed_mps".into(),
        m.speeds.iter().map(|(_, v)| *v).fold(0.0, f64::max),
    ));
    for threshold in [10.0, 20.0, 30.0] {
        out.metrics.push((
            format!("time_to_{threshold:.0}_mps_s"),
            time_to_speed(&m.speeds, threshold),
        ));
    }
    let lowest = m.gains.iter().map(|(_, g)| *g).fold(0.0, f64::min);
    out.metrics.push(("altitude_sink_m".into(), -lowest));
    out.notes
        .push("a time of -1 means the speed was never reached".into());
    out.notes.push(format!(
        "the demand is the class horizontal speed limit ({demand} m/s) in velocity mode"
    ));
    Ok(out)
}

/// Test 8: full throttle straight up from hover (acro mode, sticks centred) for `punchout_s`.
///
/// # Errors
/// `Invalid` / `Model` / `Run` from the simulation.
pub fn run_fpv_punchout(
    build: &Build,
    cond: &Conditions,
    def: &FpvPunchoutDef,
    seed_offset: u64,
    drag: &DragFactory,
) -> Result<TestOutput, CampaignError> {
    let sp = Setpoint::Acro {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        throttle: 1.0,
    };
    let m = fly_manoeuvre(
        build,
        cond,
        def.start_altitude_m,
        def.preroll_s,
        def.punchout_s,
        def.output_interval_s,
        &sp,
        seed_offset,
        drag,
    )?;
    let (mut out, m) = manoeuvre_output("fpv_punchout", &def.version, "punchout", m);
    out.metrics.push((
        "final_climb_rate_mps".into(),
        terminal_mean(&m.climbs, 0.25),
    ));
    out.metrics.push((
        "peak_climb_rate_mps".into(),
        m.climbs.iter().map(|(_, v)| *v).fold(0.0, f64::max),
    ));
    out.metrics.push((
        "altitude_gain_m".into(),
        m.gains.iter().map(|(_, g)| *g).fold(0.0, f64::max),
    ));
    let reached = m
        .gains
        .iter()
        .find(|(_, g)| *g >= def.gain_target_m)
        .map_or(-1.0, |(t, _)| *t);
    out.metrics.push(("time_to_gain_target_s".into(), reached));
    out.notes.push(format!(
        "a time of -1 means the {} m gain was never reached",
        def.gain_target_m
    ));
    out.notes.push("full throttle in acro mode with the sticks centred: attitude is held only by the rate loop, so a small tilt drift is expected and reported".into());
    Ok(out)
}

/// The tests of this phase, in report order.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TestId {
    /// Hover + payload limit.
    HoverPayload,
    /// Wind + gust tolerance.
    WindGust,
    /// Climb performance.
    Climb,
    /// Endurance.
    Endurance,
    /// Thermal soak.
    ThermalSoak,
    /// Maximum forward speed (ramp search).
    MaxSpeed,
    /// FPV sprint from hover.
    FpvSprint,
    /// FPV full-throttle punch-out from hover.
    FpvPunchout,
}

impl TestId {
    /// All tests in report order.
    pub const ALL: [Self; 8] = [
        Self::HoverPayload,
        Self::WindGust,
        Self::Climb,
        Self::Endurance,
        Self::ThermalSoak,
        Self::MaxSpeed,
        Self::FpvSprint,
        Self::FpvPunchout,
    ];

    /// File / report name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::HoverPayload => "hover_payload",
            Self::WindGust => "wind_gust",
            Self::Climb => "climb",
            Self::Endurance => "endurance",
            Self::ThermalSoak => "thermal_soak",
            Self::MaxSpeed => "max_speed",
            Self::FpvSprint => "fpv_sprint",
            Self::FpvPunchout => "fpv_punchout",
        }
    }

    /// Parses a name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|t| t.name() == name)
    }

    /// Seed offset of the test: fixed per test, so the turbulence never depends on which tests run or on the thread count.
    const fn seed_offset(self) -> u64 {
        match self {
            Self::HoverPayload => 1000,
            Self::WindGust => 2000,
            Self::Climb => 3000,
            Self::Endurance => 4000,
            Self::ThermalSoak => 5000,
            Self::MaxSpeed => 6000,
            Self::FpvSprint => 7000,
            Self::FpvPunchout => 8000,
        }
    }
}

/// Everything a campaign needs.
pub struct CampaignInputs<'a> {
    /// The built vehicle (vehicle, powertrain and controller configs).
    pub build: &'a Build,
    /// Build name (manifest).
    pub build_name: &'a str,
    /// `(role, part id)` pairs (manifest).
    pub parts: Vec<(String, String)>,
    /// Parts database version (manifest).
    pub db_version: &'a str,
    /// Git hash of the source tree, given by the caller (manifest); never read by the core.
    pub git_hash: &'a str,
    /// Conditions.
    pub conditions: &'a Conditions,
    /// Test definitions.
    pub definitions: &'a Definitions,
    /// Tests to run.
    pub tests: Vec<TestId>,
    /// Where each simulation gets its drag from; `None` = the analytic model.
    pub drag: Option<&'a DragFactory>,
    /// What the CFD surrogate did (written to the result files only when `Some`); `None` for a plain analytic campaign.
    pub cfd_surrogate: Option<&'a CfdNote>,
}

/// Result of one test: its output or the error that stopped it.
pub type TestResult = Result<TestOutput, CampaignError>;

fn run_test(inputs: &CampaignInputs<'_>, id: TestId) -> TestResult {
    let (b, c, d, o) = (
        inputs.build,
        inputs.conditions,
        inputs.definitions,
        id.seed_offset(),
    );
    let drag: &DragFactory = inputs.drag.unwrap_or(&analytic_drag);
    match id {
        TestId::HoverPayload => run_hover_payload(b, c, &d.hover_payload, o, drag),
        TestId::WindGust => run_wind_gust(b, c, &d.wind_gust, o, drag),
        TestId::Climb => run_climb(b, c, &d.climb, o, drag),
        TestId::Endurance => run_endurance(b, c, &d.endurance, o, drag),
        TestId::ThermalSoak => run_thermal_soak(b, c, &d.thermal_soak, o, drag),
        TestId::MaxSpeed => run_max_speed(b, c, &d.max_speed, o, drag),
        TestId::FpvSprint => run_fpv_sprint(b, c, &d.fpv_sprint, o, drag),
        TestId::FpvPunchout => run_fpv_punchout(b, c, &d.fpv_punchout, o, drag),
    }
}

/// Runs the selected tests, in parallel on up to `threads` threads. The results are in the order of `inputs.tests` whatever the thread count.
///
/// # Errors
/// `Run` if a worker thread panicked or the result list could not be locked.
pub fn run_campaign(
    inputs: &CampaignInputs<'_>,
    threads: usize,
) -> Result<Vec<TestResult>, CampaignError> {
    run_campaign_with(inputs, threads, &|_, _| {})
}

/// As `run_campaign`, and `on_done(position, results_so_far)` is called (serialised, under the result lock) each time a test has
/// finished; `results_so_far[i]` is `Some` for every finished position `i`. The hook only observes: results do not depend on it.
///
/// # Errors
/// As `run_campaign`.
pub fn run_campaign_with(
    inputs: &CampaignInputs<'_>,
    threads: usize,
    on_done: &(dyn Fn(usize, &[Option<TestResult>]) + Sync),
) -> Result<Vec<TestResult>, CampaignError> {
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicUsize, Ordering};
    let n = inputs.tests.len();
    let slots: Mutex<Vec<Option<TestResult>>> = Mutex::new((0..n).map(|_| None).collect());
    let next = AtomicUsize::new(0);
    let workers = threads.clamp(1, n.max(1));
    std::thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::SeqCst);
                    let Some(id) = inputs.tests.get(i).copied() else {
                        break;
                    };
                    crate::progress::set_state(i, crate::progress::state_running());
                    crate::progress::set_current(i);
                    let r = run_test(inputs, id);
                    crate::progress::set_current(crate::progress::no_test());
                    crate::progress::set_state(
                        i,
                        if r.is_ok() {
                            crate::progress::state_done()
                        } else {
                            crate::progress::state_failed()
                        },
                    );
                    if let Ok(mut g) = slots.lock()
                        && let Some(slot) = g.get_mut(i)
                    {
                        *slot = Some(r);
                        on_done(i, &g);
                    }
                }
            });
        }
    });
    let done = slots
        .into_inner()
        .map_err(|_| CampaignError::Run("result list poisoned"))?;
    done.into_iter()
        .map(|r| r.ok_or(CampaignError::Run("a test did not finish")))
        .collect()
}

// ----------------------------------------------------------------------------------------------- output files

fn jn(v: f64) -> String {
    crate::textout::num(v).unwrap_or_else(|| "null".to_string())
}

fn manifest_json(
    inputs: &CampaignInputs<'_>,
    test: &str,
    version: &str,
    definition_json: &str,
    drag_label: &str,
) -> String {
    use crate::textout::json_str;
    let c = inputs.conditions;
    let parts = inputs
        .parts
        .iter()
        .map(|(r, i)| {
            format!(
                "{{\"role\": {}, \"part_id\": {}}}",
                json_str(r),
                json_str(i)
            )
        })
        .collect::<Vec<_>>()
        .join(", ");
    let payload = c
        .payload_position_m
        .iter()
        .map(|v| jn(*v))
        .collect::<Vec<_>>()
        .join(", ");
    let cfd_extra = cfd_extra(inputs.cfd_surrogate);
    // only a NAMED condition set adds the key, so unnamed runs keep their P4 bytes
    let set_name = c.name.as_ref().map_or(String::new(), |n| {
        format!("\"condition_set\": {}, ", json_str(n))
    });
    format!(
        "{{\n  \"schema_version\": {SCHEMA_VERSION},\n  \"test\": {},\n  \"definition_version\": {},\n  \"definitions_set_version\": {},\n  \"definition\": {definition_json},\n  \"build\": {},\n  \"parts\": [{parts}],\n  \"parts_db_version\": {},\n  \"git_hash\": {},\n  \"core_interface_version\": {},\n  \"controller_config_version\": {},\n  \"step_s\": {},\n  \"drag_model\": {}{cfd_extra},\n  \"turbulence\": {},\n  \"conditions\": {{{set_name}\"ambient_c\": {}, \"site_altitude_m\": {}, \"temp_offset_c\": {}, \"wind_speed_mps\": {}, \"wind_from_deg\": {}, \"turbulence\": {}, \"gust_level\": {}, \"seed\": {}, \"payload_kg\": {}, \"payload_position_m\": [{payload}], \"battery_soc\": {}, \"battery_temp_c\": {}}}\n}}\n",
        json_str(test),
        json_str(version),
        json_str(&inputs.definitions.definitions_version),
        json_str(inputs.build_name),
        json_str(inputs.db_version),
        json_str(inputs.git_hash),
        crate::ffi_get_interface_version(),
        inputs.build.controller.config_version,
        jn(DT_S),
        json_str(drag_label),
        json_str(turbulence_label(c.turbulence)),
        jn(c.ambient_c),
        jn(c.site_altitude_m),
        jn(c.temp_offset_c()),
        jn(c.wind_speed_mps),
        jn(c.wind_from_deg),
        json_str(match c.turbulence {
            TurbulenceKind::None => "none",
            TurbulenceKind::Dryden => "dryden",
            TurbulenceKind::Simple => "simple",
        }),
        jn(c.gust_level),
        c.seed,
        jn(c.payload_kg),
        jn(c.battery_soc),
        c.battery_temp_c.map_or("null".to_string(), jn),
    )
}

fn events_json(o: &TestOutput) -> String {
    use crate::textout::json_str;
    let ev = o
        .events
        .iter()
        .map(|e| {
            format!(
                "    {{\"name\": {}, \"first_t_s\": {}, \"last_t_s\": {}, \"active_steps\": {}, \"activations\": {}}}",
                json_str(e.name),
                e.first_t_s.map_or("null".to_string(), jn),
                e.first_t_s.map_or("null".to_string(), |_| jn(e.last_t_s)),
                e.active_steps,
                e.activations
            )
        })
        .collect::<Vec<_>>()
        .join(",\n");
    let pk = |name: &str, p: &Peak| {
        format!(
            "    {{\"name\": {}, \"value\": {}, \"t_s\": {}}}",
            json_str(name),
            jn(p.value),
            jn(p.t_s)
        )
    };
    let p = &o.peaks;
    let peaks = [
        pk("winding_c", &p.winding_c),
        pk("case_c", &p.case_c),
        pk("esc_c", &p.esc_c),
        pk("battery_c", &p.battery_c),
        pk("motor_current_a", &p.motor_current_a),
        pk("pack_current_a", &p.pack_current_a),
        pk("min_pack_v", &p.min_pack_v),
        pk("tilt_rad", &p.tilt_rad),
        pk("min_thrust_reserve_ratio", &p.min_reserve),
    ]
    .join(",\n");
    format!("{{\n  \"events\": [\n{ev}\n  ],\n  \"peaks\": [\n{peaks}\n  ]\n}}\n")
}

fn result_json(o: &TestOutput, cond: &Conditions) -> String {
    use crate::textout::json_str;
    let bp = o.breaking.as_ref().map_or("null".to_string(), |b| {
        format!(
            "{{\"quantity\": {}, \"unit\": {}, \"last_pass\": {}, \"first_fail\": {}, \"criterion\": {}, \"resolution\": {}, \"evaluations\": {}}}",
            json_str(b.quantity), json_str(b.unit), jn(b.last_pass), jn(b.first_fail), json_str(&b.criterion), jn(b.resolution), b.evaluations
        )
    });
    let metrics = o
        .metrics
        .iter()
        .map(|(k, v)| format!("{{\"name\": {}, \"value\": {}}}", json_str(k), jn(*v)))
        .collect::<Vec<_>>()
        .join(", ");
    let notes = o
        .notes
        .iter()
        .map(|n| json_str(n))
        .collect::<Vec<_>>()
        .join(", ");
    let series = o
        .series
        .iter()
        .map(|s| {
            format!(
                "{{\"file\": {}, \"rows\": {}}}",
                json_str(&format!("timeseries_{}.csv", s.label)),
                s.rows
            )
        })
        .collect::<Vec<_>>()
        .join(", ");
    // the surrogate bookkeeping appears only when a surrogate was in use, so analytic runs keep their exact bytes
    let drag_extra = if o.drag.label == DRAG_MODEL && o.drag.surrogate_steps == 0 {
        String::new()
    } else {
        format!(
            ",\n  \"drag_steps\": {{\"surrogate\": {}, \"analytic_fallback\": {}, \"outside_validated_envelope\": {}}},\n  \"drag_note\": \"preliminary CFD, unvalidated\"",
            o.drag.surrogate_steps, o.drag.analytic_steps, o.drag.outside_envelope_steps
        )
    };
    format!(
        "{{\n  \"schema_version\": {SCHEMA_VERSION},\n  \"test\": {},\n  \"definition_version\": {},\n  \"outcome\": {},\n  \"breaking_point\": {bp},\n  \"metrics\": [{metrics}],\n  \"notes\": [{notes}],\n  \"series\": [{series}],\n  \"steps_simulated\": {},\n  \"turbulence\": {},\n  \"turbulence_height_clamped_steps\": {},\n  \"wind_limited_steps\": {},\n  \"drag_model\": {}{drag_extra}\n}}\n",
        json_str(o.id),
        json_str(&o.version),
        json_str(o.outcome.label()),
        o.steps_total,
        json_str(turbulence_label(cond.turbulence)),
        o.turbulence_clamped_steps,
        o.wind_limited_steps,
        json_str(&o.drag.label),
    )
}

fn definition_json(d: &Definitions, id: TestId) -> Result<String, CampaignError> {
    let r = match id {
        TestId::HoverPayload => serde_json::to_string(&d.hover_payload),
        TestId::WindGust => serde_json::to_string(&d.wind_gust),
        TestId::Climb => serde_json::to_string(&d.climb),
        TestId::Endurance => serde_json::to_string(&d.endurance),
        TestId::ThermalSoak => serde_json::to_string(&d.thermal_soak),
        TestId::MaxSpeed => serde_json::to_string(&d.max_speed),
        TestId::FpvSprint => serde_json::to_string(&d.fpv_sprint),
        TestId::FpvPunchout => serde_json::to_string(&d.fpv_punchout),
    };
    r.map_err(|_| CampaignError::Run("definition json"))
}

fn definition_version(d: &Definitions, id: TestId) -> &str {
    match id {
        TestId::HoverPayload => &d.hover_payload.version,
        TestId::WindGust => &d.wind_gust.version,
        TestId::Climb => &d.climb.version,
        TestId::Endurance => &d.endurance.version,
        TestId::ThermalSoak => &d.thermal_soak.version,
        TestId::MaxSpeed => &d.max_speed.version,
        TestId::FpvSprint => &d.fpv_sprint.version,
        TestId::FpvPunchout => &d.fpv_punchout.version,
    }
}

const SUMMARY_HEADER: &str = "test,definition_version,outcome,breaking_quantity,unit,last_pass,first_fail,criterion,resolution,evaluations,steps_simulated\n";

/// One `summary.csv` line.
fn summary_line(id: TestId, version: &str, r: &TestResult) -> String {
    use crate::textout::csv_field;
    match r {
        Ok(o) => {
            let b = o.breaking.as_ref();
            let cells = [
                b.map_or(String::new(), |b| b.quantity.to_string()),
                b.map_or(String::new(), |b| b.unit.to_string()),
                b.map_or(String::new(), |b| jn(b.last_pass)),
                b.map_or(String::new(), |b| jn(b.first_fail)),
                b.map_or(String::new(), |b| csv_field(&b.criterion)),
                b.map_or(String::new(), |b| jn(b.resolution)),
                b.map_or(String::new(), |b| b.evaluations.to_string()),
            ];
            format!(
                "{},{},{},{},{}\n",
                id.name(),
                version,
                o.outcome.label(),
                cells.join(","),
                o.steps_total
            )
        }
        Err(e) => format!(
            "{},{},ERROR,,,,,{},,,\n",
            id.name(),
            version,
            csv_field(&format!("{e:?}"))
        ),
    }
}

/// One entry of `campaign_result.json`.
fn index_entry(id: TestId, r: &TestResult) -> String {
    use crate::textout::json_str;
    let outcome = r.as_ref().map_or("ERROR", |o| o.outcome.label());
    format!(
        "    {{\"test\": {}, \"outcome\": {}, \"result\": {}}}",
        json_str(id.name()),
        json_str(outcome),
        json_str(&format!("{}/result.json", id.name()))
    )
}

/// Upper bound of the simulated time one test can use, s, from its definition: the length of one run times the most runs its search can
/// make (a ladder of levels, then a bisection to the tolerance). Used only as the denominator of the progress percentage; a test that
/// ends early (a cutoff, a first failure) jumps to 100 % when it finishes. Not a prediction and not a result.
#[must_use]
pub fn planned_sim_s(d: &Definitions, id: TestId) -> f64 {
    let bisect = |range: f64, tol: f64| -> f64 {
        if tol > 0.0 && range > tol {
            (range / tol).log2().ceil() + 2.0
        } else {
            2.0
        }
    };
    let ladder = |from: f64, to: f64, step: f64| -> f64 {
        if step > 0.0 && to > from {
            ((to - from) / step).floor() + 1.0
        } else {
            1.0
        }
    };
    match id {
        TestId::HoverPayload => {
            let x = &d.hover_payload;
            (x.settle_s + x.hold_s) * bisect(x.payload_max_kg, x.tolerance_kg)
        }
        TestId::WindGust => {
            let x = &d.wind_gust;
            (x.settle_s + x.hold_s)
                * (ladder(x.start_wind_mps, x.max_wind_mps, x.step_mps)
                    + bisect(x.step_mps, x.tolerance_mps))
        }
        TestId::Climb => d.climb.preroll_s + d.climb.time_limit_s,
        TestId::Endurance => d.endurance.settle_s + d.endurance.max_duration_s,
        TestId::ThermalSoak => d.thermal_soak.max_duration_s,
        TestId::MaxSpeed => {
            let x = &d.max_speed;
            (x.settle_s + x.hold_s)
                * (ladder(x.start_speed_mps, x.max_speed_mps, x.step_mps)
                    + bisect(x.step_mps, x.tolerance_mps))
        }
        TestId::FpvSprint => d.fpv_sprint.preroll_s + d.fpv_sprint.sprint_s,
        TestId::FpvPunchout => d.fpv_punchout.preroll_s + d.fpv_punchout.punchout_s,
    }
}

/// Files for the tests that have finished so far, as a campaign of just those tests (same bytes per test as the full render; the
/// summary and the index list only the finished tests). Used to show recordings while the rest still compute.
///
/// # Errors
/// As `render_outputs`.
pub fn render_partial(
    inputs: &CampaignInputs<'_>,
    results_so_far: &[Option<TestResult>],
) -> Result<Vec<(String, String)>, CampaignError> {
    let mut tests = Vec::new();
    let mut done = Vec::new();
    for (id, r) in inputs.tests.iter().zip(results_so_far) {
        if let Some(r) = r {
            tests.push(*id);
            done.push(r.clone());
        }
    }
    let sub = CampaignInputs {
        build: inputs.build,
        build_name: inputs.build_name,
        parts: inputs.parts.clone(),
        db_version: inputs.db_version,
        git_hash: inputs.git_hash,
        conditions: inputs.conditions,
        definitions: inputs.definitions,
        tests,
        drag: inputs.drag,
        cfd_surrogate: inputs.cfd_surrogate,
    };
    render_outputs(&sub, &done)
}

/// The files of one test: manifest, result, events and the time series (an error result has only manifest and result).
fn test_files(
    inputs: &CampaignInputs<'_>,
    id: TestId,
    r: &TestResult,
) -> Result<Vec<(String, String)>, CampaignError> {
    use crate::textout::json_str;
    let version = definition_version(inputs.definitions, id);
    let manifest = manifest_json(
        inputs,
        id.name(),
        version,
        &definition_json(inputs.definitions, id)?,
        r.as_ref().map_or(DRAG_MODEL, |o| o.drag.label.as_str()),
    );
    let mut files = vec![(format!("{}/manifest.json", id.name()), manifest)];
    match r {
        Ok(o) => {
            files.push((
                format!("{}/result.json", id.name()),
                result_json(o, inputs.conditions),
            ));
            files.push((format!("{}/events.json", id.name()), events_json(o)));
            for s in &o.series {
                files.push((
                    format!("{}/timeseries_{}.csv", id.name(), s.label),
                    s.csv.clone(),
                ));
            }
        }
        Err(e) => {
            let body = format!(
                "{{\n  \"test\": {},\n  \"outcome\": \"ERROR\",\n  \"error\": {}\n}}\n",
                json_str(id.name()),
                json_str(&format!("{e:?}"))
            );
            files.push((format!("{}/result.json", id.name()), body));
        }
    }
    Ok(files)
}

/// Renders every output file of a campaign as `(relative path, text)`; pure, so tests can compare bytes.
///
/// # Errors
/// `Run` if a definition cannot be serialised.
pub fn render_outputs(
    inputs: &CampaignInputs<'_>,
    results: &[TestResult],
) -> Result<Vec<(String, String)>, CampaignError> {
    use crate::textout::json_str;
    let mut files = Vec::new();
    let mut summary = String::from(SUMMARY_HEADER);
    let mut index = Vec::new();
    for (id, r) in inputs.tests.iter().zip(results) {
        files.extend(test_files(inputs, *id, r)?);
        summary.push_str(&summary_line(
            *id,
            definition_version(inputs.definitions, *id),
            r,
        ));
        index.push(index_entry(*id, r));
    }
    files.push(("summary.csv".to_string(), summary));
    let cfd_extra = cfd_extra(inputs.cfd_surrogate);
    files.push((
        "campaign_result.json".to_string(),
        format!(
            "{{\n  \"schema_version\": {SCHEMA_VERSION},\n  \"build\": {},\n  \"definitions_set_version\": {},\n  \"git_hash\": {},\n  \"drag_model\": {}{cfd_extra},\n  \"turbulence\": {},\n  \"tests\": [\n{}\n  ]\n}}\n",
            json_str(inputs.build_name),
            json_str(&inputs.definitions.definitions_version),
            json_str(inputs.git_hash),
            json_str(results.iter().find_map(|r| r.as_ref().ok()).map_or(DRAG_MODEL, |o| o.drag.label.as_str())),
            json_str(turbulence_label(inputs.conditions.turbulence)),
            index.join(",\n")
        ),
    ));
    Ok(files)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::controller_sim::{self, Rig, golden_dir, load_build};

    pub(crate) fn std_conditions() -> Conditions {
        Conditions {
            name: None,
            ambient_c: 20.0,
            site_altitude_m: 0.0,
            wind_speed_mps: 0.0,
            wind_from_deg: 0.0,
            turbulence: TurbulenceKind::None,
            gust_level: 0.0,
            seed: 1,
            payload_kg: 0.0,
            payload_position_m: [0.0; 3],
            battery_soc: 1.0,
            battery_temp_c: None,
        }
    }

    pub(crate) fn ref_build() -> Build {
        load_build(&golden_dir("reference_quad")).unwrap()
    }

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn test_camp_sim_dynamics_equal_the_ffi_step_bit_for_bit() {
        // Same atmosphere (ambient 20 C at the site = +5 K offset -> the density temperature is ISA + 5 K in both), same controller,
        // no wind: the campaign's own step and `ffi_step_powertrain` must give identical states. Only the THERMAL ambient differs by
        // design (typed 20 C here, 20 C + offset = 25 C in the FFI step), and the thermal nodes do not feed back into the dynamics.
        let build = ref_build();
        let cond = std_conditions();
        let mut sim = Sim::new(&build, &cond, 20.0, 0, None).unwrap();
        let mut rig = Rig::new(&build, 20.0).unwrap();
        rig.set_environment(cond.environment());
        let sp = Setpoint::Position {
            ned_m: [1.0, 0.0, -21.0],
            yaw_rad: 0.1,
        };
        for k in 0..400 {
            let a = sim.step(&sp).unwrap();
            let b = rig.step(&sp).unwrap();
            assert_eq!(
                a.position_ned_m.map(f64::to_bits),
                b.position_ned_m.map(f64::to_bits),
                "step {k}"
            );
            assert_eq!(
                a.velocity_ned_mps.map(f64::to_bits),
                b.velocity_ned_mps.map(f64::to_bits),
                "step {k}"
            );
            assert_eq!(
                a.rates_rps.map(f64::to_bits),
                b.rates_rps.map(f64::to_bits),
                "step {k}"
            );
            assert_eq!(a.tilt_rad.to_bits(), b.tilt_rad.to_bits(), "step {k}");
            assert_eq!(
                a.out.throttle.to_bits(),
                b.out.throttle.to_bits(),
                "step {k}"
            );
        }
        assert_eq!(controller_sim::DT_S, DT_S);
    }

    #[test]
    fn test_camp_thermal_ambient_is_the_typed_temperature() {
        // all thermal nodes start at the typed ambient and are driven toward it: 40 C against 0 C differ by about 40 K after 1 s of hover
        let build = ref_build();
        let sp = Setpoint::Position {
            ned_m: [0.0, 0.0, -20.0],
            yaw_rad: 0.0,
        };
        let mut warm = std_conditions();
        warm.ambient_c = 40.0;
        let mut cold = std_conditions();
        cold.ambient_c = 0.0;
        let (mut a, mut b) = (
            Sim::new(&build, &warm, 20.0, 0, None).unwrap(),
            Sim::new(&build, &cold, 20.0, 0, None).unwrap(),
        );
        let (mut fa, mut fb) = (None, None);
        for _ in 0..500 {
            fa = Some(a.step(&sp).unwrap());
            fb = Some(b.step(&sp).unwrap());
        }
        let (ta, tb) = (
            fa.unwrap().tel.motor_winding_c[0],
            fb.unwrap().tel.motor_winding_c[0],
        );
        assert!(ta - tb > 38.0 && ta - tb < 41.0, "{ta} {tb}");
    }

    #[test]
    fn test_camp_isa_offset_hand_values() {
        // ISA: 15 C at sea level, -6.5 K per km: 1000 m -> 8.5 C; ambient 20 C at 0 m is +5 K, at 1000 m is +11.5 K
        assert!(
            near(isa_temperature_c(0.0), 15.0, 1e-12)
                && near(isa_temperature_c(1000.0), 8.5, 1e-12)
        );
        let mut c = std_conditions();
        assert!(near(c.temp_offset_c(), 5.0, 1e-12));
        c.site_altitude_m = 1000.0;
        assert!(near(c.temp_offset_c(), 11.5, 1e-12));
        c.ambient_c = 8.5;
        assert!(near(c.temp_offset_c(), 0.0, 1e-12)); // standard day at that altitude
        assert!(near(c.environment().site_altitude_m, 1000.0, 0.0));
    }

    fn root() -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
    }

    fn read(rel: &str) -> String {
        std::fs::read_to_string(root().join(rel)).unwrap()
    }

    fn sample_frame() -> Frame {
        let mut sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        sim.step(&hold_setpoint(20.0)).unwrap()
    }

    // ------------------------------------------------------------------ conditions and definitions

    const COND_JSON: &str = r#"{"schema_version":1,"ambient_c":20.0,"site_altitude_m":0.0,"wind_speed_mps":5.0,"wind_from_deg":270.0,"turbulence":"dryden","gust_level":0.0,"seed":7,"payload_kg":0.5,"payload_position_m":[0.1,0.0,0.0],"battery_soc":0.9,"battery_temp_c":25.0}"#;

    #[test]
    fn test_camp_conditions_parse_and_wind_direction_hand_values() {
        let c = parse_conditions(COND_JSON).unwrap();
        assert_eq!(
            (c.seed, c.turbulence, c.battery_temp_c),
            (7, TurbulenceKind::Dryden, Some(25.0))
        );
        // wind FROM the west (270 deg) blows toward the east: NED (0, +v, 0); from north (0) toward south; from east (90) toward west; from south (180) toward north
        let w = c.mean_wind_ned(5.0);
        assert!(near(w[0], 0.0, 1e-12) && near(w[1], 5.0, 1e-12) && w[2] == 0.0);
        let mut d = c.clone();
        for (deg, want) in [(0.0, [-5.0, 0.0]), (90.0, [0.0, -5.0]), (180.0, [5.0, 0.0])] {
            d.wind_from_deg = deg;
            let v = d.mean_wind_ned(5.0);
            assert!(
                near(v[0], want[0], 1e-12) && near(v[1], want[1], 1e-12),
                "{deg}"
            );
        }
    }

    #[test]
    fn test_camp_conditions_refusals() {
        let bad = [
            (r#""ambient_c":20.0"#, r#""ambient_c":-41.0"#),
            (r#""ambient_c":20.0"#, r#""ambient_c":56.0"#),
            (r#""site_altitude_m":0.0"#, r#""site_altitude_m":4001.0"#),
            (r#""wind_speed_mps":5.0"#, r#""wind_speed_mps":20.5"#),
            (r#""wind_speed_mps":5.0"#, r#""wind_speed_mps":-1.0"#),
            (r#""wind_from_deg":270.0"#, r#""wind_from_deg":360.0"#),
            (r#""gust_level":0.0"#, r#""gust_level":1.5"#),
            (r#""battery_soc":0.9"#, r#""battery_soc":1.1"#),
            (r#""battery_temp_c":25.0"#, r#""battery_temp_c":101.0"#),
            (r#""payload_kg":0.5"#, r#""payload_kg":51.0"#),
            (
                r#""payload_position_m":[0.1,0.0,0.0]"#,
                r#""payload_position_m":[2.5,0.0,0.0]"#,
            ),
            (r#""ambient_c":20.0"#, r#""ambient_c":NaN"#),
        ];
        for (from, to) in bad {
            assert!(
                parse_conditions(&COND_JSON.replace(from, to)).is_err(),
                "{to}"
            );
        }
        assert_eq!(
            parse_conditions(&COND_JSON.replace("dryden", "storm")),
            Err(CampaignError::Parse)
        );
        assert_eq!(
            parse_conditions(&COND_JSON.replace("}", r#","extra":1}"#)),
            Err(CampaignError::Parse)
        );
        assert!(matches!(
            parse_conditions(&COND_JSON.replace(r#""schema_version":1"#, r#""schema_version":2"#)),
            Err(CampaignError::Invalid(_))
        ));
        assert_eq!(parse_conditions("{"), Err(CampaignError::Parse));
        // edges are accepted
        for (from, to) in [
            (r#""ambient_c":20.0"#, r#""ambient_c":-40.0"#),
            (r#""wind_speed_mps":5.0"#, r#""wind_speed_mps":20.0"#),
            (r#""wind_from_deg":270.0"#, r#""wind_from_deg":0.0"#),
        ] {
            assert!(
                parse_conditions(&COND_JSON.replace(from, to)).is_ok(),
                "{to}"
            );
        }
    }

    #[test]
    fn test_camp_committed_data_files_load() {
        let std = parse_conditions(&read("campaign/conditions_standard.json")).unwrap();
        let smoke = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        assert!(std.battery_soc == 1.0 && smoke.battery_soc < 0.1);
        let full = parse_definitions(&read("campaign/test_definitions.json")).unwrap();
        let short = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        assert_eq!(full.definitions_version, "1.1.0"); // 1.0.0 + the max_speed / fpv_sprint / fpv_punchout sections
        assert!(short.definitions_version.ends_with("smoke"));
        assert!(
            short.endurance.max_duration_s < full.endurance.max_duration_s
                && short.hover_payload.hold_s < full.hover_payload.hold_s
        );
        assert_eq!(full.provenance["source_type"], "ASSUMED");
    }

    #[test]
    fn test_camp_definition_refusals() {
        let base: serde_json::Value =
            serde_json::from_str(&read("campaign/test_definitions.json")).unwrap();
        let edits: [(&str, &str, serde_json::Value); 14] = [
            ("hover_payload", "payload_max_kg", serde_json::json!(0.0)),
            ("hover_payload", "tolerance_kg", serde_json::json!(100.0)),
            ("hover_payload", "eval_window_s", serde_json::json!(1000.0)),
            (
                "hover_payload",
                "min_thrust_reserve_ratio",
                serde_json::json!(0.5),
            ),
            ("hover_payload", "collective_cap", serde_json::json!(1.5)),
            ("wind_gust", "max_wind_mps", serde_json::json!(25.0)),
            ("wind_gust", "tolerance_mps", serde_json::json!(5.0)),
            ("wind_gust", "start_wind_mps", serde_json::json!(30.0)),
            ("climb", "target_height_m", serde_json::json!(-1.0)),
            ("endurance", "mission", serde_json::json!("orbit")),
            ("endurance", "crash_altitude_m", serde_json::json!(50.0)),
            (
                "thermal_soak",
                "steady_window_s",
                serde_json::json!(99999.0),
            ),
            (
                "thermal_soak",
                "steady_rate_k_per_min",
                serde_json::json!(0.0),
            ),
            ("climb", "time_limit_s", serde_json::json!(f64::NAN)),
        ];
        for (test, key, value) in edits {
            let mut v = base.clone();
            v[test][key] = value.clone();
            let text = serde_json::to_string(&v).unwrap().replace("null", "NaN");
            assert!(parse_definitions(&text).is_err(), "{test}.{key} = {value}");
        }
        let mut extra = base.clone();
        extra["climb"]["surprise"] = serde_json::json!(1);
        assert_eq!(
            parse_definitions(&serde_json::to_string(&extra).unwrap()),
            Err(CampaignError::Parse)
        );
        let mut missing = base.clone();
        missing.as_object_mut().unwrap().remove("endurance");
        assert_eq!(
            parse_definitions(&serde_json::to_string(&missing).unwrap()),
            Err(CampaignError::Parse)
        );
        assert!(parse_definitions(&serde_json::to_string(&base).unwrap()).is_ok());
    }

    // ------------------------------------------------------------------ searches

    fn threshold(
        limit: f64,
        calls: &std::cell::Cell<u32>,
    ) -> impl FnMut(f64) -> Result<Verdict, ()> + '_ {
        move |x| {
            calls.set(calls.get() + 1);
            Ok(Verdict {
                pass: x <= limit,
                criterion: (x > limit).then_some("too_big"),
            })
        }
    }

    #[test]
    fn test_camp_bisect_hand_values() {
        // [0, 10], tol 0.5, passes up to 3: ends at lo 0 (pass), hi 10 (fail), then mids 5 F, 2.5 P, 3.75 F, 3.125 F, 2.8125 P
        // -> bracket [2.8125, 3.125] (0.3125 <= 0.5); 2 + 5 = 7 evaluations
        let calls = std::cell::Cell::new(0);
        let s = bisect(0.0, 10.0, 0.5, threshold(3.0, &calls)).unwrap();
        assert_eq!(
            s,
            Search::Found {
                pass: 2.8125,
                fail: 3.125,
                criterion: "too_big",
                evaluations: 7
            }
        );
        assert_eq!(calls.get(), 7);
        // a tight tolerance brackets the threshold: pass <= limit < fail, width <= tol
        let c2 = std::cell::Cell::new(0);
        let Search::Found { pass, fail, .. } =
            bisect(0.0, 10.0, 0.001, threshold(std::f64::consts::PI, &c2)).unwrap()
        else {
            panic!("not found")
        };
        assert!(
            pass <= std::f64::consts::PI && fail > std::f64::consts::PI && fail - pass <= 0.001
        );
        assert!(c2.get() <= 16);
    }

    #[test]
    fn test_camp_bisect_edges_and_errors() {
        let c = std::cell::Cell::new(0);
        assert_eq!(
            bisect(0.0, 10.0, 0.5, threshold(20.0, &c)).unwrap(),
            Search::AllPass
        );
        assert_eq!(c.get(), 2);
        assert_eq!(
            bisect(1.0, 10.0, 0.5, threshold(0.5, &c)).unwrap(),
            Search::FailsAtStart("too_big")
        );
        // the upper bound is inclusive in the pass test: threshold exactly at hi passes everything
        assert_eq!(
            bisect(0.0, 10.0, 0.5, threshold(10.0, &c)).unwrap(),
            Search::AllPass
        );
        // an error from the evaluation is passed on at once
        let r: Result<Search, &str> = bisect(0.0, 10.0, 0.5, |x| {
            if x > 4.0 {
                Err("boom")
            } else {
                Ok(Verdict {
                    pass: true,
                    criterion: None,
                })
            }
        });
        assert_eq!(r, Err("boom"));
        // bracket already narrower than the tolerance: no bisection steps
        let d = std::cell::Cell::new(0);
        assert_eq!(
            bisect(0.0, 0.2, 0.5, threshold(0.1, &d)).unwrap(),
            Search::Found {
                pass: 0.0,
                fail: 0.2,
                criterion: "too_big",
                evaluations: 2
            }
        );
    }

    #[test]
    fn test_camp_ramp_and_refine_hand_values() {
        // levels 0, 5, 10 pass, 15 fails (threshold 12.3); refine (10, 15) with tol 1: 12.5 F, 11.25 P, 11.875 P -> [11.875, 12.5]; 4 + 3 = 7
        let c = std::cell::Cell::new(0);
        let s = ramp_and_refine(0.0, 5.0, 20.0, 1.0, threshold(12.3, &c)).unwrap();
        assert_eq!(
            s,
            Search::Found {
                pass: 11.875,
                fail: 12.5,
                criterion: "too_big",
                evaluations: 7
            }
        );
        let c = std::cell::Cell::new(0);
        assert_eq!(
            ramp_and_refine(0.0, 5.0, 20.0, 1.0, threshold(-1.0, &c)).unwrap(),
            Search::FailsAtStart("too_big")
        );
        assert_eq!(c.get(), 1);
        let c = std::cell::Cell::new(0);
        assert_eq!(
            ramp_and_refine(0.0, 5.0, 20.0, 1.0, threshold(100.0, &c)).unwrap(),
            Search::AllPass
        );
        assert_eq!(c.get(), 5); // 0, 5, 10, 15, 20: the maximum is tried
        // levels 0, 5, 10, 15 pass and 20 fails (threshold 19): refine (15, 20) with tol 1: 17.5 P, 18.75 P, 19.375 F -> [18.75, 19.375]; 5 + 3 = 8
        let c = std::cell::Cell::new(0);
        assert_eq!(
            ramp_and_refine(0.0, 5.0, 20.0, 1.0, threshold(19.0, &c)).unwrap(),
            Search::Found {
                pass: 18.75,
                fail: 19.375,
                criterion: "too_big",
                evaluations: 8
            }
        );
    }

    // ------------------------------------------------------------------ events, peaks, windows, decimation

    #[test]
    fn test_camp_event_first_last_active_and_activations() {
        let sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        let mut obs = Observer::new(100.0, f64::INFINITY, 0.98); // no rows
        let mut f = sample_frame();
        // pattern over 5 steps at t = 0.002 .. 0.010: off, on, on, off, on  ->  first 0.004, last 0.010, 3 active steps, 2 activations
        for (k, on) in [false, true, true, false, true].into_iter().enumerate() {
            f.t_s = 0.002 * (k + 1) as f64;
            f.tel.motor_over_temp_mask = i32::from(on);
            obs.observe(&sim, &f, None).unwrap();
        }
        let e = obs
            .events
            .iter()
            .find(|e| e.name == "motor_over_temp")
            .unwrap();
        assert!(near(e.first_t_s.unwrap(), 0.004, 1e-12) && near(e.last_t_s, 0.010, 1e-12));
        assert_eq!((e.active_steps, e.activations), (3, 2));
        let never = obs
            .events
            .iter()
            .find(|e| e.name == "battery_cutoff")
            .unwrap();
        assert_eq!(
            (never.first_t_s, never.active_steps, never.activations),
            (None, 0, 0)
        );
    }

    #[test]
    fn test_camp_every_event_condition_maps_to_its_flag() {
        let sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        type Edit = fn(&mut Frame);
        let cases: [(&str, Edit); 12] = [
            ("motor_over_current", |f| f.tel.motor_over_current_mask = 1),
            ("esc_over_current", |f| f.tel.esc_over_current_mask = 1),
            ("battery_over_current", |f| f.tel.battery_over_current = 1),
            ("motor_over_temp", |f| f.tel.motor_over_temp_mask = 1),
            ("esc_over_temp", |f| f.tel.esc_over_temp_mask = 1),
            ("battery_over_temp", |f| f.tel.battery_over_temp = 1),
            ("battery_cutoff", |f| f.tel.battery_cutoff = 1),
            ("prop_outside_table", |f| f.tel.prop_extrapolated = 2),
            ("axis_saturated", |f| f.axis_railed = true),
            ("collective_limited", |f| f.out.collective_limited = true),
            ("tilt_limit_active", |f| f.out.tilt_limited = true),
            ("instability", |f| f.tilt_rad = 1.5),
        ];
        for (name, edit) in cases {
            let mut obs = Observer::new(100.0, f64::INFINITY, 0.98);
            let mut f = sample_frame();
            f.tel.prop_extrapolated = 0;
            obs.observe(&sim, &f, None).unwrap();
            assert!(
                obs.events.iter().all(|e| e.first_t_s.is_none()),
                "{name}: quiet frame raised an event"
            );
            edit(&mut f);
            obs.observe(&sim, &f, None).unwrap();
            let active: Vec<&str> = obs
                .events
                .iter()
                .filter(|e| e.first_t_s.is_some())
                .map(|e| e.name)
                .collect();
            assert_eq!(active, vec![name], "{name}");
        }
        // a rate above 30 rad/s is also instability; 29.9 is not
        let mut obs = Observer::new(100.0, f64::INFINITY, 0.98);
        let mut f = sample_frame();
        f.rates_rps = [29.9, 0.0, 0.0];
        obs.observe(&sim, &f, None).unwrap();
        f.rates_rps = [0.0, -30.1, 0.0];
        obs.observe(&sim, &f, None).unwrap();
        assert_eq!(
            obs.events
                .iter()
                .find(|e| e.name == "instability")
                .unwrap()
                .active_steps,
            1
        );
    }

    #[test]
    fn test_camp_altitude_hold_lost_follows_the_limit() {
        let sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        let mut obs = Observer::new(100.0, 0.5, 0.98);
        let mut f = sample_frame();
        f.position_ned_m = [0.0, 0.0, -20.4]; // 0.4 m off the target altitude: inside
        obs.observe(&sim, &f, Some([0.0, 0.0, -20.0])).unwrap();
        f.position_ned_m = [0.0, 0.0, -20.6]; // 0.6 m: outside
        obs.observe(&sim, &f, Some([0.0, 0.0, -20.0])).unwrap();
        f.position_ned_m = [0.0, 0.0, -25.0];
        obs.observe(&sim, &f, None).unwrap(); // no target: never lost
        assert_eq!(
            obs.events
                .iter()
                .find(|e| e.name == "altitude_hold_lost")
                .unwrap()
                .active_steps,
            1
        );
    }

    #[test]
    fn test_camp_peaks_and_window_statistics_hand_values() {
        let sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        let mut obs = Observer::new(100.0, f64::INFINITY, 0.9);
        let mut f = sample_frame();
        // 4 window steps: winding 30, 50, 40, 45 C; railed on steps 2 and 4 (50 %); throttle 0.95 on step 3 only (25 % capped)
        obs.start_window();
        for (k, (winding, railed, thr)) in [
            (30.0, false, 0.5),
            (50.0, true, 0.5),
            (40.0, false, 0.95),
            (45.0, true, 0.5),
        ]
        .into_iter()
        .enumerate()
        {
            f.t_s = 1.0 + k as f64;
            f.tel.motor_winding_c = [winding, 0.0, 0.0, 0.0];
            f.axis_railed = railed;
            f.out.throttle = thr;
            f.out.collective_limited = false;
            f.position_ned_m = [3.0 * (k as f64), 4.0 * (k as f64), -20.0 - k as f64];
            obs.observe(&sim, &f, Some([0.0, 0.0, -20.0])).unwrap();
        }
        assert!(
            near(obs.peaks.winding_c.value, 50.0, 0.0) && near(obs.peaks.winding_c.t_s, 2.0, 0.0)
        );
        let w = &obs.window;
        assert_eq!(w.steps, 4);
        assert!(near(w.saturated_fraction(), 0.5, 1e-12) && near(w.capped_fraction(), 0.25, 1e-12));
        // position errors: horizontal 0, 5, 10, 15 (3-4-5 triangles); altitude 0, 1, 2, 3
        assert!(near(w.max_h_err_m, 15.0, 1e-12) && near(w.max_alt_err_m, 3.0, 1e-12));
        obs.start_window();
        assert_eq!(obs.window.steps, 0);
        assert_eq!(obs.window.saturated_fraction(), 0.0);
    }

    #[test]
    fn test_camp_decimation_and_row_format() {
        // 0.01 s = every 5th step starting with the first: steps 0, 5, 10, 15 of 20 -> 4 rows
        let mut sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        let mut obs = Observer::new(0.01, f64::INFINITY, 0.98);
        for _ in 0..20 {
            let f = sim.step(&hold_setpoint(20.0)).unwrap();
            obs.observe(&sim, &f, Some([0.0, 0.0, -20.0])).unwrap();
        }
        let (csv, rows) = obs.csv();
        assert_eq!(rows, 4);
        let lines: Vec<&str> = csv.lines().collect();
        assert_eq!(lines.len(), 5);
        assert_eq!(lines[0].split(',').count(), SERIES_COLUMNS.len());
        assert!(
            lines[1..]
                .iter()
                .all(|l| l.split(',').count() == SERIES_COLUMNS.len() && !l.contains("NaN"))
        );
        // first row is the first step: t = 0.002 s, altitude 20 m (pos_d -20 within a few mm)
        let first: Vec<f64> = lines[1].split(',').map(|c| c.parse().unwrap()).collect();
        assert!(near(first[0], 0.002, 1e-12) && near(first[3], -20.0, 0.01));
        assert_eq!(SERIES_COLUMNS[0], "t_s");
    }

    #[test]
    fn test_camp_thrust_reserve_is_the_bench_point_at_the_sagged_full_duty_voltage() {
        // Review finding 3: the reserve used the present (hover) pack voltage; four motors at full duty draw far more current and sag
        // the pack. The root V* must satisfy V* = OCV - R x I_pack(V*), checked here with `bench_point`'s own battery current
        // (battery_current_a of one motor, x 4), not with the formula the solver uses.
        let build = ref_build();
        let mut sim = Sim::new(&build, &std_conditions(), 20.0, 0, None).unwrap();
        let f = sim.step(&hold_setpoint(20.0)).unwrap();
        let r = sim.thrust_reserve_ratio(&f).unwrap();
        let ocv = f.tel.battery_ocv_v;
        let (thrust_n, v_star) =
            sagged_full_duty_point(&build.powertrain, ocv, f.rho_kgpm3, f.axial_inflow_mps)
                .unwrap();
        let b = powertrain::bench_point(
            &build.powertrain,
            1.0,
            v_star,
            f.rho_kgpm3,
            f.axial_inflow_mps,
        )
        .unwrap();
        let residual = v_star
            - (ocv - build.powertrain.battery_internal_resistance_ohm * 4.0 * b.battery_current_a);
        assert!(residual.abs() < 1e-6, "fixed-point residual {residual} V");
        assert!(near(thrust_n, b.thrust_n, 1e-12));
        assert!(near(r, 4.0 * b.thrust_n / (1.536 * 9.806_65), 1e-12));
        // the full-duty pack really sags: below the hover voltage (hover draws about 7 A, full duty about 100 A), and the reserve is
        // below the old "present voltage" number but still above 1 for this vehicle
        assert!(v_star < f.tel.battery_voltage_v - 0.5, "{v_star} V");
        let old = powertrain::bench_point(
            &build.powertrain,
            1.0,
            f.tel.battery_voltage_v,
            f.rho_kgpm3,
            f.axial_inflow_mps,
        )
        .unwrap();
        assert!(r < 4.0 * old.thrust_n / (1.536 * 9.806_65) && r > 1.0);
        // hand estimate: one motor gives 23.75 N at 14.8 V full duty (P3a bench point) and thrust goes with voltage squared, so at the
        // sagged supply V* the ratio is about 4 x 23.75 x (V* / 14.8)^2 / 15.063 (the propeller table moves it a few percent)
        let hand = 4.0 * 23.751_814 * (v_star / 14.8).powi(2) / 15.063_014_4;
        assert!(near(r / hand, 1.0, 0.06), "{r} vs {hand} at {v_star} V");
    }

    #[test]
    fn test_camp_sagged_reserve_with_zero_resistance_is_the_unsagged_bench_point() {
        // R = 0: the supply stays at the open-circuit voltage whatever the current, so V* = OCV and the thrust is the bench point there
        let build = ref_build();
        let mut cfg = build.powertrain;
        cfg.battery_internal_resistance_ohm = 0.0;
        let ocv = 16.0;
        let (thrust_n, v_star) = sagged_full_duty_point(&cfg, ocv, 1.225, 0.0).unwrap();
        assert!(near(v_star, ocv, 1e-9), "{v_star}");
        let b = powertrain::bench_point(&cfg, 1.0, ocv, 1.225, 0.0).unwrap();
        assert!(near(thrust_n, b.thrust_n, 1e-9));
        // a non-positive or non-finite OCV is refused, never turned into a number
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(sagged_full_duty_point(&cfg, bad, 1.225, 0.0).is_err());
        }
    }

    // ------------------------------------------------------------------ simulation behaviour

    #[test]
    fn test_camp_loss_criterion_mapping() {
        assert_eq!(
            loss_criterion(&CampaignError::Model("diverged")),
            Some("instability")
        );
        assert_eq!(
            loss_criterion(&CampaignError::Model("atmosphere")),
            Some("crash")
        );
        assert_eq!(loss_criterion(&CampaignError::Model("drag")), Some("crash"));
        assert_eq!(loss_criterion(&CampaignError::Model("powertrain")), None);
        assert_eq!(
            loss_criterion(&CampaignError::Run("controller input")),
            None
        );
        assert_eq!(loss_criterion(&CampaignError::Invalid("x")), None);
    }

    #[test]
    fn test_camp_a_vehicle_that_cannot_lift_the_payload_fails_as_a_crash() {
        let def = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let r = hover_run(
            &ref_build(),
            &std_conditions(),
            &def.hover_payload,
            50.0,
            0,
            &analytic_drag,
        )
        .unwrap();
        assert!(!r.verdict.pass);
        assert!(
            matches!(r.verdict.criterion, Some("crash" | "instability")),
            "{:?}",
            r.verdict
        );
        let ok = hover_run(
            &ref_build(),
            &std_conditions(),
            &def.hover_payload,
            0.0,
            0,
            &analytic_drag,
        )
        .unwrap();
        assert!(ok.verdict.pass, "{:?}", ok.verdict);
    }

    #[test]
    fn test_camp_the_gust_is_limited_to_the_core_wind_range() {
        // mean (20, 0, 0) m/s, gust (100, -100, 5): x has 9.9 m/s of room under the 29.9 limit, y has 29.9, z passes
        let mut sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        sim.mean_wind_ned_mps = [20.0, 0.0, 0.0];
        let (total, gust) = sim.limited_wind([100.0, -100.0, 5.0]);
        assert!(
            near(total[0], 29.9, 1e-12)
                && near(total[1], -29.9, 1e-12)
                && near(total[2], 5.0, 1e-12)
        );
        assert!(
            near(gust[0], 9.9, 1e-12) && near(gust[1], -29.9, 1e-12) && near(gust[2], 5.0, 1e-12)
        );
        assert_eq!(sim.wind_limited_steps, 1);
        // a gust that fits is passed on unchanged and not counted; a mean wind against the gust leaves room on the other side
        let (total, gust) = sim.limited_wind([-5.0, 3.0, 0.0]);
        assert!(
            near(total[0], 15.0, 1e-12) && near(gust[0], -5.0, 1e-12) && near(total[1], 3.0, 1e-12)
        );
        assert_eq!(sim.wind_limited_steps, 1);
        sim.mean_wind_ned_mps = [-20.0, 0.0, 0.0];
        let (total, _) = sim.limited_wind([-100.0, 0.0, 0.0]);
        assert!(near(total[0], -29.9, 1e-12));
        assert_eq!(sim.wind_limited_steps, 2);
        // a calm run never limits
        let mut calm = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        for _ in 0..100 {
            calm.step(&hold_setpoint(20.0)).unwrap();
        }
        assert_eq!(calm.wind_limited_steps, 0);
    }

    #[test]
    fn test_camp_payload_raises_the_hover_duty_and_lowers_the_reserve() {
        let build = ref_build();
        let mut heavy = std_conditions();
        heavy.payload_kg = 1.0;
        heavy.payload_position_m = [0.0, 0.0, 0.05];
        let (mut a, mut b) = (
            Sim::new(&build, &std_conditions(), 20.0, 0, None).unwrap(),
            Sim::new(&build, &heavy, 20.0, 0, None).unwrap(),
        );
        assert!(near(b.vehicle().mass_kg, 2.536, 1e-12) && near(a.vehicle().mass_kg, 1.536, 1e-12));
        let (mut fa, mut fb) = (None, None);
        for _ in 0..2500 {
            fa = Some(a.step(&hold_setpoint(20.0)).unwrap());
            fb = Some(b.step(&hold_setpoint(20.0)).unwrap());
        }
        let (fa, fb) = (fa.unwrap(), fb.unwrap());
        // after settling both hold 20 m; the loaded vehicle needs sqrt(2.536 / 1.536) = 1.285 times the duty
        assert!(near(fa.position_ned_m[2], -20.0, 0.1) && near(fb.position_ned_m[2], -20.0, 0.1));
        assert!(
            near(
                fb.out.throttle / fa.out.throttle,
                (2.536_f64 / 1.536).sqrt(),
                0.03
            ),
            "{} {}",
            fa.out.throttle,
            fb.out.throttle
        );
        assert!(b.thrust_reserve_ratio(&fb).unwrap() < a.thrust_reserve_ratio(&fa).unwrap());
    }

    // ------------------------------------------------------------------ the smoke campaign

    fn smoke_inputs<'a>(
        build: &'a Build,
        cond: &'a Conditions,
        defs: &'a Definitions,
        tests: Vec<TestId>,
    ) -> CampaignInputs<'a> {
        CampaignInputs {
            build,
            build_name: "reference_quad",
            parts: vec![("motor".to_string(), "tmotor_mn3508_kv700".to_string())],
            db_version: "0.1.0",
            git_hash: "abc123",
            conditions: cond,
            definitions: defs,
            tests,
            drag: None,
            cfd_surrogate: None,
        }
    }

    /// Results and rendered files of the smoke campaign.
    type SmokeRun = (Vec<TestResult>, Vec<(String, String)>);

    /// The smoke campaign run once for the whole test binary (about 10 s in debug).
    fn smoke() -> &'static SmokeRun {
        static CELL: std::sync::OnceLock<SmokeRun> = std::sync::OnceLock::new();
        CELL.get_or_init(|| {
            let build = ref_build();
            let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
            let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
            let inputs = smoke_inputs(&build, &cond, &defs, TestId::ALL.to_vec());
            let results = run_campaign(&inputs, 3).unwrap();
            let files = render_outputs(&inputs, &results).unwrap();
            (results, files)
        })
    }

    fn out_of(id: TestId) -> &'static TestOutput {
        let i = TestId::ALL.iter().position(|t| *t == id).unwrap();
        smoke().0[i].as_ref().unwrap()
    }

    fn metric(o: &TestOutput, name: &str) -> f64 {
        o.metrics.iter().find(|(k, _)| k == name).unwrap().1
    }

    // Bands below are ENGINEERING CHOICES around the numbers measured on 2026-10-10 with the smoke definitions (battery at 5 %).

    #[test]
    fn test_camp_smoke_hover_payload_finds_a_bracketed_limit() {
        let o = out_of(TestId::HoverPayload);
        let b = o.breaking.as_ref().unwrap();
        assert_eq!(o.outcome, Outcome::BreakingPointFound);
        assert!(
            matches!(
                b.criterion.as_str(),
                "altitude_hold" | "thrust_reserve" | "collective_cap" | "crash" | "instability"
            ),
            "{}",
            b.criterion
        );
        assert!(
            b.last_pass >= 1.0
                && b.last_pass <= 4.0
                && near(b.resolution, b.first_fail - b.last_pass, 1e-12)
                && b.resolution <= 0.5
        );
        assert_eq!(o.series.len(), 2);
        assert!(
            o.series.iter().any(|s| s.label.starts_with("pass_"))
                && o.series.iter().any(|s| s.label.starts_with("fail_"))
        );
    }

    #[test]
    fn test_camp_smoke_wind_gust_breaks_at_a_named_criterion() {
        let o = out_of(TestId::WindGust);
        let b = o.breaking.as_ref().unwrap();
        assert!(
            matches!(
                b.criterion.as_str(),
                "position_error" | "saturation" | "instability" | "crash"
            ),
            "{}",
            b.criterion
        );
        assert!(
            b.last_pass >= 8.0 && b.last_pass < 20.0 && b.resolution <= 1.5,
            "{b:?}"
        );
        assert!(
            o.notes.iter().any(|n| n.contains("UNVERIFIED")),
            "{:?}",
            o.notes
        );
    }

    #[test]
    fn test_camp_smoke_climb_reaches_the_target() {
        let o = out_of(TestId::Climb);
        assert_eq!(o.outcome, Outcome::Completed);
        let t = metric(o, "time_to_target_s");
        assert!(t > 2.0 && t < 10.0, "{t}");
        assert!(
            metric(o, "sustained_climb_rate_mps") > 2.0
                && metric(o, "max_climb_rate_mps") >= metric(o, "sustained_climb_rate_mps")
        );
    }

    #[test]
    fn test_camp_smoke_endurance_ends_by_battery_cutoff() {
        let o = out_of(TestId::Endurance);
        assert_eq!(o.outcome, Outcome::Completed);
        let d = metric(o, "duration_s");
        assert!(d > 30.0 && d < 200.0, "{d}");
        assert!(metric(o, "final_soc") < 0.04 && metric(o, "loops_completed") >= 1.0);
        // coulomb cross-check by hand: energy = charge used x mean pack voltage, with the charge from the state of charge drop of
        // the 5 Ah pack (0.05 at the start); the mean pack voltage over this low-battery run is between 12.5 and 15 V
        let charge_ah = (0.05 - metric(o, "final_soc")) * 5.0;
        let energy = metric(o, "energy_used_wh");
        assert!(
            energy > 12.5 * charge_ah && energy < 15.0 * charge_ah,
            "{energy} Wh for {charge_ah} Ah"
        );
        // the run ends after the battery has stayed in cutoff for the confirmation time (1 s in the smoke definitions)
        assert!(
            near(metric(o, "final_cutoff_run_s"), 1.0, 0.01),
            "{}",
            metric(o, "final_cutoff_run_s")
        );
        // loops: a 10 m square is 40 m; with the first leg starting at the origin the count is floor(distance / 40) within one leg
        let (loops, dist) = (metric(o, "loops_completed"), metric(o, "distance_flown_m"));
        assert!(
            loops * 40.0 - 10.0 <= dist && dist <= (loops + 1.0) * 40.0 + 20.0,
            "{loops} loops for {dist} m"
        );
        assert!(
            o.events
                .iter()
                .any(|e| e.name == "axis_saturated" && e.first_t_s.is_some())
        );
        let cutoff = o
            .events
            .iter()
            .find(|e| e.name == "battery_cutoff")
            .unwrap();
        assert!(cutoff.first_t_s.is_some());
    }

    #[test]
    fn test_camp_smoke_thermal_soak_reports_limits_and_peaks() {
        let o = out_of(TestId::ThermalSoak);
        // smoke definition: 20 s window, 5 K/min: the detector fires at 21 s, long before the 60 s limit
        assert_eq!(o.outcome, Outcome::SteadyState);
        assert!(
            metric(o, "duration_s") < 40.0
                && metric(o, "final_max_temperature_rate_k_per_min") < 5.0
        );
        assert!(
            metric(o, "winding_limit_c") == 120.0 && metric(o, "time_to_winding_limit_s") == -1.0
        );
        assert!(metric(o, "peak_winding_c") > 20.0 && metric(o, "peak_winding_c") < 30.0);
    }

    #[test]
    fn test_camp_smoke_files_name_the_drag_model_and_the_turbulence_label() {
        let files = &smoke().1;
        let get = |p: &str| {
            files
                .iter()
                .find(|(n, _)| n == p)
                .map(|(_, t)| t.as_str())
                .unwrap()
        };
        for id in TestId::ALL {
            let m = get(&format!("{}/manifest.json", id.name()));
            assert!(
                m.contains("\"drag_model\": \"analytic_legacy\"")
                    && m.contains("Dryden (MIL-HDBK-1797 low-altitude form), UNVERIFIED"),
                "{}",
                id.name()
            );
            assert!(
                m.contains("\"git_hash\": \"abc123\"")
                    && m.contains("\"parts_db_version\": \"0.1.0\"")
                    && m.contains("tmotor_mn3508_kv700")
            );
            assert!(
                m.contains("\"core_interface_version\": 14") && m.contains("\"definition\": {")
            );
            serde_json::from_str::<serde_json::Value>(m).unwrap();
            let r: serde_json::Value =
                serde_json::from_str(get(&format!("{}/result.json", id.name()))).unwrap();
            assert_eq!(r["drag_model"], "analytic_legacy");
            serde_json::from_str::<serde_json::Value>(get(&format!("{}/events.json", id.name())))
                .unwrap();
        }
        let summary = get("summary.csv");
        assert_eq!(summary.lines().count(), 9); // header + 8 tests
        let camp: serde_json::Value = serde_json::from_str(get("campaign_result.json")).unwrap();
        assert_eq!(camp["tests"].as_array().unwrap().len(), 8);
        assert!(
            files
                .iter()
                .all(|(_, t)| !t.contains("NaN") && !t.contains("inf"))
        );
        // no wall-clock or machine-specific text
        assert!(files.iter().all(|(_, t)| !t.contains("/home/")));
    }

    #[test]
    fn test_camp_the_campaign_is_identical_for_one_thread_and_when_repeated() {
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        // two quick tests, one thread, twice: bytes equal to the 3-thread smoke run of the same tests
        let inputs = smoke_inputs(
            &build,
            &cond,
            &defs,
            vec![TestId::Climb, TestId::ThermalSoak],
        );
        let one = render_outputs(&inputs, &run_campaign(&inputs, 1).unwrap()).unwrap();
        let again = render_outputs(&inputs, &run_campaign(&inputs, 1).unwrap()).unwrap();
        assert_eq!(one, again);
        for (name, text) in &one {
            if name.starts_with("climb/") || name.starts_with("thermal_soak/") {
                let reference = &smoke().1.iter().find(|(n, _)| n == name).unwrap().1;
                assert_eq!(text, reference, "{name}");
            }
        }
    }

    #[test]
    fn test_camp_hover_in_a_north_wind_leans_into_the_wind_and_the_downwind_motors_work_harder() {
        // Wind FROM the north (wind_from_deg 0) blows TOWARD the south: NED vector (-8, 0, 0), checked first. Holding position, the
        // vehicle (nose north, yaw 0) must tilt its thrust vector toward the UPWIND side (north = forward) to cancel the drag: nose
        // DOWN, i.e. pitch < 0 (aerospace ZYX, + = nose up), roll ~ 0. A nose-down moment needs the REAR (downwind) motors to thrust
        // more: RR, RL (indices 2, 3 in the mixer order FR, FL, RR, RL) spin faster than FR, FL (0, 1).
        // Force balance (hand): T cos(tilt) = m g and T sin(tilt) = D, so total thrust = m g / cos(tilt) and tan(tilt) = D / (m g).
        let build = ref_build();
        let mut cond = std_conditions();
        cond.wind_speed_mps = 8.0;
        cond.wind_from_deg = 0.0;
        let w = cond.mean_wind_ned(8.0);
        assert!(
            w[0] == -8.0 && w[1].abs() < 1e-12,
            "wind from north is a -x (toward south) vector"
        );
        let mut sim = Sim::new(&build, &cond, 20.0, 0, None).unwrap();
        let sp = Setpoint::Position {
            ned_m: [0.0, 0.0, -20.0],
            yaw_rad: 0.0,
        };
        let steps = 20_000; // 40 s: the position loop has settled
        let (mut tilt, mut pitch, mut roll, mut thrust) = (0.0, 0.0, 0.0, 0.0);
        let (mut rpm, mut n) = ([0.0_f64; 4], 0.0);
        let mut peak_rear_minus_front = f64::MIN; // rpm, over the first 8 s (the transient that tips the vehicle)
        for k in 0..steps {
            let f = sim.step(&sp).unwrap();
            if k < 4000 {
                let d = 0.5
                    * (f.tel.motor_rpm[2] + f.tel.motor_rpm[3]
                        - f.tel.motor_rpm[0]
                        - f.tel.motor_rpm[1]);
                peak_rear_minus_front = peak_rear_minus_front.max(d);
            }
            if k >= steps - 500 {
                tilt += f.tilt_rad;
                pitch += f.pitch_rad;
                roll += f.roll_rad;
                thrust += f.tel.motor_thrust_n.iter().sum::<f64>();
                for (a, b) in rpm.iter_mut().zip(f.tel.motor_rpm.iter()) {
                    *a += *b;
                }
                n += 1.0;
            }
        }
        let (tilt, pitch, roll, thrust) = (tilt / n, pitch / n, roll / n, thrust / n);
        let rpm = rpm.map(|r| r / n);
        let weight_n = build.vehicle.mass_kg * 9.80665;
        // tilt sign and size: leaning into the wind, more than a degree and less than the 30 deg class limit
        assert!(pitch < 0.0, "nose down into a north wind, pitch {pitch}");
        assert!(
            roll.abs() < 0.2 * pitch.abs(),
            "no sideways lean for a head-on wind, roll {roll}"
        );
        assert!(
            tilt > 1.0_f64.to_radians() && tilt < 30.0_f64.to_radians(),
            "tilt {tilt}"
        );
        // force balance with the model's own thrust: T cos(tilt) = m g (1 %: the loop is settled, not exactly still)
        assert!(
            (thrust * tilt.cos() - weight_n).abs() < 0.01 * weight_n,
            "thrust {thrust} N, tilt {tilt} rad, weight {weight_n} N"
        );
        // the drag the model applied is T sin(tilt): tan(tilt) = D / (m g)
        let drag_n = thrust * tilt.sin();
        assert!((drag_n / weight_n - tilt.tan()).abs() < 0.02 * tilt.tan());
        // motor order. The tipping moment is only needed while the vehicle PITCHES DOWN: during that transient the rear (downwind) motors
        // spin faster than the front ones. Once the tilt is held the moments balance and the four rpm are nearly equal (the drag acts
        // close to the centre of gravity), so the steady-state ordering is not asserted beyond "not reversed".
        assert!(
            peak_rear_minus_front > 10.0,
            "rear - front peak {peak_rear_minus_front} rpm"
        );
        assert!(
            rpm[2] + rpm[3] >= rpm[0] + rpm[1] - 1e-6,
            "steady-state rpm FR FL RR RL = {rpm:?}"
        );
        println!(
            "wind test: pitch {:.2} deg, roll {:.2} deg, tilt {:.2} deg, T {thrust:.2} N, W {weight_n:.2} N, D {drag_n:.2} N, rpm {rpm:?}, transient rear-front peak {peak_rear_minus_front:.1} rpm",
            pitch.to_degrees(),
            roll.to_degrees(),
            tilt.to_degrees()
        );
    }

    #[test]
    fn test_camp_planned_sim_time_is_the_upper_bound_of_each_tests_runs() {
        // Hand values from campaign/test_definitions.json (the denominator of the progress percentage; not a result):
        // hover: (settle 10 + hold 30) s x (ceil(log2(8 / 0.02)) = 9, + 2) = 40 x 11 = 440
        // wind: 40 x (levels 0, 2 .. 20 = 11, + bisection ceil(log2(2 / 0.25)) = 3, + 2 = 5) = 40 x 16 = 640
        // climb 5 + 30; endurance 10 + 3600; thermal 7200; max speed (8 + 4) x (levels 2, 5 .. 38 = 13, + ceil(log2(3 / 0.5)) = 3, + 2) = 12 x 18 = 216
        // sprint 5 + 10; punch-out 5 + 3
        let defs = parse_definitions(&read("campaign/test_definitions.json")).unwrap();
        let p = |id| planned_sim_s(&defs, id);
        assert_eq!(p(TestId::HoverPayload), 440.0);
        assert_eq!(p(TestId::WindGust), 640.0);
        assert_eq!(p(TestId::Climb), 35.0);
        assert_eq!(p(TestId::Endurance), 3610.0);
        assert_eq!(p(TestId::ThermalSoak), 7200.0);
        assert_eq!(p(TestId::MaxSpeed), 216.0);
        assert_eq!(p(TestId::FpvSprint), 15.0);
        assert_eq!(p(TestId::FpvPunchout), 8.0);
    }

    #[test]
    fn test_camp_progress_hook_observes_without_changing_any_result_byte() {
        // The hook sees each finished test once, in a growing list; the partial render of a finished set has the same bytes per
        // test as the full render; and the results equal a run without the hook.
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let inputs = smoke_inputs(&build, &cond, &defs, vec![TestId::Climb, TestId::FpvSprint]);
        let seen = std::sync::Mutex::new(Vec::new());
        let partial = std::sync::Mutex::new(Vec::new());
        let with = run_campaign_with(&inputs, 2, &|i, so_far| {
            seen.lock()
                .unwrap()
                .push((i, so_far.iter().flatten().count()));
            if so_far.iter().all(Option::is_some) {
                partial
                    .lock()
                    .unwrap()
                    .push(render_partial(&inputs, so_far).unwrap());
            }
        })
        .unwrap();
        let mut counts: Vec<usize> = seen.lock().unwrap().iter().map(|(_, n)| *n).collect();
        counts.sort_unstable();
        assert_eq!(
            counts,
            vec![1, 2],
            "one call per finished test, the list grows"
        );
        let without = run_campaign(&inputs, 2).unwrap();
        let full = render_outputs(&inputs, &without).unwrap();
        assert_eq!(
            render_outputs(&inputs, &with).unwrap(),
            full,
            "the hook changes no result"
        );
        let last = partial.lock().unwrap().pop().unwrap();
        assert_eq!(
            last, full,
            "the partial render of a finished campaign is the full render"
        );
        // one test alone: its own files are the same bytes as in the full render
        let one = render_partial(&inputs, &[Some(without[0].clone()), None]).unwrap();
        for (name, text) in one.iter().filter(|(n, _)| n.starts_with("climb/")) {
            assert_eq!(
                &full.iter().find(|(n, _)| n == name).unwrap().1,
                text,
                "{name}"
            );
        }
        assert!(one.iter().all(|(n, _)| !n.starts_with("fpv_sprint/")));
    }

    #[test]
    fn test_camp_a_failing_test_is_reported_as_an_error_and_the_rest_continue() {
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let inputs = smoke_inputs(
            &build,
            &cond,
            &defs,
            vec![TestId::Climb, TestId::ThermalSoak],
        );
        let mut results = vec![
            Err(CampaignError::Run("synthetic failure")),
            run_campaign(
                &smoke_inputs(&build, &cond, &defs, vec![TestId::ThermalSoak]),
                1,
            )
            .unwrap()
            .remove(0),
        ];
        let files = render_outputs(&inputs, &results).unwrap();
        let climb = &files
            .iter()
            .find(|(n, _)| n == "climb/result.json")
            .unwrap()
            .1;
        assert!(climb.contains("\"outcome\": \"ERROR\"") && climb.contains("synthetic failure"));
        assert!(files.iter().any(|(n, _)| n == "thermal_soak/result.json"));
        let summary = &files.iter().find(|(n, _)| n == "summary.csv").unwrap().1;
        assert!(summary.contains("climb,1,ERROR"));
        results.clear();
        assert!(
            run_campaign(&smoke_inputs(&build, &cond, &defs, vec![]), 4)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn test_camp_test_names_round_trip() {
        for t in TestId::ALL {
            assert_eq!(TestId::from_name(t.name()), Some(t));
        }
        assert_eq!(TestId::from_name("max_speed"), Some(TestId::MaxSpeed));
        assert_eq!(TestId::from_name("fpv_sprint"), Some(TestId::FpvSprint));
        assert_eq!(TestId::from_name("fpv_punchout"), Some(TestId::FpvPunchout));
        assert_eq!(TestId::from_name("nonsense"), None);
        let offsets: Vec<u64> = TestId::ALL.iter().map(|t| t.seed_offset()).collect();
        assert_eq!(
            offsets,
            vec![1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000]
        );
    }

    #[test]
    fn test_camp_wind_run_counts_an_altitude_loss_as_a_position_error() {
        // smoke conditions (battery at 5 %), no wind: 2.5 kg of payload holds (window altitude error 0.10 m), 3.0 kg sinks about 16 m
        // while the horizontal error stays 0 - only the altitude term of the criterion can fail it
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let mut c = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        c.payload_kg = 2.5;
        let ok = wind_run(&ref_build(), &c, &defs.wind_gust, 0.0, 0, &analytic_drag).unwrap();
        assert!(ok.verdict.pass, "{:?}", ok.verdict);
        c.payload_kg = 3.0;
        let sink = wind_run(&ref_build(), &c, &defs.wind_gust, 0.0, 0, &analytic_drag).unwrap();
        assert_eq!(sink.verdict.criterion, Some("position_error"));
        assert!(sink.obs.window.max_h_err_m < 0.01 && sink.obs.window.max_alt_err_m > 2.0);
    }

    #[test]
    fn test_camp_sustained_climb_rate_uses_the_middle_half() {
        // (time, rate) every 1 s: 0, 2, 4, 4, 4, 2 over a 6 s climb: the middle half is 1.5 s ..= 4.5 s = samples at 2, 3, 4 s -> mean 4
        let speeds = [
            (0.0, 0.0),
            (1.0, 2.0),
            (2.0, 4.0),
            (3.0, 4.0),
            (4.0, 4.0),
            (5.0, 2.0),
        ];
        assert!(near(sustained_rate(&speeds, 6.0), 4.0, 1e-12));
        assert_eq!(sustained_rate(&[], 6.0), 0.0);
        // the window edges are inclusive: t = 4 s total -> 1 s ..= 3 s -> samples at 1, 2, 3 s = (2 + 4 + 4) / 3
        assert!(near(sustained_rate(&speeds, 4.0), 10.0 / 3.0, 1e-12));
    }

    #[test]
    fn test_camp_window_min_reserve_is_the_minimum_of_the_rows() {
        // a row every step: the window's lowest thrust reserve must equal the lowest value in the written rows
        let mut sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        let mut obs = Observer::new(DT_S, f64::INFINITY, 0.98);
        obs.start_window();
        for _ in 0..30 {
            let f = sim
                .step(&Setpoint::Velocity {
                    ned_mps: [0.0, 0.0, -3.0],
                    yaw_rad: 0.0,
                })
                .unwrap();
            obs.observe(&sim, &f, None).unwrap();
        }
        let (csv, rows) = obs.csv();
        assert_eq!(rows, 30);
        let col = SERIES_COLUMNS
            .iter()
            .position(|c| *c == "thrust_reserve_ratio")
            .unwrap();
        let values: Vec<f64> = csv
            .lines()
            .skip(1)
            .map(|l| l.split(',').nth(col).unwrap().parse().unwrap())
            .collect();
        let lowest = values.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(
            values.iter().any(|v| *v > lowest + 1e-6),
            "the reserve must vary for this test to mean anything"
        );
        assert!(
            near(obs.window.min_reserve, lowest, 1e-9)
                && near(obs.peaks.min_reserve.value, lowest, 1e-9)
        );
    }

    // ------------------------------------------------------------------ drag sources

    /// A drag source that answers a fixed force, to test the bookkeeping without an ONNX Runtime.
    struct ConstDrag {
        force: [f32; 3],
        from_surrogate: bool,
        inside: bool,
    }

    impl DragSource for ConstDrag {
        fn label(&self) -> String {
            "cfd_surrogate:stub:mock".to_string()
        }

        fn force(
            &mut self,
            _a: &DroneState,
            _alt: f64,
            _env: &EnvironmentInput,
        ) -> Result<DragAnswer, CampaignError> {
            Ok(DragAnswer {
                force_ned_n: self.force,
                from_surrogate: self.from_surrogate,
                inside_envelope: self.inside,
            })
        }
    }

    fn mock(force: [f32; 3], from_surrogate: bool, inside: bool) -> Box<dyn DragSource> {
        Box::new(ConstDrag {
            force,
            from_surrogate,
            inside,
        })
    }

    #[test]
    fn test_camp_drag_counters_add_up() {
        let hold = hold_setpoint(20.0);
        let build = ref_build();
        let cond = std_conditions();
        // surrogate answers, all outside the validated envelope (as a stub model always is)
        let mut a =
            Sim::with_drag(&build, &cond, 20.0, 0, None, mock([0.0; 3], true, false)).unwrap();
        // surrogate answers inside the envelope
        let mut b =
            Sim::with_drag(&build, &cond, 20.0, 0, None, mock([0.0; 3], true, true)).unwrap();
        // no surrogate answer: the analytic fallback
        let mut c =
            Sim::with_drag(&build, &cond, 20.0, 0, None, mock([0.0; 3], false, false)).unwrap();
        let mut d = Sim::new(&build, &cond, 20.0, 0, None).unwrap();
        for _ in 0..200 {
            for s in [&mut a, &mut b, &mut c, &mut d] {
                s.step(&hold).unwrap();
            }
        }
        assert_eq!(
            (
                a.drag_surrogate_steps,
                a.drag_analytic_steps,
                a.drag_outside_envelope_steps
            ),
            (200, 0, 200)
        );
        assert_eq!(
            (
                b.drag_surrogate_steps,
                b.drag_analytic_steps,
                b.drag_outside_envelope_steps
            ),
            (200, 0, 0)
        );
        assert_eq!(
            (
                c.drag_surrogate_steps,
                c.drag_analytic_steps,
                c.drag_outside_envelope_steps
            ),
            (0, 200, 0)
        );
        assert_eq!(
            d.drag_stats(),
            DragStats {
                label: "analytic_legacy".into(),
                surrogate_steps: 0,
                analytic_steps: 200,
                outside_envelope_steps: 0
            }
        );
        assert_eq!(a.drag_stats().label, "cfd_surrogate:stub:mock");
    }

    #[test]
    fn test_camp_the_drag_source_force_is_what_acts_on_the_vehicle() {
        // a constant 10 N push toward the north on the 1.536 kg vehicle: after 25 steps (0.05 s) the extra velocity is F t / m = 0.3255 m/s
        // (the controller has barely begun to answer); the analytic drag of a vehicle at rest is zero, so the baseline is the reference
        let hold = hold_setpoint(20.0);
        let (build, cond) = (ref_build(), std_conditions());
        let mut pushed = Sim::with_drag(
            &build,
            &cond,
            20.0,
            0,
            None,
            mock([10.0, 0.0, 0.0], false, false),
        )
        .unwrap();
        let mut base = Sim::new(&build, &cond, 20.0, 0, None).unwrap();
        let (mut fp, mut fb) = (None, None);
        for _ in 0..25 {
            fp = Some(pushed.step(&hold).unwrap());
            fb = Some(base.step(&hold).unwrap());
        }
        let dv = fp.unwrap().velocity_ned_mps[0] - fb.unwrap().velocity_ned_mps[0];
        let hand = 10.0 * 0.05 / 1.536;
        assert!(near(dv, hand, 0.35 * hand), "{dv} vs {hand}");
    }

    #[test]
    fn test_camp_the_drag_label_and_counts_reach_the_files_only_for_a_surrogate() {
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let factory =
            || -> Result<Box<dyn DragSource>, CampaignError> { Ok(mock([0.0; 3], true, false)) };
        let mut inputs = smoke_inputs(&build, &cond, &defs, vec![TestId::Climb]);
        inputs.drag = Some(&factory);
        let results = run_campaign(&inputs, 1).unwrap();
        let files = render_outputs(&inputs, &results).unwrap();
        let get = |p: &str| {
            files
                .iter()
                .find(|(n, _)| n == p)
                .map(|(_, t)| t.clone())
                .unwrap()
        };
        let out = results[0].as_ref().unwrap();
        assert_eq!(
            (out.drag.analytic_steps, out.drag.surrogate_steps),
            (0, out.steps_total)
        );
        let result: serde_json::Value = serde_json::from_str(&get("climb/result.json")).unwrap();
        assert_eq!(result["drag_model"], "cfd_surrogate:stub:mock");
        assert_eq!(result["drag_steps"]["surrogate"], out.steps_total);
        assert_eq!(
            result["drag_steps"]["outside_validated_envelope"],
            out.steps_total
        );
        assert_eq!(result["drag_note"], "preliminary CFD, unvalidated");
        assert!(get("climb/manifest.json").contains("\"drag_model\": \"cfd_surrogate:stub:mock\""));
        assert!(
            get("campaign_result.json").contains("\"drag_model\": \"cfd_surrogate:stub:mock\"")
        );
        // a surrogate that had no answer for any step (all analytic fallback) is still named, with its counts
        let fallback =
            || -> Result<Box<dyn DragSource>, CampaignError> { Ok(mock([0.0; 3], false, false)) };
        let mut inputs2 = smoke_inputs(&build, &cond, &defs, vec![TestId::Climb]);
        inputs2.drag = Some(&fallback);
        let results2 = run_campaign(&inputs2, 1).unwrap();
        let files2 = render_outputs(&inputs2, &results2).unwrap();
        let r2: serde_json::Value = serde_json::from_str(
            &files2
                .iter()
                .find(|(n, _)| n == "climb/result.json")
                .unwrap()
                .1,
        )
        .unwrap();
        let steps2 = results2[0].as_ref().unwrap().steps_total;
        assert_eq!(r2["drag_model"], "cfd_surrogate:stub:mock");
        assert_eq!(
            (
                r2["drag_steps"]["surrogate"].as_u64(),
                r2["drag_steps"]["analytic_fallback"].as_u64()
            ),
            (Some(0), Some(steps2))
        );
        // the analytic run (the smoke goldens) has neither the counts nor the note
        let analytic = &smoke().1;
        let plain = &analytic
            .iter()
            .find(|(n, _)| n == "climb/result.json")
            .unwrap()
            .1;
        assert!(
            !plain.contains("drag_steps")
                && !plain.contains("drag_note")
                && plain.contains("\"drag_model\": \"analytic_legacy\"")
        );
    }

    fn note(status: &str, level: &'static str, load_error: Option<&str>) -> CfdNote {
        CfdNote {
            status: status.to_string(),
            level,
            kind: Some("stub".to_string()),
            model_sha256: Some("ab".repeat(32)),
            provenance: Some("SYNTHETIC formula, not CFD | NOT VALIDATED".to_string()),
            holdout_rms_n: Some("2.5e-9".to_string()),
            cfd_sanity_status: Some("NOT_RUN".to_string()),
            load_error: load_error.map(str::to_string),
        }
    }

    #[test]
    fn test_camp_cfd_note_json_has_fixed_keys_and_null_for_absent_values() {
        // keys in the written order; None -> null; text is JSON-escaped (the quote and backslash below)
        let n = CfdNote {
            status: "S \"q\"".to_string(),
            level: "warn",
            kind: None,
            model_sha256: None,
            provenance: Some("a\\b".to_string()),
            holdout_rms_n: None,
            cfd_sanity_status: None,
            load_error: None,
        };
        assert_eq!(
            n.to_json(),
            "{\"status\": \"S \\\"q\\\"\", \"level\": \"warn\", \"kind\": null, \"model_sha256\": null, \"provenance\": \"a\\\\b\", \"holdout_rms_n\": null, \"cfd_sanity_status\": null, \"load_error\": null}"
        );
    }

    #[test]
    fn test_camp_the_cfd_note_reaches_the_result_and_every_manifest_only_when_given() {
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let n = note(
            "CFD surrogate: SYNTHETIC (formula, not CFD)",
            "danger",
            None,
        );
        let mut inputs = smoke_inputs(&build, &cond, &defs, vec![TestId::Climb]);
        inputs.cfd_surrogate = Some(&n);
        let results = run_campaign(&inputs, 1).unwrap();
        let files = render_outputs(&inputs, &results).unwrap();
        let json = |p: &str| -> serde_json::Value {
            let t = &files.iter().find(|(n, _)| n == p).unwrap().1;
            serde_json::from_str(t).unwrap()
        };
        for file in ["campaign_result.json", "climb/manifest.json"] {
            let v = json(file);
            assert_eq!(
                v["cfd_surrogate"]["status"], "CFD surrogate: SYNTHETIC (formula, not CFD)",
                "{file}"
            );
            assert_eq!(v["cfd_surrogate"]["level"], "danger", "{file}");
            assert_eq!(v["cfd_surrogate"]["cfd_sanity_status"], "NOT_RUN", "{file}");
            // the drag model of an analytic run is untouched by a note
            assert_eq!(v["drag_model"], "analytic_legacy", "{file}");
        }
        // without a note the key is absent: the analytic goldens keep their exact bytes
        for (name, text) in &smoke().1 {
            assert!(!text.contains("cfd_surrogate"), "{name}");
        }
    }

    #[test]
    fn test_camp_a_failed_load_note_carries_its_reason_into_the_files() {
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let n = note(
            "CFD model failed to load: ONNX Runtime missing; results use analytic drag",
            "warn",
            Some("ONNX Runtime missing"),
        );
        let mut inputs = smoke_inputs(&build, &cond, &defs, vec![TestId::Climb]);
        inputs.cfd_surrogate = Some(&n);
        let results = run_campaign(&inputs, 1).unwrap();
        let files = render_outputs(&inputs, &results).unwrap();
        let t = &files
            .iter()
            .find(|(n, _)| n == "campaign_result.json")
            .unwrap()
            .1;
        let v: serde_json::Value = serde_json::from_str(t).unwrap();
        assert_eq!(v["cfd_surrogate"]["load_error"], "ONNX Runtime missing");
        assert_eq!(v["cfd_surrogate"]["level"], "warn");
    }

    #[test]
    fn test_camp_a_drag_factory_error_is_reported_as_the_tests_error() {
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let failing = || -> Result<Box<dyn DragSource>, CampaignError> {
            Err(CampaignError::Model("aero model load"))
        };
        let mut inputs = smoke_inputs(
            &build,
            &cond,
            &defs,
            vec![TestId::Climb, TestId::ThermalSoak],
        );
        inputs.drag = Some(&failing);
        let results = run_campaign(&inputs, 2).unwrap();
        assert!(
            results
                .iter()
                .all(|r| matches!(r, Err(CampaignError::Model("aero model load"))))
        );
        let files = render_outputs(&inputs, &results).unwrap();
        assert!(
            files
                .iter()
                .find(|(n, _)| n == "climb/result.json")
                .unwrap()
                .1
                .contains("aero model load")
        );
    }

    #[test]
    fn test_camp_search_tests_report_the_drag_of_the_run_they_report() {
        // hover/payload and wind/gust run many simulations; the drag bookkeeping of the REPORTED run reaches the output
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let factory =
            || -> Result<Box<dyn DragSource>, CampaignError> { Ok(mock([0.0; 3], true, true)) };
        let hover = run_hover_payload(&build, &cond, &defs.hover_payload, 1000, &factory).unwrap();
        let wind = run_wind_gust(&build, &cond, &defs.wind_gust, 2000, &factory).unwrap();
        for out in [&hover, &wind] {
            assert_eq!(out.drag.label, "cfd_surrogate:stub:mock", "{}", out.id);
            assert!(
                out.drag.surrogate_steps > 0 && out.drag.surrogate_steps <= out.steps_total,
                "{}",
                out.id
            );
            assert_eq!(
                (out.drag.analytic_steps, out.drag.outside_envelope_steps),
                (0, 0),
                "{}",
                out.id
            );
        }
    }

    #[test]
    fn test_camp_loss_columns_are_the_sums_of_the_telemetry_losses() {
        // the three columns are the sums over the four motors / ESCs of the telemetry loss arrays, written with the same number format
        let mut sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        let mut obs = Observer::new(0.002, f64::INFINITY, 0.98);
        let f = sim.step(&hold_setpoint(20.0)).unwrap();
        obs.observe(&sim, &f, Some([0.0, 0.0, -20.0])).unwrap();
        let (csv, _) = obs.csv();
        let col = |name: &str| SERIES_COLUMNS.iter().position(|c| *c == name).unwrap();
        let row: Vec<&str> = csv.lines().nth(1).unwrap().split(',').collect();
        let want = |a: &[f64; 4]| crate::textout::num(a[0] + a[1] + a[2] + a[3]).unwrap();
        assert_eq!(row[col("motor_copper_w")], want(&f.tel.motor_copper_loss_w));
        assert_eq!(row[col("motor_noload_w")], want(&f.tel.motor_noload_loss_w));
        assert_eq!(row[col("esc_loss_w")], want(&f.tel.esc_loss_w));
        // they sit right after battery_w, and a hovering motor dissipates something
        assert_eq!(col("motor_copper_w"), col("battery_w") + 1);
        assert!(row[col("motor_copper_w")].parse::<f64>().unwrap() > 0.0);
    }

    #[test]
    fn test_camp_condition_set_name_parse_and_manifest() {
        let named = COND_JSON.replace(r#""ambient_c""#, r#""name":"hot_day","ambient_c""#);
        assert_eq!(
            parse_conditions(&named).unwrap().name.as_deref(),
            Some("hot_day")
        );
        assert_eq!(parse_conditions(COND_JSON).unwrap().name, None);
        for bad in [
            "",
            "Hot",
            "a b",
            "a/b",
            "x1234567890123456789012345678901234567890",
        ] {
            let j = COND_JSON.replace(r#""ambient_c""#, &format!(r#""name":"{bad}","ambient_c""#));
            assert!(parse_conditions(&j).is_err(), "{bad}");
        }
        let build = ref_build();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let mut cond = std_conditions();
        let render = |c: &Conditions| {
            let inputs = smoke_inputs(&build, c, &defs, vec![TestId::Climb]);
            let results = run_campaign(&inputs, 1).unwrap();
            let files = render_outputs(&inputs, &results).unwrap();
            files
                .into_iter()
                .find(|(n, _)| n == "climb/manifest.json")
                .unwrap()
                .1
        };
        assert!(!render(&cond).contains("condition_set"));
        cond.name = Some("hot_day".to_string());
        assert!(
            render(&cond)
                .contains("\"conditions\": {\"condition_set\": \"hot_day\", \"ambient_c\": 20")
        );
    }

    #[test]
    fn test_camp_roll_pitch_from_known_quaternions() {
        // rotation about X by 30 deg: q = (sin 15, 0, 0, cos 15) -> roll 30, pitch 0
        let h = 15.0_f64.to_radians();
        let (r, p) = roll_pitch_rad(&DQuat::from_xyzw(h.sin(), 0.0, 0.0, h.cos()));
        assert!(near(r.to_degrees(), 30.0, 1e-9) && near(p, 0.0, 1e-12));
        // rotation about Y by 20 deg: q = (0, sin 10, 0, cos 10) -> pitch 20, roll 0
        let h = 10.0_f64.to_radians();
        let (r, p) = roll_pitch_rad(&DQuat::from_xyzw(0.0, h.sin(), 0.0, h.cos()));
        assert!(near(p.to_degrees(), 20.0, 1e-9) && near(r, 0.0, 1e-12));
        // pure yaw of 90 deg (about Z): roll and pitch stay zero
        let h = 45.0_f64.to_radians();
        let (r, p) = roll_pitch_rad(&DQuat::from_xyzw(0.0, 0.0, h.sin(), h.cos()));
        assert!(near(r, 0.0, 1e-12) && near(p, 0.0, 1e-12));
        // the identity, and a pitch of exactly 90 deg whose asin argument rounds past 1 must not give NaN
        let (r, p) = roll_pitch_rad(&DQuat::IDENTITY);
        assert!(near(r, 0.0, 0.0) && near(p, 0.0, 0.0));
        let h = 45.0_f64.to_radians();
        let (_, p) = roll_pitch_rad(&DQuat::from_xyzw(0.0, h.sin(), 0.0, h.cos()));
        assert!(near(p.to_degrees(), 90.0, 1e-6));
    }

    #[test]
    fn test_camp_series_has_roll_and_pitch_columns_after_yaw() {
        let yaw = SERIES_COLUMNS.iter().position(|c| *c == "yaw_deg").unwrap();
        assert_eq!(SERIES_COLUMNS[yaw + 1], "roll_deg");
        assert_eq!(SERIES_COLUMNS[yaw + 2], "pitch_deg");
        // a hovering vehicle at rest is level: both within a fraction of a degree on the first row
        let mut sim = Sim::new(&ref_build(), &std_conditions(), 20.0, 0, None).unwrap();
        let mut obs = Observer::new(0.002, f64::INFINITY, 0.98);
        let f = sim.step(&hold_setpoint(20.0)).unwrap();
        obs.observe(&sim, &f, Some([0.0, 0.0, -20.0])).unwrap();
        let row: Vec<f64> = obs
            .csv()
            .0
            .lines()
            .nth(1)
            .unwrap()
            .split(',')
            .map(|c| c.parse().unwrap())
            .collect();
        assert!(row[yaw + 1].abs() < 0.5 && row[yaw + 2].abs() < 0.5);
    }

    // ------------------------------------------------------------------ tests 6 to 8: max speed, FPV sprint, FPV punch-out

    #[test]
    fn test_camp_speed_verdict_priority_and_boundaries() {
        // command 10, fraction 0.9 -> the mean must reach 9.0; altitude error limit 2.0; reserve limit 1.0
        let v = |unstable, mean, alt, reserve| {
            speed_verdict(unstable, mean, 10.0, 0.9, alt, 2.0, reserve, 1.0)
        };
        assert!(
            v(false, 9.0, 2.0, 1.0).pass,
            "every limit exactly met passes"
        );
        assert_eq!(v(false, 8.99, 0.0, 5.0).criterion, Some("speed_shortfall"));
        assert_eq!(v(false, 10.0, 2.01, 5.0).criterion, Some("altitude_error"));
        assert_eq!(v(false, 10.0, 0.0, 0.99).criterion, Some("thrust_reserve"));
        assert_eq!(v(true, 10.0, 0.0, 5.0).criterion, Some("instability"));
        // priority when several fail: instability, then altitude, then reserve, then speed
        assert_eq!(v(true, 1.0, 9.0, 0.1).criterion, Some("instability"));
        assert_eq!(v(false, 1.0, 9.0, 0.1).criterion, Some("altitude_error"));
        assert_eq!(v(false, 1.0, 0.0, 0.1).criterion, Some("thrust_reserve"));
    }

    #[test]
    fn test_camp_time_to_speed_and_terminal_mean_by_hand() {
        // (t, speed): 0 s 0, 1 s 5, 2 s 12, 3 s 21 m/s. The first SAMPLE at or above the threshold counts (no interpolation).
        let s = [(0.0, 0.0), (1.0, 5.0), (2.0, 12.0), (3.0, 21.0)];
        assert_eq!(time_to_speed(&s, 10.0), 2.0);
        assert_eq!(time_to_speed(&s, 12.0), 2.0);
        assert_eq!(time_to_speed(&s, 21.0), 3.0);
        assert_eq!(time_to_speed(&s, 0.0), 0.0);
        assert_eq!(time_to_speed(&s, 21.5), -1.0);
        assert_eq!(time_to_speed(&[], 1.0), -1.0);
        // values 1..5 at t = 0..4: the last second holds t = 3 and 4 -> (4 + 5) / 2 = 4.5; span 0 -> only the last sample
        let v = [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 5.0)];
        assert_eq!(terminal_mean(&v, 1.0), 4.5);
        assert_eq!(terminal_mean(&v, 0.0), 5.0);
        assert_eq!(terminal_mean(&v, 100.0), 3.0);
        assert_eq!(terminal_mean(&[], 1.0), 0.0);
    }

    #[test]
    fn test_camp_speed_definitions_are_validated() {
        let text = read("campaign/smoke_definitions.json");
        let bad = |from: &str, to: &str| {
            assert!(text.contains(from), "{from}");
            parse_definitions(&text.replace(from, to))
        };
        assert!(parse_definitions(&text).is_ok());
        // boundaries that ARE allowed: fraction exactly 1, window exactly the hold time (3.0)
        assert!(bad("\"speed_fraction_min\": 0.9", "\"speed_fraction_min\": 1.0").is_ok());
        assert!(
            bad(
                "\"eval_window_s\": 1.5,\n    \"speed",
                "\"eval_window_s\": 3.0,\n    \"speed"
            )
            .is_ok()
        );
        for (from, to, which) in [
            // max equal to the start speed 2: nothing to ramp, refused
            (
                "\"max_speed_mps\": 40.0",
                "\"max_speed_mps\": 2.0",
                "max_speed",
            ),
            (
                "\"speed_fraction_min\": 0.9",
                "\"speed_fraction_min\": 1.5",
                "max_speed",
            ),
            (
                "\"speed_fraction_min\": 0.9",
                "\"speed_fraction_min\": 0.0",
                "max_speed",
            ),
            (
                "\"max_speed_mps\": 40.0",
                "\"max_speed_mps\": 1.0",
                "max_speed",
            ), // below the start speed 2
            (
                "\"tolerance_mps\": 2.0",
                "\"tolerance_mps\": 5.0",
                "max_speed",
            ), // not below the step 5
            (
                "\"min_thrust_reserve_ratio\": 1.0",
                "\"min_thrust_reserve_ratio\": 0.5",
                "max_speed",
            ),
            (
                "\"eval_window_s\": 1.5,\n    \"speed",
                "\"eval_window_s\": 9.0,\n    \"speed",
                "max_speed",
            ), // longer than the hold
            ("\"sprint_s\": 4.0", "\"sprint_s\": 0.0", "fpv_sprint"),
            (
                "\"punchout_s\": 1.5",
                "\"punchout_s\": -1.0",
                "fpv_punchout",
            ),
            (
                "\"gain_target_m\": 5.0",
                "\"gain_target_m\": 0.0",
                "fpv_punchout",
            ),
        ] {
            match bad(from, to) {
                Err(CampaignError::Invalid(m)) => assert!(m.contains(which), "{from}: {m}"),
                other => panic!("{from} -> {to}: {other:?}"),
            }
        }
        assert!(matches!(
            parse_definitions(
                &text.replace("\"sprint_s\": 4.0,", "\"sprint_s\": 4.0, \"surprise\": 1,")
            ),
            Err(CampaignError::Parse)
        ));
        let missing = text.replace("\"fpv_punchout\"", "\"fpv_punchout_x\"");
        assert!(matches!(
            parse_definitions(&missing),
            Err(CampaignError::Parse)
        ));
    }

    #[test]
    fn test_camp_max_speed_of_the_reference_quad_is_limit_bound_by_the_class() {
        // class quad_standard demands at most 12 m/s (parts_db/controller_classes.json): levels 2, 7, 12 all pass, so the class ends the search
        let o = out_of(TestId::MaxSpeed);
        assert_eq!(o.outcome, Outcome::NotReachedWithinBound);
        assert_eq!(metric(o, "class_speed_limit_mps"), 12.0);
        assert_eq!(metric(o, "max_passing_speed_mps"), 12.0);
        assert!(
            o.notes
                .iter()
                .any(|n| n.contains("LIMIT-BOUND") && n.contains("12"))
        );
        assert!(o.notes.iter().any(|n| n.contains("legacy analytic drag")));
        assert!(o.breaking.is_none());
        let m = metric(o, "window_mean_speed_mps");
        assert!((0.9 * 12.0..=12.5).contains(&m), "mean speed {m}");
    }

    #[test]
    fn test_camp_fpv_manoeuvres_on_the_reference_quad_by_bands() {
        // bands are engineering choices around the measured values (standard conditions, smoke definitions); the quad's class speed demand is 12 m/s
        let s = out_of(TestId::FpvSprint);
        assert_eq!(s.outcome, Outcome::Completed);
        assert_eq!(metric(s, "speed_demand_mps"), 12.0);
        // measured 12.0 m/s terminal for the 12 m/s demand: the sprint demands the class speed, not a fraction of it
        assert!(
            (10.0..=12.5).contains(&metric(s, "terminal_speed_mps"))
                && metric(s, "peak_speed_mps") >= metric(s, "terminal_speed_mps") - 0.2
        );
        assert_eq!(
            metric(s, "time_to_30_mps_s"),
            -1.0,
            "never reaches 30 m/s with a 12 m/s demand"
        );
        assert!(
            s.notes.iter().any(|n| n.contains("-1 means"))
                && s.notes.iter().any(|n| n.contains("legacy analytic drag"))
        );
        let p = out_of(TestId::FpvPunchout);
        assert_eq!(p.outcome, Outcome::Completed);
        // full throttle (measured about 20 m/s after 1.5 s): half throttle could not climb like this
        assert!(
            metric(p, "altitude_gain_m") > 10.0
                && metric(p, "peak_climb_rate_mps") > 10.0
                && metric(p, "final_climb_rate_mps") > 10.0
        );
        // +5 m at about 20 m/s after the first fraction of a second: reached, and inside the 1.5 s manoeuvre
        let t_gain = metric(p, "time_to_gain_target_s");
        assert!(t_gain > 0.0 && t_gain < 1.5, "{t_gain}");
        assert!(
            p.metrics.iter().all(|(_, v)| v.is_finite())
                && s.metrics.iter().all(|(_, v)| v.is_finite())
        );
        assert!(p.notes.iter().any(|n| n.contains("acro mode")));
    }

    #[test]
    fn test_camp_the_fpv_build_flies_the_three_speed_tests_with_the_known_drag_limit() {
        // FPV golden build, STANDARD conditions (full battery), smoke definitions. Wide bands: the legacy drag under-predicts forward speed.
        let build = load_build(&golden_dir("fpv_5in")).unwrap();
        let cond = parse_conditions(&read("campaign/conditions_standard.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let drag: &DragFactory = &analytic_drag;
        let ms = run_max_speed(&build, &cond, &defs.max_speed, 6000, drag).unwrap();
        assert_eq!(metric(&ms, "class_speed_limit_mps"), 35.0);
        let top = metric(&ms, "max_passing_speed_mps");
        assert!((5.0..=35.0).contains(&top), "max passing speed {top}");
        assert!(ms.notes.iter().any(|n| n.contains("under-predict")));
        let sp = run_fpv_sprint(&build, &cond, &defs.fpv_sprint, 7000, drag).unwrap();
        assert_eq!(metric(&sp, "speed_demand_mps"), 35.0);
        let term = metric(&sp, "terminal_speed_mps");
        assert!((8.0..=36.0).contains(&term), "terminal speed {term}");
        assert!(metric(&sp, "peak_tilt_deg") > 30.0, "the sprint tilts hard");
        assert!(
            metric(&sp, "altitude_sink_m") > 1.0,
            "the legacy drag sinks the pitched FPV (measured 4.8 m in the smoke run): positive sink"
        );
        assert!(metric(&sp, "time_to_10_mps_s") > 0.0, "10 m/s is reached");
        let po = run_fpv_punchout(&build, &cond, &defs.fpv_punchout, 8000, drag).unwrap();
        assert!(
            metric(&po, "final_climb_rate_mps") > 15.0 && metric(&po, "altitude_gain_m") > 15.0
        );
        // determinism: the same call twice gives bit-identical metrics
        let ms2 = run_max_speed(&build, &cond, &defs.max_speed, 6000, drag).unwrap();
        assert_eq!(
            ms.metrics
                .iter()
                .map(|(_, v)| v.to_bits())
                .collect::<Vec<_>>(),
            ms2.metrics
                .iter()
                .map(|(_, v)| v.to_bits())
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn test_camp_the_new_tests_do_not_depend_on_the_thread_count() {
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_smoke.json")).unwrap();
        let defs = parse_definitions(&read("campaign/smoke_definitions.json")).unwrap();
        let tests = vec![TestId::MaxSpeed, TestId::FpvSprint, TestId::FpvPunchout];
        let render = |threads: usize| {
            let inputs = smoke_inputs(&build, &cond, &defs, tests.clone());
            let results = run_campaign(&inputs, threads).unwrap();
            render_outputs(&inputs, &results).unwrap()
        };
        assert_eq!(render(1), render(3));
    }

    #[test]
    fn test_camp_ramp_step_lands_the_last_level_on_the_cap() {
        // (upper - start) / ceil((upper - start) / step): 10 / ceil(3.33) = 10 / 4 = 2.5; 33 / 11 = 3 exactly (no extra step for rounding);
        // a span smaller than the step is one step; no span or no step gives the step back
        assert_eq!(ramp_step(2.0, 3.0, 12.0), 2.5);
        assert_eq!(ramp_step(2.0, 3.0, 35.0), 3.0);
        assert_eq!(ramp_step(2.0, 5.0, 3.0), 1.0);
        assert_eq!(ramp_step(2.0, 3.0, 2.0), 3.0);
        assert_eq!(ramp_step(2.0, 0.0, 12.0), 0.0);
        // float noise must not add a level: 2.1 / 0.7 = 3.0000000000000004 in f64, still 3 steps of 0.7
        assert!((ramp_step(0.0, 0.7, 2.1) - 0.7).abs() < 1e-12);
        // the levels really end on the cap: 2, 4.5, 7, 9.5, 12
        let mut seen = Vec::new();
        let _ = ramp_and_refine(2.0, ramp_step(2.0, 3.0, 12.0), 12.0, 0.5, |v| {
            seen.push(v);
            Ok::<_, CampaignError>(Verdict {
                pass: true,
                criterion: None,
            })
        })
        .unwrap();
        assert_eq!(seen, vec![2.0, 4.5, 7.0, 9.5, 12.0]);
    }

    /// Zero-throttle acro setpoint: the motors stop and the vehicle falls.
    fn free_fall() -> Setpoint {
        Setpoint::Acro {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
        }
    }

    /// Number of steps until `Sim::step` refuses with `Model("ground")`; panics (test failure) on any other outcome or after `limit`.
    fn steps_to_ground(sim: &mut Sim, limit: u64) -> u64 {
        for k in 1..=limit {
            match sim.step(&free_fall()) {
                Ok(_) => {}
                Err(CampaignError::Model("ground")) => return k,
                Err(e) => panic!("unexpected error {e:?} at step {k}"),
            }
        }
        panic!("the vehicle never reached the ground in {limit} steps");
    }

    #[test]
    fn test_camp_ground_contact_is_a_crash_at_height_zero() {
        // Review finding 2. Free fall from 0.5 m with the motors off: h = 0.5 g t^2, t = sqrt(2 x 0.5 / 9.80665) = 0.31931 s,
        // which is 159.65 steps of 2 ms, so the 160th step is the first one at or below the ground. Drag at these speeds (<= 3.1 m/s)
        // is a few percent of weight at most, so it may delay contact by a few steps: the band is 150 to 175.
        let mut sim = Sim::new(&ref_build(), &std_conditions(), 0.5, 0, None).unwrap();
        let k = steps_to_ground(&mut sim, 1000);
        assert!((150..=175).contains(&k), "ground reached at step {k}");
        assert_eq!(
            loss_criterion(&CampaignError::Model("ground")),
            Some("crash")
        );
    }

    #[test]
    fn test_camp_ground_contact_at_a_high_site_stops_at_the_same_height() {
        // The old detector (air density failing at site + height < 0) would have let the vehicle fall 1500 m below the ground at a
        // 1500 m site. The same 0.5 m fall must end in the same band of steps (thrust is zero, density only changes drag slightly).
        let mut cond = std_conditions();
        cond.site_altitude_m = 1500.0;
        let mut sim = Sim::new(&ref_build(), &cond, 0.5, 0, None).unwrap();
        let k = steps_to_ground(&mut sim, 1000);
        assert!((150..=175).contains(&k), "ground reached at step {k}");
    }

    #[test]
    fn test_camp_a_manoeuvre_with_no_altitude_limit_still_reports_the_vehicle_lost() {
        // The sprint and punch-out observers have an infinite altitude-error limit, so before the fix a fall to the ground was only
        // caught by the atmosphere model. Free fall from 5 m: t = sqrt(2 x 5 / 9.80665) = 1.01 s, inside the 5 s manoeuvre.
        let m = fly_manoeuvre(
            &ref_build(),
            &std_conditions(),
            5.0,
            0.0,
            5.0,
            0.1,
            &free_fall(),
            0,
            &analytic_drag,
        )
        .unwrap();
        assert!(m.lost);
        let last_t = m.speeds.last().map(|(t, _)| *t).unwrap();
        assert!((0.9..=1.3).contains(&last_t), "lost after {last_t} s");
        let (out, _) = manoeuvre_output("fpv_sprint", "1", "sprint", m);
        assert_eq!(out.outcome, Outcome::Failed);
        assert!(out.notes.iter().any(|n| n.contains("vehicle was lost")));
    }

    #[test]
    fn test_camp_run_max_speed_flies_the_class_cap_with_the_full_definitions() {
        // Review finding 1: the raw step 3 flew 2, 5, 8, 11 and then called 12 m/s "limit-bound" without flying it.
        // Full definition: start 2, step 3, cap = class limit 12 m/s -> span 10, n = ceil(10 / 3) = 4, step 2.5, levels 2, 4.5, 7, 9.5, 12.
        // Each level is settle 8 s + hold 4 s = 12 s = 6000 steps of 2 ms, so 5 levels = 30000 steps (the wrong ramp gave 4 x 6000 = 24000).
        let build = ref_build();
        let cond = parse_conditions(&read("campaign/conditions_standard.json")).unwrap();
        let defs = parse_definitions(&read("campaign/test_definitions.json")).unwrap();
        assert_eq!(defs.max_speed.step_mps, 3.0);
        assert_eq!(build.controller.max_speed_h_mps, 12.0);
        let o = run_max_speed(&build, &cond, &defs.max_speed, 0, &analytic_drag).unwrap();
        assert_eq!(o.outcome, Outcome::NotReachedWithinBound);
        assert_eq!(metric(&o, "max_passing_speed_mps"), 12.0);
        assert_eq!(o.steps_total, 30_000);
        assert!(o.series.iter().any(|s| s.label == "pass_12.00mps"));
        assert!(
            o.notes
                .iter()
                .any(|n| n.contains("LIMIT-BOUND") && n.contains("12"))
        );
    }
}
