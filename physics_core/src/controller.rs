// Tier: F
//! Flight controller for the quad-X airframe (P3c): position / velocity ("level") modes and an acro mode.
//!
//! Pure and deterministic: no heap, no wall clock, no RNG. Frames: NED world, body X forward / Y right / Z down, quaternion
//! order (x, y, z, w) (ADR-001). Units are SI and carried in the names.
//!
//! Structure (outer to inner), all gains come from a `ControllerConfig` (per vehicle class, every value ASSUMED) and the
//! `VehicleConfig`:
//! 1. position P -> velocity setpoint (position mode only), limited in speed;
//! 2. velocity PI -> desired acceleration, limited in horizontal and vertical acceleration;
//! 3. thrust vector `F = m (a - g)`; its tilt from vertical is limited with the VERTICAL part kept first, so a speed demand can
//!    never take the lift away (valid at any tilt, no small-angle shortcut);
//! 4. desired attitude from the thrust direction and the yaw setpoint; the error is split into a tilt part and a yaw part
//!    (tilt is corrected first), P loop -> body-rate setpoint, clamped;
//! 5. rate PID per axis (derivative on the measurement, clamped integral, freeze on saturation) -> normalised roll / pitch / yaw;
//! 6. collective: `throttle = hover_throttle sqrt(T / (m g))` (thrust ~ duty^2), then clamped so the mixer never clips an axis
//!    command (headroom `authority x (|roll| + |pitch| + |yaw|)`); steps where that changed the value are counted.
//!
//! Acro mode: sticks -> body rates with expo, throttle passes through (still limited by the headroom), no angle limit.
//!
//! Rate-loop gains are NOT hand tuned: `Kp = bandwidth x I / tau_unit`, where `tau_unit` is the torque one unit of axis command
//! produces (mixer authority x thrust slope per duty x arm sum), so one formula serves every vehicle.
//!
//! Not modelled: the motor / ESC lag (the powertrain model follows the duty instantly), thrust dependence on battery sag (the
//! outer loops absorb it), sensor noise and delay, wind estimation. The mixer authority (0.2) and the yaw torque
//! `torque_coeff x thrust` have no source (existing TODOs in `mixer.rs`); the gains inherit that.

use crate::types::{ControlInputs, DroneState, VehicleConfig};
use glam::{DMat3, DQuat, DVec3};
use serde::Deserialize;
use std::fs;

/// Version of the controller config file layout. A file with another version is refused.
pub const CONTROLLER_CONFIG_VERSION: u32 = 1;
/// Standard gravity, m/s^2 (CGPM 1901). Same value as `mixer.rs` and `aero.rs`.
const GRAVITY_MPS2: f64 = 9.806_65;
/// Largest step the controller accepts, s. Same limit as the powertrain step.
const MAX_DT_S: f64 = 0.01;
/// Largest tilt limit accepted, rad (80 deg). Beyond this the desired-attitude construction loses its yaw reference.
const MAX_TILT_LIMIT_RAD: f64 = 1.396_263_401_595_463_7;
/// Motors of the quad-X; the yaw arm sum is `torque_coeff` times this.
const MOTOR_COUNT_F64: f64 = 4.0;
/// Smallest quaternion norm accepted from the estimate before normalising.
const MIN_QUAT_NORM: f64 = 0.5;
/// Division guard, dimensionless or in the unit of the denominator.
const EPS: f64 = 1e-9;
/// The vertical downward acceleration limit must stay below this fraction of gravity so the thrust vector keeps a lift component.
const MAX_DOWN_ACCEL_FRACTION_G: f64 = 0.9;

/// Why a controller call was refused.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControllerError {
    /// A config value is non-finite, out of range, or the version is wrong.
    InvalidConfig,
    /// The vehicle config cannot give gains (non-positive mass / inertia / arms, hover throttle outside (0, 1]).
    InvalidVehicle,
    /// `dt`, the estimate or the setpoint is non-finite or out of range.
    InvalidInput,
}

/// Why a config file could not be loaded.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControllerLoadError {
    /// The file could not be read.
    Io,
    /// The text is not valid JSON of the expected shape (unknown or missing key included).
    Parse,
    /// The values failed `validate_config`.
    Invalid,
}

/// Per-vehicle-class controller settings. Every value is ASSUMED, see `parts_db/controller_classes.json`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ControllerConfig {
    /// Must equal `CONTROLLER_CONFIG_VERSION`.
    pub config_version: u32,
    /// Duty that hovers THIS vehicle in the powertrain model (the builder's `hover_duty_estimate`), in (0, 1]. Thrust is taken as
    /// proportional to duty squared around it. `VehicleConfig::hover_throttle` (legacy static-propeller formula) is not used.
    pub hover_duty: f64,
    /// Largest thrust-vector tilt from vertical in level modes, rad, in (0, 80 deg].
    pub max_tilt_rad: f64,
    /// Largest roll / pitch rate setpoint in level modes, rad/s.
    pub max_rate_rps: f64,
    /// Largest yaw rate setpoint in level modes, rad/s.
    pub max_yaw_rate_rps: f64,
    /// Largest horizontal speed setpoint, m/s.
    pub max_speed_h_mps: f64,
    /// Largest horizontal acceleration command, m/s^2.
    pub max_accel_h_mps2: f64,
    /// Largest climb speed setpoint, m/s.
    pub max_climb_mps: f64,
    /// Largest descent speed setpoint, m/s.
    pub max_descent_mps: f64,
    /// Largest upward acceleration command, m/s^2.
    pub max_accel_up_mps2: f64,
    /// Largest downward acceleration command, m/s^2, below 0.9 g.
    pub max_accel_down_mps2: f64,
    /// Position error -> velocity setpoint gain, 1/s.
    pub position_kp_per_s: f64,
    /// Velocity error -> acceleration gain, 1/s.
    pub velocity_kp_per_s: f64,
    /// Velocity error integral -> acceleration gain, 1/s^2.
    pub velocity_ki_per_s2: f64,
    /// Largest |velocity integral term| per axis, m/s^2.
    pub accel_integral_limit_mps2: f64,
    /// Tilt error -> roll / pitch rate gain, 1/s.
    pub attitude_kp_per_s: f64,
    /// Yaw error -> yaw rate gain, 1/s.
    pub yaw_kp_per_s: f64,
    /// Closed-loop roll / pitch rate bandwidth the gains are derived for, rad/s.
    pub rate_bandwidth_rps: f64,
    /// Same for the yaw rate, rad/s.
    pub yaw_rate_bandwidth_rps: f64,
    /// Rate integral corner: `Ki = Kp x corner`, rad/s.
    pub rate_integral_corner_rps: f64,
    /// Rate derivative time: `Kd = Kp x Td`, s. 0 switches the D term off.
    pub rate_derivative_time_s: f64,
    /// Largest |rate integral term|, in units of the normalised axis command.
    pub rate_integral_limit: f64,
    /// Largest roll / pitch rate at full stick in acro mode, rad/s.
    pub acro_max_rate_rps: f64,
    /// Largest yaw rate at full stick in acro mode, rad/s.
    pub acro_max_yaw_rate_rps: f64,
    /// Expo in [0, 1]: `rate = max (expo s^3 + (1 - expo) s)`.
    pub acro_expo: f64,
    /// Authority of one normalised axis command in duty units. Must equal the 0.2 in `mixer::mix_controls`.
    pub mixer_axis_authority: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ConfigJson {
    config_version: u32,
    vehicle_class: String,
    hover_duty: f64,
    max_tilt_rad: f64,
    max_rate_rps: f64,
    max_yaw_rate_rps: f64,
    max_speed_h_mps: f64,
    max_accel_h_mps2: f64,
    max_climb_mps: f64,
    max_descent_mps: f64,
    max_accel_up_mps2: f64,
    max_accel_down_mps2: f64,
    position_kp_per_s: f64,
    velocity_kp_per_s: f64,
    velocity_ki_per_s2: f64,
    accel_integral_limit_mps2: f64,
    attitude_kp_per_s: f64,
    yaw_kp_per_s: f64,
    rate_bandwidth_rps: f64,
    yaw_rate_bandwidth_rps: f64,
    rate_integral_corner_rps: f64,
    rate_derivative_time_s: f64,
    rate_integral_limit: f64,
    acro_max_rate_rps: f64,
    acro_max_yaw_rate_rps: f64,
    acro_expo: f64,
    mixer_axis_authority: f64,
    /// Free-form sources and notes of the values; read by humans, ignored by the core.
    #[serde(default)]
    #[allow(dead_code)]
    provenance: Option<serde_json::Value>,
}

fn positive_finite(v: f64) -> bool {
    v.is_finite() && v > 0.0
}

/// Range checks of a controller config. All values finite, rates / gains / limits positive, tilt in (0, 80 deg], expo in [0, 1].
///
/// # Errors
/// `InvalidConfig` on the first violated rule.
pub fn validate_config(cfg: &ControllerConfig) -> Result<(), ControllerError> {
    let positives = [
        cfg.hover_duty,
        cfg.max_rate_rps,
        cfg.max_yaw_rate_rps,
        cfg.max_speed_h_mps,
        cfg.max_accel_h_mps2,
        cfg.max_climb_mps,
        cfg.max_descent_mps,
        cfg.max_accel_up_mps2,
        cfg.max_accel_down_mps2,
        cfg.position_kp_per_s,
        cfg.velocity_kp_per_s,
        cfg.velocity_ki_per_s2,
        cfg.accel_integral_limit_mps2,
        cfg.attitude_kp_per_s,
        cfg.yaw_kp_per_s,
        cfg.rate_bandwidth_rps,
        cfg.yaw_rate_bandwidth_rps,
        cfg.rate_integral_corner_rps,
        cfg.rate_integral_limit,
        cfg.acro_max_rate_rps,
        cfg.acro_max_yaw_rate_rps,
        cfg.mixer_axis_authority,
    ];
    let ok = cfg.config_version == CONTROLLER_CONFIG_VERSION
        && positives.iter().all(|&v| positive_finite(v))
        && cfg.hover_duty <= 1.0
        && cfg.max_tilt_rad.is_finite()
        && cfg.max_tilt_rad > 0.0
        && cfg.max_tilt_rad <= MAX_TILT_LIMIT_RAD
        && cfg.max_accel_down_mps2 < MAX_DOWN_ACCEL_FRACTION_G * GRAVITY_MPS2
        && cfg.rate_derivative_time_s.is_finite()
        && cfg.rate_derivative_time_s >= 0.0
        && cfg.acro_expo.is_finite()
        && (0.0..=1.0).contains(&cfg.acro_expo)
        && cfg.mixer_axis_authority <= 0.5;
    if ok {
        Ok(())
    } else {
        Err(ControllerError::InvalidConfig)
    }
}

/// Parses and validates a `controller_config.json` text.
///
/// # Errors
/// `Parse` for bad JSON, a missing or unknown key or an empty / over-long `vehicle_class`; `Invalid` when `validate_config` fails.
pub fn parse_controller_config(json: &str) -> Result<ControllerConfig, ControllerLoadError> {
    let j: ConfigJson = serde_json::from_str(json).map_err(|_| ControllerLoadError::Parse)?;
    if j.vehicle_class.is_empty() || j.vehicle_class.len() > 32 {
        return Err(ControllerLoadError::Parse);
    }
    let cfg = ControllerConfig {
        config_version: j.config_version,
        hover_duty: j.hover_duty,
        max_tilt_rad: j.max_tilt_rad,
        max_rate_rps: j.max_rate_rps,
        max_yaw_rate_rps: j.max_yaw_rate_rps,
        max_speed_h_mps: j.max_speed_h_mps,
        max_accel_h_mps2: j.max_accel_h_mps2,
        max_climb_mps: j.max_climb_mps,
        max_descent_mps: j.max_descent_mps,
        max_accel_up_mps2: j.max_accel_up_mps2,
        max_accel_down_mps2: j.max_accel_down_mps2,
        position_kp_per_s: j.position_kp_per_s,
        velocity_kp_per_s: j.velocity_kp_per_s,
        velocity_ki_per_s2: j.velocity_ki_per_s2,
        accel_integral_limit_mps2: j.accel_integral_limit_mps2,
        attitude_kp_per_s: j.attitude_kp_per_s,
        yaw_kp_per_s: j.yaw_kp_per_s,
        rate_bandwidth_rps: j.rate_bandwidth_rps,
        yaw_rate_bandwidth_rps: j.yaw_rate_bandwidth_rps,
        rate_integral_corner_rps: j.rate_integral_corner_rps,
        rate_derivative_time_s: j.rate_derivative_time_s,
        rate_integral_limit: j.rate_integral_limit,
        acro_max_rate_rps: j.acro_max_rate_rps,
        acro_max_yaw_rate_rps: j.acro_max_yaw_rate_rps,
        acro_expo: j.acro_expo,
        mixer_axis_authority: j.mixer_axis_authority,
    };
    validate_config(&cfg).map_err(|_| ControllerLoadError::Invalid)?;
    Ok(cfg)
}

/// Reads and parses a controller config file.
///
/// # Errors
/// `Io` if the file cannot be read, otherwise as `parse_controller_config`.
pub fn load_controller_config(path: &str) -> Result<ControllerConfig, ControllerLoadError> {
    let text = fs::read_to_string(path).map_err(|_| ControllerLoadError::Io)?;
    parse_controller_config(&text)
}

/// Config plus the vehicle-derived numbers: the controller is built once per vehicle and then stepped.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Tuning {
    /// The validated config.
    pub cfg: ControllerConfig,
    /// Vehicle mass, kg.
    pub mass_kg: f64,
    /// Duty that hovers the vehicle (`ControllerConfig::hover_duty`).
    pub hover_throttle: f64,
    /// Rate P gains, roll / pitch / yaw, in normalised command per rad/s.
    pub rate_kp: DVec3,
    /// Rate I gains, per rad (command per rad/s x s).
    pub rate_ki: DVec3,
    /// Rate D gains, command per rad/s^2.
    pub rate_kd: DVec3,
}

/// Torque, N m, that one unit of each axis command produces at hover.
///
/// `tau = authority x (dT/dduty) x arm_sum`, with `dT/dduty = 2 (m g / 4) / hover_duty` (thrust ~ duty^2). The yaw arm sum is
/// `torque_coeff x 4` (every motor adds or removes `torque_coeff x dT`).
///
/// # Errors
/// `InvalidVehicle` for non-positive mass, torque coefficient or arm sums (the hover duty is range-checked by `validate_config`).
pub fn unit_command_torques_nm(
    cfg: &ControllerConfig,
    vehicle: &VehicleConfig,
) -> Result<DVec3, ControllerError> {
    let hover = cfg.hover_duty;
    if !positive_finite(vehicle.mass_kg)
        || !positive_finite(hover)
        || !positive_finite(vehicle.torque_coeff)
    {
        return Err(ControllerError::InvalidVehicle);
    }
    let thrust_slope_n = 2.0 * (vehicle.mass_kg * GRAVITY_MPS2 / 4.0) / hover;
    let arm_x: f64 = vehicle.motor_pos_m.iter().map(|p| p[0].abs()).sum();
    let arm_y: f64 = vehicle.motor_pos_m.iter().map(|p| p[1].abs()).sum();
    let tau = DVec3::new(arm_y, arm_x, vehicle.torque_coeff * MOTOR_COUNT_F64)
        * (cfg.mixer_axis_authority * thrust_slope_n);
    if tau.to_array().iter().all(|&t| positive_finite(t)) {
        Ok(tau)
    } else {
        Err(ControllerError::InvalidVehicle)
    }
}

impl Tuning {
    /// Derives the rate gains `Kp = bandwidth x I / tau_unit`, `Ki = Kp x corner`, `Kd = Kp x Td` (see the module doc).
    ///
    /// # Errors
    /// `InvalidConfig` / `InvalidVehicle` as the two validators; non-positive inertia is `InvalidVehicle`.
    pub fn new(cfg: &ControllerConfig, vehicle: &VehicleConfig) -> Result<Self, ControllerError> {
        validate_config(cfg)?;
        let tau = unit_command_torques_nm(cfg, vehicle)?;
        let inertia = DVec3::from_array(vehicle.inertia_kgm2);
        if !inertia.to_array().iter().all(|&i| positive_finite(i)) {
            return Err(ControllerError::InvalidVehicle);
        }
        let bandwidth = DVec3::new(
            cfg.rate_bandwidth_rps,
            cfg.rate_bandwidth_rps,
            cfg.yaw_rate_bandwidth_rps,
        );
        let rate_kp = bandwidth * inertia / tau;
        Ok(Self {
            cfg: *cfg,
            mass_kg: vehicle.mass_kg,
            hover_throttle: cfg.hover_duty,
            rate_kp,
            rate_ki: rate_kp * cfg.rate_integral_corner_rps,
            rate_kd: rate_kp * cfg.rate_derivative_time_s,
        })
    }
}

/// What the controller reads from the vehicle.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Estimate {
    /// NED position, m.
    pub position_ned_m: [f64; 3],
    /// NED velocity, m/s.
    pub velocity_ned_mps: [f64; 3],
    /// Body -> NED quaternion (x, y, z, w).
    pub quat_xyzw: [f64; 4],
    /// Body rates p, q, r, rad/s.
    pub rates_rps: [f64; 3],
}

impl Estimate {
    /// Widens a `DroneState` (f32 velocity / attitude / rates) to the controller's f64 estimate.
    #[must_use]
    pub fn from_state(s: &DroneState) -> Self {
        let w = |v: f32| f64::from(v);
        Self {
            position_ned_m: s.position,
            velocity_ned_mps: s.velocity.map(w),
            quat_xyzw: s.orientation.map(w),
            rates_rps: s.angular_velocity.map(w),
        }
    }

    fn is_valid(&self) -> bool {
        let finite = |a: &[f64]| a.iter().all(|v| v.is_finite());
        finite(&self.position_ned_m)
            && finite(&self.velocity_ned_mps)
            && finite(&self.quat_xyzw)
            && finite(&self.rates_rps)
            && DQuat::from_array(self.quat_xyzw).length() >= MIN_QUAT_NORM
    }
}

/// What the pilot or the mission asks for.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Setpoint {
    /// Hold a NED position and a yaw angle (from north, + nose right).
    Position {
        /// Target NED position, m.
        ned_m: [f64; 3],
        /// Target yaw, rad.
        yaw_rad: f64,
    },
    /// Fly a NED velocity with a yaw angle.
    Velocity {
        /// Target NED velocity, m/s.
        ned_mps: [f64; 3],
        /// Target yaw, rad.
        yaw_rad: f64,
    },
    /// Rate mode: sticks in [-1, 1] (roll right +, pitch nose up +, yaw nose right +) and throttle in [0, 1].
    Acro {
        /// Roll stick.
        roll: f64,
        /// Pitch stick.
        pitch: f64,
        /// Yaw stick.
        yaw: f64,
        /// Collective throttle.
        throttle: f64,
    },
}

impl Setpoint {
    fn is_valid(&self) -> bool {
        let finite = |a: &[f64]| a.iter().all(|v| v.is_finite());
        match self {
            Self::Position { ned_m, yaw_rad } => finite(ned_m) && yaw_rad.is_finite(),
            Self::Velocity { ned_mps, yaw_rad } => finite(ned_mps) && yaw_rad.is_finite(),
            Self::Acro {
                roll,
                pitch,
                yaw,
                throttle,
            } => finite(&[*roll, *pitch, *yaw, *throttle]),
        }
    }
}

/// Integrators, the previous measured rate and the counters. Create with `ControllerState::new`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ControllerState {
    /// Velocity error integral per NED axis, m (error x s).
    pub velocity_integral: DVec3,
    /// Rate error integral per body axis, rad.
    pub rate_integral: DVec3,
    /// Previous measured body rates, rad/s.
    pub previous_rates_rps: DVec3,
    /// False until one step has run (no derivative before that).
    pub have_previous: bool,
    /// Steps taken.
    pub steps: u64,
    /// Steps on which the headroom clamp changed the collective.
    pub collective_limited_steps: u64,
    /// Steps on which any axis command reached +-1.
    pub axis_saturated_steps: u64,
    /// Steps on which the tilt limit cut the horizontal thrust.
    pub tilt_limited_steps: u64,
}

impl ControllerState {
    /// Zeroed state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            velocity_integral: DVec3::ZERO,
            rate_integral: DVec3::ZERO,
            previous_rates_rps: DVec3::ZERO,
            have_previous: false,
            steps: 0,
            collective_limited_steps: 0,
            axis_saturated_steps: 0,
            tilt_limited_steps: 0,
        }
    }
}

impl Default for ControllerState {
    fn default() -> Self {
        Self::new()
    }
}

/// One controller output in the units of `ControlInputs`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Output {
    /// Collective, 0..1.
    pub throttle: f64,
    /// Roll command, -1..1 (+ = right wing down).
    pub roll: f64,
    /// Pitch command, -1..1 (+ = nose up).
    pub pitch: f64,
    /// Yaw command, -1..1 (+ = nose right).
    pub yaw: f64,
    /// The headroom clamp changed the collective.
    pub collective_limited: bool,
    /// The tilt limit cut the horizontal thrust.
    pub tilt_limited: bool,
}

impl Output {
    /// The `ControlInputs` the core takes (f32).
    #[must_use]
    pub fn to_control_inputs(&self) -> ControlInputs {
        // The four values are clamped to [-1, 1] / [0, 1] before, so the narrowing cannot overflow; the rounding is the f32 interface.
        #[allow(clippy::cast_possible_truncation)]
        let n = |v: f64| v as f32;
        ControlInputs {
            throttle: n(self.throttle),
            roll: n(self.roll),
            pitch: n(self.pitch),
            yaw: n(self.yaw),
        }
    }
}

/// Stick -> rate fraction with expo: `expo s^3 + (1 - expo) s`, `s` clamped to [-1, 1].
#[must_use]
pub fn expo_curve(stick: f64, expo: f64) -> f64 {
    let s = stick.clamp(-1.0, 1.0);
    expo * s * s * s + (1.0 - expo) * s
}

/// Scales the horizontal (x, y) part down to `max_norm` if longer. Returns the vector and whether it was cut.
fn limit_horizontal(v: DVec3, max_norm: f64) -> (DVec3, bool) {
    let norm = v.x.hypot(v.y);
    if norm > max_norm && norm > EPS {
        let k = max_norm / norm;
        (DVec3::new(v.x * k, v.y * k, v.z), true)
    } else {
        (v, false)
    }
}

/// Velocity setpoint limits: horizontal speed, climb (NED z negative) and descent (z positive).
fn limit_velocity_setpoint(cfg: &ControllerConfig, v: DVec3) -> DVec3 {
    let (h, _) = limit_horizontal(v, cfg.max_speed_h_mps);
    DVec3::new(h.x, h.y, h.z.clamp(-cfg.max_climb_mps, cfg.max_descent_mps))
}

/// Velocity PI -> acceleration with the acceleration limits. The integrator of a limited axis is frozen while the error pushes
/// further into the limit (the horizontal pair freezes together).
fn desired_acceleration(
    t: &Tuning,
    state: &mut ControllerState,
    v_sp: DVec3,
    v: DVec3,
    dt: f64,
) -> DVec3 {
    let c = &t.cfg;
    let err = v_sp - v;
    let limit_i = c.accel_integral_limit_mps2 / c.velocity_ki_per_s2;
    let i_cand =
        (state.velocity_integral + err * dt).clamp(DVec3::splat(-limit_i), DVec3::splat(limit_i));
    let raw = err * c.velocity_kp_per_s + i_cand * c.velocity_ki_per_s2;
    let (h, h_cut) = limit_horizontal(raw, c.max_accel_h_mps2);
    let az = raw.z.clamp(-c.max_accel_up_mps2, c.max_accel_down_mps2);
    let z_cut = (raw.z > c.max_accel_down_mps2 && err.z > 0.0)
        || (raw.z < -c.max_accel_up_mps2 && err.z < 0.0);
    let h_freeze = h_cut && (err.x * raw.x + err.y * raw.y) > 0.0;
    let keep = state.velocity_integral;
    state.velocity_integral = DVec3::new(
        if h_freeze { keep.x } else { i_cand.x },
        if h_freeze { keep.y } else { i_cand.y },
        if z_cut { keep.z } else { i_cand.z },
    );
    DVec3::new(h.x, h.y, az)
}

/// Thrust force vector `m (a - g)` with the tilt cut. The vertical part is kept; only the horizontal part is scaled to
/// `lift x tan(max_tilt)`. Returns the vector (thrust acts along body -Z, so it points up) and whether it was cut.
fn thrust_vector(t: &Tuning, accel: DVec3) -> (DVec3, bool) {
    let f = (accel - DVec3::new(0.0, 0.0, GRAVITY_MPS2)) * t.mass_kg;
    let lift = (-f.z).max(EPS);
    let (h, cut) = limit_horizontal(f, lift * t.cfg.max_tilt_rad.tan());
    (DVec3::new(h.x, h.y, -lift), cut)
}

/// Body -> NED attitude whose -Z axis points along `thrust` (unit direction of the thrust force) and whose heading is `yaw_rad`.
/// `None` if the thrust is (nearly) horizontal along the heading, which the tilt limit rules out.
fn desired_attitude(thrust_dir: DVec3, yaw_rad: f64) -> Option<DQuat> {
    let z_b = -thrust_dir;
    let heading = DVec3::new(yaw_rad.cos(), yaw_rad.sin(), 0.0);
    let y_raw = z_b.cross(heading);
    if y_raw.length() < EPS {
        return None;
    }
    let y_b = y_raw.normalize();
    let x_b = y_b.cross(z_b);
    Some(DQuat::from_mat3(&DMat3::from_cols(x_b, y_b, z_b)).normalize())
}

/// Rate setpoint (body, rad/s) that removes the attitude error: tilt first (reduced attitude), then yaw.
fn attitude_rate_setpoint(t: &Tuning, q_cur: DQuat, q_des: DQuat) -> DVec3 {
    let z_des_body = (q_cur.inverse() * (q_des * DVec3::Z)).normalize();
    let q_tilt = DQuat::from_rotation_arc(DVec3::Z, z_des_body);
    let q_err = q_cur.conjugate() * q_des;
    let mut q_yaw = q_tilt.conjugate() * q_err;
    if q_yaw.w < 0.0 {
        q_yaw = -q_yaw;
    }
    let yaw_err = 2.0 * q_yaw.z.atan2(q_yaw.w);
    let tilt = q_tilt.to_scaled_axis();
    let c = &t.cfg;
    DVec3::new(
        (c.attitude_kp_per_s * tilt.x).clamp(-c.max_rate_rps, c.max_rate_rps),
        (c.attitude_kp_per_s * tilt.y).clamp(-c.max_rate_rps, c.max_rate_rps),
        (c.yaw_kp_per_s * yaw_err).clamp(-c.max_yaw_rate_rps, c.max_yaw_rate_rps),
    )
}

/// Rate PID per axis (derivative on the measurement, clamped integral, freeze while saturated in the error's direction).
fn rate_loop(
    t: &Tuning,
    state: &mut ControllerState,
    rate_sp: DVec3,
    rate: DVec3,
    dt: f64,
) -> DVec3 {
    let err = rate_sp - rate;
    let d_term = if state.have_previous {
        -(rate - state.previous_rates_rps) / dt
    } else {
        DVec3::ZERO
    };
    let limit_i = DVec3::splat(t.cfg.rate_integral_limit) / t.rate_ki;
    let i_cand = (state.rate_integral + err * dt).clamp(-limit_i, limit_i);
    let unsat = t.rate_kp * err + t.rate_ki * i_cand + t.rate_kd * d_term;
    let one = DVec3::ONE;
    let pushing =
        (unsat.cmpgt(one) & err.cmpgt(DVec3::ZERO)) | (unsat.cmplt(-one) & err.cmplt(DVec3::ZERO));
    state.rate_integral = DVec3::select(pushing, state.rate_integral, i_cand);
    state.previous_rates_rps = rate;
    state.have_previous = true;
    (t.rate_kp * err + t.rate_ki * state.rate_integral + t.rate_kd * d_term).clamp(-one, one)
}

/// Mixer headroom: the largest per-motor deviation from the collective is `authority x (|r| + |p| + |y|)` = `a`. The collective is
/// clamped to `[a, 1 - a]` so no duty leaves 0..1. If `a > 0.5` that interval is empty: the three axis commands are then scaled
/// together by `0.5 / a` (relative authority kept) and the collective is 0.5. Returns the collective, the commands and whether
/// anything was changed.
fn apply_headroom(t: &Tuning, collective: f64, u: DVec3) -> (f64, DVec3, bool) {
    let a = t.cfg.mixer_axis_authority * (u.x.abs() + u.y.abs() + u.z.abs());
    let scale = if a > 0.5 { 0.5 / a } else { 1.0 };
    let c = collective.clamp(a * scale, 1.0 - a * scale);
    (c, u * scale, (c - collective).abs() > EPS || scale < 1.0)
}

/// Rate setpoint, collective and tilt flag for the current setpoint.
fn outer_loops(
    t: &Tuning,
    state: &mut ControllerState,
    est: &Estimate,
    sp: &Setpoint,
    dt: f64,
) -> Result<(DVec3, f64, bool), ControllerError> {
    let cfg = &t.cfg;
    let pos = DVec3::from_array(est.position_ned_m);
    let vel = DVec3::from_array(est.velocity_ned_mps);
    let (v_sp, yaw_rad) = match *sp {
        Setpoint::Position { ned_m, yaw_rad } => (
            limit_velocity_setpoint(
                cfg,
                (DVec3::from_array(ned_m) - pos) * cfg.position_kp_per_s,
            ),
            yaw_rad,
        ),
        Setpoint::Velocity { ned_mps, yaw_rad } => (
            limit_velocity_setpoint(cfg, DVec3::from_array(ned_mps)),
            yaw_rad,
        ),
        Setpoint::Acro {
            roll,
            pitch,
            yaw,
            throttle,
        } => {
            let rates = DVec3::new(
                cfg.acro_max_rate_rps * expo_curve(roll, cfg.acro_expo),
                cfg.acro_max_rate_rps * expo_curve(pitch, cfg.acro_expo),
                cfg.acro_max_yaw_rate_rps * expo_curve(yaw, cfg.acro_expo),
            );
            return Ok((rates, throttle.clamp(0.0, 1.0), false));
        }
    };
    let accel = desired_acceleration(t, state, v_sp, vel, dt);
    let (force, tilt_cut) = thrust_vector(t, accel);
    let thrust_n = force.length();
    let q_des = desired_attitude(force / thrust_n, yaw_rad).ok_or(ControllerError::InvalidInput)?;
    let q_cur = DQuat::from_array(est.quat_xyzw).normalize();
    let rates = attitude_rate_setpoint(t, q_cur, q_des);
    let weight_n = t.mass_kg * GRAVITY_MPS2;
    let collective = t.hover_throttle * (thrust_n / weight_n).sqrt();
    Ok((rates, collective, tilt_cut))
}

/// One controller step.
///
/// # Units
/// `dt` in seconds, 0 < dt <= 0.01. Estimate and setpoint in SI, NED, quaternion (x, y, z, w).
///
/// # Errors
/// `InvalidInput` for a bad `dt`, a non-finite estimate / setpoint or a quaternion with norm < 0.5. On error `state` is unchanged.
pub fn step(
    t: &Tuning,
    state: &mut ControllerState,
    est: &Estimate,
    sp: &Setpoint,
    dt: f64,
) -> Result<Output, ControllerError> {
    if !((dt > 0.0 && dt <= MAX_DT_S) && est.is_valid() && sp.is_valid()) {
        return Err(ControllerError::InvalidInput);
    }
    let mut next = *state;
    let (rate_sp, collective, tilt_cut) = outer_loops(t, &mut next, est, sp, dt)?;
    let u = rate_loop(t, &mut next, rate_sp, DVec3::from_array(est.rates_rps), dt);
    let (throttle, u, limited) = apply_headroom(t, collective, u);
    next.steps = next.steps.saturating_add(1);
    next.collective_limited_steps = next
        .collective_limited_steps
        .saturating_add(u64::from(limited));
    next.tilt_limited_steps = next.tilt_limited_steps.saturating_add(u64::from(tilt_cut));
    let railed = u.abs().max_element() >= 1.0;
    next.axis_saturated_steps = next.axis_saturated_steps.saturating_add(u64::from(railed));
    *state = next;
    Ok(Output {
        throttle,
        roll: u.x,
        pitch: u.y,
        yaw: u.z,
        collective_limited: limited,
        tilt_limited: tilt_cut,
    })
}

/// Sample count -> f64. Exact for any series shorter than 2^32 samples (about 100 days at 500 Hz), saturating beyond.
fn index_to_f64(i: usize) -> f64 {
    f64::from(u32::try_from(i).unwrap_or(u32::MAX))
}

/// Result of `step_metrics` for a step response.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StepMetrics {
    /// Time from 10 % to 90 % of the step, s. `None` if 90 % is never reached.
    pub rise_time_s: Option<f64>,
    /// Peak excursion beyond the target as a percentage of the step size (0 if none).
    pub overshoot_percent: f64,
    /// Time after which the response stays within `band` of the target, s (0 if it never leaves the band).
    pub settling_time_s: f64,
    /// `|last sample - target|` as a fraction of the step size.
    pub final_error_fraction: f64,
}

/// Step-response numbers of a uniformly sampled series. `band` is the settling band as a fraction of the step (0.02 = 2 %).
///
/// Returns `None` for an empty series, `dt_s <= 0`, `band <= 0`, a zero or non-finite step, or any non-finite sample.
#[must_use]
pub fn step_metrics(
    series: &[f64],
    dt_s: f64,
    start: f64,
    target: f64,
    band: f64,
) -> Option<StepMetrics> {
    let size = target - start;
    let valid = !series.is_empty()
        && dt_s > 0.0
        && dt_s.is_finite()
        && band > 0.0
        && band.is_finite()
        && size.is_finite()
        && size.abs() > EPS
        && series.iter().all(|v| v.is_finite());
    if !valid {
        return None;
    }
    let progress = |x: f64| (x - start) / size;
    let first_at = |level: f64| series.iter().position(|&x| progress(x) >= level);
    let rise_time_s = match (first_at(0.1), first_at(0.9)) {
        (Some(lo), Some(hi)) => Some(index_to_f64(hi.saturating_sub(lo)) * dt_s),
        _ => None,
    };
    let peak = series
        .iter()
        .map(|&x| progress(x))
        .fold(f64::NEG_INFINITY, f64::max);
    let last_outside = series
        .iter()
        .rposition(|&x| (progress(x) - 1.0).abs() > band);
    let last = series.last().copied().unwrap_or(start);
    Some(StepMetrics {
        rise_time_s,
        overshoot_percent: 100.0 * (peak - 1.0).max(0.0),
        settling_time_s: last_outside.map_or(0.0, |i| index_to_f64(i + 1) * dt_s),
        final_error_fraction: (progress(last) - 1.0).abs(),
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::mixer;
    use std::f64::consts::FRAC_PI_2;

    /// Vehicle of the reference build (`tools/parts/builds/reference_quad/vehicle_config.json`, hover duty 0.398 as in the P3a tests).
    pub(crate) fn ref_vehicle() -> VehicleConfig {
        VehicleConfig {
            mass_kg: 1.536,
            inertia_kgm2: [0.030_135, 0.030_81, 0.055_933],
            motor_pos_m: [
                [0.2, 0.2, -0.052],
                [0.2, -0.2, -0.052],
                [-0.2, 0.2, -0.052],
                [-0.2, -0.2, -0.052],
            ],
            motor_spin: [1.0, -1.0, -1.0, 1.0],
            prop_diameter_m: 0.3048,
            thrust_coeff: 0.100_256,
            torque_coeff: 0.015_423,
            max_rpm: 8981.406,
            hover_throttle: 0.398,
        }
    }

    /// The `quad_standard` defaults of `parts_db/controller_classes.json` (all ASSUMED).
    pub(crate) fn std_cfg() -> ControllerConfig {
        ControllerConfig {
            config_version: 1,
            hover_duty: 0.398,
            max_tilt_rad: 35.0_f64.to_radians(),
            max_rate_rps: 220.0_f64.to_radians(),
            max_yaw_rate_rps: FRAC_PI_2,
            max_speed_h_mps: 12.0,
            max_accel_h_mps2: 6.0,
            max_climb_mps: 3.0,
            max_descent_mps: 2.0,
            max_accel_up_mps2: 4.0,
            max_accel_down_mps2: 4.0,
            position_kp_per_s: 1.0,
            velocity_kp_per_s: 3.0,
            velocity_ki_per_s2: 2.0,
            accel_integral_limit_mps2: 4.0,
            attitude_kp_per_s: 8.0,
            yaw_kp_per_s: 3.0,
            rate_bandwidth_rps: 40.0,
            yaw_rate_bandwidth_rps: 15.0,
            rate_integral_corner_rps: 5.0,
            rate_derivative_time_s: 0.0,
            rate_integral_limit: 0.3,
            acro_max_rate_rps: 5.0,
            acro_max_yaw_rate_rps: 2.0,
            acro_expo: 0.3,
            mixer_axis_authority: 0.2,
        }
    }

    pub(crate) fn tuning() -> Tuning {
        Tuning::new(&std_cfg(), &ref_vehicle()).unwrap()
    }

    pub(crate) fn rest_estimate() -> Estimate {
        Estimate {
            position_ned_m: [0.0, 0.0, -20.0],
            velocity_ned_mps: [0.0; 3],
            quat_xyzw: [0.0, 0.0, 0.0, 1.0],
            rates_rps: [0.0; 3],
        }
    }

    fn hold() -> Setpoint {
        Setpoint::Velocity {
            ned_mps: [0.0; 3],
            yaw_rad: 0.0,
        }
    }

    /// A named mutation of a config for the rejection tests.
    type Case<T> = (&'static str, Box<dyn Fn(&mut T)>);

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    fn rel(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-9 * b.abs()
    }

    // ------------------------------------------------------------------ gains

    #[test]
    fn test_ctl_unit_command_torques_hand_values() {
        // dT/dduty = 2 (m g / 4) / hover = 2 x 3.7657536 / 0.398 = 18.923385 N per motor.
        // roll / pitch: 0.2 x 18.923385 x (4 x 0.2) = 3.0277416 N m; yaw: 0.2 x 18.923385 x 0.015423 x 4 = 0.2334843 N m.
        let tau = unit_command_torques_nm(&std_cfg(), &ref_vehicle()).unwrap();
        assert!(rel(tau.x, 3.027_741_587_939_698_5));
        assert!(rel(tau.y, 3.027_741_587_939_698_5));
        assert!(rel(tau.z, 0.233_484_292_553_969_82));
    }

    #[test]
    fn test_ctl_rate_gains_hand_values() {
        // Kp = bandwidth x I / tau: roll 40 x 0.030135 / 3.0277416 = 0.398119, pitch 40 x 0.03081 / 3.0277416 = 0.407036,
        // yaw 15 x 0.055933 / 0.2334843 = 3.593368. Ki = Kp x 5, Kd = Kp x Td (0 here).
        let t = tuning();
        assert!(rel(t.rate_kp.x, 0.398_118_520_022_127_83));
        assert!(rel(t.rate_kp.y, 0.407_036_057_802_613_54));
        assert!(rel(t.rate_kp.z, 3.593_368_062_676_278_3));
        assert!(rel(t.rate_ki.x, 5.0 * 0.398_118_520_022_127_83));
        assert_eq!(t.rate_kd, DVec3::ZERO);
        let mut c = std_cfg();
        c.rate_derivative_time_s = 0.01;
        let td = Tuning::new(&c, &ref_vehicle()).unwrap();
        assert!(rel(td.rate_kd.x, 0.01 * 0.398_118_520_022_127_83));
    }

    #[test]
    fn test_ctl_gains_scale_with_inertia_and_authority() {
        // double inertia -> double Kp; double the thrust slope (half the hover duty) -> half Kp
        let mut v = ref_vehicle();
        v.inertia_kgm2 = [0.060_27, 0.061_62, 0.111_866];
        let t2 = Tuning::new(&std_cfg(), &v).unwrap();
        assert!(rel(t2.rate_kp.x, 2.0 * tuning().rate_kp.x));
        let mut c = std_cfg();
        c.hover_duty = 0.199;
        let t3 = Tuning::new(&c, &ref_vehicle()).unwrap();
        assert!(rel(t3.rate_kp.y, 0.5 * tuning().rate_kp.y));
    }

    #[test]
    fn test_ctl_vehicle_rejections() {
        let c = std_cfg();
        let bad: Vec<Case<VehicleConfig>> = vec![
            ("mass 0", Box::new(|v| v.mass_kg = 0.0)),
            ("mass nan", Box::new(|v| v.mass_kg = f64::NAN)),
            ("kappa 0", Box::new(|v| v.torque_coeff = 0.0)),
            ("no arms", Box::new(|v| v.motor_pos_m = [[0.0; 3]; 4])),
            ("inertia 0", Box::new(|v| v.inertia_kgm2[2] = 0.0)),
            ("inertia nan", Box::new(|v| v.inertia_kgm2[0] = f64::NAN)),
        ];
        for (name, f) in bad {
            let mut v = ref_vehicle();
            f(&mut v);
            assert_eq!(
                Tuning::new(&c, &v).err(),
                Some(ControllerError::InvalidVehicle),
                "{name}"
            );
        }
        let mut c1 = c;
        c1.hover_duty = 1.0; // exactly 1 is allowed
        assert!(Tuning::new(&c1, &ref_vehicle()).is_ok());
        c1.hover_duty = 1.01;
        assert_eq!(
            Tuning::new(&c1, &ref_vehicle()).err(),
            Some(ControllerError::InvalidConfig)
        );
    }

    #[test]
    fn test_ctl_mixer_authority_matches_the_mixer() {
        // the config value mirrors the 0.2 inside mix_controls: a full roll command moves the left / right pair by that much
        let m = mixer::mix_controls(&ControlInputs {
            throttle: 0.5,
            roll: 1.0,
            pitch: 0.0,
            yaw: 0.0,
        });
        assert!(near(m[1] - 0.5, std_cfg().mixer_axis_authority, 1e-6));
        assert!(near(0.5 - m[0], std_cfg().mixer_axis_authority, 1e-6));
    }

    // ------------------------------------------------------------------ config

    #[test]
    fn test_ctl_config_validation_rules() {
        assert_eq!(validate_config(&std_cfg()), Ok(()));
        let bad: Vec<Case<ControllerConfig>> = vec![
            ("version", Box::new(|c| c.config_version = 2)),
            ("hover duty 0", Box::new(|c| c.hover_duty = 0.0)),
            ("hover duty 1.1", Box::new(|c| c.hover_duty = 1.1)),
            ("hover duty nan", Box::new(|c| c.hover_duty = f64::NAN)),
            ("tilt 0", Box::new(|c| c.max_tilt_rad = 0.0)),
            (
                "tilt 81 deg",
                Box::new(|c| c.max_tilt_rad = 81.0_f64.to_radians()),
            ),
            ("tilt nan", Box::new(|c| c.max_tilt_rad = f64::NAN)),
            ("rate 0", Box::new(|c| c.max_rate_rps = 0.0)),
            ("yaw rate neg", Box::new(|c| c.max_yaw_rate_rps = -1.0)),
            ("speed inf", Box::new(|c| c.max_speed_h_mps = f64::INFINITY)),
            ("accel h 0", Box::new(|c| c.max_accel_h_mps2 = 0.0)),
            (
                "down accel 0.9 g",
                Box::new(|c| c.max_accel_down_mps2 = 0.9 * 9.806_65),
            ),
            ("up accel 0", Box::new(|c| c.max_accel_up_mps2 = 0.0)),
            ("ki 0", Box::new(|c| c.velocity_ki_per_s2 = 0.0)),
            ("corner 0", Box::new(|c| c.rate_integral_corner_rps = 0.0)),
            ("td neg", Box::new(|c| c.rate_derivative_time_s = -0.001)),
            ("td nan", Box::new(|c| c.rate_derivative_time_s = f64::NAN)),
            ("expo 1.1", Box::new(|c| c.acro_expo = 1.1)),
            ("expo neg", Box::new(|c| c.acro_expo = -0.1)),
            ("authority 0", Box::new(|c| c.mixer_axis_authority = 0.0)),
            ("authority 0.6", Box::new(|c| c.mixer_axis_authority = 0.6)),
            ("int limit 0", Box::new(|c| c.rate_integral_limit = 0.0)),
        ];
        for (name, f) in bad {
            let mut c = std_cfg();
            f(&mut c);
            assert_eq!(
                validate_config(&c),
                Err(ControllerError::InvalidConfig),
                "{name}"
            );
        }
        let mut c = std_cfg();
        c.max_tilt_rad = 80.0_f64.to_radians(); // the upper bound itself is allowed
        c.acro_expo = 1.0;
        c.rate_derivative_time_s = 0.0;
        assert_eq!(validate_config(&c), Ok(()));
    }

    fn std_json() -> String {
        r#"{"config_version":1,"vehicle_class":"quad_standard","hover_duty":0.398,"max_tilt_rad":0.6108652381980153,"max_rate_rps":3.839724354387525,
"max_yaw_rate_rps":1.5707963267948966,"max_speed_h_mps":12.0,"max_accel_h_mps2":6.0,"max_climb_mps":3.0,"max_descent_mps":2.0,
"max_accel_up_mps2":4.0,"max_accel_down_mps2":4.0,"position_kp_per_s":1.0,"velocity_kp_per_s":3.0,"velocity_ki_per_s2":2.0,
"accel_integral_limit_mps2":4.0,"attitude_kp_per_s":8.0,"yaw_kp_per_s":3.0,"rate_bandwidth_rps":40.0,"yaw_rate_bandwidth_rps":15.0,
"rate_integral_corner_rps":5.0,"rate_derivative_time_s":0.0,"rate_integral_limit":0.3,"acro_max_rate_rps":5.0,
"acro_max_yaw_rate_rps":2.0,"acro_expo":0.3,"mixer_axis_authority":0.2}"#
            .to_string()
    }

    #[test]
    fn test_ctl_json_round_trip_and_errors() {
        let c = parse_controller_config(&std_json()).unwrap();
        assert_eq!(c, std_cfg());
        // provenance is accepted and ignored
        let with_prov = std_json().replace('}', r#","provenance":{"note":"x"}}"#);
        assert_eq!(parse_controller_config(&with_prov), Ok(std_cfg()));
        let unknown = std_json().replace('}', r#","extra":1}"#);
        assert_eq!(
            parse_controller_config(&unknown),
            Err(ControllerLoadError::Parse)
        );
        let missing = std_json().replace(r#""acro_expo":0.3,"#, "");
        assert_eq!(
            parse_controller_config(&missing),
            Err(ControllerLoadError::Parse)
        );
        let empty_class = std_json().replace("quad_standard", "");
        assert_eq!(
            parse_controller_config(&empty_class),
            Err(ControllerLoadError::Parse)
        );
        let long_class = std_json().replace("quad_standard", &"x".repeat(33));
        assert_eq!(
            parse_controller_config(&long_class),
            Err(ControllerLoadError::Parse)
        );
        let bad_value = std_json().replace(r#""acro_expo":0.3"#, r#""acro_expo":1.5"#);
        assert_eq!(
            parse_controller_config(&bad_value),
            Err(ControllerLoadError::Invalid)
        );
        assert_eq!(
            parse_controller_config("{"),
            Err(ControllerLoadError::Parse)
        );
        assert_eq!(
            load_controller_config("/nonexistent/c.json"),
            Err(ControllerLoadError::Io)
        );
    }

    // ------------------------------------------------------------------ pieces

    #[test]
    fn test_ctl_expo_curve_hand_values() {
        // expo 0.3, s = 0.5: 0.3 x 0.125 + 0.7 x 0.5 = 0.3875
        assert!(near(expo_curve(0.5, 0.3), 0.3875, 1e-15));
        assert!(near(expo_curve(-0.5, 0.3), -0.3875, 1e-15));
        assert_eq!(expo_curve(0.0, 0.3), 0.0);
        assert!(near(expo_curve(1.0, 0.3), 1.0, 1e-15));
        assert!(near(expo_curve(7.0, 0.3), 1.0, 1e-15)); // clamped to the stick range
        assert!(near(expo_curve(-7.0, 0.3), -1.0, 1e-15));
        assert!(near(expo_curve(0.5, 0.0), 0.5, 1e-15)); // linear
        assert!(near(expo_curve(0.5, 1.0), 0.125, 1e-15)); // pure cubic
    }

    #[test]
    fn test_ctl_thrust_vector_unclamped_and_clamped_hand_values() {
        let t = tuning();
        // a = 2 m/s^2 forward: F = 1.536 x (2, 0, 2 - ... ) -> (3.072, 0, -15.0630144); tilt 11.53 deg < 35 deg: untouched
        let (f, cut) = thrust_vector(&t, DVec3::new(2.0, 0.0, 0.0));
        assert!(near(f.x, 3.072, 1e-12) && near(f.z, -15.063_014_4, 1e-9) && !cut);
        // a = 10 forward: F_h = 15.36 > 15.0630144 x tan 35 = 10.547236: cut to the limit, lift unchanged
        let (f, cut) = thrust_vector(&t, DVec3::new(10.0, 0.0, 0.0));
        assert!(cut && near(f.x, 10.547_236_231_041_408, 1e-9) && near(f.z, -15.063_014_4, 1e-9));
        // with 4 m/s^2 up as well: lift = 1.536 (9.80665 + 4) = 21.2070144, limit 14.849311 < 15.36: cut, lift kept
        let (f, cut) = thrust_vector(&t, DVec3::new(10.0, 0.0, -4.0));
        assert!(cut && near(f.x, 14.849_311_345_801_864, 1e-9) && near(f.z, -21.207_014_4, 1e-9));
        // the direction of the horizontal part is kept (3-4-5)
        let (f, _) = thrust_vector(&t, DVec3::new(6.0, 8.0, 0.0));
        assert!(near(f.y / f.x, 8.0 / 6.0, 1e-12));
        assert!(near(f.x.hypot(f.y), 10.547_236_231_041_408, 1e-9));
    }

    #[test]
    fn test_ctl_desired_attitude_hand_values() {
        let same = |q: DQuat, e: [f64; 4]| q.dot(DQuat::from_array(e)).abs() > 1.0 - 1e-12;
        let up = DVec3::new(0.0, 0.0, -1.0);
        assert!(same(
            desired_attitude(up, 0.0).unwrap(),
            [0.0, 0.0, 0.0, 1.0]
        ));
        // yaw 90 deg: rotation about body Z: (0, 0, sin 45, cos 45)
        let s = std::f64::consts::FRAC_1_SQRT_2;
        assert!(same(
            desired_attitude(up, FRAC_PI_2).unwrap(),
            [0.0, 0.0, s, s]
        ));
        // thrust tilted forward by 30 deg (sin30, 0, -cos30): nose down 30 deg = pitch -30: (0, -sin15, 0, cos15)
        let fwd = DVec3::new(0.5, 0.0, -(0.75_f64).sqrt());
        assert!(same(
            desired_attitude(fwd, 0.0).unwrap(),
            [0.0, -0.258_819_045_102_520_74, 0.0, 0.965_925_826_289_068_3]
        ));
        // thrust tilted to the east (right): right wing down = roll +30: (sin15, 0, 0, cos15)
        let east = DVec3::new(0.0, 0.5, -(0.75_f64).sqrt());
        assert!(same(
            desired_attitude(east, 0.0).unwrap(),
            [0.258_819_045_102_520_74, 0.0, 0.0, 0.965_925_826_289_068_3]
        ));
        // thrust along the heading direction and horizontal has no defined attitude
        assert!(desired_attitude(DVec3::new(-1.0, 0.0, 0.0), 0.0).is_none());
    }

    #[test]
    fn test_ctl_attitude_rate_setpoint_signs_and_values() {
        let t = tuning();
        let id = DQuat::IDENTITY;
        // desired roll +10 deg: rate x = Kp_att x 0.1745329 = 8 x 0.1745329 = 1.3962634 (below the 3.84 limit), y and z zero
        let r = attitude_rate_setpoint(&t, id, DQuat::from_rotation_x(10.0_f64.to_radians()));
        assert!(
            near(r.x, 1.396_263_401_595_463_6, 1e-9)
                && near(r.y, 0.0, 1e-12)
                && near(r.z, 0.0, 1e-12)
        );
        // desired pitch +0.1 rad (nose up): rate y = 0.8
        let r = attitude_rate_setpoint(&t, id, DQuat::from_rotation_y(0.1));
        assert!(near(r.y, 0.8, 1e-9) && near(r.x, 0.0, 1e-12));
        // desired yaw +0.5 rad: rate z = 3 x 0.5 = 1.5 (below the 1.5708 limit)
        let r = attitude_rate_setpoint(&t, id, DQuat::from_rotation_z(0.5));
        assert!(near(r.z, 1.5, 1e-9) && near(r.x, 0.0, 1e-12));
        // limits: 1 rad of yaw error -> 3 clamped to 1.5708; 1 rad of roll -> 8 clamped to 3.8397
        let r = attitude_rate_setpoint(&t, id, DQuat::from_rotation_z(1.0));
        assert!(near(r.z, FRAC_PI_2, 1e-12));
        let r = attitude_rate_setpoint(&t, id, DQuat::from_rotation_x(1.0));
        assert!(near(r.x, 220.0_f64.to_radians(), 1e-12));
        // tilt error is expressed in the CURRENT body frame: vehicle yawed 90 deg, desired = vehicle tilted 0.1 rad about its own x
        let cur = DQuat::from_rotation_z(FRAC_PI_2);
        let r = attitude_rate_setpoint(&t, cur, cur * DQuat::from_rotation_x(0.1));
        assert!(near(r.x, 0.8, 1e-9) && near(r.z, 0.0, 1e-9));
        // already there: zero
        assert!(attitude_rate_setpoint(&t, cur, cur).length() < 1e-12);
    }

    #[test]
    fn test_ctl_yaw_error_takes_the_short_way() {
        let t = tuning();
        // current yaw 170 deg, desired -170 deg: the short way is +20 deg, not -340
        let cur = DQuat::from_rotation_z(170.0_f64.to_radians());
        let des = DQuat::from_rotation_z((-170.0_f64).to_radians());
        let r = attitude_rate_setpoint(&t, cur, des);
        assert!(near(r.z, 3.0 * 20.0_f64.to_radians(), 1e-9));
    }

    #[test]
    fn test_ctl_rate_loop_p_i_and_d_hand_values() {
        let t = tuning();
        let mut s = ControllerState::new();
        // first step, error (1, 0, 0), dt 0.002: u = Kp e + Ki (e dt) = 0.39811852 x 1 + 1.9905926 x 0.002 = 0.4020 ...
        let u = rate_loop(&t, &mut s, DVec3::new(1.0, 0.0, 0.0), DVec3::ZERO, 0.002);
        let kp = 0.398_118_520_022_127_83;
        assert!(near(u.x, kp + 5.0 * kp * 0.002, 1e-12));
        assert!(near(s.rate_integral.x, 0.002, 1e-15));
        // D acts on the measurement: Kd = Kp x 0.01; the measured rate rises by 0.05 rad/s in one 0.002 s step
        let mut c = std_cfg();
        c.rate_derivative_time_s = 0.01;
        let td = Tuning::new(&c, &ref_vehicle()).unwrap();
        let mut s = ControllerState::new();
        rate_loop(&td, &mut s, DVec3::ZERO, DVec3::ZERO, 0.002);
        let u = rate_loop(&td, &mut s, DVec3::ZERO, DVec3::new(0.05, 0.0, 0.0), 0.002);
        // e = -0.05: P = -0.05 Kp; I = Ki x (-0.05 x 0.002) = -5 Kp x 0.0001; D term = -(0.05 / 0.002) = -25 -> Kd x -25 = -0.25 Kp
        let expect = -0.05 * kp - 5.0 * kp * 0.0001 - 0.25 * kp;
        assert!(near(u.x, expect, 1e-12));
        // a setpoint step does not kick the D term (derivative of the measurement, not of the error)
        let mut s = ControllerState::new();
        rate_loop(&td, &mut s, DVec3::ZERO, DVec3::ZERO, 0.002);
        let u = rate_loop(&td, &mut s, DVec3::new(0.2, 0.0, 0.0), DVec3::ZERO, 0.002);
        assert!(near(u.x, 0.2 * kp + 5.0 * kp * 0.2 * 0.002, 1e-12));
    }

    #[test]
    fn test_ctl_rate_loop_output_limit_and_anti_windup() {
        let t = tuning();
        let mut s = ControllerState::new();
        // an error of 10 rad/s gives Kp x 10 = 3.98 -> clamped to 1; the integral does not grow while saturated
        for _ in 0..100 {
            let u = rate_loop(&t, &mut s, DVec3::new(10.0, -10.0, 0.0), DVec3::ZERO, 0.002);
            assert_eq!((u.x, u.y), (1.0, -1.0));
        }
        assert_eq!(s.rate_integral.x, 0.0);
        assert_eq!(s.rate_integral.y, 0.0);
        // the integral term is limited to 0.3 of command: Ki I <= 0.3 -> I <= 0.3 / (5 Kp)
        let mut s = ControllerState::new();
        for _ in 0..20_000 {
            rate_loop(&t, &mut s, DVec3::new(0.05, 0.0, 0.0), DVec3::ZERO, 0.002);
        }
        assert!(near(
            s.rate_integral.x,
            0.3 / (5.0 * 0.398_118_520_022_127_83),
            1e-12
        ));
    }

    #[test]
    fn test_ctl_headroom_hand_values() {
        let t = tuning();
        let u = DVec3::new(1.0, 0.5, -0.5);
        // a = 0.2 x (1 + 0.5 + 0.5) = 0.4 -> collective in [0.4, 0.6]
        assert_eq!(apply_headroom(&t, 0.9, u), (0.6, u, true));
        assert_eq!(apply_headroom(&t, 0.1, u), (0.4, u, true));
        assert_eq!(apply_headroom(&t, 0.5, u), (0.5, u, false));
        // everything at the rail: a = 0.6 > 0.5, so the three commands are scaled by 0.5 / 0.6 and the collective is 0.5
        let (c, scaled, limited) = apply_headroom(&t, 0.5, DVec3::ONE);
        assert!(
            limited
                && near(c, 0.5, 1e-15)
                && near(scaled.x, 5.0 / 6.0, 1e-15)
                && near(scaled.z, 5.0 / 6.0, 1e-15)
        );
        assert_eq!(
            apply_headroom(&t, 0.3, DVec3::ZERO),
            (0.3, DVec3::ZERO, false)
        );
        // a = 0.5 exactly (sum 2.5) needs no scaling
        let (c, same, limited) = apply_headroom(&t, 0.5, DVec3::new(1.0, 1.0, 0.5));
        assert!(!limited && c == 0.5 && same == DVec3::new(1.0, 1.0, 0.5));
    }

    #[test]
    fn test_ctl_headroom_means_the_mixer_never_clips() {
        let t = tuning();
        let levels = [-1.0, -0.5, 0.0, 0.3, 1.0];
        for &r in &levels {
            for &p in &levels {
                for &y in &levels {
                    for c in [0.0, 0.25, 0.5, 0.75, 1.0] {
                        let (col, u, _) = apply_headroom(&t, c, DVec3::new(r, p, y));
                        let m = mixer::mix_controls(&ControlInputs {
                            throttle: col as f32,
                            roll: u.x as f32,
                            pitch: u.y as f32,
                            yaw: u.z as f32,
                        });
                        // no duty is clipped: every one lies strictly inside 0..1 or exactly on the edge it is allowed to touch
                        assert!(m.iter().all(|&d| (0.0..=1.0).contains(&d)));
                        // un-clipped duties reproduce the commanded axis differences exactly (a clipped mixer would not)
                        let roll = (m[1] + m[3] - m[0] - m[2]) / 4.0 / 0.2;
                        let pitch = (m[0] + m[1] - m[2] - m[3]) / 4.0 / 0.2;
                        assert!(
                            near(roll, u.x, 1e-5) && near(pitch, u.y, 1e-5),
                            "{r} {p} {y} {c}"
                        );
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------ step()

    #[test]
    fn test_ctl_hover_command_is_exactly_hover() {
        let t = tuning();
        let mut s = ControllerState::new();
        let o = step(&t, &mut s, &rest_estimate(), &hold(), 0.002).unwrap();
        // thrust = weight -> throttle = hover x sqrt(1); no attitude, rate or velocity error
        assert!(near(o.throttle, 0.398, 1e-12));
        assert!(near(o.roll, 0.0, 1e-12) && near(o.pitch, 0.0, 1e-12) && near(o.yaw, 0.0, 1e-12));
        assert!(!o.collective_limited && !o.tilt_limited);
        let p = Setpoint::Position {
            ned_m: [0.0, 0.0, -20.0],
            yaw_rad: 0.0,
        };
        let o = step(&t, &mut ControllerState::new(), &rest_estimate(), &p, 0.002).unwrap();
        assert!(near(o.throttle, 0.398, 1e-12) && near(o.roll, 0.0, 1e-12));
    }

    #[test]
    fn test_ctl_velocity_demand_signs() {
        let t = tuning();
        let go = |v: [f64; 3]| {
            let sp = Setpoint::Velocity {
                ned_mps: v,
                yaw_rad: 0.0,
            };
            step(
                &t,
                &mut ControllerState::new(),
                &rest_estimate(),
                &sp,
                0.002,
            )
            .unwrap()
        };
        let north = go([5.0, 0.0, 0.0]);
        assert!(north.pitch < 0.0 && near(north.roll, 0.0, 1e-9)); // nose down to go forward
        let east = go([0.0, 5.0, 0.0]);
        assert!(east.roll > 0.0 && near(east.pitch, 0.0, 1e-9)); // right wing down to go right
        let climb = go([0.0, 0.0, -2.0]);
        assert!(climb.throttle > 0.398);
        let sink = go([0.0, 0.0, 1.0]);
        assert!(sink.throttle < 0.398);
    }

    #[test]
    fn test_ctl_velocity_demand_hand_values() {
        // 5 m/s north from rest: a = Kp 5 + Ki (5 x 0.002) = 15.01 -> cut to 6 m/s^2. F = 1.536 (6, 0, -9.80665) = (9.216, 0, -15.0630144),
        // tilt atan(9.216 / 15.0630144) = 31.46 deg < 35: not tilt-limited. T = |F| = 17.6579..., throttle = 0.398 sqrt(T / 15.0630144).
        let t = tuning();
        let sp = Setpoint::Velocity {
            ned_mps: [5.0, 0.0, 0.0],
            yaw_rad: 0.0,
        };
        let mut s = ControllerState::new();
        let o = step(&t, &mut s, &rest_estimate(), &sp, 0.002).unwrap();
        let thrust = (9.216_f64.powi(2) + 15.063_014_4_f64.powi(2)).sqrt();
        assert!(near(
            o.throttle,
            0.398 * (thrust / 15.063_014_4).sqrt(),
            1e-9
        ));
        assert!(!o.tilt_limited);
        // the horizontal integrator froze (acceleration limited): it is still zero
        assert_eq!(s.velocity_integral.x, 0.0);
        assert_eq!(s.steps, 1);
    }

    #[test]
    fn test_ctl_tilt_limit_flag_and_counter() {
        let mut c = std_cfg();
        c.max_tilt_rad = 20.0_f64.to_radians(); // 6 m/s^2 needs 31.5 deg
        let t = Tuning::new(&c, &ref_vehicle()).unwrap();
        let sp = Setpoint::Velocity {
            ned_mps: [5.0, 0.0, 0.0],
            yaw_rad: 0.0,
        };
        let mut s = ControllerState::new();
        let o = step(&t, &mut s, &rest_estimate(), &sp, 0.002).unwrap();
        assert!(o.tilt_limited);
        assert_eq!((s.tilt_limited_steps, s.steps), (1, 1));
        step(&t, &mut s, &rest_estimate(), &hold(), 0.002).unwrap();
        assert_eq!((s.tilt_limited_steps, s.steps), (1, 2));
    }

    #[test]
    fn test_ctl_speed_and_climb_setpoints_are_limited() {
        let c = std_cfg();
        let v = limit_velocity_setpoint(&c, DVec3::new(30.0, 40.0, 0.0));
        assert!(near(v.x.hypot(v.y), 12.0, 1e-12) && near(v.y / v.x, 4.0 / 3.0, 1e-12));
        assert_eq!(
            limit_velocity_setpoint(&c, DVec3::new(0.0, 0.0, -9.0)).z,
            -3.0
        );
        assert_eq!(
            limit_velocity_setpoint(&c, DVec3::new(0.0, 0.0, 9.0)).z,
            2.0
        );
        assert_eq!(
            limit_velocity_setpoint(&c, DVec3::new(3.0, 4.0, 1.0)),
            DVec3::new(3.0, 4.0, 1.0)
        );
    }

    #[test]
    fn test_ctl_position_mode_commands_a_limited_velocity() {
        // 100 m north: v_sp = 1 x 100 limited to 12 m/s; same command as velocity mode at 12 m/s
        let t = tuning();
        let far = Setpoint::Position {
            ned_m: [100.0, 0.0, -20.0],
            yaw_rad: 0.0,
        };
        let v12 = Setpoint::Velocity {
            ned_mps: [12.0, 0.0, 0.0],
            yaw_rad: 0.0,
        };
        let a = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &far,
            0.002,
        )
        .unwrap();
        let b = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &v12,
            0.002,
        )
        .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn test_ctl_yaw_setpoint_gives_a_yaw_command() {
        let t = tuning();
        let right = Setpoint::Velocity {
            ned_mps: [0.0; 3],
            yaw_rad: 0.5,
        };
        let o = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &right,
            0.002,
        )
        .unwrap();
        // rate setpoint 1.5 rad/s: u = Kp_yaw x 1.5 + Ki x 1.5 x 0.002 = 3.5933681 x 1.5 x (1 + 5 x 0.002) = 5.4 -> clamped to 1
        assert_eq!(o.yaw, 1.0);
        let left = Setpoint::Velocity {
            ned_mps: [0.0; 3],
            yaw_rad: -0.1,
        };
        let o = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &left,
            0.002,
        )
        .unwrap();
        // rate setpoint -0.3: u = 3.5933681 x -0.3 x 1.01 = -1.0888 -> clamped to -1; the sign is what matters here
        assert_eq!(o.yaw, -1.0);
        let small = Setpoint::Velocity {
            ned_mps: [0.0; 3],
            yaw_rad: 0.02,
        };
        let o = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &small,
            0.002,
        )
        .unwrap();
        // rate setpoint 0.06: u = 3.5933681 x 0.06 x 1.01 = 0.217762
        assert!(near(o.yaw, 3.593_368_062_676_278_3 * 0.06 * 1.01, 1e-9));
    }

    #[test]
    fn test_ctl_acro_hand_values() {
        // stick 0.5 roll, expo 0.3: rate setpoint 5 x 0.3875 = 1.9375 rad/s; first step u = Kp x 1.9375 x (1 + 5 x 0.002) = 0.77906818
        let t = tuning();
        let sp = Setpoint::Acro {
            roll: 0.5,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.4,
        };
        let o = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &sp,
            0.002,
        )
        .unwrap();
        assert!(near(o.roll, 0.779_068_178_868_301_4, 1e-9));
        assert!(near(o.pitch, 0.0, 1e-15) && near(o.yaw, 0.0, 1e-15));
        // headroom: a = 0.2 x 0.77907 = 0.155814 < 0.4: the stick throttle passes unchanged
        assert!(near(o.throttle, 0.4, 1e-12) && !o.collective_limited);
        // full throttle with that roll command is cut to 1 - a
        let sp = Setpoint::Acro {
            roll: 0.5,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 1.0,
        };
        let o = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &sp,
            0.002,
        )
        .unwrap();
        assert!(
            o.collective_limited && near(o.throttle, 1.0 - 0.2 * 0.779_068_178_868_301_4, 1e-9)
        );
        // pitch and yaw sticks use their own limits: yaw stick 1 -> 2 rad/s
        let sp = Setpoint::Acro {
            roll: 0.0,
            pitch: -0.2,
            yaw: 0.1,
            throttle: 0.4,
        };
        let o = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &sp,
            0.002,
        )
        .unwrap();
        assert!(o.pitch < 0.0 && o.yaw > 0.0);
        // throttle stick outside 0..1 is clamped
        let sp = Setpoint::Acro {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 3.0,
        };
        let o = step(
            &t,
            &mut ControllerState::new(),
            &rest_estimate(),
            &sp,
            0.002,
        )
        .unwrap();
        assert_eq!(o.throttle, 1.0);
    }

    #[test]
    fn test_ctl_counters_count_saturation() {
        let t = tuning();
        let mut s = ControllerState::new();
        let sp = Setpoint::Acro {
            roll: 1.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.5,
        };
        // full stick: rate setpoint 5 rad/s -> u = Kp x 5 = 1.99 -> railed
        let o = step(&t, &mut s, &rest_estimate(), &sp, 0.002).unwrap();
        assert_eq!(o.roll, 1.0);
        assert_eq!((s.axis_saturated_steps, s.collective_limited_steps), (1, 0));
        let sp = Setpoint::Acro {
            roll: 1.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 1.0,
        };
        step(&t, &mut s, &rest_estimate(), &sp, 0.002).unwrap();
        assert_eq!(
            (s.axis_saturated_steps, s.collective_limited_steps, s.steps),
            (2, 1, 2)
        );
    }

    #[test]
    fn test_ctl_input_rejections_leave_the_state_untouched() {
        let t = tuning();
        let mut s = ControllerState::new();
        step(&t, &mut s, &rest_estimate(), &hold(), 0.002).unwrap();
        let before = s;
        let mut nan_pos = rest_estimate();
        nan_pos.position_ned_m[1] = f64::NAN;
        let mut inf_rate = rest_estimate();
        inf_rate.rates_rps[2] = f64::INFINITY;
        let mut zero_q = rest_estimate();
        zero_q.quat_xyzw = [0.0; 4];
        let mut small_q = rest_estimate();
        small_q.quat_xyzw = [0.0, 0.0, 0.0, 0.4];
        let nan_sp = Setpoint::Velocity {
            ned_mps: [f64::NAN, 0.0, 0.0],
            yaw_rad: 0.0,
        };
        let inf_yaw = Setpoint::Position {
            ned_m: [0.0; 3],
            yaw_rad: f64::INFINITY,
        };
        let nan_stick = Setpoint::Acro {
            roll: f64::NAN,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.5,
        };
        let e = Err(ControllerError::InvalidInput);
        let good = rest_estimate();
        assert_eq!(step(&t, &mut s, &good, &hold(), 0.0), e);
        assert_eq!(step(&t, &mut s, &good, &hold(), -0.001), e);
        assert_eq!(step(&t, &mut s, &good, &hold(), 0.010_001), e);
        assert_eq!(step(&t, &mut s, &good, &hold(), f64::NAN), e);
        assert_eq!(step(&t, &mut s, &nan_pos, &hold(), 0.002), e);
        assert_eq!(step(&t, &mut s, &inf_rate, &hold(), 0.002), e);
        assert_eq!(step(&t, &mut s, &zero_q, &hold(), 0.002), e);
        assert_eq!(step(&t, &mut s, &small_q, &hold(), 0.002), e);
        assert_eq!(step(&t, &mut s, &good, &nan_sp, 0.002), e);
        assert_eq!(step(&t, &mut s, &good, &inf_yaw, 0.002), e);
        assert_eq!(step(&t, &mut s, &good, &nan_stick, 0.002), e);
        assert_eq!(s, before);
        // exactly 10 ms and a barely valid quaternion norm (0.5) are accepted
        let mut half_q = rest_estimate();
        half_q.quat_xyzw = [0.0, 0.0, 0.0, 0.5];
        assert!(step(&t, &mut s, &half_q, &hold(), 0.01).is_ok());
    }

    #[test]
    fn test_ctl_step_is_deterministic() {
        let t = tuning();
        let run = || {
            let mut s = ControllerState::new();
            let mut est = rest_estimate();
            let sp = Setpoint::Position {
                ned_m: [5.0, -3.0, -22.0],
                yaw_rad: 1.0,
            };
            let mut last = None;
            for k in 0..200 {
                est.velocity_ned_mps[0] = 0.01 * f64::from(k);
                last = Some(step(&t, &mut s, &est, &sp, 0.002).unwrap());
            }
            (last.unwrap(), s)
        };
        let (a, sa) = run();
        let (b, sb) = run();
        assert_eq!(a.throttle.to_bits(), b.throttle.to_bits());
        assert_eq!(a.roll.to_bits(), b.roll.to_bits());
        assert_eq!(sa, sb);
    }

    #[test]
    fn test_ctl_velocity_integrator_clamps_and_freezes() {
        let t = tuning();
        // small steady error (0.1 m/s, vertical): the integral grows by e dt each step, until Kp e + Ki I reaches the 4 m/s^2 down-acceleration limit: I = (4 - 0.3) / 2 = 1.85 m (the freeze acts within one step, 0.0002)
        let mut s = ControllerState::new();
        let est = rest_estimate();
        let sp = Setpoint::Velocity {
            ned_mps: [0.0, 0.0, 0.1],
            yaw_rad: 0.0,
        };
        for _ in 0..10 {
            step(&t, &mut s, &est, &sp, 0.002).unwrap();
        }
        assert!(near(s.velocity_integral.z, 10.0 * 0.1 * 0.002, 1e-12));
        for _ in 0..100_000 {
            step(&t, &mut s, &est, &sp, 0.002).unwrap();
        }
        assert!(near(s.velocity_integral.z, 1.85, 3e-4));
        // a huge climb demand saturates the up-acceleration limit (4 m/s^2) and the integral stops growing
        let mut s = ControllerState::new();
        let big = Setpoint::Velocity {
            ned_mps: [0.0, 0.0, -2.9],
            yaw_rad: 0.0,
        };
        let mut est = rest_estimate();
        est.velocity_ned_mps[2] = 2.0; // sinking at 2 m/s: error -4.9 -> 3 x -4.9 = -14.7 < -4
        for _ in 0..50 {
            step(&t, &mut s, &est, &big, 0.002).unwrap();
        }
        assert_eq!(s.velocity_integral.z, 0.0);
    }

    #[test]
    fn test_ctl_velocity_integral_term_is_limited_to_the_configured_accel() {
        // limit 0.5 m/s^2 with Ki 2 -> |I| <= 0.25 m. A 0.01 m/s error needs 0.03 + 2 x 0.25 = 0.53 m/s^2 < every acceleration limit,
        // so only the integral limit can stop the growth.
        let mut c = std_cfg();
        c.accel_integral_limit_mps2 = 0.5;
        let t = Tuning::new(&c, &ref_vehicle()).unwrap();
        let mut s = ControllerState::new();
        let sp = Setpoint::Velocity {
            ned_mps: [0.0, 0.0, 0.01],
            yaw_rad: 0.0,
        };
        for _ in 0..20_000 {
            step(&t, &mut s, &rest_estimate(), &sp, 0.002).unwrap();
        }
        assert!(near(s.velocity_integral.z, 0.25, 1e-12));
    }

    // ------------------------------------------------------------------ metrics

    #[test]
    fn test_ctl_step_metrics_first_order_lag() {
        // x = 1 - exp(-t / 0.1) sampled at 1 kHz: 10-90 % rise = tau ln 9 = 0.219722 s; 2 % settling = tau ln 50 = 0.391202 s; no overshoot
        let series: Vec<f64> = (0..2000)
            .map(|k| 1.0 - (-(f64::from(k) * 0.001) / 0.1).exp())
            .collect();
        let m = step_metrics(&series, 0.001, 0.0, 1.0, 0.02).unwrap();
        assert!(near(m.rise_time_s.unwrap(), 0.219_722, 0.002));
        assert!(near(m.settling_time_s, 0.391_202, 0.002));
        assert_eq!(m.overshoot_percent, 0.0);
        assert!(m.final_error_fraction < 1e-8);
    }

    #[test]
    fn test_ctl_step_metrics_overshoot_and_direction() {
        // dt 1 s: 0, 0.5, 1.2, 1.0, 1.0 -> peak 1.2 = 20 % overshoot; last sample outside the 2 % band is index 2 -> settles at 3 s;
        // 10 % first at index 1, 90 % first at index 2 -> rise 1 s
        let m = step_metrics(&[0.0, 0.5, 1.2, 1.0, 1.0], 1.0, 0.0, 1.0, 0.02).unwrap();
        assert!(near(m.overshoot_percent, 20.0, 1e-9) && near(m.settling_time_s, 3.0, 1e-12));
        assert!(near(m.rise_time_s.unwrap(), 1.0, 1e-12) && m.final_error_fraction == 0.0);
        // a falling step (start 10 -> target 4) is the same by symmetry: 10, 7, 3.2, 4, 4
        let m = step_metrics(&[10.0, 7.0, 3.2, 4.0, 4.0], 1.0, 10.0, 4.0, 0.02).unwrap();
        assert!(
            near(m.overshoot_percent, 100.0 * 0.8 / 6.0, 1e-9)
                && near(m.settling_time_s, 3.0, 1e-12)
        );
        // never reaches 90 %: no rise time, settling at the end, error fraction 0.5
        let m = step_metrics(&[0.0, 0.2, 0.4, 0.5], 1.0, 0.0, 1.0, 0.02).unwrap();
        assert!(
            m.rise_time_s.is_none()
                && near(m.settling_time_s, 4.0, 1e-12)
                && near(m.final_error_fraction, 0.5, 1e-12)
        );
        // starts inside the band (a step of nothing): settling 0
        let m = step_metrics(&[1.0, 1.0], 1.0, 0.0, 1.0, 0.02).unwrap();
        assert_eq!(m.settling_time_s, 0.0);
    }

    #[test]
    fn test_ctl_step_metrics_band_edge_and_rejections() {
        // a sample exactly at the band edge counts as inside; just outside counts as outside
        let m = step_metrics(&[0.0, 1.02, 1.0], 1.0, 0.0, 1.0, 0.02).unwrap();
        assert_eq!(
            m.settling_time_s,
            0.0 + 0.0 * 1.0
                + if (1.02_f64 - 1.0).abs() > 0.02 {
                    2.0
                } else {
                    1.0
                }
        );
        assert!(step_metrics(&[0.0, 1.0], 1.0, 0.0, 1.0, 0.02).is_some());
        assert!(step_metrics(&[], 1.0, 0.0, 1.0, 0.02).is_none());
        assert!(step_metrics(&[0.0], 0.0, 0.0, 1.0, 0.02).is_none());
        assert!(step_metrics(&[0.0], 1.0, 1.0, 1.0, 0.02).is_none());
        assert!(step_metrics(&[0.0], 1.0, 0.0, 1.0, 0.0).is_none());
        assert!(step_metrics(&[0.0, f64::NAN], 1.0, 0.0, 1.0, 0.02).is_none());
        assert!(step_metrics(&[0.0], 1.0, 0.0, f64::INFINITY, 0.02).is_none());
    }
}
