// Tier: F
#[cfg(feature = "onnx")]
use crate::surrogate::SurrogateHandle;
#[cfg(feature = "onnx")]
use crate::surrogate::aero_surrogate::{body_to_ned, query_aero};
#[cfg(feature = "onnx")]
use crate::types::SurrogateQueryResult;
use crate::types::{
    DroneState, EnvironmentInput, MAX_SITE_ALTITUDE_M, MAX_TEMP_OFFSET_C, MAX_WIND_COMPONENT_MPS,
    VehicleConfig, WindInput,
};
use glam::{DQuat, DVec3};

const DRAG_COEFF_X: f64 = 1.0;
const DRAG_COEFF_Y: f64 = 1.0;
const DRAG_COEFF_Z: f64 = 1.5;

const FRONTAL_AREA_X: f64 = 0.02;
const FRONTAL_AREA_Y: f64 = 0.02;
const FRONTAL_AREA_Z: f64 = 0.08;

const STD_TEMP: f64 = 288.15;
const STD_PRESSURE: f64 = 101325.0;
const TEMP_LAPSE_RATE: f64 = 0.0065;
const SPECIFIC_GAS_CONST: f64 = 287.05;
const GRAVITY: f64 = 9.80665;

#[derive(Debug)]
pub enum AeroError {
    AltitudeOutOfRange,
}

/// Computes the air density at a given altitude using the International Standard Atmosphere (ISA) model.
fn compute_air_density(altitude: f64) -> Result<f64, AeroError> {
    air_density(altitude, &EnvironmentInput::standard())
}

/// Air density `rho = p / (R T)` at the vehicle, kg/m^3 (interface v12).
///
/// Total height `h = env.site_altitude_m + altitude_m` (metres above sea level, 0..=11000, the
/// ISA troposphere range). Pressure is the ISA value at `h`: `p = p0 (T_isa(h) / T0)^(g / (R L))`.
/// Temperature is `T = T_isa(h) + env.temp_offset_c`, with `T_isa(h) = T0 - L h`. The standard
/// environment (both fields zero) gives exactly the v11 value, bit for bit. Humidity is ignored
/// (dry air, `R` = 287.05 J/(kg K)); the pressure of a real weather day is not modelled.
/// `env` must have passed `validate_environment` (so `T > 0`).
///
/// # Errors
/// `AltitudeOutOfRange` if `h` is outside 0..=11000 m or NaN.
pub fn air_density(altitude_m: f64, env: &EnvironmentInput) -> Result<f64, AeroError> {
    let total_m = env.site_altitude_m + altitude_m;
    if !(0.0..=11000.0).contains(&total_m) {
        return Err(AeroError::AltitudeOutOfRange);
    }
    let temp_isa = STD_TEMP - (TEMP_LAPSE_RATE * total_m);
    let exponent = GRAVITY / (SPECIFIC_GAS_CONST * TEMP_LAPSE_RATE);

    let pressure_ratio = (temp_isa / STD_TEMP).powf(exponent);
    let pressure_at_alt = STD_PRESSURE * pressure_ratio;

    Ok(pressure_at_alt / (SPECIFIC_GAS_CONST * (temp_isa + env.temp_offset_c)))
}

/// Why an `EnvironmentInput` was rejected.
#[derive(Debug, PartialEq, Eq)]
pub enum EnvError {
    /// A field is NaN or infinite.
    NonFinite,
    /// `site_altitude_m` is outside 0..=`MAX_SITE_ALTITUDE_M`.
    SiteAltitude,
    /// `|temp_offset_c|` is above `MAX_TEMP_OFFSET_C`.
    TempOffset,
}

/// Checks the environment: finite fields, site altitude in 0..=5000 m, |offset| <= 40 K.
/// Both bounds are inclusive.
///
/// # Errors
/// `NonFinite` first (NaN/Inf in either field), then `SiteAltitude`, then `TempOffset`.
pub fn validate_environment(env: &EnvironmentInput) -> Result<(), EnvError> {
    if !env.site_altitude_m.is_finite() || !env.temp_offset_c.is_finite() {
        return Err(EnvError::NonFinite);
    }
    if !(0.0..=MAX_SITE_ALTITUDE_M).contains(&env.site_altitude_m) {
        return Err(EnvError::SiteAltitude);
    }
    if env.temp_offset_c.abs() > MAX_TEMP_OFFSET_C {
        return Err(EnvError::TempOffset);
    }
    Ok(())
}

/// Why a `WindInput` was rejected.
#[derive(Debug, PartialEq, Eq)]
pub enum WindError {
    /// A component is NaN or infinite.
    NonFinite,
    /// A component's magnitude is above `MAX_WIND_COMPONENT_MPS`.
    TooLarge,
}

/// Checks that the wind is finite and each |component| <= `MAX_WIND_COMPONENT_MPS` (30 m/s).
/// Exactly 30.0 is accepted; the bound is on each NED component, not on the vector norm.
///
/// # Errors
/// `NonFinite` for NaN/Inf (checked first), `TooLarge` for a component beyond the bound.
pub fn validate_wind(wind: &WindInput) -> Result<(), WindError> {
    if wind.wind_ned_mps.iter().any(|c| !c.is_finite()) {
        return Err(WindError::NonFinite);
    }
    if wind
        .wind_ned_mps
        .iter()
        .any(|c| c.abs() > MAX_WIND_COMPONENT_MPS)
    {
        return Err(WindError::TooLarge);
    }
    Ok(())
}

/// Air-relative velocity `v_air = v_ground - wind`, NED frame, m/s.
///
/// The subtraction is done in f64 and narrowed once to the f32 state precision. With zero wind
/// the f32 -> f64 -> f32 round trip is exact, so the result is bit-identical to `v_ground`.
/// (`-0.0 - 0.0 = -0.0` keeps the sign of a negative zero.) `wind` must have passed `validate_wind`.
#[must_use]
pub fn air_relative_velocity(v_ground_ned_mps: [f32; 3], wind: &WindInput) -> [f32; 3] {
    let mut v_air = [0.0f32; 3];
    for (out, (v, w)) in v_air
        .iter_mut()
        .zip(v_ground_ned_mps.iter().zip(wind.wind_ned_mps.iter()))
    {
        // Narrowing to the f32 state precision is intended (see DroneState::velocity).
        #[allow(clippy::cast_possible_truncation)]
        {
            *out = (f64::from(*v) - *w) as f32;
        }
    }
    v_air
}

/// Computes the aerodynamic drag force vector.
pub fn get_drag(state: &DroneState, altitude: f64) -> Result<[f32; 3], AeroError> {
    get_drag_env(state, altitude, &EnvironmentInput::standard())
}

/// `get_drag` in the given atmosphere (`air_density`); the standard environment is bit-identical.
pub fn get_drag_env(
    state: &DroneState,
    altitude: f64,
    env: &EnvironmentInput,
) -> Result<[f32; 3], AeroError> {
    let density = air_density(altitude, env)?;

    let vel_world = DVec3::new(
        state.velocity[0] as f64,
        state.velocity[1] as f64,
        state.velocity[2] as f64,
    );
    let rot_world = DQuat::from_xyzw(
        state.orientation[0] as f64,
        state.orientation[1] as f64,
        state.orientation[2] as f64,
        state.orientation[3] as f64,
    );

    let vel_body = rot_world.inverse() * vel_world;

    let calc_axis_drag = |v: f64, coeff: f64, area: f64| -> f64 {
        if v == 0.0 {
            return 0.0;
        }
        -0.5 * density * v.powi(2) * v.signum() * coeff * area
    };

    let drag_body = DVec3::new(
        calc_axis_drag(vel_body.x, DRAG_COEFF_X, FRONTAL_AREA_X),
        calc_axis_drag(vel_body.y, DRAG_COEFF_Y, FRONTAL_AREA_Y),
        calc_axis_drag(vel_body.z, DRAG_COEFF_Z, FRONTAL_AREA_Z),
    );

    let drag_world = rot_world * drag_body;

    Ok([
        drag_world.x as f32,
        drag_world.y as f32,
        drag_world.z as f32,
    ])
}

/// Computes the thrust generated by a single propeller at zero airspeed.
///
/// `T = C_T * rho(h) * n^2 * D^4` with `n = throttle * max_rpm / 60`.
///
/// # Units
/// * `throttle` - unitless, clamped to 0..=1.
/// * `altitude_m` - metres above sea level, sets `rho` (ISA range 0..=11000).
/// * Returns thrust in newtons (N), f64.
///
/// # Errors
/// `AltitudeOutOfRange` if `altitude_m` is outside the ISA range (including NaN).
pub fn get_thrust(
    throttle: f64,
    config: &VehicleConfig,
    altitude_m: f64,
) -> Result<f64, AeroError> {
    get_thrust_env(throttle, config, altitude_m, &EnvironmentInput::standard())
}

/// `get_thrust` in the given atmosphere (`air_density`); the standard environment is bit-identical.
///
/// # Errors
/// `AltitudeOutOfRange` if the total height is outside 0..=11000 m (including NaN).
pub fn get_thrust_env(
    throttle: f64,
    config: &VehicleConfig,
    altitude_m: f64,
    env: &EnvironmentInput,
) -> Result<f64, AeroError> {
    let density = air_density(altitude_m, env)?;
    let clamped_throttle = throttle.clamp(0.0, 1.0);
    let rps = (clamped_throttle * config.max_rpm) / 60.0;

    Ok(config.thrust_coeff * density * rps.powi(2) * config.prop_diameter_m.powi(4))
}

/// Derives the throttle at which four equal motors balance the vehicle weight.
///
/// Inverse of `get_thrust` at zero airspeed:
/// `4 * C_T * rho * (tau * n_max / 60)^2 * D^4 = m * g`
/// so `tau = (60 / n_max) * sqrt(m * g / (4 * C_T * rho * D^4))`.
///
/// # Units
/// * `altitude_m` - metres, ISA range 0..=11000 (sets `rho`).
/// * Returns the unitless throttle. It is NOT clamped: a value above 1.0 means
///   the vehicle cannot hover, and a non-finite value means bad config inputs.
///   The caller must validate the result.
///
/// # Errors
/// `AltitudeOutOfRange` if `altitude_m` is outside the ISA model range.
pub fn hover_throttle(config: &VehicleConfig, altitude_m: f64) -> Result<f64, AeroError> {
    let density = compute_air_density(altitude_m)?;
    let weight_n = config.mass_kg * GRAVITY;
    let max_thrust_per_motor_n = config.thrust_coeff
        * density
        * (config.max_rpm / 60.0).powi(2)
        * config.prop_diameter_m.powi(4);
    Ok((weight_n / (4.0 * max_thrust_per_motor_n)).sqrt())
}

/// The aero surrogate returns its force in the body frame; the integrator path takes NED.
/// Rotates `predicted_values` with the (x, y, z, w) body -> NED quaternion.
#[cfg(feature = "onnx")]
fn result_to_ned(
    mut result: SurrogateQueryResult,
    orientation_xyzw: [f32; 4],
) -> SurrogateQueryResult {
    result.predicted_values = body_to_ned(result.predicted_values, orientation_xyzw);
    result
}

/// Calculates aerodynamic drag, utilizing the surrogate model if available,
/// or falling back to the Phase 1 analytical ISA model. The returned force is NED either way.
#[cfg(feature = "onnx")]
pub fn get_drag_with_fallback(
    state: &DroneState,
    altitude: f64,
    aero_handle: Option<&mut SurrogateHandle>,
) -> Result<([f32; 3], Option<SurrogateQueryResult>), AeroError> {
    get_drag_with_fallback_env(state, altitude, &EnvironmentInput::standard(), aero_handle)
}

/// `get_drag_with_fallback` in the given atmosphere. Only the analytic fallback uses the density:
/// a surrogate answer was trained at one fixed density and ignores `env` (documented limitation).
#[cfg(feature = "onnx")]
pub fn get_drag_with_fallback_env(
    state: &DroneState,
    altitude: f64,
    env: &EnvironmentInput,
    aero_handle: Option<&mut SurrogateHandle>,
) -> Result<([f32; 3], Option<SurrogateQueryResult>), AeroError> {
    if let Some(handle) = aero_handle
        && let Ok(result) = query_aero(handle, state)
    {
        let result = result_to_ned(result, state.orientation);
        return Ok((result.predicted_values, Some(result)));
    }

    Ok((get_drag_env(state, altitude, env)?, None))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "onnx")]
    #[test]
    fn test_result_to_ned_rotates_force_and_keeps_flags() {
        // Yaw +90 deg about Z (q = (0,0,sin45,cos45)): body X (2, 0, 0) -> NED (0, 2, 0).
        // Body Z is unchanged by a yaw: (0, 0, 5) -> (0, 0, 5). tol 1e-6 (f32 q).
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let res = SurrogateQueryResult {
            predicted_values: [2.0, 0.0, 5.0],
            uncertainty: 0.25,
            in_validated_envelope: 0,
        };
        let out = result_to_ned(res, [0.0, 0.0, s, s]);
        assert!(out.predicted_values[0].abs() < 1e-6);
        assert!((out.predicted_values[1] - 2.0).abs() < 1e-6);
        assert!((out.predicted_values[2] - 5.0).abs() < 1e-6);
        assert_eq!(out.uncertainty, 0.25);
        assert_eq!(out.in_validated_envelope, 0);
    }

    fn dummy_config() -> VehicleConfig {
        VehicleConfig {
            mass_kg: 1.0,
            inertia_kgm2: [1.0, 1.0, 1.0],
            motor_pos_m: [
                [0.2, 0.2, 0.0],
                [0.2, -0.2, 0.0],
                [-0.2, 0.2, 0.0],
                [-0.2, -0.2, 0.0],
            ],
            motor_spin: [1.0, -1.0, -1.0, 1.0],
            prop_diameter_m: 0.25,
            thrust_coeff: 0.11,
            torque_coeff: 0.05,
            max_rpm: 10000.0,
            hover_throttle: 0.4095,
        }
    }

    #[test]
    fn test_b1_isa_values() {
        let cases = [
            (0.0, 1.2250),
            (500.0, 1.16727),
            (1000.0, 1.11164),
            (5000.0, 0.73611),
            (11000.0, 0.36391),
        ];
        for (alt, expected) in cases {
            let density = compute_air_density(alt).unwrap();
            assert!(
                (density - expected).abs() / expected < 1e-4,
                "Failed at {}m. Expected {}, got {}",
                alt,
                expected,
                density
            );
        }
    }

    #[test]
    fn test_b3_density_decreases_with_altitude() {
        assert!(compute_air_density(500.0).unwrap() < compute_air_density(0.0).unwrap());
        assert!(compute_air_density(1000.0).unwrap() < compute_air_density(500.0).unwrap());
    }

    #[test]
    fn test_b4_troposphere_limit() {
        let d = compute_air_density(11000.0).unwrap();
        assert!(d > 0.0 && d < 1.225);
        assert!(!d.is_nan());
        assert!(compute_air_density(12000.0).is_err());
    }

    #[test]
    fn test_b2_altitude_500m() {
        let d = compute_air_density(500.0).unwrap();
        assert!((d - 1.167).abs() < 0.005);
    }

    #[test]
    fn test_c1_zero_velocity() {
        let state = DroneState {
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };
        assert_eq!(get_drag(&state, 0.0).unwrap(), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_c2_drag_opposes_motion() {
        let state_x = DroneState {
            position: [0.0, 0.0, 0.0],
            velocity: [5.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };
        assert!(get_drag(&state_x, 0.0).unwrap()[0] < 0.0);
    }

    #[test]
    fn test_c3_drag_scales_with_velocity_squared() {
        let state_5 = DroneState {
            position: [0.0, 0.0, 0.0],
            velocity: [5.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };
        let state_10 = DroneState {
            position: [0.0, 0.0, 0.0],
            velocity: [10.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };
        let drag_5 = get_drag(&state_5, 0.0).unwrap()[0].abs();
        let drag_10 = get_drag(&state_10, 0.0).unwrap()[0].abs();
        assert!((drag_10 - (drag_5 * 4.0)).abs() < 0.1);
    }

    #[test]
    fn test_c4_different_axes_different_drag() {
        let drag_x = get_drag(
            &DroneState {
                position: [0.0, 0.0, 0.0],
                velocity: [10.0, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                angular_velocity: [0.0, 0.0, 0.0],
            },
            0.0,
        )
        .unwrap()[0]
            .abs();
        let drag_z = get_drag(
            &DroneState {
                position: [0.0, 0.0, 0.0],
                velocity: [0.0, 0.0, 10.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                angular_velocity: [0.0, 0.0, 0.0],
            },
            0.0,
        )
        .unwrap()[2]
            .abs();
        assert!(drag_z > drag_x);
    }

    #[test]
    fn test_d1_zero_throttle() {
        let cfg = dummy_config();
        assert_eq!(get_thrust(0.0, &cfg, 0.0).unwrap(), 0.0);
    }

    #[test]
    fn test_d2_thrust_scales_with_throttle_squared() {
        let cfg = dummy_config();
        let t_025 = get_thrust(0.25, &cfg, 0.0).unwrap();
        let t_050 = get_thrust(0.5, &cfg, 0.0).unwrap();
        assert!((t_050 - (t_025 * 4.0)).abs() < 0.1);
    }

    #[test]
    fn test_d5_throttle_clamping() {
        let cfg = dummy_config();
        assert_eq!(
            get_thrust(-0.5, &cfg, 0.0).unwrap(),
            get_thrust(0.0, &cfg, 0.0).unwrap()
        );
        assert_eq!(
            get_thrust(1.5, &cfg, 0.0).unwrap(),
            get_thrust(1.0, &cfg, 0.0).unwrap()
        );
    }

    // Hand calc (python3, f64), rho0 = 101325 / (287.05 * 288.15) = 1.2250122659906946:
    // T_max = 0.11 * rho0 * (10000/60)^2 * 0.25^4 = 14.621457167857683 N per motor.
    // tau = sqrt(m * 9.80665 / (4 * T_max)): m = 1.5 -> 0.5015112082212391,
    // m = 1.0 -> 0.4094821868095746.
    #[test]
    fn test_hover_throttle_shipped_config() {
        let mut cfg = dummy_config();
        cfg.mass_kg = 1.5;
        let tau = hover_throttle(&cfg, 0.0).unwrap();
        assert!((tau - 0.5015112082212391).abs() < 1e-12, "tau = {tau}");
    }

    // Reference build (APC 12x6E, MN3508): offline_pipeline/geometry/quad_ref_build_config.json loads
    // (validated) and hovers at tau = sqrt(m g / (4 Ct rho0 (rpm/60)^2 D^4)) with m = 1.516 kg,
    // Ct = 0.100256, rho0 = 1.2250122659906946, rpm = 8100, D = 0.3048 m
    // = sqrt(14.8668314 / 77.2678...) = 0.4386224371767256 (python3, f64) -> 3552.84 RPM.
    // Inputs are the UIUC static fit and ASSUMED catalog masses: the config is not validated.
    #[test]
    fn test_hover_throttle_reference_build_config() {
        let cfg = crate::config::load_vehicle_config(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../offline_pipeline/geometry/quad_ref_build_config.json"
        ))
        .unwrap();
        let tau = hover_throttle(&cfg, 0.0).unwrap();
        assert!((tau - 0.4386224371767256).abs() < 1e-12, "tau = {tau}");
        assert!((cfg.hover_throttle - tau).abs() < 1e-15);
    }

    #[test]
    fn test_hover_throttle_one_kg() {
        let cfg = dummy_config(); // mass_kg = 1.0
        let tau = hover_throttle(&cfg, 0.0).unwrap();
        assert!((tau - 0.4094821868095746).abs() < 1e-12, "tau = {tau}");
    }

    // 4 motors at tau must give back the weight: 1.5 * 9.80665 = 14.709975 N.
    // f64 hand calc: 4 * T(tau) = 14.709974999999996 (relative error 2.4e-16), so 1e-12 relative.
    #[test]
    fn test_hover_throttle_balances_weight() {
        let mut cfg = dummy_config();
        cfg.mass_kg = 1.5;
        let tau = hover_throttle(&cfg, 0.0).unwrap();
        let total_n = 4.0 * get_thrust(tau, &cfg, 0.0).unwrap();
        let weight_n = 14.709975;
        assert!(
            ((total_n - weight_n) / weight_n).abs() < 1e-12,
            "total = {total_n}"
        );
    }

    // Hand calc (python3, f64): T = 0.11 * rho0 * (10000/60)^2 * 0.25^4,
    // rho0 = 1.2250122659906946 -> 14.621457167857683 N. Rounding through f32 would be off by 6.8e-8.
    #[test]
    fn test_thrust_full_throttle_sea_level_f64() {
        let t = get_thrust(1.0, &dummy_config(), 0.0).unwrap();
        assert!((t - 14.621457167857683).abs() < 1e-12, "t = {t}");
    }

    // Hand calc (python3, f64): rho(1000 m) = 1.1116522819524908; T(0.5, 1000 m) = 3.3171047909823668 N;
    // T(0.5, 0 m) = 3.6553642919644207 N; ratio = rho(1000)/rho(0) = 0.9074621641061471.
    #[test]
    fn test_thrust_uses_altitude_density() {
        let cfg = dummy_config();
        let t_alt = get_thrust(0.5, &cfg, 1000.0).unwrap();
        let t_sl = get_thrust(0.5, &cfg, 0.0).unwrap();
        assert!(
            (t_alt - 3.3171047909823668).abs() < 1e-12,
            "t_alt = {t_alt}"
        );
        assert!((t_alt / t_sl - 0.9074621641061471).abs() < 1e-12);
    }

    #[test]
    fn test_thrust_altitude_out_of_range() {
        let cfg = dummy_config();
        for alt in [-1.0, 11001.0, f64::NAN] {
            assert!(matches!(
                get_thrust(0.5, &cfg, alt),
                Err(AeroError::AltitudeOutOfRange)
            ));
        }
    }

    #[test]
    fn test_hover_throttle_altitude_out_of_range() {
        let cfg = dummy_config();
        for alt in [-1.0, 11001.0] {
            assert!(matches!(
                hover_throttle(&cfg, alt),
                Err(AeroError::AltitudeOutOfRange)
            ));
        }
    }
    fn wind_state(velocity: [f32; 3], orientation: [f32; 4]) -> DroneState {
        DroneState {
            position: [0.0, 0.0, 0.0],
            velocity,
            orientation,
            angular_velocity: [0.0, 0.0, 0.0],
        }
    }

    fn wind_of(w: [f64; 3]) -> WindInput {
        WindInput { wind_ned_mps: w }
    }

    #[test]
    fn test_wind_zero_wind_velocity_is_bit_identical() {
        // v - 0.0 is exact in f64 for every f32 value, and the narrowing back to f32 is exact,
        // so zero wind must return the identical bits (including -0.0 and a subnormal).
        let cases = [
            [3.0f32, -2.0, 1.0],
            [-0.0, 0.0, -0.0],
            [1.0e-45, -1.0e-45, 0.1],
            [123456.78, -0.3, f32::MAX],
        ];
        for v in cases {
            let out = air_relative_velocity(v, &WindInput::still());
            for k in 0..3 {
                assert_eq!(out[k].to_bits(), v[k].to_bits(), "component {k} of {v:?}");
            }
        }
    }

    #[test]
    fn test_wind_zero_wind_drag_is_bit_identical() {
        // Moving 3 m/s north, yawed 30 deg: the drag computed from the air-relative velocity
        // with zero wind must equal the drag computed from the ground velocity, bit for bit.
        let half = 15.0_f32.to_radians();
        let q = [0.0, 0.0, half.sin(), half.cos()];
        let ground = wind_state([3.0, -2.0, 1.0], q);
        let air = wind_state(
            air_relative_velocity(ground.velocity, &WindInput::still()),
            q,
        );
        let a = get_drag(&ground, 0.0).unwrap();
        let b = get_drag(&air, 0.0).unwrap();
        for k in 0..3 {
            assert_eq!(a[k].to_bits(), b[k].to_bits());
        }
    }

    #[test]
    fn test_wind_head_wind_at_hover_drag_opposes_apparent_air_motion() {
        // Hover (v_ground = 0), identity attitude (body X = north), wind blowing TOWARD south:
        // wind_ned = (-5, 0, 0). v_air = 0 - (-5) = +5 m/s north: the vehicle moves north
        // relative to the air, so drag points south (-X) and equals
        // F = -0.5 * rho * v^2 * Cd * A = -0.5 * 1.2250 * 25 * 1.0 * 0.02 = -0.30625 N.
        // rho(0 m) = 1.2250 kg/m3 (ISA, same value and 1e-4 relative tolerance as test_b1_isa_values).
        let wind = wind_of([-5.0, 0.0, 0.0]);
        let v_air = air_relative_velocity([0.0; 3], &wind);
        assert_eq!(v_air, [5.0, 0.0, 0.0]);
        let drag = get_drag(&wind_state(v_air, [0.0, 0.0, 0.0, 1.0]), 0.0).unwrap();
        assert!((f64::from(drag[0]) + 0.30625).abs() < 0.30625 * 1e-4 + 1e-6);
        assert_eq!(drag[1], 0.0);
        assert_eq!(drag[2], 0.0);
    }

    #[test]
    fn test_wind_vertical_wind_uses_z_coefficient() {
        // wind_ned = (0, 0, -4): air moving UP (NED -Z). v_air = +4 m/s down relative to the air,
        // so drag points up (-Z): F = -0.5 * 1.2250 * 16 * 1.5 * 0.08 = -1.176 N.
        let wind = wind_of([0.0, 0.0, -4.0]);
        let v_air = air_relative_velocity([0.0; 3], &wind);
        let drag = get_drag(&wind_state(v_air, [0.0, 0.0, 0.0, 1.0]), 0.0).unwrap();
        assert!((f64::from(drag[2]) + 1.176).abs() < 1.176 * 1e-4 + 1e-6);
        assert_eq!(drag[0], 0.0);
        assert_eq!(drag[1], 0.0);
    }

    #[test]
    fn test_wind_equal_to_vehicle_velocity_gives_zero_drag() {
        // v_air = v_ground - wind = (3, -2, 1) - (3, -2, 1) = 0 exactly (all values exact in f32),
        // and get_drag returns exactly 0 for a zero velocity, in any attitude.
        let half = 15.0_f32.to_radians();
        let q = [0.0, 0.0, half.sin(), half.cos()];
        let wind = wind_of([3.0, -2.0, 1.0]);
        let v_air = air_relative_velocity([3.0, -2.0, 1.0], &wind);
        assert_eq!(v_air, [0.0, 0.0, 0.0]);
        assert_eq!(get_drag(&wind_state(v_air, q), 0.0).unwrap(), [0.0; 3]);
    }

    #[test]
    fn test_wind_validation_rejects_non_finite_and_over_30() {
        assert_eq!(validate_wind(&WindInput::still()), Ok(()));
        // The bound is inclusive: exactly +-30 is accepted.
        assert_eq!(validate_wind(&wind_of([30.0, -30.0, 30.0])), Ok(()));
        for k in 0..3 {
            for (bad, want) in [
                (f64::NAN, WindError::NonFinite),
                (f64::INFINITY, WindError::NonFinite),
                (f64::NEG_INFINITY, WindError::NonFinite),
                (31.0, WindError::TooLarge),
                (-31.0, WindError::TooLarge),
                (30.000001, WindError::TooLarge),
            ] {
                let mut w = [1.0, 2.0, 3.0];
                w[k] = bad;
                assert_eq!(
                    validate_wind(&wind_of(w)),
                    Err(want),
                    "component {k}, value {bad}"
                );
            }
        }
    }

    // v12 environment. Reference values are from an independent f64 python3 evaluation of
    // rho = p / (R T), p = p0 (T_isa / T0)^(g / (R L)), T = T_isa(h) + dT, with the constants of this file
    // (p0 = 101325, T0 = 288.15, L = 0.0065, R = 287.05, g = 9.80665). exponent g / (R L) = 5.255932362359814.
    fn env(site_altitude_m: f64, temp_offset_c: f64) -> EnvironmentInput {
        EnvironmentInput {
            site_altitude_m,
            temp_offset_c,
        }
    }

    #[test]
    fn test_env_standard_density_is_bit_identical_to_isa() {
        for alt in [0.0, 1.0, 123.456, 500.0, 5000.0, 11000.0] {
            let a = air_density(alt, &EnvironmentInput::standard()).unwrap();
            let b = compute_air_density(alt).unwrap();
            assert_eq!(a.to_bits(), b.to_bits(), "alt {alt}");
        }
    }

    // Sea level, standard day: 101325 / (287.05 * 288.15) = 1.2250122659906946 kg/m3 (python3, f64).
    // Tolerance 1e-12: only the last bits of the powf/divide chain can differ.
    #[test]
    fn test_env_sea_level_standard_day_density() {
        let rho = air_density(0.0, &env(0.0, 0.0)).unwrap();
        assert!((rho - 1.2250122659906946).abs() < 1e-12, "rho = {rho}");
    }

    // Site 3000 m, air 5 C: T_isa(3000) = 288.15 - 0.0065 * 3000 = 268.65 K = -4.5 C, so dT = 5 - (-4.5) = +9.5 K.
    // p = 101325 * (268.65 / 288.15)^5.255932362359814 = 101325 * 0.6919148121361276 = 70108.26833969312 Pa.
    // T = 268.65 + 9.5 = 278.15 K; rho = 70108.26833969312 / (287.05 * 278.15) = 0.8780770469291939 kg/m3.
    #[test]
    fn test_env_3000_m_5_c_density_hand_calc() {
        let rho = air_density(0.0, &env(3000.0, 9.5)).unwrap();
        assert!((rho - 0.8780770469291939).abs() < 1e-12, "rho = {rho}");
    }

    // Sea level, +20 K: rho = 101325 / (287.05 * 308.15) = 1.1455047361519344; -20 K: 1.3163799531800062.
    #[test]
    fn test_env_temperature_offset_sign() {
        let hot = air_density(0.0, &env(0.0, 20.0)).unwrap();
        let cold = air_density(0.0, &env(0.0, -20.0)).unwrap();
        assert!((hot - 1.1455047361519344).abs() < 1e-12, "hot = {hot}");
        assert!((cold - 1.3163799531800062).abs() < 1e-12, "cold = {cold}");
        assert!(hot < cold);
    }

    // Site 3000 m + vehicle 100 m is the same total height as site 3100 m + 0 m. Hand value at
    // total 3100 m, dT = 9.5: 0.8689988332494722 kg/m3 (python3). Integer sums are exact, so bit-equal.
    #[test]
    fn test_env_site_and_vehicle_altitude_add() {
        let a = air_density(100.0, &env(3000.0, 9.5)).unwrap();
        let b = air_density(0.0, &env(3100.0, 9.5)).unwrap();
        assert_eq!(a.to_bits(), b.to_bits());
        assert!((a - 0.8689988332494722).abs() < 1e-12, "a = {a}");
    }

    #[test]
    fn test_env_total_height_range() {
        // Total height 11000 is the last accepted value; 11000.5, negative and NaN are errors.
        assert!(air_density(6000.0, &env(5000.0, 0.0)).is_ok());
        assert!(matches!(
            air_density(6000.5, &env(5000.0, 0.0)),
            Err(AeroError::AltitudeOutOfRange)
        ));
        assert!(matches!(
            air_density(-1.0, &env(0.0, 0.0)),
            Err(AeroError::AltitudeOutOfRange)
        ));
        // A vehicle below the origin is fine when the site is high enough: total 999 m.
        assert!(air_density(-1.0, &env(1000.0, 0.0)).is_ok());
        assert!(air_density(f64::NAN, &env(0.0, 0.0)).is_err());
    }

    // Thrust is proportional to density: T(hot, +20 K) / T(standard) = 1.1455047361519344 / 1.2250122659906946
    // = 0.9350965438909621 (python3).
    #[test]
    fn test_env_thrust_scales_with_density() {
        let cfg = dummy_config();
        let t_std = get_thrust_env(0.5, &cfg, 0.0, &env(0.0, 0.0)).unwrap();
        let t_hot = get_thrust_env(0.5, &cfg, 0.0, &env(0.0, 20.0)).unwrap();
        assert!((t_hot / t_std - 0.9350965438909621).abs() < 1e-12);
        assert_eq!(
            t_std.to_bits(),
            get_thrust(0.5, &cfg, 0.0).unwrap().to_bits()
        );
    }

    // Analytic drag is proportional to density too: |F| at +20 K is 0.9350965438909621 of the standard day.
    #[test]
    fn test_env_drag_scales_with_density() {
        let mut state = DroneState {
            velocity: [4.0, 0.0, 0.0],
            ..crate::ffi_create_default_drone_state()
        };
        state.orientation = [0.0, 0.0, 0.0, 1.0];
        let std = get_drag_env(&state, 0.0, &env(0.0, 0.0)).unwrap();
        let hot = get_drag_env(&state, 0.0, &env(0.0, 20.0)).unwrap();
        assert!(std[0] < 0.0);
        assert!((f64::from(hot[0]) / f64::from(std[0]) - 0.9350965438909621).abs() < 1e-6);
        assert_eq!(std, get_drag(&state, 0.0).unwrap());
    }

    #[test]
    fn test_validate_environment_truth_table() {
        let ok = |a: f64, t: f64| validate_environment(&env(a, t)).is_ok();
        // Inclusive bounds: site 0 and 5000, offset -40 and +40.
        assert!(ok(0.0, 0.0) && ok(5000.0, 0.0) && ok(0.0, 40.0) && ok(0.0, -40.0));
        assert_eq!(
            validate_environment(&env(-0.1, 0.0)),
            Err(EnvError::SiteAltitude)
        );
        assert_eq!(
            validate_environment(&env(5000.0001, 0.0)),
            Err(EnvError::SiteAltitude)
        );
        assert_eq!(
            validate_environment(&env(0.0, 40.0001)),
            Err(EnvError::TempOffset)
        );
        assert_eq!(
            validate_environment(&env(0.0, -40.0001)),
            Err(EnvError::TempOffset)
        );
        // NaN / Inf in either field is NonFinite, checked before the range.
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                validate_environment(&env(bad, 0.0)),
                Err(EnvError::NonFinite)
            );
            assert_eq!(
                validate_environment(&env(0.0, bad)),
                Err(EnvError::NonFinite)
            );
        }
    }
}
