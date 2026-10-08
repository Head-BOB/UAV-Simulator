use crate::surrogate::SurrogateHandle;
use crate::surrogate::aero_surrogate::query_aero;
use crate::types::DroneState;
use crate::types::SurrogateQueryResult;
use glam::{DQuat, DVec3};

const DRAG_COEFF_X: f64 = 1.0;
const DRAG_COEFF_Y: f64 = 1.0;
const DRAG_COEFF_Z: f64 = 1.5;

const FRONTAL_AREA_X: f64 = 0.02;
const FRONTAL_AREA_Y: f64 = 0.02;
const FRONTAL_AREA_Z: f64 = 0.08;

const PROP_DIAMETER: f64 = 0.25;
const THRUST_COEFF: f64 = 0.11;
const MAX_MOTOR_RPM: f64 = 10000.0;

const STD_TEMP: f64 = 288.15;
const STD_PRESSURE: f64 = 101325.0;
const TEMP_LAPSE_RATE: f64 = 0.0065;
const SPECIFIC_GAS_CONST: f64 = 287.05;
const GRAVITY: f64 = 9.80665;

#[derive(Debug)]
pub enum AeroError {
    AltitudeOutOfRange,
}

fn compute_air_density(altitude: f64) -> Result<f64, AeroError> {
    if !(0.0..=11000.0).contains(&altitude) {
        return Err(AeroError::AltitudeOutOfRange);
    }
    let temp_at_alt = STD_TEMP - (TEMP_LAPSE_RATE * altitude);
    let exponent = GRAVITY / (SPECIFIC_GAS_CONST * TEMP_LAPSE_RATE);

    let pressure_ratio = (temp_at_alt / STD_TEMP).powf(exponent);
    let pressure_at_alt = STD_PRESSURE * pressure_ratio;

    Ok(pressure_at_alt / (SPECIFIC_GAS_CONST * temp_at_alt))
}

pub fn get_drag(state: &DroneState, altitude: f64) -> Result<[f32; 3], AeroError> {
    let density = compute_air_density(altitude)?;

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

pub fn get_thrust(throttle: f32) -> Result<f32, AeroError> {
    let density = compute_air_density(0.0)?;
    let clamped_throttle = (throttle as f64).clamp(0.0, 1.0);
    let rps = (clamped_throttle * MAX_MOTOR_RPM) / 60.0;

    let thrust = THRUST_COEFF * density * rps.powi(2) * PROP_DIAMETER.powi(4);
    Ok(thrust as f32)
}

pub fn get_drag_with_fallback(
    state: &DroneState,
    altitude: f64,
    aero_handle: Option<&mut SurrogateHandle>,
) -> Result<([f32; 3], Option<SurrogateQueryResult>), AeroError> {
    if let Some(handle) = aero_handle
        && let Ok(result) = query_aero(handle, state)
    {
        return Ok((result.predicted_values, Some(result)));
    }

    Ok((get_drag(state, altitude)?, None))
}

#[cfg(test)]
mod tests {
    use super::*;

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

    // Remaining basic tests
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
    fn test_d1_zero_throttle() {
        assert_eq!(get_thrust(0.0).unwrap(), 0.0);
    }
    #[test]
    fn test_d5_throttle_clamping() {
        assert_eq!(get_thrust(-0.5).unwrap(), get_thrust(0.0).unwrap());
    }
}
