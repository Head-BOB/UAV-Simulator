use crate::types::DroneState;
use glam::{Quat, Vec3};

// Drag Coefficients (Unitless - how draggy the shape is)
const DRAG_COEFF_X: f32 = 1.0;
const DRAG_COEFF_Y: f32 = 1.0;
const DRAG_COEFF_Z: f32 = 1.5;

// Frontal Areas (Square meters)
const FRONTAL_AREA_X: f32 = 0.02;
const FRONTAL_AREA_Y: f32 = 0.02;
const FRONTAL_AREA_Z: f32 = 0.08;

// Propeller & Motor
const PROP_DIAMETER: f32 = 0.25; // meters
const THRUST_COEFF: f32 = 0.11; // unitless placeholder
const MAX_MOTOR_RPM: f32 = 10000.0;

// Universal Physics Constants for Atmosphere
const STD_TEMP: f32 = 288.15; // Kelvin
const STD_PRESSURE: f32 = 101325.0; // Pascals
const TEMP_LAPSE_RATE: f32 = 0.0065; // Kelvin per meter
const SPECIFIC_GAS_CONST: f32 = 287.05;
const GRAVITY: f32 = 9.80665;

// Air Density Helper Function
fn compute_air_density(altitude: f32) -> f32 {
    // 1. Calculate temperature at altitude
    let temp_at_alt = STD_TEMP - (TEMP_LAPSE_RATE * altitude);

    // 2. Calculate pressure at altitude
    let exponent = GRAVITY / (SPECIFIC_GAS_CONST * TEMP_LAPSE_RATE);
    let pressure_at_alt = STD_PRESSURE * (temp_at_alt / STD_TEMP).powf(exponent);

    // 3. Calculate and return density (kg/m^3)
    pressure_at_alt / (SPECIFIC_GAS_CONST * temp_at_alt)
}

pub fn get_drag(state: &DroneState, altitude: f32) -> [f32; 3] {
    // 1. Calculate current air density using the ISA model helper
    let density = compute_air_density(altitude);

    // 2. Convert world-space state arrays into glam math types
    let vel_world = Vec3::from_array(state.velocity);
    let rot_world = Quat::from_array(state.orientation);

    // 3. Rotate world-space velocity into the drone's body frame using the inverse orientation
    let vel_body = rot_world.inverse() * vel_world;

    // Helper closure to calculate drag on a single axis, handling the exactly-zero velocity case
    let calc_axis_drag = |v: f32, coeff: f32, area: f32| -> f32 {
        if v == 0.0 {
            return 0.0;
        }
        // Aerodynamic drag equation: -0.5 * rho * v^2 * Cd * A
        // v.signum() ensures the force strictly opposes the direction of motion
        -0.5 * density * v.powi(2) * v.signum() * coeff * area
    };

    // 4, 5, 6, 7. Calculate drag for each body axis and assemble into a single body-frame force vector
    let drag_body = Vec3::new(
        calc_axis_drag(vel_body.x, DRAG_COEFF_X, FRONTAL_AREA_X),
        calc_axis_drag(vel_body.y, DRAG_COEFF_Y, FRONTAL_AREA_Y),
        calc_axis_drag(vel_body.z, DRAG_COEFF_Z, FRONTAL_AREA_Z),
    );

    // 8. Rotate the body-frame drag vector back into world-space coordinates
    let drag_world = rot_world * drag_body;

    // 9. Return as a standard array expected by the FFI boundary
    drag_world.to_array()
}

/* #[cfg(test)]
mod tests {
    use super::*;
    use crate::types::DroneState;

    fn create_test_state(vel: [f32; 3]) -> DroneState {
        DroneState {
            position: [0.0, 0.0, 0.0],
            velocity: vel,
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, 0.0],
        }
    }

    // GROUP B: ISA Air Density Tests
    #[test]
    fn test_b1_sea_level_density() {
        let d = compute_air_density(0.0);
        // Sea level density should be approximately 1.225
        assert!((d - 1.225).abs() < 0.01, "Expected ~1.225, but got {}", d);
    }

    #[test]
    fn test_b3_density_decreases_with_altitude() {
        // Density should be lower at 500m than 0m, and lower at 1000m than 500m
        assert!(compute_air_density(500.0) < compute_air_density(0.0));
        assert!(compute_air_density(1000.0) < compute_air_density(500.0));
    }

    // GROUP C: Drag Force Tests
    #[test]
    fn test_c1_zero_velocity() {
        let state = create_test_state([0.0, 0.0, 0.0]);
        let drag = get_drag(&state, 0.0);
        assert_eq!(drag, [0.0, 0.0, 0.0], "Zero velocity must produce zero drag");
    }

    #[test]
    fn test_c2_drag_opposes_motion() {
        // Moving positive on X axis should produce negative drag on X axis
        let state_x = create_test_state([5.0, 0.0, 0.0]);
        let drag_x = get_drag(&state_x, 0.0);
        assert!(drag_x[0] < 0.0, "Drag on X axis is not opposing motion!");
        assert_eq!(drag_x[1], 0.0);
        assert_eq!(drag_x[2], 0.0);
    }

    #[test]
    fn test_c3_drag_scales_with_velocity_squared() {
        let state_5 = create_test_state([5.0, 0.0, 0.0]);
        let state_10 = create_test_state([10.0, 0.0, 0.0]);

        let drag_5 = get_drag(&state_5, 0.0)[0].abs();
        let drag_10 = get_drag(&state_10, 0.0)[0].abs();

        // Speed is doubled (5 to 10), so drag should roughly quadruple (x4)
        assert!((drag_10 - (drag_5 * 4.0)).abs() < 0.1, "Drag did not scale by v-squared");
    }

    // GROUP D: Propeller Thrust Tests
    #[test]
    fn test_d1_zero_throttle() {
        assert_eq!(get_thrust(0.0), 0.0, "Zero throttle must produce zero thrust");
    }

    #[test]
    fn test_d2_thrust_scales_with_throttle_squared() {
        let t_025 = get_thrust(0.25);
        let t_050 = get_thrust(0.5);
        // Throttle is doubled, RPS is doubled, so thrust should roughly quadruple (x4)
        assert!((t_050 - (t_025 * 4.0)).abs() < 0.1, "Thrust did not scale correctly");
    }

    #[test]
    fn test_d5_throttle_clamping() {
        // Out of range throttles (-0.5 and 1.5) should be clamped to 0.0 and 1.0
        assert_eq!(get_thrust(-0.5), get_thrust(0.0), "Negative throttle not clamped");
        assert_eq!(get_thrust(1.5), get_thrust(1.0), "Excessive throttle not clamped");
    }

    #[test]
    fn test_b2_altitude_500m() {
        let d = compute_air_density(500.0);
        // PDF Document says: Approximately 1.167 at 500 meters
        assert!((d - 1.167).abs() < 0.005, "500m density should be ~1.167, got {}", d);
    }

    #[test]
    fn test_b4_troposphere_limit() {
        let d = compute_air_density(11000.0);
        // Checking if formula breaks at max height (11,000m)
        assert!(d > 0.0 && d < 1.225, "11000m density must be positive but less than sea level");
        assert!(!d.is_nan(), "Density calculation resulted in NaN (Not a Number)");
    }

    #[test]
    fn test_c4_different_axes_different_drag() {
        // Checking if Z axis (Up/Down) gives more drag than X axis (Forward)
        // because we set DRAG_COEFF_Z to 1.5 and DRAG_COEFF_X to 1.0
        let drag_x = get_drag(&create_test_state([10.0, 0.0, 0.0]), 0.0)[0].abs();
        let drag_z = get_drag(&create_test_state([0.0, 0.0, 10.0]), 0.0)[2].abs();
        assert!(drag_z > drag_x, "Z axis should have more drag than X axis based on our constants");
    }
} */

pub fn get_thrust(throttle: f32) -> f32 {
    // 1. Get air density. Since altitude is not passed to this specific function signature,
    // we assume sea-level altitude (0.0 meters) for the propeller thrust calculation in Phase 1.
    let density = compute_air_density(0.0);

    // 2. Clamp the incoming throttle value to ensure it strictly stays within the 0.0 to 1.0 range
    let clamped_throttle = throttle.clamp(0.0, 1.0);

    // 3. Convert linear throttle to rotational speed in Revolutions Per Second (RPS).
    // Note: This linear throttle-to-RPM relationship is a documented Phase 1 simplification.
    let rps = (clamped_throttle * MAX_MOTOR_RPM) / 60.0;

    // 4. Calculate thrust using the standard propeller thrust equation:
    // Thrust = Ct * rho * n^2 * D^4 (where n is RPS)
    THRUST_COEFF * density * rps.powi(2) * PROP_DIAMETER.powi(4)
}
