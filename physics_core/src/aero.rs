use crate::types::DroneState;
use glam::{Vec3, Quat};

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

