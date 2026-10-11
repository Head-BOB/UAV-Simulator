#![allow(clippy::needless_range_loop)]
use crate::types::{ControlInputs, VehicleConfig};
use glam::{DQuat, DVec3};

/// Sign conventions and mixing logic for the Quad-X airframe.
///
/// # Coordinate System (NED)
/// * X: North (Forward)
/// * Y: East (Right)
/// * Z: Down
///
/// # Motor Layout
/// * `Motor 0` (FR): +X, +Y.
/// * `Motor 1` (FL): +X, -Y.
/// * `Motor 2` (RR): -X, +Y.
/// * `Motor 3` (RL): -X, -Y.
///
/// # Control Sign Convention
/// * `Throttle`: Positive adds total thrust.
/// * `Roll`: Positive rolls right wing down (increases FL, RL; decreases FR, RR).
/// * `Pitch`: Positive pitches nose up (increases FR, FL; decreases RR, RL).
/// * `Yaw`: Positive yaws nose right (increases CCW motors; decreases CW motors).
pub fn mix_controls(controls: &ControlInputs) -> [f64; 4] {
    let t = controls.throttle as f64;
    let r = controls.roll as f64 * 0.2;
    let p = controls.pitch as f64 * 0.2;
    let y = controls.yaw as f64 * 0.2;

    [
        (t - r + p - y).clamp(0.0, 1.0), // FR
        (t + r + p + y).clamp(0.0, 1.0), // FL
        (t - r - p + y).clamp(0.0, 1.0), // RR
        (t + r - p - y).clamp(0.0, 1.0), // RL
    ]
}

/// Computes the net force and torque acting on the drone.
///
/// # Units
/// * `thrusts` - Newtons (N) per motor.
/// * `drag` - Newtons (N) in world frame.
/// * Returns a tuple of (net_force, net_torque, net_thrust) in f64 format.
pub fn calculate_net_forces(
    thrusts: [f64; 4],
    drag: [f64; 3],
    orientation: [f32; 4],
    config: &VehicleConfig,
) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let drag_vec = DVec3::from_array(drag);
    let rot_world = DQuat::from_xyzw(
        orientation[0] as f64,
        orientation[1] as f64,
        orientation[2] as f64,
        orientation[3] as f64,
    );

    let total_thrust = thrusts.iter().sum::<f64>();
    let thrust_body = DVec3::new(0.0, 0.0, -total_thrust);

    let thrust_world = rot_world * thrust_body;
    let gravity_world = DVec3::new(0.0, 0.0, config.mass_kg * 9.80665);
    let net_force = thrust_world + drag_vec + gravity_world;

    let mut net_torque = DVec3::ZERO;
    let mut yaw_reaction = 0.0;

    for i in 0..4 {
        let pos = DVec3::from_array(config.motor_pos_m[i]);
        net_torque += pos.cross(DVec3::new(0.0, 0.0, -thrusts[i]));
        yaw_reaction += config.motor_spin[i] * config.torque_coeff * thrusts[i];
    }

    net_torque.z += yaw_reaction;

    (
        net_force.to_array(),
        net_torque.to_array(),
        thrust_world.to_array(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

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
            motor_spin: [-1.0, 1.0, 1.0, -1.0],
            prop_diameter_m: 0.25,
            thrust_coeff: 0.11,
            torque_coeff: 0.05,
            max_rpm: 10000.0,
            hover_throttle: 0.4095,
        }
    }

    #[test]
    fn test_mixer_roll_positive() {
        let config = dummy_config();
        let inputs = ControlInputs {
            throttle: 0.5,
            roll: 0.5,
            pitch: 0.0,
            yaw: 0.0,
        };
        let throttles = mix_controls(&inputs);
        let thrusts = [
            throttles[0] * 10.0,
            throttles[1] * 10.0,
            throttles[2] * 10.0,
            throttles[3] * 10.0,
        ];
        let (_, torque, _) = calculate_net_forces(thrusts, [0.0; 3], [0.0, 0.0, 0.0, 1.0], &config);
        assert!(torque[0] > 0.0);
    }

    #[test]
    fn test_mixer_pitch_positive() {
        let config = dummy_config();
        let inputs = ControlInputs {
            throttle: 0.5,
            roll: 0.0,
            pitch: 0.5,
            yaw: 0.0,
        };
        let throttles = mix_controls(&inputs);
        let thrusts = [
            throttles[0] * 10.0,
            throttles[1] * 10.0,
            throttles[2] * 10.0,
            throttles[3] * 10.0,
        ];
        let (_, torque, _) = calculate_net_forces(thrusts, [0.0; 3], [0.0, 0.0, 0.0, 1.0], &config);
        assert!(torque[1] > 0.0);
    }

    #[test]
    fn test_mixer_yaw_positive() {
        let config = dummy_config();
        let inputs = ControlInputs {
            throttle: 0.5,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.5,
        };
        let throttles = mix_controls(&inputs);
        let thrusts = [
            throttles[0] * 10.0,
            throttles[1] * 10.0,
            throttles[2] * 10.0,
            throttles[3] * 10.0,
        ];
        let (_, torque, _) = calculate_net_forces(thrusts, [0.0; 3], [0.0, 0.0, 0.0, 1.0], &config);
        assert!(torque[2] > 0.0);
    }

    #[test]
    fn test_mixer_equal_thrust_zero_torque() {
        let config = dummy_config();
        let (_, torque, _) = calculate_net_forces(
            [1.0, 1.0, 1.0, 1.0],
            [0.0; 3],
            [0.0, 0.0, 0.0, 1.0],
            &config,
        );
        assert_eq!(torque, [0.0, 0.0, 0.0]);
    }
}
