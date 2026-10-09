// Tier: F
use crate::types::{ControlInputs, VehicleConfig};
use glam::{DQuat, DVec3};

/// Yaw mixing sign per motor, in config index order (FR, FL, RR, RL).
///
/// Equal to the CCW(+1)/CW(-1) layout of `motor_spin`: a positive yaw command speeds up the
/// motors whose reaction torque is +Z (body, nose right) and slows the others.
// TODO(HUMAN-VERIFY: matches real airframe) which physical motor turns which way.
const YAW_MIX_SIGN: [f64; 4] = [1.0, -1.0, -1.0, 1.0];

/// Sign conventions and mixing logic for the Quad-X airframe.
///
/// # Coordinate System (NED world, body X forward, Y right, Z down)
/// Body axes per ADR-001.
///
/// # Motor Layout (config index order, body frame)
/// * `Motor 0` (FR): +X, +Y, spin +1 (CCW seen from above).
/// * `Motor 1` (FL): +X, -Y, spin -1 (CW seen from above).
/// * `Motor 2` (RR): -X, +Y, spin -1 (CW seen from above).
/// * `Motor 3` (RL): -X, -Y, spin +1 (CCW seen from above).
///
/// `motor_spin` = +1 means the reaction torque on the airframe is about body +Z
/// (nose right), which is a propeller turning CCW seen from above.
/// `torque_coeff` is in `N*m` per `N` of thrust, so yaw torque = `sum(spin * torque_coeff * thrust_n)`.
/// The standard quad-X numbering (1 FR, 2 RL, 3 FL, 4 RR) is not the config index order;
/// the config positions are kept and spin is derived from position.
// TODO(HUMAN-VERIFY: matches real airframe) spin per physical motor and the torque_coeff value.
///
/// # Control Sign Convention
/// * `Throttle`: Positive adds total thrust.
/// * `Roll`: Positive rolls right wing down (increases FL, RL; decreases FR, RR).
/// * `Pitch`: Positive pitches nose up (increases FR, FL; decreases RR, RL).
/// * `Yaw`: Positive yaws nose right (increases CCW motors FR, RL; decreases CW motors FL, RR).
///
/// Each axis command is scaled by 0.2 of full throttle.
// TODO(HUMAN-VERIFY): the 0.2 axis authority has no source.
pub fn mix_controls(controls: &ControlInputs) -> [f64; 4] {
    let t = controls.throttle as f64;
    let r = controls.roll as f64 * 0.2;
    let p = controls.pitch as f64 * 0.2;
    let y = controls.yaw as f64 * 0.2;

    [
        (t - r + p + y * YAW_MIX_SIGN[0]).clamp(0.0, 1.0), // FR
        (t + r + p + y * YAW_MIX_SIGN[1]).clamp(0.0, 1.0), // FL
        (t - r - p + y * YAW_MIX_SIGN[2]).clamp(0.0, 1.0), // RR
        (t + r - p + y * YAW_MIX_SIGN[3]).clamp(0.0, 1.0), // RL
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

    // Fixed [_; 4] arrays: the bound is 4 and zip cannot index out of range.
    for ((pos_m, &spin), &thrust_n) in config
        .motor_pos_m
        .iter()
        .zip(config.motor_spin.iter())
        .zip(thrusts.iter())
    {
        let pos = DVec3::from_array(*pos_m);
        net_torque += pos.cross(DVec3::new(0.0, 0.0, -thrust_n));
        yaw_reaction += spin * config.torque_coeff * thrust_n;
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

    /// The shipped config, so the tests exercise the real spin table and kappa = 0.005.
    fn shipped_config() -> VehicleConfig {
        crate::config::load_vehicle_config(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../offline_pipeline/geometry/vehicle_config.json"
        ))
        .expect("shipped config valid")
    }

    fn controls(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> ControlInputs {
        ControlInputs {
            throttle,
            roll,
            pitch,
            yaw,
        }
    }

    /// Thrust proxy for hand calculations: 10 N per unit throttle.
    fn torque_for(inputs: &ControlInputs) -> ([f64; 4], [f64; 3]) {
        let throttles = mix_controls(inputs);
        let thrusts = throttles.map(|t| t * 10.0);
        let (_, torque, _) =
            calculate_net_forces(thrusts, [0.0; 3], [0.0, 0.0, 0.0, 1.0], &shipped_config());
        (throttles, torque)
    }

    #[test]
    fn test_mixer_roll_positive() {
        let (_, torque) = torque_for(&controls(0.5, 0.5, 0.0, 0.0));
        assert!(torque[0] > 0.0);
    }

    #[test]
    fn test_mixer_pitch_positive() {
        let (_, torque) = torque_for(&controls(0.5, 0.0, 0.5, 0.0));
        assert!(torque[1] > 0.0);
    }

    #[test]
    fn test_mixer_yaw_positive() {
        let (_, torque) = torque_for(&controls(0.5, 0.0, 0.0, 0.5));
        assert!(torque[2] > 0.0);
    }

    #[test]
    fn test_mixer_equal_thrust_zero_torque() {
        let config = shipped_config();
        let (_, torque, _) = calculate_net_forces(
            [1.0, 1.0, 1.0, 1.0],
            [0.0; 3],
            [0.0, 0.0, 0.0, 1.0],
            &config,
        );
        assert_eq!(torque, [0.0, 0.0, 0.0]);
    }

    // Hand calc, throttle 0.5, yaw +0.5: y = 0.5 * 0.2 = 0.1, tau = [0.6, 0.4, 0.4, 0.6]
    // (FR +y, FL -y, RR -y, RL +y), T = 10 tau = [6, 4, 4, 6] N.
    // Roll: -sum(py T) = -0.2 (6 - 4 + 4 - 6) = 0. Pitch: sum(px T) = 0.2 (6 + 4 - 4 - 6) = 0.
    // Yaw: 0.005 * (+1 * 6 - 1 * 4 - 1 * 4 + 1 * 6) = 0.005 * 4 = +0.02 N*m.
    #[test]
    fn test_mixer_yaw_positive_hand_calc() {
        let (tau, torque) = torque_for(&controls(0.5, 0.0, 0.0, 0.5));
        for (got, want) in tau.iter().zip([0.6, 0.4, 0.4, 0.6]) {
            assert!((got - want).abs() < 1e-15);
        }
        assert!(torque[0].abs() < 1e-12);
        assert!(torque[1].abs() < 1e-12);
        assert!((torque[2] - 0.02).abs() < 1e-12);
    }

    // Hand calc, yaw -0.5: tau = [0.4, 0.6, 0.6, 0.4], T = [4, 6, 6, 4] N,
    // yaw = 0.005 * (4 - 6 - 6 + 4) = -0.02 N*m.
    #[test]
    fn test_mixer_yaw_negative_hand_calc() {
        let (tau, torque) = torque_for(&controls(0.5, 0.0, 0.0, -0.5));
        for (got, want) in tau.iter().zip([0.4, 0.6, 0.6, 0.4]) {
            assert!((got - want).abs() < 1e-15);
        }
        assert!((torque[2] + 0.02).abs() < 1e-12);
    }

    // Hand calc, throttle 1.0, yaw +0.5: raw [1.1, 0.9, 0.9, 1.1] clamps to [1, 0.9, 0.9, 1],
    // T = [10, 9, 9, 10] N, yaw = 0.005 * (10 - 9 - 9 + 10) = +0.01 N*m (sign survives saturation).
    #[test]
    fn test_mixer_yaw_saturated_keeps_sign() {
        let (tau, torque) = torque_for(&controls(1.0, 0.0, 0.0, 0.5));
        for (got, want) in tau.iter().zip([1.0, 0.9, 0.9, 1.0]) {
            assert!((got - want).abs() < 1e-15);
        }
        assert!((torque[2] - 0.01).abs() < 1e-12);
    }

    // The yaw mix signs equal the shipped spin table, and the shipped positions are
    // x = [+, +, -, -], y = [+, -, +, -] (FR, FL, RR, RL).
    #[test]
    fn test_mixer_yaw_table_matches_shipped_config() {
        let config = shipped_config();
        assert_eq!(YAW_MIX_SIGN, config.motor_spin);
        let x_sign = config.motor_pos_m.map(|p| p[0].signum());
        let y_sign = config.motor_pos_m.map(|p| p[1].signum());
        assert_eq!(x_sign, [1.0, 1.0, -1.0, -1.0]);
        assert_eq!(y_sign, [1.0, -1.0, 1.0, -1.0]);
    }

    // Hand calc: authority 0.2 per axis, so a full-scale axis command moves a motor by 0.2.
    // throttle 0.95, pitch -1: [0.95-0.2, 0.95-0.2, 0.95+0.2, 0.95+0.2] = [0.75, 0.75, 1.15, 1.15] -> clamp [0.75, 0.75, 1, 1]
    // throttle 0.95, roll -1:  [0.95+0.2, 0.95-0.2, 0.95+0.2, 0.95-0.2] -> [1, 0.75, 1, 0.75]
    // throttle 0.95, roll +1:  [0.75, 1.15, 0.75, 1.15] -> [0.75, 1, 0.75, 1]
    // throttle 0.05, pitch +1: [0.25, 0.25, -0.15, -0.15] -> [0.25, 0.25, 0, 0]
    // throttle 0.05, pitch -1: [-0.15, -0.15, 0.25, 0.25] -> [0, 0, 0.25, 0.25]
    // Tolerance 1e-6: throttle 0.95 and 0.05 are f32 inputs (error about 6e-8).
    #[test]
    fn test_mixer_clamps_every_motor_to_unit_range() {
        let cases = [
            (controls(0.95, 0.0, -1.0, 0.0), [0.75, 0.75, 1.0, 1.0]),
            (controls(0.95, -1.0, 0.0, 0.0), [1.0, 0.75, 1.0, 0.75]),
            (controls(0.95, 1.0, 0.0, 0.0), [0.75, 1.0, 0.75, 1.0]),
            (controls(0.05, 0.0, 1.0, 0.0), [0.25, 0.25, 0.0, 0.0]),
            (controls(0.05, 0.0, -1.0, 0.0), [0.0, 0.0, 0.25, 0.25]),
        ];
        for (input, want) in cases {
            let got = mix_controls(&input);
            for (g, w) in got.iter().zip(want) {
                assert!((g - w).abs() < 1e-6, "got {got:?} want {want:?}");
            }
        }
    }
}
