use glam::{Quat, Vec3};

const MOTOR_FL_POS: Vec3 = Vec3::new(0.2, 0.2, 0.0);
const MOTOR_FR_POS: Vec3 = Vec3::new(0.2, -0.2, 0.0);
const MOTOR_BL_POS: Vec3 = Vec3::new(-0.2, 0.2, 0.0);
const MOTOR_BR_POS: Vec3 = Vec3::new(-0.2, -0.2, 0.0);

const MOTOR_FL_SPIN: f32 = 1.0;
const MOTOR_BR_SPIN: f32 = 1.0;
const MOTOR_FR_SPIN: f32 = -1.0;
const MOTOR_BL_SPIN: f32 = -1.0;

const REACTION_TORQUE_COEFF: f32 = 0.05;
const GRAVITY: f32 = 9.80665;

/// Computes the net force and torque acting on the drone.
///
/// Applies motor thrusts and reaction torques based on the standard X-quadcopter layout.
/// Gravity is applied in the +Z (Down) direction, and thrust is applied in the -Z (Up) direction
/// within the body frame, conforming to the NED coordinate standard (ADR-001).
///
/// # Units
/// * `thrusts` - Newtons (N) per motor.
/// * `drag` - Newtons (N) in world frame.
/// * `mass` - Kilograms (kg).
/// * Returns a tuple of (net_force, net_torque, net_thrust) in Newtons and Newton-meters.
pub fn calculate_net_forces(
    thrusts: [f32; 4],
    drag: [f32; 3],
    orientation: [f32; 4],
    mass: f32,
) -> ([f32; 3], [f32; 3], [f32; 3]) {
    let drag_vec = Vec3::from_array(drag);
    let rot_world = Quat::from_array(orientation);

    let total_thrust = thrusts[0] + thrusts[1] + thrusts[2] + thrusts[3];
    let thrust_body = Vec3::new(0.0, 0.0, -total_thrust);

    let thrust_world = rot_world * thrust_body;
    let gravity_world = Vec3::new(0.0, 0.0, mass * GRAVITY);
    let net_force = thrust_world + drag_vec + gravity_world;

    let torque_fl = MOTOR_FL_POS.cross(Vec3::new(0.0, 0.0, -thrusts[0]));
    let torque_fr = MOTOR_FR_POS.cross(Vec3::new(0.0, 0.0, -thrusts[1]));
    let torque_bl = MOTOR_BL_POS.cross(Vec3::new(0.0, 0.0, -thrusts[2]));
    let torque_br = MOTOR_BR_POS.cross(Vec3::new(0.0, 0.0, -thrusts[3]));

    let mut net_torque = torque_fl + torque_fr + torque_bl + torque_br;

    let yaw_reaction = (MOTOR_FL_SPIN * REACTION_TORQUE_COEFF * thrusts[0])
        + (MOTOR_FR_SPIN * REACTION_TORQUE_COEFF * thrusts[1])
        + (MOTOR_BL_SPIN * REACTION_TORQUE_COEFF * thrusts[2])
        + (MOTOR_BR_SPIN * REACTION_TORQUE_COEFF * thrusts[3]);

    net_torque.z += yaw_reaction;

    (
        net_force.to_array(),
        net_torque.to_array(),
        thrust_world.to_array(),
    )
}