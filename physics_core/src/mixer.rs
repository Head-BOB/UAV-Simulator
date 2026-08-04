use glam::{Vec3, Quat}; // For vector math

// Motor positions relative to center of gravity (X, Y, Z in meters)
// Standard "X" quadcopter layout
const MOTOR_FL_POS: Vec3 = Vec3::new(0.2, 0.2, 0.0);  // Front-Left
const MOTOR_FR_POS: Vec3 = Vec3::new(0.2, -0.2, 0.0); // Front-Right
const MOTOR_BL_POS: Vec3 = Vec3::new(-0.2, 0.2, 0.0); // Back-Left
const MOTOR_BR_POS: Vec3 = Vec3::new(-0.2, -0.2, 0.0); // Back-Right

// Spin directions for yaw reaction (+1.0 or -1.0)
// Diagonally opposite motors spin in the same direction
const MOTOR_FL_SPIN: f32 = 1.0;
const MOTOR_BR_SPIN: f32 = 1.0;
const MOTOR_FR_SPIN: f32 = -1.0;
const MOTOR_BL_SPIN: f32 = -1.0;

// Yaw reaction torque coefficient (Simplified Phase 1 placeholder)
const REACTION_TORQUE_COEFF: f32 = 0.05;

// Gravity constant
const GRAVITY: f32 = 9.80665;


pub fn calculate_net_forces(
    thrusts: [f32; 4],
    drag: [f32; 3],
    orientation: [f32; 4],
    mass: f32
) -> ([f32; 3], [f32; 3], [f32; 3]) { 

    let drag_vec = Vec3::from_array(drag);
    let rot_world = Quat::from_array(orientation);

    let total_thrust = thrusts[0] + thrusts[1] + thrusts[2] + thrusts[3];
    let thrust_body = Vec3::new(0.0, 0.0, total_thrust);

    let thrust_world = rot_world * thrust_body;
    let gravity_world = Vec3::new(0.0, 0.0, -mass * GRAVITY);
    let net_force = thrust_world + drag_vec + gravity_world;

    let torque_fl = MOTOR_FL_POS.cross(Vec3::new(0.0, 0.0, thrusts[0]));
    let torque_fr = MOTOR_FR_POS.cross(Vec3::new(0.0, 0.0, thrusts[1]));
    let torque_bl = MOTOR_BL_POS.cross(Vec3::new(0.0, 0.0, thrusts[2]));
    let torque_br = MOTOR_BR_POS.cross(Vec3::new(0.0, 0.0, thrusts[3]));

    let mut net_torque = torque_fl + torque_fr + torque_bl + torque_br;

    let yaw_reaction = (MOTOR_FL_SPIN * REACTION_TORQUE_COEFF * thrusts[0])
        + (MOTOR_FR_SPIN * REACTION_TORQUE_COEFF * thrusts[1])
        + (MOTOR_BL_SPIN * REACTION_TORQUE_COEFF * thrusts[2])
        + (MOTOR_BR_SPIN * REACTION_TORQUE_COEFF * thrusts[3]);

    net_torque.z += yaw_reaction;


    (net_force.to_array(), net_torque.to_array(), thrust_world.to_array())
}