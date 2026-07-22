use glam::{Vec3, Quat};
use crate::types::DroneState;

/// Internal helper to calculate rates of change
fn compute_derivative(
    velocity: Vec3,
    orientation: Quat,
    angular_velocity: Vec3,
    force: Vec3,
    torque: Vec3,
    mass: f32,
    inertia: Vec3,
) -> (Vec3, Vec3, Quat, Vec3) {
    let pos_rate = velocity;
    let vel_rate = force / mass;

    let angular_momentum = inertia * angular_velocity;
    let correction = angular_velocity.cross(angular_momentum);
    let ang_vel_rate = (torque - correction) / inertia;

    // FIX: Changed order so orientation comes before the 0.5 scalar
    let q_rate = orientation * Quat::from_vec4(angular_velocity.extend(0.0)) * 0.5;

    (pos_rate, vel_rate, q_rate, ang_vel_rate)
}

// FIX: Renamed from integrate_rk4 to step_rk4 to match Lead Architect's lib.rs
// Also changed inertia parameter to f32 to match the Lead Architect's call in lib.rs
pub fn step_rk4(
    state: &DroneState,
    force: [f32; 3],
    torque: [f32; 3],
    mass: f32,
    inertia_scalar: f32, // Lead Architect is passing a single number in lib.rs
    dt: f32,
) -> DroneState {
    let p0 = Vec3::from_array(state.position);
    let v0 = Vec3::from_array(state.velocity);
    let q0 = Quat::from_array(state.orientation);
    let w0 = Vec3::from_array(state.angular_velocity);

    let f = Vec3::from_array(force);
    let t = Vec3::from_array(torque);
    let i = Vec3::splat(inertia_scalar); // Turn the single number into a 3D vector

    // Step k1
    let (dp1, dv1, dq1, dw1) = compute_derivative(v0, q0, w0, f, t, mass, i);

    // Step k2
    let (dp2, dv2, dq2, dw2) = compute_derivative(
        v0 + dv1 * dt * 0.5,
        q0 + dq1 * dt * 0.5,
        w0 + dw1 * dt * 0.5,
        f, t, mass, i
    );

    // Step k3
    let (dp3, dv3, dq3, dw3) = compute_derivative(
        v0 + dv2 * dt * 0.5,
        q0 + dq2 * dt * 0.5,
        w0 + dw2 * dt * 0.5,
        f, t, mass, i
    );

    // Step k4
    let (dp4, dv4, dq4, dw4) = compute_derivative(
        v0 + dv3 * dt,
        q0 + dq3 * dt,
        w0 + dw3 * dt,
        f, t, mass, i
    );

    // Combine results - FIX: Put scalars (dt / 6.0) at the end of multiplications
    let p_final = p0 + (dp1 + dp2 * 2.0 + dp3 * 2.0 + dp4) * (dt / 6.0);
    let v_final = v0 + (dv1 + dv2 * 2.0 + dv3 * 2.0 + dv4) * (dt / 6.0);
    let q_final = q0 + (dq1 + dq2 * 2.0 + dq3 * 2.0 + dq4) * (dt / 6.0);
    let w_final = w0 + (dw1 + dw2 * 2.0 + dw3 * 2.0 + dw4) * (dt / 6.0);

    let q_normalized = q_final.normalize();

    DroneState {
        position: p_final.to_array(),
        velocity: v_final.to_array(),
        orientation: q_normalized.to_array(),
        angular_velocity: w_final.to_array(),
    }
}
/*
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_c1_free_fall_match() {
        let mut state = DroneState {
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };

        let mass = 2.0;
        let gravity_force = [0.0, 0.0, -19.62]; // Force = 2kg * 9.81 m/s^2
        let dt = 0.01;

        // Run for 1000 steps (10 seconds)
        for _ in 0..1000 {
            state = step_rk4(&state, gravity_force, [0.0, 0.0, 0.0], mass, 1.0, dt);
        }

        // Analytical formula: s = 0.5 * a * t^2
        // s = 0.5 * 9.81 * 10^2 = 490.5 meters
        let expected_y = -490.5;
        let actual_y = state.position[2];

        assert!((actual_y - expected_y).abs() < 0.01,
                "Free-fall failed! Expected {}, got {}", expected_y, actual_y);
        println!("Free-fall test passed!");
    }

    #[test]
    fn test_c2_hover_equilibrium() {
        let start_state = DroneState {
            position: [0.0, 0.0, 10.0],
            velocity: [0.0, 0.0, 0.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };

        // Thrust perfectly matches weight

        let net_force = [0.0, 0.0, 0.0];

        let next_state = step_rk4(&start_state, net_force, [0.0, 0.0, 0.0], 2.0, 1.0, 0.1);

        assert_eq!(next_state.position[2], 10.0, "Drone drifted during hover!");
        assert_eq!(next_state.velocity[2], 0.0, "Drone gained velocity during hover!");
        println!("Hover equilibrium test passed!");
    }
}
*/
