use crate::types::DroneState;
use glam::{DVec3, Quat, Vec3};

/// Internal helper to calculate rates of change.
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

    let q_rate = orientation * Quat::from_vec4(angular_velocity.extend(0.0)) * 0.5;

    (pos_rate, vel_rate, q_rate, ang_vel_rate)
}

/// Advances the drone's physical state forward by one timestep using 4th-order Runge-Kutta integration.
///
/// # Units
/// * `force` - Newtons (N).
/// * `torque` - Newton-meters (Nm).
/// * `mass` - Kilograms (kg).
/// * `inertia_scalar` - Kilogram-square meters (kg*m^2).
/// * `dt` - Seconds (s).
pub fn step_rk4(
    state: &DroneState,
    force: [f32; 3],
    torque: [f32; 3],
    mass: f32,
    inertia_scalar: f32,
    dt: f32,
) -> DroneState {
    let p0 = DVec3::from_array(state.position);
    let v0 = Vec3::from_array(state.velocity);
    let q0 = Quat::from_array(state.orientation);
    let w0 = Vec3::from_array(state.angular_velocity);

    let f = Vec3::from_array(force);
    let t = Vec3::from_array(torque);
    let i = Vec3::splat(inertia_scalar);

    let (dp1, dv1, dq1, dw1) = compute_derivative(v0, q0, w0, f, t, mass, i);

    let (dp2, dv2, dq2, dw2) = compute_derivative(
        v0 + dv1 * dt * 0.5,
        q0 + dq1 * dt * 0.5,
        w0 + dw1 * dt * 0.5,
        f,
        t,
        mass,
        i,
    );

    let (dp3, dv3, dq3, dw3) = compute_derivative(
        v0 + dv2 * dt * 0.5,
        q0 + dq2 * dt * 0.5,
        w0 + dw2 * dt * 0.5,
        f,
        t,
        mass,
        i,
    );

    let (dp4, dv4, dq4, dw4) =
        compute_derivative(v0 + dv3 * dt, q0 + dq3 * dt, w0 + dw3 * dt, f, t, mass, i);

    let p_step = (dp1 + dp2 * 2.0 + dp3 * 2.0 + dp4) * (dt / 6.0);
    let p_final = p0 + p_step.as_dvec3();

    let v_final = v0 + (dv1 + dv2 * 2.0 + dv3 * 2.0 + dv4) * (dt / 6.0);
    let q_final = q0 + (dq1 + dq2 * 2.0 + dq3 * 2.0 + dq4) * (dt / 6.0);
    let w_final = w0 + (dw1 + dw2 * 2.0 + dw3 * 2.0 + dw4) * (dt / 6.0);

    DroneState {
        position: p_final.to_array(),
        velocity: v_final.to_array(),
        orientation: q_final.normalize().to_array(),
        angular_velocity: w_final.to_array(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_c1_free_fall_match() {
        let mut state = DroneState {
            position: [0.0f64, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };

        let mass = 2.0;
        let gravity_force = [0.0, 0.0, -19.62];
        let dt = 0.01;

        for _ in 0..1000 {
            state = step_rk4(&state, gravity_force, [0.0, 0.0, 0.0], mass, 1.0, dt);
        }

        let expected_y = -490.5;
        let actual_y = state.position[2];

        assert!((actual_y - expected_y).abs() < 0.01, "Free-fall mismatch");
    }

    #[test]
    fn test_c2_hover_equilibrium() {
        let start_state = DroneState {
            position: [0.0f64, 0.0, 10.0],
            velocity: [0.0, 0.0, 0.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };

        let net_force = [0.0, 0.0, 0.0];
        let next_state = step_rk4(&start_state, net_force, [0.0, 0.0, 0.0], 2.0, 1.0, 0.1);

        assert_eq!(next_state.position[2], 10.0);
        assert_eq!(next_state.velocity[2], 0.0);
    }
}
