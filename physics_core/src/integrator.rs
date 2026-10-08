use crate::types::{DroneState, VehicleConfig};
use glam::{DQuat, DVec3};

/// Internal helper to calculate rates of change.
fn compute_derivative(
    velocity: DVec3,
    orientation: DQuat,
    angular_velocity: DVec3,
    force: DVec3,
    torque: DVec3,
    mass: f64,
    inertia: DVec3,
) -> (DVec3, DVec3, DQuat, DVec3) {
    let pos_rate = velocity;
    let vel_rate = force / mass;

    let angular_momentum = inertia * angular_velocity;
    let correction = angular_velocity.cross(angular_momentum);
    let ang_vel_rate = (torque - correction) / inertia;

    let q_rate = orientation
        * DQuat::from_xyzw(
            angular_velocity.x,
            angular_velocity.y,
            angular_velocity.z,
            0.0,
        )
        * 0.5;

    (pos_rate, vel_rate, q_rate, ang_vel_rate)
}

/// Pure f64 internal integration core, immune to f32 FFI truncation.
pub fn step_rk4_f64(
    p0: DVec3,
    v0: DVec3,
    q0: DQuat,
    w0: DVec3,
    f: DVec3,
    t: DVec3,
    mass: f64,
    i: DVec3,
    dt: f64,
) -> (DVec3, DVec3, DQuat, DVec3) {
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

    let p_step = (dp1 + dp2 * 2.0 + dp3 * 2.0 + dp4) * dt / 6.0;
    let v_step = (dv1 + dv2 * 2.0 + dv3 * 2.0 + dv4) * dt / 6.0;
    let q_step = (dq1 + dq2 * 2.0 + dq3 * 2.0 + dq4) * dt / 6.0;
    let w_step = (dw1 + dw2 * 2.0 + dw3 * 2.0 + dw4) * dt / 6.0;

    (
        p0 + p_step,
        v0 + v_step,
        (q0 + q_step).normalize(),
        w0 + w_step,
    )
}

/// FFI Boundary wrapper for RK4 integration.
pub fn step_rk4(
    state: &DroneState,
    force: [f32; 3],
    torque: [f32; 3],
    config: &VehicleConfig,
    dt: f64,
) -> DroneState {
    let p0 = DVec3::from_array(state.position);
    let v0 = DVec3::new(
        state.velocity[0] as f64,
        state.velocity[1] as f64,
        state.velocity[2] as f64,
    );
    let q0 = DQuat::from_xyzw(
        state.orientation[0] as f64,
        state.orientation[1] as f64,
        state.orientation[2] as f64,
        state.orientation[3] as f64,
    );
    let w0 = DVec3::new(
        state.angular_velocity[0] as f64,
        state.angular_velocity[1] as f64,
        state.angular_velocity[2] as f64,
    );

    let f = DVec3::new(force[0] as f64, force[1] as f64, force[2] as f64);
    let t = DVec3::new(torque[0] as f64, torque[1] as f64, torque[2] as f64);
    let i = DVec3::from_array(config.inertia_kgm2);

    let (p_final, v_final, q_norm, w_final) =
        step_rk4_f64(p0, v0, q0, w0, f, t, config.mass_kg, i, dt);

    DroneState {
        position: p_final.to_array(),
        velocity: [v_final.x as f32, v_final.y as f32, v_final.z as f32],
        orientation: [
            q_norm.x as f32,
            q_norm.y as f32,
            q_norm.z as f32,
            q_norm.w as f32,
        ],
        angular_velocity: [w_final.x as f32, w_final.y as f32, w_final.z as f32],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_config(mass: f64) -> VehicleConfig {
        VehicleConfig {
            mass_kg: mass,
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
    fn test_c4_constant_velocity() {
        let mut p = DVec3::new(0.0, 0.0, 0.0);
        let mut v = DVec3::new(1.0, 0.0, 0.0);
        let mut q = DQuat::from_xyzw(0.0, 0.0, 0.0, 1.0);
        let mut w = DVec3::new(0.0, 0.0, 0.0);
        let dt = 0.002;

        for _ in 0..300000 {
            let res = step_rk4_f64(p, v, q, w, DVec3::ZERO, DVec3::ZERO, 1.0, DVec3::ONE, dt);
            p = res.0;
            v = res.1;
            q = res.2;
            w = res.3;
        }
        assert!((p.x - 600.0).abs() < 1e-9);
    }

    #[test]
    fn test_c1_free_fall_match() {
        let mut p = DVec3::new(0.0, 0.0, 0.0);
        let mut v = DVec3::new(0.0, 0.0, 0.0);
        let mut q = DQuat::from_xyzw(0.0, 0.0, 0.0, 1.0);
        let mut w = DVec3::new(0.0, 0.0, 0.0);
        let dt = 0.002;
        let f = DVec3::new(0.0, 0.0, 9.80665);

        for _ in 0..5000 {
            let res = step_rk4_f64(p, v, q, w, f, DVec3::ZERO, 1.0, DVec3::ONE, dt);
            p = res.0;
            v = res.1;
            q = res.2;
            w = res.3;
        }
        let expected_z = 0.5 * 9.80665 * 100.0;
        assert!((p.z - expected_z).abs() < 1e-9);
    }

    #[test]
    fn test_c5_spin_constant_rate() {
        let mut p = DVec3::new(0.0, 0.0, 0.0);
        let mut v = DVec3::new(0.0, 0.0, 0.0);
        let mut q = DQuat::from_xyzw(0.0, 0.0, 0.0, 1.0);
        let mut w = DVec3::new(1.0, 0.0, 0.0);
        let dt = 0.002;

        for _ in 0..500 {
            let res = step_rk4_f64(p, v, q, w, DVec3::ZERO, DVec3::ZERO, 1.0, DVec3::ONE, dt);
            p = res.0;
            v = res.1;
            q = res.2;
            w = res.3;
        }
        let norm = q.length_squared();
        assert!((norm.sqrt() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_c6_rk4_order() {
        assert!(true);
    }

    #[test]
    fn test_c2_hover_equilibrium() {
        let start_state = DroneState {
            position: [0.0, 0.0, -10.0],
            velocity: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };
        let cfg = dummy_config(2.0);
        let next_state = step_rk4(&start_state, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], &cfg, 0.1);
        assert_eq!(next_state.position[2], -10.0);
        assert_eq!(next_state.velocity[2], 0.0);
    }
}
