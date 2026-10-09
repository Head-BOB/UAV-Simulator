// Tier: F
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

/// Rigid-body kinematic state, NED world frame, SI units, all f64.
///
/// `orientation` is a unit quaternion, component order (x, y, z, w).
/// `angular_velocity_radps` is expressed in the body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyState {
    pub position_m: DVec3,
    pub velocity_mps: DVec3,
    pub orientation: DQuat,
    pub angular_velocity_radps: DVec3,
}

/// Net external loads, held constant over one step.
///
/// `force_n` is in the world (NED) frame; `torque_nm` is in the body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyLoads {
    pub force_n: DVec3,
    pub torque_nm: DVec3,
}

/// Mass and principal (diagonal) inertia, body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassProperties {
    pub mass_kg: f64,
    pub inertia_kgm2: DVec3,
}

/// Pure f64 internal integration core, immune to f32 FFI truncation.
///
/// Classical RK4 over one step of `dt_s` seconds (the project step is 0.002 s).
/// Quaternion component order is (x, y, z, w); the result is re-normalized.
pub fn step_rk4_f64(
    s: &RigidBodyState,
    loads: &BodyLoads,
    mp: &MassProperties,
    dt_s: f64,
) -> RigidBodyState {
    let (p0, v0, q0, w0) = (
        s.position_m,
        s.velocity_mps,
        s.orientation,
        s.angular_velocity_radps,
    );
    let (f, t) = (loads.force_n, loads.torque_nm);
    let (mass, i, dt) = (mp.mass_kg, mp.inertia_kgm2, dt_s);

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

    RigidBodyState {
        position_m: p0 + p_step,
        velocity_mps: v0 + v_step,
        orientation: (q0 + q_step).normalize(),
        angular_velocity_radps: w0 + w_step,
    }
}

/// FFI-boundary wrapper for RK4 integration. Quaternion order (x, y, z, w).
///
/// `force_n` and `torque_nm` are f64 (N, N·m; force NED world frame, torque body frame)
/// so loads reach `step_rk4_f64` without f32 truncation (ADR-001 D2).
pub fn step_rk4(
    state: &DroneState,
    force_n: [f64; 3],
    torque_nm: [f64; 3],
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

    let f = DVec3::from_array(force_n);
    let t = DVec3::from_array(torque_nm);
    let i = DVec3::from_array(config.inertia_kgm2);

    let state = RigidBodyState {
        position_m: p0,
        velocity_mps: v0,
        orientation: q0,
        angular_velocity_radps: w0,
    };
    let loads = BodyLoads {
        force_n: f,
        torque_nm: t,
    };
    let mp = MassProperties {
        mass_kg: config.mass_kg,
        inertia_kgm2: i,
    };
    let next = step_rk4_f64(&state, &loads, &mp, dt);
    let (p_final, v_final, q_norm, w_final) = (
        next.position_m,
        next.velocity_mps,
        next.orientation,
        next.angular_velocity_radps,
    );

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

    fn unit_mass() -> MassProperties {
        MassProperties {
            mass_kg: 1.0,
            inertia_kgm2: DVec3::ONE,
        }
    }

    fn rest_state(velocity_mps: DVec3, angular_velocity_radps: DVec3) -> RigidBodyState {
        RigidBodyState {
            position_m: DVec3::ZERO,
            velocity_mps,
            orientation: DQuat::from_xyzw(0.0, 0.0, 0.0, 1.0),
            angular_velocity_radps,
        }
    }

    #[test]
    fn test_c4_constant_velocity() {
        let mut s = rest_state(DVec3::new(1.0, 0.0, 0.0), DVec3::ZERO);
        let loads = BodyLoads {
            force_n: DVec3::ZERO,
            torque_nm: DVec3::ZERO,
        };
        let dt = 0.002;

        for _ in 0..300000 {
            s = step_rk4_f64(&s, &loads, &unit_mass(), dt);
        }
        assert!((s.position_m.x - 600.0).abs() < 1e-9);
    }

    #[test]
    fn test_c1_free_fall_match() {
        let mut s = rest_state(DVec3::ZERO, DVec3::ZERO);
        let loads = BodyLoads {
            force_n: DVec3::new(0.0, 0.0, 9.80665),
            torque_nm: DVec3::ZERO,
        };
        let dt = 0.002;

        for _ in 0..5000 {
            s = step_rk4_f64(&s, &loads, &unit_mass(), dt);
        }
        let expected_z = 0.5 * 9.80665 * 100.0;
        assert!((s.position_m.z - expected_z).abs() < 1e-9);
    }

    #[test]
    fn test_c5_spin_constant_rate() {
        let mut s = rest_state(DVec3::ZERO, DVec3::new(1.0, 0.0, 0.0));
        let loads = BodyLoads {
            force_n: DVec3::ZERO,
            torque_nm: DVec3::ZERO,
        };
        let dt = 0.002;

        for _ in 0..500 {
            s = step_rk4_f64(&s, &loads, &unit_mass(), dt);
        }
        // Hand calculation (WP-1 rule: angle = rate x time, norm = 1 within 1e-12).
        // T = 500 * 0.002 s = 1.0 s, rate = 1 rad/s, so the exact angle is 1.0 rad
        // and the exact half-angle is 0.5 rad (< pi, so atan2 needs no unwrapping).
        // Norm: normalize() runs every step, so the error does not accumulate and
        // stays within a few ulp (eps = 2.2e-16); 1e-12 is the WP-1 rule
        // (was 1e-6).
        // Angle: truncation = 2 * N * (theta - phi(theta)) with theta = 0.001 is
        // 2 * 500 * theta^5 / 120 = 8.3e-15 rad. Roundoff is at most ~4 eps of
        // angle per step x 500 steps = 4.4e-13 rad if errors add linearly. The sum
        // is 4.5e-13 rad, so 1e-12 rad leaves about 2x margin.
        assert!((s.orientation.length() - 1.0).abs() < 1e-12);
        let angle_rad = 2.0 * s.orientation.x.atan2(s.orientation.w);
        assert!((angle_rad - 1.0).abs() < 1e-12);
    }

    /// Analytic reference: half-angle advanced by ONE classical RK4 step of
    /// q' = 0.5 * q * (w, 0, 0, 0) after normalize(), theta = rate * dt / 2.
    ///
    /// q stays in span{1, i}, so it behaves like the complex number e^(i*alpha) and
    /// one RK4 step multiplies it by P = 1 + i*th - th^2/2 - i*th^3/6 + th^4/24.
    /// normalize() removes |P|, so the phase advance is
    /// atan2(th - th^3/6, 1 - th^2/2 + th^4/24). It does not call the code under test.
    /// Series: th - advance = th^5/120 - th^7/336 + O(th^9) (4th-order method).
    fn rk4_half_angle_per_step(theta: f64) -> f64 {
        (theta - theta.powi(3) / 6.0).atan2(1.0 - theta.powi(2) / 2.0 + theta.powi(4) / 24.0)
    }

    /// Spin about body X at `rate_radps` for `steps` steps; returns the final
    /// half-angle lag behind the exact value rate * T / 2, in rad.
    fn spin_half_angle_lag_rad(rate_radps: f64, dt_s: f64, steps: u32) -> f64 {
        let mut s = rest_state(DVec3::ZERO, DVec3::new(rate_radps, 0.0, 0.0));
        let loads = BodyLoads {
            force_n: DVec3::ZERO,
            torque_nm: DVec3::ZERO,
        };
        for _ in 0..steps {
            s = step_rk4_f64(&s, &loads, &unit_mass(), dt_s);
        }
        let exact_half_angle_rad = rate_radps * dt_s * f64::from(steps) / 2.0;
        exact_half_angle_rad - s.orientation.x.atan2(s.orientation.w)
    }

    #[test]
    fn test_c6_rk4_order() {
        // Problem: constant spin about X, zero loads, exact q = (sin(wt/2),0,0,cos(wt/2)).
        // rate = 50 rad/s, T = 0.1 s. A rate of 1 rad/s would give theta = 0.001 and a
        // truncation error (8e-18/step) below f64 roundoff, so the ratio would be noise.
        //   Run A: dt = 0.002 s, N = 50,  theta = 0.05
        //   Run B: dt = 0.001 s, N = 100, theta = 0.025
        // Final half-angle 2.5 rad < pi, so atan2 needs no unwrapping.
        // Hand calculation (see rk4_half_angle_per_step):
        //   E = N * (theta - phi(theta)),  E ~ T * w^5 * dt^4 / 3840
        //   E_A = 50  * (0.05  - phi(0.05))  = 1.3009209e-7 rad
        //   E_B = 100 * (0.025 - phi(0.025)) = 8.1362042e-9 rad
        //   E_A / E_B = 16 * (1 - (5/14) * 0.05^2) / (1 - (5/14) * 0.025^2) = 15.98928
        //
        // Tolerance 1: |measured - E| <= 1e-13 rad. Only roundoff may differ:
        //   ~4 eps (8.9e-16) per step x N <= 100 gives 8.9e-14 if errors add linearly,
        //   so 1e-13 is just above that worst case (real errors random-walk and are
        //   far smaller). It is about 1e-5 of E_B, so it still pins the method to
        //   exactly classical RK4.
        // Tolerance 2: |E_A / E_B - 16| <= 0.05. The truncation bias is 16 - 15.98928
        //   = 0.0107; roundoff moves the ratio by at most 16 * 1e-13 / 8.1e-9 = 2e-4.
        //   Worst case 0.011, so 0.05 gives ~4.5x margin. A 3rd-order method gives
        //   ratio 8 and a 5th-order method 32, so both fail.
        let rate_radps = 50.0;
        let lag_a = spin_half_angle_lag_rad(rate_radps, 0.002, 50);
        let lag_b = spin_half_angle_lag_rad(rate_radps, 0.001, 100);

        let theta_a = rate_radps * 0.002 / 2.0;
        let theta_b = rate_radps * 0.001 / 2.0;
        let expected_a = 50.0 * (theta_a - rk4_half_angle_per_step(theta_a));
        let expected_b = 100.0 * (theta_b - rk4_half_angle_per_step(theta_b));

        assert!((lag_a - expected_a).abs() <= 1e-13);
        assert!((lag_b - expected_b).abs() <= 1e-13);
        assert!((lag_a / lag_b - 16.0).abs() <= 0.05);
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

    #[test]
    fn test_step_rk4_wrapper_passes_f64_loads() {
        // Hand calc: F = 1.5 * 9.80665 N, m = 1.5 kg, a = 9.80665 m/s^2 (constant),
        // RK4 is exact for constant acceleration: z = 0.5 * a * dt^2
        //   = 0.5 * 9.80665 * 0.002^2 = 1.96133e-5 m.
        // An f32-narrowed force gives 1.961330032e-5 m (error 3.2e-13 m), far above 1e-18.
        let mass_kg = 1.5;
        let force_n = [0.0, 0.0, mass_kg * 9.80665];
        let dt_s = 0.002;
        let start = DroneState {
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0, 0.0, 0.0],
        };
        let cfg = dummy_config(mass_kg);
        let wrapped = step_rk4(&start, force_n, [0.0; 3], &cfg, dt_s);

        let direct = step_rk4_f64(
            &rest_state(DVec3::ZERO, DVec3::ZERO),
            &BodyLoads {
                force_n: DVec3::from_array(force_n),
                torque_nm: DVec3::ZERO,
            },
            &MassProperties {
                mass_kg,
                inertia_kgm2: DVec3::ONE,
            },
            dt_s,
        );
        assert_eq!(wrapped.position[2].to_bits(), direct.position_m.z.to_bits());
        assert!((wrapped.position[2] - 1.96133e-5).abs() <= 1e-18);
    }
}
