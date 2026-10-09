// Tier: F
use crate::surrogate::{SurrogateHandle, SurrogateLoadError, query_model};
use crate::types::{DroneState, SurrogateQueryResult};
use glam::{DQuat, DVec3};

/// Number of inputs the aero surrogate takes: airspeed, angle of attack, sideslip.
pub const AERO_INPUT_WIDTH: usize = 3;

fn quat_from_xyzw(orientation_xyzw: [f32; 4]) -> DQuat {
    DQuat::from_xyzw(
        f64::from(orientation_xyzw[0]),
        f64::from(orientation_xyzw[1]),
        f64::from(orientation_xyzw[2]),
        f64::from(orientation_xyzw[3]),
    )
}

/// Builds the aero surrogate inputs `[airspeed_mps, alpha_rad, beta_rad]` (WP-6.8).
///
/// Quaternion order is (x, y, z, w); it rotates body -> NED. Frames: NED world, body
/// X forward / Y right / Z down (ADR-001). `v_body = q^-1 * v_ned`, with (u, v, w) its components.
///
/// * airspeed `|v_body|` in m/s. Still air is assumed: air velocity is zero, so ground
///   velocity is the velocity relative to the air. TODO(HUMAN-VERIFY): wind.
/// * `alpha = atan2(w, u)` in rad: positive when the air comes from below (body +Z).
/// * `beta = asin(v / |V|)` in rad, computed as `atan2(v, hypot(u, w))`, which is equal for
///   `|V| > 0` and cannot produce NaN from rounding `v / |V|` above 1.
/// * At `|V| == 0` all three are exactly 0 (alpha and beta are undefined there; `atan2(0, 0)`
///   is not used). TODO(HUMAN-VERIFY): convention for undefined angles.
///
/// Non-finite input gives non-finite output, which `check_inputs` rejects at query time.
pub fn aero_inputs(
    velocity_ned_mps: [f32; 3],
    orientation_xyzw: [f32; 4],
) -> [f64; AERO_INPUT_WIDTH] {
    let v_ned = DVec3::new(
        f64::from(velocity_ned_mps[0]),
        f64::from(velocity_ned_mps[1]),
        f64::from(velocity_ned_mps[2]),
    );
    let v_body = quat_from_xyzw(orientation_xyzw).inverse() * v_ned;
    inputs_from_body_velocity(v_body)
}

/// `[airspeed, alpha, beta]` from the body-frame velocity (see `aero_inputs`).
fn inputs_from_body_velocity(v_body: DVec3) -> [f64; AERO_INPUT_WIDTH] {
    let airspeed_mps = v_body.length();
    if airspeed_mps == 0.0 {
        return [0.0, 0.0, 0.0];
    }
    let alpha_rad = v_body.z.atan2(v_body.x);
    let beta_rad = v_body.y.atan2(v_body.x.hypot(v_body.z));
    [airspeed_mps, alpha_rad, beta_rad]
}

/// Rotates a body-frame vector into NED with the (x, y, z, w) body -> NED quaternion.
///
/// Used because the aero surrogate returns its force in the BODY frame (WP-6.8 decision,
/// TODO(HUMAN-VERIFY)), while the integrator takes NED forces.
pub fn body_to_ned(f_body: [f32; 3], orientation_xyzw: [f32; 4]) -> [f32; 3] {
    let f = DVec3::new(
        f64::from(f_body[0]),
        f64::from(f_body[1]),
        f64::from(f_body[2]),
    );
    let r = quat_from_xyzw(orientation_xyzw) * f;
    // Narrowing to the f32 FFI/model precision is intended here.
    [r.x as f32, r.y as f32, r.z as f32]
}

/// Queries the aerodynamic surrogate model to predict drag force.
///
/// # Units
/// * Inputs - `[airspeed_mps, alpha_rad, beta_rad]` from `aero_inputs` (see there for frames).
/// * Returns - 3D force vector in Newtons in the BODY frame (X forward, Y right, Z down).
///   TODO(HUMAN-VERIFY): frame decision; callers rotate with `body_to_ned`.
///
/// # Errors
/// * `EngineError` - Forwarded from the underlying generic ONNX query.
/// * `InputCountMismatch` / `NonFiniteInput` - model width is not 3, or an input is not finite.
pub fn query_aero(
    handle: &mut SurrogateHandle,
    state: &DroneState,
) -> Result<SurrogateQueryResult, SurrogateLoadError> {
    let a = aero_inputs(state.velocity, state.orientation);
    // Narrowing to the f32 model input is intended; the model is f32.
    let inputs = [a[0] as f32, a[1] as f32, a[2] as f32];
    query_model(handle, &inputs)
}

#[cfg(test)]
mod tests {
    use super::*;

    const IDENTITY: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

    fn assert_close(got: [f64; 3], want: [f64; 3], tol: f64) {
        for (g, w) in got.iter().zip(want.iter()) {
            assert!((g - w).abs() <= tol, "got {got:?} want {want:?}");
        }
    }

    #[test]
    fn test_aero_inputs_alpha_3_4_5_triangle() {
        // Identity: v_body = (3, 0, 4). V = 5; alpha = atan2(4, 3) = 0.9272952180016122
        // (= asin 0.8); beta = atan2(0, 5) = 0. Exact up to f64 rounding: tol 1e-15.
        assert_close(
            aero_inputs([3.0, 0.0, 4.0], IDENTITY),
            [5.0, 0.9272952180016122, 0.0],
            1e-15,
        );
    }

    #[test]
    fn test_aero_inputs_beta_3_4_5_triangle() {
        // Identity: v_body = (4, 3, 0). V = 5; alpha = atan2(0, 4) = 0;
        // beta = asin(3/5) = 0.6435011087932844.
        assert_close(
            aero_inputs([4.0, 3.0, 0.0], IDENTITY),
            [5.0, 0.0, 0.6435011087932844],
            1e-15,
        );
    }

    #[test]
    fn test_aero_inputs_pure_sideslip_is_half_pi() {
        // Identity: v_body = (0, 5, 0). V = 5; alpha = atan2(0, 0) = 0; beta = atan2(5, 0) = pi/2.
        assert_close(
            aero_inputs([0.0, 5.0, 0.0], IDENTITY),
            [5.0, 0.0, std::f64::consts::FRAC_PI_2],
            1e-15,
        );
    }

    #[test]
    fn test_aero_inputs_yaw_90_aligns_body_x_with_east() {
        // q = yaw +90 deg about Z (down) = (0, 0, sin45, cos45). Body X points east (NED +Y),
        // so v_ned = (0, 10, 0) is v_body = (10, 0, 0): V 10, alpha 0, beta 0.
        // f32 quaternion components limit accuracy to ~1e-7 relative: tol 1e-6.
        let s = std::f32::consts::FRAC_1_SQRT_2;
        assert_close(
            aero_inputs([0.0, 10.0, 0.0], [0.0, 0.0, s, s]),
            [10.0, 0.0, 0.0],
            1e-6,
        );
    }

    #[test]
    fn test_aero_inputs_pitch_30_gives_alpha_30() {
        // q = pitch +30 deg about Y = (0, sin15, 0, cos15). Flying north at 10 m/s with the nose
        // 30 deg up: v_body = Ry(-30 deg) * (10, 0, 0) = (10 cos30, 0, 10 sin30) = (8.660254, 0, 5).
        // V = 10; alpha = atan2(5, 8.660254) = 30 deg = 0.5235987755982988; beta 0.
        // Positive alpha (air from below) for nose up above the flight path. tol 1e-6 (f32 q).
        let half = 15.0_f32.to_radians();
        let q = [0.0, half.sin(), 0.0, half.cos()];
        assert_close(
            aero_inputs([10.0, 0.0, 0.0], q),
            [10.0, 0.5235987755982988, 0.0],
            1e-6,
        );
    }

    #[test]
    fn test_aero_inputs_zero_velocity_is_exactly_zero() {
        // |V| = 0: atan2(-0.0, -0.0) would give -pi; the explicit branch must return 0.
        assert_eq!(aero_inputs([0.0, 0.0, 0.0], IDENTITY), [0.0, 0.0, 0.0]);
        assert_eq!(
            aero_inputs([-0.0, -0.0, -0.0], [0.0, 0.0, 1.0, 0.0]),
            [0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn test_inputs_from_body_velocity_negative_zero_is_exactly_zero() {
        // atan2(-0.0, -0.0) = -pi in IEEE 754; the |V| == 0 branch must still return +0.
        let r = inputs_from_body_velocity(DVec3::new(-0.0, -0.0, -0.0));
        assert_eq!(r, [0.0, 0.0, 0.0]);
        assert!(r.iter().all(|v| v.is_sign_positive()));
    }

    #[test]
    fn test_inputs_from_body_velocity_beta_uses_total_airspeed() {
        // v_body = (3, 4, 12): V = sqrt(9 + 16 + 144) = 13. alpha = atan2(12, 3) =
        // 1.3258176636680326; beta = asin(4/13) = 0.312766721941545 (Python math.asin(4/13); u and w both non-zero, so
        // atan2(v, u) alone would give atan(4/3) = 0.9273 instead). tol 1e-15.
        let r = inputs_from_body_velocity(DVec3::new(3.0, 4.0, 12.0));
        assert_close(
            r,
            [13.0, 1.3258176636680326, (4.0_f64 / 13.0).asin()],
            1e-15,
        );
        assert!((r[2] - 0.312766721941545).abs() < 1e-14);
    }

    #[test]
    fn test_aero_inputs_nan_velocity_is_not_finite() {
        let a = aero_inputs([f32::NAN, 0.0, 0.0], IDENTITY);
        assert!(a.iter().any(|v| !v.is_finite()));
    }

    #[test]
    fn test_body_to_ned_yaw_90() {
        // Yaw +90 deg about Z: body X (1,0,0) maps to NED +Y (0,1,0). tol 1e-6 (f32 q).
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let r = body_to_ned([1.0, 0.0, 0.0], [0.0, 0.0, s, s]);
        assert!(
            r[0].abs() < 1e-6 && (r[1] - 1.0).abs() < 1e-6 && r[2].abs() < 1e-6,
            "{r:?}"
        );
    }

    #[test]
    fn test_aero_input_width_matches_inputs() {
        assert_eq!(
            AERO_INPUT_WIDTH,
            aero_inputs([1.0, 2.0, 3.0], IDENTITY).len()
        );
    }
}
