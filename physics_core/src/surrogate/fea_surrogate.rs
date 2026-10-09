// Tier: F
use crate::surrogate::{SurrogateHandle, SurrogateLoadError, query_model};
use crate::types::SurrogateQueryResult;
use glam::{DQuat, DVec3};

/// Number of inputs the FEA surrogate takes: 4 arm loads, 3 direction components, 1 temperature.
pub const FEA_INPUT_WIDTH: usize = 8;

/// 0 deg C in kelvin (SI definition of the Celsius scale).
const KELVIN_AT_0_C: f64 = 273.15;

/// Structural surrogate inputs (WP-7 Step D.3).
///
/// TODO(HUMAN-VERIFY): this layout is a stub-level decision, not validated against any CalculiX
/// load case. Model input order: `arm_loads_n[0..4]`, `load_dir_body[0..3]`, `motor_temp_k`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeaInputs {
    /// Thrust carried by each arm, in motor index order. Newtons.
    pub arm_loads_n: [f64; 4],
    /// Unit vector of the net aerodynamic + thrust load in the body frame (X fwd, Y right,
    /// Z down). Exactly (0, 0, 0) when the load is zero.
    pub load_dir_body: [f64; 3],
    /// Highest motor temperature (worst case), kelvin. NaN if any input temperature is not finite.
    pub motor_temp_k: f64,
}

impl FeaInputs {
    /// Packs the inputs in model order as f32 (the model precision; narrowing is intended).
    pub fn to_model_inputs(&self) -> [f32; FEA_INPUT_WIDTH] {
        let l = &self.arm_loads_n;
        let d = &self.load_dir_body;
        [
            l[0] as f32,
            l[1] as f32,
            l[2] as f32,
            l[3] as f32,
            d[0] as f32,
            d[1] as f32,
            d[2] as f32,
            self.motor_temp_k as f32,
        ]
    }
}

/// Highest of four temperatures; NaN if any is not finite (`f64::max` would hide a NaN).
fn max_finite_or_nan(values: [f64; 4]) -> f64 {
    if values.iter().any(|v| !v.is_finite()) {
        return f64::NAN;
    }
    values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
}

/// Builds the FEA surrogate inputs from the step's thrusts, drag, attitude and temperatures.
///
/// * `thrusts_n` - per-motor thrust, N, index order of the vehicle config.
/// * `drag_ned_n` - aerodynamic force, N, NED frame.
/// * `orientation_xyzw` - quaternion (x, y, z, w), body -> NED.
/// * `motor_temp_c` - winding temperatures, deg C.
///
/// The load is thrust (body `(0, 0, -sum T)`) plus drag rotated into the body frame. Gravity is
/// excluded on purpose: in free flight it does not load the frame. The direction is the unit
/// vector of that load, or exactly zero when the load is zero. TODO(HUMAN-VERIFY): direction
/// convention and the choice of the hottest motor.
pub fn fea_inputs(
    thrusts_n: [f64; 4],
    drag_ned_n: [f32; 3],
    orientation_xyzw: [f32; 4],
    motor_temp_c: [f64; 4],
) -> FeaInputs {
    let q = DQuat::from_xyzw(
        f64::from(orientation_xyzw[0]),
        f64::from(orientation_xyzw[1]),
        f64::from(orientation_xyzw[2]),
        f64::from(orientation_xyzw[3]),
    );
    let drag_body = q.inverse()
        * DVec3::new(
            f64::from(drag_ned_n[0]),
            f64::from(drag_ned_n[1]),
            f64::from(drag_ned_n[2]),
        );
    let load = DVec3::new(0.0, 0.0, -thrusts_n.iter().sum::<f64>()) + drag_body;
    let length_n = load.length();
    let dir = if length_n == 0.0 {
        DVec3::ZERO
    } else {
        load / length_n
    };
    FeaInputs {
        arm_loads_n: thrusts_n,
        load_dir_body: dir.to_array(),
        motor_temp_k: max_finite_or_nan(motor_temp_c) + KELVIN_AT_0_C,
    }
}

/// Queries the structural FEA surrogate model to predict safety margin.
///
/// # Units
/// * `inputs` - see `FeaInputs` (N, unit vector, K).
/// * Returns - Structural safety margin (unitless, < 1.0 is failure)
///
/// # Errors
/// * `EngineError` - Forwarded from the underlying generic ONNX query.
/// * `InputCountMismatch` / `NonFiniteInput` - model width is not 8, or an input is not finite.
pub fn query_structural(
    handle: &mut SurrogateHandle,
    inputs: &FeaInputs,
) -> Result<SurrogateQueryResult, SurrogateLoadError> {
    query_model(handle, &inputs.to_model_inputs())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surrogate::check_inputs;

    const IDENTITY: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

    #[test]
    fn test_fea_inputs_hover_like_load_and_hottest_motor() {
        // Thrusts 2+3+4+5 = 14 N, no drag, identity: load (0, 0, -14), direction (0, 0, -1)
        // (thrust acts along body -Z, up). Hottest motor 90.5 C = 90.5 + 273.15 = 363.65 K.
        let f = fea_inputs(
            [2.0, 3.0, 4.0, 5.0],
            [0.0; 3],
            IDENTITY,
            [20.0, 25.0, 90.5, 30.0],
        );
        assert_eq!(f.arm_loads_n, [2.0, 3.0, 4.0, 5.0]);
        assert_eq!(f.load_dir_body, [0.0, 0.0, -1.0]);
        assert!((f.motor_temp_k - 363.65).abs() < 1e-12);
    }

    #[test]
    fn test_fea_inputs_drag_tilts_direction() {
        // Thrusts 4 x 2.5 = 10 N; drag NED (-3, 0, 0); identity: load (-3, 0, -10),
        // |load| = sqrt(9 + 100) = sqrt(109); direction (-3/sqrt109, 0, -10/sqrt109)
        // = (-0.2873478855663454, 0, -0.9578262852211513). tol 1e-14 (a few ulps).
        let f = fea_inputs([2.5; 4], [-3.0, 0.0, 0.0], IDENTITY, [20.0; 4]);
        assert!((f.load_dir_body[0] + 0.2873478855663454).abs() < 1e-14);
        assert_eq!(f.load_dir_body[1], 0.0);
        assert!((f.load_dir_body[2] + 0.9578262852211513).abs() < 1e-14);
    }

    #[test]
    fn test_fea_inputs_drag_is_rotated_into_body_frame() {
        // Yaw +90 deg (q = (0,0,sin45,cos45)): body X = NED +Y. Drag NED (0, -3, 0) is body
        // (-3, 0, 0). Zero thrust: load (-3, 0, 0), direction (-1, 0, 0). tol 1e-6 (f32 q).
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let f = fea_inputs([0.0; 4], [0.0, -3.0, 0.0], [0.0, 0.0, s, s], [20.0; 4]);
        assert!((f.load_dir_body[0] + 1.0).abs() < 1e-6);
        assert!(f.load_dir_body[1].abs() < 1e-6 && f.load_dir_body[2].abs() < 1e-6);
    }

    #[test]
    fn test_fea_inputs_zero_load_has_zero_direction() {
        let f = fea_inputs([0.0; 4], [0.0; 3], IDENTITY, [20.0; 4]);
        assert_eq!(f.load_dir_body, [0.0, 0.0, 0.0]);
        // 20 C = 293.15 K.
        assert!((f.motor_temp_k - 293.15).abs() < 1e-12);
    }

    #[test]
    fn test_fea_inputs_nan_temperature_is_rejected_at_query() {
        // A NaN temperature must not be hidden by max(); the packed inputs fail check_inputs.
        let f = fea_inputs([1.0; 4], [0.0; 3], IDENTITY, [20.0, f64::NAN, 30.0, 25.0]);
        assert!(f.motor_temp_k.is_nan());
        assert!(matches!(
            check_inputs(&f.to_model_inputs(), FEA_INPUT_WIDTH),
            Err(SurrogateLoadError::NonFiniteInput)
        ));
    }

    #[test]
    fn test_fea_inputs_nan_in_first_position_also_rejected() {
        // NaN in position 0 is the case a plain fold over f64::max would silently skip.
        let f = fea_inputs([1.0; 4], [0.0; 3], IDENTITY, [f64::NAN, 20.0, 30.0, 25.0]);
        assert!(f.motor_temp_k.is_nan());
    }

    #[test]
    fn test_fea_to_model_inputs_order() {
        let f = FeaInputs {
            arm_loads_n: [1.0, 2.0, 3.0, 4.0],
            load_dir_body: [5.0, 6.0, 7.0],
            motor_temp_k: 8.0,
        };
        assert_eq!(
            f.to_model_inputs(),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        );
        assert_eq!(f.to_model_inputs().len(), FEA_INPUT_WIDTH);
    }
}
