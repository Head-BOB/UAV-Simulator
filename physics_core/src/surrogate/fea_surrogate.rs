//! FEA Structural Surrogate Mapping & Envelope Verification
//! Author: Dev 2 (Dev A) — FEA & Structural Surrogate Owner

use crate::types::SurrogateQueryResult;

/// Formally validated operational envelope boundaries (from Task 8)
pub const G_LOAD_MIN: f32 = 0.5;
pub const G_LOAD_MAX: f32 = 8.0;
pub const VIB_FREQ_MIN: f32 = 100.0;
pub const VIB_FREQ_MAX: f32 = 600.0;
pub const LOAD_ANGLE_MIN: f32 = 0.0;
pub const LOAD_ANGLE_MAX: f32 = 45.0;

/// Ultimate tensile strength of carbon-fiber laminate (Pa)
pub const SIGMA_T_ULT_PA: f32 = 1.50e9;

/// Checks if query conditions are within the validated envelope.
pub fn is_in_validated_envelope(g_load: f32, vibration_freq_hz: f32, load_angle_deg: f32) -> bool {
    (G_LOAD_MIN..=G_LOAD_MAX).contains(&g_load)
        && (VIB_FREQ_MIN..=VIB_FREQ_MAX).contains(&vibration_freq_hz)
        && (LOAD_ANGLE_MIN..=LOAD_ANGLE_MAX).contains(&load_angle_deg)
}

/// Evaluates the structural surrogate model given current flight conditions.
///
/// Inputs (ordered identically to the training/ONNX model):
/// 1. `g_load`: Dynamic acceleration load (Gs)
/// 2. `vibration_freq_hz`: Dominant motor rotational vibration frequency (Hz)
/// 3. `load_angle_deg`: Angle of net load relative to frame normal (degrees)
pub fn evaluate_fea_surrogate(
    g_load: f32,
    vibration_freq_hz: f32,
    load_angle_deg: f32,
) -> SurrogateQueryResult {
    let in_envelope = if is_in_validated_envelope(g_load, vibration_freq_hz, load_angle_deg) {
        1
    } else {
        0
    };

    // Evaluates the response surface matching the trained surrogate
    // W_G = 39.502, W_vib = -0.4086, W_ang = -3.0415, Bias = 273.642 MPa
    let predicted_stress_mpa = (39.502 * g_load)
        + (-0.4086 * vibration_freq_hz)
        + (-3.0415 * load_angle_deg)
        + 273.642;

    let clamped_stress_mpa = predicted_stress_mpa.max(10.0);
    let predicted_stress_pa = clamped_stress_mpa * 1.0e6;

    // Safety Margin = (Allowable Ultimate Strength / Peak Stress) - 1.0
    let safety_margin = (SIGMA_T_ULT_PA / predicted_stress_pa) - 1.0;
    let uncertainty = if in_envelope == 1 { 0.035 } else { 0.850 };

    SurrogateQueryResult {
        predicted_value: safety_margin,
        uncertainty,
        in_validated_envelope: in_envelope,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_envelope_bounds() {
        assert!(is_in_validated_envelope(3.0, 250.0, 15.0));
        assert!(!is_in_validated_envelope(12.0, 250.0, 15.0)); // Over G-limit
        assert!(!is_in_validated_envelope(3.0, 800.0, 15.0)); // Over RPM limit
        assert!(!is_in_validated_envelope(3.0, 250.0, 70.0)); // Over angle limit
    }

    #[test]
    fn test_nominal_evaluation() {
        let res = evaluate_fea_surrogate(2.0, 200.0, 10.0);
        assert_eq!(res.in_validated_envelope, 1);
        assert!(res.predicted_value > 0.0); // Must be safe
        assert!(res.uncertainty < 0.1);
    }
}
