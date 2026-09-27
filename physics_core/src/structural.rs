//! Phase 2 Live Structural Safety Subsystem
//! Author: Dev 2 (Dev A) — FEA & Structural Surrogate Owner

use crate::surrogate::fea_surrogate::evaluate_fea_surrogate;
use crate::types::SurrogateQueryResult;

/// Nominal Earth gravity acceleration magnitude in m/s^2
const GRAVITY_ACCEL: f32 = 9.80665;

/// Evaluates drone structural integrity and safety margin during a physics tick.
///
/// Inputs:
/// - `accel_xyz`: Linear acceleration vector in world space [ax, ay, az] (m/s^2)
/// - `motor_rpms`: Motor speeds for each of the 4 motors [m1, m2, m3, m4] (RPM)
///
/// Returns:
/// `SurrogateQueryResult` containing the predicted safety margin, uncertainty,
/// and envelope-validity flag.
pub fn update_structural_safety(
    accel_xyz: [f32; 3],
    motor_rpms: [f32; 4],
) -> SurrogateQueryResult {
    // 1. Compute effective G-load including gravity reaction
    let az_effective = accel_xyz[2] + GRAVITY_ACCEL;
    let accel_mag = (accel_xyz[0].powi(2) + accel_xyz[1].powi(2) + az_effective.powi(2)).sqrt();
    let g_load = (accel_mag / GRAVITY_ACCEL).max(0.1);

    // 2. Compute dominant motor vibration frequency (average RPM / 60)
    let avg_rpm = (motor_rpms[0] + motor_rpms[1] + motor_rpms[2] + motor_rpms[3]) * 0.25;
    let vibration_freq_hz = (avg_rpm / 60.0).max(0.0);

    // 3. Compute load angle relative to the frame normal
    let horizontal_accel = (accel_xyz[0].powi(2) + accel_xyz[1].powi(2)).sqrt();
    let load_angle_deg = if accel_mag > 1e-3 {
        (horizontal_accel / accel_mag).asin().to_degrees()
    } else {
        0.0
    };

    // 4. Query the FEA surrogate model
    evaluate_fea_surrogate(g_load, vibration_freq_hz, load_angle_deg)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hover_structural_safety() {
        let accel_hover = [0.0, 0.0, 0.0];
        let rpms_hover = [12000.0, 12000.0, 12000.0, 12000.0]; // 200 Hz
        let result = update_structural_safety(accel_hover, rpms_hover);

        assert_eq!(result.in_validated_envelope, 1);
        assert!(result.predicted_value > 2.0); // Healthy positive margin in hover
    }

    #[test]
    fn test_excessive_acceleration_flags_envelope() {
        let extreme_accel = [0.0, 0.0, 100.0]; // ~11G maneuver
        let rpms = [12000.0; 4];
        let result = update_structural_safety(extreme_accel, rpms);

        assert_eq!(result.in_validated_envelope, 0); // Out of envelope!
        assert!(result.uncertainty > 0.5);
    }
}
