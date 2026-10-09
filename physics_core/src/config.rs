// Tier: F
use crate::aero;
use crate::types::VehicleConfig;
use std::fs;

/// Altitude at which `hover_throttle` is derived: a sea-level (0 m) reference.
/// `aero::get_thrust` uses rho(h), so the hover throttle at altitude h is higher.
const HOVER_REFERENCE_ALTITUDE_M: f64 = 0.0;

/// Reason a vehicle configuration was rejected. Each variant maps to one
/// distinct FFI return code in `ffi_load_vehicle_config`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    IoError,
    ParseError,
    InvalidMass,
    InvalidInertia,
    InvalidSpin,
    InvalidMotorPosition,
    InvalidPropDiameter,
    InvalidThrustCoeff,
    InvalidTorqueCoeff,
    InvalidMaxRpm,
    InvalidHoverThrottle,
}

/// True for a finite value strictly greater than zero.
///
/// Why: `x <= 0.0 || x.is_nan()` lets +Inf through; a non-finite physical
/// property would poison the integrator.
fn is_positive_finite(x: f64) -> bool {
    x.is_finite() && x > 0.0
}

/// Validates every field of a `VehicleConfig`. The first failure wins, checked in
/// struct order (mass, inertia, motor positions, spin, prop diameter, C_T, kappa (N*m/N),
/// max RPM, hover throttle).
///
/// Rules (units as in `types::VehicleConfig`):
/// * `mass_kg`, each `inertia_kgm2`, `prop_diameter_m`, `thrust_coeff`,
///   `torque_coeff`, `max_rpm`: finite and > 0.
/// * each `motor_pos_m` component: finite (any sign, body frame, metres).
/// * each `motor_spin`: exactly 1.0 or -1.0 (exact sentinel comparison; NaN fails).
/// * `hover_throttle`: finite, 0 < x <= 1.
///
/// "Four motors" is enforced by the fixed-size arrays (a wrong count is a parse error).
///
/// TODO(HUMAN-VERIFY): plausibility upper bounds (mass, RPM, ...) need sourced
/// vehicle data (dev-docs B-5); none are invented here.
///
/// # Errors
/// Returns the `ConfigError` variant of the first field that violates its rule.
pub fn validate_vehicle_config(cfg: &VehicleConfig) -> Result<(), ConfigError> {
    if !is_positive_finite(cfg.mass_kg) {
        return Err(ConfigError::InvalidMass);
    }
    if !cfg.inertia_kgm2.iter().all(|&i| is_positive_finite(i)) {
        return Err(ConfigError::InvalidInertia);
    }
    if !cfg.motor_pos_m.iter().flatten().all(|c| c.is_finite()) {
        return Err(ConfigError::InvalidMotorPosition);
    }
    // Exact comparison is intentional: spin is a +1/-1 sentinel, not a measurement.
    if !cfg.motor_spin.iter().all(|&s| s == 1.0 || s == -1.0) {
        return Err(ConfigError::InvalidSpin);
    }
    if !is_positive_finite(cfg.prop_diameter_m) {
        return Err(ConfigError::InvalidPropDiameter);
    }
    if !is_positive_finite(cfg.thrust_coeff) {
        return Err(ConfigError::InvalidThrustCoeff);
    }
    if !is_positive_finite(cfg.torque_coeff) {
        return Err(ConfigError::InvalidTorqueCoeff);
    }
    if !is_positive_finite(cfg.max_rpm) {
        return Err(ConfigError::InvalidMaxRpm);
    }
    if !(is_positive_finite(cfg.hover_throttle) && cfg.hover_throttle <= 1.0) {
        return Err(ConfigError::InvalidHoverThrottle);
    }
    Ok(())
}

/// Why `validate_runtime_config` rejected a config passed to `ffi_step_physics`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeConfigError {
    /// A checked field is NaN or infinite (reported as a divergence: code 8 + latch).
    NonFinite,
    /// `mass_kg` <= 0.
    Mass,
    /// An inertia component <= 0.
    Inertia,
    /// `max_rpm` < 0 (0 is allowed: motors that cannot spin give zero thrust).
    MaxRpm,
}

/// Cheap per-step re-check of a `VehicleConfig` that C++ may have filled by hand, bypassing
/// `ffi_load_vehicle_config`. Checks only what would make the step silently wrong or divide
/// by zero: every numeric field finite; `mass_kg > 0`; each inertia > 0; `max_rpm >= 0`.
/// The full load-time rules (spin sentinels, hover throttle, ...) stay in `validate_vehicle_config`.
pub fn validate_runtime_config(cfg: &VehicleConfig) -> Result<(), RuntimeConfigError> {
    let scalars = [
        cfg.mass_kg,
        cfg.prop_diameter_m,
        cfg.thrust_coeff,
        cfg.torque_coeff,
        cfg.max_rpm,
        cfg.hover_throttle,
    ];
    let finite = scalars.iter().all(|v| v.is_finite())
        && cfg.inertia_kgm2.iter().all(|v| v.is_finite())
        && cfg.motor_pos_m.iter().flatten().all(|v| v.is_finite())
        && cfg.motor_spin.iter().all(|v| v.is_finite());
    if !finite {
        return Err(RuntimeConfigError::NonFinite);
    }
    if !is_positive_finite(cfg.mass_kg) {
        return Err(RuntimeConfigError::Mass);
    }
    if !cfg.inertia_kgm2.iter().all(|&i| is_positive_finite(i)) {
        return Err(RuntimeConfigError::Inertia);
    }
    if cfg.max_rpm < 0.0 {
        return Err(RuntimeConfigError::MaxRpm);
    }
    Ok(())
}

/// Parses vehicle configuration JSON text and validates it.
///
/// # Errors
/// `ParseError` for malformed JSON, a missing field, a wrong array length or an
/// out-of-range number; otherwise the error from `validate_vehicle_config`.
pub fn parse_vehicle_config(json: &str) -> Result<VehicleConfig, ConfigError> {
    let mut config: VehicleConfig =
        serde_json::from_str(json).map_err(|_| ConfigError::ParseError)?;
    // Derived, not stored: a stored value goes stale when mass or C_T changes.
    // Bad mass/C_T/etc. give NaN/Inf here; validate reports the field in order,
    // and hover_throttle > 1 (cannot hover) is rejected by the same check.
    config.hover_throttle = aero::hover_throttle(&config, HOVER_REFERENCE_ALTITUDE_M)
        .map_err(|_| ConfigError::InvalidHoverThrottle)?;
    validate_vehicle_config(&config)?;
    Ok(config)
}

/// Loads and validates physical properties from a vehicle JSON configuration file.
///
/// # Errors
/// `IoError` if the file cannot be read; otherwise see `parse_vehicle_config`.
pub fn load_vehicle_config(path: &str) -> Result<VehicleConfig, ConfigError> {
    let data = fs::read_to_string(path).map_err(|_| ConfigError::IoError)?;
    parse_vehicle_config(&data)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHIPPED_PATH: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../offline_pipeline/geometry/vehicle_config.json"
    );

    /// Equal to the shipped vehicle_config.json; every field satisfies its rule
    /// (positive finite, spins +-1, 0 < 0.5015 <= 1).
    fn base() -> VehicleConfig {
        VehicleConfig {
            mass_kg: 1.5,
            inertia_kgm2: [0.015, 0.015, 0.025],
            motor_pos_m: [
                [0.2, 0.2, 0.0],
                [0.2, -0.2, 0.0],
                [-0.2, 0.2, 0.0],
                [-0.2, -0.2, 0.0],
            ],
            motor_spin: [1.0, -1.0, -1.0, 1.0],
            prop_diameter_m: 0.25,
            thrust_coeff: 0.11,
            torque_coeff: 0.005,
            max_rpm: 10000.0,
            hover_throttle: 0.5015112082212391, // derived for 1.5 kg, see aero tests
        }
    }

    const BAD_POSITIVE: [f64; 4] = [0.0, -1.0, f64::INFINITY, f64::NAN];

    fn shipped_text() -> String {
        fs::read_to_string(SHIPPED_PATH).expect("shipped config readable")
    }

    // Hand calc: the shipped config passes; each single-field break maps to exactly one variant.
    // mass 0 and -1 -> Mass; inertia[1] 0 -> Inertia; max_rpm -1 -> MaxRpm, max_rpm 0 is allowed;
    // NaN / +Inf / -Inf in any checked field (mass, spin, position, torque_coeff) -> NonFinite.
    #[test]
    fn test_validate_runtime_config_truth_table() {
        assert_eq!(validate_runtime_config(&base()), Ok(()));
        let mut c = base();
        c.mass_kg = 0.0;
        assert_eq!(validate_runtime_config(&c), Err(RuntimeConfigError::Mass));
        c.mass_kg = -1.0;
        assert_eq!(validate_runtime_config(&c), Err(RuntimeConfigError::Mass));
        let mut c = base();
        c.inertia_kgm2[1] = 0.0;
        assert_eq!(
            validate_runtime_config(&c),
            Err(RuntimeConfigError::Inertia)
        );
        let mut c = base();
        c.max_rpm = -1.0;
        assert_eq!(validate_runtime_config(&c), Err(RuntimeConfigError::MaxRpm));
        c.max_rpm = 0.0;
        assert_eq!(validate_runtime_config(&c), Ok(()));
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut c = base();
            c.mass_kg = bad;
            assert_eq!(
                validate_runtime_config(&c),
                Err(RuntimeConfigError::NonFinite)
            );
            let mut c = base();
            c.motor_spin[2] = bad;
            assert_eq!(
                validate_runtime_config(&c),
                Err(RuntimeConfigError::NonFinite)
            );
            let mut c = base();
            c.motor_pos_m[3][1] = bad;
            assert_eq!(
                validate_runtime_config(&c),
                Err(RuntimeConfigError::NonFinite)
            );
            let mut c = base();
            c.torque_coeff = bad;
            assert_eq!(
                validate_runtime_config(&c),
                Err(RuntimeConfigError::NonFinite)
            );
        }
    }

    #[test]
    fn test_config_base_is_valid() {
        assert_eq!(validate_vehicle_config(&base()), Ok(()));
    }

    #[test]
    fn test_config_shipped_file_is_valid() {
        let cfg = load_vehicle_config(SHIPPED_PATH).expect("shipped config valid");
        assert_eq!(cfg.mass_kg, 1.5);
    }

    #[test]
    fn test_config_rejects_negative_mass() {
        let mut c = base();
        c.mass_kg = -1.0;
        assert_eq!(validate_vehicle_config(&c), Err(ConfigError::InvalidMass));
    }

    #[test]
    fn test_config_rejects_bad_mass() {
        for bad in [0.0, f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            let mut c = base();
            c.mass_kg = bad;
            assert_eq!(validate_vehicle_config(&c), Err(ConfigError::InvalidMass));
        }
    }

    #[test]
    fn test_config_rejects_bad_inertia() {
        for i in 0..3 {
            for bad in BAD_POSITIVE {
                let mut c = base();
                c.inertia_kgm2[i] = bad;
                assert_eq!(
                    validate_vehicle_config(&c),
                    Err(ConfigError::InvalidInertia)
                );
            }
        }
    }

    #[test]
    fn test_config_rejects_bad_motor_position() {
        for m in 0..4 {
            for a in 0..3 {
                for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                    let mut c = base();
                    c.motor_pos_m[m][a] = bad;
                    assert_eq!(
                        validate_vehicle_config(&c),
                        Err(ConfigError::InvalidMotorPosition)
                    );
                }
            }
        }
        // Rule is finiteness only: zero and negative coordinates are valid.
        let mut c = base();
        c.motor_pos_m[0] = [0.0, -0.3, 0.0];
        assert_eq!(validate_vehicle_config(&c), Ok(()));
    }

    #[test]
    fn test_config_rejects_bad_spin() {
        for m in 0..4 {
            for bad in [0.0, 0.5, 2.0, f64::NAN] {
                let mut c = base();
                c.motor_spin[m] = bad;
                assert_eq!(validate_vehicle_config(&c), Err(ConfigError::InvalidSpin));
            }
        }
    }

    #[test]
    fn test_config_rejects_bad_prop_diameter() {
        for bad in BAD_POSITIVE {
            let mut c = base();
            c.prop_diameter_m = bad;
            assert_eq!(
                validate_vehicle_config(&c),
                Err(ConfigError::InvalidPropDiameter)
            );
        }
    }

    #[test]
    fn test_config_rejects_bad_thrust_coeff() {
        for bad in BAD_POSITIVE {
            let mut c = base();
            c.thrust_coeff = bad;
            assert_eq!(
                validate_vehicle_config(&c),
                Err(ConfigError::InvalidThrustCoeff)
            );
        }
    }

    #[test]
    fn test_config_rejects_bad_torque_coeff() {
        for bad in BAD_POSITIVE {
            let mut c = base();
            c.torque_coeff = bad;
            assert_eq!(
                validate_vehicle_config(&c),
                Err(ConfigError::InvalidTorqueCoeff)
            );
        }
    }

    #[test]
    fn test_config_rejects_bad_max_rpm() {
        for bad in BAD_POSITIVE {
            let mut c = base();
            c.max_rpm = bad;
            assert_eq!(validate_vehicle_config(&c), Err(ConfigError::InvalidMaxRpm));
        }
    }

    #[test]
    fn test_config_hover_throttle_range() {
        for bad in [0.0, -0.1, 1.0 + f64::EPSILON, f64::INFINITY, f64::NAN] {
            let mut c = base();
            c.hover_throttle = bad;
            assert_eq!(
                validate_vehicle_config(&c),
                Err(ConfigError::InvalidHoverThrottle)
            );
        }
        let mut c = base();
        c.hover_throttle = 1.0;
        assert_eq!(validate_vehicle_config(&c), Ok(()));
    }

    // Hand calc: see aero::tests (tau = 0.5015112082212391 for the shipped 1.5 kg).
    #[test]
    fn test_config_shipped_hover_throttle_derived() {
        let cfg = load_vehicle_config(SHIPPED_PATH).expect("shipped config valid");
        assert!((cfg.hover_throttle - 0.5015112082212391).abs() < 1e-12);
    }

    #[test]
    fn test_config_hover_throttle_key_rejected() {
        let text = shipped_text().replace(
            "\"max_rpm\": 10000.0",
            "\"max_rpm\": 10000.0, \"hover_throttle\": 0.4095",
        );
        assert!(text.contains("hover_throttle"), "fixture edit applied");
        assert_eq!(
            parse_vehicle_config(&text).err(),
            Some(ConfigError::ParseError)
        );
    }

    // Hand calc: 4 * T_max / g = 4 * 14.621457167857683 / 9.80665 = 5.9639 kg is the
    // heaviest vehicle that can hover. 6.0 kg -> tau 1.00303 (reject), 5.9 kg -> 0.99463.
    #[test]
    fn test_config_cannot_hover_rejected() {
        let heavy = shipped_text().replace("\"mass_kg\": 1.5", "\"mass_kg\": 6.0");
        let light = shipped_text().replace("\"mass_kg\": 1.5", "\"mass_kg\": 5.9");
        assert!(
            heavy.contains("6.0") && light.contains("5.9"),
            "fixture edit applied"
        );
        assert_eq!(
            parse_vehicle_config(&heavy).err(),
            Some(ConfigError::InvalidHoverThrottle)
        );
        assert!(parse_vehicle_config(&light).is_ok());
    }

    #[test]
    fn test_config_missing_field_is_parse_error() {
        let text = shipped_text().replace("\"mass_kg\"", "\"mass_kg_renamed\"");
        assert_eq!(
            parse_vehicle_config(&text).err(),
            Some(ConfigError::ParseError)
        );
    }

    #[test]
    fn test_config_wrong_motor_count_is_parse_error() {
        let text = shipped_text().replace("[1.0, -1.0, -1.0, 1.0]", "[1.0, -1.0, -1.0]");
        assert!(text.contains("[1.0, -1.0, -1.0]"), "fixture edit applied");
        assert_eq!(
            parse_vehicle_config(&text).err(),
            Some(ConfigError::ParseError)
        );
    }

    #[test]
    fn test_config_overflow_number_rejected() {
        let text = shipped_text().replace("1.5,", "1e400,");
        assert!(text.contains("1e400"), "fixture edit applied");
        // serde_json reports an out-of-range number as a parse error.
        assert_eq!(
            parse_vehicle_config(&text).err(),
            Some(ConfigError::ParseError)
        );
    }

    #[test]
    fn test_config_missing_file_is_io_error() {
        assert_eq!(
            load_vehicle_config("/nonexistent/vehicle_config.json").err(),
            Some(ConfigError::IoError)
        );
    }
}
