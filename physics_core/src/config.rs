use crate::types::VehicleConfig;
use std::fs;

pub enum ConfigError {
    IoError,
    ParseError,
    InvalidMass,
    InvalidInertia,
    InvalidSpin,
}

/// Loads and validates physical properties from a vehicle JSON configuration file.
///
/// # Errors
/// Returns IoError if the file cannot be read.
/// Returns ParseError if the JSON structure is malformed.
/// Returns InvalidMass if mass_kg is negative, zero, or NaN.
/// Returns InvalidInertia if any inertia component is negative, zero, or NaN.
/// Returns InvalidSpin if motor spin directions are not exactly 1.0 or -1.0.
pub fn load_vehicle_config(path: &str) -> Result<VehicleConfig, ConfigError> {
    let data = fs::read_to_string(path).map_err(|_| ConfigError::IoError)?;
    let config: VehicleConfig = serde_json::from_str(&data).map_err(|_| ConfigError::ParseError)?;

    if config.mass_kg <= 0.0 || config.mass_kg.is_nan() {
        return Err(ConfigError::InvalidMass);
    }

    for &i in &config.inertia_kgm2 {
        if i <= 0.0 || i.is_nan() {
            return Err(ConfigError::InvalidInertia);
        }
    }

    for &s in &config.motor_spin {
        if s != 1.0 && s != -1.0 {
            return Err(ConfigError::InvalidSpin);
        }
    }

    Ok(config)
}
