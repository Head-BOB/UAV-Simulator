// Phase 2 placeholder values. To be replaced with real measured component specs later.
const BATT_INTERNAL_RESISTANCE: f32 = 0.02; // Ohms
const ESC_RESISTANCE: f32 = 0.01; // Ohms
const MOTOR_WINDING_RESISTANCE: f32 = 0.05; // Ohms
const BATT_VOLTAGE_NOMINAL: f32 = 22.2; // Volts (6S LiPo)
const COOLING_COEFFICIENT: f32 = 0.5; // W/(m/s * C)
const AMBIENT_TEMP: f32 = 20.0; // Celsius
const THERMAL_MASS: f32 = 50.0; // Joules/Celsius

/// Calculates the new motor temperatures based on electrical heating and aerodynamic cooling.
///
/// # Units
/// * `throttles` - Normalized [0.0, 1.0] per motor
/// * `current_temps` - Degrees Celsius per motor
/// * `airspeed` - Meters per second
/// * `dt` - Seconds
/// * Returns - Updated temperatures in Degrees Celsius
pub fn update_temperatures(
    throttles: &[f32; 4],
    current_temps: &[f32; 4],
    airspeed: f32,
    dt: f32,
) -> [f32; 4] {
    let mut new_temps = [0.0; 4];

    for i in 0..4 {
        let effective_voltage = BATT_VOLTAGE_NOMINAL * throttles[i];
        let total_resistance = BATT_INTERNAL_RESISTANCE + ESC_RESISTANCE + MOTOR_WINDING_RESISTANCE;
        let current = effective_voltage / total_resistance;

        let heating_power = current * current * MOTOR_WINDING_RESISTANCE;
        let heat_added = heating_power * dt;

        let effective_cooling_rate = COOLING_COEFFICIENT * (1.0 + airspeed);
        let temp_diff = current_temps[i] - AMBIENT_TEMP;
        let heat_removed = effective_cooling_rate * temp_diff * dt;

        let temp_change = (heat_added - heat_removed) / THERMAL_MASS;
        new_temps[i] = current_temps[i] + temp_change;
    }

    new_temps
}
