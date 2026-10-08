// PLACEHOLDER: battery internal resistance (Ohms) — not measured from real
// hardware yet. See Dev 4 Phase 2 guide, Task 2.
const BATT_INTERNAL_RESISTANCE: f32 = 0.02;

// PLACEHOLDER: ESC internal resistance (Ohms) — not measured from real
// hardware yet. See Dev 4 Phase 2 guide, Task 2.
const ESC_RESISTANCE: f32 = 0.01;

// PLACEHOLDER: motor winding resistance (Ohms) — not measured from real
// hardware yet. See Dev 4 Phase 2 guide, Task 2.
const MOTOR_WINDING_RESISTANCE: f32 = 0.05;

// PLACEHOLDER: nominal battery pack voltage (Volts, 6S LiPo) — not measured from real
// hardware yet. See Dev 4 Phase 2 guide, Task 2.
const BATT_VOLTAGE_NOMINAL: f32 = 22.2;

// PLACEHOLDER: convective cooling coefficient (W/(m/s * C)) — not measured from real
// hardware yet. See Dev 4 Phase 2 guide, Task 2.
const COOLING_COEFFICIENT: f32 = 0.5;

// PLACEHOLDER: ambient operating temperature (Celsius) — not measured from real
// hardware yet. See Dev 4 Phase 2 guide, Task 2.
const AMBIENT_TEMP: f32 = 20.0;

// PLACEHOLDER: lumped motor thermal mass (Joules/Celsius) — not measured from real
// hardware yet. See Dev 4 Phase 2 guide, Task 2.
const THERMAL_MASS: f32 = 50.0;

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
