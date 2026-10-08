use crate::types::ThermalState;

const BATT_INTERNAL_RESISTANCE: f64 = 0.02;
const ESC_RESISTANCE: f64 = 0.01;
const MOTOR_WINDING_RESISTANCE: f64 = 0.05;
const BATT_VOLTAGE_NOMINAL: f64 = 22.2;
const AMBIENT_TEMP: f64 = 20.0;
const THERMAL_MASS: f64 = 50.0;
const COOLING_AREA: f64 = 0.01;
const MOTOR_KV_RPM_PER_V: f64 = 1000.0;

/// Calculates the new motor temperatures based on electrical heating and aerodynamic cooling.
///
/// Electrical model:
/// I = (V - k_e * w) / R
///
/// Thermal model:
/// P_cool = h * A * (T - T_ambient)
///
/// # Units
/// * `throttles` - Normalized [0.0, 1.0]
/// * `motor_rpms` - Revolutions per minute
/// * `airspeed` - Meters per second
/// * `dt` - Seconds
pub fn update_temperatures(
    state: &mut ThermalState,
    throttles: &[f64; 4],
    motor_rpms: &[f64; 4],
    airspeed: f64,
    dt: f64,
) {
    let total_resistance = BATT_INTERNAL_RESISTANCE + ESC_RESISTANCE + MOTOR_WINDING_RESISTANCE;
    let k_e = 60.0 / (2.0 * std::f64::consts::PI * MOTOR_KV_RPM_PER_V);
    let h = 10.0 + 3.0 * airspeed;
    let ha_over_mc = (h * COOLING_AREA) / THERMAL_MASS;
    let exp_term = (-ha_over_mc * dt).exp();

    for i in 0..4 {
        let effective_voltage = BATT_VOLTAGE_NOMINAL * throttles[i];
        let w_rad_s = motor_rpms[i] * 2.0 * std::f64::consts::PI / 60.0;
        let back_emf = k_e * w_rad_s;

        let mut current = (effective_voltage - back_emf) / total_resistance;
        if current < 0.0 {
            current = 0.0;
        }

        let heat_power = current * current * MOTOR_WINDING_RESISTANCE;
        let equilibrium_temp = AMBIENT_TEMP + (heat_power / (h * COOLING_AREA));

        state.motor_temp_c[i] =
            equilibrium_temp + (state.motor_temp_c[i] - equilibrium_temp) * exp_term;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_thermal_zero_throttle_long_time() {
        let mut state = ThermalState {
            motor_temp_c: [100.0; 4],
        };
        update_temperatures(&mut state, &[0.0; 4], &[0.0; 4], 0.0, 10000.0);
        assert!((state.motor_temp_c[0] - AMBIENT_TEMP).abs() < 1e-6);
    }

    #[test]
    fn test_thermal_fixed_power_no_airflow_short_time() {
        let mut state = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        let dt = 1.0;

        update_temperatures(&mut state, &[1.0; 4], &[0.0; 4], 0.0, dt);

        let h = 10.0;
        let ha_over_mc = (h * COOLING_AREA) / THERMAL_MASS;
        let exp_term = (-ha_over_mc * dt).exp();
        let eq_temp = AMBIENT_TEMP + (3850.3125 / (h * COOLING_AREA));
        let expected_t = eq_temp + (20.0 - eq_temp) * exp_term;

        assert!((state.motor_temp_c[0] - expected_t).abs() < 1e-6);
    }

    #[test]
    fn test_thermal_steady_state_fixed_power() {
        let mut state = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        update_temperatures(&mut state, &[0.5; 4], &[5000.0; 4], 10.0, 10000.0);

        let k_e = 60.0 / (2.0 * std::f64::consts::PI * MOTOR_KV_RPM_PER_V);
        let w_rad_s = 5000.0 * 2.0 * std::f64::consts::PI / 60.0;
        let back_emf = k_e * w_rad_s;
        let current = (BATT_VOLTAGE_NOMINAL * 0.5 - back_emf)
            / (BATT_INTERNAL_RESISTANCE + ESC_RESISTANCE + MOTOR_WINDING_RESISTANCE);
        let heat_power = current * current * MOTOR_WINDING_RESISTANCE;
        let h = 10.0 + 3.0 * 10.0;
        let expected_t = AMBIENT_TEMP + (heat_power / (h * COOLING_AREA));

        assert!((state.motor_temp_c[0] - expected_t).abs() < 0.1);
    }

    #[test]
    fn test_thermal_dt_invariance() {
        let mut state1 = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        let mut state2 = ThermalState {
            motor_temp_c: [20.0; 4],
        };

        update_temperatures(&mut state1, &[1.0; 4], &[1000.0; 4], 5.0, 0.002);
        update_temperatures(&mut state2, &[1.0; 4], &[1000.0; 4], 5.0, 0.001);
        update_temperatures(&mut state2, &[1.0; 4], &[1000.0; 4], 5.0, 0.001);

        assert!((state1.motor_temp_c[0] - state2.motor_temp_c[0]).abs() < 0.01);
    }
}
