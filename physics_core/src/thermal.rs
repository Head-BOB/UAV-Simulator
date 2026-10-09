// Tier: F
use crate::types::ThermalState;

// All constants below are ASSUMED. TODO(HUMAN-VERIFY): no source for any of them (item B-4).
// Battery internal resistance, ohm.
const BATT_INTERNAL_RESISTANCE: f64 = 0.02;
// ESC resistance, ohm.
const ESC_RESISTANCE: f64 = 0.01;
// Motor winding resistance, ohm (the only resistance that heats the motor).
const MOTOR_WINDING_RESISTANCE: f64 = 0.05;
// Nominal battery voltage, V.
const BATT_VOLTAGE_NOMINAL: f64 = 22.2;
// Ambient temperature, deg C.
const AMBIENT_TEMP: f64 = 20.0;
// Motor heat capacity, J/K (used as C in C dT/dt = P - h A (T - Ta)).
const MOTOR_HEAT_CAPACITY_J_PER_K: f64 = 50.0;
// Convective cooling area, m^2.
const COOLING_AREA: f64 = 0.01;
// Motor velocity constant, rpm per volt.
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
    let h = 10.0 + 3.0 * airspeed;
    step_motor_temps(state, throttles, motor_rpms, h, dt);
}

/// Below this `h * A` (W/K) the exponential form divides by ~0, so the adiabatic limit
/// `dT = P dt / C` is used instead. Numerical guard only: the model never reaches it
/// (`h >= 10` for any airspeed magnitude; only a test passes `h = 0`).
const ADIABATIC_HA_LIMIT_W_PER_K: f64 = 1.0e-12;

/// One thermal step with an explicit convection coefficient `h_w_per_m2k` (W/m^2/K, >= 0).
///
/// Split out of [`update_temperatures`] so the no-cooling case (`h = 0`) can be tested against
/// the closed form `rise = P t / C`; production code always passes `h >= 10`.
fn step_motor_temps(
    state: &mut ThermalState,
    throttles: &[f64; 4],
    motor_rpms: &[f64; 4],
    h_w_per_m2k: f64,
    dt: f64,
) {
    let total_resistance = BATT_INTERNAL_RESISTANCE + ESC_RESISTANCE + MOTOR_WINDING_RESISTANCE;
    let k_e = 60.0 / (2.0 * std::f64::consts::PI * MOTOR_KV_RPM_PER_V);
    let h_a_w_per_k = h_w_per_m2k * COOLING_AREA;
    let ha_over_mc = h_a_w_per_k / MOTOR_HEAT_CAPACITY_J_PER_K;
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
        if h_a_w_per_k < ADIABATIC_HA_LIMIT_W_PER_K {
            state.motor_temp_c[i] += heat_power * dt / MOTOR_HEAT_CAPACITY_J_PER_K;
            continue;
        }
        let equilibrium_temp = AMBIENT_TEMP + (heat_power / h_a_w_per_k);

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

        // Derivation of 3850.3125 W (rpm 0, so no back EMF):
        // I = 22.2 / (0.02 + 0.01 + 0.05) = 277.5 A; P = I^2 * 0.05 = 77006.25 * 0.05 = 3850.3125 W.
        let h = 10.0;
        let ha_over_mc = (h * COOLING_AREA) / MOTOR_HEAT_CAPACITY_J_PER_K;
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

    // Expected ratio written before running: with rpm proportional to throttle, back EMF
    // = rpm / Kv is proportional to throttle, so I and sqrt(P) are proportional, P ~ throttle^2,
    // ratio = 0.25. Hand calc (rpm = throttle * 10000, Kv 1000, R = 0.08, Rw = 0.05, hA = 0.1 W/K):
    //   full: (22.2 - 10) / 0.08 = 152.5 A; P = 152.5^2 * 0.05 = 1162.8125 W; rise 11628.125 C
    //   half: (11.1 - 5) / 0.08 = 76.25 A;  P = 290.703125 W;               rise 2907.03125 C
    // dt = 1e6 s makes exp(-0.002 * dt) underflow to 0, so T is the steady state.
    // Tolerance 1e-9 C: f64 ulp at 1e4 is 1.8e-12.
    #[test]
    fn test_thermal_power_ratio_half_vs_full_throttle() {
        let mut full = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        let mut half = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        update_temperatures(&mut full, &[1.0; 4], &[10000.0; 4], 0.0, 1.0e6);
        update_temperatures(&mut half, &[0.5; 4], &[5000.0; 4], 0.0, 1.0e6);

        let rise_full = full.motor_temp_c[0] - 20.0;
        let rise_half = half.motor_temp_c[0] - 20.0;
        assert!((full.motor_temp_c[0] - 11648.125).abs() < 1e-9);
        assert!((half.motor_temp_c[0] - 2927.03125).abs() < 1e-9);
        assert!((rise_half / rise_full - 0.25).abs() < 1e-12);
    }

    // Closed form of C dT/dt = P - hA (T - Ta), constant inputs, throttle 1, rpm 10000, 5 m/s:
    //   P = 1162.8125 W (see ratio test), h = 10 + 3 * 5 = 25, hA = 0.25 W/K, a = hA / C = 0.005 1/s
    //   T(1 s) = 20 + (1162.8125 / 0.25) * (1 - exp(-0.005)) = 43.19820615503639 C
    // The exact exponential step is dt-invariant for constant inputs. Rounding bound
    // 5000 * ulp(4671 C = 9.1e-13) = 4.5e-9; observed 3.7e-10 in an f64 replica. Tolerance 1e-8 C
    // (an explicit Euler step differs by 1.0e-4 C, so it is caught); plan bound 0.01 C also asserted.
    #[test]
    fn test_thermal_dt_0p002_vs_0p0002_one_second() {
        let mut coarse = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        let mut fine = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        for _ in 0..500 {
            update_temperatures(&mut coarse, &[1.0; 4], &[10000.0; 4], 5.0, 0.002);
        }
        for _ in 0..5000 {
            update_temperatures(&mut fine, &[1.0; 4], &[10000.0; 4], 5.0, 0.0002);
        }

        let closed_form = 43.19820615503639;
        assert!((coarse.motor_temp_c[0] - fine.motor_temp_c[0]).abs() < 0.01);
        assert!((coarse.motor_temp_c[0] - fine.motor_temp_c[0]).abs() < 1e-8);
        assert!((coarse.motor_temp_c[0] - closed_form).abs() < 1e-8);
        assert!((fine.motor_temp_c[0] - closed_form).abs() < 1e-8);
    }

    // Hand calc, no cooling (h overridden to 0), throttle 1, rpm 0 (no back EMF), dt = 1 s:
    //   I = V / R_total = 22.2 / (0.02 + 0.01 + 0.05) = 277.5 A
    //   P = I^2 * R_motor = 277.5^2 * 0.05 = 77006.25 * 0.05 = 3850.3125 W
    //   rise = P t / C = 3850.3125 * 1 / 50 = 77.00625 C, so T = 20 + 77.00625 = 97.00625 C
    // Tolerance 1e-6 C (the plan rule); f64 rounding here is about 1e-13.
    // Also: 2 x 0.5 s must equal 1 x 1 s (the adiabatic step is linear in dt) and a rest-start
    // at 50 C rises by the same 77.00625 C.
    #[test]
    fn test_thermal_no_cooling_rise_equals_p_t_over_c() {
        let mut one_step = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        step_motor_temps(&mut one_step, &[1.0; 4], &[0.0; 4], 0.0, 1.0);
        assert!((one_step.motor_temp_c[0] - 97.00625).abs() < 1e-6);

        let mut two_steps = ThermalState {
            motor_temp_c: [20.0; 4],
        };
        step_motor_temps(&mut two_steps, &[1.0; 4], &[0.0; 4], 0.0, 0.5);
        step_motor_temps(&mut two_steps, &[1.0; 4], &[0.0; 4], 0.0, 0.5);
        assert!((two_steps.motor_temp_c[0] - 97.00625).abs() < 1e-6);

        let mut warm = ThermalState {
            motor_temp_c: [50.0; 4],
        };
        step_motor_temps(&mut warm, &[1.0; 4], &[0.0; 4], 0.0, 1.0);
        assert!((warm.motor_temp_c[0] - 127.00625).abs() < 1e-6);
    }
}
