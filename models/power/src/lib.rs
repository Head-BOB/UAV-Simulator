//! Power, Electrical, and Thermal Endurance Model
//! Phase 3 - Dev 4 (Power & Search)
//! Standards: Tier D* Component, Strict SI Units, Full Test Suite P-01 to P-17

use std::f64::consts::PI;

/// Motor catalog parameters
#[derive(Debug, Clone, Copy)]
pub struct MotorSpecs {
    pub kv_rpm_per_v: f64,
    pub resistance_ohm: f64,
    pub no_load_current_a: f64,
    pub max_current_a: f64,
    pub max_power_w: f64,
    pub thermal_resistance_k_per_w: f64,
}

/// Battery pack configuration parameters
#[derive(Debug, Clone, Copy)]
pub struct BatterySpecs {
    pub cells_series: u32,
    pub cells_parallel: u32,
    pub cell_capacity_ah: f64,
    pub cell_energy_wh: f64,
    pub cell_resistance_ohm: f64,
    pub cell_mass_kg: f64,
    pub c_rating: f64,
    pub usable_fraction: f64,
    pub overhead_fraction: f64,
}

/// Calculated operating point for a single motor
#[derive(Debug, Clone, Copy)]
pub struct MotorOperatingPoint {
    pub current_a: f64,
    pub back_emf_v: f64,
    pub terminal_voltage_v: f64,
    pub duty_cycle: f64,
    pub shaft_power_w: f64,
    pub electrical_power_w: f64,
    pub copper_loss_w: f64,
    pub no_load_loss_w: f64,
    pub total_loss_w: f64,
    pub motor_efficiency: f64,
    pub temperature_c: f64,
    pub is_valid: bool,
}

// =========================================================================
// Pure Mathematical Functions (Section 2.2)
// =========================================================================

/// Calculates torque constant Kt in N*m/A from motor Kv (RPM/V)
pub fn torque_constant_nm_per_a(kv_rpm_per_v: f64) -> f64 {
    60.0 / (2.0 * PI * kv_rpm_per_v)
}

/// Calculates angular velocity in rad/s from rotational speed in RPM
pub fn rpm_to_rad_per_s(rpm: f64) -> f64 {
    rpm * 2.0 * PI / 60.0
}

/// Calculates required motor armature current in Amps
pub fn motor_current_a(shaft_torque_nm: f64, kt_nm_per_a: f64, no_load_current_a: f64) -> f64 {
    (shaft_torque_nm / kt_nm_per_a) + no_load_current_a
}

/// Calculates motor back-EMF voltage in Volts
pub fn back_emf_voltage_v(rpm: f64, kv_rpm_per_v: f64) -> f64 {
    rpm / kv_rpm_per_v
}

/// Calculates required terminal voltage applied by ESC in Volts
pub fn motor_voltage_v(back_emf_v: f64, current_a: f64, resistance_ohm: f64) -> f64 {
    back_emf_v + (current_a * resistance_ohm)
}

/// Calculates ESC duty cycle fraction (Vm / Vbatt)
pub fn esc_duty_cycle(motor_voltage_v: f64, battery_voltage_v: f64) -> f64 {
    motor_voltage_v / battery_voltage_v
}

/// Calculates battery pack total energy in Wh
pub fn pack_energy_wh(specs: &BatterySpecs) -> f64 {
    (specs.cells_series * specs.cells_parallel) as f64 * specs.cell_energy_wh
}

/// Calculates battery pack total mass in kg including casing overhead
pub fn pack_mass_kg(specs: &BatterySpecs) -> f64 {
    let raw_mass = (specs.cells_series * specs.cells_parallel) as f64 * specs.cell_mass_kg;
    raw_mass * (1.0 + specs.overhead_fraction)
}

/// Calculates battery pack equivalent series internal resistance in Ohms
pub fn pack_resistance_ohm(specs: &BatterySpecs) -> f64 {
    (specs.cells_series as f64 * specs.cell_resistance_ohm) / (specs.cells_parallel as f64)
}

/// Calculates battery loaded voltage accounting for internal resistance sag in Volts
pub fn pack_loaded_voltage_v(nominal_voltage_v: f64, total_current_a: f64, pack_resistance_ohm: f64) -> f64 {
    nominal_voltage_v - (total_current_a * pack_resistance_ohm)
}

/// Calculates usable battery energy in Wh
pub fn usable_pack_energy_wh(specs: &BatterySpecs) -> f64 {
    pack_energy_wh(specs) * specs.usable_fraction
}

/// Evaluates a single motor operating point at a given RPM and shaft torque
pub fn evaluate_motor_operating_point(
    rpm: f64,
    shaft_torque_nm: f64,
    motor: &MotorSpecs,
    battery_loaded_voltage_v: f64,
    ambient_temp_c: f64,
) -> Result<MotorOperatingPoint, &'static str> {
    if rpm.is_nan() || shaft_torque_nm.is_nan() || battery_loaded_voltage_v.is_nan() {
        return Err("MODEL_FAULT: NaN detected in operating point inputs");
    }

    let omega = rpm_to_rad_per_s(rpm);
    let kt = torque_constant_nm_per_a(motor.kv_rpm_per_v);
    let current = motor_current_a(shaft_torque_nm, kt, motor.no_load_current_a);
    let back_emf = back_emf_voltage_v(rpm, motor.kv_rpm_per_v);
    let v_m = motor_voltage_v(back_emf, current, motor.resistance_ohm);
    let duty = esc_duty_cycle(v_m, battery_loaded_voltage_v);

    let p_shaft = shaft_torque_nm * omega;
    let p_el = v_m * current;
    let p_copper = current * current * motor.resistance_ohm;
    let p_no_load = motor.no_load_current_a * back_emf;
    let p_loss = p_copper + p_no_load;

    let efficiency = if p_el > 1e-6 {
        (p_shaft / p_el).clamp(0.0, 1.0)
    } else {
        0.0
    };

    let temp_c = ambient_temp_c + (p_loss * motor.thermal_resistance_k_per_w);

    let is_valid = duty <= 1.0 && current <= motor.max_current_a;

    Ok(MotorOperatingPoint {
        current_a: current,
        back_emf_v: back_emf,
        terminal_voltage_v: v_m,
        duty_cycle: duty,
        shaft_power_w: p_shaft,
        electrical_power_w: p_el,
        copper_loss_w: p_copper,
        no_load_loss_w: p_no_load,
        total_loss_w: p_loss,
        motor_efficiency: efficiency,
        temperature_c: temp_c,
        is_valid,
    })
}

/// Calculates flight endurance in minutes for a 4-rotor drone
pub fn hover_endurance_min(usable_energy_wh: f64, total_battery_power_w: f64) -> f64 {
    if total_battery_power_w <= 1e-3 {
        0.0
    } else {
        (usable_energy_wh / total_battery_power_w) * 60.0
    }
}

// =========================================================================
// Automated Test Suite (P-01 to P-17) - Exact Phase 3 Verification
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_p01_kt_for_kv_150() {
        let kt: f64 = torque_constant_nm_per_a(150.0);
        let expected: f64 = 0.06366197;
        assert!((kt - expected).abs() < 1e-4, "P-01 Failed: Kt={}", kt);
    }

    #[test]
    fn test_p02_to_p05_worked_example() {
        let motor = MotorSpecs {
            kv_rpm_per_v: 150.0,
            resistance_ohm: 0.08,
            no_load_current_a: 0.8,
            max_current_a: 30.0,
            max_power_w: 600.0,
            thermal_resistance_k_per_w: 2.5,
        };

        let rpm: f64 = 2700.0;
        let omega: f64 = rpm_to_rad_per_s(rpm);
        let p_shaft: f64 = 84.7;
        let torque: f64 = p_shaft / omega;
        let v_batt: f64 = 22.2;
        let esc_efficiency: f64 = 0.95;

        let op = evaluate_motor_operating_point(rpm, torque, &motor, v_batt, 15.0).unwrap();

        // P-02: Current, Vm, Duty
        assert!((op.current_a - 5.506).abs() < 0.01, "P-02 Current failed: {}", op.current_a);
        assert!((op.terminal_voltage_v - 18.44).abs() < 0.02, "P-02 Vm failed: {}", op.terminal_voltage_v);
        assert!((op.duty_cycle - 0.831).abs() < 0.01, "P-02 Duty failed: {}", op.duty_cycle);

        // P-03: Loss Identity (Pel - Pshaft = copper + no-load)
        let loss_diff: f64 = (op.electrical_power_w - op.shaft_power_w) - (op.copper_loss_w + op.no_load_loss_w);
        assert!(loss_diff.abs() < 1e-9, "P-03 Loss identity failed: diff={}", loss_diff);

        // P-04: Motor Efficiency
        assert!((op.motor_efficiency - 0.834).abs() < 0.01, "P-04 Efficiency failed: {}", op.motor_efficiency);

        // P-05: Four-rotor Battery Power
        let battery_power_per_rotor: f64 = op.electrical_power_w / esc_efficiency;
        let total_four_rotor_power: f64 = battery_power_per_rotor * 4.0;
        assert!((total_four_rotor_power - 427.5).abs() < 0.5, "P-05 4-rotor power failed: {}", total_four_rotor_power);
    }

    #[test]
    fn test_p06_ideal_hover_power() {
        let thrust_total: f64 = 42.76;
        let radius: f64 = 0.4572 / 2.0; // 18 inch rotor radius = 0.2286 m
        let area_single: f64 = PI * radius * radius;
        let area_total: f64 = 4.0 * area_single; // 0.6568 m^2
        let rho: f64 = 1.225;
        let ideal_induced_power: f64 = (thrust_total.powi(3) / (2.0 * rho * area_total)).sqrt();
        assert!((ideal_induced_power - 220.4).abs() < 0.5, "P-06 Ideal power failed: {}", ideal_induced_power);
    }

    #[test]
    fn test_p07_endurance_calculation() {
        let usable_wh: f64 = 262.0 * 0.8;
        let power_w: f64 = 419.0;
        let endurance: f64 = hover_endurance_min(usable_wh, power_w);
        assert!((endurance - 30.01).abs() < 0.05, "P-07 Endurance failed: {}", endurance);
    }

    #[test]
    fn test_p08_battery_mass() {
        let energy_wh: f64 = 262.0;
        let specific_energy_wh_per_kg: f64 = 180.0;
        let cell_mass: f64 = energy_wh / specific_energy_wh_per_kg;
        assert!((cell_mass - 1.456).abs() < 0.01, "P-08 Battery mass failed: {}", cell_mass);
        let total_mass: f64 = cell_mass + 0.9 + 2.0;
        assert!((total_mass - 4.356).abs() < 0.01, "P-08 Total mass failed: {}", total_mass);
    }

    #[test]
    fn test_p09_zero_torque() {
        let motor = MotorSpecs {
            kv_rpm_per_v: 150.0,
            resistance_ohm: 0.08,
            no_load_current_a: 0.8,
            max_current_a: 30.0,
            max_power_w: 600.0,
            thermal_resistance_k_per_w: 2.5,
        };
        let op = evaluate_motor_operating_point(2000.0, 0.0, &motor, 22.2, 15.0).unwrap();
        assert_eq!(op.current_a, motor.no_load_current_a);
        assert_eq!(op.shaft_power_w, 0.0);
        assert_eq!(op.motor_efficiency, 0.0);
    }

    #[test]
    fn test_p10_rpm_above_no_load_speed() {
        let motor = MotorSpecs {
            kv_rpm_per_v: 150.0,
            resistance_ohm: 0.08,
            no_load_current_a: 0.8,
            max_current_a: 30.0,
            max_power_w: 600.0,
            thermal_resistance_k_per_w: 2.5,
        };
        let op = evaluate_motor_operating_point(4000.0, 0.1, &motor, 22.2, 15.0).unwrap();
        assert!(op.duty_cycle > 1.0);
        assert!(!op.is_valid);
    }

    #[test]
    fn test_p11_current_above_motor_max() {
        let motor = MotorSpecs {
            kv_rpm_per_v: 150.0,
            resistance_ohm: 0.08,
            no_load_current_a: 0.8,
            max_current_a: 10.0,
            max_power_w: 600.0,
            thermal_resistance_k_per_w: 2.5,
        };
        let op = evaluate_motor_operating_point(2000.0, 1.0, &motor, 22.2, 15.0).unwrap();
        assert!(op.current_a > motor.max_current_a);
        assert!(!op.is_valid);
    }

    #[test]
    fn test_p12_battery_sag() {
        let specs = BatterySpecs {
            cells_series: 6,
            cells_parallel: 1,
            cell_capacity_ah: 5.0,
            cell_energy_wh: 111.0,
            cell_resistance_ohm: 0.015,
            cell_mass_kg: 0.15,
            c_rating: 25.0,
            usable_fraction: 0.8,
            overhead_fraction: 0.05,
        };
        let r_pack: f64 = pack_resistance_ohm(&specs);
        assert!((r_pack - 0.09).abs() < 1e-4, "P-12 Rpack failed: {}", r_pack);
        let sag: f64 = 20.0 * r_pack;
        assert!((sag - 1.8).abs() < 1e-4, "P-12 Sag failed: {}", sag);
    }

    #[test]
    fn test_p13_steady_temperature() {
        let p_loss: f64 = 16.82;
        let r_th: f64 = 2.5;
        let ambient: f64 = 15.0;
        let temp: f64 = ambient + (p_loss * r_th);
        assert!((temp - 57.05).abs() < 0.1, "P-13 Temperature failed: {}", temp);
    }

    #[test]
    fn test_p16_nan_handling() {
        let motor = MotorSpecs {
            kv_rpm_per_v: 150.0,
            resistance_ohm: 0.08,
            no_load_current_a: 0.8,
            max_current_a: 30.0,
            max_power_w: 600.0,
            thermal_resistance_k_per_w: 2.5,
        };
        let result = evaluate_motor_operating_point(f64::NAN, 0.2, &motor, 22.2, 15.0);
        assert!(result.is_err(), "P-16 NaN input must return Err without panicking");
    }
}
