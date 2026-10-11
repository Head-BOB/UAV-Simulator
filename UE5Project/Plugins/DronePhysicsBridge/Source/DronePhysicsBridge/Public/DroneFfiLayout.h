// Tier: F
#pragma once

// Compile-time FFI contract checks for the Rust physics core (WP-4.5, WP-4.6, ADR-001 Decision 7).
//
// UE-free on purpose: includes only <cstddef>, <cstdint> and drone_ffi.h, so CI can check it with
// g++ -std=c++17 -fsyntax-only (tools/cpp_ffi_check/ffi_layout_check.cpp) without Unreal.
//
// Every number below must match physics_core/src/types.rs (the `const _` block and the layout
// tests). Rules: field at the next multiple of its alignment; size rounded up to struct alignment;
// double = 8 B (align 8); float and int32_t = 4 B (align 4); arrays: n * element size.
//
// Any change to a repr(C) struct needs: user approval, a regenerated drone_ffi.h, an interface
// version bump (Rust ffi_get_interface_version AND the constant below), and updates here.

#include <cstddef>
#include <cstdint>

#include "Generated/drone_ffi.h"

// Must equal Rust ffi_get_interface_version(). Checked by the Rust test
// types::tests::test_cpp_expected_version_matches_rust, which parses the next line, so keep its form.
constexpr std::int32_t kDroneFfiExpectedInterfaceVersion = 14;

// DroneState: 3*8 + 3*4 + 4*4 + 3*4 = 64
static_assert(sizeof(DroneState) == 64, "DroneState size");
static_assert(alignof(DroneState) == 8, "DroneState align");
static_assert(offsetof(DroneState, position) == 0, "DroneState.position");
static_assert(offsetof(DroneState, velocity) == 24, "DroneState.velocity");
static_assert(offsetof(DroneState, orientation) == 36, "DroneState.orientation");
static_assert(offsetof(DroneState, angular_velocity) == 52, "DroneState.angular_velocity");

// ControlInputs: 4*4 = 16
static_assert(sizeof(ControlInputs) == 16, "ControlInputs size");
static_assert(alignof(ControlInputs) == 4, "ControlInputs align");
static_assert(offsetof(ControlInputs, throttle) == 0, "ControlInputs.throttle");
static_assert(offsetof(ControlInputs, roll) == 4, "ControlInputs.roll");
static_assert(offsetof(ControlInputs, pitch) == 8, "ControlInputs.pitch");
static_assert(offsetof(ControlInputs, yaw) == 12, "ControlInputs.yaw");

// WindInput (interface v11): 3*8 = 24
static_assert(sizeof(WindInput) == 24, "WindInput size");
static_assert(alignof(WindInput) == 8, "WindInput align");
static_assert(offsetof(WindInput, wind_ned_mps) == 0, "WindInput.wind_ned_mps");

// EnvironmentInput (interface v12): 2*8 = 16
static_assert(sizeof(EnvironmentInput) == 16, "EnvironmentInput size");
static_assert(alignof(EnvironmentInput) == 8, "EnvironmentInput align");
static_assert(offsetof(EnvironmentInput, site_altitude_m) == 0, "EnvironmentInput.site_altitude_m");
static_assert(offsetof(EnvironmentInput, temp_offset_c) == 8, "EnvironmentInput.temp_offset_c");

// ThermalState: 4*8 = 32
static_assert(sizeof(ThermalState) == 32, "ThermalState size");
static_assert(alignof(ThermalState) == 8, "ThermalState align");
static_assert(offsetof(ThermalState, motor_temp_c) == 0, "ThermalState.motor_temp_c");

// FaultLatch: 4
static_assert(sizeof(FaultLatch) == 4, "FaultLatch size");
static_assert(alignof(FaultLatch) == 4, "FaultLatch align");
static_assert(offsetof(FaultLatch, physics_fault) == 0, "FaultLatch.physics_fault");

// SurrogateQueryResult: 3*4 + 4 + 4 = 20
static_assert(sizeof(SurrogateQueryResult) == 20, "SurrogateQueryResult size");
static_assert(alignof(SurrogateQueryResult) == 4, "SurrogateQueryResult align");
static_assert(offsetof(SurrogateQueryResult, predicted_values) == 0, "SurrogateQueryResult.predicted_values");
static_assert(offsetof(SurrogateQueryResult, uncertainty) == 12, "SurrogateQueryResult.uncertainty");
static_assert(offsetof(SurrogateQueryResult, in_validated_envelope) == 16, "SurrogateQueryResult.in_validated_envelope");

// VehicleConfig: 8 + 24 + 96 + 32 + 5*8 = 200
static_assert(sizeof(VehicleConfig) == 200, "VehicleConfig size");
static_assert(alignof(VehicleConfig) == 8, "VehicleConfig align");
static_assert(offsetof(VehicleConfig, mass_kg) == 0, "VehicleConfig.mass_kg");
static_assert(offsetof(VehicleConfig, inertia_kgm2) == 8, "VehicleConfig.inertia_kgm2");
static_assert(offsetof(VehicleConfig, motor_pos_m) == 32, "VehicleConfig.motor_pos_m");
static_assert(offsetof(VehicleConfig, motor_spin) == 128, "VehicleConfig.motor_spin");
static_assert(offsetof(VehicleConfig, prop_diameter_m) == 160, "VehicleConfig.prop_diameter_m");
static_assert(offsetof(VehicleConfig, thrust_coeff) == 168, "VehicleConfig.thrust_coeff");
static_assert(offsetof(VehicleConfig, torque_coeff) == 176, "VehicleConfig.torque_coeff");
static_assert(offsetof(VehicleConfig, max_rpm) == 184, "VehicleConfig.max_rpm");
static_assert(offsetof(VehicleConfig, hover_throttle) == 192, "VehicleConfig.hover_throttle");

// DebugTelemetry: 4 vec3 f32 (48) + 3 vec4 f32 (48) + margin f32 (4) + 9 int32 (36) = 136
static_assert(sizeof(DebugTelemetry) == 136, "DebugTelemetry size");
static_assert(alignof(DebugTelemetry) == 4, "DebugTelemetry align");
static_assert(offsetof(DebugTelemetry, net_thrust) == 0, "DebugTelemetry.net_thrust");
static_assert(offsetof(DebugTelemetry, aero_drag) == 12, "DebugTelemetry.aero_drag");
static_assert(offsetof(DebugTelemetry, gravity) == 24, "DebugTelemetry.gravity");
static_assert(offsetof(DebugTelemetry, net_force) == 36, "DebugTelemetry.net_force");
static_assert(offsetof(DebugTelemetry, motor_thrusts) == 48, "DebugTelemetry.motor_thrusts");
static_assert(offsetof(DebugTelemetry, motor_rpms) == 64, "DebugTelemetry.motor_rpms");
static_assert(offsetof(DebugTelemetry, motor_temperatures_c) == 80, "DebugTelemetry.motor_temperatures_c");
static_assert(offsetof(DebugTelemetry, structural_safety_margin) == 96, "DebugTelemetry.structural_safety_margin");
static_assert(offsetof(DebugTelemetry, is_validated_envelope) == 100, "DebugTelemetry.is_validated_envelope");
static_assert(offsetof(DebugTelemetry, stub_loaded) == 104, "DebugTelemetry.stub_loaded");
static_assert(offsetof(DebugTelemetry, aero_source) == 108, "DebugTelemetry.aero_source");
static_assert(offsetof(DebugTelemetry, fea_source) == 112, "DebugTelemetry.fea_source");
static_assert(offsetof(DebugTelemetry, physics_fault) == 116, "DebugTelemetry.physics_fault");
static_assert(offsetof(DebugTelemetry, dropped_time_events) == 120, "DebugTelemetry.dropped_time_events");
static_assert(offsetof(DebugTelemetry, uncalibrated_flags) == 124, "DebugTelemetry.uncalibrated_flags");
static_assert(offsetof(DebugTelemetry, thermal_valid) == 128, "DebugTelemetry.thermal_valid");
static_assert(offsetof(DebugTelemetry, thermal_over_temp) == 132, "DebugTelemetry.thermal_over_temp");

// Interface v13 (powertrain) and v14 (propeller table). Offsets mirror the Rust tests types::tests::test_powertrain_struct_layouts.
// PowertrainConfig: 25 double (200) + 2 x 16 double tables (256) + 4 int32 (16) = 472, then (v14) 8 double rpm (64) + 3 tables of 8 x 48 double (9216)
// + 8 int32 point counts (32) + 2 int32 = 9792
static_assert(sizeof(PowertrainConfig) == 9792, "PowertrainConfig size");
static_assert(alignof(PowertrainConfig) == 8, "PowertrainConfig align");
static_assert(offsetof(PowertrainConfig, motor_kv_rpm_per_v) == 0, "PowertrainConfig.motor_kv_rpm_per_v");
static_assert(offsetof(PowertrainConfig, prop_diameter_m) == 40, "PowertrainConfig.prop_diameter_m");
static_assert(offsetof(PowertrainConfig, esc_efficiency) == 64, "PowertrainConfig.esc_efficiency");
static_assert(offsetof(PowertrainConfig, battery_capacity_ah) == 88, "PowertrainConfig.battery_capacity_ah");
static_assert(offsetof(PowertrainConfig, motor_r_winding_case_k_per_w) == 128, "PowertrainConfig.motor_r_winding_case_k_per_w");
static_assert(offsetof(PowertrainConfig, cooling_ref_airspeed_mps) == 192, "PowertrainConfig.cooling_ref_airspeed_mps");
static_assert(offsetof(PowertrainConfig, ocv_soc) == 200, "PowertrainConfig.ocv_soc");
static_assert(offsetof(PowertrainConfig, ocv_v_per_cell) == 328, "PowertrainConfig.ocv_v_per_cell");
static_assert(offsetof(PowertrainConfig, battery_cells) == 456, "PowertrainConfig.battery_cells");
static_assert(offsetof(PowertrainConfig, ocv_points) == 460, "PowertrainConfig.ocv_points");
static_assert(offsetof(PowertrainConfig, parameters_verified) == 464, "PowertrainConfig.parameters_verified");
static_assert(offsetof(PowertrainConfig, reserved) == 468, "PowertrainConfig.reserved");
static_assert(offsetof(PowertrainConfig, prop_curve_rpm) == 472, "PowertrainConfig.prop_curve_rpm");
static_assert(offsetof(PowertrainConfig, prop_j) == 536, "PowertrainConfig.prop_j");
static_assert(offsetof(PowertrainConfig, prop_ct_table) == 3608, "PowertrainConfig.prop_ct_table");
static_assert(offsetof(PowertrainConfig, prop_cp_table) == 6680, "PowertrainConfig.prop_cp_table");
static_assert(offsetof(PowertrainConfig, prop_curve_points) == 9752, "PowertrainConfig.prop_curve_points");
static_assert(offsetof(PowertrainConfig, prop_curve_count) == 9784, "PowertrainConfig.prop_curve_count");
static_assert(offsetof(PowertrainConfig, prop_model) == 9788, "PowertrainConfig.prop_model");

// PowertrainState: 5 double (40) + 3 x 4 double (96) + 1 double = 144
static_assert(sizeof(PowertrainState) == 144, "PowertrainState size");
static_assert(alignof(PowertrainState) == 8, "PowertrainState align");
static_assert(offsetof(PowertrainState, soc) == 0, "PowertrainState.soc");
static_assert(offsetof(PowertrainState, battery_current_a) == 16, "PowertrainState.battery_current_a");
static_assert(offsetof(PowertrainState, time_below_cutoff_s) == 32, "PowertrainState.time_below_cutoff_s");
static_assert(offsetof(PowertrainState, motor_winding_c) == 40, "PowertrainState.motor_winding_c");
static_assert(offsetof(PowertrainState, motor_case_c) == 72, "PowertrainState.motor_case_c");
static_assert(offsetof(PowertrainState, esc_temp_c) == 104, "PowertrainState.esc_temp_c");
static_assert(offsetof(PowertrainState, prop_extrapolated_time_s) == 136, "PowertrainState.prop_extrapolated_time_s");

// PowertrainTelemetry: 11 x 4 double (352) + 9 double (72) + 14 int32 (56) + 3 x 4 double (96) = 576
static_assert(sizeof(PowertrainTelemetry) == 576, "PowertrainTelemetry size");
static_assert(alignof(PowertrainTelemetry) == 8, "PowertrainTelemetry align");
static_assert(offsetof(PowertrainTelemetry, motor_current_a) == 0, "PowertrainTelemetry.motor_current_a");
static_assert(offsetof(PowertrainTelemetry, motor_rpm) == 64, "PowertrainTelemetry.motor_rpm");
static_assert(offsetof(PowertrainTelemetry, motor_thrust_n) == 128, "PowertrainTelemetry.motor_thrust_n");
static_assert(offsetof(PowertrainTelemetry, esc_temp_c) == 320, "PowertrainTelemetry.esc_temp_c");
static_assert(offsetof(PowertrainTelemetry, battery_voltage_v) == 352, "PowertrainTelemetry.battery_voltage_v");
static_assert(offsetof(PowertrainTelemetry, battery_soc) == 384, "PowertrainTelemetry.battery_soc");
static_assert(offsetof(PowertrainTelemetry, time_below_cutoff_s) == 416, "PowertrainTelemetry.time_below_cutoff_s");
static_assert(offsetof(PowertrainTelemetry, step_valid) == 424, "PowertrainTelemetry.step_valid");
static_assert(offsetof(PowertrainTelemetry, motor_over_current_mask) == 444, "PowertrainTelemetry.motor_over_current_mask");
static_assert(offsetof(PowertrainTelemetry, thermal_valid) == 460, "PowertrainTelemetry.thermal_valid");
static_assert(offsetof(PowertrainTelemetry, prop_extrapolated) == 464, "PowertrainTelemetry.prop_extrapolated");
static_assert(offsetof(PowertrainTelemetry, reserved2) == 476, "PowertrainTelemetry.reserved2");
static_assert(offsetof(PowertrainTelemetry, motor_advance_ratio) == 480, "PowertrainTelemetry.motor_advance_ratio");
static_assert(offsetof(PowertrainTelemetry, motor_ct) == 512, "PowertrainTelemetry.motor_ct");
static_assert(offsetof(PowertrainTelemetry, motor_cp) == 544, "PowertrainTelemetry.motor_cp");

// PowertrainBenchOut: 15 double (120) + 2 int32 = 128
static_assert(sizeof(PowertrainBenchOut) == 128, "PowertrainBenchOut size");
static_assert(alignof(PowertrainBenchOut) == 8, "PowertrainBenchOut align");
static_assert(offsetof(PowertrainBenchOut, rpm) == 0, "PowertrainBenchOut.rpm");
static_assert(offsetof(PowertrainBenchOut, battery_current_a) == 72, "PowertrainBenchOut.battery_current_a");
static_assert(offsetof(PowertrainBenchOut, system_efficiency) == 88, "PowertrainBenchOut.system_efficiency");
static_assert(offsetof(PowertrainBenchOut, advance_ratio) == 96, "PowertrainBenchOut.advance_ratio");
static_assert(offsetof(PowertrainBenchOut, ct) == 104, "PowertrainBenchOut.ct");
static_assert(offsetof(PowertrainBenchOut, cp) == 112, "PowertrainBenchOut.cp");
static_assert(offsetof(PowertrainBenchOut, extrapolated) == 120, "PowertrainBenchOut.extrapolated");

// PropCoefficients: 2 double + 2 int32 = 24
static_assert(sizeof(PropCoefficients) == 24, "PropCoefficients size");
static_assert(alignof(PropCoefficients) == 8, "PropCoefficients align");
static_assert(offsetof(PropCoefficients, cp) == 8, "PropCoefficients.cp");
static_assert(offsetof(PropCoefficients, extrapolated) == 16, "PropCoefficients.extrapolated");
