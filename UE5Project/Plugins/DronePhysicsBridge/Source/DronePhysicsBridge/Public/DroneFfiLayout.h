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
constexpr std::int32_t kDroneFfiExpectedInterfaceVersion = 9;

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

// DebugTelemetry: 4 vec3 f32 (48) + 3 vec4 f32 (48) + margin f32 (4) + 7 int32 (28) = 128
static_assert(sizeof(DebugTelemetry) == 128, "DebugTelemetry size");
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
