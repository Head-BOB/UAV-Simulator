pub mod aero;
pub mod config;
pub mod integrator;
pub mod mixer;
pub mod structural;
pub mod surrogate;
pub mod thermal;
pub mod types;

use glam::Vec3;
use std::sync::Mutex;
use types::{ControlInputs, DebugTelemetry, DroneState, SurrogateQueryResult, ThermalState};

static TELEMETRY_CACHE: Mutex<DebugTelemetry> = Mutex::new(DebugTelemetry::new());

/// Returns the current layout version to UE5 to prevent memory corruption on mismatch.
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_interface_version() -> i32 {
    5
}

/// Allows UE5 to verify the byte size of DroneState during module initialization.
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_drone_state_size() -> i32 {
    std::mem::size_of::<DroneState>() as i32
}

/// Instantiates a default, zeroed drone state with an identity quaternion.
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_create_default_drone_state() -> DroneState {
    DroneState {
        position: [0.0f64, 0.0, 0.0],
        velocity: [0.0, 0.0, 0.0],
        orientation: [0.0, 0.0, 0.0, 1.0],
        angular_velocity: [0.0, 0.0, 0.0],
    }
}

/// Resets the provided state to default values.
///
/// # Safety
/// * `state` must be a valid, aligned, and mutable pointer to a DroneState instance.
/// * The memory must not be concurrently accessed by another thread.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_reset_drone_state(state: *mut DroneState) -> i32 {
    if state.is_null() {
        return 1;
    }

    unsafe {
        *state = ffi_create_default_drone_state();
    }
    0
}

/// Main execution block for the fixed-timestep RK4 physics pipeline.
///
/// Executes modules in strict order: Aero -> Structural -> Thermal -> Mixer -> Integrator.
///
/// # Units
/// * `dt` - Timestep in seconds.
///
/// # Safety
/// * `state` must be a valid, aligned, mutable pointer to a DroneState.
/// * `thermal_state` must be a valid, aligned, mutable pointer to a ThermalState.
/// * `controls` must be a valid, aligned, immutable pointer to ControlInputs.
/// * `aero_handle` and `fea_handle` may be null. If non-null, must be valid SurrogateHandles.
/// * `config` must be a valid, aligned, immutable pointer to a VehicleConfig.
/// * `out_telemetry` must be a valid, aligned, mutable pointer to a DebugTelemetry struct.
/// * Pointers must not alias or be subject to concurrent mutation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_step_physics(
    state: *mut DroneState,
    thermal_state: *mut ThermalState,
    controls: *const ControlInputs,
    aero_handle: *mut surrogate::SurrogateHandle,
    fea_handle: *mut surrogate::SurrogateHandle,
    config: *const types::VehicleConfig,
    out_telemetry: *mut DebugTelemetry,
    dt: f64,
) -> i32 {
    if state.is_null()
        || thermal_state.is_null()
        || controls.is_null()
        || config.is_null()
        || out_telemetry.is_null()
    {
        return 1;
    }

    if dt <= 0.0 || dt.is_nan() || dt > 0.01 {
        return 7;
    }

    unsafe {
        let original_state = *state;
        let original_thermal = *thermal_state;
        let current_controls = *controls;
        let current_config = *config;

        let aero_opt = if aero_handle.is_null() {
            None
        } else {
            Some(&mut *aero_handle)
        };
        let fea_opt = if fea_handle.is_null() {
            None
        } else {
            Some(&mut *fea_handle)
        };

        let altitude = -original_state.position[2];
        let airspeed = Vec3::from_array(original_state.velocity).length() as f64;

        let (drag, aero_result) =
            match aero::get_drag_with_fallback(&original_state, altitude, aero_opt) {
                Ok(res) => res,
                Err(_) => return 6,
            };

        let throttles = mixer::mix_controls(&current_controls);

        let thrusts = [
            aero::get_thrust(throttles[0] as f32, &current_config).unwrap_or(0.0) as f64,
            aero::get_thrust(throttles[1] as f32, &current_config).unwrap_or(0.0) as f64,
            aero::get_thrust(throttles[2] as f32, &current_config).unwrap_or(0.0) as f64,
            aero::get_thrust(throttles[3] as f32, &current_config).unwrap_or(0.0) as f64,
        ];

        let load_proxy = thrusts.iter().sum::<f64>() as f32 + Vec3::from_array(drag).length();
        let (safety_margin, fea_result) =
            structural::get_safety_margin_with_fallback(load_proxy, fea_opt);

        let motor_rpms = [
            (throttles[0] * current_config.max_rpm),
            (throttles[1] * current_config.max_rpm),
            (throttles[2] * current_config.max_rpm),
            (throttles[3] * current_config.max_rpm),
        ];

        thermal::update_temperatures(&mut *thermal_state, &throttles, &motor_rpms, airspeed, dt);

        let (net_force, net_torque, net_thrust) = mixer::calculate_net_forces(
            thrusts,
            [drag[0] as f64, drag[1] as f64, drag[2] as f64],
            original_state.orientation,
            &current_config,
        );

        let new_state = integrator::step_rk4(
            &original_state,
            [
                net_force[0] as f32,
                net_force[1] as f32,
                net_force[2] as f32,
            ],
            [
                net_torque[0] as f32,
                net_torque[1] as f32,
                net_torque[2] as f32,
            ],
            &current_config,
            dt,
        );

        let has_nan = new_state
            .position
            .iter()
            .any(|x| x.is_nan() || x.is_infinite())
            || new_state
                .velocity
                .iter()
                .any(|x| x.is_nan() || x.is_infinite())
            || new_state
                .orientation
                .iter()
                .any(|x| x.is_nan() || x.is_infinite())
            || new_state
                .angular_velocity
                .iter()
                .any(|x| x.is_nan() || x.is_infinite());

        if has_nan {
            *state = original_state;
            *thermal_state = original_thermal;
            (*out_telemetry).physics_fault = 1;
            return 8;
        }

        *state = new_state;

        let mut all_valid = 1;
        let mut aero_src = 1;
        if let Some(res) = aero_result {
            aero_src = 2;
            if res.in_validated_envelope == 0 {
                all_valid = 0;
            }
        }

        let mut fea_src = 1;
        if let Some(res) = fea_result {
            fea_src = 2;
            if res.in_validated_envelope == 0 {
                all_valid = 0;
            }
        }

        *out_telemetry = DebugTelemetry {
            aero_drag: drag,
            motor_thrusts: [
                thrusts[0] as f32,
                thrusts[1] as f32,
                thrusts[2] as f32,
                thrusts[3] as f32,
            ],
            net_force: [
                net_force[0] as f32,
                net_force[1] as f32,
                net_force[2] as f32,
            ],
            gravity: [0.0, 0.0, (9.80665 * current_config.mass_kg) as f32],
            net_thrust: [
                net_thrust[0] as f32,
                net_thrust[1] as f32,
                net_thrust[2] as f32,
            ],
            motor_temperatures_c: [
                (*thermal_state).motor_temp_c[0] as f32,
                (*thermal_state).motor_temp_c[1] as f32,
                (*thermal_state).motor_temp_c[2] as f32,
                (*thermal_state).motor_temp_c[3] as f32,
            ],
            motor_rpms: [
                motor_rpms[0] as f32,
                motor_rpms[1] as f32,
                motor_rpms[2] as f32,
                motor_rpms[3] as f32,
            ],
            structural_safety_margin: safety_margin,
            is_validated_envelope: all_valid,
            stub_loaded: if cfg!(feature = "allow_stub_models") {
                1
            } else {
                0
            },
            aero_source: aero_src,
            fea_source: fea_src,
            physics_fault: 0,
            dropped_time_events: 0,
            uncalibrated_flags: 0,
        };
    }

    0
}

/// Retrieves the most recent physics telemetry data for the UE5 OSD.
///
/// # Safety
/// * `out_telemetry` must be a valid, aligned, and mutable pointer to a DebugTelemetry struct.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_get_debug_telemetry(out_telemetry: *mut DebugTelemetry) -> i32 {
    if out_telemetry.is_null() {
        return 1;
    }

    if let Ok(telemetry) = TELEMETRY_CACHE.lock() {
        unsafe {
            *out_telemetry = *telemetry;
        }
        return 0;
    }

    2
}

/// Loads a trained ONNX surrogate model and returns an opaque handle to C++.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer provided or Invalid UTF-8 path
/// * `2` - Missing provenance metadata
/// * `3` - Geometry hash mismatch
/// * `4` - Engine/ONNX initialization failure
/// * `5` - Stub model rejected (allow_stub_models feature not enabled)
///
/// # Safety
/// * `path` must be a valid, null-terminated C string.
/// * `out_handle` must be a valid, aligned, mutable pointer to a pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_load_surrogate_model(
    path: *const std::ffi::c_char,
    out_handle: *mut *mut surrogate::SurrogateHandle,
) -> i32 {
    if path.is_null() || out_handle.is_null() {
        return 1;
    }

    let c_str = unsafe { std::ffi::CStr::from_ptr(path) };
    let path_str = match c_str.to_str() {
        Ok(s) => s,
        Err(_) => return 1,
    };

    match surrogate::load_surrogate(path_str) {
        Ok(handle) => {
            unsafe {
                *out_handle = Box::into_raw(Box::new(handle));
            }
            0
        }
        Err(surrogate::SurrogateLoadError::IoError) => 1,
        Err(surrogate::SurrogateLoadError::MissingProvenance) => 2,
        Err(surrogate::SurrogateLoadError::GeometryMismatch { .. }) => 3,
        Err(surrogate::SurrogateLoadError::EngineError(_)) => 4,
        Err(surrogate::SurrogateLoadError::StubModel) => 5,
    }
}

/// Queries a loaded surrogate model.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer or invalid array length provided
/// * `4` - Engine/ONNX inference error
///
/// # Safety
/// * `handle` must be a valid pointer created by `ffi_load_surrogate_model`.
/// * `inputs` must point to an array of exactly `input_count` floats.
/// * `out_result` must be a valid, aligned pointer.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_query_surrogate(
    handle: *mut surrogate::SurrogateHandle,
    inputs: *const f32,
    input_count: i32,
    out_result: *mut SurrogateQueryResult,
) -> i32 {
    if handle.is_null() || inputs.is_null() || out_result.is_null() || input_count < 0 {
        return 1;
    }

    let handle_ref = unsafe { &mut *handle };
    let input_slice = unsafe { std::slice::from_raw_parts(inputs, input_count as usize) };

    match surrogate::query_model(handle_ref, input_slice) {
        Ok(res) => {
            unsafe {
                *out_result = res;
            }
            0
        }
        Err(_) => 4,
    }
}

/// Unloads a surrogate model and frees its memory.
///
/// # Safety
/// * `handle` must be a valid pointer created by `ffi_load_surrogate_model`.
/// * `handle` must not be accessed after this function returns.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_unload_surrogate_model(handle: *mut surrogate::SurrogateHandle) {
    if !handle.is_null() {
        unsafe {
            let _ = Box::from_raw(handle);
        }
    }
}

/// Loads a vehicle configuration file.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer or Invalid UTF-8 path
/// * `2` - File IO Error
/// * `3` - JSON Parse Error
/// * `4` - Invalid Mass
/// * `5` - Invalid Inertia
/// * `6` - Invalid Motor Spin
///
/// # Safety
/// * `path` must be a valid, null-terminated C string.
/// * `out_config` must be a valid, aligned, mutable pointer to a VehicleConfig.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_load_vehicle_config(
    path: *const std::ffi::c_char,
    out_config: *mut types::VehicleConfig,
) -> i32 {
    if path.is_null() || out_config.is_null() {
        return 1;
    }

    let c_str = unsafe { std::ffi::CStr::from_ptr(path) };
    let path_str = match c_str.to_str() {
        Ok(s) => s,
        Err(_) => return 1,
    };

    match config::load_vehicle_config(path_str) {
        Ok(cfg) => {
            unsafe {
                *out_config = cfg;
            }
            0
        }
        Err(config::ConfigError::IoError) => 2,
        Err(config::ConfigError::ParseError) => 3,
        Err(config::ConfigError::InvalidMass) => 4,
        Err(config::ConfigError::InvalidInertia) => 5,
        Err(config::ConfigError::InvalidSpin) => 6,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::VehicleConfig;
    use std::ptr;

    fn dummy_config() -> VehicleConfig {
        VehicleConfig {
            mass_kg: 1.0,
            inertia_kgm2: [1.0, 1.0, 1.0],
            motor_pos_m: [
                [0.2, 0.2, 0.0],
                [0.2, -0.2, 0.0],
                [-0.2, 0.2, 0.0],
                [-0.2, -0.2, 0.0],
            ],
            motor_spin: [1.0, -1.0, -1.0, 1.0],
            prop_diameter_m: 0.25,
            thrust_coeff: 0.11,
            torque_coeff: 0.05,
            max_rpm: 10000.0,
            hover_throttle: 0.5,
        }
    }

    #[test]
    fn test_ffi_null_pointers() {
        let mut state = ffi_create_default_drone_state();
        let mut thermal = ThermalState::new();
        let controls = ControlInputs {
            throttle: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();

        unsafe {
            assert_eq!(
                ffi_step_physics(
                    ptr::null_mut(),
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    0.002
                ),
                1
            );
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    ptr::null_mut(),
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    0.002
                ),
                1
            );
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    ptr::null(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    0.002
                ),
                1
            );
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    ptr::null(),
                    &mut telemetry,
                    0.002
                ),
                1
            );
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    ptr::null_mut(),
                    0.002
                ),
                1
            );
        }
    }

    #[test]
    fn test_ffi_invalid_dt() {
        let mut state = ffi_create_default_drone_state();
        let mut thermal = ThermalState::new();
        let controls = ControlInputs {
            throttle: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();

        unsafe {
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    f64::NAN
                ),
                7
            );
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    0.0
                ),
                7
            );
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    -0.002
                ),
                7
            );
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    0.1
                ),
                7
            );
        }
    }

    #[test]
    fn test_ffi_nan_propagation_catch() {
        let mut state = ffi_create_default_drone_state();
        state.position[0] = f64::NAN;

        let mut thermal = ThermalState::new();
        let controls = ControlInputs {
            throttle: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();

        unsafe {
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    0.002
                ),
                8
            );
            assert_eq!(telemetry.physics_fault, 1);
        }
    }
}
