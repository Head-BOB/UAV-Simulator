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
use types::{ControlInputs, DebugTelemetry, DroneState, SurrogateQueryResult};

static TELEMETRY_CACHE: Mutex<DebugTelemetry> = Mutex::new(DebugTelemetry::new());

/// Returns the current layout version to UE5 to prevent memory corruption on mismatch.
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_interface_version() -> i32 {
    3
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
/// * `controls` must be a valid, aligned, immutable pointer to ControlInputs.
/// * `aero_handle` and `fea_handle` may be null. If non-null, must be valid SurrogateHandles.
/// * Pointers must not alias or be subject to concurrent mutation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_step_physics(
    state: *mut DroneState,
    controls: *const ControlInputs,
    aero_handle: *mut surrogate::SurrogateHandle,
    fea_handle: *mut surrogate::SurrogateHandle,
    dt: f64, // <-- FIX: Changed from f32 to f64
) -> i32 {
    if state.is_null() || controls.is_null() {
        return 1;
    }

    unsafe {
        let current_state = *state;
        let current_controls = *controls;

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

        let altitude = -current_state.position[2]; // Now natively f64

        // FIX: Catch Altitude Out of Bounds Error (Returns code 6)
        let (drag, aero_result) =
            match aero::get_drag_with_fallback(&current_state, altitude, aero_opt) {
                Ok(res) => res,
                Err(_) => return 6,
            };

        let airspeed = Vec3::from_array(current_state.velocity).length();

        let t = current_controls.throttle;
        let r = current_controls.roll * 0.2;
        let p = current_controls.pitch * 0.2;
        let y = current_controls.yaw * 0.2;

        let thrusts = [
            aero::get_thrust((t - p + r - y).clamp(0.0, 1.0)).unwrap_or(0.0),
            aero::get_thrust((t - p - r + y).clamp(0.0, 1.0)).unwrap_or(0.0),
            aero::get_thrust((t + p + r + y).clamp(0.0, 1.0)).unwrap_or(0.0),
            aero::get_thrust((t + p - r - y).clamp(0.0, 1.0)).unwrap_or(0.0),
        ];

        let load_proxy = thrusts.iter().sum::<f32>() + Vec3::from_array(drag).length();
        let (safety_margin, fea_result) =
            structural::get_safety_margin_with_fallback(load_proxy, fea_opt);

        let current_temps = if let Ok(telemetry) = TELEMETRY_CACHE.lock() {
            telemetry.motor_temperatures_c
        } else {
            [20.0; 4]
        };

        let new_temps = thermal::update_temperatures(&thrusts, &current_temps, airspeed, dt as f32);

        let (net_force, net_torque, net_thrust) =
            mixer::calculate_net_forces(thrusts, drag, current_state.orientation, 1.0);

        let new_state = integrator::step_rk4(&current_state, net_force, net_torque, 1.0, 1.0, dt);

        *state = new_state;

        if let Ok(mut telemetry) = TELEMETRY_CACHE.lock() {
            telemetry.aero_drag = drag;
            telemetry.motor_thrusts = thrusts;
            telemetry.net_force = net_force;
            telemetry.gravity = [0.0, 0.0, 9.80665];
            telemetry.net_thrust = net_thrust;
            telemetry.motor_temperatures_c = new_temps;
            telemetry.structural_safety_margin = safety_margin;

            let mut all_valid = 1;
            if let Some(res) = aero_result
                && res.in_validated_envelope == 0
            {
                all_valid = 0;
            }
            if let Some(res) = fea_result
                && res.in_validated_envelope == 0
            {
                all_valid = 0;
            }
            telemetry.is_validated_envelope = all_valid;
            telemetry.stub_loaded = if cfg!(feature = "allow_stub_models") {
                1
            } else {
                0
            };
        }
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
