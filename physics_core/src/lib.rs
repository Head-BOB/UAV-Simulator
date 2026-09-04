pub mod aero;
pub mod integrator;
pub mod mixer;
pub mod surrogate;
pub mod types;

use std::sync::Mutex;
use types::{ControlInputs, DebugTelemetry, DroneState, SurrogateQueryResult};

static TELEMETRY_CACHE: Mutex<DebugTelemetry> = Mutex::new(DebugTelemetry::new());

/// Returns the current layout version to UE5 to prevent memory corruption on mismatch.
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_interface_version() -> i32 {
    1
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

    // SAFETY: Verified null check. Callers must uphold standard borrowing rules.
    unsafe {
        *state = ffi_create_default_drone_state();
    }
    0
}

/// Main execution block for the fixed-timestep RK4 physics pipeline.
///
/// # Units
/// * `dt` - Timestep in seconds.
///
/// # Safety
/// * `state` must be a valid, aligned, mutable pointer to a DroneState.
/// * `controls` must be a valid, aligned, immutable pointer to ControlInputs.
/// * `aero_handle` may be null. If non-null, it must be a valid pointer to a SurrogateHandle.
/// * Pointers must not alias or be subject to concurrent mutation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_step_physics(
    state: *mut DroneState,
    controls: *const ControlInputs,
    aero_handle: *mut surrogate::SurrogateHandle,
    dt: f32,
) -> i32 {
    if state.is_null() || controls.is_null() {
        return 1;
    }

    // SAFETY: Verified null checks.
    unsafe {
        let current_state = *state;
        let current_controls = *controls;

        let aero_opt = if aero_handle.is_null() {
            None
        } else {
            Some(&mut *aero_handle)
        };

        let altitude = (-current_state.position[2]) as f32;
        let (drag, aero_result) = aero::get_drag_with_fallback(&current_state, altitude, aero_opt);

        let t = current_controls.throttle;
        let r = current_controls.roll * 0.2;
        let p = current_controls.pitch * 0.2;
        let y = current_controls.yaw * 0.2;

        let thrusts = [
            aero::get_thrust((t - p + r - y).clamp(0.0, 1.0)),
            aero::get_thrust((t - p - r + y).clamp(0.0, 1.0)),
            aero::get_thrust((t + p + r + y).clamp(0.0, 1.0)),
            aero::get_thrust((t + p - r - y).clamp(0.0, 1.0)),
        ];

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

            if let Some(res) = aero_result {
                telemetry.is_validated_envelope = res.in_validated_envelope;
            }
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
        // SAFETY: Pointer is null-checked.
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

    // SAFETY: Caller guarantees path is null-terminated per # Safety contract.
    let c_str = unsafe { std::ffi::CStr::from_ptr(path) };
    let path_str = match c_str.to_str() {
        Ok(s) => s,
        Err(_) => return 1,
    };

    match surrogate::load_surrogate(path_str) {
        Ok(handle) => {
            // SAFETY: out_handle null check validated above.
            unsafe {
                *out_handle = Box::into_raw(Box::new(handle));
            }
            0
        }
        Err(surrogate::SurrogateLoadError::IoError) => 1,
        Err(surrogate::SurrogateLoadError::MissingProvenance) => 2,
        Err(surrogate::SurrogateLoadError::GeometryMismatch { .. }) => 3,
        Err(surrogate::SurrogateLoadError::EngineError(_)) => 4,
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

    // SAFETY: Caller guarantees pointers and array length.
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
        // SAFETY: Takes ownership of the raw pointer and drops it, freeing memory.
        unsafe {
            let _ = Box::from_raw(handle);
        }
    }
}
