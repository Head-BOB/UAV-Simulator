pub mod aero;
pub mod integrator;
pub mod mixer;
pub mod types;

use std::sync::Mutex;
use types::{ControlInputs, DebugTelemetry, DroneState};

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
pub extern "C" fn ffi_reset_drone_state(state: *mut DroneState) -> i32 {
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
/// # Units
/// * `dt` - Timestep in seconds.
///
/// # Safety
/// * `state` must be a valid, aligned, mutable pointer to a DroneState.
/// * `controls` must be a valid, aligned, immutable pointer to ControlInputs.
/// * Pointers must not alias or be subject to concurrent mutation.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_step_physics(
    state: *mut DroneState,
    controls: *const ControlInputs,
    dt: f32,
) -> i32 {
    if state.is_null() || controls.is_null() {
        return 1;
    }

    unsafe {
        let current_state = *state;
        let current_controls = *controls;

        let altitude = (-current_state.position[2]) as f32;
        let drag = aero::get_drag(&current_state, altitude);

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
        }
    }

    0
}

/// Retrieves the most recent physics telemetry data for the UE5 OSD.
///
/// # Safety
/// * `out_telemetry` must be a valid, aligned, and mutable pointer to a DebugTelemetry struct.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_debug_telemetry(out_telemetry: *mut DebugTelemetry) -> i32 {
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