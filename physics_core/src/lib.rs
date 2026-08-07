pub mod aero;
pub mod integrator;
pub mod mixer;
pub mod types;

use std::sync::Mutex;
use types::{ControlInputs, DebugTelemetry, DroneState};

/// Thread-safe telemetry cache. Updated every physics tick and polled by the UE5 rendering thread.
static TELEMETRY_CACHE: Mutex<DebugTelemetry> = Mutex::new(DebugTelemetry::new());

/// ffi_get_interface_version
/// Returns the current layout version to UE5 to prevent memory corruption on mismatch.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_interface_version() -> i32 {
    1
}

/// ffi_get_drone_state_size
/// Allows UE5 to verify the byte size of DroneState during module initialization.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_drone_state_size() -> i32 {
    std::mem::size_of::<DroneState>() as i32
}

/// ffi_create_default_drone_state
#[unsafe(no_mangle)]
pub extern "C" fn ffi_create_default_drone_state() -> DroneState {
    DroneState {
        position: [0.0, 0.0, 0.0],
        velocity: [0.0, 0.0, 0.0],
        orientation: [1.0, 0.0, 0.0, 0.0],
        angular_velocity: [0.0, 0.0, 0.0],
    }
}

/// ffi_reset_drone_state
#[unsafe(no_mangle)]
pub extern "C" fn ffi_reset_drone_state(state: *mut DroneState) -> i32 {
    if state.is_null() {
        return 1;
    }

    // SAFETY: Null check performed above. Trusting UE5 wrapper to provide a valid pointer.
    unsafe {
        *state = ffi_create_default_drone_state();
    }
    0
}

/// ffi_step_physics
/// Main execution block for the fixed-timestep RK4 physics pipeline.
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

        let drag = aero::get_drag(&current_state, 0.0);

        let thrust = aero::get_thrust(current_controls.throttle);
        let thrusts = [thrust, thrust, thrust, thrust];

        // 3. Call Mixer Module (Passing orientation and a placeholder mass of 1.0)
        // 3. Call Mixer Module
        let (net_force, net_torque, net_thrust) =
            mixer::calculate_net_forces(thrusts, drag, current_state.orientation, 1.0);

        let new_state = integrator::step_rk4(&current_state, net_force, net_torque, 1.0, 1.0, dt);

        *state = new_state;

        // Synchronize telemetry data for the OSD module
        if let Ok(mut telemetry) = TELEMETRY_CACHE.lock() {
            telemetry.aero_drag = drag;
            telemetry.motor_thrusts = thrusts;
            telemetry.net_force = net_force;
            telemetry.gravity = [0.0, 0.0, -9.81];

            telemetry.net_thrust = net_thrust;
        }
    }

    0
}

/// ffi_get_debug_telemetry
/// Retrieves the most recent physics telemetry data for the UE5 OSD.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_debug_telemetry(out_telemetry: *mut DebugTelemetry) -> i32 {
    if out_telemetry.is_null() {
        return 1;
    }

    if let Ok(telemetry) = TELEMETRY_CACHE.lock() {
        // SAFETY: Null check performed above.
        unsafe {
            *out_telemetry = *telemetry;
        }
        return 0;
    }

    2 // Mutex lock poisoned
}
