// Declare our modules so Rust knows they exist
pub mod types;
pub mod integrator;
pub mod aero;
pub mod mixer;

use types::{DroneState, ControlInputs};

/// ffi_get_interface_version
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_interface_version() -> i32 {
    1 // Bumping this if the struct changes prevents silent crashes in C++
}

/// ffi_get_drone_state_size
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
        orientation: [1.0, 0.0, 0.0, 0.0], // Identity quaternion (no rotation)
        angular_velocity: [0.0, 0.0, 0.0],
    }
}

/// ffi_reset_drone_state
#[unsafe(no_mangle)]
pub extern "C" fn ffi_reset_drone_state(state: *mut DroneState) -> i32 {
    if state.is_null() {
        return 1; // Error code: Pointer was null
    }

    // SAFETY: We checked for null. We are trusting C++ to give us a valid pointer.
    unsafe {
        *state = ffi_create_default_drone_state();
    }
    0 // Success
}

/// ffi_step_physics
#[unsafe(no_mangle)]
pub extern "C" fn ffi_step_physics(state: *mut DroneState, controls: *const ControlInputs, dt: f32) -> i32 {
    if state.is_null() || controls.is_null() {
        return 1; // Error
    }

    unsafe {
        // 1. Read current state and inputs out of memory
        let current_state = *state;
        let current_controls = *controls;

        // 2. Call Aero Module
        let drag = aero::get_drag(&current_state, 0.0); // 0.0 altitude placeholder
        // (Assuming 4 identical throttles for a basic quadcopter placeholder)
        let thrust = aero::get_thrust(current_controls.throttle);
        let thrusts = [thrust, thrust, thrust, thrust];

        // 3. Call Mixer Module (Gravity placeholder 9.81)
        let (net_force, net_torque) = mixer::calculate_net_forces(thrusts, drag, 9.81);

        // 4. Call Integrator (Placeholder mass 1.0, inertia 1.0)
        let new_state = integrator::step_rk4(&current_state, net_force, net_torque, 1.0, 1.0, dt);

        // 5. Write the new state back into the C++ memory pointer
        *state = new_state;
    }

    0 // Success
}