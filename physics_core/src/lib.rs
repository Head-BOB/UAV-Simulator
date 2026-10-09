// Tier: F
pub mod aero;
pub mod config;
pub mod impact;
pub mod integrator;
pub mod mixer;
#[cfg(feature = "onnx")]
pub mod structural;
#[cfg(feature = "onnx")]
pub mod surrogate;
pub mod thermal;
pub mod types;

use std::collections::BTreeSet;
use std::sync::Mutex;
#[cfg(feature = "onnx")]
use surrogate::fea_surrogate::fea_inputs;
#[cfg(feature = "onnx")]
use types::{ControlInputs, DebugTelemetry, SurrogateQueryResult, ThermalState};
use types::{DroneState, FaultLatch};

// Without the `onnx` feature the FFI entry points that use surrogate handles are compiled out, so
// the pure helpers below are only reached by their tests. They stay compiled so miri runs them.

/// `stub_loaded` telemetry value: 1 if either surrogate handle came from a stub model.
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn stub_loaded_flag(aero_is_stub: bool, fea_is_stub: bool) -> i32 {
    i32::from(aero_is_stub || fea_is_stub)
}

/// Telemetry copy of the caller's latch: any nonzero value reads as 1.
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn latched_flag(latch: &FaultLatch) -> i32 {
    i32::from(latch.physics_fault != 0)
}

/// Addresses of every surrogate handle created by `ffi_load_surrogate_model` and not yet
/// unloaded. Every FFI entry point that receives a handle checks it here BEFORE
/// dereferencing, so a stale, foreign or double-unloaded pointer returns an error code
/// instead of undefined behaviour. The lock is held for the whole query / step so a
/// concurrent unload cannot free a handle that is in use.
///
/// Limit: if a freed address is reused by a later load, the old pointer looks live again.
/// The caller must still null its pointer after unload.
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
static LIVE_HANDLES: Mutex<BTreeSet<usize>> = Mutex::new(BTreeSet::new());

/// Records a newly created handle address as live.
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn register_in(live: &mut BTreeSet<usize>, addr: usize) {
    live.insert(addr);
}

/// Removes a handle address. Returns false if it was not live (unknown or already unloaded).
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn unregister_from(live: &mut BTreeSet<usize>, addr: usize) -> bool {
    live.remove(&addr)
}

/// True when the step's two handle addresses are usable: 0 (null) is allowed, a non-null
/// address must be live, and the two must differ (the step takes `&mut` to both, so equal
/// non-null addresses would alias).
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn step_handles_ok(aero_addr: usize, fea_addr: usize, live: &BTreeSet<usize>) -> bool {
    let live_or_null = |a: usize| a == 0 || live.contains(&a);
    live_or_null(aero_addr) && live_or_null(fea_addr) && (aero_addr == 0 || aero_addr != fea_addr)
}

/// Per-motor thrust in newtons at `altitude_m`. Any aero error is returned, never
/// replaced by zero thrust (a silent 0 N would look like a valid motor-out result).
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn motor_thrusts_n(
    throttles: [f64; 4],
    config: &types::VehicleConfig,
    altitude_m: f64,
) -> Result<[f64; 4], aero::AeroError> {
    Ok([
        aero::get_thrust(throttles[0], config, altitude_m)?,
        aero::get_thrust(throttles[1], config, altitude_m)?,
        aero::get_thrust(throttles[2], config, altitude_m)?,
        aero::get_thrust(throttles[3], config, altitude_m)?,
    ])
}

/// Airspeed magnitude in m/s, computed in f64. The f32 components are widened
/// exactly (f32 -> f64 is lossless); only the FFI struct is f32 (ADR-001 D2).
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn airspeed_mps(velocity_mps: [f32; 3]) -> f64 {
    glam::DVec3::new(
        f64::from(velocity_mps[0]),
        f64::from(velocity_mps[1]),
        f64::from(velocity_mps[2]),
    )
    .length()
}

/// Aggregated envelope flag for one tick: 0 if any queried surrogate reported "outside the
/// validated envelope", else 1. A surrogate that was not queried does not lower the flag.
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn envelope_flag(
    aero: Option<&types::SurrogateQueryResult>,
    fea: Option<&types::SurrogateQueryResult>,
) -> i32 {
    let outside = |r: Option<&types::SurrogateQueryResult>| {
        r.is_some_and(|res| res.in_validated_envelope == 0)
    };
    if outside(aero) || outside(fea) { 0 } else { 1 }
}

/// Returns the current layout version to UE5 to prevent memory corruption on mismatch.
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_interface_version() -> i32 {
    9
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
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer
/// * `6` - Aero model error (drag or thrust), including altitude outside ISA 0..=11000 m;
///   no output is written
/// * `7` - Invalid dt (NaN, <= 0 or > 0.01)
/// * `8` - NaN/Inf in the new state: `state` and `thermal_state` are restored to their
///   previous values and `fault_latch` is set to 1.
/// * `9` - A non-null surrogate handle is not live (never loaded, already unloaded, foreign),
///   or `aero_handle` and `fea_handle` are the same non-null handle; nothing is written
///
/// # Fault latch
/// `fault_latch` is caller-owned and must be zero-initialized by the caller. Rust sets it to 1
/// on a code-8 step and never clears it; later steps keep running from the restored state and
/// report `physics_fault = 1` in telemetry. The caller clears it on purpose (for example at
/// the start of a new run). Data produced while it is set is invalid and must not be used as
/// validation or pass/fail evidence.
///
/// # Safety
/// * `state` must be a valid, aligned, mutable pointer to a DroneState.
/// * `thermal_state` must be a valid, aligned, mutable pointer to a ThermalState.
/// * `controls` must be a valid, aligned, immutable pointer to ControlInputs.
/// * `aero_handle` and `fea_handle` may be null. If non-null, must be valid SurrogateHandles.
/// * `config` must be a valid, aligned, immutable pointer to a VehicleConfig.
/// * `out_telemetry` must be a valid, aligned, mutable pointer to a DebugTelemetry struct.
/// * `fault_latch` must be a valid, aligned, mutable pointer to a FaultLatch.
/// * Pointers must not alias or be subject to concurrent mutation.
#[cfg(feature = "onnx")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_step_physics(
    state: *mut DroneState,
    thermal_state: *mut ThermalState,
    controls: *const ControlInputs,
    aero_handle: *mut surrogate::SurrogateHandle,
    fea_handle: *mut surrogate::SurrogateHandle,
    config: *const types::VehicleConfig,
    out_telemetry: *mut DebugTelemetry,
    fault_latch: *mut FaultLatch,
    dt: f64,
) -> i32 {
    if state.is_null()
        || thermal_state.is_null()
        || controls.is_null()
        || config.is_null()
        || out_telemetry.is_null()
        || fault_latch.is_null()
    {
        return 1;
    }

    if dt <= 0.0 || dt.is_nan() || dt > 0.01 {
        return 7;
    }

    // Held until return so an unload on another thread cannot free a handle mid-step.
    let live_guard = if aero_handle.is_null() && fea_handle.is_null() {
        None
    } else {
        match LIVE_HANDLES.lock() {
            Ok(g) => Some(g),
            Err(_) => return 9,
        }
    };
    if let Some(live) = &live_guard
        && !step_handles_ok(aero_handle.addr(), fea_handle.addr(), live)
    {
        return 9;
    }

    unsafe {
        let original_state = *state;
        let original_thermal = *thermal_state;
        let current_controls = *controls;
        let current_config = *config;

        let aero_is_stub = !aero_handle.is_null() && (*aero_handle).is_stub;
        let fea_is_stub = !fea_handle.is_null() && (*fea_handle).is_stub;
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
        let airspeed = airspeed_mps(original_state.velocity);

        let (drag, aero_result) =
            match aero::get_drag_with_fallback(&original_state, altitude, aero_opt) {
                Ok(res) => res,
                Err(_) => return 6,
            };

        let throttles = mixer::mix_controls(&current_controls);

        let thrusts = match motor_thrusts_n(throttles, &current_config, altitude) {
            Ok(t) => t,
            Err(_) => return 6,
        };

        let fea_in = fea_inputs(
            thrusts,
            drag,
            original_state.orientation,
            original_thermal.motor_temp_c,
        );
        let (safety_margin, fea_result) =
            structural::get_safety_margin_with_fallback(&fea_in, fea_opt);

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

        let new_state =
            integrator::step_rk4(&original_state, net_force, net_torque, &current_config, dt);

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
            (*fault_latch).physics_fault = 1;
            (*out_telemetry).physics_fault = 1;
            return 8;
        }

        *state = new_state;

        let all_valid = envelope_flag(aero_result.as_ref(), fea_result.as_ref());
        let aero_src = if aero_result.is_some() { 2 } else { 1 };
        // No structural fallback exists: without a model the margin is NaN and the source is 0.
        let fea_src = if fea_result.is_some() { 2 } else { 0 };

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
            stub_loaded: stub_loaded_flag(aero_is_stub, fea_is_stub),
            aero_source: aero_src,
            fea_source: fea_src,
            physics_fault: latched_flag(&*fault_latch),
            dropped_time_events: 0,
            uncalibrated_flags: 0,
        };
    }

    0
}

/// Loads a trained ONNX surrogate model and returns an opaque handle to C++.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer, invalid UTF-8 path, or the path is not a readable file
/// * `2` - Missing provenance metadata
/// * `3` - Geometry hash mismatch
/// * `4` - Engine/ONNX initialization failure (including a missing ONNX Runtime library)
/// * `5` - Stub model rejected (allow_stub_models feature not enabled)
/// * `6` - Model inputs/outputs are not the supported shapes
/// * `7` - Handle registry unavailable; the model is dropped and `out_handle` is untouched
/// * `8` - Vehicle config hash mismatch (model `config_hash` differs from the compiled `vehicle_config.json`)
///
/// # Safety
/// * `path` must be a valid, null-terminated C string.
/// * `out_handle` must be a valid, aligned, mutable pointer to a pointer.
#[cfg(feature = "onnx")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_load_surrogate_model(
    path: *const std::ffi::c_char,
    out_handle: *mut *mut surrogate::SurrogateHandle,
) -> i32 {
    if path.is_null() || out_handle.is_null() {
        return 1;
    }

    // SAFETY: `path` is non-null and the caller guarantees a valid NUL-terminated C string.
    let c_str = unsafe { std::ffi::CStr::from_ptr(path) };
    let path_str = match c_str.to_str() {
        Ok(s) => s,
        Err(_) => return 1,
    };

    match surrogate::load_surrogate(path_str) {
        Ok(handle) => publish_handle(handle, out_handle),
        Err(e) => load_error_code(&e),
    }
}

/// Maps a surrogate load failure to its `ffi_load_surrogate_model` return code.
#[cfg(feature = "onnx")]
fn load_error_code(e: &surrogate::SurrogateLoadError) -> i32 {
    use surrogate::SurrogateLoadError as E;
    match e {
        E::IoError => 1,
        E::MissingProvenance => 2,
        E::GeometryMismatch { .. } => 3,
        E::StubModel => 5,
        E::BadModelShape => 6,
        E::ConfigMismatch { .. } => 8,
        E::EngineError(_) | E::InputCountMismatch { .. } | E::NonFiniteInput | E::BadOutput => 4,
    }
}

/// Boxes a loaded handle, registers its address as live and writes it to `out_handle`.
/// Returns 0, or 7 if the registry is unusable (the box is dropped, `out_handle` untouched).
#[cfg(feature = "onnx")]
fn publish_handle(
    handle: surrogate::SurrogateHandle,
    out_handle: *mut *mut surrogate::SurrogateHandle,
) -> i32 {
    let raw = Box::into_raw(Box::new(handle));
    match LIVE_HANDLES.lock() {
        Ok(mut live) => {
            register_in(&mut live, raw.addr());
            // SAFETY: caller of the FFI function guarantees `out_handle` is valid and aligned.
            unsafe {
                *out_handle = raw;
            }
            0
        }
        Err(_) => {
            // SAFETY: `raw` came from `Box::into_raw` just above and was never shared.
            drop(unsafe { Box::from_raw(raw) });
            7
        }
    }
}

/// Queries a loaded surrogate model.
///
/// `input_count` must equal the model's declared input width; no slice is built from it
/// before that check, so a huge value cannot read out of bounds.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer or negative `input_count`
/// * `2` - `handle` is not a live handle (never loaded, already unloaded, foreign)
/// * `3` - `input_count` differs from the model's input width
/// * `4` - Engine/ONNX inference error
/// * `5` - An input value is NaN or infinite
/// * `6` - Malformed model output (wrong count, non-finite, envelope flag not 0/1)
///
/// # Safety
/// * `handle` must be a pointer created by `ffi_load_surrogate_model` (checked against the
///   live registry before use).
/// * `inputs` must point to an array of exactly `input_count` floats.
/// * `out_result` must be a valid, aligned pointer.
#[cfg(feature = "onnx")]
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

    // Held for the whole query so an unload on another thread cannot free the handle.
    let live = match LIVE_HANDLES.lock() {
        Ok(g) => g,
        Err(_) => return 2,
    };
    if !live.contains(&handle.addr()) {
        return 2;
    }

    // SAFETY: `handle` is non-null and in the live registry, so it came from
    // `Box::into_raw` in `publish_handle` and has not been freed; the lock excludes unload.
    let handle_ref = unsafe { &mut *handle };
    let count = match surrogate::check_input_count(input_count, handle_ref.input_width) {
        Ok(n) => n,
        Err(_) => return 3,
    };
    // SAFETY: `inputs` is non-null; `count` equals the model width (at most
    // MAX_INPUT_WIDTH) and the caller guarantees `input_count` readable floats.
    let input_slice = unsafe { std::slice::from_raw_parts(inputs, count) };

    use surrogate::SurrogateLoadError as E;
    match surrogate::query_model(handle_ref, input_slice) {
        Ok(res) => {
            // SAFETY: `out_result` is non-null; the caller guarantees it is valid and aligned.
            unsafe {
                *out_result = res;
            }
            0
        }
        Err(E::InputCountMismatch { .. }) => 3,
        Err(E::NonFiniteInput) => 5,
        Err(E::BadOutput) => 6,
        Err(_) => 4,
    }
}

/// Unloads a surrogate model and frees its memory.
///
/// # Returns
/// * `0` - Freed
/// * `1` - Null pointer
/// * `2` - `handle` is not live (never loaded or already unloaded); nothing is freed
///
/// # Safety
/// * `handle` must be a pointer created by `ffi_load_surrogate_model` (checked against the
///   live registry before it is freed).
/// * The caller must not use `handle` after this function returns 0.
#[cfg(feature = "onnx")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_unload_surrogate_model(
    handle: *mut surrogate::SurrogateHandle,
) -> i32 {
    if handle.is_null() {
        return 1;
    }
    let removed = match LIVE_HANDLES.lock() {
        Ok(mut live) => unregister_from(&mut live, handle.addr()),
        Err(_) => false,
    };
    if !removed {
        return 2;
    }
    // SAFETY: the address was live, so it came from `Box::into_raw` in `publish_handle`;
    // removing it from the registry above means no other call can free or use it now.
    drop(unsafe { Box::from_raw(handle) });
    0
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
/// * `7` - Invalid Motor Position (non-finite)
/// * `8` - Invalid Propeller Diameter
/// * `9` - Invalid Thrust Coefficient
/// * `10` - Invalid Torque Coefficient
/// * `11` - Invalid Max RPM
/// * `12` - Invalid Hover Throttle (must be in (0, 1])
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
        Err(e) => config_error_code(e),
    }
}

/// Maps a config rejection to its FFI return code. Codes 2-6 are unchanged
/// from interface v5; 7-12 were added in v6.
fn config_error_code(e: config::ConfigError) -> i32 {
    use config::ConfigError as E;
    match e {
        E::IoError => 2,
        E::ParseError => 3,
        E::InvalidMass => 4,
        E::InvalidInertia => 5,
        E::InvalidSpin => 6,
        E::InvalidMotorPosition => 7,
        E::InvalidPropDiameter => 8,
        E::InvalidThrustCoeff => 9,
        E::InvalidTorqueCoeff => 10,
        E::InvalidMaxRpm => 11,
        E::InvalidHoverThrottle => 12,
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
    #[cfg(feature = "onnx")]
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
        let mut latch = FaultLatch::new();

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
                    &mut latch,
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
                    &mut latch,
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
                    &mut latch,
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
                    &mut latch,
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
                    &mut latch,
                    0.002
                ),
                1
            );
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
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
        let mut latch = FaultLatch::new();

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
                    &mut latch,
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
                    &mut latch,
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
                    &mut latch,
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
                    &mut latch,
                    0.1
                ),
                7
            );
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
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
        let mut latch = FaultLatch::new();

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
                    &mut latch,
                    0.002
                ),
                8
            );
            assert_eq!(telemetry.physics_fault, 1);
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_gravity_not_truncated() {
        // Hand calc: zero throttle -> zero thrust; zero velocity -> zero drag; zero torque.
        // Net force = (0, 0, 1.5 * 9.80665) N, a = 9.80665 m/s^2, RK4 exact for constant a:
        // z = 0.5 * 9.80665 * 0.002^2 = 1.96133e-5 m. An f32-narrowed force gives
        // 1.961330032e-5 m (error 3.2e-13 m), 3e5 times the 1e-18 tolerance.
        let mut state = ffi_create_default_drone_state();
        let mut thermal = ThermalState::new();
        let controls = ControlInputs {
            throttle: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let mut config = dummy_config();
        config.mass_kg = 1.5;
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();

        // SAFETY: all pointers are valid, aligned and unaliased for the call.
        let code = unsafe {
            ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            )
        };
        assert_eq!(code, 0);
        assert_eq!(state.position[0], 0.0);
        assert_eq!(state.position[1], 0.0);
        assert!((state.position[2] - 1.96133e-5).abs() <= 1e-18);
    }

    // Hand calc (python3, f64): rho(1000 m) = 1.1116522819524908,
    // T(0.5) = 0.11 * rho * (0.5 * 10000 / 60)^2 * 0.25^4 = 3.3171047909823668 N per motor.
    #[test]
    fn test_motor_thrusts_n_propagates_error() {
        let cfg = dummy_config();
        assert!(motor_thrusts_n([0.5; 4], &cfg, 12000.0).is_err());
        let t = motor_thrusts_n([0.5; 4], &cfg, 1000.0).unwrap();
        for v in t {
            assert!((v - 3.3171047909823668).abs() < 1e-12, "v = {v}");
        }
    }

    // Same hand calc. The telemetry field is f32 (ulp 2.4e-7 at 3.3), so 1e-6 N.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_thrust_at_altitude() {
        let mut state = ffi_create_default_drone_state();
        state.position[2] = -1000.0;
        let mut thermal = ThermalState::new();
        let controls = ControlInputs {
            throttle: 0.5,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();

        // SAFETY: all pointers are valid, aligned and unaliased for the call.
        let code = unsafe {
            ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            )
        };
        assert_eq!(code, 0);
        for t in telemetry.motor_thrusts {
            assert!((f64::from(t) - 3.3171047909823668).abs() < 1e-6, "t = {t}");
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_altitude_out_of_range_code_6() {
        let mut state = ffi_create_default_drone_state();
        state.position[2] = -12000.0;
        let initial = state;
        let mut thermal = ThermalState::new();
        let controls = zero_controls();
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();

        // SAFETY: all pointers are valid, aligned and unaliased for the call.
        let code = unsafe {
            ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            )
        };
        assert_eq!(code, 6);
        assert_eq!(state.position, initial.position);
        assert_eq!(state.velocity, initial.velocity);
        assert_eq!(thermal.motor_temp_c, [20.0; 4]);
        assert_eq!(latch.physics_fault, 0);
    }

    #[cfg(feature = "onnx")]
    fn query_result(envelope: i32) -> SurrogateQueryResult {
        SurrogateQueryResult {
            predicted_values: [0.0; 3],
            uncertainty: 0.0,
            in_validated_envelope: envelope,
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_envelope_flag_truth_table() {
        // 1 unless a queried surrogate reports 0; an unqueried surrogate does not lower it.
        let (in_env, out_env) = (query_result(1), query_result(0));
        assert_eq!(envelope_flag(None, None), 1);
        assert_eq!(envelope_flag(Some(&in_env), None), 1);
        assert_eq!(envelope_flag(None, Some(&in_env)), 1);
        assert_eq!(envelope_flag(Some(&out_env), None), 0);
        assert_eq!(envelope_flag(None, Some(&out_env)), 0);
        assert_eq!(envelope_flag(Some(&in_env), Some(&in_env)), 1);
        assert_eq!(envelope_flag(Some(&out_env), Some(&in_env)), 0);
        assert_eq!(envelope_flag(Some(&in_env), Some(&out_env)), 0);
        assert_eq!(envelope_flag(Some(&out_env), Some(&out_env)), 0);
    }

    #[test]
    fn test_airspeed_mps_is_f64() {
        // Hand calc: |(1, 1, 0)| = sqrt(2) = 1.4142135623730951 (f64). The f32 path gives
        // 1.4142135381698608 (error 2.4e-8). |(3, 4, 0)| = 5 exactly.
        let v = airspeed_mps([1.0, 1.0, 0.0]);
        assert!((v - std::f64::consts::SQRT_2).abs() <= 2.3e-16);
        assert_eq!(airspeed_mps([3.0, 4.0, 0.0]), 5.0);
    }

    #[test]
    fn test_ffi_load_vehicle_config_codes() {
        let shipped = std::ffi::CString::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../offline_pipeline/geometry/vehicle_config.json"
        ))
        .unwrap();
        let missing = std::ffi::CString::new("/nonexistent/vehicle_config.json").unwrap();
        let mut out = dummy_config();
        out.mass_kg = 0.0; // sentinel: must be overwritten on success only

        // SAFETY: pointers are valid for the call; null cases are the point of the test.
        unsafe {
            assert_eq!(ffi_load_vehicle_config(ptr::null(), &mut out), 1);
            assert_eq!(
                ffi_load_vehicle_config(shipped.as_ptr(), ptr::null_mut()),
                1
            );
            assert_eq!(ffi_load_vehicle_config(missing.as_ptr(), &mut out), 2);
            assert_eq!(out.mass_kg, 0.0);
            assert_eq!(ffi_load_vehicle_config(shipped.as_ptr(), &mut out), 0);
        }
        assert_eq!(out.mass_kg, 1.5);
    }

    #[test]
    fn test_ffi_config_error_codes_distinct() {
        use crate::config::ConfigError as E;
        let expected = [
            (E::IoError, 2),
            (E::ParseError, 3),
            (E::InvalidMass, 4),
            (E::InvalidInertia, 5),
            (E::InvalidSpin, 6),
            (E::InvalidMotorPosition, 7),
            (E::InvalidPropDiameter, 8),
            (E::InvalidThrustCoeff, 9),
            (E::InvalidTorqueCoeff, 10),
            (E::InvalidMaxRpm, 11),
            (E::InvalidHoverThrottle, 12),
        ];
        for (err, code) in expected {
            assert_eq!(config_error_code(err), code);
        }
    }

    #[cfg(feature = "onnx")]
    fn zero_controls() -> ControlInputs {
        ControlInputs {
            throttle: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_fault_latch_sticky_until_caller_clears() {
        // Hand calc, mass_kg = 0, v = 0, throttle 0.5: thrust per motor T > 0 (C_T*rho*n^2*D^4),
        // drag at v = 0 is exactly 0, gravity = 0 * 9.80665 = 0, so the vertical force is -4T
        // (NED, up) and a = F / 0 = -inf (IEEE 754). The new velocity is non-finite -> code 8.
        // The discarded step heated the motors above 20 C, so the [20; 4] check proves the
        // thermal rollback. (`*state = original_state` is redundant: state is written only after
        // the check, so no test can observe it.)
        // Start at 100 m (NED z = -100) so the free fall stays inside the ISA range 0..11000 m:
        // 6 healthy steps = 0.012 s, drop = 0.5 * 9.80665 * 0.012^2 = 7.1e-4 m.
        let mut initial = ffi_create_default_drone_state();
        initial.position[2] = -100.0;
        let mut state = initial;
        let mut thermal = ThermalState::new();
        let mut controls = zero_controls();
        controls.throttle = 0.5;
        let mut config = dummy_config();
        config.mass_kg = 0.0;
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();

        // SAFETY: all pointers are valid, aligned and unaliased for each call.
        unsafe {
            let code = ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            );
            assert_eq!(code, 8);
            assert_eq!(state.position, initial.position);
            assert_eq!(state.velocity, initial.velocity);
            assert_eq!(state.orientation, initial.orientation);
            assert_eq!(state.angular_velocity, initial.angular_velocity);
            assert_eq!(thermal.motor_temp_c, [20.0; 4]);
            assert_eq!(latch.physics_fault, 1);
            assert_eq!(telemetry.physics_fault, 1);

            // Healthy config: five more steps. Flag must stay set, state must stay finite.
            config.mass_kg = 1.0;
            for _ in 0..5 {
                let code = ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    &mut latch,
                    0.002,
                );
                assert_eq!(code, 0);
                assert!(state.position.iter().all(|x| x.is_finite()));
                assert!(state.velocity.iter().all(|x| x.is_finite()));
                assert_eq!(telemetry.physics_fault, 1);
                assert_eq!(latch.physics_fault, 1);
            }

            // Only the caller clears it.
            latch.physics_fault = 0;
            let code = ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            );
            assert_eq!(code, 0);
            assert_eq!(telemetry.physics_fault, 0);
            assert_eq!(latch.physics_fault, 0);
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_null_fault_latch() {
        let initial = ffi_create_default_drone_state();
        let mut state = initial;
        let mut thermal = ThermalState::new();
        let controls = zero_controls();
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();

        // SAFETY: valid pointers except the null latch, which is the point of the test.
        let code = unsafe {
            ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                ptr::null_mut(),
                0.002,
            )
        };
        assert_eq!(code, 1);
        assert_eq!(state.position, initial.position);
        assert_eq!(state.velocity, initial.velocity);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_no_models_reports_sources() {
        // No handles: aero uses the analytic path (1); no structural model exists, so the
        // source is 0 and the margin is NaN, never a fake safe number (HZ-02).
        let mut state = ffi_create_default_drone_state();
        let mut thermal = ThermalState::new();
        let controls = zero_controls();
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();

        // SAFETY: all pointers are valid, aligned and unaliased for the call.
        let code = unsafe {
            ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            )
        };
        assert_eq!(code, 0);
        assert_eq!(telemetry.fea_source, 0);
        assert_eq!(telemetry.aero_source, 1);
        assert!(telemetry.structural_safety_margin.is_nan());
        assert_eq!(telemetry.stub_loaded, 0);
        assert_eq!(telemetry.physics_fault, 0);
    }

    #[test]
    fn test_stub_loaded_flag_truth_table() {
        assert_eq!(stub_loaded_flag(false, false), 0);
        assert_eq!(stub_loaded_flag(true, false), 1);
        assert_eq!(stub_loaded_flag(false, true), 1);
        assert_eq!(stub_loaded_flag(true, true), 1);
    }

    #[test]
    fn test_ffi_interface_version() {
        assert_eq!(ffi_get_interface_version(), 9);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_load_error_codes() {
        use surrogate::SurrogateLoadError as E;
        let s = || String::from("x");
        assert_eq!(load_error_code(&E::IoError), 1);
        assert_eq!(load_error_code(&E::MissingProvenance), 2);
        assert_eq!(
            load_error_code(&E::GeometryMismatch {
                expected: s(),
                found: s()
            }),
            3
        );
        assert_eq!(load_error_code(&E::EngineError(s())), 4);
        assert_eq!(
            load_error_code(&E::InputCountMismatch {
                expected: 1,
                found: 2
            }),
            4
        );
        assert_eq!(load_error_code(&E::NonFiniteInput), 4);
        assert_eq!(load_error_code(&E::BadOutput), 4);
        assert_eq!(load_error_code(&E::StubModel), 5);
        assert_eq!(load_error_code(&E::BadModelShape), 6);
        assert_eq!(
            load_error_code(&E::ConfigMismatch {
                expected: s(),
                found: s()
            }),
            8
        );
    }

    /// Address of a handle that was never loaded. Never dereferenced: the registry is empty
    /// in unit tests (they use local sets), so every FFI entry point must reject it.
    #[cfg(feature = "onnx")]
    fn never_loaded_handle() -> *mut surrogate::SurrogateHandle {
        ptr::NonNull::<surrogate::SurrogateHandle>::dangling().as_ptr()
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_load_surrogate_null_and_bad_path() {
        let good = c"no_such_model.onnx";
        let mut out: *mut surrogate::SurrogateHandle = ptr::null_mut();
        unsafe {
            assert_eq!(ffi_load_surrogate_model(ptr::null(), &mut out), 1);
            assert_eq!(ffi_load_surrogate_model(good.as_ptr(), ptr::null_mut()), 1);
            // 0xFF is never valid UTF-8.
            let bad_utf8 = b"\xff\0";
            assert_eq!(
                ffi_load_surrogate_model(bad_utf8.as_ptr().cast(), &mut out),
                1
            );
            // Missing file: refused before ONNX Runtime is touched.
            assert_eq!(ffi_load_surrogate_model(good.as_ptr(), &mut out), 1);
        }
        assert!(out.is_null());
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_query_surrogate_null_and_unknown_handle() {
        let inputs = [1.0f32, 2.0, 3.0, 4.0];
        let sentinel = SurrogateQueryResult {
            predicted_values: [7.0, 8.0, 9.0],
            uncertainty: 5.0,
            in_validated_envelope: 3,
        };
        let mut out = sentinel;
        let h = never_loaded_handle();
        unsafe {
            assert_eq!(
                ffi_query_surrogate(ptr::null_mut(), inputs.as_ptr(), 4, &mut out),
                1
            );
            assert_eq!(ffi_query_surrogate(h, ptr::null(), 4, &mut out), 1);
            assert_eq!(
                ffi_query_surrogate(h, inputs.as_ptr(), 4, ptr::null_mut()),
                1
            );
            assert_eq!(ffi_query_surrogate(h, inputs.as_ptr(), -1, &mut out), 1);
            // Unknown handle: rejected before the count is used (no slice of i32::MAX floats).
            assert_eq!(ffi_query_surrogate(h, inputs.as_ptr(), 4, &mut out), 2);
            assert_eq!(
                ffi_query_surrogate(h, inputs.as_ptr(), i32::MAX, &mut out),
                2
            );
        }
        assert_eq!(out.predicted_values, sentinel.predicted_values);
        assert_eq!(out.uncertainty, sentinel.uncertainty);
        assert_eq!(out.in_validated_envelope, sentinel.in_validated_envelope);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_unload_surrogate_codes() {
        unsafe {
            assert_eq!(ffi_unload_surrogate_model(ptr::null_mut()), 1);
            // Never loaded: nothing is freed (freeing it would be undefined behaviour).
            assert_eq!(ffi_unload_surrogate_model(never_loaded_handle()), 2);
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_unknown_handle_code_9() {
        let controls = zero_controls();
        let config = dummy_config();
        for (aero, fea) in [
            (never_loaded_handle(), ptr::null_mut()),
            (ptr::null_mut(), never_loaded_handle()),
        ] {
            let mut state = ffi_create_default_drone_state();
            state.position = [1.0, 2.0, -3.0];
            let before = state;
            let mut thermal = ThermalState::new();
            let thermal_before = thermal;
            let mut telemetry = DebugTelemetry::new();
            telemetry.net_force = [4.0, 5.0, 6.0];
            let mut latch = FaultLatch::new();
            let code = unsafe {
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    aero,
                    fea,
                    &config,
                    &mut telemetry,
                    &mut latch,
                    0.002,
                )
            };
            assert_eq!(code, 9);
            assert_eq!(state.position, before.position);
            assert_eq!(state.velocity, before.velocity);
            assert_eq!(thermal.motor_temp_c, thermal_before.motor_temp_c);
            assert_eq!(latch.physics_fault, 0);
            assert_eq!(telemetry.net_force, [4.0, 5.0, 6.0]);
        }
    }

    #[test]
    fn test_step_handles_ok_truth_table() {
        let (a, b, c) = (0x1000usize, 0x2000usize, 0x3000usize);
        let live = BTreeSet::from([a, b]);
        assert!(step_handles_ok(0, 0, &live));
        assert!(step_handles_ok(a, 0, &live));
        assert!(step_handles_ok(0, b, &live));
        assert!(step_handles_ok(a, b, &live));
        // Same handle twice would alias two &mut.
        assert!(!step_handles_ok(a, a, &live));
        assert!(!step_handles_ok(c, 0, &live));
        assert!(!step_handles_ok(0, c, &live));
    }

    #[test]
    fn test_handle_registry_register_unregister() {
        // Fake addresses, never dereferenced.
        let mut live = BTreeSet::new();
        register_in(&mut live, 0x1000);
        register_in(&mut live, 0x2000);
        assert!(live.contains(&0x1000));
        assert!(unregister_from(&mut live, 0x1000));
        assert!(!live.contains(&0x1000));
        assert!(live.contains(&0x2000));
        // Double unload and a never-registered address are both reported.
        assert!(!unregister_from(&mut live, 0x1000));
        assert!(!unregister_from(&mut live, 0x3000));
    }

    #[test]
    fn test_ffi_reset_drone_state_null_and_ok() {
        unsafe {
            assert_eq!(ffi_reset_drone_state(ptr::null_mut()), 1);
        }
        let mut state = ffi_create_default_drone_state();
        state.position = [f64::NAN; 3];
        state.velocity = [f32::INFINITY; 3];
        state.orientation = [f32::NAN; 4];
        state.angular_velocity = [f32::NEG_INFINITY; 3];
        assert_eq!(unsafe { ffi_reset_drone_state(&mut state) }, 0);
        assert_eq!(state.position, [0.0; 3]);
        assert_eq!(state.velocity, [0.0; 3]);
        assert_eq!(state.orientation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(state.angular_velocity, [0.0; 3]);
    }

    /// A NaN that enters through the controls or the config must end in code 8, with the
    /// state restored exactly and the latch set; it must never be written out.
    #[cfg(feature = "onnx")]
    fn assert_nan_step_latches(controls: ControlInputs, config: VehicleConfig) {
        let mut state = ffi_create_default_drone_state();
        state.position = [1.0, 2.0, -50.0];
        state.velocity = [0.5, -0.5, 0.25];
        let before = state;
        let mut thermal = ThermalState::new();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let code = unsafe {
            ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            )
        };
        assert_eq!(code, 8);
        assert_eq!(state.position, before.position);
        assert_eq!(state.velocity, before.velocity);
        assert_eq!(state.orientation, before.orientation);
        assert_eq!(state.angular_velocity, before.angular_velocity);
        assert_eq!(thermal.motor_temp_c, [20.0; 4]);
        assert_eq!(latch.physics_fault, 1);
        assert_eq!(telemetry.physics_fault, 1);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_nan_controls_latches() {
        // Hand calc: f64::clamp(NaN) is NaN, so throttle NaN -> thrust NaN -> force NaN -> a NaN.
        let mut controls = zero_controls();
        controls.throttle = f32::NAN;
        assert_nan_step_latches(controls, dummy_config());
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_nan_config_latches() {
        // Hand calc: mass NaN -> a = F / m = NaN even with zero force.
        let mut config = dummy_config();
        config.mass_kg = f64::NAN;
        assert_nan_step_latches(zero_controls(), config);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_positive_yaw_gives_positive_yaw_rate() {
        // Hand calc (shipped config, kappa = 0.005 N*m/N, throttle 0.5, yaw +0.5):
        // mixer gives tau = [0.6, 0.4, 0.4, 0.6], FR/RL are the CCW (+1) motors, so
        // yaw torque = 0.005 * (T(0.6) - T(0.4) - T(0.4) + T(0.6)) > 0 for any increasing T.
        // Roll and pitch torques cancel by symmetry (|pos| equal, T equal in pairs).
        let config = config::load_vehicle_config(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../offline_pipeline/geometry/vehicle_config.json"
        ))
        .expect("shipped config valid");
        let mut state = ffi_create_default_drone_state();
        let mut thermal = ThermalState::new();
        let mut controls = zero_controls();
        controls.throttle = 0.5;
        controls.yaw = 0.5;
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();

        // SAFETY: all pointers are valid, aligned and unaliased for the call.
        let code = unsafe {
            ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            )
        };
        assert_eq!(code, 0);
        assert!(state.angular_velocity[2] > 0.0);
        assert!(state.angular_velocity[0].abs() < 1e-9);
        assert!(state.angular_velocity[1].abs() < 1e-9);
    }
}
