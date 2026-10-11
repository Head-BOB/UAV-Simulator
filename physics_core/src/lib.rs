// Tier: F
pub mod aero;
#[cfg(feature = "onnx")]
pub mod campaign;
pub mod config;
pub mod controller;
#[cfg(feature = "onnx")]
#[doc(hidden)]
pub mod controller_sim;
pub mod impact;
pub mod integrator;
pub mod mixer;
pub mod payload;
pub mod powertrain;
pub mod progress;
pub mod registry;
#[cfg(feature = "onnx")]
pub mod structural;
#[cfg(feature = "onnx")]
pub mod surrogate;
pub mod textout;
pub mod thermal;
pub mod turbulence;
pub mod types;
pub mod validation;

use std::panic::{AssertUnwindSafe, catch_unwind};
#[cfg(feature = "onnx")]
use surrogate::fea_surrogate::fea_inputs;
#[cfg(feature = "onnx")]
use types::{
    ControlInputs, DebugTelemetry, EnvironmentInput, SurrogateQueryResult, ThermalState, WindInput,
};
use types::{DroneState, FaultLatch};

// Without the `onnx` feature the FFI entry points that use surrogate handles are compiled out, so
// the pure helpers below are only reached by their tests. They stay compiled so miri runs them.

/// Return code: a panic was caught inside the FFI function, or the handle was faulted by an
/// earlier panic. The call's effect is undefined except that no memory was corrupted; the
/// caller should treat the run as invalid. Telemetry is not written.
pub const FFI_CODE_PANIC: i32 = 99;

/// Return code: an internal lock was poisoned and has been recovered. The call did nothing;
/// calling again works.
pub const FFI_CODE_LOCK_RECOVERED: i32 = 98;

/// `ffi_get_interface_version` / `ffi_get_drone_state_size` value if they panic.
const FFI_VALUE_PANIC: i32 = -1;

/// Process-wide registry of live surrogate handles (see `registry`).
#[cfg(feature = "onnx")]
static REGISTRY: registry::Registry = registry::Registry::new();

thread_local! {
    /// Panic injection point for the tests of the catch_unwind guard. Private and never set by
    /// production code or by anything outside this crate (no exported function writes it); it is
    /// not compiled out of non-test builds because `tools/check_mass_literals.py` allows only
    /// the one test-only attribute that introduces `mod tests`.
    static INJECT_PANIC: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Panics when a test set `INJECT_PANIC` on this thread; otherwise one thread-local read.
#[inline]
fn panic_point() {
    if INJECT_PANIC.with(std::cell::Cell::get) {
        panic!("injected test panic");
    }
}

/// Runs an FFI body and converts a panic into `fallback` (a panic must never unwind into C++).
/// Needs `panic = "unwind"`; with `panic = "abort"` this cannot catch anything.
fn guarded_or<T>(fallback: T, body: impl FnOnce() -> T) -> T {
    catch_unwind(AssertUnwindSafe(|| {
        panic_point();
        body()
    }))
    .unwrap_or(fallback)
}

/// `guarded_or` with the panic return code.
fn guarded(body: impl FnOnce() -> i32) -> i32 {
    guarded_or(FFI_CODE_PANIC, body)
}

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

/// Per-motor thrust in newtons at `altitude_m`. Any aero error is returned, never
/// replaced by zero thrust (a silent 0 N would look like a valid motor-out result).
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn motor_thrusts_n(
    throttles: [f64; 4],
    config: &types::VehicleConfig,
    altitude_m: f64,
    env: &types::EnvironmentInput,
) -> Result<[f64; 4], aero::AeroError> {
    Ok([
        aero::get_thrust_env(throttles[0], config, altitude_m, env)?,
        aero::get_thrust_env(throttles[1], config, altitude_m, env)?,
        aero::get_thrust_env(throttles[2], config, altitude_m, env)?,
        aero::get_thrust_env(throttles[3], config, altitude_m, env)?,
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

/// Envelope flag actually reported for a step: 0 if a stub model drove the step (a stub is
/// never validated, review finding A) or a passed-in model's query failed (its number is
/// missing or replaced by a fallback); otherwise `envelope_flag`.
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn step_envelope_flag(
    aero: Option<&types::SurrogateQueryResult>,
    fea: Option<&types::SurrogateQueryResult>,
    any_model_failed: bool,
    any_stub: bool,
) -> i32 {
    if any_model_failed || any_stub {
        0
    } else {
        envelope_flag(aero, fea)
    }
}

/// `aero_source` telemetry: 2 surrogate answered; 3 a handle was passed but its query failed
/// (analytic drag used instead); 1 no handle passed (analytic drag).
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn aero_source_code(handle_given: bool, surrogate_ok: bool) -> i32 {
    match (handle_given, surrogate_ok) {
        (_, true) => 2,
        (true, false) => 3,
        (false, false) => 1,
    }
}

/// `fea_source` telemetry: 2 surrogate answered; 3 a handle was passed but its query failed
/// (margin NaN); 0 no handle passed (margin NaN, no analytic fallback exists).
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn fea_source_code(handle_given: bool, surrogate_ok: bool) -> i32 {
    match (handle_given, surrogate_ok) {
        (_, true) => 2,
        (true, false) => 3,
        (false, false) => 0,
    }
}

/// Telemetry written for a REJECTED step (return codes 6, 7, 9, 10, 11, 8): everything at its
/// "no data" value so stale numbers from an earlier tick can never be read as this call's
/// result. Margin NaN, envelope 0, sources 0, thermal unvalidated, `physics_fault` = latch.
#[cfg_attr(not(feature = "onnx"), allow(dead_code))]
fn rejected_telemetry(latch_value: i32) -> types::DebugTelemetry {
    let mut t = types::DebugTelemetry::new();
    t.is_validated_envelope = 0;
    t.physics_fault = i32::from(latch_value != 0);
    t
}

/// Returns the current layout version to UE5 to prevent memory corruption on mismatch.
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_interface_version() -> i32 {
    guarded_or(FFI_VALUE_PANIC, || 14)
}

/// Allows UE5 to verify the byte size of DroneState during module initialization.
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_get_drone_state_size() -> i32 {
    // size_of::<DroneState>() is 64, asserted at compile time in types.rs, so the cast is exact.
    guarded_or(FFI_VALUE_PANIC, || std::mem::size_of::<DroneState>() as i32)
}

/// Instantiates a default, zeroed drone state with an identity quaternion.
/// If a panic is caught the state is all NaN, which the next step rejects loudly (code 8).
///
/// # Safety
/// Safe to call at any time.
#[unsafe(no_mangle)]
pub extern "C" fn ffi_create_default_drone_state() -> DroneState {
    let poisoned = DroneState {
        position: [f64::NAN; 3],
        velocity: [f32::NAN; 3],
        orientation: [f32::NAN; 4],
        angular_velocity: [f32::NAN; 3],
    };
    guarded_or(poisoned, default_drone_state)
}

fn default_drone_state() -> DroneState {
    DroneState {
        position: [0.0f64, 0.0, 0.0],
        velocity: [0.0, 0.0, 0.0],
        orientation: [0.0, 0.0, 0.0, 1.0],
        angular_velocity: [0.0, 0.0, 0.0],
    }
}

/// Resets the provided state to default values.
///
/// # Returns
/// `0` ok, `1` null pointer, `99` panic caught.
///
/// # Safety
/// * `state` must be a valid, aligned, and mutable pointer to a DroneState instance.
/// * The memory must not be concurrently accessed by another thread.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_reset_drone_state(state: *mut DroneState) -> i32 {
    guarded(|| {
        if state.is_null() {
            return 1;
        }
        // SAFETY: `state` is non-null and the caller guarantees it is valid, aligned and unshared.
        unsafe {
            *state = default_drone_state();
        }
        0
    })
}

/// Why `ffi_step_physics` returned without running the physics.
#[cfg(feature = "onnx")]
enum StepStop {
    /// Rejected before any computation (codes 7, 9, 11, 98, 99): nothing is written.
    Silent(i32),
    /// The tick failed during computation (code 6): telemetry reset to "no data", latch untouched.
    NoData(i32),
    /// Invalid or non-finite input, or non-finite output (code 8): latch set, telemetry reset.
    Diverged,
}

/// Writes the telemetry for a stopped step and returns its code.
///
/// # Safety
/// `out_telemetry` and `fault_latch` must be valid, aligned and mutable.
#[cfg(feature = "onnx")]
unsafe fn finish_stopped(
    stop: StepStop,
    out_telemetry: *mut DebugTelemetry,
    fault_latch: *mut FaultLatch,
) -> i32 {
    // SAFETY: the caller guarantees both pointers are valid.
    unsafe {
        match stop {
            StepStop::Silent(code) => code,
            StepStop::NoData(code) => {
                *out_telemetry = rejected_telemetry((*fault_latch).physics_fault);
                code
            }
            StepStop::Diverged => {
                (*fault_latch).physics_fault = 1;
                *out_telemetry = rejected_telemetry(1);
                8
            }
        }
    }
}

/// Everything `ffi_step_physics` validates before it touches any state.
/// Returns the work orientation (validated and re-normalized) on success.
#[cfg(feature = "onnx")]
fn validate_step_inputs(
    state: &DroneState,
    thermal_state: &ThermalState,
    config: &types::VehicleConfig,
    wind: &WindInput,
    environment: &EnvironmentInput,
    dt: f64,
) -> Result<[f32; 4], StepStop> {
    if dt <= 0.0 || dt.is_nan() || dt > 0.01 {
        return Err(StepStop::Silent(7));
    }
    // A hand-filled config that is non-finite, has mass <= 0, inertia <= 0 or max_rpm < 0 would
    // make the run meaningless, so it latches like a divergence (code 8), as NaN config always did.
    if config::validate_runtime_config(config).is_err() {
        return Err(StepStop::Diverged);
    }
    if !thermal::all_finite(&thermal_state.motor_temp_c) {
        return Err(StepStop::Diverged);
    }
    let work_orientation = match types::unit_quaternion_xyzw(state.orientation) {
        Ok(q) => q,
        Err(types::QuatError::NonFinite) => return Err(StepStop::Diverged),
        Err(types::QuatError::NotUnit) => return Err(StepStop::Silent(11)),
    };
    // Checked last so every earlier rejection keeps its code. Nothing is written (code 12).
    if aero::validate_wind(wind).is_err() {
        return Err(StepStop::Silent(12));
    }
    // Last of all, so every earlier rejection keeps its code. Nothing is written (code 13).
    if aero::validate_environment(environment).is_err() {
        return Err(StepStop::Silent(13));
    }
    Ok(work_orientation)
}

/// True if any component of the new state is NaN or infinite.
#[cfg(feature = "onnx")]
fn state_has_nan(s: &DroneState) -> bool {
    s.position.iter().any(|x| !x.is_finite())
        || s.velocity.iter().any(|x| !x.is_finite())
        || s.orientation.iter().any(|x| !x.is_finite())
        || s.angular_velocity.iter().any(|x| !x.is_finite())
}

/// Main execution block for the fixed-timestep RK4 physics pipeline.
///
/// Executes modules in strict order: Aero -> Structural -> Thermal -> Mixer -> Integrator.
///
/// # Wind (interface v11)
/// `wind` is the ambient air velocity in the NED world frame, m/s, all zeros = still air. Only the
/// aerodynamic drag uses it, through the air-relative velocity `v_air = v_ground - wind`. Thrust,
/// thermal convection (ground speed) and everything else are unchanged.
///
/// # Environment (interface v12)
/// `environment` sets the atmosphere: `site_altitude_m` (height of the NED origin above sea level) and
/// `temp_offset_c` (air temperature minus the ISA standard day). Air density for thrust and for the
/// analytic drag fallback comes from `aero::air_density`; the motor thermal ambient becomes
/// `thermal::AMBIENT_TEMP + temp_offset_c`. All zeros is the v11 behaviour bit for bit. A surrogate
/// aero answer ignores the environment (it was trained at one density). Humidity is not modelled.
///
/// # Units
/// * `dt` - Timestep in seconds.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer (nothing is written)
/// * `6` - Aero model error (drag or thrust), including altitude outside ISA 0..=11000 m
/// * `7` - Invalid dt (NaN, <= 0 or > 0.01)
/// * `8` - Divergence or invalid input: a NaN/Inf in `config`, `thermal_state.motor_temp_c` or
///   `state.orientation`; `config.mass_kg <= 0`, an inertia <= 0 or `max_rpm < 0` (re-checked
///   every step because C++ may fill the struct by hand); or a NaN/Inf in the new state or new
///   thermal state. `state` and `thermal_state` are left/restored to their previous values and
///   `fault_latch` is set to 1.
/// * `9` - A non-null surrogate handle is not live (never loaded, already unloaded, foreign),
///   or `aero_handle` and `fea_handle` are the same non-null handle
/// * `11` - `state.orientation` is finite but its norm differs from 1 by more than
///   `types::QUAT_NORM_TOLERANCE` (rejected, never silently rescaled)
/// * `12` - `wind.wind_ned_mps` has a NaN/Inf component or a component with magnitude above
///   `types::MAX_WIND_COMPONENT_MPS` (30 m/s). Rejected before any computation: NOTHING is written.
/// * `13` - `environment` has a NaN/Inf field, `site_altitude_m` outside 0..=5000, or
///   `|temp_offset_c| > 40`. Rejected before any computation, after the wind check: NOTHING is written.
///   (A total height `site_altitude_m` + vehicle altitude outside 0..=11000 m is code 6.)
/// * `98` - an internal lock was poisoned and recovered; nothing was done, call again
/// * `99` - a panic was caught, or a handle was faulted by an earlier panic; the run is invalid
///
/// # Behavior on a failed step
/// * `state` and `thermal_state` are always unchanged.
/// * Codes 1, 7, 9, 11, 12, 13, 98 and 99 are rejected before any computation: NOTHING is written
///   (telemetry and latch keep their previous values).
/// * Codes 6 and 8 fail during the tick: `out_telemetry` is overwritten with "no data" (margin
///   NaN, `is_validated_envelope` 0, sources 0, `thermal_valid` 0, `physics_fault` = latch value,
///   so it is never left holding the previous tick's numbers).
/// * `fault_latch` is set to 1 ONLY by code 8. Code 6 (altitude outside the ISA range) is a
///   scenario error, not a divergence, and leaves the latch as it was.
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
/// * `wind` must be a valid, aligned, immutable pointer to a `WindInput` (null gives code 1).
/// * `environment` must be a valid, aligned, immutable pointer to an `EnvironmentInput` (null gives code 1).
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
    wind: *const WindInput,
    environment: *const EnvironmentInput,
    aero_handle: *mut surrogate::SurrogateHandle,
    fea_handle: *mut surrogate::SurrogateHandle,
    config: *const types::VehicleConfig,
    out_telemetry: *mut DebugTelemetry,
    fault_latch: *mut FaultLatch,
    dt: f64,
) -> i32 {
    guarded(|| {
        if state.is_null()
            || thermal_state.is_null()
            || controls.is_null()
            || wind.is_null()
            || environment.is_null()
            || config.is_null()
            || out_telemetry.is_null()
            || fault_latch.is_null()
        {
            return 1;
        }
        // SAFETY: every pointer is non-null; the caller guarantees validity and no aliasing.
        unsafe {
            match step_checked(
                state,
                thermal_state,
                controls,
                wind,
                environment,
                aero_handle,
                fea_handle,
                config,
                out_telemetry,
                fault_latch,
                dt,
            ) {
                Ok(()) => 0,
                Err(stop) => finish_stopped(stop, out_telemetry, fault_latch),
            }
        }
    })
}

#[cfg(feature = "onnx")]
type SlotGuard<'a> = std::sync::MutexGuard<'a, registry::SlotState>;

/// Maps a locked handle's status to a stop reason (`None` when usable or no handle).
#[cfg(feature = "onnx")]
fn locked_status_to_stop(guard: &Option<SlotGuard<'_>>) -> Option<StepStop> {
    use registry::SlotStatus;
    match guard.as_deref().map(registry::slot_status) {
        Some(SlotStatus::Dead) => Some(StepStop::Silent(9)),
        Some(SlotStatus::Faulted) => Some(StepStop::Silent(FFI_CODE_PANIC)),
        Some(SlotStatus::Usable) | None => None,
    }
}

/// Aerodynamic drag (NED force) for the air-relative velocity `v_air = v_ground - wind`.
/// Only the velocity differs from `state`; the integrator still advances the ground velocity.
#[cfg(feature = "onnx")]
fn drag_in_wind(
    state: &DroneState,
    wind: &WindInput,
    environment: &EnvironmentInput,
    altitude_m: f64,
    aero_handle: Option<&mut surrogate::SurrogateHandle>,
) -> Result<([f32; 3], Option<SurrogateQueryResult>), aero::AeroError> {
    let air_state = DroneState {
        velocity: aero::air_relative_velocity(state.velocity, wind),
        ..*state
    };
    aero::get_drag_with_fallback_env(&air_state, altitude_m, environment, aero_handle)
}

/// Step body after the null checks. Returns `Ok(())` after writing state and telemetry.
///
/// # Safety
/// Same contract as `ffi_step_physics`, with every pointer other than the handles non-null.
#[cfg(feature = "onnx")]
#[allow(clippy::too_many_arguments)]
unsafe fn step_checked(
    state: *mut DroneState,
    thermal_state: *mut ThermalState,
    controls: *const ControlInputs,
    wind: *const WindInput,
    environment: *const EnvironmentInput,
    aero_handle: *mut surrogate::SurrogateHandle,
    fea_handle: *mut surrogate::SurrogateHandle,
    config: *const types::VehicleConfig,
    out_telemetry: *mut DebugTelemetry,
    fault_latch: *mut FaultLatch,
    dt: f64,
) -> Result<(), StepStop> {
    // SAFETY: the caller guarantees the pointers are valid; these are plain Copy reads.
    let (
        original_state,
        original_thermal,
        current_controls,
        current_config,
        current_wind,
        current_env,
    ) = unsafe {
        (
            *state,
            *thermal_state,
            *controls,
            *config,
            *wind,
            *environment,
        )
    };

    let work_orientation = validate_step_inputs(
        &original_state,
        &original_thermal,
        &current_config,
        &current_wind,
        &current_env,
        dt,
    )?;

    // Slots are looked up under the registry lock, which is released before any inference.
    let (aero_slot, fea_slot) = if aero_handle.is_null() && fea_handle.is_null() {
        (None, None)
    } else {
        match REGISTRY.step_slots(aero_handle.addr(), fea_handle.addr()) {
            Ok(Some(pair)) => pair,
            Ok(None) => return Err(StepStop::Silent(9)),
            Err(_) => return Err(StepStop::Silent(FFI_CODE_LOCK_RECOVERED)),
        }
    };
    let (aero_guard, fea_guard) = registry::lock_pair(
        aero_slot.as_deref().map(|s| (aero_handle.addr(), s)),
        fea_slot.as_deref().map(|s| (fea_handle.addr(), s)),
    );
    if let Some(stop) = locked_status_to_stop(&aero_guard).or(locked_status_to_stop(&fea_guard)) {
        return Err(stop);
    }

    // SAFETY: each non-null handle is live (checked above) and its slot lock is held in
    // `aero_guard` / `fea_guard` until the end of this function, so no unload can free it and no
    // other step uses it. The two addresses differ (`step_handles_ok`), so the `&mut` do not alias.
    let (aero_opt, fea_opt, aero_is_stub, fea_is_stub) = unsafe {
        (
            aero_handle.as_mut(),
            fea_handle.as_mut(),
            aero_handle.as_ref().is_some_and(|h| h.is_stub),
            fea_handle.as_ref().is_some_and(|h| h.is_stub),
        )
    };
    let aero_given = aero_opt.is_some();
    let fea_given = fea_opt.is_some();

    let work_state = DroneState {
        orientation: work_orientation,
        ..original_state
    };
    let altitude = -work_state.position[2];
    let airspeed = airspeed_mps(work_state.velocity);

    let (drag, aero_result) =
        drag_in_wind(&work_state, &current_wind, &current_env, altitude, aero_opt)
            .map_err(|_| StepStop::NoData(6))?;
    let throttles = mixer::mix_controls(&current_controls);
    let thrusts = motor_thrusts_n(throttles, &current_config, altitude, &current_env)
        .map_err(|_| StepStop::NoData(6))?;

    let temps_c = original_thermal.motor_temp_c;
    let fea_in = fea_inputs(thrusts, drag, work_orientation, temps_c);
    let (safety_margin, fea_result) = structural::get_safety_margin_with_fallback(&fea_in, fea_opt);

    let motor_rpms = throttles.map(|t| t * current_config.max_rpm);
    let mut new_thermal = original_thermal;
    thermal::update_temperatures_ambient(
        &mut new_thermal,
        &throttles,
        &motor_rpms,
        airspeed,
        dt,
        thermal::AMBIENT_TEMP + current_env.temp_offset_c,
    );

    let (net_force, net_torque, net_thrust) = mixer::calculate_net_forces(
        thrusts,
        drag.map(f64::from),
        work_orientation,
        &current_config,
    );
    let new_state = integrator::step_rk4(&work_state, net_force, net_torque, &current_config, dt);

    if state_has_nan(&new_state) || !thermal::all_finite(&new_thermal.motor_temp_c) {
        return Err(StepStop::Diverged);
    }

    let aero_failed = aero_given && aero_result.is_none();
    let fea_failed = fea_given && fea_result.is_none();
    // SAFETY: `fault_latch` is valid per the caller's contract.
    let latch_copy = latched_flag(unsafe { &*fault_latch });
    let telemetry = DebugTelemetry {
        aero_drag: drag,
        motor_thrusts: thrusts.map(|t| t as f32),
        net_force: net_force.map(|f| f as f32),
        gravity: [0.0, 0.0, (9.80665 * current_config.mass_kg) as f32],
        net_thrust: net_thrust.map(|f| f as f32),
        motor_temperatures_c: new_thermal.motor_temp_c.map(|t| t as f32),
        motor_rpms: motor_rpms.map(|r| r as f32),
        structural_safety_margin: safety_margin,
        is_validated_envelope: step_envelope_flag(
            aero_result.as_ref(),
            fea_result.as_ref(),
            aero_failed || fea_failed,
            aero_is_stub || fea_is_stub,
        ),
        stub_loaded: stub_loaded_flag(aero_is_stub, fea_is_stub),
        aero_source: aero_source_code(aero_given, aero_result.is_some()),
        fea_source: fea_source_code(fea_given, fea_result.is_some()),
        physics_fault: latch_copy,
        dropped_time_events: 0,
        uncalibrated_flags: 0,
        thermal_valid: i32::from(thermal::THERMAL_CONSTANTS_VALIDATED),
        thermal_over_temp: i32::from(thermal::any_over_temp(
            &new_thermal.motor_temp_c,
            thermal::MOTOR_TEMP_LIMIT_C,
        )),
    };

    // SAFETY: valid, aligned, exclusive per the caller's contract; written only on success.
    unsafe {
        *state = new_state;
        *thermal_state = new_thermal;
        *out_telemetry = telemetry;
    }
    drop((aero_guard, fea_guard));
    Ok(())
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
/// * `8` - Vehicle config hash mismatch (model `config_hash` differs from the compiled `vehicle_config.json`)
/// * `98` - Handle registry lock was poisoned and recovered; the model is dropped, `out_handle`
///   is untouched, call again (replaces the old code 7)
/// * `99` - A panic was caught; `out_handle` is untouched
///
/// A model whose `kind` is not `trained` (a stub) loads only with the `allow_stub_models`
/// feature, and its queries always report `in_validated_envelope = 0`.
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
    guarded(|| {
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
            Ok(handle) => publish_handle(&REGISTRY, handle, out_handle),
            Err(e) => load_error_code(&e),
        }
    })
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
/// Returns 0, or 98 if the registry lock was poisoned (the box is dropped, `out_handle` untouched).
#[cfg(feature = "onnx")]
fn publish_handle(
    registry: &registry::Registry,
    handle: surrogate::SurrogateHandle,
    out_handle: *mut *mut surrogate::SurrogateHandle,
) -> i32 {
    let raw = Box::into_raw(Box::new(handle));
    match registry.register(raw.addr()) {
        Ok(()) => {
            // SAFETY: caller of the FFI function guarantees `out_handle` is valid and aligned.
            unsafe {
                *out_handle = raw;
            }
            0
        }
        Err(_) => {
            // SAFETY: `raw` came from `Box::into_raw` just above and was never shared.
            drop(unsafe { Box::from_raw(raw) });
            FFI_CODE_LOCK_RECOVERED
        }
    }
}

/// Queries a loaded surrogate model.
///
/// `input_count` must equal the model's declared input width; no slice is built from it
/// before that check, so a huge value cannot read out of bounds. The registry lock is NOT held
/// during inference, only this handle's own lock.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer or negative `input_count`
/// * `2` - `handle` is not a live handle (never loaded, already unloaded, foreign)
/// * `3` - `input_count` differs from the model's input width
/// * `4` - Engine/ONNX inference error
/// * `5` - An input value is NaN or infinite
/// * `6` - Malformed model output (wrong count, non-finite, envelope flag not 0/1)
/// * `98` - Registry lock was poisoned and recovered; nothing was done, call again
/// * `99` - A panic was caught, or this handle was faulted by an earlier panic (unload it)
///
/// A stub model always reports `in_validated_envelope = 0`.
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
    guarded(|| {
        if handle.is_null() || inputs.is_null() || out_result.is_null() || input_count < 0 {
            return 1;
        }
        let slot = match REGISTRY.slot_of(handle.addr()) {
            Ok(Some(slot)) => slot,
            Ok(None) => return 2,
            Err(_) => return FFI_CODE_LOCK_RECOVERED,
        };
        // The registry lock is already released; only this handle is locked from here on.
        let guard = slot.lock();
        match registry::slot_status(&guard) {
            registry::SlotStatus::Dead => return 2,
            registry::SlotStatus::Faulted => return FFI_CODE_PANIC,
            registry::SlotStatus::Usable => {}
        }

        // SAFETY: `handle` is non-null and its slot is alive under the slot lock held in
        // `guard`, so it came from `Box::into_raw` in `publish_handle` and cannot be freed.
        let handle_ref = unsafe { &mut *handle };
        let count = match surrogate::check_input_count(input_count, handle_ref.input_width) {
            Ok(n) => n,
            Err(_) => return 3,
        };
        // SAFETY: `inputs` is non-null; `count` equals the model width (at most
        // MAX_INPUT_WIDTH) and the caller guarantees `input_count` readable floats.
        let input_slice = unsafe { std::slice::from_raw_parts(inputs, count) };

        use surrogate::SurrogateLoadError as E;
        let code = match surrogate::query_model(handle_ref, input_slice) {
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
        };
        drop(guard);
        code
    })
}

/// Unloads a surrogate model and frees its memory.
///
/// Waits for any query or step still using the handle, then frees it.
///
/// # Returns
/// * `0` - Freed
/// * `1` - Null pointer
/// * `2` - `handle` is not live (never loaded or already unloaded); nothing is freed
/// * `98` - Registry lock was poisoned and recovered; nothing was freed, call again
/// * `99` - A panic was caught while freeing
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
    guarded(|| {
        if handle.is_null() {
            return 1;
        }
        let slot = match REGISTRY.remove(handle.addr()) {
            Ok(Some(slot)) => slot,
            Ok(None) => return 2,
            Err(_) => return FFI_CODE_LOCK_RECOVERED,
        };
        // Waits for an in-flight query; anyone who looked the slot up earlier sees `alive == false`.
        let mut guard = slot.lock();
        guard.alive = false;
        // SAFETY: the address was live, so it came from `Box::into_raw` in `publish_handle`;
        // it is out of the registry and marked dead under its slot lock, so nothing else can
        // use or free it.
        drop(unsafe { Box::from_raw(handle) });
        drop(guard);
        0
    })
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
/// * `99` - A panic was caught
///
/// # Safety
/// * `path` must be a valid, null-terminated C string.
/// * `out_config` must be a valid, aligned, mutable pointer to a VehicleConfig.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_load_vehicle_config(
    path: *const std::ffi::c_char,
    out_config: *mut types::VehicleConfig,
) -> i32 {
    guarded(|| {
        if path.is_null() || out_config.is_null() {
            return 1;
        }

        // SAFETY: `path` is non-null and the caller guarantees a valid NUL-terminated C string.
        let c_str = unsafe { std::ffi::CStr::from_ptr(path) };
        let path_str = match c_str.to_str() {
            Ok(s) => s,
            Err(_) => return 1,
        };

        match config::load_vehicle_config(path_str) {
            Ok(cfg) => {
                // SAFETY: `out_config` is non-null; the caller guarantees it is valid and aligned.
                unsafe {
                    *out_config = cfg;
                }
                0
            }
            Err(e) => config_error_code(e),
        }
    })
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

// ---------------------------------------------------------------------------------------------------------------
// Powertrain (interface v13): motor + ESC + battery + thermal network. Additive: `ffi_step_physics`, `VehicleConfig`
// and the legacy thermal model are untouched. See `powertrain.rs` for the model and its limits.
// ---------------------------------------------------------------------------------------------------------------

/// Maps a powertrain config load failure to its FFI return code (2 IO, 3 parse, 4 invalid values).
fn powertrain_load_error_code(e: powertrain::LoadError) -> i32 {
    match e {
        powertrain::LoadError::Io => 2,
        powertrain::LoadError::Parse => 3,
        powertrain::LoadError::Invalid => 4,
    }
}

/// Loads and validates a powertrain configuration file (`powertrain_config.json`).
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer or invalid UTF-8 path
/// * `2` - File IO error
/// * `3` - JSON parse error (malformed, missing or unknown key, OCV table longer than 16 points)
/// * `4` - A value is out of range or the OCV table is malformed (see `powertrain::validate_powertrain_config`)
/// * `99` - A panic was caught
///
/// # Safety
/// * `path` must be a valid, null-terminated C string.
/// * `out_config` must be a valid, aligned, mutable pointer to a `PowertrainConfig`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_load_powertrain_config(
    path: *const std::ffi::c_char,
    out_config: *mut types::PowertrainConfig,
) -> i32 {
    guarded(|| {
        if path.is_null() || out_config.is_null() {
            return 1;
        }
        // SAFETY: `path` is non-null and the caller guarantees a valid NUL-terminated C string.
        let c_str = unsafe { std::ffi::CStr::from_ptr(path) };
        let Ok(path_str) = c_str.to_str() else {
            return 1;
        };
        match powertrain::load_powertrain_config(path_str) {
            Ok(cfg) => {
                // SAFETY: `out_config` is non-null; the caller guarantees it is valid and aligned.
                unsafe {
                    *out_config = cfg;
                }
                0
            }
            Err(e) => powertrain_load_error_code(e),
        }
    })
}

/// Creates a fresh `PowertrainState`: state of charge `soc`, every thermal node at `ambient_c`, no current history.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer
/// * `4` - `config` fails `validate_powertrain_config`
/// * `5` - `soc` outside [0, 1], or `ambient_c` not finite / outside -100..=2000 deg C. `out_state` is untouched.
/// * `99` - A panic was caught
///
/// # Safety
/// * `config` must be a valid, aligned, immutable pointer to a `PowertrainConfig`.
/// * `out_state` must be a valid, aligned, mutable pointer to a `PowertrainState`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_init_powertrain_state(
    config: *const types::PowertrainConfig,
    ambient_c: f64,
    soc: f64,
    out_state: *mut types::PowertrainState,
) -> i32 {
    guarded(|| {
        if config.is_null() || out_state.is_null() {
            return 1;
        }
        // SAFETY: `config` is non-null; the caller guarantees it is valid and aligned.
        if powertrain::validate_powertrain_config(unsafe { &*config }).is_err() {
            return 4;
        }
        match powertrain::initial_state(soc, ambient_c) {
            Ok(state) => {
                // SAFETY: `out_state` is non-null; the caller guarantees it is valid and aligned.
                unsafe {
                    *out_state = state;
                }
                0
            }
            Err(_) => 5,
        }
    })
}

/// Stateless bench point: one motor + propeller + ESC at a fixed pack voltage and duty, in air of density `rho_kgpm3`, with the
/// vehicle moving `axial_inflow_mps` along the rotor axis (0 for a static bench; the scenario of a manufacturer's thrust table:
/// voltage, propeller, throttle -> current, thrust, rpm).
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer
/// * `4` - `config` fails `validate_powertrain_config`
/// * `5` - `duty` outside [0, 1], `v_pack_v` negative, `rho_kgpm3` not positive, or `axial_inflow_mps` not finite
/// * `6` - The solve produced a non-finite value. `out` is untouched on every nonzero code.
/// * `99` - A panic was caught
///
/// # Safety
/// * `config` must be a valid, aligned, immutable pointer to a `PowertrainConfig`.
/// * `out` must be a valid, aligned, mutable pointer to a `PowertrainBenchOut`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_powertrain_bench_point(
    config: *const types::PowertrainConfig,
    duty: f64,
    v_pack_v: f64,
    rho_kgpm3: f64,
    axial_inflow_mps: f64,
    out: *mut types::PowertrainBenchOut,
) -> i32 {
    guarded(|| {
        if config.is_null() || out.is_null() {
            return 1;
        }
        // SAFETY: `config` is non-null; the caller guarantees it is valid and aligned.
        let cfg = unsafe { &*config };
        if powertrain::validate_powertrain_config(cfg).is_err() {
            return 4;
        }
        match powertrain::bench_point(cfg, duty, v_pack_v, rho_kgpm3, axial_inflow_mps) {
            Ok(point) => {
                // SAFETY: `out` is non-null; the caller guarantees it is valid and aligned.
                unsafe {
                    *out = point;
                }
                0
            }
            Err(powertrain::PowertrainError::NonFinite) => 6,
            Err(_) => 5,
        }
    })
}

/// Propeller coefficients `Ct` and `Cp` at advance ratio `j` and shaft speed `rpm` (the same lookup the step uses).
/// With `prop_model` 0 they are the constants; with 1 they come from the UIUC table, the nearest edge value is held outside the
/// measured data and `extrapolated` is set to 1.
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer
/// * `4` - `config` fails `validate_powertrain_config`
/// * `5` - `j` or `rpm` not finite. `out` is untouched on every nonzero code.
/// * `99` - A panic was caught
///
/// # Safety
/// * `config` must be a valid, aligned, immutable pointer to a `PowertrainConfig`.
/// * `out` must be a valid, aligned, mutable pointer to a `PropCoefficients`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_propeller_coefficients(
    config: *const types::PowertrainConfig,
    j: f64,
    rpm: f64,
    out: *mut types::PropCoefficients,
) -> i32 {
    guarded(|| {
        if config.is_null() || out.is_null() {
            return 1;
        }
        // SAFETY: `config` is non-null; the caller guarantees it is valid and aligned.
        let cfg = unsafe { &*config };
        if powertrain::validate_powertrain_config(cfg).is_err() {
            return 4;
        }
        if !j.is_finite() || !rpm.is_finite() {
            return 5;
        }
        match powertrain::coefficients(cfg, j, rpm) {
            Ok(p) => {
                // SAFETY: `out` is non-null; the caller guarantees it is valid and aligned.
                unsafe {
                    *out = types::PropCoefficients {
                        ct: p.ct,
                        cp: p.cp,
                        extrapolated: i32::from(p.extrapolated),
                        reserved: 0,
                    };
                }
                0
            }
            Err(_) => 5,
        }
    })
}

/// Everything `ffi_step_powertrain` validates before it touches any state. Shared checks keep their legacy order and codes;
/// the powertrain checks come last. Returns the work orientation (validated, re-normalized).
#[cfg(feature = "onnx")]
fn validate_powertrain_step_inputs(
    input: &PowertrainStepRefs<'_>,
    dt: f64,
) -> Result<[f32; 4], StepStop> {
    if dt <= 0.0 || dt.is_nan() || dt > 0.01 {
        return Err(StepStop::Silent(7));
    }
    if config::validate_runtime_config(input.config).is_err() {
        return Err(StepStop::Diverged);
    }
    let work_orientation = match types::unit_quaternion_xyzw(input.state.orientation) {
        Ok(q) => q,
        Err(types::QuatError::NonFinite) => return Err(StepStop::Diverged),
        Err(types::QuatError::NotUnit) => return Err(StepStop::Silent(11)),
    };
    if aero::validate_wind(input.wind).is_err() {
        return Err(StepStop::Silent(12));
    }
    if aero::validate_environment(input.environment).is_err() {
        return Err(StepStop::Silent(13));
    }
    if powertrain::validate_powertrain_config(input.powertrain).is_err()
        || powertrain::check_consistency(input.config, input.powertrain).is_err()
    {
        return Err(StepStop::Silent(14));
    }
    if powertrain::validate_state(input.pt_state).is_err() {
        return Err(StepStop::Silent(15));
    }
    Ok(work_orientation)
}

/// Borrowed copies of the caller's inputs, grouped to keep the helper signatures short.
#[cfg(feature = "onnx")]
struct PowertrainStepRefs<'a> {
    state: &'a DroneState,
    pt_state: &'a types::PowertrainState,
    config: &'a types::VehicleConfig,
    powertrain: &'a types::PowertrainConfig,
    wind: &'a WindInput,
    environment: &'a EnvironmentInput,
}

/// Maps a powertrain error raised during the computation to a stop reason (nothing has been written yet).
#[cfg(feature = "onnx")]
fn powertrain_stop(e: powertrain::PowertrainError) -> StepStop {
    use powertrain::PowertrainError as E;
    match e {
        E::InvalidConfig | E::ThermalStep => StepStop::Silent(14),
        E::InvalidState => StepStop::Silent(15),
        E::InvalidInput => StepStop::Diverged,
        E::NonFinite => StepStop::NoData(16),
    }
}

/// Whether one surrogate was supplied and whether it is a stub (for the telemetry flags).
#[cfg(feature = "onnx")]
#[derive(Clone, Copy)]
struct SurrogateUse {
    given: bool,
    is_stub: bool,
}

/// The two surrogates of a step.
#[cfg(feature = "onnx")]
#[derive(Clone, Copy)]
struct SurrogateFlags {
    aero: SurrogateUse,
    fea: SurrogateUse,
}

/// Narrows an f64 result to the f32 the FFI telemetry carries. The narrowing is the contract of the f32 telemetry fields
/// (ADR-001 D2: f32 only at the FFI boundary), so the lint is allowed here once instead of at every field.
#[cfg(feature = "onnx")]
#[allow(clippy::cast_possible_truncation)]
fn narrow(x: f64) -> f32 {
    x as f32
}

/// Results of one successful powertrain step, committed by the caller only after every check passed.
#[cfg(feature = "onnx")]
struct PowertrainOutcome {
    state: DroneState,
    pt_state: types::PowertrainState,
    debug: DebugTelemetry,
    pt_telemetry: types::PowertrainTelemetry,
}

/// The physics of one powertrain step: aero -> mixer duty -> powertrain -> structural -> forces -> RK4. Pure: it reads the
/// copies and returns the new values.
#[cfg(feature = "onnx")]
#[allow(clippy::too_many_arguments)]
fn compute_powertrain_step(
    refs: &PowertrainStepRefs<'_>,
    controls: &ControlInputs,
    orientation: [f32; 4],
    aero_opt: Option<&mut surrogate::SurrogateHandle>,
    fea_opt: Option<&mut surrogate::SurrogateHandle>,
    flags: SurrogateFlags,
    latch_value: i32,
    dt: f64,
) -> Result<PowertrainOutcome, StepStop> {
    let work_state = DroneState {
        orientation,
        ..*refs.state
    };
    let altitude = -work_state.position[2];
    let air_relative = aero::air_relative_velocity(work_state.velocity, refs.wind);
    let air_speed = airspeed_mps(air_relative);
    let axial_inflow = powertrain::axial_inflow_mps(air_relative, orientation);
    let rho = aero::air_density(altitude, refs.environment).map_err(|_| StepStop::NoData(6))?;
    let (drag, aero_result) =
        drag_in_wind(&work_state, refs.wind, refs.environment, altitude, aero_opt)
            .map_err(|_| StepStop::NoData(6))?;
    let step_input = powertrain::StepInput {
        duties: mixer::mix_controls(controls),
        rho_kgpm3: rho,
        airspeed_mps: air_speed,
        axial_inflow_mps: axial_inflow,
        ambient_c: thermal::AMBIENT_TEMP + refs.environment.temp_offset_c,
        dt_s: dt,
    };
    let mut pt_state = *refs.pt_state;
    let pt_tel =
        powertrain::step(refs.powertrain, &mut pt_state, &step_input).map_err(powertrain_stop)?;
    let thrusts = pt_tel.motor_thrust_n;
    let fea_in = fea_inputs(thrusts, drag, orientation, refs.pt_state.motor_winding_c);
    let (safety_margin, fea_result) = structural::get_safety_margin_with_fallback(&fea_in, fea_opt);
    let (net_force, net_torque, net_thrust) =
        mixer::calculate_net_forces(thrusts, drag.map(f64::from), orientation, refs.config);
    let new_state = integrator::step_rk4(&work_state, net_force, net_torque, refs.config, dt);
    if state_has_nan(&new_state) {
        return Err(StepStop::Diverged);
    }
    let aero_failed = flags.aero.given && aero_result.is_none();
    let fea_failed = flags.fea.given && fea_result.is_none();
    let debug = DebugTelemetry {
        aero_drag: drag,
        motor_thrusts: thrusts.map(narrow),
        net_force: net_force.map(narrow),
        gravity: [0.0, 0.0, narrow(9.80665 * refs.config.mass_kg)],
        net_thrust: net_thrust.map(narrow),
        motor_temperatures_c: pt_state.motor_winding_c.map(narrow),
        motor_rpms: pt_tel.motor_rpm.map(narrow),
        structural_safety_margin: safety_margin,
        is_validated_envelope: step_envelope_flag(
            aero_result.as_ref(),
            fea_result.as_ref(),
            aero_failed || fea_failed,
            flags.aero.is_stub || flags.fea.is_stub,
        ),
        stub_loaded: stub_loaded_flag(flags.aero.is_stub, flags.fea.is_stub),
        aero_source: aero_source_code(flags.aero.given, aero_result.is_some()),
        fea_source: fea_source_code(flags.fea.given, fea_result.is_some()),
        physics_fault: latch_value,
        dropped_time_events: 0,
        uncalibrated_flags: 0,
        thermal_valid: pt_tel.thermal_valid,
        thermal_over_temp: i32::from(powertrain::any_over_temp(&pt_tel)),
    };
    Ok(PowertrainOutcome {
        state: new_state,
        pt_state,
        debug,
        pt_telemetry: pt_tel,
    })
}

/// Writes the telemetry for a stopped powertrain step and returns its code (same rules as `finish_stopped`, plus the
/// powertrain telemetry reset to "no data").
///
/// # Safety
/// All three pointers must be valid, aligned and mutable.
#[cfg(feature = "onnx")]
unsafe fn finish_stopped_powertrain(
    stop: &StepStop,
    out_telemetry: *mut DebugTelemetry,
    out_powertrain: *mut types::PowertrainTelemetry,
    fault_latch: *mut FaultLatch,
) -> i32 {
    // SAFETY: the caller guarantees the pointers are valid.
    unsafe {
        match stop {
            StepStop::Silent(code) => *code,
            StepStop::NoData(code) => {
                *out_telemetry = rejected_telemetry((*fault_latch).physics_fault);
                *out_powertrain = types::PowertrainTelemetry::no_data();
                *code
            }
            StepStop::Diverged => {
                (*fault_latch).physics_fault = 1;
                *out_telemetry = rejected_telemetry(1);
                *out_powertrain = types::PowertrainTelemetry::no_data();
                8
            }
        }
    }
}

/// Fixed-timestep RK4 step with the powertrain model (interface v13): the motors' speed, thrust, current and temperatures
/// come from the motor/propeller torque balance at the voltage the battery can supply, instead of `throttle x max_rpm`.
///
/// Order: aero -> mixer (throttle = ESC duty) -> powertrain (battery, ESC, motors, thermal) -> structural -> forces -> integrator.
/// `VehicleConfig::max_rpm` and `hover_throttle` are NOT used for the motors here (hover duty differs from `hover_throttle`;
/// read it from the build sidecar). Cooling uses the AIR-relative speed (ground velocity minus wind).
///
/// # Returns
/// * `0` - Success
/// * `1` - Null pointer (`aero_handle` and `fea_handle` may be null)
/// * `6` - Aero model error, altitude outside the ISA range, or a non-finite powertrain result (16 is used for the latter)
/// * `7` - Invalid dt (NaN, <= 0 or > 0.01)
/// * `8` - Divergence or invalid input (as in `ffi_step_physics`); latch set
/// * `9` / `11` / `12` / `13` / `98` / `99` - as in `ffi_step_physics`
/// * `14` - `powertrain` is invalid, contradicts `config` (propeller diameter, Ct, kappa), or the thermal network is too
///   fast for `dt` (`dt > 0.2 x tau_min`). Nothing is written.
/// * `15` - `powertrain_state` is invalid (SOC outside [0, 1], non-finite or implausible temperature, negative charge). Nothing is written.
/// * `16` - A powertrain result was not finite. `out_telemetry` and `out_powertrain` are reset to "no data"; the latch is untouched.
///
/// # Behavior on a failed step
/// `state` and `powertrain_state` are always unchanged. Codes 1, 7, 9, 11, 12, 13, 14, 15, 98, 99 write nothing. Codes 6, 16
/// and 8 overwrite both telemetry structs with "no data" (NaN powertrain floats, `step_valid` 0); only 8 sets the latch.
///
/// # Safety
/// * `state`, `powertrain_state`, `out_telemetry`, `out_powertrain`, `fault_latch` must be valid, aligned, mutable pointers.
/// * `controls`, `wind`, `environment`, `config`, `powertrain` must be valid, aligned, immutable pointers.
/// * `aero_handle` and `fea_handle` may be null; if non-null they must be valid `SurrogateHandle`s.
/// * Pointers must not alias or be subject to concurrent mutation.
#[cfg(feature = "onnx")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn ffi_step_powertrain(
    state: *mut DroneState,
    powertrain_state: *mut types::PowertrainState,
    controls: *const ControlInputs,
    wind: *const WindInput,
    environment: *const EnvironmentInput,
    aero_handle: *mut surrogate::SurrogateHandle,
    fea_handle: *mut surrogate::SurrogateHandle,
    config: *const types::VehicleConfig,
    powertrain: *const types::PowertrainConfig,
    out_telemetry: *mut DebugTelemetry,
    out_powertrain: *mut types::PowertrainTelemetry,
    fault_latch: *mut FaultLatch,
    dt: f64,
) -> i32 {
    guarded(|| {
        if state.is_null()
            || powertrain_state.is_null()
            || controls.is_null()
            || wind.is_null()
            || environment.is_null()
            || config.is_null()
            || powertrain.is_null()
            || out_telemetry.is_null()
            || out_powertrain.is_null()
            || fault_latch.is_null()
        {
            return 1;
        }
        // SAFETY: every pointer is non-null; the caller guarantees validity and no aliasing.
        unsafe {
            match step_powertrain_checked(
                state,
                powertrain_state,
                controls,
                wind,
                environment,
                aero_handle,
                fea_handle,
                config,
                powertrain,
                out_telemetry,
                out_powertrain,
                fault_latch,
                dt,
            ) {
                Ok(()) => 0,
                Err(stop) => {
                    finish_stopped_powertrain(&stop, out_telemetry, out_powertrain, fault_latch)
                }
            }
        }
    })
}

/// Step body after the null checks: validate, lock the surrogate slots, compute, then commit.
///
/// # Safety
/// Same contract as `ffi_step_powertrain`, with every pointer other than the handles non-null.
#[cfg(feature = "onnx")]
#[allow(clippy::too_many_arguments)]
unsafe fn step_powertrain_checked(
    state: *mut DroneState,
    powertrain_state: *mut types::PowertrainState,
    controls: *const ControlInputs,
    wind: *const WindInput,
    environment: *const EnvironmentInput,
    aero_handle: *mut surrogate::SurrogateHandle,
    fea_handle: *mut surrogate::SurrogateHandle,
    config: *const types::VehicleConfig,
    powertrain: *const types::PowertrainConfig,
    out_telemetry: *mut DebugTelemetry,
    out_powertrain: *mut types::PowertrainTelemetry,
    fault_latch: *mut FaultLatch,
    dt: f64,
) -> Result<(), StepStop> {
    // SAFETY: the caller guarantees the pointers are valid; these are plain Copy reads.
    let (s, pts, ctl, cfg, ptc, w, env) = unsafe {
        (
            *state,
            *powertrain_state,
            *controls,
            *config,
            *powertrain,
            *wind,
            *environment,
        )
    };
    let refs = PowertrainStepRefs {
        state: &s,
        pt_state: &pts,
        config: &cfg,
        powertrain: &ptc,
        wind: &w,
        environment: &env,
    };
    let orientation = validate_powertrain_step_inputs(&refs, dt)?;

    // Slots are looked up under the registry lock, which is released before any inference.
    let (aero_slot, fea_slot) = if aero_handle.is_null() && fea_handle.is_null() {
        (None, None)
    } else {
        match REGISTRY.step_slots(aero_handle.addr(), fea_handle.addr()) {
            Ok(Some(pair)) => pair,
            Ok(None) => return Err(StepStop::Silent(9)),
            Err(_) => return Err(StepStop::Silent(FFI_CODE_LOCK_RECOVERED)),
        }
    };
    let (aero_guard, fea_guard) = registry::lock_pair(
        aero_slot.as_deref().map(|s| (aero_handle.addr(), s)),
        fea_slot.as_deref().map(|s| (fea_handle.addr(), s)),
    );
    if let Some(stop) = locked_status_to_stop(&aero_guard).or(locked_status_to_stop(&fea_guard)) {
        return Err(stop);
    }
    // SAFETY: each non-null handle is live (checked above) and its slot lock is held in `aero_guard` / `fea_guard`
    // until the end of this function, so no unload can free it and no other step uses it. The two addresses differ
    // (`step_handles_ok`), so the `&mut` do not alias.
    let (aero_opt, fea_opt, flags) = unsafe {
        (
            aero_handle.as_mut(),
            fea_handle.as_mut(),
            SurrogateFlags {
                aero: SurrogateUse {
                    given: !aero_handle.is_null(),
                    is_stub: aero_handle.as_ref().is_some_and(|h| h.is_stub),
                },
                fea: SurrogateUse {
                    given: !fea_handle.is_null(),
                    is_stub: fea_handle.as_ref().is_some_and(|h| h.is_stub),
                },
            },
        )
    };
    // SAFETY: `fault_latch` is valid per the caller's contract.
    let latch_copy = latched_flag(unsafe { &*fault_latch });
    let out = compute_powertrain_step(
        &refs,
        &ctl,
        orientation,
        aero_opt,
        fea_opt,
        flags,
        latch_copy,
        dt,
    )?;
    // SAFETY: valid, aligned, exclusive per the caller's contract; written only on success.
    unsafe {
        *state = out.state;
        *powertrain_state = out.pt_state;
        *out_telemetry = out.debug;
        *out_powertrain = out.pt_telemetry;
    }
    drop((aero_guard, fea_guard));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::{register_in, step_handles_ok, unregister_from};
    use crate::types::VehicleConfig;
    use std::collections::BTreeSet;
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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
        assert!(
            motor_thrusts_n(
                [0.5; 4],
                &cfg,
                12000.0,
                &types::EnvironmentInput::standard()
            )
            .is_err()
        );
        let t =
            motor_thrusts_n([0.5; 4], &cfg, 1000.0, &types::EnvironmentInput::standard()).unwrap();
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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
        assert_eq!(ffi_get_interface_version(), 14);
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
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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
                &WindInput::still(),
                &EnvironmentInput::standard(),
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

    // ---- Review findings A-F -------------------------------------------------------------

    /// Runs `body` with a panic injected at the entry of the next guarded FFI call on this thread.
    fn with_injected_panic<T>(body: impl FnOnce() -> T) -> T {
        INJECT_PANIC.with(|c| c.set(true));
        let out = body();
        INJECT_PANIC.with(|c| c.set(false));
        out
    }

    // Finding B: a panic must come back as a code, never unwind into C++.
    #[test]
    fn test_guarded_converts_panic_to_code_99() {
        assert_eq!(guarded(|| panic!("deliberate")), FFI_CODE_PANIC);
        assert_eq!(guarded(|| 5), 5);
        assert_eq!(guarded_or(-7, || panic!("deliberate")), -7);
        assert_eq!(FFI_CODE_PANIC, 99);
        assert_eq!(FFI_CODE_LOCK_RECOVERED, 98);
    }

    // The catch only works with panic = "unwind". A release profile with `panic = "abort"`
    // would turn every caught panic into a process abort, so the manifest must not set it.
    #[test]
    fn test_cargo_profile_does_not_abort_on_panic() {
        const MANIFEST: &str = include_str!("../Cargo.toml");
        assert!(!MANIFEST.contains("panic = \"abort\""));
    }

    // Deliberate panic through each exported function that has no surrogate handle.
    #[test]
    fn test_ffi_panic_is_caught_in_pure_exports() {
        assert_eq!(with_injected_panic(|| ffi_get_interface_version()), -1);
        assert_eq!(with_injected_panic(|| ffi_get_drone_state_size()), -1);
        let poisoned = with_injected_panic(|| ffi_create_default_drone_state());
        assert!(poisoned.position.iter().all(|v| v.is_nan()));
        assert!(poisoned.orientation.iter().all(|v| v.is_nan()));

        let mut state = ffi_create_default_drone_state();
        state.position = [1.0, 2.0, 3.0];
        // SAFETY: `state` is valid, aligned and unaliased.
        let code = with_injected_panic(|| unsafe { ffi_reset_drone_state(&mut state) });
        assert_eq!(code, 99);
        assert_eq!(state.position, [1.0, 2.0, 3.0]);

        let path = std::ffi::CString::new("x").expect("no NUL");
        let mut cfg = dummy_config();
        // SAFETY: valid NUL-terminated path and a valid config pointer.
        let code =
            with_injected_panic(|| unsafe { ffi_load_vehicle_config(path.as_ptr(), &mut cfg) });
        assert_eq!(code, 99);
        // The next call is not poisoned by the caught panic.
        assert_eq!(ffi_get_interface_version(), 14);
    }

    // Deliberate panic through the step and the three handle functions.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_panic_is_caught_in_step_and_handle_exports() {
        let mut state = ffi_create_default_drone_state();
        state.position = [1.0, 2.0, -3.0];
        let before = state;
        let mut thermal = ThermalState::new();
        let controls = zero_controls();
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();
        telemetry.net_force = [4.0, 5.0, 6.0];
        let mut latch = FaultLatch::new();
        // SAFETY: all pointers are valid, aligned and unaliased for the call.
        let code = with_injected_panic(|| unsafe {
            ffi_step_physics(
                &mut state,
                &mut thermal,
                &controls,
                &WindInput::still(),
                &EnvironmentInput::standard(),
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                &mut telemetry,
                &mut latch,
                0.002,
            )
        });
        assert_eq!(code, 99);
        assert_eq!(state.position, before.position);
        assert_eq!(thermal.motor_temp_c, [20.0; 4]);
        assert_eq!(telemetry.net_force, [4.0, 5.0, 6.0]);
        assert_eq!(latch.physics_fault, 0);

        let path = std::ffi::CString::new("x").expect("no NUL");
        let mut out: *mut surrogate::SurrogateHandle = ptr::null_mut();
        // SAFETY: valid path and out pointer.
        let code =
            with_injected_panic(|| unsafe { ffi_load_surrogate_model(path.as_ptr(), &mut out) });
        assert_eq!(code, 99);
        assert!(out.is_null());

        let inputs = [0.0f32; 3];
        let mut result = SurrogateQueryResult {
            predicted_values: [0.0; 3],
            uncertainty: 0.0,
            in_validated_envelope: 7,
        };
        let fake = 0x1000usize as *mut surrogate::SurrogateHandle;
        // SAFETY: the panic fires before any pointer is used.
        let code = with_injected_panic(|| unsafe {
            ffi_query_surrogate(fake, inputs.as_ptr(), 3, &mut result)
        });
        assert_eq!(code, 99);
        assert_eq!(result.in_validated_envelope, 7);
        // SAFETY: the panic fires before the pointer is used.
        assert_eq!(
            with_injected_panic(|| unsafe { ffi_unload_surrogate_model(fake) }),
            99
        );
    }

    // Finding A (step level): a stub drives the flag to 0 even if the model said 1.
    // Hand calc: results both 1, no failure: 1. stub: 0. failure: 0. a result of 0 (outside): 0.
    #[test]
    fn test_step_envelope_flag_truth_table() {
        let r = |env: i32| types::SurrogateQueryResult {
            predicted_values: [0.0; 3],
            uncertainty: 0.0,
            in_validated_envelope: env,
        };
        assert_eq!(step_envelope_flag(None, None, false, false), 1);
        assert_eq!(
            step_envelope_flag(Some(&r(1)), Some(&r(1)), false, false),
            1
        );
        assert_eq!(step_envelope_flag(Some(&r(1)), Some(&r(1)), false, true), 0);
        assert_eq!(step_envelope_flag(Some(&r(1)), None, true, false), 0);
        assert_eq!(
            step_envelope_flag(Some(&r(0)), Some(&r(1)), false, false),
            0
        );
        assert_eq!(step_envelope_flag(None, None, false, true), 0);
    }

    // Finding F: "model failed" (3) is never the same value as "no model loaded" (1 aero, 0 fea).
    #[test]
    fn test_source_codes_distinguish_failed_from_absent() {
        assert_eq!(aero_source_code(false, false), 1);
        assert_eq!(aero_source_code(true, true), 2);
        assert_eq!(aero_source_code(true, false), 3);
        assert_eq!(fea_source_code(false, false), 0);
        assert_eq!(fea_source_code(true, true), 2);
        assert_eq!(fea_source_code(true, false), 3);
    }

    #[test]
    fn test_rejected_telemetry_is_no_data() {
        let t = rejected_telemetry(0);
        assert!(t.structural_safety_margin.is_nan());
        assert_eq!(t.is_validated_envelope, 0);
        assert_eq!((t.aero_source, t.fea_source, t.stub_loaded), (0, 0, 0));
        assert_eq!((t.thermal_valid, t.thermal_over_temp), (0, 0));
        assert_eq!(t.physics_fault, 0);
        assert_eq!(t.net_force, [0.0; 3]);
        assert_eq!(rejected_telemetry(1).physics_fault, 1);
        assert_eq!(rejected_telemetry(5).physics_fault, 1);
    }

    #[cfg(feature = "onnx")]
    fn run_step(
        state: &mut DroneState,
        thermal: &mut ThermalState,
        controls: &ControlInputs,
        config: &VehicleConfig,
        telemetry: &mut DebugTelemetry,
        latch: &mut FaultLatch,
    ) -> i32 {
        // SAFETY: all pointers are valid, aligned and unaliased for the call.
        unsafe {
            ffi_step_physics(
                state,
                thermal,
                controls,
                &WindInput::still(),
                &EnvironmentInput::standard(),
                ptr::null_mut(),
                ptr::null_mut(),
                config,
                telemetry,
                latch,
                0.002,
            )
        }
    }

    // Finding D, code 6: altitude -5 m (NED z = +5) is outside ISA 0..=11000 m. Defined behavior:
    // state and thermal unchanged, latch unchanged (0), telemetry reset to "no data" (the stale
    // [4,5,6] force and envelope 1 must be gone).
    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_code_6_resets_telemetry_and_leaves_latch() {
        let mut state = ffi_create_default_drone_state();
        state.position = [0.0, 0.0, 5.0];
        let before = state;
        let mut thermal = ThermalState::new();
        let mut telemetry = DebugTelemetry::new();
        telemetry.net_force = [4.0, 5.0, 6.0];
        telemetry.structural_safety_margin = 3.0;
        telemetry.is_validated_envelope = 1;
        telemetry.aero_source = 2;
        let mut latch = FaultLatch::new();
        let code = run_step(
            &mut state,
            &mut thermal,
            &zero_controls(),
            &dummy_config(),
            &mut telemetry,
            &mut latch,
        );
        assert_eq!(code, 6);
        assert_eq!(state.position, before.position);
        assert_eq!(thermal.motor_temp_c, [20.0; 4]);
        assert_eq!(latch.physics_fault, 0);
        assert_eq!(telemetry.net_force, [0.0; 3]);
        assert!(telemetry.structural_safety_margin.is_nan());
        assert_eq!(telemetry.is_validated_envelope, 0);
        assert_eq!(telemetry.aero_source, 0);
        assert_eq!(telemetry.physics_fault, 0);

        // With the latch already set, the telemetry copy reads 1 and the latch stays 1.
        latch.physics_fault = 1;
        let code = run_step(
            &mut state,
            &mut thermal,
            &zero_controls(),
            &dummy_config(),
            &mut telemetry,
            &mut latch,
        );
        assert_eq!(code, 6);
        assert_eq!(latch.physics_fault, 1);
        assert_eq!(telemetry.physics_fault, 1);
    }

    // Codes rejected before computation (11 here) write nothing: telemetry and latch keep
    // their previous values.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_code_11_non_unit_quaternion_writes_nothing() {
        for q in [
            [0.0, 0.0, 0.0, 1.01],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.99],
        ] {
            let mut state = ffi_create_default_drone_state();
            state.orientation = q;
            let mut thermal = ThermalState::new();
            let mut telemetry = DebugTelemetry::new();
            telemetry.net_force = [4.0, 5.0, 6.0];
            let mut latch = FaultLatch::new();
            let code = run_step(
                &mut state,
                &mut thermal,
                &zero_controls(),
                &dummy_config(),
                &mut telemetry,
                &mut latch,
            );
            assert_eq!(code, 11, "q {q:?}");
            assert_eq!(state.orientation, q);
            assert_eq!(telemetry.net_force, [4.0, 5.0, 6.0]);
            assert_eq!(latch.physics_fault, 0);
        }
    }

    // Hand calc: |q| = 1.0005 is within 1e-3, so the step runs on the re-normalized (0,0,0,1);
    // free fall with zero throttle from rest: v_z after one step = g dt = 9.80665 * 0.002 = 0.0196133
    // (same as with an exactly unit quaternion). Output orientation stays a unit quaternion.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_slightly_non_unit_quaternion_is_normalized() {
        let mut state = ffi_create_default_drone_state();
        state.position[2] = -100.0;
        state.orientation = [0.0, 0.0, 0.0, 1.0005];
        let mut thermal = ThermalState::new();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let code = run_step(
            &mut state,
            &mut thermal,
            &zero_controls(),
            &dummy_config(),
            &mut telemetry,
            &mut latch,
        );
        assert_eq!(code, 0);
        assert!((f64::from(state.velocity[2]) - 0.0196133).abs() < 1e-6);
        let n: f64 = state
            .orientation
            .iter()
            .map(|c| f64::from(*c).powi(2))
            .sum();
        assert!((n.sqrt() - 1.0).abs() < 1e-6);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_nan_quaternion_latches() {
        let mut state = ffi_create_default_drone_state();
        state.orientation = [f32::NAN, 0.0, 0.0, 1.0];
        let mut thermal = ThermalState::new();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let code = run_step(
            &mut state,
            &mut thermal,
            &zero_controls(),
            &dummy_config(),
            &mut telemetry,
            &mut latch,
        );
        assert_eq!(code, 8);
        assert_eq!(latch.physics_fault, 1);
        assert!(state.orientation[0].is_nan());
        assert_eq!(thermal.motor_temp_c, [20.0; 4]);
    }

    // Finding D: hand-filled config checks. mass 0, inertia 0, max_rpm -1, mass +Inf all
    // diverge (code 8 + latch, nothing changed); max_rpm 0 is legal (motors give 0 N) -> code 0.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_revalidates_config() {
        let bad: [fn(&mut VehicleConfig); 4] = [
            |c| c.mass_kg = 0.0,
            |c| c.inertia_kgm2[2] = 0.0,
            |c| c.max_rpm = -1.0,
            |c| c.mass_kg = f64::INFINITY,
        ];
        for mutate in bad {
            let mut config = dummy_config();
            mutate(&mut config);
            let mut state = ffi_create_default_drone_state();
            state.position[2] = -100.0;
            let before = state;
            let mut thermal = ThermalState::new();
            let mut telemetry = DebugTelemetry::new();
            let mut latch = FaultLatch::new();
            let code = run_step(
                &mut state,
                &mut thermal,
                &zero_controls(),
                &config,
                &mut telemetry,
                &mut latch,
            );
            assert_eq!(code, 8);
            assert_eq!(state.position, before.position);
            assert_eq!(thermal.motor_temp_c, [20.0; 4]);
            assert_eq!(latch.physics_fault, 1);
        }
        let mut config = dummy_config();
        config.max_rpm = 0.0;
        let mut state = ffi_create_default_drone_state();
        state.position[2] = -100.0;
        let mut thermal = ThermalState::new();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let mut controls = zero_controls();
        controls.throttle = 1.0;
        let code = run_step(
            &mut state,
            &mut thermal,
            &controls,
            &config,
            &mut telemetry,
            &mut latch,
        );
        assert_eq!(code, 0);
        assert_eq!(telemetry.motor_thrusts, [0.0; 4]);
    }

    // Finding C: a NaN in the caller's motor_temp_c is rejected (code 8, latch), not carried on.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_nan_thermal_state_latches() {
        for bad in [f64::NAN, f64::INFINITY] {
            let mut state = ffi_create_default_drone_state();
            state.position[2] = -100.0;
            let mut thermal = ThermalState::new();
            thermal.motor_temp_c[2] = bad;
            let mut telemetry = DebugTelemetry::new();
            let mut latch = FaultLatch::new();
            let code = run_step(
                &mut state,
                &mut thermal,
                &zero_controls(),
                &dummy_config(),
                &mut telemetry,
                &mut latch,
            );
            assert_eq!(code, 8);
            assert_eq!(latch.physics_fault, 1);
            assert_eq!(thermal.motor_temp_c[0], 20.0);
            assert!(!thermal.motor_temp_c[2].is_finite());
        }
    }

    // Finding C: telemetry says thermal is unvalidated on every successful step. Over-temp flag:
    // dummy config, throttle 1.0 -> rpm 10000 -> I = (22.2 - 10) / 0.08 = 152.5 A,
    // P = 152.5^2 * 0.05 = 1162.8125 W, so dT/dt = P / C = 23.26 K/s and one 0.002 s step adds
    // about 0.0465 K (cooling removes about 0.0004 K). Start 119.99 C -> 120.036 C > 120: flag 1.
    // Start 20 C -> about 20.05 C: flag 0.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_ffi_step_reports_thermal_unvalidated_and_over_temp() {
        let mut controls = zero_controls();
        controls.throttle = 1.0;
        for (start_c, want_over) in [(119.99, 1), (20.0, 0)] {
            let mut state = ffi_create_default_drone_state();
            state.position[2] = -100.0;
            let mut thermal = ThermalState {
                motor_temp_c: [start_c; 4],
            };
            let mut telemetry = DebugTelemetry::new();
            let mut latch = FaultLatch::new();
            let code = run_step(
                &mut state,
                &mut thermal,
                &controls,
                &dummy_config(),
                &mut telemetry,
                &mut latch,
            );
            assert_eq!(code, 0);
            assert_eq!(telemetry.thermal_valid, 0);
            assert_eq!(telemetry.thermal_over_temp, want_over, "start {start_c}");
        }
    }

    #[cfg(feature = "onnx")]
    fn golden_words() -> Vec<u64> {
        // Scenario: 100 m up, ground velocity (3, -2, 1) m/s, yaw 30 deg, throttle 0.5, roll 0.1,
        // 500 steps of 0.002 s, analytic aero (no handles).
        let mut state = ffi_create_default_drone_state();
        state.position = [0.0, 0.0, -100.0];
        state.velocity = [3.0, -2.0, 1.0];
        let half = 15.0_f32.to_radians();
        state.orientation = [0.0, 0.0, half.sin(), half.cos()];
        state.angular_velocity = [0.1, 0.0, 0.0];
        let mut thermal = ThermalState::new();
        let mut controls = zero_controls();
        controls.throttle = 0.5;
        controls.roll = 0.1;
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        for _ in 0..500 {
            // SAFETY: all pointers are valid, aligned and unaliased for the call.
            let code = unsafe {
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    &WindInput::still(),
                    &EnvironmentInput::standard(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    &mut latch,
                    0.002,
                )
            };
            assert_eq!(code, 0);
        }
        let mut w: Vec<u64> = Vec::new();
        w.extend(state.position.iter().map(|x| x.to_bits()));
        w.extend(state.velocity.iter().map(|x| u64::from(x.to_bits())));
        w.extend(state.orientation.iter().map(|x| u64::from(x.to_bits())));
        w.extend(
            state
                .angular_velocity
                .iter()
                .map(|x| u64::from(x.to_bits())),
        );
        w.extend(thermal.motor_temp_c.iter().map(|x| x.to_bits()));
        w.extend(telemetry.aero_drag.iter().map(|x| u64::from(x.to_bits())));
        w.extend(telemetry.net_force.iter().map(|x| u64::from(x.to_bits())));
        w.extend(
            telemetry
                .motor_thrusts
                .iter()
                .map(|x| u64::from(x.to_bits())),
        );
        w
    }

    /// Bit patterns captured from the pre-wind build (interface v10, git HEAD 4a4659b) by running
    /// the identical `golden_words` scenario there. Order: position (3 x f64), velocity (3 x f32),
    /// orientation (4 x f32), angular velocity (3 x f32), motor temperatures (4 x f64),
    /// telemetry aero_drag (3), net_force (3), motor_thrusts (4) (f32 values in the low 32 bits).
    #[cfg(feature = "onnx")]
    const GOLDEN_V10: [u64; 27] = [
        0x40062903c31ec19e,
        0xbffa1dc47b754389,
        0xc0595337f4a92ef5,
        0x000000004012d6d6,
        0x00000000bf56ae45,
        0x00000000c05c2d48,
        0x000000003dd51620,
        0x000000003ce462ac,
        0x000000003e83be8f,
        0x000000003f75d698,
        0x000000003ea9d47a,
        0x00000000a083126d,
        0x0000000000000000,
        0x403958c64f7ca6c0,
        0x403a4668bd89c740,
        0x403958c64f7ca6c0,
        0x403a4668bd89c740,
        0x000000003b377a28,
        0x00000000bd840f58,
        0x000000003f230bd9,
        0x00000000bfc5d6fd,
        0x0000000040278487,
        0x00000000c06e651a,
        0x0000000040558368,
        0x00000000407a94e3,
        0x0000000040558368,
        0x00000000407a94e3,
    ];

    #[test]
    #[cfg(feature = "onnx")]
    fn test_wind_zero_wind_step_is_bit_identical_to_v10_build() {
        // Reference = the previous build (a pre-change binary), not a hand calculation: with zero
        // wind v_air = v_ground - 0 must change no bit over 500 steps (drag is nonzero here:
        // |v| = 3.7 m/s, so a wrong velocity path would show).
        assert_eq!(golden_words(), GOLDEN_V10.to_vec());
    }

    #[cfg(feature = "onnx")]
    fn step_with_wind(
        state: &mut DroneState,
        wind: &WindInput,
        telemetry: &mut DebugTelemetry,
        latch: &mut FaultLatch,
    ) -> i32 {
        let mut thermal = ThermalState::new();
        let controls = zero_controls();
        let config = dummy_config();
        // SAFETY: all pointers are valid, aligned and unaliased for the call.
        unsafe {
            ffi_step_physics(
                state,
                &mut thermal,
                &controls,
                wind,
                &EnvironmentInput::standard(),
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                telemetry,
                latch,
                0.002,
            )
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_wind_head_wind_at_hover_step_drag_is_minus_0_30625_n() {
        // Hover at 0 m altitude (rho = 1.2250), identity attitude, ground velocity 0, wind
        // toward south (-5, 0, 0): v_air = (+5, 0, 0). Drag = -0.5 * 1.2250 * 5^2 * Cd_x(1.0) *
        // A_x(0.02) = -0.30625 N along body X = north, i.e. it opposes the apparent air motion.
        // Still air gives exactly 0 (control).
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let mut still = ffi_create_default_drone_state();
        assert_eq!(
            step_with_wind(&mut still, &WindInput::still(), &mut telemetry, &mut latch),
            0
        );
        assert_eq!(telemetry.aero_drag, [0.0; 3]);

        let mut state = ffi_create_default_drone_state();
        let wind = WindInput {
            wind_ned_mps: [-5.0, 0.0, 0.0],
        };
        assert_eq!(
            step_with_wind(&mut state, &wind, &mut telemetry, &mut latch),
            0
        );
        assert!((f64::from(telemetry.aero_drag[0]) + 0.30625).abs() < 0.30625 * 1e-4 + 1e-6);
        assert_eq!(telemetry.aero_drag[1], 0.0);
        assert_eq!(telemetry.aero_drag[2], 0.0);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_wind_equal_to_vehicle_velocity_step_has_zero_drag() {
        // Ground velocity (3, -2, 1) and wind (3, -2, 1): v_air = 0 exactly, so drag is exactly 0
        // (without the wind the drag would be nonzero: the control below).
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let mut moving = ffi_create_default_drone_state();
        moving.velocity = [3.0, -2.0, 1.0];
        let mut no_wind = moving;
        assert_eq!(
            step_with_wind(
                &mut no_wind,
                &WindInput::still(),
                &mut telemetry,
                &mut latch
            ),
            0
        );
        assert!(telemetry.aero_drag[0] < 0.0);

        let wind = WindInput {
            wind_ned_mps: [3.0, -2.0, 1.0],
        };
        assert_eq!(
            step_with_wind(&mut moving, &wind, &mut telemetry, &mut latch),
            0
        );
        assert_eq!(telemetry.aero_drag, [0.0; 3]);
        // The integrator still advances the GROUND velocity: with zero thrust and zero drag the
        // horizontal force is exactly 0, so vN and vE are unchanged (3 and -2). Integrating the
        // air-relative velocity (0) instead would give 0 here.
        assert_eq!(moving.velocity[0], 3.0);
        assert_eq!(moving.velocity[1], -2.0);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_wind_invalid_is_code_12_and_writes_nothing() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 31.0, -31.0] {
            let mut state = ffi_create_default_drone_state();
            state.velocity = [1.0, 2.0, 3.0];
            let before_state = state;
            let mut telemetry = DebugTelemetry::new();
            telemetry.net_force = [4.0, 5.0, 6.0];
            telemetry.is_validated_envelope = 1;
            let before_telemetry = telemetry;
            let mut latch = FaultLatch::new();
            let wind = WindInput {
                wind_ned_mps: [0.0, bad, 0.0],
            };
            assert_eq!(
                step_with_wind(&mut state, &wind, &mut telemetry, &mut latch),
                12,
                "wind y = {bad}"
            );
            assert_eq!(state.position, before_state.position);
            assert_eq!(state.velocity, before_state.velocity);
            assert_eq!(telemetry.net_force, before_telemetry.net_force);
            assert_eq!(telemetry.is_validated_envelope, 1);
            assert_eq!(latch.physics_fault, 0);
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_wind_30_mps_component_is_accepted() {
        let mut state = ffi_create_default_drone_state();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let wind = WindInput {
            wind_ned_mps: [30.0, -30.0, 30.0],
        };
        assert_eq!(
            step_with_wind(&mut state, &wind, &mut telemetry, &mut latch),
            0
        );
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_wind_null_pointer_is_code_1_and_bad_dt_keeps_code_7() {
        let mut state = ffi_create_default_drone_state();
        let mut thermal = ThermalState::new();
        let controls = zero_controls();
        let config = dummy_config();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let bad_wind = WindInput {
            wind_ned_mps: [f64::NAN; 3],
        };
        // SAFETY: all non-null pointers are valid, aligned and unaliased for each call.
        unsafe {
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    ptr::null(),
                    &EnvironmentInput::standard(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    &mut latch,
                    0.002,
                ),
                1
            );
            // dt is validated before the wind, so an earlier rejection keeps its code.
            assert_eq!(
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    &bad_wind,
                    &EnvironmentInput::standard(),
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    &mut latch,
                    0.0,
                ),
                7
            );
        }
    }

    // ---- interface v12: environment ----------------------------------------------------------

    #[cfg(feature = "onnx")]
    fn step_with_env(
        state: &mut DroneState,
        wind: &WindInput,
        env: *const EnvironmentInput,
        throttle: f32,
        telemetry: &mut DebugTelemetry,
        latch: &mut FaultLatch,
    ) -> i32 {
        let mut thermal = ThermalState::new();
        let controls = ControlInputs {
            throttle,
            ..zero_controls()
        };
        let config = dummy_config();
        // SAFETY: every pointer is valid, aligned and unaliased for the call (`env` may be null on purpose).
        unsafe {
            ffi_step_physics(
                state,
                &mut thermal,
                &controls,
                wind,
                env,
                ptr::null_mut(),
                ptr::null_mut(),
                &config,
                telemetry,
                latch,
                0.002,
            )
        }
    }

    // Hand calc (python3, f64): site 3000 m, +9.5 K -> rho = 0.8780770469291939 kg/m3;
    // T(0.5) = 0.11 * rho * (0.5 * 10000 / 60)^2 * 0.25^4 = 2.6201300770999163 N per motor
    // (dummy config: C_T 0.11, D 0.25 m, 10000 rpm). Telemetry is f32: tolerance 1e-6 N.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_env_ffi_step_thrust_at_3000_m_site() {
        let mut state = ffi_create_default_drone_state();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let env = EnvironmentInput {
            site_altitude_m: 3000.0,
            temp_offset_c: 9.5,
        };
        assert_eq!(
            step_with_env(
                &mut state,
                &WindInput::still(),
                &env,
                0.5,
                &mut telemetry,
                &mut latch
            ),
            0
        );
        for t in telemetry.motor_thrusts {
            assert!((f64::from(t) - 2.6201300770999163).abs() < 1e-6, "t = {t}");
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_env_ffi_null_environment_is_code_1_and_writes_nothing() {
        let mut state = ffi_create_default_drone_state();
        let before = state;
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let code = step_with_env(
            &mut state,
            &WindInput::still(),
            ptr::null(),
            0.5,
            &mut telemetry,
            &mut latch,
        );
        assert_eq!(code, 1);
        assert_eq!(state.position, before.position);
        assert_eq!(telemetry.motor_thrusts, [0.0; 4]);
    }

    // Each bad environment gives code 13 and changes NOTHING (state, telemetry, latch).
    #[test]
    #[cfg(feature = "onnx")]
    fn test_env_ffi_invalid_environment_is_code_13_and_writes_nothing() {
        let bad = [
            EnvironmentInput {
                site_altitude_m: f64::NAN,
                temp_offset_c: 0.0,
            },
            EnvironmentInput {
                site_altitude_m: 0.0,
                temp_offset_c: f64::INFINITY,
            },
            EnvironmentInput {
                site_altitude_m: -1.0,
                temp_offset_c: 0.0,
            },
            EnvironmentInput {
                site_altitude_m: 5000.5,
                temp_offset_c: 0.0,
            },
            EnvironmentInput {
                site_altitude_m: 0.0,
                temp_offset_c: 40.5,
            },
            EnvironmentInput {
                site_altitude_m: 0.0,
                temp_offset_c: -40.5,
            },
        ];
        for env in bad {
            let mut state = ffi_create_default_drone_state();
            let before = state;
            let mut telemetry = DebugTelemetry::new();
            let mut latch = FaultLatch::new();
            let code = step_with_env(
                &mut state,
                &WindInput::still(),
                &env,
                0.5,
                &mut telemetry,
                &mut latch,
            );
            assert_eq!(
                code, 13,
                "env ({}, {})",
                env.site_altitude_m, env.temp_offset_c
            );
            assert_eq!(state.position, before.position);
            assert_eq!(state.velocity, before.velocity);
            assert_eq!(telemetry.motor_thrusts, [0.0; 4]);
            assert_eq!(latch.physics_fault, 0);
        }
    }

    // The environment is checked after the wind: a bad wind AND a bad environment report code 12.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_env_ffi_wind_error_is_reported_before_environment_error() {
        let mut state = ffi_create_default_drone_state();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let bad_wind = WindInput {
            wind_ned_mps: [31.0, 0.0, 0.0],
        };
        let bad_env = EnvironmentInput {
            site_altitude_m: -5.0,
            temp_offset_c: 0.0,
        };
        assert_eq!(
            step_with_env(
                &mut state,
                &bad_wind,
                &bad_env,
                0.5,
                &mut telemetry,
                &mut latch
            ),
            12
        );
    }

    // Total height outside the ISA range (site 5000 + vehicle 6001 = 11001 m) is the existing code 6.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_env_ffi_total_height_above_isa_range_is_code_6() {
        let mut state = ffi_create_default_drone_state();
        state.position[2] = -6001.0;
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let env = EnvironmentInput {
            site_altitude_m: 5000.0,
            temp_offset_c: 0.0,
        };
        assert_eq!(
            step_with_env(
                &mut state,
                &WindInput::still(),
                &env,
                0.5,
                &mut telemetry,
                &mut latch
            ),
            6
        );
        assert_eq!(latch.physics_fault, 0);
    }

    // Thermal ambient follows the offset: throttle 0, zero RPM, 5 s of 0.002 s steps at +15 K.
    // The motor starts at 20 C (ThermalState::new) and relaxes toward 35 C: after 2500 steps
    // t = 5 s the temperature must have risen above 20 C and stay below 35 C.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_env_ffi_thermal_ambient_follows_temp_offset() {
        let mut state = ffi_create_default_drone_state();
        let mut thermal = ThermalState::new();
        let controls = zero_controls();
        let config = dummy_config();
        let env = EnvironmentInput {
            site_altitude_m: 0.0,
            temp_offset_c: 15.0,
        };
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        for _ in 0..2500 {
            // SAFETY: all pointers are valid, aligned and unaliased for the call.
            let code = unsafe {
                ffi_step_physics(
                    &mut state,
                    &mut thermal,
                    &controls,
                    &WindInput::still(),
                    &env,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &config,
                    &mut telemetry,
                    &mut latch,
                    0.002,
                )
            };
            assert_eq!(code, 0);
            state.position = [0.0, 0.0, -100.0];
            state.velocity = [0.0; 3];
        }
        for t in thermal.motor_temp_c {
            assert!(t > 20.0 && t < 35.0, "t = {t}");
        }
    }

    // Analytic drag through the step uses the environment: ground velocity (4, 0, 0), still air, identity
    // attitude. Drag = -0.5 rho 16 Cd_x(1.0) A_x(0.02); the +20 K / standard ratio is
    // 1.1455047361519344 / 1.2250122659906946 = 0.9350965438909621 (python3). Telemetry is f32: rel 1e-6.
    #[test]
    #[cfg(feature = "onnx")]
    fn test_env_ffi_step_drag_uses_density() {
        let drag_x = |temp_offset_c: f64| {
            let mut state = ffi_create_default_drone_state();
            state.velocity = [4.0, 0.0, 0.0];
            let mut telemetry = DebugTelemetry::new();
            let mut latch = FaultLatch::new();
            let env = EnvironmentInput {
                site_altitude_m: 0.0,
                temp_offset_c,
            };
            let code = step_with_env(
                &mut state,
                &WindInput::still(),
                &env,
                0.0,
                &mut telemetry,
                &mut latch,
            );
            assert_eq!(code, 0);
            f64::from(telemetry.aero_drag[0])
        };
        let standard = drag_x(0.0);
        // -0.5 * 1.2250122659906946 * 16 * 1.0 * 0.02 = -0.19600196255851114
        assert!(
            (standard + 0.196_001_962_558_511_14).abs() < 1e-6,
            "standard = {standard}"
        );
        assert!((drag_x(20.0) / standard - 0.9350965438909621).abs() < 1e-6);
    }

    // ---------------------------------------------------------------- powertrain FFI (interface v13)

    #[cfg(feature = "onnx")]
    fn pt_vehicle() -> VehicleConfig {
        // Matches powertrain::tests::ref_cfg: D 0.3048, Ct 0.100256, kappa = Cp D / (2 pi Ct) = 0.015423.
        VehicleConfig {
            mass_kg: 1.536,
            inertia_kgm2: [0.030_135, 0.030_81, 0.055_933],
            motor_pos_m: [
                [0.2, 0.2, -0.052],
                [0.2, -0.2, -0.052],
                [-0.2, 0.2, -0.052],
                [-0.2, -0.2, -0.052],
            ],
            motor_spin: [1.0, -1.0, -1.0, 1.0],
            prop_diameter_m: 0.3048,
            thrust_coeff: 0.100_256,
            torque_coeff: 0.015_423,
            max_rpm: 8981.406,
            hover_throttle: 0.398,
        }
    }

    fn pt_temp_file(name: &str, text: &str) -> std::ffi::CString {
        let path = std::env::temp_dir().join(format!("pt_ffi_{}_{name}.json", std::process::id()));
        std::fs::write(&path, text).expect("write temp file");
        std::ffi::CString::new(path.to_str().expect("utf-8 path")).expect("no NUL")
    }

    #[test]
    fn test_pt_ffi_load_config_codes() {
        let good = pt_temp_file("good", &crate::powertrain::tests::ref_json());
        let bad_json = pt_temp_file("badjson", "{ not json");
        let invalid = pt_temp_file(
            "invalid",
            &crate::powertrain::tests::ref_json()
                .replace("\"esc_efficiency\":0.95", "\"esc_efficiency\":2.0"),
        );
        let missing = std::ffi::CString::new("/nonexistent/powertrain.json").unwrap();
        let mut out = crate::powertrain::tests::ref_cfg();
        out.motor_kv_rpm_per_v = -1.0; // sentinel: overwritten only on success
        // SAFETY: pointers are valid for the call; the null cases are the point of the test.
        unsafe {
            assert_eq!(ffi_load_powertrain_config(ptr::null(), &mut out), 1);
            assert_eq!(
                ffi_load_powertrain_config(good.as_ptr(), ptr::null_mut()),
                1
            );
            assert_eq!(ffi_load_powertrain_config(missing.as_ptr(), &mut out), 2);
            assert_eq!(ffi_load_powertrain_config(bad_json.as_ptr(), &mut out), 3);
            assert_eq!(ffi_load_powertrain_config(invalid.as_ptr(), &mut out), 4);
            assert_eq!(out.motor_kv_rpm_per_v, -1.0);
            assert_eq!(ffi_load_powertrain_config(good.as_ptr(), &mut out), 0);
        }
        assert_eq!(out.motor_kv_rpm_per_v, 700.0);
        assert_eq!(out.ocv_points, 8);
    }

    #[test]
    fn test_pt_ffi_init_state_codes() {
        let cfg = crate::powertrain::tests::ref_cfg();
        let mut bad_cfg = cfg;
        bad_cfg.esc_efficiency = 0.0;
        let mut out = crate::powertrain::initial_state(0.5, 10.0).unwrap();
        // SAFETY: pointers are valid for the call; the null cases are the point of the test.
        unsafe {
            assert_eq!(
                ffi_init_powertrain_state(ptr::null(), 20.0, 1.0, &mut out),
                1
            );
            assert_eq!(
                ffi_init_powertrain_state(&cfg, 20.0, 1.0, ptr::null_mut()),
                1
            );
            assert_eq!(ffi_init_powertrain_state(&bad_cfg, 20.0, 1.0, &mut out), 4);
            assert_eq!(ffi_init_powertrain_state(&cfg, 20.0, 1.5, &mut out), 5);
            assert_eq!(ffi_init_powertrain_state(&cfg, f64::NAN, 1.0, &mut out), 5);
            assert_eq!(out.soc, 0.5); // untouched by every failure
            assert_eq!(ffi_init_powertrain_state(&cfg, 25.0, 0.8, &mut out), 0);
        }
        assert_eq!(
            (
                out.soc,
                out.battery_temp_c,
                out.motor_winding_c,
                out.charge_used_ah
            ),
            (0.8, 25.0, [25.0; 4], 0.0)
        );
    }

    #[test]
    fn test_pt_ffi_bench_point_codes_and_values() {
        let cfg = crate::powertrain::tests::ref_cfg();
        let mut bad_cfg = cfg;
        bad_cfg.prop_ct = 0.0;
        let mut out = types::PowertrainBenchOut {
            rpm: -1.0,
            motor_current_a: 0.0,
            torque_nm: 0.0,
            thrust_n: 0.0,
            shaft_w: 0.0,
            motor_electrical_w: 0.0,
            copper_loss_w: 0.0,
            noload_loss_w: 0.0,
            esc_loss_w: 0.0,
            battery_current_a: 0.0,
            battery_power_w: 0.0,
            system_efficiency: 0.0,
            advance_ratio: 0.0,
            ct: 0.0,
            cp: 0.0,
            extrapolated: 0,
            reserved: 0,
        };
        let rho = 1.225_012_265_990_694_6;
        // SAFETY: pointers are valid for the call; the null cases are the point of the test.
        unsafe {
            assert_eq!(
                ffi_powertrain_bench_point(ptr::null(), 1.0, 14.8, rho, 0.0, &mut out),
                1
            );
            assert_eq!(
                ffi_powertrain_bench_point(&cfg, 1.0, 14.8, rho, 0.0, ptr::null_mut()),
                1
            );
            assert_eq!(
                ffi_powertrain_bench_point(&bad_cfg, 1.0, 14.8, rho, 0.0, &mut out),
                4
            );
            assert_eq!(
                ffi_powertrain_bench_point(&cfg, 1.2, 14.8, rho, 0.0, &mut out),
                5
            );
            assert_eq!(
                ffi_powertrain_bench_point(&cfg, 1.0, -1.0, rho, 0.0, &mut out),
                5
            );
            assert_eq!(
                ffi_powertrain_bench_point(&cfg, 1.0, 14.8, 0.0, 0.0, &mut out),
                5
            );
            assert_eq!(out.rpm, -1.0);
            assert_eq!(
                ffi_powertrain_bench_point(&cfg, 1.0, 14.8, rho, 0.0, &mut out),
                0
            );
        }
        // the reference point of the model: 14.8 V, full duty -> 8981.4057 rpm, 27.353 A in the motor, 23.752 N
        assert!((out.rpm - 8_981.405_667_8).abs() < 1e-4);
        assert!((out.motor_current_a - 27.353_062).abs() < 1e-5);
        assert!((out.thrust_n - 23.751_814).abs() < 1e-5);
        assert!((out.battery_current_a - 404.825_32 / 0.95 / 14.8).abs() < 1e-4);
    }

    #[cfg(feature = "onnx")]
    struct PtRig {
        state: DroneState,
        pts: types::PowertrainState,
        telemetry: DebugTelemetry,
        pt_tel: types::PowertrainTelemetry,
        latch: FaultLatch,
        cfg: types::PowertrainConfig,
        vehicle: VehicleConfig,
        controls: ControlInputs,
        wind: WindInput,
        env: EnvironmentInput,
    }

    #[cfg(feature = "onnx")]
    impl PtRig {
        fn new(throttle: f32) -> Self {
            let mut state = ffi_create_default_drone_state();
            state.position = [0.0, 0.0, -10.0]; // 10 m above the origin (NED z down)
            Self {
                state,
                pts: crate::powertrain::initial_state(1.0, 20.0).unwrap(),
                telemetry: DebugTelemetry::new(),
                pt_tel: types::PowertrainTelemetry::no_data(),
                latch: FaultLatch::new(),
                cfg: crate::powertrain::tests::ref_cfg(),
                vehicle: pt_vehicle(),
                controls: ControlInputs {
                    throttle,
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: 0.0,
                },
                wind: WindInput::still(),
                env: EnvironmentInput::standard(),
            }
        }

        fn step(&mut self, dt: f64) -> i32 {
            // SAFETY: all pointers are valid, aligned and unaliased for the call; the handles are null.
            unsafe {
                ffi_step_powertrain(
                    &mut self.state,
                    &mut self.pts,
                    &self.controls,
                    &self.wind,
                    &self.env,
                    ptr::null_mut(),
                    ptr::null_mut(),
                    &self.vehicle,
                    &self.cfg,
                    &mut self.telemetry,
                    &mut self.pt_tel,
                    &mut self.latch,
                    dt,
                )
            }
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_null_pointers() {
        let mut rig = PtRig::new(0.4);
        // SAFETY: only the null checks run (each call has exactly one null argument), nothing is dereferenced.
        unsafe {
            macro_rules! call {
                ($($arg:expr),*) => { ffi_step_powertrain($($arg),*) };
            }
            let (s, p, c, w, e) = (
                &mut rig.state as *mut _,
                &mut rig.pts as *mut _,
                &rig.controls as *const _,
                &rig.wind as *const _,
                &rig.env as *const _,
            );
            let (v, pc, t, pt, l) = (
                &rig.vehicle as *const _,
                &rig.cfg as *const _,
                &mut rig.telemetry as *mut _,
                &mut rig.pt_tel as *mut _,
                &mut rig.latch as *mut _,
            );
            let n = ptr::null_mut();
            assert_eq!(
                call!(ptr::null_mut(), p, c, w, e, n, n, v, pc, t, pt, l, 0.002),
                1
            );
            assert_eq!(
                call!(s, ptr::null_mut(), c, w, e, n, n, v, pc, t, pt, l, 0.002),
                1
            );
            assert_eq!(
                call!(s, p, ptr::null(), w, e, n, n, v, pc, t, pt, l, 0.002),
                1
            );
            assert_eq!(
                call!(s, p, c, ptr::null(), e, n, n, v, pc, t, pt, l, 0.002),
                1
            );
            assert_eq!(
                call!(s, p, c, w, ptr::null(), n, n, v, pc, t, pt, l, 0.002),
                1
            );
            assert_eq!(
                call!(s, p, c, w, e, n, n, ptr::null(), pc, t, pt, l, 0.002),
                1
            );
            assert_eq!(
                call!(s, p, c, w, e, n, n, v, ptr::null(), t, pt, l, 0.002),
                1
            );
            assert_eq!(
                call!(s, p, c, w, e, n, n, v, pc, ptr::null_mut(), pt, l, 0.002),
                1
            );
            assert_eq!(
                call!(s, p, c, w, e, n, n, v, pc, t, ptr::null_mut(), l, 0.002),
                1
            );
            assert_eq!(
                call!(s, p, c, w, e, n, n, v, pc, t, pt, ptr::null_mut(), 0.002),
                1
            );
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_success_writes_everything_consistently() {
        let mut rig = PtRig::new(0.4);
        assert_eq!(rig.step(0.002), 0);
        let t = rig.pt_tel;
        assert_eq!(t.step_valid, 1);
        // the legacy telemetry mirrors the powertrain numbers (f32 on the UE side)
        for i in 0..4 {
            assert_eq!(rig.telemetry.motor_rpms[i], t.motor_rpm[i] as f32);
            assert_eq!(rig.telemetry.motor_thrusts[i], t.motor_thrust_n[i] as f32);
            assert_eq!(
                rig.telemetry.motor_temperatures_c[i],
                t.motor_winding_c[i] as f32
            );
        }
        assert_eq!(rig.telemetry.thermal_valid, 0); // every thermal input is ASSUMED
        assert_eq!(rig.telemetry.thermal_over_temp, 0);
        assert_eq!(rig.latch.physics_fault, 0);
        assert!(t.battery_current_a > 0.0 && rig.pts.battery_current_a == t.battery_current_a);
        assert!(rig.pts.soc < 1.0);
        // the vehicle moved: thrust acts, so the velocity changed from zero
        assert!(rig.state.velocity.iter().any(|v| *v != 0.0));
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_first_step_thrust_is_the_bench_point_at_open_circuit_voltage() {
        // Step 1 has no sag (previous current 0): each motor sees duty x 4 x 4.20 = duty x 16.8 V. At 10 m the density is the ISA value.
        let mut rig = PtRig::new(0.4);
        assert_eq!(rig.step(0.002), 0);
        let rho = aero::air_density(10.0, &EnvironmentInput::standard()).unwrap();
        // the control throttle is an f32, so the duty the step sees is 0.4_f32 widened to f64
        let bench = powertrain::bench_point(&rig.cfg, f64::from(0.4_f32), 16.8, rho, 0.0).unwrap();
        assert!((rig.pt_tel.motor_thrust_n[0] - bench.thrust_n).abs() < 1e-9 * bench.thrust_n);
        assert!((rig.pt_tel.motor_rpm[0] - bench.rpm).abs() < 1e-6);
        assert!((rig.pt_tel.motor_current_a[0] - bench.motor_current_a).abs() < 1e-9);
        // four equal motors: the pack current is four times the bench battery current
        assert!((rig.pt_tel.battery_current_a - 4.0 * bench.battery_current_a).abs() < 1e-9);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_rejections_write_nothing() {
        type Mutator = Box<dyn Fn(&mut PtRig)>;
        let cases: Vec<(&str, i32, Mutator)> = vec![
            (
                "dt zero",
                7,
                Box::new(|r| {
                    let _ = r;
                }),
            ),
            (
                "quaternion not unit",
                11,
                Box::new(|r| r.state.orientation = [0.0, 0.0, 0.0, 1.01]),
            ),
            (
                "wind too strong",
                12,
                Box::new(|r| r.wind.wind_ned_mps = [31.0, 0.0, 0.0]),
            ),
            (
                "site altitude too high",
                13,
                Box::new(|r| r.env.site_altitude_m = 6000.0),
            ),
            (
                "powertrain efficiency zero",
                14,
                Box::new(|r| r.cfg.esc_efficiency = 0.0),
            ),
            (
                "propeller diameter differs from the vehicle",
                14,
                Box::new(|r| r.cfg.prop_diameter_m = 0.3),
            ),
            (
                "thrust coefficient differs from the vehicle",
                14,
                Box::new(|r| r.vehicle.thrust_coeff = 0.11),
            ),
            (
                "kappa differs from the vehicle",
                14,
                Box::new(|r| r.vehicle.torque_coeff = 0.02),
            ),
            (
                "thermal network too fast for dt",
                14,
                Box::new(|r| r.cfg.esc_c_j_per_k = 0.001),
            ),
            ("soc nan", 15, Box::new(|r| r.pts.soc = f64::NAN)),
            ("soc above one", 15, Box::new(|r| r.pts.soc = 1.2)),
            (
                "motor temperature absurd",
                15,
                Box::new(|r| r.pts.motor_case_c[1] = 5000.0),
            ),
        ];
        for (name, code, mutate) in cases {
            let mut rig = PtRig::new(0.4);
            mutate(&mut rig);
            rig.telemetry.net_force = [4.0, 5.0, 6.0];
            let (state_before, pts_before, tel_before) = (rig.state, rig.pts, rig.pt_tel);
            let dt = if name == "dt zero" { 0.0 } else { 0.002 };
            assert_eq!(rig.step(dt), code, "{name}");
            assert_eq!(rig.state.position, state_before.position, "{name}");
            assert_eq!(rig.state.velocity, state_before.velocity, "{name}");
            assert_eq!(rig.pts.soc.to_bits(), pts_before.soc.to_bits(), "{name}");
            assert_eq!(rig.pts.motor_case_c, pts_before.motor_case_c, "{name}");
            assert_eq!(rig.telemetry.net_force, [4.0, 5.0, 6.0], "{name}"); // legacy telemetry untouched
            assert_eq!(rig.pt_tel.step_valid, tel_before.step_valid, "{name}");
            assert_eq!(rig.latch.physics_fault, 0, "{name}");
        }
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_dt_limit_is_ten_milliseconds() {
        // 0.01 s is the largest accepted step (code 7 above it), the same limit as ffi_step_physics
        let mut rig = PtRig::new(0.4);
        assert_eq!(rig.step(0.01), 0);
        let mut rig = PtRig::new(0.4);
        assert_eq!(rig.step(0.010_000_1), 7);
        assert_eq!(rig.step(f64::NAN), 7);
        assert_eq!(rig.step(-0.002), 7);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_cooling_uses_the_air_relative_speed() {
        // Same ground speed (zero), different wind: the wind cools the ESC and the battery (they see airspeed only, no prop wash).
        let temps = |wind_x: f64| {
            let mut rig = PtRig::new(0.4);
            rig.wind.wind_ned_mps = [wind_x, 0.0, 0.0];
            for _ in 0..3000 {
                assert_eq!(rig.step(0.002), 0);
            }
            (rig.pts.esc_temp_c[0], rig.pts.battery_temp_c)
        };
        let (esc_still, batt_still) = temps(0.0);
        let (esc_wind, batt_wind) = temps(20.0);
        assert!(esc_wind < esc_still, "{esc_wind} {esc_still}");
        assert!(batt_wind < batt_still, "{batt_wind} {batt_still}");
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_zero_air_relative_speed_cools_like_hover_in_still_air() {
        // Moving at 15 m/s in a 15 m/s tailwind: the AIR-relative speed is 0, so the thermal cooling (and the drag) must equal the
        // hovering still-air case bit for bit. The same motion in still air has 15 m/s of airspeed and cools better.
        let run = |velocity_x: f32, wind_x: f64| {
            let mut rig = PtRig::new(0.4);
            rig.state.velocity = [velocity_x, 0.0, 0.0];
            rig.wind.wind_ned_mps = [wind_x, 0.0, 0.0];
            for _ in 0..500 {
                assert_eq!(rig.step(0.002), 0);
            }
            (
                rig.pts.esc_temp_c[0],
                rig.pts.battery_temp_c,
                rig.pts.motor_case_c[0],
            )
        };
        let hover = run(0.0, 0.0);
        let with_tailwind = run(15.0, 15.0);
        let through_still_air = run(15.0, 0.0);
        assert_eq!(with_tailwind.0.to_bits(), hover.0.to_bits());
        assert_eq!(with_tailwind.1.to_bits(), hover.1.to_bits());
        assert_eq!(with_tailwind.2.to_bits(), hover.2.to_bits());
        assert!(through_still_air.0 < hover.0 && through_still_air.1 < hover.1);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_ambient_includes_the_environment_temperature_offset() {
        // A +30 K day moves the ambient from 20 to 50 C: with identical heat inputs the nodes end warmer.
        let temps = |offset: f64| {
            let mut rig = PtRig::new(0.4);
            rig.env.temp_offset_c = offset;
            for _ in 0..3000 {
                assert_eq!(rig.step(0.002), 0);
            }
            (
                rig.pts.esc_temp_c[0],
                rig.pts.battery_temp_c,
                rig.pts.motor_case_c[0],
            )
        };
        let cool = temps(0.0);
        let hot = temps(30.0);
        assert!(
            hot.0 > cool.0 && hot.1 > cool.1 && hot.2 > cool.2,
            "{hot:?} {cool:?}"
        );
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_divergence_and_altitude_reset_both_telemetries() {
        // NaN throttle -> NaN duty -> code 8: latch set, both telemetries "no data", state unchanged
        let mut rig = PtRig::new(f32::NAN);
        let before = rig.state;
        let pts_before = rig.pts;
        assert_eq!(rig.step(0.002), 8);
        assert_eq!(rig.latch.physics_fault, 1);
        assert!(
            rig.pt_tel.battery_voltage_v.is_nan()
                && rig.pt_tel.motor_rpm.iter().all(|v| v.is_nan())
        );
        assert_eq!(rig.pt_tel.step_valid, 0);
        assert_eq!(rig.telemetry.physics_fault, 1);
        assert_eq!(rig.state.position, before.position);
        assert_eq!(rig.pts.soc, pts_before.soc);
        // altitude outside the ISA range -> code 6: telemetry reset, latch untouched
        let mut rig = PtRig::new(0.4);
        rig.state.position = [0.0, 0.0, 5.0];
        rig.pt_tel.step_valid = 1;
        assert_eq!(rig.step(0.002), 6);
        assert_eq!(rig.latch.physics_fault, 0);
        assert_eq!(rig.pt_tel.step_valid, 0);
        assert!(rig.pt_tel.motor_current_a.iter().all(|v| v.is_nan()));
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_unknown_handle_code_9() {
        let mut rig = PtRig::new(0.4);
        let before = rig.state;
        // SAFETY: all pointers are valid; the handle address is never dereferenced (the registry rejects it).
        let code = unsafe {
            ffi_step_powertrain(
                &mut rig.state,
                &mut rig.pts,
                &rig.controls,
                &rig.wind,
                &rig.env,
                never_loaded_handle(),
                ptr::null_mut(),
                &rig.vehicle,
                &rig.cfg,
                &mut rig.telemetry,
                &mut rig.pt_tel,
                &mut rig.latch,
                0.002,
            )
        };
        assert_eq!(code, 9);
        assert_eq!(rig.state.position, before.position);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_stop_mapping_of_powertrain_errors() {
        use crate::powertrain::PowertrainError as E;
        assert!(matches!(
            powertrain_stop(E::InvalidConfig),
            StepStop::Silent(14)
        ));
        assert!(matches!(
            powertrain_stop(E::ThermalStep),
            StepStop::Silent(14)
        ));
        assert!(matches!(
            powertrain_stop(E::InvalidState),
            StepStop::Silent(15)
        ));
        assert!(matches!(
            powertrain_stop(E::InvalidInput),
            StepStop::Diverged
        ));
        assert!(matches!(
            powertrain_stop(E::NonFinite),
            StepStop::NoData(16)
        ));
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_is_bit_identical_twice() {
        let run = || {
            let mut rig = PtRig::new(0.45);
            for _ in 0..300 {
                assert_eq!(rig.step(0.002), 0);
            }
            (
                rig.state.position.map(f64::to_bits),
                rig.state.velocity.map(f32::to_bits),
                rig.pts.soc.to_bits(),
                rig.pts.motor_winding_c.map(f64::to_bits),
                rig.pt_tel.battery_current_a.to_bits(),
            )
        };
        assert_eq!(run(), run());
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_legacy_step_is_untouched_by_the_new_path() {
        // The legacy export still behaves exactly as before: same call, same inputs -> thermal still the (flagged) legacy model.
        let mut state = ffi_create_default_drone_state();
        state.position = [0.0, 0.0, -10.0];
        let mut thermal = ThermalState::new();
        let mut telemetry = DebugTelemetry::new();
        let mut latch = FaultLatch::new();
        let controls = ControlInputs {
            throttle: 0.4,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        let code = run_step(
            &mut state,
            &mut thermal,
            &controls,
            &pt_vehicle(),
            &mut telemetry,
            &mut latch,
        );
        assert_eq!(code, 0);
        assert_eq!(telemetry.thermal_valid, 0);
        // legacy rpm = throttle x max_rpm = 0.4 x 8981.406
        assert!((f64::from(telemetry.motor_rpms[0]) - 0.4 * 8981.406).abs() < 0.01);
    }

    // ---------------------------------------------------------------- propeller table FFI (interface v14)

    #[test]
    fn test_pt_ffi_propeller_coefficients_codes_and_values() {
        let cfg = crate::powertrain::tests::hand_cfg();
        let mut bad_cfg = cfg;
        bad_cfg.prop_cp_table[0][1] = 0.0;
        let mut out = types::PropCoefficients {
            ct: -1.0,
            cp: -1.0,
            extrapolated: -1,
            reserved: 0,
        };
        // SAFETY: pointers are valid for the call; the null cases are the point of the test.
        unsafe {
            assert_eq!(
                ffi_propeller_coefficients(ptr::null(), 0.1, 3000.0, &mut out),
                1
            );
            assert_eq!(
                ffi_propeller_coefficients(&cfg, 0.1, 3000.0, ptr::null_mut()),
                1
            );
            assert_eq!(
                ffi_propeller_coefficients(&bad_cfg, 0.1, 3000.0, &mut out),
                4
            );
            assert_eq!(
                ffi_propeller_coefficients(&cfg, f64::NAN, 3000.0, &mut out),
                5
            );
            assert_eq!(
                ffi_propeller_coefficients(&cfg, 0.1, f64::INFINITY, &mut out),
                5
            );
            assert_eq!(
                ffi_propeller_coefficients(&cfg, f64::NEG_INFINITY, 3000.0, &mut out),
                5
            );
            assert_eq!(out.ct, -1.0); // untouched by every failure
            // hand values of the two-curve test table: J 0.1, rpm 3000 -> Ct 0.10, Cp 0.03975, inside
            assert_eq!(ffi_propeller_coefficients(&cfg, 0.1, 3000.0, &mut out), 0);
        }
        assert!(
            (out.ct - 0.10).abs() < 1e-12
                && (out.cp - 0.03975).abs() < 1e-12
                && out.extrapolated == 0
        );
        // J 0.55 is beyond the shorter bracketing curve: Ct (0.01 + 0.04) / 2 = 0.025, flagged
        // SAFETY: valid pointers.
        assert_eq!(
            unsafe { ffi_propeller_coefficients(&cfg, 0.55, 3000.0, &mut out) },
            0
        );
        assert!((out.ct - 0.025).abs() < 1e-12 && out.extrapolated == 1);
        // the static model returns the constants whatever the point
        let st = crate::powertrain::tests::ref_cfg();
        // SAFETY: valid pointers.
        assert_eq!(
            unsafe { ffi_propeller_coefficients(&st, 9.0, 1.0, &mut out) },
            0
        );
        assert_eq!((out.ct, out.extrapolated), (0.100_256, 0));
    }

    #[test]
    fn test_pt_ffi_bench_point_with_inflow_reports_j_and_the_coefficients() {
        let cfg = crate::powertrain::tests::wide_cfg();
        let rho = 1.225_012_265_990_694_6;
        let mut still = types::PowertrainBenchOut {
            rpm: 0.0,
            motor_current_a: 0.0,
            torque_nm: 0.0,
            thrust_n: 0.0,
            shaft_w: 0.0,
            motor_electrical_w: 0.0,
            copper_loss_w: 0.0,
            noload_loss_w: 0.0,
            esc_loss_w: 0.0,
            battery_current_a: 0.0,
            battery_power_w: 0.0,
            system_efficiency: 0.0,
            advance_ratio: -1.0,
            ct: -1.0,
            cp: -1.0,
            extrapolated: -1,
            reserved: 0,
        };
        let mut climb = still;
        // SAFETY: valid pointers for every call.
        unsafe {
            assert_eq!(
                ffi_powertrain_bench_point(&cfg, 1.0, 14.8, rho, 0.0, &mut still),
                0
            );
            assert_eq!(
                ffi_powertrain_bench_point(&cfg, 1.0, 14.8, rho, 8.0, &mut climb),
                0
            );
            assert_eq!(
                ffi_powertrain_bench_point(&cfg, 1.0, 14.8, rho, f64::NAN, &mut climb),
                5
            );
        }
        assert_eq!(still.advance_ratio, 0.0);
        assert!(climb.thrust_n < still.thrust_n && climb.rpm > still.rpm);
        let n = climb.rpm / 60.0;
        assert!((climb.advance_ratio - 8.0 / (n * 0.3048)).abs() < 1e-9);
        // Ct at J = 0 is 0.10 in the table; with inflow it fell
        assert!((still.ct - 0.10).abs() < 1e-12 && climb.ct < 0.10);
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_axial_inflow_comes_from_the_attitude_and_velocity() {
        let first_step = |velocity: [f32; 3], wind: [f64; 3]| {
            let mut rig = PtRig::new(0.5);
            rig.cfg = crate::powertrain::tests::wide_cfg();
            rig.state.velocity = velocity;
            rig.wind.wind_ned_mps = wind;
            assert_eq!(rig.step(0.002), 0);
            rig.pt_tel
        };
        let hover = first_step([0.0; 3], [0.0; 3]);
        // climbing at 8 m/s: axial inflow 8 m/s -> J = 8 / (n D) > 0, less thrust than hovering
        let climb = first_step([0.0, 0.0, -8.0], [0.0; 3]);
        let n = climb.motor_rpm[0] / 60.0;
        assert!((climb.motor_advance_ratio[0] - 8.0 / (n * 0.3048)).abs() < 1e-9);
        assert!(climb.motor_thrust_n[0] < hover.motor_thrust_n[0]);
        // horizontal flight with a level rotor has no axial component: bit-identical to hover (edgewise flow is not modelled)
        let sideways = first_step([15.0, 0.0, 0.0], [0.0; 3]);
        assert_eq!(
            sideways.motor_thrust_n.map(f64::to_bits),
            hover.motor_thrust_n.map(f64::to_bits)
        );
        // a vertical wind of 8 m/s upward on a hovering vehicle is the same relative flow as descending at 8 m/s ... and the opposite sign
        // of the climb: the air moves up through the rotor, J < 0, which the table holds at J = 0 and flags
        let updraft = first_step([0.0; 3], [0.0, 0.0, -8.0]);
        assert!(updraft.motor_advance_ratio[0] < 0.0);
        assert_eq!(updraft.prop_extrapolated, 0b1111);
        assert_eq!(
            updraft.motor_thrust_n.map(f64::to_bits),
            hover.motor_thrust_n.map(f64::to_bits)
        ); // held at the J = 0 value
    }

    #[test]
    #[cfg(feature = "onnx")]
    fn test_pt_ffi_step_reports_the_extrapolated_time() {
        let mut rig = PtRig::new(0.37);
        rig.cfg = crate::powertrain::tests::range_cfg();
        for _ in 0..10 {
            assert_eq!(rig.step(0.002), 0);
        }
        // hover duty: about 3500 rpm, J = 0: inside 3040 .. 6018.5 -> nothing flagged
        assert_eq!(
            (
                rig.pt_tel.prop_extrapolated,
                rig.pts.prop_extrapolated_time_s
            ),
            (0, 0.0)
        );
        rig.controls.throttle = 0.1; // idle: below the first curve
        for k in 1..=10 {
            assert_eq!(rig.step(0.002), 0);
            assert_eq!(rig.pt_tel.prop_extrapolated, 0b1111);
            assert!(
                (rig.pts.prop_extrapolated_time_s - 0.002 * f64::from(k)).abs() < 1e-12,
                "{k}"
            );
        }
    }
}
