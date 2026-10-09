#ifndef DRONE_FFI_H
#define DRONE_FFI_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Return code: a panic was caught inside the FFI function, or the handle was faulted by an
 * earlier panic. The call's effect is undefined except that no memory was corrupted; the
 * caller should treat the run as invalid. Telemetry is not written.
 */
#define FFI_CODE_PANIC 99

/**
 * Return code: an internal lock was poisoned and has been recovered. The call did nothing;
 * calling again works.
 */
#define FFI_CODE_LOCK_RECOVERED 98

/**
 * Engineering bound on the model input width, not a physics limit. The widest planned
 * input today is 4 (aero); raise only by a reviewed change.
 */
#define MAX_INPUT_WIDTH 64

/**
 * Length of `SurrogateQueryResult::predicted_values`.
 */
#define PREDICTED_SLOTS 3

/**
 * Number of inputs the aero surrogate takes: airspeed, angle of attack, sideslip.
 */
#define AERO_INPUT_WIDTH 3

/**
 * Number of inputs the FEA surrogate takes: 4 arm loads, 3 direction components, 1 temperature.
 */
#define FEA_INPUT_WIDTH 8

/**
 * True only when every thermal constant above has a sourced, verified value (item B-4).
 * Stays `false` until a human closes B-4; reported as `DebugTelemetry::thermal_valid`.
 */
#define THERMAL_CONSTANTS_VALIDATED false

/**
 * Motor winding over-temperature limit, deg C. PLACEHOLDER, no source.
 * TODO(HUMAN-VERIFY): replace with the motor datasheet limit (B-4). Not a `VehicleConfig`
 * field on purpose: that would change the config hash that every model is bound to.
 */
#define MOTOR_TEMP_LIMIT_C 120.0

/**
 * Largest accepted deviation of a state quaternion's norm from 1 at the FFI boundary.
 * f32 storage keeps a unit quaternion within about 2.4e-7 of norm 1 and the integrator
 * re-normalizes every step, so 1e-3 only trips on a caller bug (zero, scaled or garbage
 * quaternion), never on rounding. TODO(HUMAN-VERIFY): the tolerance is an engineering choice.
 */
#define QUAT_NORM_TOLERANCE 1.0e-3

/**
 * Handle to an active ONNX surrogate model session.
 */
typedef struct SurrogateHandle SurrogateHandle;

/**
 * Represents the drone's current physical condition.
 *
 * #[repr(C)] guarantees this struct has the exact same memory layout on
 * both sides of the FFI boundary.
 *
 * COORDINATE FRAME (see ADR-001, Section 1.3 of the Standards Framework):
 * position is expressed in a local North-East-Down (NED) frame, in meters,
 * relative to a fixed WGS84 geodetic origin defined once per simulation
 * scenario. Do not reinterpret this as UE5 world-space — conversion
 * happens ONLY inside the wrapper class described in Section 1.4.
 */
typedef struct DroneState {
  /**
   * North-East-Down position relative to the scenario's WGS84 origin.
   *
   * # Units
   * Meters. f64: see ADR-001 — f32 loses sub-meter precision at realistic mission ranges.
   */
  double position[3];
  /**
   * Local-frame linear velocity.
   *
   * # Units
   * Meters per second. f32 is sufficient: magnitude never grows large enough to lose useful precision.
   */
  float velocity[3];
  /**
   * Rotation as a unit quaternion, stored in order (x, y, z, w).
   *
   * # Units
   * Unitless quaternion. f32 is sufficient: components are always within [-1.0, 1.0].
   */
  float orientation[4];
  /**
   * Angular velocity in the body frame.
   *
   * # Units
   * Radians per second. f32 is sufficient for the same reason as velocity.
   */
  float angular_velocity[3];
} DroneState;

/**
 * State container for the thermal and electrical simulation.
 */
typedef struct ThermalState {
  /**
   * Internal winding temperatures for each motor.
   *
   * # Units
   * Degrees Celsius.
   */
  double motor_temp_c[4];
} ThermalState;

/**
 * Represents what the pilot/controller is commanding.
 */
typedef struct ControlInputs {
  /**
   * Commanded throttle.
   *
   * # Units
   * Normalized range 0.0 (none) to 1.0 (full).
   */
  float throttle;
  /**
   * Commanded roll input.
   *
   * # Units
   * Normalized range -1.0 to 1.0.
   */
  float roll;
  /**
   * Commanded pitch input.
   *
   * # Units
   * Normalized range -1.0 to 1.0.
   */
  float pitch;
  /**
   * Commanded yaw input.
   *
   * # Units
   * Normalized range -1.0 to 1.0.
   */
  float yaw;
} ControlInputs;

/**
 * Physical properties and layout of the specific airframe being simulated.
 */
typedef struct VehicleConfig {
  /**
   * Total mass of the vehicle.
   *
   * # Units
   * Kilograms.
   */
  double mass_kg;
  /**
   * Moment of inertia about the center of mass (Ixx, Iyy, Izz).
   *
   * # Units
   * Kilogram-square meters.
   */
  double inertia_kgm2[3];
  /**
   * Positions of the 4 motors in the body frame.
   *
   * # Units
   * Meters.
   */
  double motor_pos_m[4][3];
  /**
   * Spin direction of each motor (+1.0 or -1.0), in the same index order as `motor_pos_m`.
   * +1.0 means the reaction torque on the airframe is about body +Z (nose right), i.e. the
   * propeller turns CCW seen from above.
   *
   * # Units
   * Unitless multiplier.
   */
  double motor_spin[4];
  /**
   * Diameter of the propellers.
   *
   * # Units
   * Meters.
   */
  double prop_diameter_m;
  /**
   * Propeller thrust coefficient.
   *
   * # Units
   * Unitless.
   */
  double thrust_coeff;
  /**
   * Propeller yaw-torque coefficient: reaction torque per unit of thrust (`kappa`).
   * Yaw torque on the airframe is `sum(motor_spin * torque_coeff * thrust_n)`.
   *
   * # Units
   * Newton-metres per Newton of thrust (N*m/N, equivalent to metres).
   */
  double torque_coeff;
  /**
   * Maximum rotational speed of the motors.
   *
   * # Units
   * Revolutions per minute.
   */
  double max_rpm;
  /**
   * Throttle level required to maintain steady hover.
   *
   * Derived at load from mass, C_T, propeller diameter, max RPM and ISA
   * sea-level density (`aero::hover_throttle`); never read from JSON.
   *
   * # Units
   * Normalized range 0.0 to 1.0.
   */
  double hover_throttle;
} VehicleConfig;

/**
 * Telemetry data exposed exclusively for UE5 On-Screen Display (OSD) and debug visualization.
 *
 * Must be synced every physics tick.
 */
typedef struct DebugTelemetry {
  /**
   * Net thrust vector across all motors in world space.
   *
   * # Units
   * Newtons.
   */
  float net_thrust[3];
  /**
   * Aerodynamic drag force vector in world space.
   *
   * # Units
   * Newtons.
   */
  float aero_drag[3];
  /**
   * Gravity vector applied to the drone.
   *
   * # Units
   * Newtons.
   */
  float gravity[3];
  /**
   * Resulting net force vector.
   *
   * # Units
   * Newtons.
   */
  float net_force[3];
  /**
   * Individual motor thrust outputs.
   *
   * # Units
   * Newtons.
   */
  float motor_thrusts[4];
  /**
   * Individual motor RPMs.
   *
   * # Units
   * Revolutions per minute.
   */
  float motor_rpms[4];
  /**
   * Temperature of each motor winding.
   *
   * # Units
   * Degrees Celsius.
   */
  float motor_temperatures_c[4];
  /**
   * Structural failure margin from the FEA surrogate.
   *
   * # Units
   * Unitless multiplier. Values strictly less than 1.0 indicate structural failure.
   */
  float structural_safety_margin;
  /**
   * Aggregated envelope validation flag for all surrogate queries in the current tick.
   *
   * # Units
   * Unitless boolean flag (1 if all queries are valid, 0 if any query extrapolated).
   */
  int32_t is_validated_envelope;
  /**
   * Flag indicating if an un-trained stub model is currently driving the physics.
   *
   * # Units
   * Unitless boolean flag (1 if a surrogate handle passed to this step came
   * from a model whose `kind` is not `trained`, 0 otherwise).
   */
  int32_t stub_loaded;
  /**
   * Source of aerodynamic data. 0 = none (step rejected), 1 = analytic model, no surrogate
   * handle was passed, 2 = surrogate, 3 = a surrogate handle was passed but its query FAILED
   * and the analytic model was used instead (the two are never reported as the same value).
   */
  int32_t aero_source;
  /**
   * Source of structural data. 0 = none (no handle passed, or step rejected), 2 = surrogate,
   * 3 = a surrogate handle was passed but its query FAILED (margin is NaN). There is no
   * analytic structural fallback, so 1 is not used.
   */
  int32_t fea_source;
  /**
   * Copy of the caller's `FaultLatch`: 1 from the faulting step until the caller clears it.
   * Data produced while this is 1 is invalid.
   */
  int32_t physics_fault;
  /**
   * Count of fixed-timestep execution cycles dropped to prevent a "spiral of death".
   */
  int32_t dropped_time_events;
  /**
   * Indicates physics constants lacking validated sources are in use.
   */
  int32_t uncalibrated_flags;
  /**
   * 1 only when every thermal constant has a sourced, verified value. 0 means the thermal
   * model is UNVALIDATED: `motor_temperatures_c` and `thermal_over_temp` are arithmetic on
   * ASSUMED constants (item B-4) and must not be used as evidence. Always 0 until B-4 closes.
   */
  int32_t thermal_valid;
  /**
   * 1 if any motor winding temperature exceeds `thermal::MOTOR_TEMP_LIMIT_C` after this step
   * (limit is a placeholder, TODO(HUMAN-VERIFY)); 0 otherwise. Only meaningful when
   * `thermal_valid` is 1.
   */
  int32_t thermal_over_temp;
} DebugTelemetry;

/**
 * Caller-owned latched fault flag for `ffi_step_physics` (WP-4.4).
 *
 * The caller allocates it, zero-initializes it, and passes it to every step.
 * Rust sets `physics_fault` to 1 when a step produces a NaN or Inf state and
 * NEVER clears it. Only the caller clears it, on purpose (for example at the
 * start of a new run). Rust holds no copy, so two vehicles or two tests do
 * not share fault state.
 *
 * Data produced while the latch is set is invalid. It must not be used as
 * validation or pass/fail evidence.
 */
typedef struct FaultLatch {
  /**
   * 0 = no fault latched, 1 = a NaN/Inf divergence occurred since the caller last cleared it.
   */
  int32_t physics_fault;
} FaultLatch;

/**
 * Standardized return payload for any trained surrogate model query.
 */
typedef struct SurrogateQueryResult {
  /**
   * The primary output of the surrogate model (e.g., drag vector or scalar safety margin).
   *
   * # Units
   * Context-dependent based on the specific model queried.
   */
  float predicted_values[3];
  /**
   * Statistical confidence metric from the Gaussian Process Regression.
   *
   * # Units
   * Context-dependent variance.
   */
  float uncertainty;
  /**
   * Evaluates if the queried condition fell within the model's training data envelope.
   *
   * # Units
   * Unitless boolean flag (1 for true, 0 for false).
   */
  int32_t in_validated_envelope;
} SurrogateQueryResult;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * Returns the current layout version to UE5 to prevent memory corruption on mismatch.
 *
 * # Safety
 * Safe to call at any time.
 */
int32_t ffi_get_interface_version(void);

/**
 * Allows UE5 to verify the byte size of DroneState during module initialization.
 *
 * # Safety
 * Safe to call at any time.
 */
int32_t ffi_get_drone_state_size(void);

/**
 * Instantiates a default, zeroed drone state with an identity quaternion.
 * If a panic is caught the state is all NaN, which the next step rejects loudly (code 8).
 *
 * # Safety
 * Safe to call at any time.
 */
struct DroneState ffi_create_default_drone_state(void);

/**
 * Resets the provided state to default values.
 *
 * # Returns
 * `0` ok, `1` null pointer, `99` panic caught.
 *
 * # Safety
 * * `state` must be a valid, aligned, and mutable pointer to a DroneState instance.
 * * The memory must not be concurrently accessed by another thread.
 */
int32_t ffi_reset_drone_state(struct DroneState *state);

/**
 * Main execution block for the fixed-timestep RK4 physics pipeline.
 *
 * Executes modules in strict order: Aero -> Structural -> Thermal -> Mixer -> Integrator.
 *
 * # Units
 * * `dt` - Timestep in seconds.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer (nothing is written)
 * * `6` - Aero model error (drag or thrust), including altitude outside ISA 0..=11000 m
 * * `7` - Invalid dt (NaN, <= 0 or > 0.01)
 * * `8` - Divergence or invalid input: a NaN/Inf in `config`, `thermal_state.motor_temp_c` or
 *   `state.orientation`; `config.mass_kg <= 0`, an inertia <= 0 or `max_rpm < 0` (re-checked
 *   every step because C++ may fill the struct by hand); or a NaN/Inf in the new state or new
 *   thermal state. `state` and `thermal_state` are left/restored to their previous values and
 *   `fault_latch` is set to 1.
 * * `9` - A non-null surrogate handle is not live (never loaded, already unloaded, foreign),
 *   or `aero_handle` and `fea_handle` are the same non-null handle
 * * `11` - `state.orientation` is finite but its norm differs from 1 by more than
 *   `types::QUAT_NORM_TOLERANCE` (rejected, never silently rescaled)
 * * `98` - an internal lock was poisoned and recovered; nothing was done, call again
 * * `99` - a panic was caught, or a handle was faulted by an earlier panic; the run is invalid
 *
 * # Behavior on a failed step
 * * `state` and `thermal_state` are always unchanged.
 * * Codes 1, 7, 9, 11, 98 and 99 are rejected before any computation: NOTHING is written
 *   (telemetry and latch keep their previous values).
 * * Codes 6 and 8 fail during the tick: `out_telemetry` is overwritten with "no data" (margin
 *   NaN, `is_validated_envelope` 0, sources 0, `thermal_valid` 0, `physics_fault` = latch value,
 *   so it is never left holding the previous tick's numbers).
 * * `fault_latch` is set to 1 ONLY by code 8. Code 6 (altitude outside the ISA range) is a
 *   scenario error, not a divergence, and leaves the latch as it was.
 *
 * # Fault latch
 * `fault_latch` is caller-owned and must be zero-initialized by the caller. Rust sets it to 1
 * on a code-8 step and never clears it; later steps keep running from the restored state and
 * report `physics_fault = 1` in telemetry. The caller clears it on purpose (for example at
 * the start of a new run). Data produced while it is set is invalid and must not be used as
 * validation or pass/fail evidence.
 *
 * # Safety
 * * `state` must be a valid, aligned, mutable pointer to a DroneState.
 * * `thermal_state` must be a valid, aligned, mutable pointer to a ThermalState.
 * * `controls` must be a valid, aligned, immutable pointer to ControlInputs.
 * * `aero_handle` and `fea_handle` may be null. If non-null, must be valid SurrogateHandles.
 * * `config` must be a valid, aligned, immutable pointer to a VehicleConfig.
 * * `out_telemetry` must be a valid, aligned, mutable pointer to a DebugTelemetry struct.
 * * `fault_latch` must be a valid, aligned, mutable pointer to a FaultLatch.
 * * Pointers must not alias or be subject to concurrent mutation.
 */
int32_t ffi_step_physics(struct DroneState *state,
                         struct ThermalState *thermal_state,
                         const struct ControlInputs *controls,
                         struct SurrogateHandle *aero_handle,
                         struct SurrogateHandle *fea_handle,
                         const struct VehicleConfig *config,
                         struct DebugTelemetry *out_telemetry,
                         struct FaultLatch *fault_latch,
                         double dt);

/**
 * Loads a trained ONNX surrogate model and returns an opaque handle to C++.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer, invalid UTF-8 path, or the path is not a readable file
 * * `2` - Missing provenance metadata
 * * `3` - Geometry hash mismatch
 * * `4` - Engine/ONNX initialization failure (including a missing ONNX Runtime library)
 * * `5` - Stub model rejected (allow_stub_models feature not enabled)
 * * `6` - Model inputs/outputs are not the supported shapes
 * * `8` - Vehicle config hash mismatch (model `config_hash` differs from the compiled `vehicle_config.json`)
 * * `98` - Handle registry lock was poisoned and recovered; the model is dropped, `out_handle`
 *   is untouched, call again (replaces the old code 7)
 * * `99` - A panic was caught; `out_handle` is untouched
 *
 * A model whose `kind` is not `trained` (a stub) loads only with the `allow_stub_models`
 * feature, and its queries always report `in_validated_envelope = 0`.
 *
 * # Safety
 * * `path` must be a valid, null-terminated C string.
 * * `out_handle` must be a valid, aligned, mutable pointer to a pointer.
 */
int32_t ffi_load_surrogate_model(const char *path,
                                 struct SurrogateHandle **out_handle);

/**
 * Queries a loaded surrogate model.
 *
 * `input_count` must equal the model's declared input width; no slice is built from it
 * before that check, so a huge value cannot read out of bounds. The registry lock is NOT held
 * during inference, only this handle's own lock.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer or negative `input_count`
 * * `2` - `handle` is not a live handle (never loaded, already unloaded, foreign)
 * * `3` - `input_count` differs from the model's input width
 * * `4` - Engine/ONNX inference error
 * * `5` - An input value is NaN or infinite
 * * `6` - Malformed model output (wrong count, non-finite, envelope flag not 0/1)
 * * `98` - Registry lock was poisoned and recovered; nothing was done, call again
 * * `99` - A panic was caught, or this handle was faulted by an earlier panic (unload it)
 *
 * A stub model always reports `in_validated_envelope = 0`.
 *
 * # Safety
 * * `handle` must be a pointer created by `ffi_load_surrogate_model` (checked against the
 *   live registry before use).
 * * `inputs` must point to an array of exactly `input_count` floats.
 * * `out_result` must be a valid, aligned pointer.
 */
int32_t ffi_query_surrogate(struct SurrogateHandle *handle,
                            const float *inputs,
                            int32_t input_count,
                            struct SurrogateQueryResult *out_result);

/**
 * Unloads a surrogate model and frees its memory.
 *
 * Waits for any query or step still using the handle, then frees it.
 *
 * # Returns
 * * `0` - Freed
 * * `1` - Null pointer
 * * `2` - `handle` is not live (never loaded or already unloaded); nothing is freed
 * * `98` - Registry lock was poisoned and recovered; nothing was freed, call again
 * * `99` - A panic was caught while freeing
 *
 * # Safety
 * * `handle` must be a pointer created by `ffi_load_surrogate_model` (checked against the
 *   live registry before it is freed).
 * * The caller must not use `handle` after this function returns 0.
 */
int32_t ffi_unload_surrogate_model(struct SurrogateHandle *handle);

/**
 * Loads a vehicle configuration file.
 *
 * # Returns
 * * `0` - Success
 * * `1` - Null pointer or Invalid UTF-8 path
 * * `2` - File IO Error
 * * `3` - JSON Parse Error
 * * `4` - Invalid Mass
 * * `5` - Invalid Inertia
 * * `6` - Invalid Motor Spin
 * * `7` - Invalid Motor Position (non-finite)
 * * `8` - Invalid Propeller Diameter
 * * `9` - Invalid Thrust Coefficient
 * * `10` - Invalid Torque Coefficient
 * * `11` - Invalid Max RPM
 * * `12` - Invalid Hover Throttle (must be in (0, 1])
 * * `99` - A panic was caught
 *
 * # Safety
 * * `path` must be a valid, null-terminated C string.
 * * `out_config` must be a valid, aligned, mutable pointer to a VehicleConfig.
 */
int32_t ffi_load_vehicle_config(const char *path, struct VehicleConfig *out_config);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* DRONE_FFI_H */
