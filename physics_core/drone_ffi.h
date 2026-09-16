#ifndef DRONE_FFI_H
#define DRONE_FFI_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

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
   * Rotation as a unit quaternion, stored in order (w, x, y, z).
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
} DebugTelemetry;

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
 *
 * # Safety
 * Safe to call at any time.
 */
struct DroneState ffi_create_default_drone_state(void);

/**
 * Resets the provided state to default values.
 *
 * # Safety
 * * `state` must be a valid, aligned, and mutable pointer to a DroneState instance.
 * * The memory must not be concurrently accessed by another thread.
 */
int32_t ffi_reset_drone_state(struct DroneState *state);

/**
 * Main execution block for the fixed-timestep RK4 physics pipeline.
 *
 * # Units
 * * `dt` - Timestep in seconds.
 *
 * # Safety
 * * `state` must be a valid, aligned, mutable pointer to a DroneState.
 * * `controls` must be a valid, aligned, immutable pointer to ControlInputs.
 * * Pointers must not alias or be subject to concurrent mutation.
 */
int32_t ffi_step_physics(struct DroneState *state, const struct ControlInputs *controls, float dt);

/**
 * Retrieves the most recent physics telemetry data for the UE5 OSD.
 *
 * # Safety
 * * `out_telemetry` must be a valid, aligned, and mutable pointer to a DebugTelemetry struct.
 */
int32_t ffi_get_debug_telemetry(struct DebugTelemetry *out_telemetry);

#endif  /* DRONE_FFI_H */
