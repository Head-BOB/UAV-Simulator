#ifndef DRONE_FFI_H
#define DRONE_FFI_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Represents the drone's current physical condition.
 * #[repr(C)] guarantees this struct has the exact same memory layout as a C/C++ struct.
 */
typedef struct DroneState {
  /**
   * World-space X, Y, Z position of the center of gravity (meters)
   */
  float position[3];
  /**
   * World-space linear velocity (meters per second)
   */
  float velocity[3];
  /**
   * Rotation as a unit quaternion, stored in order (w, x, y, z)
   */
  float orientation[4];
  /**
   * Angular velocity in the body frame (radians per second)
   */
  float angular_velocity[3];
} DroneState;

/**
 * Represents what the pilot/controller is commanding.
 */
typedef struct ControlInputs {
  /**
   * Commanded throttle, range 0.0 (none) to 1.0 (full)
   */
  float throttle;
  /**
   * Commanded roll input, range -1.0 to 1.0
   */
  float roll;
  /**
   * Commanded pitch input, range -1.0 to 1.0
   */
  float pitch;
  /**
   * Commanded yaw input, range -1.0 to 1.0
   */
  float yaw;
} ControlInputs;

/**
 * Telemetry data exposed exclusively for UE5 On-Screen Display (OSD) and debug visualization.
 * Must be synced every physics tick.
 */
typedef struct DebugTelemetry {
  /**
   * Net thrust vector across all motors in world space (Newtons)
   */
  float net_thrust[3];
  /**
   * Aerodynamic drag force vector in world space (Newtons)
   */
  float aero_drag[3];
  /**
   * Gravity vector applied to the drone (Newtons)
   */
  float gravity[3];
  /**
   * Resulting net force vector (Newtons)
   */
  float net_force[3];
  /**
   * Individual motor thrust outputs (Newtons)
   */
  float motor_thrusts[4];
  /**
   * Individual motor RPMs
   */
  float motor_rpms[4];
} DebugTelemetry;

/**
 * ffi_get_interface_version
 * Returns the current layout version to UE5 to prevent memory corruption on mismatch.
 */
int32_t ffi_get_interface_version(void);

/**
 * ffi_get_drone_state_size
 * Allows UE5 to verify the byte size of DroneState during module initialization.
 */
int32_t ffi_get_drone_state_size(void);

/**
 * ffi_create_default_drone_state
 */
struct DroneState ffi_create_default_drone_state(void);

/**
 * ffi_reset_drone_state
 */
int32_t ffi_reset_drone_state(struct DroneState *state);

/**
 * ffi_step_physics
 * Main execution block for the fixed-timestep RK4 physics pipeline.
 */
int32_t ffi_step_physics(struct DroneState *state, const struct ControlInputs *controls, float dt);

/**
 * ffi_get_debug_telemetry
 * Retrieves the most recent physics telemetry data for the UE5 OSD.
 */
int32_t ffi_get_debug_telemetry(struct DebugTelemetry *out_telemetry);

#endif  /* DRONE_FFI_H */
