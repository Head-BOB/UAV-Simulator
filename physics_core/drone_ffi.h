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
 * ffi_get_interface_version
 */
int32_t ffi_get_interface_version(void);

/**
 * ffi_get_drone_state_size
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
 */
int32_t ffi_step_physics(struct DroneState *state, const struct ControlInputs *controls, float dt);

#endif  /* DRONE_FFI_H */
