#ifndef DRONE_FFI_H
#define DRONE_FFI_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

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
DroneState ffi_create_default_drone_state(void);

/**
 * ffi_reset_drone_state
 */
int32_t ffi_reset_drone_state(DroneState *state);

/**
 * ffi_step_physics
 */
int32_t ffi_step_physics(DroneState *state, const ControlInputs *controls, float dt);

#endif  /* DRONE_FFI_H */
