#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DroneState {
	float position[3];
	float velocity[3];
	float orientation[4];
	float angular_velocity[3];
} DroneState;

typedef struct ControlInputs {
	float throttle;
	float roll;
	float pitch;
	float yaw;
} ControlInputs;

int32_t ffi_get_interface_version(void);

int32_t ffi_get_drone_state_size(void);

DroneState ffi_create_default_drone_state(void);

int32_t ffi_reset_drone_state(DroneState *state);

int32_t ffi_step_physics(DroneState *state, const ControlInputs *controls, float dt);

#ifdef __cplusplus
}
#endif