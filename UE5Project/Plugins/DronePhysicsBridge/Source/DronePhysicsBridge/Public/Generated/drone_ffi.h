#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Represents the drone's current physical condition.
 *
 * position is expressed in a local North-East-Down (NED) frame, in meters,
 * relative to a fixed WGS84 geodetic origin.
 */
typedef struct DroneState {
	double position[3];
	float velocity[3];
	float orientation[4];
	float angular_velocity[3];
} DroneState;

/**
 * \brief Represents what the pilot/controller is commanding.
 */
typedef struct ControlInputs {
	float throttle;
	float roll;
	float pitch;
	float yaw;
} ControlInputs;

/**
 * \brief Returns the current layout version to UE5.
 * \return Integer version number.
 */
int32_t ffi_get_interface_version(void);

/**
 * \brief Returns the byte size of DroneState.
 * \return Size in bytes.
 */
int32_t ffi_get_drone_state_size(void);

/**
 * \brief Instantiates a default, zeroed drone state.
 * \return A DroneState struct with identity orientation.
 */
DroneState ffi_create_default_drone_state(void);

/**
 * \brief Resets the provided state to default values.
 * \param state Pointer to the DroneState to reset.
 * \return 0 on success, 1 on null pointer.
 * \pre state must be non-null and previously validated by the caller.
 */
int32_t ffi_reset_drone_state(DroneState *state);

/**
 * \brief Main execution block for the fixed-timestep RK4 physics pipeline.
 * \param state Pointer to the current DroneState.
 * \param controls Pointer to the current ControlInputs.
 * \param dt Timestep in seconds.
 * \return 0 on success, 1 on null pointer.
 * \pre state and controls must be non-null and previously validated by the caller.
 */
int32_t ffi_step_physics(DroneState *state, const ControlInputs *controls, float dt);

/**
 * \brief Telemetry data exposed exclusively for UE5 On-Screen Display (OSD).
 */
typedef struct DebugTelemetry {
	float net_thrust[3];
	float aero_drag[3];
	float gravity[3];
	float net_force[3];
	float motor_thrusts[4];
	float motor_rpms[4];
} DebugTelemetry;

/**
 * \brief Retrieves the most recent physics telemetry data.
 * \param out_telemetry Pointer to the DebugTelemetry struct to populate.
 * \return 0 on success, 1 on null pointer.
 * \pre out_telemetry must be non-null.
 */
int32_t ffi_get_debug_telemetry(DebugTelemetry *out_telemetry);

#ifdef __cplusplus
}
#endif