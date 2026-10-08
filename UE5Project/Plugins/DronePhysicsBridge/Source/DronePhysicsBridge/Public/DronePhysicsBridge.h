#pragma once

#include "Modules/ModuleManager.h"
#include "Generated/drone_ffi.h"

/**
 * \brief Module interface and static FFI gateway bridging Rust physics_core to Unreal Engine 5.
 */
class DRONEPHYSICSBRIDGE_API FDronePhysicsBridgeModule : public IModuleInterface
{
public:
    /**
     * \brief Initializes the module and validates FFI interface version and ABI struct sizes against Rust runtime.
     */
    virtual void StartupModule() override;

    /**
     * \brief Cleans up and unloads the physics bridge module.
     */
    virtual void ShutdownModule() override;

    /**
     * \brief Creates a baseline DroneState instance initialized with zeroed translation and identity orientation.
     *
     * \return DroneState Default initialized state in NED coordinates.
     */
    static DroneState CreateDefaultState();

    /**
     * \brief Resets an active DroneState structure back to scenario default initial conditions.
     *
     * \param State Mutable pointer to the DroneState struct to be reset.
     * \return bool True if reset succeeded, false if State pointer was null.
     * \pre State must be non-null.
     */
    static bool ResetState(DroneState* State);

    /**
     * \brief Advances physics simulation forward by DeltaTime using 4th-order Runge-Kutta integration.
     *
     * \param State Pointer to mutable drone physical state in NED frame.
     * \param Inputs Pointer to pilot control inputs (throttle, roll, pitch, yaw).
     * \param AeroHandle Optional pointer to active aerodynamic ONNX surrogate session.
     * \param FeaHandle Optional pointer to active structural FEA ONNX surrogate session.
     * \param DeltaTime Simulation timestep duration in seconds.
     * \return bool True if integration succeeded, false if State or Inputs was null or FFI returned an error code.
     * \pre State and Inputs must be non-null and valid.
     */
    static bool StepPhysics(DroneState* State, const ControlInputs* Inputs, SurrogateHandle* AeroHandle, SurrogateHandle* FeaHandle, double DeltaTime);

    /**
     * \brief Queries the most recent snapshot of intermediate physics forces, moments, and surrogate telemetry.
     *
     * \param OutTelemetry Pointer to caller-allocated DebugTelemetry struct to populate.
     * \return bool True if telemetry was successfully retrieved, false if OutTelemetry was null or unavailable.
     * \pre OutTelemetry must be non-null.
     */
    static bool GetDebugTelemetry(DebugTelemetry* OutTelemetry);

    /**
     * \brief Converts a 3D position vector from NED meters to Unreal Engine world coordinates (centimeters).
     *
     * Scales X and Y by 100.0 (m to cm) and inverts Z (-Z * 100.0) to convert Down to Up.
     *
     * \param NedPosition 3-element double array [North, East, Down] in meters.
     * \return FVector Position in Unreal world space (X=North, Y=East, Z=Up) in centimeters.
     */
    static FVector NedToUnrealWorld(const double NedPosition[3]);

    /**
     * \brief Converts a 3D force, velocity, or direction vector from NED coordinates to Unreal Engine coordinate space.
     *
     * Inverts the Z axis (Down to Up) without distance scaling, preserving physical units (e.g. Newtons, m/s).
     *
     * \param NedVector 3-element float array [X, Y, Z] in NED frame.
     * \return FVector Vector in Unreal frame (X, Y, -Z).
     */
    static FVector NedForceToUnreal(const float NedVector[3]);
};