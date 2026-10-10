// Tier: F
#pragma once

#include "Modules/ModuleManager.h"
#include "Generated/drone_ffi.h"

class DRONEPHYSICSBRIDGE_API FDronePhysicsBridgeModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    static DroneState CreateDefaultState();
    static ThermalState CreateDefaultThermalState();
    static bool ResetState(DroneState* State);
    static bool LoadVehicleConfig(const FString& Path, VehicleConfig* OutConfig);

    // Wind is NED m/s, all zeros = still air; Rust rejects a non-finite or >30 m/s component (code 12).
    // Latch is caller-owned and must be zero-initialized by the caller. Rust sets it on a NaN/Inf step and never clears it.
    static bool StepPhysics(DroneState* State, ThermalState* Thermal, const ControlInputs* Inputs, const WindInput* Wind, const EnvironmentInput* Environment, SurrogateHandle* AeroHandle, SurrogateHandle* FeaHandle, const VehicleConfig* Config, DebugTelemetry* OutTelemetry, FaultLatch* Latch, double DeltaTime);

    static FVector NedToUnrealWorld(const double NedPosition[3]);
    // Free vector (force N, velocity m/s): axis mapping only, no unit scale. (x, y, z) -> (x, y, -z).
    static FVector NedToUnrealVector(const float NedVector[3]);
    static FQuat NedToUnrealQuat(const float NedQuat[4]);
};