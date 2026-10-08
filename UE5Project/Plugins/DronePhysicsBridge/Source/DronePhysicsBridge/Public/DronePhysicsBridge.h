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

    static bool StepPhysics(DroneState* State, ThermalState* Thermal, const ControlInputs* Inputs, SurrogateHandle* AeroHandle, SurrogateHandle* FeaHandle, const VehicleConfig* Config, double DeltaTime);
    static bool GetDebugTelemetry(DebugTelemetry* OutTelemetry);

    static FVector NedToUnrealWorld(const double NedPosition[3]);
    static FQuat NedToUnrealQuat(const float NedQuat[4]);
};