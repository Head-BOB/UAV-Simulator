#pragma once

#include "Modules/ModuleManager.h"
#include "Generated/drone_ffi.h"

class FDronePhysicsBridgeModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    static DroneState CreateDefaultState();
    static bool ResetState(DroneState* State);
    static bool StepPhysics(DroneState* State, const ControlInputs* Inputs, SurrogateHandle* AeroHandle, SurrogateHandle* FeaHandle, float DeltaTime);
    static bool GetDebugTelemetry(DebugTelemetry* OutTelemetry);
    static FVector NedToUnrealWorld(const double NedPosition[3]);
};