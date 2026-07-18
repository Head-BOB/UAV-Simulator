#pragma once

#include "Modules/ModuleManager.h"

#include "Generated/drone_ffi.h"

class FDronePhysicsBridgeModule : public IModuleInterface
{
public:
    // IModuleInterface implementation (required by Unreal Engine)
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    static DroneState CreateDefaultState();
    static bool ResetState(DroneState* State);
    static bool StepPhysics(DroneState* State, const ControlInputs* Inputs, float DeltaTime);
};