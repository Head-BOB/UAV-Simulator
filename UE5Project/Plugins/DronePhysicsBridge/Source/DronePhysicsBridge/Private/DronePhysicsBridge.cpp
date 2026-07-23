#include "DronePhysicsBridge.h"
#include "Modules/ModuleManager.h"

// Tells Unreal Engine this is the main class for the plugin module
IMPLEMENT_MODULE(FDronePhysicsBridgeModule, DronePhysicsBridge)

void FDronePhysicsBridgeModule::StartupModule()
{
    // Task 10 Requirement: Safety Checks!
    // This is the most important code in the wrapper. It prevents silent memory corruption.

    int32 ExpectedVersion = 1; // This matches the 1 we returned in lib.rs
    int32 RustVersion = ffi_get_interface_version();

    if (RustVersion != ExpectedVersion)
    {
        // Fatal error crashes the engine on purpose. Better to crash than silently corrupt data!
        UE_LOG(LogTemp, Fatal, TEXT("Rust FFI Version Mismatch! Expected %d, got %d"), ExpectedVersion, RustVersion);
    }

    int32 ExpectedSize = sizeof(DroneState);
    int32 RustSize = ffi_get_drone_state_size();

    if (RustSize != ExpectedSize)
    {
        UE_LOG(LogTemp, Fatal, TEXT("Rust struct size mismatch! C++ thinks it is %d bytes, Rust thinks it is %d bytes."), ExpectedSize, RustSize);
    }

    UE_LOG(LogTemp, Log, TEXT("Drone Physics Rust Bridge successfully loaded and verified!"));
}

void FDronePhysicsBridgeModule::ShutdownModule()
{
    // Called when the engine shuts down. We don't need to do anything here for now.
}

DroneState FDronePhysicsBridgeModule::CreateDefaultState()
{
    return ffi_create_default_drone_state();
}

bool FDronePhysicsBridgeModule::ResetState(DroneState* State)
{
    if (!State) return false;

    // Returns 0 on success (based on our Rust implementation)
    return ffi_reset_drone_state(State) == 0;
}

bool FDronePhysicsBridgeModule::StepPhysics(DroneState* State, const ControlInputs* Inputs, float DeltaTime)
{
    if (!State || !Inputs) return false;

    return ffi_step_physics(State, Inputs, DeltaTime) == 0;
}

bool FDronePhysicsBridgeModule::GetDebugTelemetry(DebugTelemetry* OutTelemetry)
{
    if (!OutTelemetry)
    {
        return false;
    }

    // Returns 0 on successful lock and read from the Rust FFI boundary
    return ffi_get_debug_telemetry(OutTelemetry) == 0;
}

