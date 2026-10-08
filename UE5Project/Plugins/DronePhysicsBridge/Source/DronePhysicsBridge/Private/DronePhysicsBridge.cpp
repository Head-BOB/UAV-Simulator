#include "DronePhysicsBridge.h"
#include "Modules/ModuleManager.h"

IMPLEMENT_MODULE(FDronePhysicsBridgeModule, DronePhysicsBridge)

void FDronePhysicsBridgeModule::StartupModule()
{
    int32 ExpectedVersion = 1;
    int32 RustVersion = ffi_get_interface_version();

    if (RustVersion != ExpectedVersion)
    {
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
}

DroneState FDronePhysicsBridgeModule::CreateDefaultState()
{
    return ffi_create_default_drone_state();
}

bool FDronePhysicsBridgeModule::ResetState(DroneState* State)
{
    if (!State) return false;
    return ffi_reset_drone_state(State) == 0;
}

bool FDronePhysicsBridgeModule::StepPhysics(DroneState* State, const ControlInputs* Inputs, SurrogateHandle* AeroHandle, SurrogateHandle* FeaHandle, float DeltaTime)
{
    if (!State || !Inputs) return false;
    return ffi_step_physics(State, Inputs, AeroHandle, FeaHandle, DeltaTime) == 0;
}

bool FDronePhysicsBridgeModule::GetDebugTelemetry(DebugTelemetry* OutTelemetry)
{
    if (!OutTelemetry) return false;
    return ffi_get_debug_telemetry(OutTelemetry) == 0;
}

FVector FDronePhysicsBridgeModule::NedToUnrealWorld(const double NedPosition[3])
{
    return FVector(NedPosition[0] * 100.0, NedPosition[1] * 100.0, -NedPosition[2] * 100.0);
}

FVector FDronePhysicsBridgeModule::NedForceToUnreal(const float NedVector[3])
{
    return FVector(NedVector[0], NedVector[1], -NedVector[2]);
}