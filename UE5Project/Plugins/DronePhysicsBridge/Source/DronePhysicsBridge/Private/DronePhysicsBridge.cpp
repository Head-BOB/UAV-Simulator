#include "DronePhysicsBridge.h"
#include "Modules/ModuleManager.h"

IMPLEMENT_MODULE(FDronePhysicsBridgeModule, DronePhysicsBridge)

void FDronePhysicsBridgeModule::StartupModule()
{
    int32 ExpectedVersion = 5;
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

ThermalState FDronePhysicsBridgeModule::CreateDefaultThermalState()
{
    ThermalState State;
    State.motor_temp_c[0] = 20.0;
    State.motor_temp_c[1] = 20.0;
    State.motor_temp_c[2] = 20.0;
    State.motor_temp_c[3] = 20.0;
    return State;
}

bool FDronePhysicsBridgeModule::ResetState(DroneState* State)
{
    if (!State) return false;
    return ffi_reset_drone_state(State) == 0;
}

bool FDronePhysicsBridgeModule::LoadVehicleConfig(const FString& Path, VehicleConfig* OutConfig)
{
    if (!OutConfig) return false;
    return ffi_load_vehicle_config(TCHAR_TO_UTF8(*Path), OutConfig) == 0;
}

bool FDronePhysicsBridgeModule::StepPhysics(DroneState* State, ThermalState* Thermal, const ControlInputs* Inputs, SurrogateHandle* AeroHandle, SurrogateHandle* FeaHandle, const VehicleConfig* Config, DebugTelemetry* OutTelemetry, double DeltaTime)
{
    if (!State || !Thermal || !Inputs || !Config || !OutTelemetry) return false;
    return ffi_step_physics(State, Thermal, Inputs, AeroHandle, FeaHandle, Config, OutTelemetry, DeltaTime) == 0;
}

FVector FDronePhysicsBridgeModule::NedToUnrealWorld(const double NedPosition[3])
{
    return FVector(NedPosition[0] * 100.0, NedPosition[1] * 100.0, -NedPosition[2] * 100.0);
}

FQuat FDronePhysicsBridgeModule::NedToUnrealQuat(const float NedQuat[4])
{
    return FQuat(-NedQuat[0], -NedQuat[1], NedQuat[2], NedQuat[3]);
}