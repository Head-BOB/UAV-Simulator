// Tier: V
// DEMO pawn, not flight code. Owns the fixed-step loop that calls the Rust physics core, a demo
// flight controller (DroneDemoControl.h), the demo wind (DroneWind.h), the skeletal drone model
// with spinning props, wind arrows, and the HUD. Unreal only renders and takes input here:
// the physics truth is the Rust core. NOT COMPILED OR RUN by its author (no Unreal in the
// sandbox): every Unreal API use is TODO(HUMAN-VERIFY: builds on the project's Unreal version).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "DronePhysicsBridge.h"
#include "DroneDemoControl.h"
#include "DroneWind.h"
#include "DroneDemoPawn.generated.h"

class UPoseableMeshComponent;
class USpringArmComponent;
class UCameraComponent;
class UStaticMeshComponent;
class UMaterialInstanceDynamic;
class UMaterialInterface;
class APlayerController;

UCLASS()
class UAV_SIMULATOR_API ADroneDemoPawn : public APawn
{
	GENERATED_BODY()

public:
	ADroneDemoPawn();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

private:
	// One motor's bones, in the order of DroneModelMap.json (FR, FL, RR, RL = config index order).
	struct FMotorVisual
	{
		FName HubBone;
		TArray<FName> SpinBones;
		TArray<FQuat> RestComponentRot;  // component-space rest rotation of each spin bone
		double SpinAngleDeg = 0.0;
	};

	// Setup
	bool LoadPhysics();
	bool LoadModelMap(double& OutModelLeftRightM, double& OutModelFrontRearM);
	void SetupVisuals();
	void BuildFallbackShapes();
	void BuildMotionDiscs();
	void SetupEnvironment();
	void PlaceOnGround();

	// Per frame
	void ReadInput(APlayerController* PC, DroneDemo::Command& OutCommand);
	void ReadWindKeys(APlayerController* PC);
	void StepFixed(double FixedDt, const DroneDemo::Command& Command);
	void PinToGround();
	void UpdateVisuals(float DeltaTime);
	void DrawWindArrows() const;
	void DrawHud() const;

	// Physics state (Rust owns the truth; these are plain data passed through the FFI).
	DroneState PhysicsState = {};
	ThermalState Thermal = {};
	VehicleConfig Config = {};
	FaultLatch PhysicsFaultLatch = {};
	DebugTelemetry LastTelemetry = {};
	bool bHaveTelemetry = false;
	bool bPhysicsReady = false;
	bool bPhysicsFault = false;
	double Accumulator = 0.0;
	int32 DroppedTimeEvents = 0;

	// Demo controller and wind.
	TUniquePtr<DroneDemo::Controller> Controller;
	DroneWind::Params WindParams;
	DroneWind::Generator WindGenerator;
	DroneWind::Sample WindSample = {};

	// World placement: the NED origin in Unreal world space (cm).
	FVector WorldOriginCm = FVector::ZeroVector;

	// Visuals
	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TObjectPtr<UPoseableMeshComponent> PoseMesh;
	UPROPERTY() TObjectPtr<USpringArmComponent> CameraArm;
	UPROPERTY() TObjectPtr<UCameraComponent> Camera;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> MotionDiscs;
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> DiscMaterials;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> FallbackShapes;  // body + 4 props when the model is unusable
	TArray<FMotorVisual> Motors;
	bool bUseFallbackShapes = false;
	bool bDiscMaterialMissingLogged = false;
};
