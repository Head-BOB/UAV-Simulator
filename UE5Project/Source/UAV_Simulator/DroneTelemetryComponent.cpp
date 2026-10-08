#include "DroneTelemetryComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"
#include "Components/TextRenderComponent.h"
#include "Misc/Paths.h"

UDroneTelemetryComponent::UDroneTelemetryComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UDroneTelemetryComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// NED +Z is Down, UE5 +Z is Up. Scale factor 100.0 converts physics meters to rendering centimeters.
	return FVector(NedPosition[0] * 100.0, NedPosition[1] * 100.0, -NedPosition[2] * 100.0);
}

void UDroneTelemetryComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bShowOSD || !GetOwner()) return;

	// --- BEGIN INTEGRATION TEST BLOCK (H4 & M1) ---
	// Isolated hover test logic. Remove or comment out when connecting real player inputs.
	/*if (!bIsPhysicsInitialized)
	{
		PhysicsState = FDronePhysicsBridgeModule::CreateDefaultState();
		Thermal = FDronePhysicsBridgeModule::CreateDefaultThermalState();

		FString BasePath = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir());
		FString ConfigPath = BasePath + TEXT("offline_pipeline/geometry/vehicle_config.json");
		FString AeroPath = BasePath + TEXT("trained_models/aero_stub.onnx");
		FString FeaPath = BasePath + TEXT("trained_models/fea_stub.onnx");

		if (!FDronePhysicsBridgeModule::LoadVehicleConfig(ConfigPath, &Config))
		{
			UE_LOG(LogTemp, Fatal, TEXT("Failed to load VehicleConfig!"));
		}

		ffi_load_surrogate_model(TCHAR_TO_UTF8(*AeroPath), &AeroHandle);
		ffi_load_surrogate_model(TCHAR_TO_UTF8(*FeaPath), &FeaHandle);

		bIsPhysicsInitialized = true;
	}

	ControlInputs TestInputs;
	// 0.4095f precisely balances gravity (9.80665) against the Phase 1 placeholder thrust coefficient.
	TestInputs.throttle = 0.4095f; 
	TestInputs.roll = 0.0f;
	TestInputs.pitch = 0.0f;
	TestInputs.yaw = 0.0f;

	ffi_step_physics(&PhysicsState, &TestInputs, DeltaTime);
	
	FVector NewPos = NedToUnrealWorld(PhysicsState.position);
	FQuat NewRot(PhysicsState.orientation[0], PhysicsState.orientation[1], PhysicsState.orientation[2], PhysicsState.orientation[3]);
	GetOwner()->SetActorLocationAndRotation(NewPos, NewRot);*/
	// --- END INTEGRATION TEST BLOCK ---
	
	DebugTelemetry Telemetry;

	FVector ActorLocation = GetOwner()->GetActorLocation();
	
	// Constant scale limits unbounded line lengths from Newton values directly rendering in UE5 units.
	const float ForceScale = 10.0f; 

	// Vectors must be inverted on Z to match UE5's left-handed Z-up frame.
	FVector NetThrust(Telemetry.net_thrust[0], Telemetry.net_thrust[1], -Telemetry.net_thrust[2]);
	FVector AeroDrag(Telemetry.aero_drag[0], Telemetry.aero_drag[1], -Telemetry.aero_drag[2]);
	FVector Gravity(Telemetry.gravity[0], Telemetry.gravity[1], -Telemetry.gravity[2]);
	FVector NetForce(Telemetry.net_force[0], Telemetry.net_force[1], -Telemetry.net_force[2]);

	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (NetThrust * ForceScale), 50.0f, FColor::Blue, false, -1.0f, 0, 3.0f);
	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (AeroDrag * ForceScale), 50.0f, FColor::Red, false, -1.0f, 0, 2.0f);
	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (Gravity * ForceScale), 50.0f, FColor::Yellow, false, -1.0f, 0, 2.0f);
	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (NetForce * ForceScale), 50.0f, FColor::Green, false, -1.0f, 0, 5.0f);

	UTextRenderComponent* Hologram = GetOwner()->FindComponentByClass<UTextRenderComponent>();
	if (!Hologram)
	{
		Hologram = NewObject<UTextRenderComponent>(GetOwner());
		Hologram->RegisterComponent();
		Hologram->AttachToComponent(GetOwner()->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
		Hologram->SetRelativeLocation(FVector(0, 0, 150.0f));
		Hologram->SetHorizontalAlignment(EHTA_Center);
		Hologram->SetWorldSize(26.0f);
	}

	FString MarginString = FMath::IsNaN(Telemetry.structural_safety_margin) ? TEXT("NaN") : FString::Printf(TEXT("%.2f"), Telemetry.structural_safety_margin);

	FString HologramText = FString::Printf(TEXT(
		"Speed: %.1f m/s\n"
		"Altitude: %.1f m\n"
		"Struct Margin: %s\n"
		"Motor Temp: %.1f C\n"
		"RPM: %.0f, %.0f, %.0f, %.0f\n"
		"Dropped Ticks: %d"
	),
	(double)(FVector(PhysicsState.velocity[0], PhysicsState.velocity[1], PhysicsState.velocity[2]).Length()),
	(double)(-PhysicsState.position[2]),
	*MarginString,
	(double)(Telemetry.motor_temperatures_c[0]),
	(double)(Telemetry.motor_rpms[0]), (double)(Telemetry.motor_rpms[1]), (double)(Telemetry.motor_rpms[2]), (double)(Telemetry.motor_rpms[3]),
	DroppedTimeEvents);

	if (bPhysicsFault || Telemetry.physics_fault == 1)
	{
		HologramText += TEXT("\n[!] PHYSICS DIVERGENCE (NaN)");
		Hologram->SetTextRenderColor(FColor::Red);
	}
	else if (Telemetry.stub_loaded == 1)
	{
		HologramText += TEXT("\n[!] STUB MODEL LOADED - NOT FOR ANALYSIS");
		Hologram->SetTextRenderColor(FColor::Red);
	}
	else if (Telemetry.is_validated_envelope == 0)
	{
		HologramText += TEXT("\n[!] EXTRAPOLATION WARNING");
		Hologram->SetTextRenderColor(FColor::Orange);
	}
	else
	{
		Hologram->SetTextRenderColor(FColor::Cyan);
	}

	Hologram->SetText(FText::FromString(HologramText));

	if (GEngine)
	{
		FVector RustVelocity(PhysicsState.velocity[0], PhysicsState.velocity[1], -PhysicsState.velocity[2]);
		FRotator Rotation = NewRot.Rotator();

		FString OSDText = FString::Printf(TEXT(
			"TEST H4 & M1: PURE HOVER\n"
			"Speed: %.2f m/s\n"
			"Altitude: %.2f m\n"
			"Roll: %.1f, Pitch: %.1f, Yaw: %.1f\n"
			"Angular Vel (X,Y,Z): %.3f, %.3f, %.3f\n"
		),
		RustVelocity.Length(), 
		-PhysicsState.position[2],
		Rotation.Roll, Rotation.Pitch, Rotation.Yaw,
		PhysicsState.angular_velocity[0], PhysicsState.angular_velocity[1], PhysicsState.angular_velocity[2]); 

		// Key '1' prevents log spam by overwriting the same message block every frame.
		GEngine->AddOnScreenDebugMessage(1, 0.0f, FColor::Cyan, OSDText);
	}
}