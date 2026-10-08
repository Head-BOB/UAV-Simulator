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
	if (AeroHandle) ffi_unload_surrogate_model(AeroHandle);
	if (FeaHandle) ffi_unload_surrogate_model(FeaHandle);
	Super::EndPlay(EndPlayReason);
}

void UDroneTelemetryComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bShowOSD || !GetOwner()) return;

	if (!bIsPhysicsInitialized)
	{
		PhysicsState = FDronePhysicsBridgeModule::CreateDefaultState();

		FString BasePath = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir()) + TEXT("trained_models/");
		FString AeroPath = BasePath + TEXT("aero_stub.onnx");
		FString FeaPath = BasePath + TEXT("fea_stub.onnx");

		int32 AeroStatus = ffi_load_surrogate_model(TCHAR_TO_UTF8(*AeroPath), &AeroHandle);
		if (AeroStatus != 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("Aero Surrogate failed to load. Status: %d"), AeroStatus);
		}

		int32 FeaStatus = ffi_load_surrogate_model(TCHAR_TO_UTF8(*FeaPath), &FeaHandle);
		if (FeaStatus != 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("FEA Surrogate failed to load. Status: %d"), FeaStatus);
		}

		bIsPhysicsInitialized = true;
	}

	ControlInputs LiveInputs;
	LiveInputs.throttle = 0.4095f;
	LiveInputs.roll = 0.0f;
	LiveInputs.pitch = 0.0f;
	LiveInputs.yaw = 0.0f;

	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (PC)
	{
		if (PC->IsInputKeyDown(EKeys::SpaceBar)) LiveInputs.throttle = 0.8f;
		else if (PC->IsInputKeyDown(EKeys::C)) LiveInputs.throttle = 0.2f;

		if (PC->IsInputKeyDown(EKeys::W)) LiveInputs.pitch = 0.3f;
		if (PC->IsInputKeyDown(EKeys::S)) LiveInputs.pitch = -0.3f;
		if (PC->IsInputKeyDown(EKeys::A)) LiveInputs.roll = -0.3f;
		if (PC->IsInputKeyDown(EKeys::D)) LiveInputs.roll = 0.3f;
	}

	FDronePhysicsBridgeModule::StepPhysics(&PhysicsState, &LiveInputs, AeroHandle, FeaHandle, (double)DeltaTime);

	FVector NewPos = FDronePhysicsBridgeModule::NedToUnrealWorld(PhysicsState.position);
	FQuat NewRot(PhysicsState.orientation[0], PhysicsState.orientation[1], PhysicsState.orientation[2], PhysicsState.orientation[3]);
	GetOwner()->SetActorLocationAndRotation(NewPos, NewRot);

	DebugTelemetry Telemetry;
	if (!FDronePhysicsBridgeModule::GetDebugTelemetry(&Telemetry)) return;

	FVector ActorLocation = GetOwner()->GetActorLocation();
	const float ForceScale = 10.0f;
	FVector NetThrust(Telemetry.net_thrust[0], Telemetry.net_thrust[1], -Telemetry.net_thrust[2]);
	FVector NetForce(Telemetry.net_force[0], Telemetry.net_force[1], -Telemetry.net_force[2]);

	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (NetThrust * ForceScale), 50.0f, FColor::Blue, false, -1.0f, 0, 3.0f);
	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (NetForce * ForceScale), 50.0f, FColor::Green, false, -1.0f, 0, 5.0f);

	UTextRenderComponent* Hologram = GetOwner()->FindComponentByClass<UTextRenderComponent>();
	if (!Hologram)
	{
		Hologram = NewObject<UTextRenderComponent>(GetOwner());
		Hologram->RegisterComponent();
		Hologram->AttachToComponent(GetOwner()->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
		Hologram->SetRelativeLocation(FVector(0, 0, 150.0f));
		Hologram->SetTextRenderColor(FColor::Cyan);
		Hologram->SetHorizontalAlignment(EHTA_Center);
		Hologram->SetWorldSize(26.0f);
	}

	FString HologramText = FString::Printf(TEXT(
		"Speed: %.1f m/s\n"
		"Altitude: %.1f m\n"
		"Struct Margin: %.2f\n"
		"Motor Temp: %.1f C"
	),
	(double)(FVector(PhysicsState.velocity[0], PhysicsState.velocity[1], PhysicsState.velocity[2]).Length()),
	(double)(-PhysicsState.position[2]),
	(double)(Telemetry.structural_safety_margin),
	(double)(Telemetry.motor_temperatures_c[0]));

	if (Telemetry.stub_loaded == 1)
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
		if (Telemetry.stub_loaded == 1)
		{
			GEngine->AddOnScreenDebugMessage(1, 0.0f, FColor::Red, TEXT("STUB MODEL LOADED - NOT FOR ANALYSIS"));
		}

		if (Telemetry.is_validated_envelope == 0)
		{
			GEngine->AddOnScreenDebugMessage(2, 0.0f, FColor::Orange, TEXT("WARNING: OUT OF VALIDATED ENVELOPE - EXTRAPOLATING"));
		}

		if (Telemetry.structural_safety_margin < 1.0f)
		{
			GEngine->AddOnScreenDebugMessage(3, 0.0f, FColor::Red, TEXT("CRITICAL: STRUCTURAL FAILURE IMMINENT"));
		}
	}
}