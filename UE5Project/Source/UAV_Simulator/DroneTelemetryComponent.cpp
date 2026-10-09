// Tier: V
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
		Thermal = FDronePhysicsBridgeModule::CreateDefaultThermalState();
		PhysicsFaultLatch.physics_fault = 0;

		// ProjectDir is UE5Project/; the config and the training output live one level up, in the repo root.
		FString RepoRoot = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT(".."));
		FPaths::NormalizeDirectoryName(RepoRoot);
		FString ConfigPath = RepoRoot / TEXT("offline_pipeline/geometry/vehicle_config.json");
		FString AeroPath = RepoRoot / TEXT("offline_pipeline/trained_models/aero_stub.onnx");
		FString FeaPath = RepoRoot / TEXT("offline_pipeline/trained_models/fea_stub.onnx");

		if (!FDronePhysicsBridgeModule::LoadVehicleConfig(ConfigPath, &Config))
		{
			UE_LOG(LogTemp, Fatal, TEXT("Failed to load VehicleConfig from '%s'"), *ConfigPath);
		}

		// Rust leaves the out handle untouched on every error, so a failed load keeps the handle null.
		// TODO(HUMAN-VERIFY): policy B-15, whether a failed model load must be fatal on a Tier F run.
		const int32 AeroCode = ffi_load_surrogate_model(TCHAR_TO_UTF8(*AeroPath), &AeroHandle);
		if (AeroCode != 0)
		{
			AeroHandle = nullptr;
			UE_LOG(LogTemp, Error, TEXT("Aero surrogate load failed, code %d, path '%s'. Running without the aero surrogate (codes: ffi_load_surrogate_model in drone_ffi.h)"), AeroCode, *AeroPath);
		}
		const int32 FeaCode = ffi_load_surrogate_model(TCHAR_TO_UTF8(*FeaPath), &FeaHandle);
		if (FeaCode != 0)
		{
			FeaHandle = nullptr;
			UE_LOG(LogTemp, Error, TEXT("FEA surrogate load failed, code %d, path '%s'. Running without the FEA surrogate (codes: ffi_load_surrogate_model in drone_ffi.h)"), FeaCode, *FeaPath);
		}

		bIsPhysicsInitialized = true;
	}

	ControlInputs LiveInputs;
	LiveInputs.throttle = Config.hover_throttle;
	LiveInputs.roll = 0.0f;
	LiveInputs.pitch = 0.0f;
	LiveInputs.yaw = 0.0f;

	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (PC)
	{
		if (PC->IsInputKeyDown(EKeys::SpaceBar)) LiveInputs.throttle = 1.0f;
		else if (PC->IsInputKeyDown(EKeys::C)) LiveInputs.throttle = 0.0f;

		if (PC->IsInputKeyDown(EKeys::W)) LiveInputs.pitch = 1.0f;
		if (PC->IsInputKeyDown(EKeys::S)) LiveInputs.pitch = -1.0f;
		if (PC->IsInputKeyDown(EKeys::A)) LiveInputs.roll = -1.0f;
		if (PC->IsInputKeyDown(EKeys::D)) LiveInputs.roll = 1.0f;
	}

	constexpr double FixedDt = 0.002;
	constexpr int32 MaxSubsteps = 20;

	Accumulator += (double)DeltaTime;
	int32 Steps = 0;
	bool bPhysicsFault = false;

	while (Accumulator >= FixedDt && Steps < MaxSubsteps)
	{
		DebugTelemetry StepTelemetry = {};
		if (!FDronePhysicsBridgeModule::StepPhysics(&PhysicsState, &Thermal, &LiveInputs, AeroHandle, FeaHandle, &Config, &StepTelemetry, &PhysicsFaultLatch, FixedDt))
		{
			bPhysicsFault = true;
			break;
		}
		LastTelemetry = StepTelemetry;
		bHaveTelemetry = true;
		Accumulator -= FixedDt;
		++Steps;
	}
	const DebugTelemetry& Telemetry = LastTelemetry;

	if (Steps == MaxSubsteps && Accumulator >= FixedDt)
	{
		++DroppedTimeEvents;
		UE_LOG(LogTemp, Warning, TEXT("Physics fell behind: dropped %.6f s (cap %d steps per frame), event %d"), Accumulator, MaxSubsteps, DroppedTimeEvents);
		Accumulator = 0.0;
	}

	FVector NewPos = FDronePhysicsBridgeModule::NedToUnrealWorld(PhysicsState.position);
	FQuat NewRot = FDronePhysicsBridgeModule::NedToUnrealQuat(PhysicsState.orientation);
	GetOwner()->SetActorLocationAndRotation(NewPos, NewRot);

	FVector ActorLocation = GetOwner()->GetActorLocation();
	const float ForceScale = 10.0f;
	if (bHaveTelemetry)
	{
		// Frame conversion only in FDronePhysicsBridgeModule (ADR-001 D5); ForceScale is a display scale.
		FVector NetThrust = FDronePhysicsBridgeModule::NedToUnrealVector(Telemetry.net_thrust);
		FVector AeroDrag = FDronePhysicsBridgeModule::NedToUnrealVector(Telemetry.aero_drag);
		FVector Gravity = FDronePhysicsBridgeModule::NedToUnrealVector(Telemetry.gravity);
		FVector NetForce = FDronePhysicsBridgeModule::NedToUnrealVector(Telemetry.net_force);

		DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (NetThrust * ForceScale), 50.0f, FColor::Blue, false, -1.0f, 0, 3.0f);
		DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (AeroDrag * ForceScale), 50.0f, FColor::Red, false, -1.0f, 0, 2.0f);
		DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (Gravity * ForceScale), 50.0f, FColor::Yellow, false, -1.0f, 0, 2.0f);
		DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (NetForce * ForceScale), 50.0f, FColor::Green, false, -1.0f, 0, 5.0f);
	}

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

	if (!bHaveTelemetry)
	{
		// No step has succeeded yet: never show zeroed telemetry as a margin, temperature or warning state.
		FString NoDataText = TEXT("Telemetry: NO DATA");
		if (bPhysicsFault || PhysicsFaultLatch.physics_fault != 0)
		{
			NoDataText += TEXT("\n[!] PHYSICS DIVERGENCE (NaN)");
		}
		Hologram->SetTextRenderColor(FColor::Red);
		Hologram->SetText(FText::FromString(NoDataText));
		return;
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

	if (bPhysicsFault || PhysicsFaultLatch.physics_fault != 0)
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
		if (Telemetry.stub_loaded == 1)
		{
			GEngine->AddOnScreenDebugMessage(1, 0.0f, FColor::Red, TEXT("STUB MODEL LOADED - NOT FOR ANALYSIS"));
		}

		if (FMath::IsNaN(Telemetry.structural_safety_margin))
		{
			GEngine->AddOnScreenDebugMessage(3, 0.0f, FColor::Red, TEXT("STRUCTURE: NO MODEL"));
		}
		else if (Telemetry.structural_safety_margin < 1.0f)
		{
			GEngine->AddOnScreenDebugMessage(3, 0.0f, FColor::Red, TEXT("CRITICAL: STRUCTURAL FAILURE IMMINENT"));
		}

		if (Telemetry.is_validated_envelope == 0)
		{
			GEngine->AddOnScreenDebugMessage(2, 0.0f, FColor::Orange, TEXT("WARNING: OUT OF VALIDATED ENVELOPE - EXTRAPOLATING"));
		}
	}
}