#include "DroneTelemetryComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"

UDroneTelemetryComponent::UDroneTelemetryComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

FVector UDroneTelemetryComponent::NedToUnrealWorld(const double NedPosition[3])
{
	return FVector(NedPosition[0] * 100.0, NedPosition[1] * 100.0, -NedPosition[2] * 100.0);
}

void UDroneTelemetryComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bShowOSD || !GetOwner()) return;

	if (!bIsPhysicsInitialized)
	{
		PhysicsState = ffi_create_default_drone_state();
		bIsPhysicsInitialized = true;
	}

	ControlInputs TestInputs;
	TestInputs.throttle = 0.4095f; 
	TestInputs.roll = 0.0f;
	TestInputs.pitch = 0.0f;
	TestInputs.yaw = 0.0f;

	ffi_step_physics(&PhysicsState, &TestInputs, DeltaTime);
	
	FVector NewPos = NedToUnrealWorld(PhysicsState.position);
	FQuat NewRot(PhysicsState.orientation[0], PhysicsState.orientation[1], PhysicsState.orientation[2], PhysicsState.orientation[3]);
	GetOwner()->SetActorLocationAndRotation(NewPos, NewRot);
	
	DebugTelemetry Telemetry;
	if (ffi_get_debug_telemetry(&Telemetry) != 0) return;

	FVector ActorLocation = GetOwner()->GetActorLocation();
	const float ForceScale = 10.0f; 

	FVector NetThrust(Telemetry.net_thrust[0], Telemetry.net_thrust[1], -Telemetry.net_thrust[2]);
	FVector AeroDrag(Telemetry.aero_drag[0], Telemetry.aero_drag[1], -Telemetry.aero_drag[2]);
	FVector Gravity(Telemetry.gravity[0], Telemetry.gravity[1], -Telemetry.gravity[2]);
	FVector NetForce(Telemetry.net_force[0], Telemetry.net_force[1], -Telemetry.net_force[2]);

	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (NetThrust * ForceScale), 50.0f, FColor::Blue, false, -1.0f, 0, 2.0f);
	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (AeroDrag * ForceScale), 50.0f, FColor::Red, false, -1.0f, 0, 2.0f);
	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (Gravity * ForceScale), 50.0f, FColor::Yellow, false, -1.0f, 0, 2.0f);
	DrawDebugDirectionalArrow(GetWorld(), ActorLocation, ActorLocation + (NetForce * ForceScale), 50.0f, FColor::Green, false, -1.0f, 0, 4.0f);

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

		GEngine->AddOnScreenDebugMessage(1, 0.0f, FColor::Cyan, OSDText);
	}
}