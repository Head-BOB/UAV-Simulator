#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DronePhysicsBridge.h"
#include "DroneTelemetryComponent.generated.h"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UAV_SIMULATOR_API UDroneTelemetryComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	UDroneTelemetryComponent();

public:	
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/// Controls the visibility of the On-Screen Display (OSD) and debug vectors.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Telemetry")
	bool bShowOSD = true;
	
	// Integration Test variables
    	DroneState PhysicsState;
    	bool bIsPhysicsInitialized = false;
};