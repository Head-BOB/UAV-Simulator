#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DronePhysicsBridge.h"
#include "DroneTelemetryComponent.generated.h"

/**
 * \brief Component responsible for updating the drone's physics state and rendering telemetry.
 *
 * Interfaces with the Rust physics core via FFI and translates the NED physics frame
 * to the Unreal Engine left-handed Z-up frame.
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UAV_SIMULATOR_API UDroneTelemetryComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	UDroneTelemetryComponent();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * \brief Controls the visibility of the On-Screen Display (OSD) and debug vectors.
	 */

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Telemetry")
	bool bShowOSD = true;

private:
	DroneState PhysicsState;
	bool bIsPhysicsInitialized = false;
};