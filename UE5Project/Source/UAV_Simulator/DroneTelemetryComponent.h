#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DronePhysicsBridge.h"
#include "DroneTelemetryComponent.generated.h"

/**
 * \brief Actor component responsible for driving drone physics updates and rendering on-screen telemetry overlays.
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UAV_SIMULATOR_API UDroneTelemetryComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	/**
	 * \brief Default constructor initializing component tick settings.
	 */
	UDroneTelemetryComponent();

	/**
	 * \brief Advances physics simulation each frame and renders telemetry vectors/OSD.
	 *
	 * \param DeltaTime Frame elapsed time in seconds.
	 * \param TickType Level tick type categorization.
	 * \param ThisTickFunction Tick function managing execution schedule.
	 */
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * \brief Cleans up and frees loaded surrogate handles when actor play session ends.
	 *
	 * \param EndPlayReason Enumeration specifying reason play terminated.
	 */
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Telemetry")
	bool bShowOSD = true;

private:
	DroneState PhysicsState;
	bool bIsPhysicsInitialized = false;

	SurrogateHandle* AeroHandle = nullptr;
	SurrogateHandle* FeaHandle = nullptr;
};