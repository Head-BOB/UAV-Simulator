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

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Telemetry")
	bool bShowOSD = true;

private:
	DroneState PhysicsState;
	bool bIsPhysicsInitialized = false;

	SurrogateHandle* AeroHandle = nullptr;
	SurrogateHandle* FeaHandle = nullptr;
};