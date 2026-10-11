// Tier: V
// The user's own vehicle in the 3D view: one engine basic shape (cube or cylinder) per part, scaled to the part's real dimensions from the
// builder's assembly.json (DroneBuildFiles.h). Every part is a dimension-true STAND-IN unless the assembly says a real mesh exists. Used by
// the Build screen and, posed from the recorded run, by the Run screen. NOT COMPILED OR RUN by its author:
// TODO(HUMAN-VERIFY: builds on the project's Unreal version; the basic shapes' native sizes are assumed, see the .cpp).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include <string>
#include <vector>

#include "DroneBuildFiles.h"
#include "DroneBuildPreview.generated.h"

class UStaticMeshComponent;
class UMaterialInstanceDynamic;

UCLASS()
class UAV_SIMULATOR_API ADroneBuildPreview : public AActor
{
	GENERATED_BODY()

public:
	ADroneBuildPreview();

	// Replaces the shown parts. Positions are relative to the actor (NED to Unreal through the audited converter).
	void SetPrimitives(const std::vector<DroneApp::Primitive>& Prims);
	void ClearPrimitives();
	bool HasParts() const { return Parts.Num() > 0; }
	// Radius in cm of the shown vehicle, for the camera.
	float GetRadiusCm() const { return RadiusCm; }
	// Heat tint of the motors (cylinders), ESCs and the battery from the recorded temperatures; colours are sRGB 0..1.
	void SetHeatTints(const FLinearColor& Motor, const FLinearColor& Esc, const FLinearColor& Battery);
	void ResetTints();

private:
	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Parts;
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> Materials;
	std::vector<std::string> Roles;
	std::vector<bool> Real;
	float RadiusCm = 60.0f;
	void ApplyBaseColours();
	void SetRoleColour(const char* Role, const FLinearColor& Colour);
};
