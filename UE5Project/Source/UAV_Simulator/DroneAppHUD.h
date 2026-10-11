// Tier: V
// The application HUD: owns the engine-free AppModel, the display list and the one Slate canvas; ticks the model, rebuilds the display
// list at 30 Hz while something moves (and after every input), drives the 3D scene (build preview, replay pose, camera) and sets the
// input mode. Selected by ADroneDemoGameMode. NOT COMPILED OR RUN by its author:
// TODO(HUMAN-VERIFY: builds on the project's Unreal version; needs a visual check of every screen against dev-docs/ui-reference).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"

#include <memory>

#include "DroneAppModel.h"
#include "DroneDrawList.h"
// The full definition, not a forward declaration: std::unique_ptr<FDroneAppIO> needs a complete type wherever the destructor is
// instantiated, and the UHT-generated DroneAppHUD.gen.cpp does that (FVTableHelper constructor) without seeing DroneAppHUD.cpp.
#include "UI/DroneAppIO.h"
#include "DroneAppHUD.generated.h"

class ADroneDemoPawn;
class ADroneBuildPreview;
class SDroneAppCanvas;

UCLASS()
class UAV_SIMULATOR_API ADroneAppHUD : public AHUD
{
	GENERATED_BODY()

public:
	ADroneAppHUD();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	bool LoadHelperSettings(DroneApp::HelperSettings& Out, FString& OutWorkDir) const;
	void Rebuild();
	void UpdateScene(float DeltaSeconds);
	void MarkDirty() { bDirty = true; }
	ADroneDemoPawn* GetDronePawn() const;

	std::unique_ptr<FDroneAppIO> IO;
	std::unique_ptr<DroneApp::AppModel> Model;
	DroneApp::DrawList List;
	TSharedPtr<SDroneAppCanvas> AppCanvas;   // not `Canvas`: AHUD::Canvas (UCanvas*) is a base member
	UPROPERTY() TObjectPtr<ADroneBuildPreview> Preview;
	bool bDirty = true;
	double DataAccumS = 0.0;
	DroneApp::Screen LastScreen = DroneApp::Screen::Vehicle;
	bool bPreviewDressed = false;
	std::string LastPreviewBuildDir;
};
