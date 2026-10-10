// Tier: V
// DEMO HUD: owns the Slate screens (start menu, flight display, control panel, report), feeds them from the pawn
// at 30 Hz, smooths the gauges every frame, runs the report on a worker thread, and switches the input mode.
// Selected by ADroneDemoGameMode. NOT COMPILED OR RUN by its author: TODO(HUMAN-VERIFY: builds on the project's
// Unreal version; needs a visual check of every screen).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "DroneUiMath.h"
#include "UI/DroneUiModel.h"
#include "UI/DroneUiSnapshot.h"
#include "DroneHUDTypes.h"
#include "DroneDemoHUD.generated.h"

class ADroneDemoPawn;
class SDroneOsd;
class SDroneControlPanel;
class SDroneStartMenu;
class SDroneReportScreen;
class SWidget;

UCLASS()
class UAV_SIMULATOR_API ADroneDemoHUD : public AHUD
{
	GENERATED_BODY()

public:
	ADroneDemoHUD();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	enum class EScreen : uint8 { StartMenu = 0, Flight = 1, ControlPanel = 2, Report = 3 };

	ADroneDemoPawn* GetDronePawn() const;
	void BuildWidgets();
	void SetScreen(EScreen S);
	void UpdateFades(float Dt);
	void SmoothGauges(float Dt);
	void UpdateData();            // 30 Hz
	void UpdateMargins();         // 5 Hz
	void RefreshStartMenu();
	void StartFlight(const FString& EnvId);
	void ApplyWeather(const FDroneUiWeatherInput& In);
	void PickEnvironment(const FString& Id);
	void StartReportJob();
	void OnReportReady(TSharedPtr<FDroneReportPayload> Payload);
	void ExportReport();

	// Plain data the widgets read. Widgets hold a raw pointer to it: it lives as long as this HUD.
	FDroneUiSnapshot Snap;
	FDroneUiReport Report;
	TSharedPtr<FDroneReportPayload> LastPayload;
	bool bReportRunning = false;
	EScreen Screen = EScreen::StartMenu;
	float UiScale = 1.0f;
	DroneUi::DataGate DataRate;
	double MarginAccumS = 0.0;
	double Alpha[4] = {0, 0, 0, 0};
	bool bWidgetsBuilt = false;
	FString SelectedEnvironmentId;

	TSharedPtr<SDroneStartMenu> StartMenu;
	TSharedPtr<SDroneOsd> Osd;
	TSharedPtr<SDroneControlPanel> ControlPanel;
	TSharedPtr<SDroneReportScreen> ReportScreen;
	TSharedPtr<SWidget> Roots[4];
};
