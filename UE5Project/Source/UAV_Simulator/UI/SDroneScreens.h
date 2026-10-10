// Tier: V
// DEMO: the four screens as Slate compound widgets: SDroneOsd (flight display), SDroneControlPanel (Tab),
// SDroneStartMenu (environment cards) and SDroneReportScreen. Pure C++ Slate, no Blueprint or UMG assets. Static
// text is set only in the 30 Hz update (never while painting). NOT COMPILED OR RUN by its author:
// TODO(HUMAN-VERIFY: Slate declarative syntax and widget names on the project's Unreal version; needs a build
// and a visual check of every screen at 1080p and 4K).
#pragma once

#include "CoreMinimal.h"
#include "DroneUiModel.h"
#include "DroneUiSnapshot.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/DeclarativeSyntaxSupport.h"

class STextBlock;
class SEditableTextBox;
class SDroneMarginRow;
class SVerticalBox;
class SSlider;
struct FDroneEnvironmentEntry;

// ---------------------------------------------------------------------------------------------
// Flight display
// ---------------------------------------------------------------------------------------------
class SDroneOsd : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneOsd) {}
		SLATE_ARGUMENT(const FDroneUiSnapshot*, Snapshot)
		SLATE_ARGUMENT(float, UiScale)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	// 30 Hz: copies the prebuilt strings into the text blocks.
	void UpdateText();
	// Up to 5 live margins (label, margin), at most 5 Hz.
	void SetMargins(const TArray<TPair<FString, double>>& Rows);
private:
	const FDroneUiSnapshot* Snap = nullptr;
	float Scale = 1.0f;
	TSharedPtr<STextBlock> ModeText, EnvText, ClockText, NoticeText, BatteryText, DensityText, TempText, HintText;
	TArray<TSharedPtr<SDroneMarginRow>> MarginRows;
};

// ---------------------------------------------------------------------------------------------
// Control panel (Tab)
// ---------------------------------------------------------------------------------------------
DECLARE_DELEGATE_OneParam(FOnWeatherApply, const FDroneUiWeatherInput&);
DECLARE_DELEGATE_OneParam(FOnEnvironmentPick, const FString&);

class SDroneControlPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneControlPanel) {}
		SLATE_ARGUMENT(float, UiScale)
		SLATE_ARGUMENT(TArray<FString>, EnvironmentIds)
		SLATE_EVENT(FOnWeatherApply, OnApplyWeather)
		SLATE_EVENT(FOnEnvironmentPick, OnPickEnvironment)
		SLATE_EVENT(FSimpleDelegate, OnTakeoff)
		SLATE_EVENT(FSimpleDelegate, OnLand)
		SLATE_EVENT(FSimpleDelegate, OnReport)
		SLATE_EVENT(FSimpleDelegate, OnClose)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	void SetMessage(const FString& Text, bool bError);
	void SetFields(const FDroneUiWeatherInput& In);
private:
	FReply HandleApply();
	float Scale = 1.0f;
	FOnWeatherApply ApplyDelegate;
	TSharedPtr<SEditableTextBox> WindSpeedBox, WindDirBox, GustBox, AltBox, TempBox, HumidityBox;
	TSharedPtr<STextBlock> MessageText;
};

// ---------------------------------------------------------------------------------------------
// Start menu
// ---------------------------------------------------------------------------------------------
DECLARE_DELEGATE_OneParam(FOnStartFlight, const FString&);

class SDroneStartMenu : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneStartMenu) {}
		SLATE_ARGUMENT(float, UiScale)
		SLATE_ARGUMENT(FString, Notice)
		SLATE_EVENT(FOnStartFlight, OnStart)
	SLATE_END_ARGS()
	// Entries are copied (id, name, one-line details, level installed).
	struct FCard { FString Id, Name, Detail; bool bLevelInstalled = false; bool bValid = true; FString Problem; };
	void Construct(const FArguments& InArgs);
	void SetCards(const TArray<FCard>& InCards, const FString& InSelectedId);
private:
	void Rebuild();
	FReply Select(FString Id);
	float Scale = 1.0f;
	TArray<FCard> Cards;
	FString SelectedId;
	FOnStartFlight StartDelegate;
	TSharedPtr<SVerticalBox> Host;
	FString NoticeText;
};

// ---------------------------------------------------------------------------------------------
// Report screen
// ---------------------------------------------------------------------------------------------
class SDroneReportScreen : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SDroneReportScreen) {}
		SLATE_ARGUMENT(float, UiScale)
		SLATE_EVENT(FSimpleDelegate, OnExport)
		SLATE_EVENT(FSimpleDelegate, OnClose)
		SLATE_EVENT(FSimpleDelegate, OnRerun)
	SLATE_END_ARGS()
	void Construct(const FArguments& InArgs);
	// Rebuilds the content from the model (the report is computed on a worker thread; this runs on the game thread).
	void SetReport(const FDroneUiReport& Report);
private:
	float Scale = 1.0f;
	TSharedPtr<SVerticalBox> Content;
	FSimpleDelegate ExportDelegate, CloseDelegate, RerunDelegate;
};
