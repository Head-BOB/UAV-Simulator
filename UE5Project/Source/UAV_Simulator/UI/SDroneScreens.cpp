// Tier: V
// DEMO. NOT COMPILED OR RUN by its author. See SDroneScreens.h.
#include "SDroneScreens.h"

#include "DroneUiStyle.h"
#include "SDroneInstruments.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SSlider.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SInvalidationPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
using namespace DroneUiStyle;

TSharedRef<STextBlock> Label(const FString& Text, int32 ScaleIdx, const FLinearColor& Color, float UiScale, bool bBold = false)
{
	return SNew(STextBlock).Text(FText::FromString(Text)).Font(Font(ScaleIdx, bBold, UiScale)).ColorAndOpacity(FSlateColor(Color));
}

// Small letter-spaced caps label (14 px minimum at 1080p).
TSharedRef<STextBlock> Caption(const FString& Text, float UiScale)
{
	return Label(Text.ToUpper(), 0, Muted(), UiScale, true);
}

TSharedRef<SWidget> Panel(const TSharedRef<SWidget>& Content, float Pad = 16.0f)
{
	return SNew(SBorder).BorderImage(PanelBrush()).Padding(Pad)[Content];
}

FLinearColor ColourOf(DroneUi::Status S)
{
	return StatusColour(S);
}
}  // namespace

// =============================================================================================
// Flight display
// =============================================================================================
void SDroneOsd::Construct(const FArguments& InArgs)
{
	Snap = InArgs._Snapshot;
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
	const float G = Grid(1.0f);

	TSharedRef<SVerticalBox> RightColumn = SNew(SVerticalBox);
	RightColumn->AddSlot().AutoHeight().Padding(0, 0, 0, G)[Caption(TEXT("System"), Scale)];
	RightColumn->AddSlot().AutoHeight()[SAssignNew(BatteryText, STextBlock).Font(Font(1, false, Scale)).ColorAndOpacity(FSlateColor(Text()))];
	RightColumn->AddSlot().AutoHeight()[SAssignNew(DensityText, STextBlock).Font(Font(1, false, Scale)).ColorAndOpacity(FSlateColor(Text()))];
	RightColumn->AddSlot().AutoHeight().Padding(0, 0, 0, G)[SAssignNew(TempText, STextBlock).Font(Font(0, false, Scale)).ColorAndOpacity(FSlateColor(Muted()))];
	RightColumn->AddSlot().AutoHeight().Padding(0, G, 0, G)[Caption(TEXT("Margins (live)"), Scale)];
	for (int32 I = 0; I < 5; ++I)
	{
		TSharedPtr<SDroneMarginRow> Row;
		RightColumn->AddSlot().AutoHeight()[SAssignNew(Row, SDroneMarginRow).UiScale(Scale)];
		MarginRows.Add(Row);
	}
	RightColumn->AddSlot().AutoHeight().Padding(0, G, 0, 0)[SNew(SDroneChart).Snapshot(Snap).Kind(static_cast<int32>(SDroneChart::EKind::Altitude)).Title(TEXT("ALTITUDE")).UiScale(Scale)];
	RightColumn->AddSlot().AutoHeight().Padding(0, G, 0, 0)[SNew(SDroneChart).Snapshot(Snap).Kind(static_cast<int32>(SDroneChart::EKind::Speed)).Title(TEXT("SPEED")).UiScale(Scale)];
	RightColumn->AddSlot().AutoHeight().Padding(0, G, 0, 0)[SNew(SDroneChart).Snapshot(Snap).Kind(static_cast<int32>(SDroneChart::EKind::BatteryA)).Title(TEXT("BATTERY CURRENT A")).UiScale(Scale)];

	ChildSlot
	[
		SNew(SOverlay)
		// Top bar: mode, environment, clock.
		+ SOverlay::Slot().HAlign(HAlign_Fill).VAlign(VAlign_Top).Padding(G * 2)
		[
			Panel(SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.0f).HAlign(HAlign_Left)[SAssignNew(ModeText, STextBlock).Font(FontCondensed(3, Scale)).ColorAndOpacity(FSlateColor(Accent()))]
				+ SHorizontalBox::Slot().FillWidth(1.0f).HAlign(HAlign_Center)[SAssignNew(EnvText, STextBlock).Font(Font(2, true, Scale)).ColorAndOpacity(FSlateColor(Text()))]
				+ SHorizontalBox::Slot().FillWidth(1.0f).HAlign(HAlign_Right)[SAssignNew(ClockText, STextBlock).Font(Font(3, true, Scale)).ColorAndOpacity(FSlateColor(Text()))],
				G)
		]
		// Notice line under the top bar (fallbacks, faults).
		+ SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Top).Padding(0, G * 10, 0, 0)
		[
			SAssignNew(NoticeText, STextBlock).Font(Font(1, true, Scale)).ColorAndOpacity(FSlateColor(Warning()))
		]
		// Bottom left: tapes around the horizon.
		+ SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Bottom).Padding(G * 2)
		[
			Panel(SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth()[SNew(SDroneTape).Snapshot(Snap).bAltitude(false).UiScale(Scale)]
				+ SHorizontalBox::Slot().AutoWidth().Padding(G, 0)[SNew(SDroneHorizon).Snapshot(Snap).UiScale(Scale)]
				+ SHorizontalBox::Slot().AutoWidth()[SNew(SDroneTape).Snapshot(Snap).bAltitude(true).UiScale(Scale)],
				G)
		]
		// Bottom centre: compass above the motor bars. The centre of the screen stays clear.
		+ SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Bottom).Padding(G * 2)
		[
			Panel(SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()[SNew(SDroneCompass).Snapshot(Snap).UiScale(Scale)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, G, 0, 0)[SNew(SDroneMotorBars).Snapshot(Snap).UiScale(Scale)],
				G)
		]
		// Right edge: system status, margins, charts.
		+ SOverlay::Slot().HAlign(HAlign_Right).VAlign(VAlign_Center).Padding(G * 2, G * 12, G * 2, G * 12)
		[
			Panel(SNew(SBox).WidthOverride(260.0f * Scale)[RightColumn], G * 2)
		]
		// Control hint (static panel, one invalidation box).
		+ SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(G * 2, G * 10, 0, 0)
		[
			SNew(SInvalidationPanel)
			[
				SAssignNew(HintText, STextBlock).Font(Font(0, false, Scale)).ColorAndOpacity(FSlateColor(Muted()))
					.Text(FText::FromString(TEXT("TAB  panel      T  take off      L  land      R  report      WASD / arrows  fly\nModel: Animated Drone by hartwelkisaka, CC BY 4.0")))
			]
		]
	];
}

void SDroneOsd::UpdateText()
{
	if (!Snap) return;
	const FDroneUiText& T = Snap->Text;
	ModeText->SetText(FText::FromString(T.Mode));
	EnvText->SetText(FText::FromString(T.Environment));
	ClockText->SetText(FText::FromString(T.Clock));
	NoticeText->SetText(FText::FromString(T.Notice));
	BatteryText->SetText(FText::FromString(T.Battery));
	DensityText->SetText(FText::FromString(T.Density));
	TempText->SetText(FText::FromString(T.Temp));
}

void SDroneOsd::SetMargins(const TArray<TPair<FString, double>>& Rows)
{
	for (int32 I = 0; I < MarginRows.Num(); ++I)
	{
		if (Rows.IsValidIndex(I)) MarginRows[I]->Set(Rows[I].Key, Rows[I].Value);
	}
}

// =============================================================================================
// Control panel
// =============================================================================================
void SDroneControlPanel::Construct(const FArguments& InArgs)
{
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
	ApplyDelegate = InArgs._OnApplyWeather;
	const float G = Grid(1.0f);
	const FDroneUiWeatherInput Defaults;

	auto Field = [&](const TCHAR* Name, const FString& Initial, TSharedPtr<SEditableTextBox>& Out) {
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(0.5f).VAlign(VAlign_Center)[Label(Name, 1, Text(), Scale)]
			+ SHorizontalBox::Slot().FillWidth(0.5f)[SAssignNew(Out, SEditableTextBox).Text(FText::FromString(Initial)).Font(Font(1, false, Scale))];
	};
	auto Button = [&](const TCHAR* Caption, FSimpleDelegate Click, bool bAccent) {
		return SNew(SButton).OnClicked_Lambda([Click]() { Click.ExecuteIfBound(); return FReply::Handled(); })
			[SNew(SBorder).BorderImage(CardBrush(bAccent)).Padding(G).HAlign(HAlign_Center)[Label(Caption, 1, bAccent ? Accent() : Text(), Scale, true)]];
	};

	TSharedRef<SVerticalBox> Envs = SNew(SVerticalBox);
	for (const FString& Id : InArgs._EnvironmentIds)
	{
		FOnEnvironmentPick Pick = InArgs._OnPickEnvironment;
		Envs->AddSlot().AutoHeight().Padding(0, 0, 0, 4)
		[
			SNew(SButton).OnClicked_Lambda([Pick, Id]() { Pick.ExecuteIfBound(Id); return FReply::Handled(); })
				[SNew(SBorder).BorderImage(CardBrush(false)).Padding(G * 0.5f)[Label(Id, 1, Text(), Scale)]]
		];
	}

	ChildSlot
	[
		SNew(SBox).WidthOverride(380.0f * Scale)
		[
			Panel(SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()[Caption(TEXT("Control panel"), Scale)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, G, 0, G)[Label(TEXT("Flight is paused for input. Values are checked, never silently changed."), 0, Muted(), Scale)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, G)[Caption(TEXT("Weather (drives the real physics core)"), Scale)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)[Field(TEXT("Wind speed m/s (0 to 15)"), Defaults.WindSpeed, WindSpeedBox)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)[Field(TEXT("Wind toward deg (0 to 360)"), Defaults.WindDirection, WindDirBox)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)[Field(TEXT("Gust level m/s (0 to 5)"), Defaults.Gust, GustBox)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)[Field(TEXT("Altitude m (0 to 5000)"), Defaults.Altitude, AltBox)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)[Field(TEXT("Temperature C"), Defaults.Temperature, TempBox)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)[Field(TEXT("Humidity % (shown only)"), Defaults.Humidity, HumidityBox)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, G, 0, 0)[SNew(SButton).OnClicked(this, &SDroneControlPanel::HandleApply)
					[SNew(SBorder).BorderImage(CardBrush(true)).Padding(G).HAlign(HAlign_Center)[Label(TEXT("APPLY WEATHER"), 1, Accent(), Scale, true)]]]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4)[SAssignNew(MessageText, STextBlock).Font(Font(0, false, Scale)).ColorAndOpacity(FSlateColor(Muted())).AutoWrapText(true)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, G, 0, G)[Caption(TEXT("Environment"), Scale)]
				+ SVerticalBox::Slot().AutoHeight()[Envs]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, G, 0, 0)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0)[Button(TEXT("TAKE OFF"), InArgs._OnTakeoff, false)]
					+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(4, 0, 0, 0)[Button(TEXT("LAND"), InArgs._OnLand, false)]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)[Button(TEXT("FLIGHT REPORT"), InArgs._OnReport, true)]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)[Button(TEXT("CLOSE (TAB)"), InArgs._OnClose, false)],
				G * 2)
		]
	];
}

FReply SDroneControlPanel::HandleApply()
{
	FDroneUiWeatherInput In;
	In.WindSpeed = WindSpeedBox->GetText().ToString();
	In.WindDirection = WindDirBox->GetText().ToString();
	In.Gust = GustBox->GetText().ToString();
	In.Altitude = AltBox->GetText().ToString();
	In.Temperature = TempBox->GetText().ToString();
	In.Humidity = HumidityBox->GetText().ToString();
	ApplyDelegate.ExecuteIfBound(In);
	return FReply::Handled();
}

void SDroneControlPanel::SetMessage(const FString& Text, bool bError)
{
	MessageText->SetText(FText::FromString(Text));
	MessageText->SetColorAndOpacity(FSlateColor(bError ? Critical() : Accent()));
}

void SDroneControlPanel::SetFields(const FDroneUiWeatherInput& In)
{
	WindSpeedBox->SetText(FText::FromString(In.WindSpeed));
	WindDirBox->SetText(FText::FromString(In.WindDirection));
	GustBox->SetText(FText::FromString(In.Gust));
	AltBox->SetText(FText::FromString(In.Altitude));
	TempBox->SetText(FText::FromString(In.Temperature));
	HumidityBox->SetText(FText::FromString(In.Humidity));
}

// =============================================================================================
// Start menu
// =============================================================================================
void SDroneStartMenu::Construct(const FArguments& InArgs)
{
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
	StartDelegate = InArgs._OnStart;
	NoticeText = InArgs._Notice;
	ChildSlot
	[
		SNew(SBorder).BorderImage(WhiteBrush()).BorderBackgroundColor(Rgb(DroneUi::kPanelTop, 0.92f)).Padding(Grid(6.0f))
		[
			SAssignNew(Host, SVerticalBox)
		]
	];
	Rebuild();
}

void SDroneStartMenu::SetCards(const TArray<FCard>& InCards, const FString& InSelectedId)
{
	Cards = InCards;
	SelectedId = InSelectedId;
	Rebuild();
}

FReply SDroneStartMenu::Select(FString Id)
{
	SelectedId = Id;
	Rebuild();
	return FReply::Handled();
}

void SDroneStartMenu::Rebuild()
{
	if (!Host.IsValid()) return;
	const float G = Grid(1.0f);
	Host->ClearChildren();
	Host->AddSlot().AutoHeight().HAlign(HAlign_Center)[Label(TEXT("UAV DIGITAL TWIN"), 4, Accent(), Scale, true)];
	Host->AddSlot().AutoHeight().HAlign(HAlign_Center).Padding(0, G, 0, G * 3)
		[Label(TEXT("Quad reference build (APC 12x6E, MN3508)  -  PRELIMINARY: most inputs are ASSUMED"), 1, Muted(), Scale)];
	Host->AddSlot().AutoHeight().HAlign(HAlign_Center).Padding(0, 0, 0, G)[Caption(TEXT("Choose an environment"), Scale)];

	TSharedRef<SHorizontalBox> Row = SNew(SHorizontalBox);
	for (const FCard& C : Cards)
	{
		const bool bSel = C.Id.Equals(SelectedId, ESearchCase::IgnoreCase);
		const FString Id = C.Id;
		TSharedRef<SVerticalBox> Body = SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()[Label(C.Name, 3, bSel ? Accent() : Text(), Scale, true)]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, G * 0.5f, 0, 0)[SNew(STextBlock).Text(FText::FromString(C.Detail)).Font(Font(0, false, Scale)).ColorAndOpacity(FSlateColor(Muted())).AutoWrapText(true)];
		if (!C.bLevelInstalled)
		{
			Body->AddSlot().AutoHeight().Padding(0, G, 0, 0)[Label(TEXT("level not installed: default level"), 0, Warning(), Scale)];
		}
		if (!C.bValid)
		{
			Body->AddSlot().AutoHeight().Padding(0, G, 0, 0)[Label(C.Problem, 0, Critical(), Scale)];
		}
		Row->AddSlot().AutoWidth().Padding(G)
		[
			SNew(SBox).WidthOverride(230.0f * Scale)
			[
				SNew(SButton).IsEnabled(C.bValid).OnClicked(this, &SDroneStartMenu::Select, Id)
				[SNew(SBorder).BorderImage(CardBrush(bSel)).Padding(G * 2)[Body]]
			]
		];
	}
	Host->AddSlot().AutoHeight().HAlign(HAlign_Center)[Row];
	if (!NoticeText.IsEmpty())
	{
		Host->AddSlot().AutoHeight().HAlign(HAlign_Center).Padding(0, G * 2, 0, 0)[Label(NoticeText, 1, Warning(), Scale)];
	}
	const FString Start = SelectedId;
	Host->AddSlot().AutoHeight().HAlign(HAlign_Center).Padding(0, G * 4, 0, 0)
	[
		SNew(SBox).WidthOverride(320.0f * Scale)
		[
			SNew(SButton).IsEnabled(!SelectedId.IsEmpty()).OnClicked_Lambda([this, Start]() { StartDelegate.ExecuteIfBound(Start); return FReply::Handled(); })
				[SNew(SBorder).BorderImage(CardBrush(true)).Padding(G * 2).HAlign(HAlign_Center)[Label(TEXT("START FLIGHT"), 3, Accent(), Scale, true)]]
		]
	];
	Host->AddSlot().AutoHeight().HAlign(HAlign_Center).Padding(0, G * 3, 0, 0)
		[Label(TEXT("T take off  -  L land  -  WASD / arrows fly  -  TAB control panel and weather  -  wind keys: see the README"), 0, Muted(), Scale)];
	// Required credit (CC BY 4.0), also in the plugin README.
	Host->AddSlot().AutoHeight().HAlign(HAlign_Center).Padding(0, G, 0, 0)
		[Label(TEXT("Model: Animated Drone by hartwelkisaka, CC BY 4.0"), 0, Muted(), Scale)];
}

// =============================================================================================
// Report screen
// =============================================================================================
void SDroneReportScreen::Construct(const FArguments& InArgs)
{
	Scale = InArgs._UiScale > 0.0f ? InArgs._UiScale : 1.0f;
	ExportDelegate = InArgs._OnExport;
	CloseDelegate = InArgs._OnClose;
	RerunDelegate = InArgs._OnRerun;
	ChildSlot
	[
		SNew(SBorder).BorderImage(WhiteBrush()).BorderBackgroundColor(Rgb(DroneUi::kPanelTop, 0.94f)).Padding(Grid(4.0f))
		[
			SAssignNew(Content, SVerticalBox)
		]
	];
	FDroneUiReport Empty;
	SetReport(Empty);
}

void SDroneReportScreen::SetReport(const FDroneUiReport& R)
{
	if (!Content.IsValid()) return;
	const float G = Grid(1.0f);
	Content->ClearChildren();
	Content->AddSlot().AutoHeight()[Label(TEXT("FLIGHT REPORT"), 4, Accent(), Scale, true)];
	if (!R.Banner.IsEmpty())
	{
		Content->AddSlot().AutoHeight().Padding(0, G, 0, G)[SNew(STextBlock).Text(FText::FromString(R.Banner)).Font(Font(0, false, Scale)).ColorAndOpacity(FSlateColor(Warning())).AutoWrapText(true)];
	}
	if (R.bRunning)
	{
		Content->AddSlot().AutoHeight().Padding(0, G * 2)[Label(TEXT("Flying the standard mission in every environment... (a few seconds)"), 2, Text(), Scale)];
	}
	else if (!R.bReady)
	{
		Content->AddSlot().AutoHeight().Padding(0, G * 2)[Label(R.Message.IsEmpty() ? FString(TEXT("No report yet. Land, or press FLIGHT REPORT in the control panel.")) : R.Message, 2, Text(), Scale)];
	}
	else
	{
		Content->AddSlot().AutoHeight().Padding(0, G)[Label(FString::Printf(TEXT("%s: %s"), *R.Environment, *R.VerdictText.ToUpper()), 3, ColourOf(R.VerdictStatus), Scale, true)];
		Content->AddSlot().AutoHeight()[Label(R.VerdictReason, 1, Muted(), Scale)];
		TSharedRef<SScrollBox> Scroll = SNew(SScrollBox);
		Scroll->AddSlot().Padding(0, G, 0, 0)[Caption(TEXT("Components, weakest first"), Scale)];
		int32 Rank = 1;
		for (const FDroneUiReportRow& Row : R.Rows)
		{
			TSharedRef<SVerticalBox> Cell = SNew(SVerticalBox);
			Cell->AddSlot().AutoHeight()[Label(FString::Printf(TEXT("%d.  %s  -  %s"), Rank++, *Row.Component, *Row.Metric), 1, Text(), Scale, true)];
			Cell->AddSlot().AutoHeight()[Label(Row.bAssessed ? FString::Printf(TEXT("margin %.2f    peak %s    limit %s"), Row.Margin, *Row.PeakText, *Row.LimitText) : FString(TEXT("NOT ASSESSED")), 1, Row.bAssessed ? ColourOf(Row.Status) : Muted(), Scale)];
			Cell->AddSlot().AutoHeight()[SNew(STextBlock).Text(FText::FromString(Row.Cause)).Font(Font(0, false, Scale)).ColorAndOpacity(FSlateColor(Muted())).AutoWrapText(true)];
			for (const FString& S : Row.Suggestions)
			{
				Cell->AddSlot().AutoHeight().Padding(16, 0, 0, 0)[SNew(STextBlock).Text(FText::FromString(FString(TEXT("-> ")) + S)).Font(Font(0, false, Scale)).ColorAndOpacity(FSlateColor(Text())).AutoWrapText(true)];
			}
			Scroll->AddSlot().Padding(0, G, 0, 0)[Panel(Cell, G)];
		}
		Scroll->AddSlot().Padding(0, G * 2, 0, 0)[Caption(TEXT("All environments"), Scale)];
		for (const FDroneUiEnvRow& E : R.Envs)
		{
			Scroll->AddSlot().Padding(0, 4, 0, 0)
			[
				Panel(SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(0.22f)[Label(E.Name, 1, Text(), Scale, true)]
					+ SHorizontalBox::Slot().FillWidth(0.2f)[Label(E.Verdict.ToUpper(), 1, ColourOf(E.Status), Scale, true)]
					+ SHorizontalBox::Slot().FillWidth(0.1f)[Label(FString::Printf(TEXT("%.2f"), E.MinMargin), 1, ColourOf(E.Status), Scale)]
					+ SHorizontalBox::Slot().FillWidth(0.48f)[SNew(STextBlock).Text(FText::FromString(E.Limiting + TEXT("   ") + E.Detail)).Font(Font(0, false, Scale)).ColorAndOpacity(FSlateColor(Muted())).AutoWrapText(true)],
					G)
			];
		}
		Content->AddSlot().FillHeight(1.0f).Padding(0, G, 0, G)[Scroll];
	}
	if (!R.Message.IsEmpty() && (R.bReady || R.bRunning))
	{
		Content->AddSlot().AutoHeight()[Label(R.Message, 0, Accent(), Scale)];
	}
	if (!R.FolderPath.IsEmpty())
	{
		Content->AddSlot().AutoHeight()[Label(FString(TEXT("Files: ")) + R.FolderPath, 0, Muted(), Scale)];
	}
	auto Btn = [&](const TCHAR* Caption, FSimpleDelegate Click, bool bAccent) {
		return SNew(SButton).OnClicked_Lambda([Click]() { Click.ExecuteIfBound(); return FReply::Handled(); })
			[SNew(SBorder).BorderImage(CardBrush(bAccent)).Padding(G).HAlign(HAlign_Center)[Label(Caption, 1, bAccent ? Accent() : Text(), Scale, true)]];
	};
	Content->AddSlot().AutoHeight().Padding(0, G, 0, 0)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0)[Btn(TEXT("EXPORT REPORT (HTML, JSON, CSV)"), ExportDelegate, true)]
		+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(4, 0, 4, 0)[Btn(TEXT("RUN AGAIN"), RerunDelegate, false)]
		+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(4, 0, 0, 0)[Btn(TEXT("BACK TO FLIGHT"), CloseDelegate, false)]
	];
}
