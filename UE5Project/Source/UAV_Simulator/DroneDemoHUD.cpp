// Tier: V
// DEMO. NOT COMPILED OR RUN by its author. See DroneDemoHUD.h.
#include "DroneDemoHUD.h"

#include "Async/Async.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "DroneDemoPawn.h"
#include "DronePhysicsBridge.h"
#include "DroneMissionRunner.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformProcess.h"
#include "InputCoreTypes.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "UI/DroneUiStyle.h"
#include "UI/SDroneScreens.h"
#include "Widgets/Layout/SBox.h"

namespace
{
constexpr double kGaugeOmegaPerS = 8.0;     // critically damped needles settle in about 0.5 s
constexpr double kMarginPeriodS = 0.2;      // live margins at 5 Hz
constexpr unsigned long long kReportSeed = 12345;

FString Utf8(const std::string& S)
{
	return FString(UTF8_TO_TCHAR(S.c_str()));
}

FString Num(double V, int Decimals, int Width = 0)
{
	char Buf[32];
	DroneUi::FormatFixed(Buf, sizeof(Buf), V, Decimals, Width);
	return FString(UTF8_TO_TCHAR(Buf));
}

DroneUi::Status StatusOf(DroneElec::Verdict V)
{
	return V == DroneElec::Verdict::WithinLimits ? DroneUi::Status::Ok : (V == DroneElec::Verdict::Marginal ? DroneUi::Status::Warning : DroneUi::Status::Critical);
}

// The UE-free bundle as strings for the report screen.
void ConvertBundle(const DroneReport::Bundle& B, FDroneUiReport& Out)
{
	Out.Environment = Utf8(B.Flown.Name);
	Out.VerdictText = Utf8(DroneElec::VerdictText(B.A.V));
	Out.VerdictReason = Utf8(B.A.VerdictReason);
	Out.VerdictStatus = StatusOf(B.A.V);
	Out.Banner = Utf8(DroneReport::kStatusBanner);
	Out.Rows.Reset();
	for (const DroneReport::Finding& F : B.A.Findings)
	{
		FDroneUiReportRow Row;
		Row.Component = Utf8(F.Component);
		Row.Metric = Utf8(F.Metric);
		Row.bAssessed = F.Assessed;
		Row.Margin = F.Margin;
		Row.Status = DroneUi::StatusFromMargin(F.Margin);
		Row.PeakText = FString::Printf(TEXT("%.4g %s"), F.PeakValue, *Utf8(F.Unit));
		Row.LimitText = FString::Printf(TEXT("%.4g %s (%s)"), F.Limit, *Utf8(F.Unit), *Utf8(F.LimitStatus));
		Row.Cause = Utf8(F.Cause);
		Row.Consequence = Utf8(F.Consequence);
		for (const DroneReport::Suggestion& S : F.Suggestions)
		{
			const TCHAR* Tag = S.Tested ? (S.Improves ? TEXT("TESTED: helps") : TEXT("TESTED: no help")) : TEXT("NOT TESTED");
			Row.Suggestions.Add(FString::Printf(TEXT("%s [%s] %s"), *Utf8(S.Text), Tag, *Utf8(S.Evidence)));
		}
		Out.Rows.Add(Row);
	}
	Out.Envs.Reset();
	for (const DroneReport::CrossRow& R : B.Cross)
	{
		FDroneUiEnvRow E;
		E.Name = Utf8(R.Env.Name);
		E.Verdict = Utf8(DroneElec::VerdictText(R.V)) + (R.Completed ? TEXT("") : TEXT(" (mission not completed)"));
		E.Limiting = Utf8(R.Limiting);
		E.MinMargin = R.MinMargin;
		E.Status = StatusOf(R.V);
		E.Detail = FString::Printf(TEXT("%.0f m, %.0f C, wind %.0f m/s, rho %.3f"), R.Env.SiteAltitudeM, R.Env.AmbientTempC, R.Env.WindSpeedMps, R.DensityKgM3);
		Out.Envs.Add(E);
	}
}

bool WriteReportFiles(const DroneReport::Bundle& B, const FString& Folder, FString& OutError)
{
	IFileManager::Get().MakeDirectory(*Folder, true);
	const struct { const TCHAR* Name; std::string Text; } Files[] = {
		{TEXT("report.json"), DroneReport::ToJson(B)},
		{TEXT("report_findings.csv"), DroneReport::ToCsvFindings(B)},
		{TEXT("report_environments.csv"), DroneReport::ToCsvEnvironments(B)},
		{TEXT("report.html"), DroneReport::ToHtml(B)},
	};
	for (const auto& F : Files)
	{
		if (!FFileHelper::SaveStringToFile(Utf8(F.Text), *(Folder / F.Name), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			OutError = FString::Printf(TEXT("could not write %s"), F.Name);
			return false;
		}
	}
	return true;
}
}  // namespace

ADroneDemoHUD::ADroneDemoHUD()
{
	PrimaryActorTick.bCanEverTick = true;
}

ADroneDemoPawn* ADroneDemoHUD::GetDronePawn() const
{
	return Cast<ADroneDemoPawn>(GetOwningPawn());
}

void ADroneDemoHUD::BeginPlay()
{
	Super::BeginPlay();
	if (!GEngine || !GEngine->GameViewport) return;
	FVector2D Size = FVector2D(1920, 1080);
	GEngine->GameViewport->GetViewportSize(Size);
	UiScale = static_cast<float>(DroneUi::UiScale(Size.Y, 1.0));  // fixed for this session (a resize needs a restart: TODO)
	BuildWidgets();
}

void ADroneDemoHUD::EndPlay(const EEndPlayReason::Type Reason)
{
	if (GEngine && GEngine->GameViewport)
	{
		for (TSharedPtr<SWidget>& R : Roots)
		{
			if (R.IsValid()) GEngine->GameViewport->RemoveViewportWidgetContent(R.ToSharedRef());
		}
	}
	Super::EndPlay(Reason);
}

void ADroneDemoHUD::BuildWidgets()
{
	// Menu cards come from the pawn's catalog; the pawn loads it in its BeginPlay (before the HUD ticks).
	StartMenu = SNew(SDroneStartMenu).UiScale(UiScale).OnStart(FOnStartFlight::CreateUObject(this, &ADroneDemoHUD::StartFlight));
	Osd = SNew(SDroneOsd).Snapshot(&Snap).UiScale(UiScale);
	TArray<FString> Ids;
	if (const ADroneDemoPawn* P = GetDronePawn())
		for (const FDroneEnvironmentEntry& E : P->GetEnvironmentCatalog().Entries()) Ids.Add(UTF8_TO_TCHAR(E.Preset.Id.c_str()));
	ControlPanel = SNew(SDroneControlPanel)
		.UiScale(UiScale)
		.EnvironmentIds(Ids)
		.OnApplyWeather(FOnWeatherApply::CreateUObject(this, &ADroneDemoHUD::ApplyWeather))
		.OnPickEnvironment(FOnEnvironmentPick::CreateUObject(this, &ADroneDemoHUD::PickEnvironment))
		.OnTakeoff(FSimpleDelegate::CreateLambda([this]() { if (ADroneDemoPawn* P = GetDronePawn()) P->RequestTakeoffFromUi(); }))
		.OnLand(FSimpleDelegate::CreateLambda([this]() { if (ADroneDemoPawn* P = GetDronePawn()) P->RequestLandFromUi(); }))
		.OnReport(FSimpleDelegate::CreateLambda([this]() { StartReportJob(); }))
		.OnClose(FSimpleDelegate::CreateLambda([this]() { SetScreen(EScreen::Flight); }));
	ReportScreen = SNew(SDroneReportScreen)
		.UiScale(UiScale)
		.OnExport(FSimpleDelegate::CreateLambda([this]() { ExportReport(); }))
		.OnRerun(FSimpleDelegate::CreateLambda([this]() { StartReportJob(); }))
		.OnClose(FSimpleDelegate::CreateLambda([this]() { SetScreen(EScreen::Flight); }));

	// Layout of the roots: menu and report fill the screen; the OSD fills it too; the panel sits at the left edge.
	Roots[0] = StartMenu;
	Roots[1] = Osd;
	Roots[2] = SNew(SBox).HAlign(HAlign_Left).VAlign(VAlign_Center).Padding(16.0f)[ControlPanel.ToSharedRef()];
	Roots[3] = ReportScreen;
	for (int32 I = 0; I < 4; ++I)
	{
		Roots[I]->SetVisibility(EVisibility::Collapsed);
		GEngine->GameViewport->AddViewportWidgetContent(Roots[I].ToSharedRef(), 10 + I);
	}
	bWidgetsBuilt = true;

	// "?SkipMenu=1" comes from the start menu's OpenLevel: go straight to flight in the new level.
	bool bSkip = false;
	if (const UWorld* World = GetWorld())
		if (const AGameModeBase* Mode = World->GetAuthGameMode()) bSkip = !UGameplayStatics::ParseOption(Mode->OptionsString, TEXT("SkipMenu")).IsEmpty();
	Report.Banner = Utf8(DroneReport::kStatusBanner);
	SetScreen(bSkip ? EScreen::Flight : EScreen::StartMenu);
	Alpha[static_cast<int32>(Screen)] = 1.0;
}

void ADroneDemoHUD::SetScreen(EScreen S)
{
	Screen = S;
	APlayerController* PC = GetOwningPlayerController();
	ADroneDemoPawn* Pawn = GetDronePawn();
	const bool bFlight = S == EScreen::Flight;
	if (Pawn)
	{
		Pawn->SetPilotInputEnabled(bFlight);                      // the panel, menu and report pause pilot input
		Pawn->SetSimulationActive(S != EScreen::StartMenu);       // the start menu pauses the simulation
	}
	if (PC)
	{
		PC->bShowMouseCursor = !bFlight;
		if (bFlight) PC->SetInputMode(FInputModeGameOnly());
		else
		{
			FInputModeGameAndUI Mode;
			Mode.SetHideCursorDuringCapture(false);
			PC->SetInputMode(Mode);
		}
	}
	if (S == EScreen::StartMenu) RefreshStartMenu();
}

void ADroneDemoHUD::UpdateFades(float Dt)
{
	// The OSD stays visible under the panel and the report; the menu and report replace it.
	const double Target[4] = {Screen == EScreen::StartMenu ? 1.0 : 0.0, Screen == EScreen::StartMenu || Screen == EScreen::Report ? 0.0 : 1.0,
							  Screen == EScreen::ControlPanel ? 1.0 : 0.0, Screen == EScreen::Report ? 1.0 : 0.0};
	for (int32 I = 0; I < 4; ++I)
	{
		Alpha[I] = DroneUi::FadeStep(Alpha[I], Target[I], Dt);
		if (!Roots[I].IsValid()) continue;
		Roots[I]->SetRenderOpacity(static_cast<float>(Alpha[I]));
		Roots[I]->SetVisibility(Alpha[I] > 0.001 ? (I == 1 ? EVisibility::SelfHitTestInvisible : EVisibility::Visible) : EVisibility::Collapsed);
	}
}

void ADroneDemoHUD::SmoothGauges(float Dt)
{
	FDroneUiSmooth& S = Snap.Smooth;
	const FDroneUiRaw& R = Snap.Raw;
	S.Altitude.Step(R.AltitudeM, kGaugeOmegaPerS, Dt);
	S.Speed.Step(R.SpeedMps, kGaugeOmegaPerS, Dt);
	S.Climb.Step(R.ClimbMps, kGaugeOmegaPerS, Dt);
	S.Roll.Step(R.RollRad, 12.0, Dt);
	S.Pitch.Step(R.PitchRad, 12.0, Dt);
	// Heading wraps: smooth the shortest-way difference so the tape never spins backwards through 360.
	double H = R.HeadingDeg;
	if (S.Heading.bSet)
	{
		const double Cur = DroneUi::WrapDeg360(S.Heading.X);
		double Diff = H - Cur;
		if (Diff > 180.0) Diff -= 360.0;
		if (Diff < -180.0) Diff += 360.0;
		H = S.Heading.X + Diff;
	}
	S.Heading.Step(H, kGaugeOmegaPerS, Dt);
	for (int32 M = 0; M < 4; ++M) S.Rpm[M].Step(R.RpmFraction[M], kGaugeOmegaPerS, Dt);
}

void ADroneDemoHUD::Tick(float Dt)
{
	Super::Tick(Dt);
	if (!bWidgetsBuilt) return;
	ADroneDemoPawn* Pawn = GetDronePawn();
	if (!Pawn) return;

	// A menu with no cards yet (the pawn loads the catalog in BeginPlay): refresh once the pawn is there.
	static bool bMenuFilled = false;
	if (!bMenuFilled && Screen == EScreen::StartMenu)
	{
		RefreshStartMenu();
		bMenuFilled = true;
	}

	if (APlayerController* PC = GetOwningPlayerController())
	{
		if (PC->WasInputKeyJustPressed(EKeys::Tab))
		{
			if (Screen == EScreen::Flight) SetScreen(EScreen::ControlPanel);
			else if (Screen == EScreen::ControlPanel) SetScreen(EScreen::Flight);
		}
		if (PC->WasInputKeyJustPressed(EKeys::R) && Screen == EScreen::Flight) StartReportJob();
	}
	if (Pawn->ConsumeLandedEvent() && Screen == EScreen::Flight) StartReportJob();

	SmoothGauges(Dt);
	if (DataRate.Due(Dt)) UpdateData();
	MarginAccumS += Dt;
	if (MarginAccumS >= kMarginPeriodS)
	{
		MarginAccumS = 0.0;
		UpdateMargins();
	}
	UpdateFades(Dt);
}

void ADroneDemoHUD::UpdateData()
{
	ADroneDemoPawn* Pawn = GetDronePawn();
	if (!Pawn) return;
	FDroneUiRaw& R = Snap.Raw;
	Pawn->FillUiRaw(R);
	if (!Snap.Smooth.Altitude.bSet)  // first data: the gauges start where the vehicle is
	{
		Snap.Smooth.Altitude.Snap(R.AltitudeM);
		Snap.Smooth.Speed.Snap(R.SpeedMps);
		Snap.Smooth.Climb.Snap(R.ClimbMps);
		Snap.Smooth.Roll.Snap(R.RollRad);
		Snap.Smooth.Pitch.Snap(R.PitchRad);
		Snap.Smooth.Heading.Snap(R.HeadingDeg);
		for (int32 M = 0; M < 4; ++M) Snap.Smooth.Rpm[M].Snap(R.RpmFraction[M]);
	}
	Snap.Charts.Altitude.Push(R.AltitudeM);
	Snap.Charts.Speed.Push(R.SpeedMps);
	Snap.Charts.BatteryA.Push(R.BatteryA);
	Snap.Charts.ThrottleMean.Push(R.ThrottleMean);

	FDroneUiText& T = Snap.Text;
	char Clock[16];
	DroneUi::FormatClock(Clock, sizeof(Clock), R.FlightTimeS);
	T.Mode = Pawn->GetModeString();
	T.Environment = Pawn->GetEnvironmentId().ToUpper();
	T.Clock = FString(UTF8_TO_TCHAR(Clock));
	T.Altitude = Num(R.AltitudeM, 1, 6);
	T.Speed = Num(R.SpeedMps, 1, 6);
	T.Heading = FString::Printf(TEXT("HDG %03.0f"), R.HeadingDeg);
	T.Wind = FString::Printf(TEXT("WIND %.1f m/s  gust %.1f"), R.WindMps, R.GustMps);
	T.Battery = FString::Printf(TEXT("BAT %s V   %s A   used %.0f %%"), *Num(R.PackV, 1), *Num(R.BatteryA, 1), 100.0 * R.BatteryUsedFraction);
	// No surrogate model is loaded by the demo: drag is the core's analytic model, and the screen says so.
	T.Density = FString::Printf(TEXT("AIR %.3f kg/m3   AERO analytic drag (no CFD model loaded)"), R.AirDensityKgM3);
	T.Temp = FString::Printf(TEXT("site %.0f m   ambient %.1f C   humidity not modelled"), R.SiteAltitudeM, R.AmbientTempC);
	T.Notice = R.bPhysicsFault ? FString(TEXT("PHYSICS FAULT: data below is not valid")) : Pawn->GetEnvironmentNotice();
	static const TCHAR* const kNames[4] = {TEXT("FR"), TEXT("FL"), TEXT("RR"), TEXT("RL")};
	for (int32 M = 0; M < 4; ++M)
	{
		T.Rpm[M] = FString::Printf(TEXT("%s %.0f"), kNames[M], R.Rpm[M]);
		T.Thrust[M] = FString::Printf(TEXT("%.1f N"), R.ThrustN[M]);
	}
	if (Osd.IsValid()) Osd->UpdateText();
}

void ADroneDemoHUD::UpdateMargins()
{
	if (!Osd.IsValid()) return;
	const FDroneUiRaw& R = Snap.Raw;
	const DroneReport::Build B = DroneReport::Build::Reference();
	auto M = [](double Limit, double Peak) {
		double Out = DroneElec::kNoLoadMargin;
		DroneElec::Margin(Limit, Peak, &Out);
		return Out;
	};
	const double Usable = B.BatteryCapacityAh * B.BatteryUsable;
	TArray<TPair<FString, double>> Rows;
	Rows.Add(TPair<FString, double>(TEXT("Prop thrust"), R.MaxThrustMarginUtil > 0.0 ? M(1.0, R.MaxThrustMarginUtil) : DroneElec::kNoLoadMargin));
	Rows.Add(TPair<FString, double>(TEXT("Motor current"), M(B.MotorMaxCurrentA, R.MotorCurrentMaxA)));
	Rows.Add(TPair<FString, double>(TEXT("ESC current"), M(B.EscRatedA, R.MotorCurrentMaxA)));
	Rows.Add(TPair<FString, double>(TEXT("Battery current"), M(B.BatteryMaxC * B.BatteryCapacityAh, R.BatteryA)));
	Rows.Add(TPair<FString, double>(TEXT("Battery energy"), M(Usable, R.BatteryUsedFraction * Usable)));
	Osd->SetMargins(Rows);
}

void ADroneDemoHUD::RefreshStartMenu()
{
	ADroneDemoPawn* Pawn = GetDronePawn();
	if (!Pawn || !StartMenu.IsValid()) return;
	TArray<SDroneStartMenu::FCard> Cards;
	for (const FDroneEnvironmentEntry& E : Pawn->GetEnvironmentCatalog().Entries())
	{
		SDroneStartMenu::FCard C;
		C.Id = UTF8_TO_TCHAR(E.Preset.Id.c_str());
		C.Name = UTF8_TO_TCHAR(E.Preset.Name.c_str());
		double Rho = 0.0;
		DroneEnv::DensityKgM3(E.Preset.SiteAltitudeM, E.Preset.AmbientTempC, &Rho);
		C.Detail = FString::Printf(TEXT("%.0f m   %.0f C   wind %.0f m/s (gust %.1f)   rho %.3f"), E.Preset.SiteAltitudeM, E.Preset.AmbientTempC, E.Preset.WindSpeedMps, E.Preset.GustSigmaMps, Rho);
		bool bMissing = true;
		FDroneEnvironmentCatalog::ResolveLevelPackage(E, bMissing);
		C.bLevelInstalled = !bMissing;
		C.bValid = E.ValidationMessage.IsEmpty();
		C.Problem = E.ValidationMessage;
		Cards.Add(C);
	}
	if (SelectedEnvironmentId.IsEmpty()) SelectedEnvironmentId = Pawn->GetEnvironmentId();
	StartMenu->SetCards(Cards, SelectedEnvironmentId);
}

void ADroneDemoHUD::StartFlight(const FString& EnvId)
{
	ADroneDemoPawn* Pawn = GetDronePawn();
	if (!Pawn) return;
	const FDroneEnvironmentEntry* Entry = Pawn->GetEnvironmentCatalog().Find(EnvId);
	if (!Entry)
	{
		SetScreen(EScreen::Flight);
		return;
	}
	bool bMissing = true;
	const FString Package = FDroneEnvironmentCatalog::ResolveLevelPackage(*Entry, bMissing);
	if (!bMissing && FPackageName::GetShortName(Package) != UGameplayStatics::GetCurrentLevelName(this, true))
	{
		// Another level: open it; its HUD starts straight in flight ("?SkipMenu=1") with this environment's physics.
		UGameplayStatics::OpenLevel(this, FName(*Package), true, FString::Printf(TEXT("Env=%s?SkipMenu=1"), *EnvId));
		return;
	}
	FString Message;
	if (!Pawn->ApplyEnvironment(*Entry, Message)) Pawn->SetEnvironmentNotice(Message);
	else if (bMissing) Pawn->SetEnvironmentNotice(FString::Printf(TEXT("Level for %s is not installed: flying in the default level"), *EnvId));
	else Pawn->SetEnvironmentNotice(FString());
	SetScreen(EScreen::Flight);
}

void ADroneDemoHUD::PickEnvironment(const FString& Id)
{
	StartFlight(Id);  // same path as the menu: opens its level if installed, else applies the physics here
}

void ADroneDemoHUD::ApplyWeather(const FDroneUiWeatherInput& In)
{
	ADroneDemoPawn* Pawn = GetDronePawn();
	if (!Pawn || !ControlPanel.IsValid()) return;
	auto Parse = [](const FString& S, const TCHAR* What, double& Out, FString& Err) {
		const FString T = S.TrimStartAndEnd();
		if (!FCString::IsNumeric(*T)) { Err = FString::Printf(TEXT("%s is not a number"), What); return false; }
		Out = FCString::Atod(*T);
		return true;
	};
	double Wind = 0, Dir = 0, Gust = 0, Alt = 0, Temp = 0, Hum = 0;
	FString Err;
	if (!Parse(In.WindSpeed, TEXT("Wind speed"), Wind, Err) || !Parse(In.WindDirection, TEXT("Wind direction"), Dir, Err) || !Parse(In.Gust, TEXT("Gust level"), Gust, Err) ||
		!Parse(In.Altitude, TEXT("Altitude"), Alt, Err) || !Parse(In.Temperature, TEXT("Temperature"), Temp, Err) || !Parse(In.Humidity, TEXT("Humidity"), Hum, Err))
	{
		ControlPanel->SetMessage(Err, true);
		return;
	}
	FString Message;
	if (!Pawn->SetWeather(Wind, Dir, Gust, Alt, Temp, Hum, Message))
	{
		ControlPanel->SetMessage(FString::Printf(TEXT("Rejected, nothing changed: %s"), *Message), true);  // never clamped
		return;
	}
	ControlPanel->SetMessage(FString::Printf(TEXT("Applied: air density %.3f kg/m3, wind %.1f m/s"), Pawn->GetAirDensityKgM3(), Wind), false);
}

void ADroneDemoHUD::StartReportJob()
{
	ADroneDemoPawn* Pawn = GetDronePawn();
	if (!Pawn || bReportRunning) return;
	bReportRunning = true;
	Report.bRunning = true;
	Report.bReady = false;
	Report.Message = FString();
	Report.Banner = Utf8(DroneReport::kStatusBanner);
	ReportScreen->SetReport(Report);
	SetScreen(EScreen::Report);

	const VehicleConfig Config = Pawn->GetVehicleConfig();
	const DroneState Start = FDronePhysicsBridgeModule::CreateDefaultState();
	const ThermalState Thermal = FDronePhysicsBridgeModule::CreateDefaultThermalState();
	const DroneReport::EnvInfo Flown = Pawn->MakeEnvInfo();
	std::vector<DroneReport::EnvInfo> Envs;
	for (std::size_t I = 0; I < DroneEnvTable::kRowCount; ++I) Envs.push_back(DroneRun::EnvFromRow(DroneEnvTable::kRows[I]));
	const std::string Utc = TCHAR_TO_UTF8(*FDateTime::UtcNow().ToIso8601());
	const FString Folder = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Reports") / Utf8(Flown.Id));
	TWeakObjectPtr<ADroneDemoHUD> Weak(this);

	// The same physics step the pawn uses (stateless with no surrogate handles), on a worker thread: a few seconds.
	Async(EAsyncExecution::ThreadPool, [Weak, Config, Start, Thermal, Flown, Envs, Utc, Folder]() {
		auto Step = [](DroneState* S, ThermalState* T, const ControlInputs* C, const WindInput* W, const EnvironmentInput* E, const VehicleConfig* Cfg,
					   DebugTelemetry* Tele, FaultLatch* L, double Dt) { return FDronePhysicsBridgeModule::StepPhysics(S, T, C, W, E, nullptr, nullptr, Cfg, Tele, L, Dt) ? 0 : 1; };
		TSharedPtr<FDroneReportPayload> Payload = MakeShared<FDroneReportPayload>();
		Payload->Folder = Folder;
		Payload->Bundle = DroneRun::BuildBundle(Step, Config, Start, Thermal, Flown, Envs, kReportSeed, Utc);
		WriteReportFiles(Payload->Bundle, Folder, Payload->Error);
		AsyncTask(ENamedThreads::GameThread, [Weak, Payload]() {
			if (Weak.IsValid()) Weak->OnReportReady(Payload);
		});
	});
}

void ADroneDemoHUD::OnReportReady(TSharedPtr<FDroneReportPayload> Payload)
{
	bReportRunning = false;
	LastPayload = Payload;
	Report.bRunning = false;
	Report.bReady = true;
	ConvertBundle(Payload->Bundle, Report);
	Report.FolderPath = Payload->Folder;
	Report.Message = Payload->Error.IsEmpty() ? FString(TEXT("Report written (HTML, JSON, CSV). The standard mission was flown with the same seed in every environment.")) : Payload->Error;
	ReportScreen->SetReport(Report);
}

void ADroneDemoHUD::ExportReport()
{
	if (!LastPayload.IsValid()) return;
	FString Error;
	if (WriteReportFiles(LastPayload->Bundle, LastPayload->Folder, Error)) FPlatformProcess::ExploreFolder(*LastPayload->Folder);
	Report.Message = Error.IsEmpty() ? FString::Printf(TEXT("Exported to %s"), *LastPayload->Folder) : Error;
	ReportScreen->SetReport(Report);
}
