// Tier: V
// NOT COMPILED OR RUN by its author. See DroneAppHUD.h.
#include "DroneAppHUD.h"

#include "DroneBuildPreview.h"
#include "DroneDataPaths.h"
#include "DroneDemoPawn.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "UI/DroneAppIO.h"
#include "UI/SDroneAppCanvas.h"
#include "Widgets/SWeakWidget.h"

#include "DroneScreens.h"

namespace
{
constexpr double kDataPeriodS = 1.0 / 30.0;   // 30 Hz data refresh (design spec)
}

ADroneAppHUD::ADroneAppHUD()
{
	PrimaryActorTick.bCanEverTick = true;
}

ADroneDemoPawn* ADroneAppHUD::GetDronePawn() const
{
	return Cast<ADroneDemoPawn>(GetOwningPawn());
}

// helpers.json (DroneData/helpers/ in a packaged build, or the repo root in the editor): {"python", "repo_root", "run_campaign_exe", "validate_models_exe", "threads", "git_hash"}.
// Without a file the repo root is the folder above the project (editor use) and the helpers run through `python` and `cargo`.
bool ADroneAppHUD::LoadHelperSettings(DroneApp::HelperSettings& Out, FString& OutWorkDir) const
{
	FString Root = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT(".."));
	FPaths::NormalizeDirectoryName(Root);
	// packaged: DroneData/helpers/helpers.json (written by tools/package/stage_package.py); editor: helpers.json in the repo root
	FString File = DroneData::Resolve(TEXT("helpers/helpers.json"));
	if (File.IsEmpty()) File = DroneData::Resolve(TEXT("helpers.json"));
	std::string Err;
	if (!File.IsEmpty())
	{
		FString Text;
		if (FFileHelper::LoadFileToString(Text, *File) && DroneApp::ParseHelperSettings(std::string(TCHAR_TO_UTF8(*Text)), Out, Err))
		{
			// a packaged build ships relative paths: resolve them against the folder that holds helpers.json (DroneData/helpers)
			DroneApp::ResolveHelperPaths(Out, std::string(TCHAR_TO_UTF8(*FPaths::GetPath(File))));
			Out.Progress = true;   // the helpers print UAVPROGRESS lines
			OutWorkDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("DroneApp"));
			return true;
		}
		UE_LOG(LogTemp, Warning, TEXT("DroneApp: helpers.json unusable (%s); using defaults."), *FString(UTF8_TO_TCHAR(Err.c_str())));
	}
	Out = DroneApp::HelperSettings();
	Out.RepoRoot = std::string(TCHAR_TO_UTF8(*Root));
	Out.Progress = true;
	OutWorkDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("DroneApp"));
	return true;
}

void ADroneAppHUD::BeginPlay()
{
	Super::BeginPlay();
	DroneApp::HelperSettings H;
	FString Work;
	LoadHelperSettings(H, Work);
	IO = std::make_unique<FDroneAppIO>(FString(UTF8_TO_TCHAR(H.RepoRoot.c_str())));
	Model = std::make_unique<DroneApp::AppModel>(*IO, H, std::string(TCHAR_TO_UTF8(*Work)));
	if (!Model->Init()) UE_LOG(LogTemp, Error, TEXT("DroneApp: %s"), *FString(UTF8_TO_TCHAR(Model->Message.c_str())));

	if (UWorld* World = GetWorld())
	{
		FActorSpawnParameters P;
		P.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Preview = World->SpawnActor<ADroneBuildPreview>(FVector::ZeroVector, FRotator::ZeroRotator, P);
	}
	if (ADroneDemoPawn* Pawn = GetDronePawn()) Pawn->SetReplayMode(true);   // the demo's own flight stays off: the app shows recorded runs

	AppCanvas = SNew(SDroneAppCanvas).Model(Model.get()).List(&List).OnChanged(FSimpleDelegate::CreateUObject(this, &ADroneAppHUD::MarkDirty));
	if (GEngine && GEngine->GameViewport) GEngine->GameViewport->AddViewportWidgetContent(SNew(SWeakWidget).PossiblyNullContent(AppCanvas.ToSharedRef()), 10);
	if (APlayerController* PC = GetOwningPlayerController())
	{
		FInputModeGameAndUI Mode;
		Mode.SetWidgetToFocus(AppCanvas);
		Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
		PC->SetInputMode(Mode);
		PC->bShowMouseCursor = true;
	}
	Rebuild();
}

void ADroneAppHUD::EndPlay(const EEndPlayReason::Type Reason)
{
	if (GEngine && GEngine->GameViewport && AppCanvas.IsValid()) GEngine->GameViewport->RemoveAllViewportWidgets();
	AppCanvas.Reset();
	Super::EndPlay(Reason);
}

void ADroneAppHUD::Rebuild()
{
	if (!Model) return;
	DroneApp::BuildScreen(List, *Model);
	bDirty = false;
}

void ADroneAppHUD::UpdateScene(float DeltaSeconds)
{
	(void)DeltaSeconds;
	ADroneDemoPawn* Pawn = GetDronePawn();
	if (!Pawn || !Preview || !Model) return;
	// Fly mode: the live physics pawn flies and its numbers feed the fly HUD; the recorded-run preview is hidden.
	if (Model->FlyMode != Pawn->IsFlyMode()) Pawn->SetFlyMode(Model->FlyMode);
	if (Model->FlyMode)
	{
		Preview->SetActorHiddenInGame(true);
		Pawn->ApplyFlyPanel(Model->WindP);
		Pawn->SetPilotInputEnabled(Model->Focus.empty());   // typing in a wind field must not fly the vehicle
		FDroneUiRaw Raw;
		Pawn->FillUiRaw(Raw);
		auto& T = Model->FlyT;
		T.AltitudeM = Raw.AltitudeM;
		T.SpeedMps = Raw.SpeedMps;
		for (int I = 0; I < 4; ++I) T.Rpm[I] = Raw.Rpm[I];
		T.RpmMean = (Raw.Rpm[0] + Raw.Rpm[1] + Raw.Rpm[2] + Raw.Rpm[3]) * 0.25;
		T.SocFraction = FMath::Clamp(1.0 - Raw.BatteryUsedFraction, 0.0, 1.0);
		Pawn->GetWindForHud(T.WindNed, T.GustMps);
		T.Crashed = Pawn->IsCrashed();
		T.StateText = std::string(TCHAR_TO_UTF8(*Pawn->GetModeString())) + (Model->FlyModeSel == DroneApp::Fly::Mode::Acro ? "   (acro is not implemented in the live model: flying assisted)" : "");
		bDirty = true;   // numbers change every frame
		return;
	}
	// Build screen and Run screen show the user's vehicle; the others show the empty scene behind their panels.
	const bool bShowVehicle = Model->Current == DroneApp::Screen::Build || Model->Current == DroneApp::Screen::Run;
	if (Model->Prog.BuildDone && !Model->Prims.empty() && LastPreviewBuildDir != Model->BuildDir)
	{
		Preview->SetPrimitives(Model->Prims);
		LastPreviewBuildDir = Model->BuildDir;
	}
	if (!Model->Prog.BuildDone && !LastPreviewBuildDir.empty()) { Preview->ClearPrimitives(); LastPreviewBuildDir.clear(); }
	Preview->SetActorHiddenInGame(!bShowVehicle || !Preview->HasParts());
	if (!bShowVehicle) return;

	if (Model->Current == DroneApp::Screen::Run && !Model->ReplayTest.empty())
	{
		const DroneApp::Pose& P = Model->Now;
		const float Q[4] = {static_cast<float>(P.Q.X), static_cast<float>(P.Q.Y), static_cast<float>(P.Q.Z), static_cast<float>(P.Q.W)};
		Pawn->SetReplayView(P.PosNed, Q, P.WindNed);
		Preview->SetActorLocationAndRotation(Pawn->GetActorLocation(), Pawn->GetActorRotation());
		// heat tint from the recorded temperatures (the same function and limits the detail panel uses)
		auto Limit = [&](const char* K) { const DroneApp::Json::Value* V = Model->Powertrain.Get(K); return V ? V->AsNumber(0.0) : 0.0; };
		const double Amb = Model->Camp.Ambient();
		auto Tint = [&](double T, double Lim) {
			const DroneApp::Rgba C = DroneApp::Screens::HeatTint(DroneApp::Screens::HeatFraction(T, Lim, Amb));
			FLinearColor L = FLinearColor::FromSRGBColor(FColor(static_cast<uint8>(C.R * 255), static_cast<uint8>(C.G * 255), static_cast<uint8>(C.B * 255), 255));
			return L;
		};
		Preview->SetHeatTints(Tint(P.WindingC, Limit("motor_winding_temp_limit_c")), Tint(P.EscC, Limit("esc_temp_limit_c")), Tint(P.BatteryC, Limit("battery_temp_limit_c")));
	}
	else
	{
		// Build screen: the vehicle at the origin, level, camera on it.
		const double Zero[3] = {0.0, 0.0, -0.3};
		const float Identity[4] = {0.f, 0.f, 0.f, 1.f};
		const double NoWind[3] = {0.0, 0.0, 0.0};
		Pawn->SetReplayView(Zero, Identity, NoWind);
		Preview->SetActorLocationAndRotation(Pawn->GetActorLocation(), Pawn->GetActorRotation());
		Preview->ResetTints();
	}
}

void ADroneAppHUD::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!Model) return;
	const DroneApp::Screen Before = Model->Current;
	Model->Tick(static_cast<double>(DeltaSeconds));
	DataAccumS += static_cast<double>(DeltaSeconds);
	const bool bMoving = (Model->Current == DroneApp::Screen::Run && Model->Rep.Playing()) || Model->Busy();
	if (Model->Current != LastScreen || Before != Model->Current) { bDirty = true; LastScreen = Model->Current; }
	if (Model->TakeRepaint()) bDirty = true;   // a helper ended / said something / the message changed: repaint without waiting for input
	if (bDirty || (bMoving && DataAccumS >= kDataPeriodS))
	{
		Rebuild();
		DataAccumS = 0.0;
	}
	UpdateScene(DeltaSeconds);
}
