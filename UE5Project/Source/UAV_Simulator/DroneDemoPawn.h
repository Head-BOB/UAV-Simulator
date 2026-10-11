// Tier: V
// DEMO pawn, not flight code. Owns the fixed-step loop that calls the Rust physics core, a demo
// flight controller (DroneDemoControl.h), the demo wind (DroneWind.h), the skeletal drone model
// with spinning props, wind arrows, and the HUD. Unreal only renders and takes input here:
// the physics truth is the Rust core. NOT COMPILED OR RUN by its author (no Unreal in the
// sandbox): every Unreal API use is TODO(HUMAN-VERIFY: builds on the project's Unreal version).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "DronePhysicsBridge.h"
#include "DroneDemoControl.h"
#include "DroneFly.h"
#include "DroneWind.h"
#include "DroneEnvironment.h"
#include "DroneEnvironmentCatalog.h"
#include "DroneReport.h"
#include "UI/DroneUiSnapshot.h"
#include "DroneDemoPawn.generated.h"

class UPoseableMeshComponent;
class USpringArmComponent;
class UCameraComponent;
class UStaticMeshComponent;
class UMaterialInstanceDynamic;
class UMaterialInterface;
class APlayerController;

UCLASS()
class UAV_SIMULATOR_API ADroneDemoPawn : public APawn
{
	GENERATED_BODY()

public:
	ADroneDemoPawn();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	// Applies a preset: atmosphere for the core (EnvironmentInput), steady wind, gust level. Returns false and
	// OutMessage when the preset fails DroneEnv validation; nothing changes then.
	bool ApplyEnvironment(const FDroneEnvironmentEntry& Entry, FString& OutMessage);
	// Typed weather from the control panel. Validated by DroneEnv::ValidateWeather (nothing is clamped):
	// returns false and a message to show on screen, and changes nothing.
	bool SetWeather(double WindSpeedMps, double WindTowardDeg, double GustSigmaMps, double SiteAltitudeM, double AmbientTempC,
					double HumidityPct, FString& OutMessage);

	const FString& GetEnvironmentName() const { return EnvironmentName; }
	const FString& GetEnvironmentNotice() const { return EnvironmentNotice; }
	double GetAirDensityKgM3() const { return AirDensityKgM3; }
	double GetSiteAltitudeM() const { return SiteAltitudeM; }
	double GetAmbientTempC() const { return AmbientTempC; }
	double GetHumidityPct() const { return HumidityPct; }
	const FDroneEnvironmentCatalog& GetEnvironmentCatalog() const { return Catalog; }
	const FString& GetEnvironmentId() const { return EnvironmentName; }

	// ---- UI hooks (ADroneDemoHUD). All run on the game thread. ----
	// Fills the raw numbers of the flight display from the current state (no allocation).
	void FillUiRaw(FDroneUiRaw& Out) const;
	FString GetModeString() const;
	// The flight display can pause pilot input (control panel open) or the whole simulation (start menu).
	void SetPilotInputEnabled(bool bEnabled) { bPilotInputEnabled = bEnabled; }
	void SetSimulationActive(bool bActive) { bSimulationActive = bActive; }
	bool IsSimulationActive() const { return bSimulationActive; }
	void RequestTakeoffFromUi();
	void RequestLandFromUi();
	// True once after each landing (then false until the next one): the HUD opens the report.
	bool ConsumeLandedEvent();
	// The vehicle and atmosphere the report is flown for (what the pawn is using right now).
	const VehicleConfig& GetVehicleConfig() const { return Config; }
	DroneReport::EnvInfo MakeEnvInfo() const;
	void SetEnvironmentNotice(const FString& Text) { EnvironmentNotice = Text; }

	// ---- Replay viewer (P7) ----
	// The application replays a RECORDED campaign run: physics stepping and the demo controller are off, this pawn only hosts the camera,
	// the environment and the wind arrows. The vehicle itself is drawn by ADroneBuildPreview (the user's own build, not the demo model).
	void SetReplayMode(bool bEnable);
	bool IsReplayMode() const { return bReplayMode; }
	// Camera target and wind arrow from a recorded sample: NED metres, NED orientation (x, y, z, w), NED m/s. Position and rotation go through
	// the bridge's audited NED-to-Unreal conversion.
	void SetReplayView(const double PosNed[3], const float OrientNed[4], const double WindNed[3]);

	// ---- Fly mode (live physics, standard model of the class; NOT the user's build) ----
	// Keys / gamepad go through DroneApp::Fly (engine-free, tested). The pawn flies with the demo controller in assisted mode; ACRO is not
	// implemented in the demo controller: TODO(HUMAN-VERIFY) it flies assisted and the HUD says so.
	void SetFlyMode(bool bOn);
	bool IsFlyMode() const { return bFlyMode; }
	bool IsCrashed() const { return bCrashed; }
	// Applies the live wind panel: wind, gusts, air temperature and payload mass reach the physics through the same WindInput /
	// EnvironmentInput as the demo wind. Cheap when nothing changed.
	void ApplyFlyPanel(const DroneApp::Fly::WindPanel& Panel);
	// X / gamepad Y: back to the ground, level, battery full, crash cleared.
	void ResetFlight();
	// The wind the physics step received (NED m/s) and the gust part, for the HUD wind rose.
	void GetWindForHud(double OutTotalNed[3], double& OutGustMps) const;

private:
	// One motor's bones, in the order of DroneModelMap.json (FR, FL, RR, RL = config index order).
	struct FMotorVisual
	{
		FName HubBone;
		TArray<FName> SpinBones;
		TArray<FQuat> RestComponentRot;  // component-space rest rotation of each spin bone
		double SpinAngleDeg = 0.0;
		bool bBladesHidden = false;  // blades hidden, motion disc shown (DroneVisual::BladesHidden)
	};

	// Setup
	bool LoadPhysics();
	bool LoadModelMap(double& OutModelLeftRightM, double& OutModelFrontRearM);
	void SetupVisuals();
	void BuildFallbackShapes();
	void BuildMotionDiscs();
	void SetupEnvironment();
	void PlaceOnGround();

	// Per frame
	void ReadInput(APlayerController* PC, DroneDemo::Command& OutCommand);
	void ReadWindKeys(APlayerController* PC);
	void StepFixed(double FixedDt, const DroneDemo::Command& Command);
	void PinToGround();
	void UpdateVisuals(float DeltaTime);
	void DrawWindArrows() const;
	void DrawHud() const;

	// Physics state (Rust owns the truth; these are plain data passed through the FFI).
	DroneState PhysicsState = {};
	EnvironmentInput PhysicsEnvironment = {};  // zeros = ISA standard day at sea level (interface v12)
	ThermalState Thermal = {};
	VehicleConfig Config = {};
	FaultLatch PhysicsFaultLatch = {};
	DebugTelemetry LastTelemetry = {};
	bool bHaveTelemetry = false;
	bool bPhysicsReady = false;
	bool bPhysicsFault = false;
	double Accumulator = 0.0;
	int32 DroppedTimeEvents = 0;

	// Environment (interface v12). Density is derived from altitude and temperature; humidity is shown only.
	FDroneEnvironmentCatalog Catalog;
	FString EnvironmentName = TEXT("Standard day");
	FString EnvironmentNotice;           // shown on screen when something fell back (file missing, level missing)
	double SiteAltitudeM = 0.0;
	double AmbientTempC = 15.0;
	double HumidityPct = 0.0;
	double AirDensityKgM3 = 1.2250122659906946;
	void SelectStartEnvironment();
	bool bPilotInputEnabled = true;
	bool bSimulationActive = true;
	bool bLandedEvent = false;
	bool bFlewSinceLanding = false;
	double FlightTimeS = 0.0;
	// Electrical state integrated at the physics rate (DroneElectrical.h with ASSUMED parameters).
	DroneReport::Build ReportBuild = DroneReport::Build::Reference();
	double BatteryUsedAh = 0.0;
	double LastBatteryA = 0.0;
	double LastPackV = 0.0;
	double LastMotorCurrentA[4] = {0, 0, 0, 0};
	void ApplyEnvironmentLook(const FDroneEnvironmentEntry* Entry);

	// Demo controller and wind.
	TUniquePtr<DroneDemo::Controller> DemoController;   // not `Controller`: APawn::Controller (AController*) is a base member
	DroneWind::Params WindParams;
	DroneWind::Generator WindGenerator;
	DroneWind::Sample WindSample = {};

	// World placement: the NED origin in Unreal world space (cm).
	FVector WorldOriginCm = FVector::ZeroVector;

	// Visuals
	UPROPERTY() TObjectPtr<USceneComponent> Root;
	UPROPERTY() TObjectPtr<UPoseableMeshComponent> PoseMesh;
	UPROPERTY() TObjectPtr<USpringArmComponent> CameraArm;
	UPROPERTY() TObjectPtr<UCameraComponent> Camera;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> MotionDiscs;
	UPROPERTY() TArray<TObjectPtr<UMaterialInstanceDynamic>> DiscMaterials;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> FallbackShapes;  // body + 4 props when the model is unusable
	TArray<FMotorVisual> Motors;
	bool bUseFallbackShapes = false;
	bool bDiscMaterialMissingLogged = false;
	bool bReplayMode = false;
	bool bFlyMode = false;
	bool bCrashed = false;
	double PrevGustMps = 0.0;
	double GustPulseValue = 0.0;
	double BaseMassKg = 0.0;
	VehicleConfig BaseConfig = {};
	DroneDemo::Tuning LastTune;
	double LastPanelKey[6] = {-1e9, -1e9, -1e9, -1e9, -1e9, -1e9};
};
