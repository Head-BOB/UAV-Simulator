// Tier: V
// DEMO, not flight code. See DroneDemoPawn.h. NOT COMPILED OR RUN by its author.
#include "DroneDemoPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/PoseableMeshComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "DrawDebugHelpers.h"
#include "DroneVisualMath.h"
#include "Engine/Engine.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "HAL/IConsoleManager.h"
#include "InputCoreTypes.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
// Asset paths. TODO(HUMAN-VERIFY): confirm after importing the model (README step 3).
const TCHAR* const kDroneMeshPath = TEXT("/Game/Drone/Drone.Drone");
// Translucent material with scalar parameter "Opacity" and vector parameter "Color", created by
// hand in the editor (README step 4). If it is missing the discs are hidden and a Warning is logged.
const TCHAR* const kDiscMaterialPath = TEXT("/Game/Drone/M_MotionDisc.M_MotionDisc");
const TCHAR* const kCylinderMeshPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
const TCHAR* const kCubeMeshPath = TEXT("/Engine/BasicShapes/Cube.Cube");
const TCHAR* const kPlaneMeshPath = TEXT("/Engine/BasicShapes/Plane.Plane");

// Required credit (CC BY 4.0). Also in the plugin README. Do not edit DroneModelMap.json's copy.
const TCHAR* const kModelCredit = TEXT("Model: Animated Drone by hartwelkisaka, CC BY 4.0");

// Demo values. TODO(HUMAN-VERIFY: demo value) for every constant below: look-and-feel choices,
// not physics constants.
constexpr double kMaxHorizontalSpeedMps = 3.0;
constexpr double kClimbSpeedMps = 1.5;
constexpr double kYawRateRadS = 1.0;
constexpr double kWindSpeedStepMps = 1.0;
constexpr double kWindDirectionStepDeg = 15.0;
constexpr double kWindArrowCmPerMps = 30.0;
constexpr double kGroundClearanceCm = 8.0;     // vehicle origin height above the ground when landed
constexpr double kMeshYawOffsetDeg = 0.0;      // TODO(HUMAN-VERIFY): import orientation of the model
constexpr float kCameraArmLengthCm = 450.0f;
constexpr float kCameraPitchDeg = -18.0f;
constexpr double kDiscThicknessCm = 0.5;
constexpr int32 kMaxSubsteps = 20;
constexpr double kFixedDt = 0.002;  // ADR-001: fixed step 0.002 s

// Runtime spin selection, read every frame so a change in the console shows at once. They exist to
// find the right axis/sign in one Play session; once found, copy the values into kSpinAxisBoneLocal
// / kSpinDirectionScale in DroneVisualMath.h. TODO(HUMAN-VERIFY): builds on the project's Unreal version.
TAutoConsoleVariable<int32> CVarSpinAxis(TEXT("drone.SpinAxis"), DroneVisual::kSpinAxisIndexDefault,
	TEXT("Prop spin axis in the bone's local frame: 0=X, 1=Y, 2=Z."), ECVF_Default);
TAutoConsoleVariable<float> CVarSpinSign(TEXT("drone.SpinSign"), static_cast<float>(DroneVisual::kSpinDirectionScale),
	TEXT("Prop spin direction scale, +1 or -1 (sign of the value is used)."), ECVF_Default);

// Current console choice, validated. A bad drone.SpinAxis logs a Warning and uses the default.
void ReadSpinSelection(FVector& OutAxisLocal, double& OutSign)
{
	double Axis[3];
	const int32 Index = CVarSpinAxis.GetValueOnGameThread();
	if (!DroneVisual::SpinAxisFromIndex(Index, Axis))
	{
		UE_LOG(LogTemp, Warning, TEXT("DroneDemo: drone.SpinAxis=%d is not 0, 1 or 2; using the default %d."), Index, DroneVisual::kSpinAxisIndexDefault);
		DroneVisual::SpinAxisFromIndex(DroneVisual::kSpinAxisIndexDefault, Axis);
	}
	OutAxisLocal = FVector(Axis[0], Axis[1], Axis[2]);
	OutSign = DroneVisual::NormalizeSpinSign(CVarSpinSign.GetValueOnGameThread());
}

// Component-space reference-pose rotation of a bone, from the mesh's ref skeleton (not from the
// poseable component's current pose, which can be empty or already spun at BeginPlay). The ref
// skeleton stores each bone relative to its parent, so walk up to the root: CS = P_root*...*P_parent*L.
// Returns false if the bone is not in the skeleton. The walk is bounded by the bone count.
bool RefPoseComponentRotation(const FReferenceSkeleton& RefSkel, const FName& Bone, FQuat& OutRot)
{
	int32 Index = RefSkel.FindBoneIndex(Bone);
	if (Index == INDEX_NONE) return false;
	const TArray<FTransform>& Local = RefSkel.GetRefBonePose();
	FQuat Q = FQuat::Identity;
	for (int32 Hops = 0; Index != INDEX_NONE && Hops <= RefSkel.GetNum(); ++Hops)
	{
		if (!Local.IsValidIndex(Index)) return false;
		Q = Local[Index].GetRotation() * Q;  // parent * child: the child's rotation acts first
		Index = RefSkel.GetParentIndex(Index);
	}
	if (Index != INDEX_NONE) return false;  // loop bound hit: a cyclic skeleton, which is corrupt
	OutRot = Q.GetNormalized();
	return true;
}
}  // namespace

ADroneDemoPawn::ADroneDemoPawn()
{
	PrimaryActorTick.bCanEverTick = true;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	PoseMesh = CreateDefaultSubobject<UPoseableMeshComponent>(TEXT("DroneMesh"));
	PoseMesh->SetupAttachment(Root);
	PoseMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	CameraArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraArm"));
	CameraArm->SetupAttachment(Root);
	CameraArm->TargetArmLength = kCameraArmLengthCm;
	CameraArm->bDoCollisionTest = false;
	CameraArm->bEnableCameraLag = true;          // smooth follow
	CameraArm->bEnableCameraRotationLag = true;
	CameraArm->CameraLagSpeed = 5.0f;
	CameraArm->CameraRotationLagSpeed = 4.0f;
	CameraArm->SetUsingAbsoluteRotation(true);   // the camera must not roll and pitch with the drone

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(CameraArm, USpringArmComponent::SocketName);

	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

// ---------------------------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------------------------

bool ADroneDemoPawn::LoadPhysics()
{
	PhysicsState = FDronePhysicsBridgeModule::CreateDefaultState();
	Thermal = FDronePhysicsBridgeModule::CreateDefaultThermalState();
	PhysicsFaultLatch.physics_fault = 0;

	// ProjectDir is UE5Project/; the config lives one level up, in the repo root.
	FString RepoRoot = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT(".."));
	FPaths::NormalizeDirectoryName(RepoRoot);
	const FString ConfigPath = RepoRoot / TEXT("offline_pipeline/geometry/vehicle_config.json");
	if (!FDronePhysicsBridgeModule::LoadVehicleConfig(ConfigPath, &Config))
	{
		UE_LOG(LogTemp, Error, TEXT("DroneDemo: failed to load VehicleConfig from '%s'. Physics is disabled."), *ConfigPath);
		return false;
	}
	return true;
}

bool ADroneDemoPawn::LoadModelMap(double& OutModelLeftRightM, double& OutModelFrontRearM)
{
	const FString MapPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectPluginsDir() / TEXT("DronePhysicsBridge/Content/DroneModelMap.json"));
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *MapPath))
	{
		UE_LOG(LogTemp, Error, TEXT("DroneDemo: cannot read '%s'. Using simple shapes."), *MapPath);
		return false;
	}
	TSharedPtr<FJsonObject> Json;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Text);
	if (!FJsonSerializer::Deserialize(Reader, Json) || !Json.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("DroneDemo: '%s' is not valid JSON. Using simple shapes."), *MapPath);
		return false;
	}

	const TSharedPtr<FJsonObject>* ModelObj = nullptr;
	const TSharedPtr<FJsonObject>* SpanObj = nullptr;
	const TSharedPtr<FJsonObject>* MotorsObj = nullptr;
	const TArray<TSharedPtr<FJsonValue>>* OrderArr = nullptr;
	if (!Json->TryGetObjectField(TEXT("model"), ModelObj) || !(*ModelObj)->TryGetObjectField(TEXT("span_between_motor_axes_m"), SpanObj) ||
		!Json->TryGetObjectField(TEXT("motors"), MotorsObj) || !Json->TryGetArrayField(TEXT("motor_order"), OrderArr) || OrderArr->Num() != 4 ||
		!(*SpanObj)->TryGetNumberField(TEXT("left_right"), OutModelLeftRightM) || !(*SpanObj)->TryGetNumberField(TEXT("front_rear"), OutModelFrontRearM))
	{
		UE_LOG(LogTemp, Error, TEXT("DroneDemo: DroneModelMap.json is missing model spans, motors or a 4-entry motor_order. Using simple shapes."));
		return false;
	}

	Motors.Reset();
	for (const TSharedPtr<FJsonValue>& Name : *OrderArr)
	{
		const TSharedPtr<FJsonObject>* MotorObj = nullptr;
		const TArray<TSharedPtr<FJsonValue>>* BonesArr = nullptr;
		FString Hub;
		if (!(*MotorsObj)->TryGetObjectField(Name->AsString(), MotorObj) || !(*MotorObj)->TryGetArrayField(TEXT("spin_bones"), BonesArr) ||
			!(*MotorObj)->TryGetStringField(TEXT("hub_bone"), Hub))
		{
			UE_LOG(LogTemp, Error, TEXT("DroneDemo: DroneModelMap.json motor '%s' is incomplete. Using simple shapes."), *Name->AsString());
			return false;
		}
		FMotorVisual M;
		M.HubBone = FName(*Hub);
		for (const TSharedPtr<FJsonValue>& B : *BonesArr) M.SpinBones.Add(FName(*B->AsString()));
		Motors.Add(M);
	}
	return true;
}

void ADroneDemoPawn::SetupVisuals()
{
	// Config spans decide the scale (uniform, from the left-right span; never distorts the props).
	double CfgLR = 0.0, CfgFR = 0.0;
	const double Pos[4][3] = {{Config.motor_pos_m[0][0], Config.motor_pos_m[0][1], Config.motor_pos_m[0][2]},
							  {Config.motor_pos_m[1][0], Config.motor_pos_m[1][1], Config.motor_pos_m[1][2]},
							  {Config.motor_pos_m[2][0], Config.motor_pos_m[2][1], Config.motor_pos_m[2][2]},
							  {Config.motor_pos_m[3][0], Config.motor_pos_m[3][1], Config.motor_pos_m[3][2]}};
	DroneVisual::ConfigSpansM(Pos, &CfgLR, &CfgFR);

	double ModelLR = 0.0, ModelFR = 0.0;
	USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, kDroneMeshPath);
	bool bOk = Mesh != nullptr;
	if (!bOk)
	{
		UE_LOG(LogTemp, Error, TEXT("DroneDemo: skeletal mesh '%s' not found (import it, README step 3). Using simple shapes."), kDroneMeshPath);
	}
	bOk = bOk && LoadModelMap(ModelLR, ModelFR);

	double Scale = 1.0;
	bOk = bOk && DroneVisual::ComputeUniformScale(CfgLR, ModelLR, &Scale);
	if (bOk)
	{
		PoseMesh->SetSkinnedAssetAndUpdate(Mesh, true);
		for (const FMotorVisual& M : Motors)
		{
			for (const FName& Bone : M.SpinBones)
			{
				if (PoseMesh->GetBoneIndex(Bone) == INDEX_NONE)
				{
					UE_LOG(LogTemp, Error, TEXT("DroneDemo: bone '%s' is missing in '%s'. Using simple shapes."), *Bone.ToString(), kDroneMeshPath);
					bOk = false;
				}
			}
			if (PoseMesh->GetBoneIndex(M.HubBone) == INDEX_NONE)
			{
				UE_LOG(LogTemp, Error, TEXT("DroneDemo: hub bone '%s' is missing in '%s'. Using simple shapes."), *M.HubBone.ToString(), kDroneMeshPath);
				bOk = false;
			}
		}
	}

	if (!bOk)
	{
		bUseFallbackShapes = true;
		PoseMesh->SetVisibility(false);
		BuildFallbackShapes();
		BuildMotionDiscs();
		return;
	}

	PoseMesh->SetRelativeScale3D(FVector(Scale));
	PoseMesh->SetRelativeRotation(FRotator(0.0, kMeshYawOffsetDeg, 0.0));
	const double Leftover = DroneVisual::FrontRearMismatchM(Scale, ModelFR, CfgFR);
	UE_LOG(LogTemp, Log, TEXT("DroneDemo: uniform mesh scale %.5f (config left-right span %.4f m / model %.4f m). Front-rear leftover %+.4f m (scaled model %.4f m vs config %.4f m); props are not distorted."),
		Scale, CfgLR, ModelLR, Leftover, Scale * ModelFR, CfgFR);

	// Reference-pose rotations in component space, from the ref skeleton, stored once. UpdateVisuals
	// writes ref * spin, never the spin alone. An unreadable bone uses the simple shapes instead.
	const FReferenceSkeleton& RefSkel = Mesh->GetRefSkeleton();
	for (FMotorVisual& M : Motors)
	{
		M.RestComponentRot.Reset();
		for (const FName& Bone : M.SpinBones)
		{
			FQuat Ref;
			if (!RefPoseComponentRotation(RefSkel, Bone, Ref))
			{
				UE_LOG(LogTemp, Error, TEXT("DroneDemo: no reference pose for bone '%s'. Using simple shapes."), *Bone.ToString());
				bUseFallbackShapes = true;
				PoseMesh->SetVisibility(false);
				BuildFallbackShapes();
				BuildMotionDiscs();
				return;
			}
			M.RestComponentRot.Add(Ref);
			UE_LOG(LogTemp, Log, TEXT("DroneDemo: bone '%s' ref rotation (component space) quat xyzw = (%.5f, %.5f, %.5f, %.5f), euler pitch/yaw/roll = %s"),
				*Bone.ToString(), Ref.X, Ref.Y, Ref.Z, Ref.W, *Ref.Rotator().ToString());
		}
	}
	{
		FVector AxisLocal;
		double Sign;
		ReadSpinSelection(AxisLocal, Sign);
		UE_LOG(LogTemp, Log, TEXT("DroneDemo: drone.SpinAxis=%d (local axis %s), drone.SpinSign=%+.0f. Change at runtime with `drone.SpinAxis 0|1|2` and `drone.SpinSign 1|-1`."),
			CVarSpinAxis.GetValueOnGameThread(), *AxisLocal.ToString(), Sign);
	}
	BuildMotionDiscs();
}

void ADroneDemoPawn::BuildFallbackShapes()
{
	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCubeMeshPath);
	UStaticMesh* Cylinder = LoadObject<UStaticMesh>(nullptr, kCylinderMeshPath);
	if (!Cube || !Cylinder)
	{
		UE_LOG(LogTemp, Error, TEXT("DroneDemo: engine basic shapes not found; nothing will be drawn for the vehicle."));
		return;
	}
	auto Add = [this](UStaticMesh* Shape, const FVector& LocalCm, const FVector& ScaleXyz) {
		UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this);
		C->SetStaticMesh(Shape);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->SetupAttachment(Root);
		C->RegisterComponent();
		C->SetRelativeLocation(LocalCm);
		C->SetRelativeScale3D(ScaleXyz);
		FallbackShapes.Add(C);
	};
	Add(Cube, FVector::ZeroVector, FVector(0.30, 0.30, 0.08));  // 30 cm x 30 cm x 8 cm body
	for (int32 I = 0; I < 4; ++I)
	{
		// Body frame (x fwd, y right, z down) metres to actor-local cm: axis mapping by the bridge, then x100.
		const float Body[3] = {static_cast<float>(Config.motor_pos_m[I][0]), static_cast<float>(Config.motor_pos_m[I][1]), static_cast<float>(Config.motor_pos_m[I][2])};
		const FVector Local = FDronePhysicsBridgeModule::NedToUnrealVector(Body) * 100.0;
		Add(Cylinder, Local + FVector(0.0, 0.0, 6.0), FVector(0.05, 0.05, 0.004));  // thin disc stand-in for a prop
	}
}

void ADroneDemoPawn::BuildMotionDiscs()
{
	UStaticMesh* Cylinder = LoadObject<UStaticMesh>(nullptr, kCylinderMeshPath);
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, kDiscMaterialPath);
	if (!Cylinder) return;
	const double DiameterCm = Config.prop_diameter_m * 100.0;
	for (int32 I = 0; I < 4; ++I)
	{
		UStaticMeshComponent* Disc = NewObject<UStaticMeshComponent>(this);
		Disc->SetStaticMesh(Cylinder);
		Disc->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Disc->SetCastShadow(false);
		Disc->SetupAttachment(Root);
		Disc->RegisterComponent();
		// The engine cylinder is 100 cm across and 100 cm tall.
		Disc->SetRelativeScale3D(FVector(DiameterCm / 100.0, DiameterCm / 100.0, kDiscThicknessCm / 100.0));
		MotionDiscs.Add(Disc);
		if (Base)
		{
			UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, this);
			Mid->SetScalarParameterValue(TEXT("Opacity"), 0.0f);
			Disc->SetMaterial(0, Mid);
			DiscMaterials.Add(Mid);
		}
		else
		{
			Disc->SetVisibility(false);
		}
	}
	if (!Base)
	{
		UE_LOG(LogTemp, Warning, TEXT("DroneDemo: material '%s' not found; motion discs are hidden (README step 4)."), kDiscMaterialPath);
	}
}

void ADroneDemoPawn::SetupEnvironment()
{
	UWorld* World = GetWorld();
	if (!World) return;

	// Lumen: dynamic GI and reflections method 1 (also set in DefaultEngine.ini). Guarded: a
	// missing or read-only variable is skipped, never fatal.
	for (const TCHAR* Name : {TEXT("r.DynamicGlobalIlluminationMethod"), TEXT("r.ReflectionMethod")})
	{
		if (IConsoleVariable* Var = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			Var->Set(1, ECVF_SetByGameSetting);
		}
	}

	bool bHasSky = false, bHasSun = false, bHasSkyLight = false, bHasFog = false;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		bHasSky = bHasSky || It->FindComponentByClass<USkyAtmosphereComponent>() != nullptr;
		bHasSun = bHasSun || It->FindComponentByClass<UDirectionalLightComponent>() != nullptr;
		bHasSkyLight = bHasSkyLight || It->FindComponentByClass<USkyLightComponent>() != nullptr;
		bHasFog = bHasFog || It->FindComponentByClass<UExponentialHeightFogComponent>() != nullptr;
	}
	if (bHasSky && bHasSun && bHasSkyLight && bHasFog) return;  // the level already has its own

	AActor* Env = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity);
	if (!Env) return;
	USceneComponent* EnvRoot = NewObject<USceneComponent>(Env, TEXT("EnvRoot"));
	Env->SetRootComponent(EnvRoot);
	EnvRoot->RegisterComponent();

	if (!bHasSky)
	{
		USkyAtmosphereComponent* Sky = NewObject<USkyAtmosphereComponent>(Env);
		Sky->SetupAttachment(EnvRoot);
		Sky->RegisterComponent();
	}
	if (!bHasSun)
	{
		UDirectionalLightComponent* Sun = NewObject<UDirectionalLightComponent>(Env);
		Sun->SetupAttachment(EnvRoot);
		Sun->SetMobility(EComponentMobility::Movable);
		Sun->SetAtmosphereSunLight(true);
		Sun->SetIntensity(10.0f);  // lux, TODO(HUMAN-VERIFY: demo value)
		Sun->RegisterComponent();
		Sun->SetWorldRotation(FRotator(-45.0, 135.0, 0.0));
	}
	if (!bHasSkyLight)
	{
		USkyLightComponent* SkyLight = NewObject<USkyLightComponent>(Env);
		SkyLight->SetupAttachment(EnvRoot);
		SkyLight->SetMobility(EComponentMobility::Movable);
		SkyLight->bRealTimeCapture = true;
		SkyLight->RegisterComponent();
	}
	if (!bHasFog)
	{
		UExponentialHeightFogComponent* Fog = NewObject<UExponentialHeightFogComponent>(Env);
		Fog->SetupAttachment(EnvRoot);
		Fog->RegisterComponent();
	}
}

void ADroneDemoPawn::PlaceOnGround()
{
	UWorld* World = GetWorld();
	if (!World) return;
	const FVector Start = GetActorLocation();
	FHitResult Hit;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(DroneDemoGround), false, this);
	if (World->LineTraceSingleByChannel(Hit, Start + FVector(0, 0, 2000.0), Start - FVector(0, 0, 500000.0), ECC_Visibility, Params))
	{
		WorldOriginCm = FVector(Start.X, Start.Y, Hit.ImpactPoint.Z + kGroundClearanceCm);
		return;
	}

	// No ground under the start point (an empty level): lay a flat plane so there is something to land on.
	WorldOriginCm = FVector(Start.X, Start.Y, Start.Z);
	if (UStaticMesh* Plane = LoadObject<UStaticMesh>(nullptr, kPlaneMeshPath))
	{
		UStaticMeshComponent* Floor = NewObject<UStaticMeshComponent>(this);
		Floor->SetStaticMesh(Plane);
		Floor->SetupAttachment(Root);
		Floor->RegisterComponent();
		Floor->SetUsingAbsoluteLocation(true);
		Floor->SetUsingAbsoluteRotation(true);
		Floor->SetUsingAbsoluteScale(true);
		Floor->SetWorldLocation(WorldOriginCm - FVector(0.0, 0.0, kGroundClearanceCm));
		Floor->SetWorldRotation(FRotator::ZeroRotator);
		Floor->SetWorldScale3D(FVector(400.0, 400.0, 1.0));  // 400 x 100 cm = 400 m square
		FallbackShapes.Add(Floor);
	}
}

void ADroneDemoPawn::BeginPlay()
{
	Super::BeginPlay();

	bPhysicsReady = LoadPhysics();
	if (bPhysicsReady)
	{
		// No surrogate models are loaded on purpose: with no aero handle the Rust step uses the
		// analytic drag model, which is the path the wind acts on. (The stubs are not validated.)
		DroneDemo::Tuning Tune;
		Tune.HoverThrottle = Config.hover_throttle;
		Controller = MakeUnique<DroneDemo::Controller>(Tune);
	}
	WindGenerator = DroneWind::Generator(DroneWind::Generator::kDefaultSeed);

	PlaceOnGround();
	SetupEnvironment();
	SetupVisuals();
	UpdateVisuals(0.0f);
}

// ---------------------------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------------------------

void ADroneDemoPawn::ReadInput(APlayerController* PC, DroneDemo::Command& OutCommand)
{
	OutCommand = DroneDemo::Command();
	if (!PC || !Controller) return;

	if (PC->WasInputKeyJustPressed(EKeys::T) || PC->WasInputKeyJustPressed(EKeys::Gamepad_FaceButton_Bottom))
	{
		Controller->RequestTakeoff(-PhysicsState.position[2]);
	}
	if (PC->WasInputKeyJustPressed(EKeys::L) || PC->WasInputKeyJustPressed(EKeys::Gamepad_FaceButton_Right))
	{
		Controller->RequestLand();
	}

	double Forward = 0.0, Right = 0.0, Climb = 0.0, Yaw = 0.0;
	if (PC->IsInputKeyDown(EKeys::W)) Forward += 1.0;
	if (PC->IsInputKeyDown(EKeys::S)) Forward -= 1.0;
	if (PC->IsInputKeyDown(EKeys::D)) Right += 1.0;
	if (PC->IsInputKeyDown(EKeys::A)) Right -= 1.0;
	if (PC->IsInputKeyDown(EKeys::E)) Yaw += 1.0;
	if (PC->IsInputKeyDown(EKeys::Q)) Yaw -= 1.0;
	if (PC->IsInputKeyDown(EKeys::SpaceBar)) Climb += 1.0;
	if (PC->IsInputKeyDown(EKeys::LeftControl)) Climb -= 1.0;

	// Gamepad sticks add to the keys (each axis is limited to -1..1 below).
	Forward += PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftY);
	Right += PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftX);
	Yaw += PC->GetInputAnalogKeyState(EKeys::Gamepad_RightX);
	Climb += PC->GetInputAnalogKeyState(EKeys::Gamepad_RightY);

	OutCommand.ForwardMps = FMath::Clamp(Forward, -1.0, 1.0) * kMaxHorizontalSpeedMps;
	OutCommand.RightMps = FMath::Clamp(Right, -1.0, 1.0) * kMaxHorizontalSpeedMps;
	OutCommand.ClimbMps = FMath::Clamp(Climb, -1.0, 1.0) * kClimbSpeedMps;
	OutCommand.YawRateRadS = FMath::Clamp(Yaw, -1.0, 1.0) * kYawRateRadS;
}

void ADroneDemoPawn::ReadWindKeys(APlayerController* PC)
{
	if (!PC) return;
	if (PC->WasInputKeyJustPressed(EKeys::Up))
	{
		WindParams.SteadySpeedMps = DroneWind::ClampSteadySpeed(WindParams.SteadySpeedMps + kWindSpeedStepMps, WindParams.MaxTotalMps);
	}
	if (PC->WasInputKeyJustPressed(EKeys::Down))
	{
		WindParams.SteadySpeedMps = DroneWind::ClampSteadySpeed(WindParams.SteadySpeedMps - kWindSpeedStepMps, WindParams.MaxTotalMps);
	}
	if (PC->WasInputKeyJustPressed(EKeys::Right))
	{
		WindParams.SteadyDirectionDeg = DroneWind::WrapDirectionDeg(WindParams.SteadyDirectionDeg + kWindDirectionStepDeg);
	}
	if (PC->WasInputKeyJustPressed(EKeys::Left))
	{
		WindParams.SteadyDirectionDeg = DroneWind::WrapDirectionDeg(WindParams.SteadyDirectionDeg - kWindDirectionStepDeg);
	}
	if (PC->WasInputKeyJustPressed(EKeys::G))
	{
		WindParams.bGustsEnabled = !WindParams.bGustsEnabled;
	}
	if (PC->WasInputKeyJustPressed(EKeys::BackSpace))
	{
		// Zero wind: no steady wind and no gusts (the direction is kept).
		WindParams.SteadySpeedMps = 0.0;
		WindParams.bGustsEnabled = false;
	}
}

// ---------------------------------------------------------------------------------------------
// Fixed-step loop
// ---------------------------------------------------------------------------------------------

void ADroneDemoPawn::PinToGround()
{
	// DEMO ground handling: the Rust core has no ground contact (altitude < 0 is step code 6), so
	// the vehicle is pinned level and at rest on the ground, keeping its heading.
	const DroneState Before = PhysicsState;
	const double Q[4] = {Before.orientation[0], Before.orientation[1], Before.orientation[2], Before.orientation[3]};
	const DroneDemo::Euler Att = DroneDemo::QuatToEuler(Q);
	const double Half = 0.5 * Att.YawRad;
	PhysicsState.position[2] = 0.0;
	for (int32 I = 0; I < 3; ++I)
	{
		PhysicsState.velocity[I] = 0.0f;
		PhysicsState.angular_velocity[I] = 0.0f;
	}
	PhysicsState.orientation[0] = 0.0f;
	PhysicsState.orientation[1] = 0.0f;
	PhysicsState.orientation[2] = static_cast<float>(FMath::Sin(Half));
	PhysicsState.orientation[3] = static_cast<float>(FMath::Cos(Half));
	Controller->NotifyOnGround();
}

void ADroneDemoPawn::StepFixed(double FixedDt, const DroneDemo::Command& Command)
{
	// Wind first (its own seeded stream), then the controller, then the physics step.
	DroneWind::Sample Sample;
	if (WindGenerator.Step(WindParams, FixedDt, &Sample)) WindSample = Sample;
	const WindInput Wind = {{WindSample.Total[0], WindSample.Total[1], WindSample.Total[2]}};

	DroneDemo::Estimate Est;
	Est.AltitudeM = -PhysicsState.position[2];
	for (int32 I = 0; I < 3; ++I)
	{
		Est.VelNedMps[I] = PhysicsState.velocity[I];
		Est.BodyRatesRadS[I] = PhysicsState.angular_velocity[I];
	}
	for (int32 I = 0; I < 4; ++I) Est.QuatXyzw[I] = PhysicsState.orientation[I];

	DroneDemo::Output Out;
	if (!Controller->Update(Est, Command, FixedDt, &Out)) return;
	if (Controller->GetMode() == DroneDemo::Mode::OnGround) return;  // pinned: no physics step

	ControlInputs Inputs;
	Inputs.throttle = static_cast<float>(Out.Throttle);
	Inputs.roll = static_cast<float>(Out.Roll);
	Inputs.pitch = static_cast<float>(Out.Pitch);
	Inputs.yaw = static_cast<float>(Out.Yaw);

	DebugTelemetry StepTelemetry = {};
	if (!FDronePhysicsBridgeModule::StepPhysics(&PhysicsState, &Thermal, &Inputs, &Wind, nullptr, nullptr, &Config, &StepTelemetry, &PhysicsFaultLatch, FixedDt))
	{
		bPhysicsFault = true;
		return;
	}
	bPhysicsFault = false;
	LastTelemetry = StepTelemetry;
	bHaveTelemetry = true;

	if (DroneDemo::ShouldTouchDown(Controller->GetMode(), -PhysicsState.position[2]))
	{
		PinToGround();
	}
}

void ADroneDemoPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (!bPhysicsReady || !Controller)
	{
		DrawHud();  // shows why physics is off
		return;
	}

	APlayerController* PC = Cast<APlayerController>(GetController());
	DroneDemo::Command Command;
	ReadInput(PC, Command);
	ReadWindKeys(PC);

	Accumulator += static_cast<double>(DeltaTime);
	int32 Steps = 0;
	while (Accumulator >= kFixedDt && Steps < kMaxSubsteps)
	{
		StepFixed(kFixedDt, Command);
		if (bPhysicsFault) break;
		Accumulator -= kFixedDt;
		++Steps;
	}
	if (Steps == kMaxSubsteps && Accumulator >= kFixedDt)
	{
		++DroppedTimeEvents;
		UE_LOG(LogTemp, Warning, TEXT("DroneDemo: physics fell behind: dropped %.6f s (cap %d steps per frame), event %d"), Accumulator, kMaxSubsteps, DroppedTimeEvents);
		Accumulator = 0.0;
	}
	if (bPhysicsFault) Accumulator = 0.0;

	// Pose from the Rust state, through the one audited conversion in the bridge.
	const FVector Ned = FDronePhysicsBridgeModule::NedToUnrealWorld(PhysicsState.position);
	const FQuat Rot = FDronePhysicsBridgeModule::NedToUnrealQuat(PhysicsState.orientation);
	SetActorLocationAndRotation(WorldOriginCm + Ned, Rot);

	// Camera follows the heading only (no roll or pitch), with the arm's lag.
	CameraArm->SetWorldRotation(FRotator(kCameraPitchDeg, Rot.Rotator().Yaw, 0.0f));

	UpdateVisuals(DeltaTime);
	DrawWindArrows();
	DrawHud();
}

// ---------------------------------------------------------------------------------------------
// Visuals and HUD
// ---------------------------------------------------------------------------------------------

void ADroneDemoPawn::UpdateVisuals(float DeltaTime)
{
	FVector AxisLocal;
	double SpinDirection;
	ReadSpinSelection(AxisLocal, SpinDirection);
	const double AxisArr[3] = {AxisLocal.X, AxisLocal.Y, AxisLocal.Z};

	for (int32 I = 0; I < Motors.Num() && I < 4; ++I)
	{
		FMotorVisual& M = Motors[I];
		const double Rpm = bHaveTelemetry ? static_cast<double>(LastTelemetry.motor_rpms[I]) : 0.0;

		M.SpinAngleDeg = DroneVisual::AdvanceSpinDeg(M.SpinAngleDeg, Rpm, Config.motor_spin[I], DeltaTime, SpinDirection);
		if (!bUseFallbackShapes)
		{
			const double AngleRad = FMath::DegreesToRadians(M.SpinAngleDeg);
			for (int32 B = 0; B < M.SpinBones.Num() && B < M.RestComponentRot.Num(); ++B)
			{
				// final = ref * spin: the spin acts in the bone's LOCAL frame on top of the stored
				// reference rotation (DroneVisual::SpinOnRefRotation, tested without Unreal).
				// Absolute component-space set, so a bone that is a child of a spinning bone is not
				// rotated twice. TODO(HUMAN-VERIFY): that holds only if a parent spin bone is written
				// before its child; check the nut/cap/bolt bones do not drift.
				const FQuat& Ref = M.RestComponentRot[B];
				const DroneVisual::Quat Final = DroneVisual::SpinOnRefRotation(DroneVisual::Quat{Ref.X, Ref.Y, Ref.Z, Ref.W}, AxisArr, AngleRad);
				PoseMesh->SetBoneRotationByName(M.SpinBones[B], FQuat(Final.x, Final.y, Final.z, Final.w).Rotator(), EBoneSpaces::ComponentSpace);
			}
		}

		if (MotionDiscs.IsValidIndex(I))
		{
			FVector HubWorld;
			if (bUseFallbackShapes)
			{
				const float Body[3] = {static_cast<float>(Config.motor_pos_m[I][0]), static_cast<float>(Config.motor_pos_m[I][1]), static_cast<float>(Config.motor_pos_m[I][2])};
				HubWorld = GetActorTransform().TransformPosition(FDronePhysicsBridgeModule::NedToUnrealVector(Body) * 100.0 + FVector(0.0, 0.0, 6.0));
			}
			else
			{
				HubWorld = PoseMesh->GetBoneLocationByName(M.HubBone, EBoneSpaces::WorldSpace);
			}
			MotionDiscs[I]->SetWorldLocation(HubWorld);
			if (DiscMaterials.IsValidIndex(I))
			{
				DiscMaterials[I]->SetScalarParameterValue(TEXT("Opacity"), static_cast<float>(DroneVisual::MotionDiscOpacity(Rpm, Config.max_rpm)));
			}
		}
	}
}

void ADroneDemoPawn::DrawWindArrows() const
{
	UWorld* World = GetWorld();
	if (!World) return;
	const FVector Base = GetActorLocation() + FVector(0.0, 0.0, 120.0);

	// One arrow from a NED wind vector (m/s); the frame conversion is the bridge's.
	auto Arrow = [&](const double Ned[3], const FVector& From, const FColor& Color, float Thickness) {
		const float F[3] = {static_cast<float>(Ned[0]), static_cast<float>(Ned[1]), static_cast<float>(Ned[2])};
		const FVector Dir = FDronePhysicsBridgeModule::NedToUnrealVector(F) * kWindArrowCmPerMps;
		if (Dir.SizeSquared() < 1.0) return;
		const FVector Tip = From + Dir;
		DrawDebugLine(World, From, Tip, Color, false, -1.0f, 0, Thickness);
		const FVector Unit = Dir.GetSafeNormal();
		const FVector Side = FVector::CrossProduct(Unit, FVector::UpVector).GetSafeNormal();
		const float Head = FMath::Min(25.0f, static_cast<float>(Dir.Size()) * 0.3f);
		DrawDebugLine(World, Tip, Tip - Unit * Head + Side * Head * 0.5f, Color, false, -1.0f, 0, Thickness);
		DrawDebugLine(World, Tip, Tip - Unit * Head - Side * Head * 0.5f, Color, false, -1.0f, 0, Thickness);
	};
	Arrow(WindSample.Total, Base, FColor::Cyan, 3.0f);                                  // total wind
	Arrow(WindSample.Gust, Base + FVector(0.0, 0.0, 30.0), FColor::Orange, 2.0f);       // the gust part
}

void ADroneDemoPawn::DrawHud() const
{
	if (!GEngine) return;
	auto Line = [](int32 Key, const FColor& Color, const FString& Text) { GEngine->AddOnScreenDebugMessage(Key, 0.0f, Color, Text); };

	// Airspeed = speed relative to the air = |v_ground - wind| (NED), the quantity the drag uses.
	const double Rel[3] = {PhysicsState.velocity[0] - WindSample.Total[0], PhysicsState.velocity[1] - WindSample.Total[1], PhysicsState.velocity[2] - WindSample.Total[2]};
	const double Airspeed = FMath::Sqrt(Rel[0] * Rel[0] + Rel[1] * Rel[1] + Rel[2] * Rel[2]);
	const double WindMag = FMath::Sqrt(WindSample.Total[0] * WindSample.Total[0] + WindSample.Total[1] * WindSample.Total[1] + WindSample.Total[2] * WindSample.Total[2]);

	const TCHAR* ModeText = TEXT("?");
	if (Controller)
	{
		switch (Controller->GetMode())
		{
		case DroneDemo::Mode::OnGround: ModeText = TEXT("ON GROUND (press T)"); break;
		case DroneDemo::Mode::Takeoff: ModeText = TEXT("TAKEOFF"); break;
		case DroneDemo::Mode::Flying: ModeText = TEXT("FLYING"); break;
		case DroneDemo::Mode::Landing: ModeText = TEXT("LANDING"); break;
		}
	}

	Line(101, FColor::White, TEXT("DEMO, not flight code"));
	Line(102, FColor::Cyan, FString::Printf(TEXT("Mode: %s"), ModeText));
	Line(103, FColor::Cyan, FString::Printf(TEXT("Airspeed: %.1f m/s"), Airspeed));
	Line(104, FColor::Cyan, FString::Printf(TEXT("Altitude: %.1f m"), -PhysicsState.position[2]));
	Line(105, FColor::Cyan, FString::Printf(TEXT("Wind: %.1f m/s (steady %.0f m/s toward %03.0f deg, gusts %s)"), WindMag, WindParams.SteadySpeedMps, WindParams.SteadyDirectionDeg,
		WindParams.bGustsEnabled ? TEXT("ON") : TEXT("off")));
	if (bHaveTelemetry)
	{
		Line(106, FColor::Cyan, FString::Printf(TEXT("RPM FR/FL/RR/RL: %.0f / %.0f / %.0f / %.0f"), static_cast<double>(LastTelemetry.motor_rpms[0]), static_cast<double>(LastTelemetry.motor_rpms[1]),
			static_cast<double>(LastTelemetry.motor_rpms[2]), static_cast<double>(LastTelemetry.motor_rpms[3])));
	}
	else
	{
		Line(106, FColor::Cyan, TEXT("RPM FR/FL/RR/RL: 0 / 0 / 0 / 0"));
	}
	Line(107, FColor::Silver, TEXT("T takeoff  L land  WASD move  Q/E yaw  Space/Ctrl climb  Arrows wind  G gusts  Backspace zero wind"));
	Line(108, FColor::Silver, kModelCredit);
	if (bPhysicsFault || PhysicsFaultLatch.physics_fault != 0)
	{
		Line(109, FColor::Red, TEXT("PHYSICS FAULT: a step was rejected (check the log); data is invalid"));
	}
	if (!bPhysicsReady)
	{
		Line(110, FColor::Red, TEXT("Physics disabled: vehicle_config.json could not be loaded (see the log)"));
	}
}
