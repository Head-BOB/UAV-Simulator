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
#include "DroneDataPaths.h"
#include "DroneMissionRunner.h"
#include "DroneVisualMath.h"
#include "Engine/Engine.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/GameModeBase.h"
#include "Kismet/GameplayStatics.h"
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
constexpr double kCrashSinkMps = 2.5;   // touching the ground faster than this is a crash. TODO(HUMAN-VERIFY: demo value)

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

// Number of parents above a bone (root = 0). Bounded by the bone count, so a corrupt cyclic skeleton ends.
int32 RefBoneDepth(const FReferenceSkeleton& RefSkel, const FName& Bone)
{
	int32 Index = RefSkel.FindBoneIndex(Bone);
	int32 Depth = 0;
	while (Index != INDEX_NONE && Depth <= RefSkel.GetNum())
	{
		Index = RefSkel.GetParentIndex(Index);
		if (Index != INDEX_NONE) ++Depth;
	}
	return Depth;
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

	// The demo flies the REFERENCE BUILD (APC 12x6E, MN3508; masses ASSUMED, see components/catalog.json), not the
	// shipped test config. Found without absolute paths: Content/DroneData (packaged) or the repo root (editor).
	const FString ConfigPath = DroneData::Resolve(TEXT("offline_pipeline/geometry/quad_ref_build_config.json"));
	if (ConfigPath.IsEmpty() || !FDronePhysicsBridgeModule::LoadVehicleConfig(ConfigPath, &Config))
	{
		UE_LOG(LogTemp, Error, TEXT("DroneDemo: failed to load the reference-build VehicleConfig ('%s'). Physics is disabled."), *ConfigPath);
		return false;
	}
	BaseConfig = Config;
	BaseMassKg = Config.mass_kg;
	return true;
}

bool ADroneDemoPawn::LoadModelMap(double& OutModelLeftRightM, double& OutModelFrontRearM)
{
	const FString MapPath = DroneData::Resolve(TEXT("UE5Project/Plugins/DronePhysicsBridge/Content/DroneModelMap.json"));
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
		// Parents before children, so an absolute ComponentSpace write of a child is not undone by a
		// later write of its parent. Stable sort keeps the JSON order among bones of equal depth.
		M.SpinBones.StableSort([&RefSkel](const FName& A, const FName& B) { return RefBoneDepth(RefSkel, A) < RefBoneDepth(RefSkel, B); });
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
			// One line per spun bone: parent, ref pose (quat + euler) in component space, the bone's own
			// X/Y/Z axes in component space (which local axis is the shaft), ref location, and the
			// axis/sign applied now. Location is the bone's offset from its parent (ref skeleton).
			FVector AxisNow;
			double SignNow;
			ReadSpinSelection(AxisNow, SignNow);
			const int32 BoneIdx = RefSkel.FindBoneIndex(Bone);
			const int32 ParentIdx = RefSkel.GetParentIndex(BoneIdx);
			const FString ParentName = ParentIdx == INDEX_NONE ? FString(TEXT("(root)")) : RefSkel.GetBoneName(ParentIdx).ToString();
			// Three short lines, each with its arguments next to its specifiers (all %s get an *FString, all numbers are double/int32).
			const FString LocationText = RefSkel.GetRefBonePose()[BoneIdx].GetLocation().ToString();
			const FString EulerText = Ref.Rotator().ToString();
			const FString LocalX = Ref.RotateVector(FVector::XAxisVector).ToString();
			const FString LocalY = Ref.RotateVector(FVector::YAxisVector).ToString();
			const FString LocalZ = Ref.RotateVector(FVector::ZAxisVector).ToString();
			const FString BoneText = Bone.ToString();
			const int32 Depth = RefBoneDepth(RefSkel, Bone);
			UE_LOG(LogTemp, Log, TEXT("DroneDemo-DIAG: bone '%s' parent '%s' depth %d ref-local location %s"), *BoneText, *ParentName, Depth, *LocationText);
			UE_LOG(LogTemp, Log, TEXT("DroneDemo-DIAG: bone '%s' ref quat xyzw = (%.5f, %.5f, %.5f, %.5f) euler (pitch,yaw,roll) = %s"), *BoneText,
				static_cast<double>(Ref.X), static_cast<double>(Ref.Y), static_cast<double>(Ref.Z), static_cast<double>(Ref.W), *EulerText);
			UE_LOG(LogTemp, Log, TEXT("DroneDemo-DIAG: bone '%s' local X in component = %s, local Y = %s, local Z = %s"), *BoneText, *LocalX, *LocalY, *LocalZ);
			UE_LOG(LogTemp, Log, TEXT("DroneDemo-DIAG: bone '%s' spin axis %s sign %+.0f"), *BoneText, *AxisNow.ToString(), SignNow);
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

	SelectStartEnvironment();
	bPhysicsReady = LoadPhysics();
	if (bPhysicsReady)
	{
		// No surrogate models are loaded on purpose: with no aero handle the Rust step uses the
		// analytic drag model, which is the path the wind acts on. (The stubs are not validated.)
		DroneDemo::Tuning Tune;
		// config.hover_throttle is derived at sea-level ISA density; thin or hot air needs more throttle
		// (DroneEnv::HoverThrottleScale). Above 1 means this site cannot hover: say so, do not hide it.
		double HoverScale = 1.0;
		DroneEnv::HoverThrottleScale(AirDensityKgM3, &HoverScale);
		Tune.HoverThrottle = Config.hover_throttle * HoverScale;
		if (Tune.HoverThrottle > 1.0)
		{
			UE_LOG(LogTemp, Warning, TEXT("DroneDemo: hover needs throttle %.3f > 1 at this site (density %.4f kg/m3): the vehicle cannot hover here."), Tune.HoverThrottle, AirDensityKgM3);
			EnvironmentNotice = FString::Printf(TEXT("Cannot hover here: needs %.0f%% throttle"), Tune.HoverThrottle * 100.0);
			Tune.HoverThrottle = 1.0;
		}
		LastTune = Tune;
		DemoController = MakeUnique<DroneDemo::Controller>(Tune);
	}
	WindGenerator = DroneWind::Generator(DroneWind::Generator::kDefaultSeed);

	PlaceOnGround();
	SetupEnvironment();
	ApplyEnvironmentLook(Catalog.Find(EnvironmentName));
	SetupVisuals();
	UpdateVisuals(0.0f);
}

// ---------------------------------------------------------------------------------------------
// Environment
// ---------------------------------------------------------------------------------------------

void ADroneDemoPawn::SelectStartEnvironment()
{
	FString Error;
	if (!Catalog.Load(Error))
	{
		EnvironmentNotice = FString::Printf(TEXT("%s: standard day, default wind"), *Error);
		UE_LOG(LogTemp, Warning, TEXT("DroneDemo: %s"), *EnvironmentNotice);
		return;
	}
	// The start menu opens the level with "?Env=<id>"; no option means the table's default.
	FString Wanted;
	if (const UWorld* World = GetWorld())
	{
		if (const AGameModeBase* Mode = World->GetAuthGameMode()) Wanted = UGameplayStatics::ParseOption(Mode->OptionsString, TEXT("Env"));
	}
	const FDroneEnvironmentEntry* Entry = Wanted.IsEmpty() ? Catalog.Default() : Catalog.Find(Wanted);
	if (!Entry)
	{
		EnvironmentNotice = FString::Printf(TEXT("Unknown environment '%s': using the default"), *Wanted);
		Entry = Catalog.Default();
	}
	FString Message;
	if (Entry && !ApplyEnvironment(*Entry, Message))
	{
		EnvironmentNotice = Message;
		UE_LOG(LogTemp, Warning, TEXT("DroneDemo: environment rejected: %s"), *Message);
	}
}

bool ADroneDemoPawn::ApplyEnvironment(const FDroneEnvironmentEntry& Entry, FString& OutMessage)
{
	const DroneEnv::Preset& P = Entry.Preset;
	if (!Entry.ValidationMessage.IsEmpty())
	{
		OutMessage = FString::Printf(TEXT("%s: %s"), UTF8_TO_TCHAR(P.Name.c_str()), *Entry.ValidationMessage);
		return false;
	}
	double Direction = 0.0;
	DroneEnv::WrapDirectionDeg(P.WindTowardDeg, &Direction);
	if (!SetWeather(P.WindSpeedMps, Direction, P.GustSigmaMps, P.SiteAltitudeM, P.AmbientTempC, P.HumidityPct, OutMessage)) return false;
	EnvironmentName = UTF8_TO_TCHAR(P.Id.c_str());
	ApplyEnvironmentLook(&Entry);  // harmless before BeginPlay finished: the level lights do not exist yet
	return true;
}

bool ADroneDemoPawn::SetWeather(double WindSpeedMps, double WindTowardDeg, double GustSigmaMps, double SiteAltM, double AmbientC,
								double HumidityPercent, FString& OutMessage)
{
	double Env[2] = {0.0, 0.0};
	const DroneEnv::Status Status = DroneEnv::ValidateWeather(WindSpeedMps, GustSigmaMps, SiteAltM, AmbientC, HumidityPercent, Env);
	double Direction = 0.0;
	if (Status == DroneEnv::Status::Ok && !DroneEnv::WrapDirectionDeg(WindTowardDeg, &Direction))
	{
		OutMessage = TEXT("wind direction is not a number");
		return false;
	}
	if (Status != DroneEnv::Status::Ok)
	{
		OutMessage = UTF8_TO_TCHAR(DroneEnv::CheckMessage(Status));
		return false;
	}
	double Rho = 0.0;
	if (!DroneEnv::DensityKgM3(SiteAltM, AmbientC, &Rho))
	{
		OutMessage = TEXT("air density is outside the model range");
		return false;
	}
	// Everything checked: now change state, all at once.
	PhysicsEnvironment.site_altitude_m = Env[0];
	PhysicsEnvironment.temp_offset_c = Env[1];
	WindParams.SteadySpeedMps = WindSpeedMps;
	WindParams.SteadyDirectionDeg = Direction;
	WindParams.GustSigmaMps = GustSigmaMps;
	WindParams.bGustsEnabled = GustSigmaMps > 0.0;
	SiteAltitudeM = SiteAltM;
	AmbientTempC = AmbientC;
	HumidityPct = HumidityPercent;
	AirDensityKgM3 = Rho;
	UE_LOG(LogTemp, Log, TEXT("DroneDemo: weather set: site %.0f m, ambient %.1f C (offset %+.2f K from the standard day), density %.4f kg/m3, wind %.1f m/s toward %03.0f deg, gust sigma %.1f m/s, humidity %.0f%% (shown only, not modelled)."),
		SiteAltM, AmbientC, Env[1], Rho, WindSpeedMps, Direction, GustSigmaMps, HumidityPercent);
	OutMessage = TEXT("ok");
	return true;
}

void ADroneDemoPawn::ApplyEnvironmentLook(const FDroneEnvironmentEntry* Entry)
{
	// Secondary effect only. TODO(HUMAN-VERIFY: demo value, Unreal API): tint, fog and light scale per environment.
	UWorld* World = GetWorld();
	if (!World || !Entry) return;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (UExponentialHeightFogComponent* Fog = It->FindComponentByClass<UExponentialHeightFogComponent>())
		{
			Fog->SetFogDensity(static_cast<float>(Entry->FogDensity));
			Fog->SetFogInscatteringColor(Entry->SkyTint);
		}
		if (UDirectionalLightComponent* Sun = It->FindComponentByClass<UDirectionalLightComponent>())
		{
			Sun->SetIntensity(Sun->Intensity * static_cast<float>(Entry->LightScale));
		}
	}
}

// ---------------------------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------------------------

void ADroneDemoPawn::ReadInput(APlayerController* PC, DroneDemo::Command& OutCommand)
{
	OutCommand = DroneDemo::Command();
	if (!PC || !DemoController || !bPilotInputEnabled) return;  // the control panel pauses pilot input

	if (bFlyMode)
	{
		// Fly mode: the engine-free mapping (DroneFly.h): W/S pitch, A/D roll, Q/E yaw, Space/R up, Shift/F down; T / G / X buttons;
		// gamepad left stick throttle + yaw, right stick pitch + roll, A / B / Y.
		DroneApp::Fly::KeyState K;
		K.W = PC->IsInputKeyDown(EKeys::W); K.S = PC->IsInputKeyDown(EKeys::S);
		K.A = PC->IsInputKeyDown(EKeys::A); K.D = PC->IsInputKeyDown(EKeys::D);
		K.Q = PC->IsInputKeyDown(EKeys::Q); K.E = PC->IsInputKeyDown(EKeys::E);
		K.Up = PC->IsInputKeyDown(EKeys::SpaceBar) || PC->IsInputKeyDown(EKeys::R);
		K.Down = PC->IsInputKeyDown(EKeys::LeftShift) || PC->IsInputKeyDown(EKeys::F);
		const DroneApp::Fly::Axes Pad = DroneApp::Fly::FromGamepad(PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftX), PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftY),
			PC->GetInputAnalogKeyState(EKeys::Gamepad_RightX), PC->GetInputAnalogKeyState(EKeys::Gamepad_RightY));
		OutCommand = DroneApp::Fly::ToCommand(DroneApp::Fly::Sum(DroneApp::Fly::FromKeys(K), Pad));
		if (PC->WasInputKeyJustPressed(EKeys::X) || PC->WasInputKeyJustPressed(EKeys::Gamepad_FaceButton_Top)) { ResetFlight(); return; }
		if (bCrashed) { OutCommand = DroneDemo::Command(); return; }
		if (PC->WasInputKeyJustPressed(EKeys::T) || PC->WasInputKeyJustPressed(EKeys::Gamepad_FaceButton_Bottom)) DemoController->RequestTakeoff(-PhysicsState.position[2]);
		if (PC->WasInputKeyJustPressed(EKeys::G) || PC->WasInputKeyJustPressed(EKeys::Gamepad_FaceButton_Right)) DemoController->RequestLand();
		return;
	}

	if (PC->WasInputKeyJustPressed(EKeys::T) || PC->WasInputKeyJustPressed(EKeys::Gamepad_FaceButton_Bottom))
	{
		DemoController->RequestTakeoff(-PhysicsState.position[2]);
	}
	if (PC->WasInputKeyJustPressed(EKeys::L) || PC->WasInputKeyJustPressed(EKeys::Gamepad_FaceButton_Right))
	{
		DemoController->RequestLand();
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
	if (bFlyMode && Before.velocity[2] > kCrashSinkMps) bCrashed = true;   // NED z is down: sinking faster than the limit on touchdown
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
	DemoController->NotifyOnGround();
	if (bFlewSinceLanding)
	{
		bLandedEvent = true;  // the HUD opens the flight report once
		bFlewSinceLanding = false;
	}
}

void ADroneDemoPawn::StepFixed(double FixedDt, const DroneDemo::Command& Command)
{
	if (bCrashed) return;   // a crashed vehicle stays where it is until reset (X)
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
	if (!DemoController->Update(Est, Command, FixedDt, &Out)) return;
	if (DemoController->GetMode() == DroneDemo::Mode::OnGround) return;  // pinned: no physics step

	ControlInputs Inputs;
	Inputs.throttle = static_cast<float>(Out.Throttle);
	Inputs.roll = static_cast<float>(Out.Roll);
	Inputs.pitch = static_cast<float>(Out.Pitch);
	Inputs.yaw = static_cast<float>(Out.Yaw);

	DebugTelemetry StepTelemetry = {};
	if (!FDronePhysicsBridgeModule::StepPhysics(&PhysicsState, &Thermal, &Inputs, &Wind, &PhysicsEnvironment, nullptr, nullptr, &Config, &StepTelemetry, &PhysicsFaultLatch, FixedDt))
	{
		bPhysicsFault = true;
		return;
	}
	bPhysicsFault = false;
	LastTelemetry = StepTelemetry;
	bHaveTelemetry = true;
	FlightTimeS += FixedDt;
	bFlewSinceLanding = true;

	// Electrical operating point per motor (same model as the report: DroneRun::ComputeMotorPoint).
	{
		double Total = 0.0;
		for (int32 M = 0; M < 4; ++M)
		{
			const DroneRun::MotorPoint Mp = DroneRun::ComputeMotorPoint(ReportBuild, static_cast<double>(StepTelemetry.motor_rpms[M]), AirDensityKgM3);
			LastMotorCurrentA[M] = Mp.CurrentA;
			Total += Mp.BatteryA;
		}
		LastBatteryA = Total;
		double Vt = ReportBuild.BatteryVoc;
		DroneElec::PackTerminalV(ReportBuild.BatteryVoc, Total, ReportBuild.BatteryResistanceOhm, &Vt);
		LastPackV = Vt;
		BatteryUsedAh += Total * FixedDt / 3600.0;
	}

	if (DroneDemo::ShouldTouchDown(DemoController->GetMode(), -PhysicsState.position[2]))
	{
		PinToGround();
	}
}

void ADroneDemoPawn::SetReplayMode(bool bEnable)
{
	bReplayMode = bEnable;
	SetActorHiddenInGame(bEnable);   // the demo model stays out of the way; the replayed vehicle is ADroneBuildPreview
	bSimulationActive = !bEnable;
	Accumulator = 0.0;
}

void ADroneDemoPawn::SetReplayView(const double PosNed[3], const float OrientNed[4], const double WindNed[3])
{
	const FVector Ned = FDronePhysicsBridgeModule::NedToUnrealWorld(PosNed);
	const FQuat Rot = FDronePhysicsBridgeModule::NedToUnrealQuat(OrientNed);
	SetActorLocationAndRotation(WorldOriginCm + Ned, Rot);
	if (CameraArm) CameraArm->SetWorldRotation(FRotator(kCameraPitchDeg, Rot.Rotator().Yaw, 0.0f));
	for (int32 I = 0; I < 3; ++I)
	{
		WindSample.Total[I] = WindNed[I];
		WindSample.Steady[I] = WindNed[I];
		WindSample.Gust[I] = 0.0;
	}
}

void ADroneDemoPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (bReplayMode)
	{
		DrawWindArrows();   // the replay layer calls SetReplayView() each data tick
		return;
	}
	if (!bPhysicsReady || !DemoController)
	{
		DrawHud();  // shows why physics is off
		return;
	}

	if (!bSimulationActive)
	{
		Accumulator = 0.0;  // start menu: nothing advances
		return;
	}
	APlayerController* PC = Cast<APlayerController>(GetController());
	DroneDemo::Command Command;
	ReadInput(PC, Command);
	if (bPilotInputEnabled && !bFlyMode) ReadWindKeys(PC);   // fly mode: the wind panel owns the wind

	Accumulator += static_cast<double>(DeltaTime);
	int32 Steps = 0;
	while (Accumulator >= kFixedDt && Steps < kMaxSubsteps)
	{
		StepFixed(kFixedDt, Command);
		if (bPhysicsFault) break;
		Accumulator -= kFixedDt;
		++Steps;
	}
	{
		const double G = FMath::Sqrt(WindSample.Gust[0] * WindSample.Gust[0] + WindSample.Gust[1] * WindSample.Gust[1] + WindSample.Gust[2] * WindSample.Gust[2]);
		GustPulseValue = DroneApp::Fly::GustPulse(PrevGustMps, G, static_cast<double>(DeltaTime));
		PrevGustMps = G;
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

		if (!bUseFallbackShapes)
		{
			// Above ~kBladeHideRpm the blades alias into a wobble: hide the hub (prop) bone's mesh and
			// let the motion disc carry the look. TODO(HUMAN-VERIFY: hides only that bone's weighted
			// vertices; if the nut/cap vanish with it, list those bones to keep.)
			const bool bHide = DroneVisual::BladesHidden(M.bBladesHidden, Rpm);
			if (bHide != M.bBladesHidden)
			{
				M.bBladesHidden = bHide;
				if (bHide) PoseMesh->HideBoneByName(M.HubBone, PBO_None);
				else PoseMesh->UnHideBoneByName(M.HubBone);
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
	// Flowing streaks drifting with the wind (DroneApp::Fly::WindStreaks): count and drift speed scale with the wind, a gust makes them
	// flash. The wind is the one the physics step received (WindSample.Total), not an estimate. No arrow.
	UWorld* World = GetWorld();
	if (!World) return;
	std::vector<DroneApp::Fly::Streak> Streaks;
	DroneApp::Fly::WindStreaks(WindSample.Total, static_cast<double>(World->GetTimeSeconds()), GustPulseValue, Streaks);
	const FVector Centre = GetActorLocation();
	for (const DroneApp::Fly::Streak& S : Streaks)
	{
		const float Pos[3] = {static_cast<float>(S.X), static_cast<float>(S.Y), static_cast<float>(S.Z)};
		const double Speed = FMath::Sqrt(WindSample.Total[0] * WindSample.Total[0] + WindSample.Total[1] * WindSample.Total[1] + WindSample.Total[2] * WindSample.Total[2]);
		const float Dir[3] = {static_cast<float>(WindSample.Total[0] / Speed), static_cast<float>(WindSample.Total[1] / Speed), static_cast<float>(WindSample.Total[2] / Speed)};
		const FVector From = Centre + FDronePhysicsBridgeModule::NedToUnrealVector(Pos) * 100.0;
		const FVector To = From + FDronePhysicsBridgeModule::NedToUnrealVector(Dir) * (S.LenM * 100.0);
		const uint8 Grey = static_cast<uint8>(120.0 + 135.0 * S.Alpha);   // white; brighter on a gust pulse
		DrawDebugLine(World, From, To, FColor(Grey, Grey, Grey), false, -1.0f, 0, 1.5f);
	}
}

void ADroneDemoPawn::DrawHud() const
{
	// The Slate HUD (ADroneDemoHUD) replaces this debug text; `drone.DebugHud 1` brings it back.
	static TAutoConsoleVariable<int32> CVarDebugHud(TEXT("drone.DebugHud"), 0, TEXT("1 shows the old on-screen debug text next to the Slate HUD."), ECVF_Default);
	if (CVarDebugHud.GetValueOnGameThread() == 0 && bPhysicsReady) return;
	if (!GEngine) return;
	auto Line = [](int32 Key, const FColor& Color, const FString& Text) { GEngine->AddOnScreenDebugMessage(Key, 0.0f, Color, Text); };

	// Airspeed = speed relative to the air = |v_ground - wind| (NED), the quantity the drag uses.
	const double Rel[3] = {PhysicsState.velocity[0] - WindSample.Total[0], PhysicsState.velocity[1] - WindSample.Total[1], PhysicsState.velocity[2] - WindSample.Total[2]};
	const double Airspeed = FMath::Sqrt(Rel[0] * Rel[0] + Rel[1] * Rel[1] + Rel[2] * Rel[2]);
	const double WindMag = FMath::Sqrt(WindSample.Total[0] * WindSample.Total[0] + WindSample.Total[1] * WindSample.Total[1] + WindSample.Total[2] * WindSample.Total[2]);

	const TCHAR* ModeText = TEXT("?");
	if (DemoController)
	{
		switch (DemoController->GetMode())
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
		Line(110, FColor::Red, TEXT("Physics disabled: quad_ref_build_config.json could not be loaded (see the log)"));
	}
}

// ---------------------------------------------------------------------------------------------
// UI hooks
// ---------------------------------------------------------------------------------------------

void ADroneDemoPawn::RequestTakeoffFromUi()
{
	if (DemoController) DemoController->RequestTakeoff(-PhysicsState.position[2]);
}

void ADroneDemoPawn::RequestLandFromUi()
{
	if (DemoController) DemoController->RequestLand();
}

bool ADroneDemoPawn::ConsumeLandedEvent()
{
	const bool bEvent = bLandedEvent;
	bLandedEvent = false;
	return bEvent;
}

FString ADroneDemoPawn::GetModeString() const
{
	if (!bPhysicsReady || !DemoController) return TEXT("PHYSICS OFF");
	if (bPhysicsFault) return TEXT("PHYSICS FAULT");
	switch (DemoController->GetMode())
	{
		case DroneDemo::Mode::OnGround: return TEXT("ON GROUND");
		case DroneDemo::Mode::Takeoff: return TEXT("TAKEOFF");
		case DroneDemo::Mode::Flying: return TEXT("FLYING");
		case DroneDemo::Mode::Landing: return TEXT("LANDING");
	}
	return TEXT("?");
}

DroneReport::EnvInfo ADroneDemoPawn::MakeEnvInfo() const
{
	DroneReport::EnvInfo E;
	E.Id = TCHAR_TO_UTF8(*EnvironmentName);
	E.Name = TCHAR_TO_UTF8(*EnvironmentName);
	E.SiteAltitudeM = SiteAltitudeM;
	E.AmbientTempC = AmbientTempC;
	E.HumidityPct = HumidityPct;
	E.WindSpeedMps = WindParams.SteadySpeedMps;
	E.WindTowardDeg = WindParams.SteadyDirectionDeg;
	E.GustSigmaMps = WindParams.bGustsEnabled ? WindParams.GustSigmaMps : 0.0;
	E.TempOffsetC = PhysicsEnvironment.temp_offset_c;
	return E;
}

void ADroneDemoPawn::FillUiRaw(FDroneUiRaw& Out) const
{
	const double Q[4] = {PhysicsState.orientation[0], PhysicsState.orientation[1], PhysicsState.orientation[2], PhysicsState.orientation[3]};
	const DroneDemo::Euler Att = DroneDemo::QuatToEuler(Q);
	Out.AltitudeM = -PhysicsState.position[2];
	Out.SpeedMps = FMath::Sqrt(static_cast<double>(PhysicsState.velocity[0]) * PhysicsState.velocity[0] + static_cast<double>(PhysicsState.velocity[1]) * PhysicsState.velocity[1]);
	const double Rel[3] = {PhysicsState.velocity[0] - WindSample.Total[0], PhysicsState.velocity[1] - WindSample.Total[1], PhysicsState.velocity[2] - WindSample.Total[2]};
	Out.AirspeedMps = FMath::Sqrt(Rel[0] * Rel[0] + Rel[1] * Rel[1] + Rel[2] * Rel[2]);
	Out.ClimbMps = -static_cast<double>(PhysicsState.velocity[2]);
	Out.HeadingDeg = DroneUi::WrapDeg360(Att.YawRad * 180.0 / DroneUi::kPi);
	Out.RollRad = Att.RollRad;
	Out.PitchRad = Att.PitchRad;
	Out.WindMps = FMath::Sqrt(WindSample.Total[0] * WindSample.Total[0] + WindSample.Total[1] * WindSample.Total[1] + WindSample.Total[2] * WindSample.Total[2]);
	Out.WindTowardDeg = Out.WindMps > 0.05 ? DroneUi::WrapDeg360(FMath::Atan2(WindSample.Total[1], WindSample.Total[0]) * 180.0 / DroneUi::kPi) : WindParams.SteadyDirectionDeg;
	Out.GustMps = FMath::Sqrt(WindSample.Gust[0] * WindSample.Gust[0] + WindSample.Gust[1] * WindSample.Gust[1] + WindSample.Gust[2] * WindSample.Gust[2]);
	const double MaxRpm = Config.max_rpm > 0.0 ? Config.max_rpm : 1.0;
	const double Nmax = MaxRpm / 60.0;
	const double TMax = Config.thrust_coeff * AirDensityKgM3 * Nmax * Nmax * FMath::Pow(Config.prop_diameter_m, 4.0);
	double Mean = 0.0, MaxUtil = 0.0;
	for (int32 M = 0; M < 4; ++M)
	{
		const double Rpm = bHaveTelemetry ? static_cast<double>(LastTelemetry.motor_rpms[M]) : 0.0;
		const double Th = bHaveTelemetry ? static_cast<double>(LastTelemetry.motor_thrusts[M]) : 0.0;
		Out.Rpm[M] = Rpm;
		Out.RpmFraction[M] = Rpm / MaxRpm;
		Out.ThrustN[M] = Th;
		Out.MotorTempC[M] = bHaveTelemetry ? static_cast<double>(LastTelemetry.motor_temperatures_c[M]) : 0.0;
		Mean += Rpm / MaxRpm / 4.0;
		if (TMax > 0.0) MaxUtil = FMath::Max(MaxUtil, Th / TMax);
	}
	Out.ThrottleMean = Mean;
	Out.MaxThrustMarginUtil = MaxUtil;
	Out.MotorCurrentMaxA = FMath::Max(FMath::Max(LastMotorCurrentA[0], LastMotorCurrentA[1]), FMath::Max(LastMotorCurrentA[2], LastMotorCurrentA[3]));
	Out.BatteryA = LastBatteryA;
	Out.PackV = LastPackV > 0.0 ? LastPackV : ReportBuild.BatteryVoc;
	const double Usable = ReportBuild.BatteryCapacityAh * ReportBuild.BatteryUsable;
	Out.BatteryUsedFraction = Usable > 0.0 ? BatteryUsedAh / Usable : 0.0;
	Out.AirDensityKgM3 = AirDensityKgM3;
	Out.SiteAltitudeM = SiteAltitudeM;
	Out.AmbientTempC = AmbientTempC;
	Out.FlightTimeS = FlightTimeS;
	Out.bPhysicsFault = bPhysicsFault;
	Out.bHaveTelemetry = bHaveTelemetry;
}

void ADroneDemoPawn::SetFlyMode(bool bOn)
{
	bFlyMode = bOn;
	if (bOn) ResetFlight();
	SetReplayMode(!bOn);   // fly: the physics steps and the pawn is visible; app: recorded runs
	if (bOn) { bSimulationActive = true; SetActorHiddenInGame(false); }
}

void ADroneDemoPawn::ResetFlight()
{
	PhysicsState = FDronePhysicsBridgeModule::CreateDefaultState();
	Thermal = FDronePhysicsBridgeModule::CreateDefaultThermalState();
	PhysicsFaultLatch = {};
	bHaveTelemetry = false;
	bPhysicsFault = false;
	bCrashed = false;
	bFlewSinceLanding = false;
	Accumulator = 0.0;
	BatteryUsedAh = 0.0;
	FlightTimeS = 0.0;
	if (bPhysicsReady) DemoController = MakeUnique<DroneDemo::Controller>(LastTune);
}

void ADroneDemoPawn::ApplyFlyPanel(const DroneApp::Fly::WindPanel& P)
{
	const double Key[6] = {P.EffectiveSteadyMps(), P.TowardDeg(),   // the panel says FROM; the physics wants TOWARD
		 P.GustSigmaMps, P.GustIntervalS, P.AirTempC, P.PayloadKg};
	bool bSame = true;
	for (int I = 0; I < 6; ++I) bSame = bSame && Key[I] == LastPanelKey[I];
	if (bSame) return;
	for (int I = 0; I < 6; ++I) LastPanelKey[I] = Key[I];
	FString Msg;
	// same validation and the same state change as the environment presets (DroneEnv::ValidateWeather inside SetWeather)
	if (!SetWeather(Key[0], Key[1], Key[2], SiteAltitudeM, Key[4], HumidityPct, Msg)) { EnvironmentNotice = Msg; return; }
	WindParams.GustCorrelationTimeS = Key[3];
	// Payload: added to the mass (and the inertia in proportion). The controller's hover feed-forward is NOT rescaled: its integrator
	// compensates. TODO(HUMAN-VERIFY): payload position / CG shift is not modelled in the live model.
	if (BaseMassKg > 0.0)
	{
		const double Scale = (BaseMassKg + Key[5]) / BaseMassKg;
		Config = BaseConfig;
		Config.mass_kg = BaseMassKg * Scale;
		for (int I = 0; I < 3; ++I) Config.inertia_kgm2[I] = BaseConfig.inertia_kgm2[I] * Scale;
	}
}

void ADroneDemoPawn::GetWindForHud(double OutTotalNed[3], double& OutGustMps) const
{
	for (int I = 0; I < 3; ++I) OutTotalNed[I] = WindSample.Total[I];
	OutGustMps = PrevGustMps;
}
