// Tier: V
// NOT COMPILED OR RUN by its author. See DroneBuildPreview.h.
#include "DroneBuildPreview.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
const TCHAR* const kCubePath = TEXT("/Engine/BasicShapes/Cube.Cube");
const TCHAR* const kCylinderPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
const TCHAR* const kMaterialPath = TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial");
// The engine's basic shapes are assumed to be 100 cm cubes / 100 cm diameter x 100 cm high cylinders, so scale = size_cm / 100.
// TODO(HUMAN-VERIFY: check the native sizes in the editor; if the cylinder is taller, change kCylinderNativeHeightCm).
constexpr float kNativeCm = 100.0f;
constexpr float kCylinderNativeHeightCm = 100.0f;

FLinearColor Srgb(uint32 Hex, float Alpha = 1.0f)
{
	FLinearColor C = FLinearColor::FromSRGBColor(FColor(static_cast<uint8>((Hex >> 16) & 0xFF), static_cast<uint8>((Hex >> 8) & 0xFF), static_cast<uint8>(Hex & 0xFF), 255));
	C.A = Alpha;
	return C;
}
}  // namespace

ADroneBuildPreview::ADroneBuildPreview()
{
	PrimaryActorTick.bCanEverTick = false;
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);
}

void ADroneBuildPreview::ClearPrimitives()
{
	for (UStaticMeshComponent* C : Parts)
		if (C) C->DestroyComponent();
	Parts.Reset();
	Materials.Reset();
	Roles.clear();
	Real.clear();
}

void ADroneBuildPreview::SetPrimitives(const std::vector<DroneApp::Primitive>& Prims)
{
	ClearPrimitives();
	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCubePath);
	UStaticMesh* Cylinder = LoadObject<UStaticMesh>(nullptr, kCylinderPath);
	UMaterialInterface* Mat = LoadObject<UMaterialInterface>(nullptr, kMaterialPath);
	if (!Cube || !Cylinder)
	{
		UE_LOG(LogTemp, Error, TEXT("DroneApp: engine basic shapes not found; the build preview is empty."));
		return;
	}
	float Max = 1.0f;
	for (const DroneApp::Primitive& P : Prims)
	{
		const DroneApp::UePlacement U = DroneApp::ToUnreal(P);   // position through DroneNedToUnreal.h; sizes are lengths
		UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this);
		C->SetStaticMesh(P.Kind == DroneApp::PrimitiveKind::Box ? Cube : Cylinder);
		C->SetupAttachment(Root);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->RegisterComponent();
		const bool bCyl = P.Kind != DroneApp::PrimitiveKind::Box;
		// box: X length, Y width, Z height. Cylinder: diameter along X and Y, height along Z.
		C->SetRelativeScale3D(FVector(U.SizeCm[0] / kNativeCm, U.SizeCm[1] / kNativeCm, U.SizeCm[2] / (bCyl ? kCylinderNativeHeightCm : kNativeCm)));
		C->SetRelativeLocationAndRotation(FVector(U.LocationCm[0], U.LocationCm[1], U.LocationCm[2]), FRotator(0.0f, static_cast<float>(U.YawDeg), 0.0f));
		UMaterialInstanceDynamic* M = Mat ? UMaterialInstanceDynamic::Create(Mat, this) : nullptr;
		if (M) C->SetMaterial(0, M);
		Parts.Add(C);
		Materials.Add(M);
		Roles.push_back(P.Role);
		Real.push_back(P.Real);
		Max = FMath::Max(Max, static_cast<float>(FMath::Sqrt(U.LocationCm[0] * U.LocationCm[0] + U.LocationCm[1] * U.LocationCm[1])) + 0.5f * FMath::Max3(static_cast<float>(U.SizeCm[0]), static_cast<float>(U.SizeCm[1]), static_cast<float>(U.SizeCm[2])));
	}
	RadiusCm = Max;
	ApplyBaseColours();
}

void ADroneBuildPreview::ApplyBaseColours()
{
	// Stand-ins: cool grey with the accent blended in; real models would be neutral white. One colour per role keeps the vehicle readable.
	for (int32 I = 0; I < Parts.Num(); ++I)
	{
		if (!Materials.IsValidIndex(I) || !Materials[I]) continue;
		const std::string& Role = Roles[static_cast<size_t>(I)];
		const uint32 Hex = Role == "frame" ? 0x6B7480 : (Role == "motor" ? 0x8C95A1 : (Role == "propeller" ? 0xA9C6E4 : (Role == "battery" ? 0x555D68 : 0x7A838F)));
		Materials[I]->SetVectorParameterValue(TEXT("Color"), Real[static_cast<size_t>(I)] ? Srgb(0xE8ECEF) : Srgb(Hex));   // TODO(HUMAN-VERIFY): BasicShapeMaterial parameter name "Color"
	}
}

void ADroneBuildPreview::SetRoleColour(const char* Role, const FLinearColor& Colour)
{
	for (int32 I = 0; I < Parts.Num(); ++I)
		if (Roles[static_cast<size_t>(I)] == Role && Materials.IsValidIndex(I) && Materials[I]) Materials[I]->SetVectorParameterValue(TEXT("Color"), Colour);
}

void ADroneBuildPreview::SetHeatTints(const FLinearColor& Motor, const FLinearColor& Esc, const FLinearColor& Battery)
{
	SetRoleColour("motor", Motor);
	SetRoleColour("esc", Esc);
	SetRoleColour("battery", Battery);
}

void ADroneBuildPreview::ResetTints() { ApplyBaseColours(); }
