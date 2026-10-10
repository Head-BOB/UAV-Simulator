// Tier: V
// DEMO. NOT COMPILED OR RUN by its author.
#include "DroneEnvironmentCatalog.h"

#include "Dom/JsonObject.h"
#include "DroneDataPaths.h"
#include "Misc/FileHelper.h"
#include "Misc/PackageName.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
const TCHAR* const kEnvironmentsRelative = TEXT("UE5Project/Plugins/DronePhysicsBridge/Content/DroneEnvironments.json");

bool ReadNumber(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, double& Out)
{
	return Obj->TryGetNumberField(Field, Out);
}

bool ReadVec3(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, double Out[3])
{
	const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
	if (!Obj->TryGetArrayField(Field, Arr) || !Arr || Arr->Num() != 3) return false;
	for (int32 I = 0; I < 3; ++I)
	{
		if (!(*Arr)[I].IsValid() || !(*Arr)[I]->TryGetNumber(Out[I])) return false;
	}
	return true;
}

bool ParseEntry(const TSharedPtr<FJsonObject>& Obj, FDroneEnvironmentEntry& Out)
{
	FString Id, Name, Level;
	double Start[3] = {0, 0, 0}, Tint[3] = {1, 1, 1};
	if (!Obj->TryGetStringField(TEXT("id"), Id) || !Obj->TryGetStringField(TEXT("name"), Name)) return false;
	Obj->TryGetStringField(TEXT("level_asset_path"), Level);  // optional: empty = not filled in
	DroneEnv::Preset& P = Out.Preset;
	if (!ReadNumber(Obj, TEXT("site_altitude_m"), P.SiteAltitudeM) || !ReadNumber(Obj, TEXT("ambient_temp_c"), P.AmbientTempC) ||
		!ReadNumber(Obj, TEXT("humidity_pct"), P.HumidityPct) || !ReadNumber(Obj, TEXT("wind_speed_mps"), P.WindSpeedMps) ||
		!ReadNumber(Obj, TEXT("wind_toward_deg"), P.WindTowardDeg) || !ReadNumber(Obj, TEXT("gust_sigma_mps"), P.GustSigmaMps) ||
		!ReadNumber(Obj, TEXT("player_yaw_deg"), Out.PlayerYawDeg) || !ReadNumber(Obj, TEXT("fog_density"), Out.FogDensity) ||
		!ReadNumber(Obj, TEXT("light_intensity_scale"), Out.LightScale) || !ReadVec3(Obj, TEXT("player_start_cm"), Start) ||
		!ReadVec3(Obj, TEXT("sky_tint_rgb"), Tint))
	{
		return false;
	}
	P.Id = TCHAR_TO_UTF8(*Id);
	P.Name = TCHAR_TO_UTF8(*Name);
	P.LevelAssetPath = TCHAR_TO_UTF8(*Level);
	Out.LevelAssetPath = Level;
	Out.PlayerStartCm = FVector(Start[0], Start[1], Start[2]);
	Out.SkyTint = FLinearColor(static_cast<float>(Tint[0]), static_cast<float>(Tint[1]), static_cast<float>(Tint[2]));
	double Env[2];
	const DroneEnv::Status S = DroneEnv::ValidatePreset(P, Env);
	Out.ValidationMessage = S == DroneEnv::Status::Ok ? FString() : FString(UTF8_TO_TCHAR(DroneEnv::CheckMessage(S)));
	return true;
}
}  // namespace

bool FDroneEnvironmentCatalog::Load(FString& OutError)
{
	EntriesArr.Reset();
	const FString Path = DroneData::Resolve(kEnvironmentsRelative);
	FString Text;
	if (Path.IsEmpty() || !FFileHelper::LoadFileToString(Text, *Path))
	{
		OutError = TEXT("DroneEnvironments.json not found (looked in Content/DroneData and the repo)");
		return false;
	}
	TSharedPtr<FJsonObject> Json;
	if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Json) || !Json.IsValid())
	{
		OutError = FString::Printf(TEXT("'%s' is not valid JSON"), *Path);
		return false;
	}
	const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
	if (!Json->TryGetArrayField(TEXT("environments"), Arr) || !Arr || Arr->Num() == 0)
	{
		OutError = TEXT("DroneEnvironments.json has no 'environments' array");
		return false;
	}
	for (const TSharedPtr<FJsonValue>& V : *Arr)
	{
		const TSharedPtr<FJsonObject>* Obj = nullptr;
		FDroneEnvironmentEntry E;
		if (!V.IsValid() || !V->TryGetObject(Obj) || !Obj || !ParseEntry(*Obj, E))
		{
			OutError = TEXT("an environment entry is missing a field (see DroneEnvironments.json)");
			EntriesArr.Reset();
			return false;
		}
		EntriesArr.Add(E);
	}
	Json->TryGetStringField(TEXT("default_environment"), DefaultId);
	return true;
}

const FDroneEnvironmentEntry* FDroneEnvironmentCatalog::Find(const FString& Id) const
{
	return EntriesArr.FindByPredicate([&Id](const FDroneEnvironmentEntry& E) { return Id.Equals(UTF8_TO_TCHAR(E.Preset.Id.c_str()), ESearchCase::IgnoreCase); });
}

const FDroneEnvironmentEntry* FDroneEnvironmentCatalog::Default() const
{
	if (const FDroneEnvironmentEntry* E = Find(DefaultId)) return E;
	return EntriesArr.Num() > 0 ? &EntriesArr[0] : nullptr;
}

FString FDroneEnvironmentCatalog::ResolveLevelPackage(const FDroneEnvironmentEntry& Entry, bool& bOutMissing)
{
	bOutMissing = true;
	if (Entry.LevelAssetPath.IsEmpty()) return FString();
	// The path is "/Game/.../L_Name" or "/Game/.../L_Name.L_Name"; DoesPackageExist wants the package part.
	FString Package = Entry.LevelAssetPath;
	int32 Dot = INDEX_NONE;
	if (Package.FindChar(TEXT('.'), Dot)) Package.LeftInline(Dot);
	if (!FPackageName::DoesPackageExist(Package)) return FString();
	bOutMissing = false;
	return Package;
}
