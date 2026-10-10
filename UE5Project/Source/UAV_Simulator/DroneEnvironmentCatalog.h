// Tier: V
// DEMO: reads DroneEnvironments.json into DroneEnv::Preset entries (UE-free validation lives in
// DroneEnvironment.h). NOT COMPILED OR RUN by its author: TODO(HUMAN-VERIFY: builds on the project's Unreal version).
#pragma once

#include "CoreMinimal.h"
#include "DroneEnvironment.h"

struct FDroneEnvironmentEntry
{
	DroneEnv::Preset Preset;          // ids, climate and wind (UE-free)
	FString LevelAssetPath;           // "" until the human fills it (TODO(HUMAN-FILL))
	FVector PlayerStartCm = FVector::ZeroVector;
	double PlayerYawDeg = 0.0;
	FLinearColor SkyTint = FLinearColor::White;
	double FogDensity = 0.002;
	double LightScale = 1.0;
	FString ValidationMessage;        // empty = the preset passed DroneEnv::ValidatePreset
};

class FDroneEnvironmentCatalog
{
public:
	// Reads the table. Returns false and sets OutError if the file is missing or malformed. An entry that
	// fails validation is KEPT with ValidationMessage set (the menu shows it greyed out with the reason).
	bool Load(FString& OutError);

	const TArray<FDroneEnvironmentEntry>& Entries() const { return EntriesArr; }
	const FDroneEnvironmentEntry* Find(const FString& Id) const;
	const FDroneEnvironmentEntry* Default() const;

	// Package name of the level if it exists in this build, else an empty string and bOutMissing = true
	// (the caller opens the default level and shows "level not installed" on screen).
	static FString ResolveLevelPackage(const FDroneEnvironmentEntry& Entry, bool& bOutMissing);

private:
	TArray<FDroneEnvironmentEntry> EntriesArr;
	FString DefaultId;
};
