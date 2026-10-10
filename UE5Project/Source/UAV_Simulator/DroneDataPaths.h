// Tier: V
// DEMO: finds the project's data files (configs, environment table, models) without absolute paths.
// NOT COMPILED OR RUN by its author: TODO(HUMAN-VERIFY: builds on the project's Unreal version).
//
// Places, in order:
//   1. <ProjectDir>/DroneData/<relative path>           packaged build: UAV_Simulator.Build.cs stages the files here
//      (RuntimeDependencies, plain files, not cooked).
//   2. <ProjectContentDir>/DroneData/<relative path>    alternative staging through DefaultGame.ini (non-UFS Content folder).
//   3. <ProjectDir>/../<relative path>                  editor: the repo root, one level above UE5Project/.
// <relative path> is the path inside the repo, e.g. "offline_pipeline/geometry/quad_ref_build_config.json".
#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/Paths.h"

namespace DroneData
{
// Full path of the first existing candidate, or an empty string if neither exists.
inline FString Resolve(const FString& RelativeInRepo)
{
	const FString StagedInProject = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT("DroneData") / RelativeInRepo);
	if (FPlatformFileManager::Get().GetPlatformFile().FileExists(*StagedInProject)) return StagedInProject;
	const FString StagedInContent = FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir() / TEXT("DroneData") / RelativeInRepo);
	if (FPlatformFileManager::Get().GetPlatformFile().FileExists(*StagedInContent)) return StagedInContent;

	FString RepoRoot = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT(".."));
	FPaths::NormalizeDirectoryName(RepoRoot);
	const FString InRepo = RepoRoot / RelativeInRepo;
	if (FPlatformFileManager::Get().GetPlatformFile().FileExists(*InRepo)) return InRepo;
	return FString();
}
}  // namespace DroneData
