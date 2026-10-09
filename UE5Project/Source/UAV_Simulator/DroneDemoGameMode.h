// Tier: V
// DEMO game mode: makes ADroneDemoPawn the default pawn in code, so no editor work is needed.
// Selected by DefaultEngine.ini: GlobalDefaultGameMode=/Script/UAV_Simulator.DroneDemoGameMode
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "DroneDemoGameMode.generated.h"

UCLASS()
class UAV_SIMULATOR_API ADroneDemoGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ADroneDemoGameMode();
};
