// Tier: V
#include "DroneDemoGameMode.h"
#include "DroneDemoPawn.h"

ADroneDemoGameMode::ADroneDemoGameMode()
{
	DefaultPawnClass = ADroneDemoPawn::StaticClass();
}
