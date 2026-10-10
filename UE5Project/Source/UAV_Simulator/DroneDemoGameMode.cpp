// Tier: V
#include "DroneDemoGameMode.h"
#include "DroneDemoHUD.h"
#include "DroneDemoPawn.h"

ADroneDemoGameMode::ADroneDemoGameMode()
{
	DefaultPawnClass = ADroneDemoPawn::StaticClass();
	HUDClass = ADroneDemoHUD::StaticClass();  // the Slate UI (start menu, flight display, control panel, report)
}
