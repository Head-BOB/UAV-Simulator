// Tier: V
#include "DroneDemoGameMode.h"
#include "DroneAppHUD.h"
#include "DroneDemoPawn.h"

ADroneDemoGameMode::ADroneDemoGameMode()
{
	DefaultPawnClass = ADroneDemoPawn::StaticClass();
	HUDClass = ADroneAppHUD::StaticClass();  // the application UI (8 screens, P7); the demo HUD was removed
}
