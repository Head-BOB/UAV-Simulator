// Tier: V
// DEMO: the finished report as it travels from the worker thread to the game thread. UE-free report types
// wrapped in a struct so the HUD header does not pull in the whole report engine. NOT COMPILED OR RUN.
#pragma once

#include "CoreMinimal.h"
#include "DroneReportOutput.h"

struct FDroneReportPayload
{
	DroneReport::Bundle Bundle;
	FString Folder;
	FString Error;   // non-empty if writing the files failed
};
