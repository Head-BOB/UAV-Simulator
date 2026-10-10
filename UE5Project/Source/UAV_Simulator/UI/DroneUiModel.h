// Tier: V
// DEMO: the data the report screen and the control panel exchange with the HUD. Plain structs. NOT COMPILED OR
// RUN by its author.
#pragma once

#include "CoreMinimal.h"
#include "DroneUiMath.h"

struct FDroneUiReportRow
{
	FString Component;
	FString Metric;
	FString PeakText;      // "7.3 A"
	FString LimitText;     // "23 A (CITED)"
	FString Cause;
	FString Consequence;
	TArray<FString> Suggestions;   // one line each, with TESTED / NOT TESTED
	double Margin = 0.0;
	bool bAssessed = true;
	DroneUi::Status Status = DroneUi::Status::Ok;
};

struct FDroneUiEnvRow
{
	FString Name;
	FString Verdict;
	FString Limiting;
	FString Detail;        // altitude, temperature, wind, density
	double MinMargin = 0.0;
	DroneUi::Status Status = DroneUi::Status::Ok;
};

struct FDroneUiReport
{
	bool bRunning = false;
	bool bReady = false;
	FString Environment;
	FString VerdictText;
	FString VerdictReason;
	FString Banner;        // the PRELIMINARY / ASSUMED statement, always shown
	FString FolderPath;    // where the files were written
	FString Message;       // errors or "exported"
	DroneUi::Status VerdictStatus = DroneUi::Status::Warning;
	TArray<FDroneUiReportRow> Rows;
	TArray<FDroneUiEnvRow> Envs;
};

// What the typed-weather fields hold (strings, because the user types them). Parsed and validated by
// DroneEnv::ValidateWeather in the pawn: nothing is clamped.
struct FDroneUiWeatherInput
{
	FString WindSpeed = TEXT("4.0");
	FString WindDirection = TEXT("270");
	FString Gust = TEXT("1.0");
	FString Altitude = TEXT("0");
	FString Temperature = TEXT("26");
	FString Humidity = TEXT("70");
};
