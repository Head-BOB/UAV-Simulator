// Tier: V
// DEMO: plain data the Slate screens read. The HUD fills Raw at 30 Hz (DroneUi::DataGate) and Smooth
// (critically damped copies for needles and bars) every frame. No Unreal types except FString, so painting
// never allocates: every string below is rebuilt only in the 30 Hz update. NOT COMPILED OR RUN by its author.
#pragma once

#include "CoreMinimal.h"
#include "DroneUiMath.h"

struct FDroneUiRaw
{
	// Flight
	double AltitudeM = 0.0;
	double SpeedMps = 0.0;          // ground speed
	double AirspeedMps = 0.0;       // relative to the air
	double ClimbMps = 0.0;
	double HeadingDeg = 0.0;
	double RollRad = 0.0;
	double PitchRad = 0.0;
	double WindMps = 0.0;
	double WindTowardDeg = 0.0;
	double GustMps = 0.0;
	double ThrottleMean = 0.0;
	double RpmFraction[4] = {0, 0, 0, 0};   // rpm / max rpm
	double Rpm[4] = {0, 0, 0, 0};
	double ThrustN[4] = {0, 0, 0, 0};
	double MotorTempC[4] = {0, 0, 0, 0};    // core thermal model, UNVALIDATED placeholder
	double MaxThrustMarginUtil = 0.0;       // max thrust / full-throttle thrust at this density
	double MotorCurrentMaxA = 0.0;
	// Electrical (DroneElectrical.h, ASSUMED parameters)
	double BatteryA = 0.0;
	double PackV = 0.0;
	double BatteryUsedFraction = 0.0;       // used / usable
	// Environment
	double AirDensityKgM3 = 1.225;
	double SiteAltitudeM = 0.0;
	double AmbientTempC = 15.0;
	double FlightTimeS = 0.0;
	bool bPhysicsFault = false;
	bool bHaveTelemetry = false;
};

struct FDroneUiSmooth
{
	DroneUi::Smoother Altitude, Speed, Heading, Roll, Pitch, Climb, Battery;
	DroneUi::Smoother Rpm[4];
};

// Text rebuilt at 30 Hz.
struct FDroneUiText
{
	FString Mode;
	FString Environment;
	FString Clock;
	FString Altitude, Speed, Heading, Wind, Battery, Density, Temp, Notice;
	FString Rpm[4];
	FString Thrust[4];
};

// Mini charts (ring buffers, 30 Hz, about 20 s).
struct FDroneUiCharts
{
	DroneUi::Ring<600> Altitude;
	DroneUi::Ring<600> Speed;
	DroneUi::Ring<600> BatteryA;
	DroneUi::Ring<600> ThrottleMean;
};

struct FDroneUiSnapshot
{
	FDroneUiRaw Raw;
	FDroneUiSmooth Smooth;
	FDroneUiText Text;
	FDroneUiCharts Charts;
};
