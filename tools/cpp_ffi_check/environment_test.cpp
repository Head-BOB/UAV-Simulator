// Tier: V
// Tests for DroneEnvironment.h. Plain C++17, no Unreal, no framework.
// From the repo root:
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/environment_test.cpp -o environment_test
//   ./environment_test
// Exit code 0 = all pass. Expected values are hand-derived or from an independent python3 f64 evaluation.

#include "DroneEnvironment.h"

#include <cmath>
#include <cstdio>

static int g_failures = 0;

static void Check(bool ok, const char* name)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", name);
        ++g_failures;
    }
}

static bool Near(double a, double b, double tol)
{
    return std::fabs(a - b) <= tol;
}

using namespace DroneEnv;

static void TestTemperature()
{
    // T_isa(h) = 15 - 0.0065 h: 0 m -> 15 C; 3000 m -> 15 - 19.5 = -4.5 C; 11000 m -> 15 - 71.5 = -56.5 C.
    Check(Near(IsaTemperatureC(0.0), 15.0, 1e-12), "isa 0 m");
    Check(Near(IsaTemperatureC(3000.0), -4.5, 1e-12), "isa 3000 m");
    Check(Near(IsaTemperatureC(11000.0), -56.5, 1e-12), "isa 11000 m");
    // dT = ambient - T_isa: 5 C at 3000 m -> 5 - (-4.5) = +9.5; 26 C at 0 m -> +11; 15 C at 0 m -> 0.
    Check(Near(TempOffsetC(3000.0, 5.0), 9.5, 1e-12), "offset 3000 m 5 C");
    Check(Near(TempOffsetC(0.0, 26.0), 11.0, 1e-12), "offset beach");
    Check(TempOffsetC(0.0, 15.0) == 0.0, "standard day offset is exactly zero");
}

static void TestDensity()
{
    double Rho = 0.0;
    // Sea level 15 C: 101325 / (287.05 * 288.15) = 1.2250122659906946 (python3 f64). Tolerance 1e-12: pow rounding.
    Check(DensityKgM3(0.0, 15.0, &Rho) && Near(Rho, 1.2250122659906946, 1e-12), "sea level 15 C");
    // 3000 m, 5 C: T_isa = 268.65 K, dT = +9.5, p = 101325 (268.65/288.15)^5.255932362359814 = 70108.26833969312 Pa,
    // T = 278.15 K, rho = 70108.26833969312 / (287.05 * 278.15) = 0.8780770469291939.
    Check(DensityKgM3(3000.0, 5.0, &Rho) && Near(Rho, 0.8780770469291939, 1e-12), "3000 m 5 C");
    // Sea level +20 K (35 C) and -20 K (-5 C).
    Check(DensityKgM3(0.0, 35.0, &Rho) && Near(Rho, 1.1455047361519344, 1e-12), "sea level 35 C");
    Check(DensityKgM3(0.0, -5.0, &Rho) && Near(Rho, 1.3163799531800062, 1e-12), "sea level -5 C");
    // Hot air is thinner, high sites are thinner.
    double Hot = 0.0, Cold = 0.0, High = 0.0;
    Check(DensityKgM3(0.0, 35.0, &Hot) && DensityKgM3(0.0, -5.0, &Cold) && Hot < Cold, "hot thinner than cold");
    Check(DensityKgM3(2500.0, 15.0 - 0.0065 * 2500.0, &High) && High < 1.2250122659906946, "high site thinner");
    // Invalid input leaves the output untouched.
    double Keep = 7.0;
    Check(!DensityKgM3(-1.0, 15.0, &Keep) && Keep == 7.0, "negative height rejected");
    Check(!DensityKgM3(11000.5, 15.0, &Keep) && Keep == 7.0, "above 11000 m rejected");
    Check(!DensityKgM3(std::nan(""), 15.0, &Keep) && Keep == 7.0, "NaN height rejected");
    Check(!DensityKgM3(0.0, HUGE_VAL, &Keep) && Keep == 7.0, "Inf temperature rejected");
    Check(!DensityKgM3(0.0, 15.0, nullptr), "null out rejected");
    Check(!DensityKgM3(0.0, -273.15 - 288.15, &Keep) && Keep == 7.0, "non-positive absolute temperature rejected");
}

static void TestDensityAtHeight()
{
    double Rho = 0.0;
    // Same hand values as the typed-temperature form: total 3000 m, offset +9.5 K -> 0.8780770469291939.
    Check(DensityAtHeightKgM3(3000.0, 9.5, &Rho) && Near(Rho, 0.8780770469291939, 1e-12), "height 3000 m offset 9.5");
    // 3100 m (site 3000 + vehicle 100), +9.5 K -> 0.8689988332494722 (python3 f64).
    Check(DensityAtHeightKgM3(3100.0, 9.5, &Rho) && Near(Rho, 0.8689988332494722, 1e-12), "height 3100 m offset 9.5");
    // Sea level standard day.
    Check(DensityAtHeightKgM3(0.0, 0.0, &Rho) && Near(Rho, 1.2250122659906946, 1e-12), "sea level");
    double Keep = 4.0;
    Check(!DensityAtHeightKgM3(-0.5, 0.0, &Keep) && !DensityAtHeightKgM3(11000.5, 0.0, &Keep) && Keep == 4.0, "height range");
    Check(!DensityAtHeightKgM3(0.0, -300.0, &Keep) && Keep == 4.0, "non-positive temperature rejected");
    Check(!DensityAtHeightKgM3(std::nan(""), 0.0, &Keep) && !DensityAtHeightKgM3(0.0, HUGE_VAL, &Keep), "non-finite rejected");
}

static void TestHoverScale()
{
    double K = 0.0;
    // Reference density gives exactly 1. T ~ throttle^2 rho, so throttle ~ 1/sqrt(rho).
    Check(HoverThrottleScale(kHoverReferenceDensityKgM3, &K) && K == 1.0, "scale 1 at the reference density");
    // rho 0.8780770469291939 (3000 m, 5 C): sqrt(1.2250122659906946 / 0.8780770469291939) = 1.18114... (python3 f64).
    Check(HoverThrottleScale(0.8780770469291939, &K) && Near(K, 1.1811468777021716, 1e-12) && K > 1.0, "thin air needs more throttle");
    // rho 1.1455047361519344 (sea level +20 K): sqrt(1.2250122659906946 / 1.1455047361519344) = 1.03414...
    Check(HoverThrottleScale(1.1455047361519344, &K) && Near(K, 1.0341219919773335, 1e-12), "hot air needs a little more");
    // Four times the density halves the throttle: exact sqrt(1/4) = 0.5.
    Check(HoverThrottleScale(4.0 * kHoverReferenceDensityKgM3, &K) && Near(K, 0.5, 1e-15), "4x density halves the throttle");
    double Keep = 3.0;
    Check(!HoverThrottleScale(0.0, &Keep) && Keep == 3.0, "zero density rejected");
    Check(!HoverThrottleScale(-1.0, &Keep) && !HoverThrottleScale(std::nan(""), &Keep) && Keep == 3.0, "bad density rejected");
    Check(!HoverThrottleScale(1.0, nullptr), "null out rejected");
}

static void TestValidate()
{
    double E[2] = {-1.0, -1.0};
    // Good: 3000 m, 5 C -> E = {3000, 9.5}.
    Check(ValidateWeather(5.0, 1.0, 3000.0, 5.0, 40.0, E) == Status::Ok && E[0] == 3000.0 && Near(E[1], 9.5, 1e-12), "nominal weather");
    // Wind: 0 and 15 inclusive; 15.0001, -0.1, NaN rejected.
    Check(ValidateWeather(0.0, 0.0, 0.0, 15.0, 0.0, E) == Status::Ok, "wind 0");
    Check(ValidateWeather(15.0, 0.0, 0.0, 15.0, 0.0, E) == Status::Ok, "wind 15");
    Check(ValidateWeather(15.0001, 0.0, 0.0, 15.0, 0.0, E) == Status::WindSpeedRange, "wind 15.0001");
    Check(ValidateWeather(-0.1, 0.0, 0.0, 15.0, 0.0, E) == Status::WindSpeedRange, "wind negative");
    Check(ValidateWeather(std::nan(""), 0.0, 0.0, 15.0, 0.0, E) == Status::NonFinite, "wind NaN");
    // Gust 0..5.
    Check(ValidateWeather(1.0, 5.0, 0.0, 15.0, 0.0, E) == Status::Ok, "gust 5");
    Check(ValidateWeather(1.0, 5.1, 0.0, 15.0, 0.0, E) == Status::GustRange, "gust 5.1");
    Check(ValidateWeather(1.0, -1.0, 0.0, 15.0, 0.0, E) == Status::GustRange, "gust negative");
    // Altitude 0..5000.
    Check(ValidateWeather(1.0, 1.0, 5000.0, 15.0 - 0.0065 * 5000.0, 0.0, E) == Status::Ok, "altitude 5000");
    Check(ValidateWeather(1.0, 1.0, 5000.1, 0.0, 0.0, E) == Status::SiteAltitudeRange, "altitude 5000.1");
    Check(ValidateWeather(1.0, 1.0, -1.0, 15.0, 0.0, E) == Status::SiteAltitudeRange, "altitude -1");
    // Humidity 0..100.
    Check(ValidateWeather(1.0, 1.0, 0.0, 15.0, 100.0, E) == Status::Ok, "humidity 100");
    Check(ValidateWeather(1.0, 1.0, 0.0, 15.0, 100.5, E) == Status::HumidityRange, "humidity 100.5");
    // Temperature offset +-40 inclusive at sea level: 55 C -> +40, -25 C -> -40; just beyond rejected.
    Check(ValidateWeather(1.0, 1.0, 0.0, 55.0, 0.0, E) == Status::Ok && E[1] == 40.0, "offset +40");
    Check(ValidateWeather(1.0, 1.0, 0.0, -25.0, 0.0, E) == Status::Ok && E[1] == -40.0, "offset -40");
    Check(ValidateWeather(1.0, 1.0, 0.0, 55.001, 0.0, E) == Status::TempOffsetRange, "offset +40.001");
    Check(ValidateWeather(1.0, 1.0, 0.0, -25.001, 0.0, E) == Status::TempOffsetRange, "offset -40.001");
    // First failing check wins: bad wind and bad altitude report the wind.
    Check(ValidateWeather(99.0, 1.0, -5.0, 15.0, 0.0, E) == Status::WindSpeedRange, "order wind before altitude");
    // A rejected call leaves the output as it was.
    double Keep[2] = {11.0, 22.0};
    Check(ValidateWeather(99.0, 1.0, 0.0, 15.0, 0.0, Keep) != Status::Ok && Keep[0] == 11.0 && Keep[1] == 22.0, "output untouched on error");
    Check(ValidateWeather(1.0, 1.0, 0.0, 15.0, 0.0, nullptr) != Status::Ok, "null out rejected");
    // Every check has a message.
    Check(std::string(CheckMessage(Status::WindSpeedRange)).find("15") != std::string::npos, "wind message names the limit");
}

static void TestPresetAndDirection()
{
    Preset P;
    P.Id = "Mountains";
    P.SiteAltitudeM = 2500.0;
    P.AmbientTempC = 2.0;
    P.WindSpeedMps = 8.0;
    P.GustSigmaMps = 2.5;
    double E[2] = {0.0, 0.0};
    // T_isa(2500) = 15 - 16.25 = -1.25 C; dT = 2 - (-1.25) = +3.25.
    Check(ValidatePreset(P, E) == Status::Ok && E[0] == 2500.0 && Near(E[1], 3.25, 1e-12), "mountains preset");
    P.WindSpeedMps = 16.0;
    Check(ValidatePreset(P, E) == Status::WindSpeedRange, "preset with too much wind");

    double D = -1.0;
    Check(WrapDirectionDeg(370.0, &D) && Near(D, 10.0, 1e-12), "wrap 370");
    Check(WrapDirectionDeg(-10.0, &D) && Near(D, 350.0, 1e-12), "wrap -10");
    Check(WrapDirectionDeg(360.0, &D) && D == 0.0, "wrap 360");
    Check(WrapDirectionDeg(0.0, &D) && D == 0.0, "wrap 0");
    D = 5.0;
    Check(!WrapDirectionDeg(std::nan(""), &D) && D == 5.0, "wrap NaN rejected");
}

int main()
{
    TestTemperature();
    TestDensity();
    TestDensityAtHeight();
    TestHoverScale();
    TestValidate();
    TestPresetAndDirection();
    if (g_failures == 0)
    {
        std::printf("environment_test: all passed\n");
        return 0;
    }
    std::printf("environment_test: %d failure(s)\n", g_failures);
    return 1;
}
