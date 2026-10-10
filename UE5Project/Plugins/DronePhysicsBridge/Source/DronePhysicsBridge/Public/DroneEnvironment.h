// Tier: V
// DEMO ONLY: environment presets and typed-weather validation for the Unreal demo. UE-free (only the
// standard library), so tools/cpp_ffi_check/environment_test.cpp tests it with plain g++. The numbers
// in DroneEnvironments.json are ASSUMED; the limits below mirror the Rust core (interface v12) and a
// Rust test checks that they agree.
//
// What the core takes (EnvironmentInput): site altitude (height of the NED origin above sea level)
// and the temperature offset from the ISA standard day. Typed AMBIENT temperature is converted here:
//   dT = ambient_c - T_isa(site),  T_isa(h) = 15 - 0.0065 h  [C],  L = 0.0065 K/m (ISA).
// Density preview (same formula as physics_core aero::air_density, dry air):
//   rho = p / (R T),  p = p0 (T_isa_K / T0)^(g / (R L)),  T = T_isa_K + dT.
// Humidity is NOT modelled by the core. Nothing is silently clamped: an out-of-range input is rejected
// with a message the UI shows.
#pragma once

#include <cmath>
#include <string>

namespace DroneEnv
{
// Limits. kMaxSiteAltitudeM and kMaxTempOffsetC must equal physics_core types.rs (checked by a Rust test).
constexpr double kMaxSiteAltitudeM = 5000.0;
constexpr double kMaxTempOffsetC = 40.0;
// TODO(HUMAN-VERIFY: demo value): the typed-wind ceiling and the gust ceiling are choices, not limits of any model.
constexpr double kMaxWindSpeedMps = 15.0;
constexpr double kMaxGustSigmaMps = 5.0;
constexpr double kIsaSeaLevelTempC = 15.0;
constexpr double kIsaLapseKPerM = 0.0065;

constexpr double kStdTempK = 288.15;
constexpr double kStdPressurePa = 101325.0;
constexpr double kGasConstJPerKgK = 287.05;
constexpr double kGravityMps2 = 9.80665;
constexpr double kMaxTotalHeightM = 11000.0;  // ISA troposphere range of the core

struct Preset
{
    std::string Id;
    std::string Name;
    std::string LevelAssetPath;  // empty = not filled in yet (TODO(HUMAN-FILL))
    double SiteAltitudeM = 0.0;
    double AmbientTempC = 15.0;
    double HumidityPct = 50.0;
    double WindSpeedMps = 0.0;
    double WindTowardDeg = 0.0;
    double GustSigmaMps = 1.0;
};

enum class Status
{
    Ok,
    NonFinite,
    WindSpeedRange,
    GustRange,
    SiteAltitudeRange,
    TempOffsetRange,
    HumidityRange,
};

inline const char* CheckMessage(Status C)
{
    switch (C)
    {
        case Status::Ok: return "ok";
        case Status::NonFinite: return "a value is not a number";
        case Status::WindSpeedRange: return "wind speed must be 0 to 15 m/s";
        case Status::GustRange: return "gust level must be 0 to 5 m/s";
        case Status::SiteAltitudeRange: return "altitude must be 0 to 5000 m";
        case Status::TempOffsetRange: return "temperature is more than 40 K from the standard day at that altitude";
        case Status::HumidityRange: return "humidity must be 0 to 100 %";
    }
    return "unknown";
}

// ISA temperature at a height, degrees C. Valid for 0..11000 m (not range-checked here).
inline double IsaTemperatureC(double SiteAltitudeM)
{
    return kIsaSeaLevelTempC - kIsaLapseKPerM * SiteAltitudeM;
}

// Offset from the standard day for a typed ambient temperature at the site, K.
inline double TempOffsetC(double SiteAltitudeM, double AmbientTempC)
{
    return AmbientTempC - IsaTemperatureC(SiteAltitudeM);
}

// Air density, kg/m^3, at a total height above sea level with a given offset from the standard day (same formula
// as the Rust core's aero::air_density): rho = p / (R T), p from the ISA at that height, T = T_isa + offset.
// Returns false (Out untouched) for non-finite input, a height outside 0..11000 m, or T <= 0.
inline bool DensityAtHeightKgM3(double TotalHeightM, double OffsetC, double* Out)
{
    if (!Out || !std::isfinite(TotalHeightM) || !std::isfinite(OffsetC)) return false;
    if (TotalHeightM < 0.0 || TotalHeightM > kMaxTotalHeightM) return false;
    const double TempIsaK = kStdTempK - kIsaLapseKPerM * TotalHeightM;
    const double Exponent = kGravityMps2 / (kGasConstJPerKgK * kIsaLapseKPerM);
    const double PressurePa = kStdPressurePa * std::pow(TempIsaK / kStdTempK, Exponent);
    const double TempK = TempIsaK + OffsetC;
    if (TempK <= 0.0) return false;
    *Out = PressurePa / (kGasConstJPerKgK * TempK);
    return true;
}

// Air density at the NED origin height for a typed ambient temperature. Returns false (Out untouched) for
// non-finite input or a height outside 0..11000 m.
inline bool DensityKgM3(double SiteAltitudeM, double AmbientTempC, double* Out)
{
    if (!std::isfinite(SiteAltitudeM) || !std::isfinite(AmbientTempC)) return false;
    return DensityAtHeightKgM3(SiteAltitudeM, TempOffsetC(SiteAltitudeM, AmbientTempC), Out);
}

// Sea-level ISA density the core's hover throttle (config.hover_throttle) is derived at, kg/m^3.
constexpr double kHoverReferenceDensityKgM3 = 1.2250122659906946;

// Factor to apply to config.hover_throttle at another air density. Thrust T ~ throttle^2 * rho, so
// hover needs throttle ~ 1 / sqrt(rho): scale = sqrt(rho_ref / rho). Returns false (Out untouched)
// for a non-finite or non-positive density. The product can exceed 1: that means this air is too thin
// to hover at full throttle, and the caller must say so, not clamp silently.
inline bool HoverThrottleScale(double DensityKgM3, double* Out)
{
    if (!Out || !std::isfinite(DensityKgM3) || DensityKgM3 <= 0.0) return false;
    *Out = std::sqrt(kHoverReferenceDensityKgM3 / DensityKgM3);
    return true;
}

// Checks typed or preset weather. On Ok, OutEnvironment[0] = site altitude m, [1] = temperature
// offset K: exactly the two fields of EnvironmentInput. First failing check wins, in this order:
// non-finite, wind, gust, altitude, humidity, temperature offset.
inline Status ValidateWeather(double WindSpeedMps, double GustSigmaMps, double SiteAltitudeM, double AmbientTempC,
                             double HumidityPct, double OutEnvironment[2])
{
    if (!OutEnvironment) return Status::NonFinite;
    if (!std::isfinite(WindSpeedMps) || !std::isfinite(GustSigmaMps) || !std::isfinite(SiteAltitudeM) ||
        !std::isfinite(AmbientTempC) || !std::isfinite(HumidityPct))
    {
        return Status::NonFinite;
    }
    if (WindSpeedMps < 0.0 || WindSpeedMps > kMaxWindSpeedMps) return Status::WindSpeedRange;
    if (GustSigmaMps < 0.0 || GustSigmaMps > kMaxGustSigmaMps) return Status::GustRange;
    if (SiteAltitudeM < 0.0 || SiteAltitudeM > kMaxSiteAltitudeM) return Status::SiteAltitudeRange;
    if (HumidityPct < 0.0 || HumidityPct > 100.0) return Status::HumidityRange;
    const double Offset = TempOffsetC(SiteAltitudeM, AmbientTempC);
    if (std::fabs(Offset) > kMaxTempOffsetC) return Status::TempOffsetRange;
    OutEnvironment[0] = SiteAltitudeM;
    OutEnvironment[1] = Offset;
    return Status::Ok;
}

inline Status ValidatePreset(const Preset& P, double OutEnvironment[2])
{
    return ValidateWeather(P.WindSpeedMps, P.GustSigmaMps, P.SiteAltitudeM, P.AmbientTempC, P.HumidityPct, OutEnvironment);
}

// Wind direction from the UI, degrees, wrapped into [0, 360). Returns false for a non-finite value.
inline bool WrapDirectionDeg(double Deg, double* Out)
{
    if (!Out || !std::isfinite(Deg)) return false;
    double W = std::fmod(Deg, 360.0);
    if (W < 0.0) W += 360.0;
    *Out = W >= 360.0 ? 0.0 : W;
    return true;
}
}  // namespace DroneEnv
