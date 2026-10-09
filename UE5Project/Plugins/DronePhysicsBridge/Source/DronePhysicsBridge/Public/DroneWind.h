// Tier: V
// DEMO ONLY wind generator: steady wind plus seeded gusts. Not flight code, not a validated
// turbulence model. UE-free (only <cmath>, <cstdint>), so tools/cpp_ffi_check/wind_test.cpp can
// test it with plain g++. Output is the NED wind (north, east, down) in m/s for WindInput.
//
// Model: steady vector + gust vector.
//   steady: speed_mps blowing TOWARD direction_deg (degrees clockwise from north):
//           N = speed cos(dir), E = speed sin(dir), D = 0.
//   gust:   independent first-order filtered noise (Ornstein-Uhlenbeck) per NED component,
//           exact discretisation  x[k+1] = a x[k] + s sqrt(1 - a^2) n,  a = exp(-dt / tau),
//           n ~ N(0,1). Stationary standard deviation is s; the lag-1 autocorrelation is a.
//           s = sigma for N and E, s = sigma * vertical_scale for D.
//   total:  steady + gust, magnitude clamped to max_total_mps by scaling (direction kept).
// Randomness: SplitMix64 (seed passed in) and Box-Muller. No wall-clock time anywhere. The same
// seed and the same call sequence give bit-identical output on the same build.
//
// Every number in DroneWindParams is TODO(HUMAN-VERIFY: demo value): a demo default chosen by
// the project owner, not a physics constant.
#pragma once

#include <cmath>
#include <cstdint>

namespace DroneWind
{
constexpr double kPi = 3.14159265358979323846;
// Rust rejects any |component| > 30 m/s (step code 12); a vector norm <= 30 can never exceed it.
constexpr double kRustComponentLimitMps = 30.0;

struct Params
{
    double SteadySpeedMps = 0.0;          // TODO(HUMAN-VERIFY: demo value)
    double SteadyDirectionDeg = 0.0;      // from north, direction the wind blows toward
    bool bGustsEnabled = true;            // TODO(HUMAN-VERIFY: demo value)
    double GustSigmaMps = 1.0;            // TODO(HUMAN-VERIFY: demo value)
    double GustCorrelationTimeS = 2.0;    // TODO(HUMAN-VERIFY: demo value)
    double GustVerticalScale = 0.3;       // TODO(HUMAN-VERIFY: demo value)
    double MaxTotalMps = 15.0;            // TODO(HUMAN-VERIFY: demo value)
};

struct Sample
{
    double Steady[3];  // NED m/s
    double Gust[3];    // NED m/s, total - steady after the clamp (zero if gusts are off)
    double Total[3];   // NED m/s, what goes to WindInput
};

inline bool IsFinite(double v)
{
    return std::isfinite(v);
}

// True if every parameter is usable: finite, sigma >= 0, tau > 0, scale >= 0, speed >= 0,
// 0 < max <= 30. A false result means "do not step the generator".
inline bool ParamsValid(const Params& P)
{
    return IsFinite(P.SteadySpeedMps) && P.SteadySpeedMps >= 0.0 && IsFinite(P.SteadyDirectionDeg) &&
           IsFinite(P.GustSigmaMps) && P.GustSigmaMps >= 0.0 && IsFinite(P.GustCorrelationTimeS) &&
           P.GustCorrelationTimeS > 0.0 && IsFinite(P.GustVerticalScale) && P.GustVerticalScale >= 0.0 &&
           IsFinite(P.MaxTotalMps) && P.MaxTotalMps > 0.0 && P.MaxTotalMps <= kRustComponentLimitMps;
}

// Steady wind vector, NED m/s. Direction is degrees from north, toward which the wind blows.
inline void SteadyVector(double SpeedMps, double DirectionDeg, double Out[3])
{
    const double Rad = DirectionDeg * kPi / 180.0;
    Out[0] = SpeedMps * std::cos(Rad);
    Out[1] = SpeedMps * std::sin(Rad);
    Out[2] = 0.0;
}

// Scales V so its magnitude is at most MaxMps. A vector already inside is returned unchanged.
inline void ClampMagnitude(double V[3], double MaxMps)
{
    const double Mag = std::sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
    if (Mag > MaxMps)
    {
        const double K = MaxMps / Mag;
        V[0] *= K;
        V[1] *= K;
        V[2] *= K;
    }
}

// Key helpers (demo UI): strength is held in [0, MaxMps], direction is wrapped into [0, 360).
inline double ClampSteadySpeed(double SpeedMps, double MaxMps)
{
    if (!IsFinite(SpeedMps) || SpeedMps < 0.0) return 0.0;
    return SpeedMps > MaxMps ? MaxMps : SpeedMps;
}

inline double WrapDirectionDeg(double Deg)
{
    if (!IsFinite(Deg)) return 0.0;
    double W = std::fmod(Deg, 360.0);
    if (W < 0.0) W += 360.0;
    return W >= 360.0 ? 0.0 : W;
}

// SplitMix64 (Steele, Lea, Flood 2014), public-domain reference algorithm.
class SplitMix64
{
public:
    explicit SplitMix64(std::uint64_t Seed) : State(Seed) {}

    std::uint64_t Next()
    {
        State += 0x9E3779B97F4A7C15ULL;
        std::uint64_t Z = State;
        Z = (Z ^ (Z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        Z = (Z ^ (Z >> 27)) * 0x94D049BB133111EBULL;
        return Z ^ (Z >> 31);
    }

    // Uniform in (0, 1]: 53 random bits, so the log in Box-Muller never sees 0.
    double NextUnitOpen()
    {
        return 1.0 - static_cast<double>(Next() >> 11) * (1.0 / 9007199254740992.0);
    }

    // Standard normal by Box-Muller; uses two uniforms per call (the second variate is dropped
    // on purpose so the number of draws per call is fixed and easy to reason about).
    double NextNormal()
    {
        const double U1 = NextUnitOpen();
        const double U2 = NextUnitOpen();
        return std::sqrt(-2.0 * std::log(U1)) * std::cos(2.0 * kPi * U2);
    }

private:
    std::uint64_t State;
};

class Generator
{
public:
    static constexpr std::uint64_t kDefaultSeed = 12345ULL;  // TODO(HUMAN-VERIFY: demo value)

    explicit Generator(std::uint64_t Seed = kDefaultSeed) : Rng(Seed) { Gust[0] = Gust[1] = Gust[2] = 0.0; }

    // Advances the gust state by DtS and fills Out. Returns false (Out untouched, state
    // untouched) if the parameters or DtS are invalid. The gust state advances even when gusts
    // are switched off, so toggling does not change the random sequence.
    bool Step(const Params& P, double DtS, Sample* Out)
    {
        if (!Out || !ParamsValid(P) || !IsFinite(DtS) || DtS <= 0.0) return false;

        const double A = std::exp(-DtS / P.GustCorrelationTimeS);
        const double Drive = std::sqrt(1.0 - A * A);
        const double Sigma[3] = {P.GustSigmaMps, P.GustSigmaMps, P.GustSigmaMps * P.GustVerticalScale};
        for (int K = 0; K < 3; ++K)
        {
            Gust[K] = A * Gust[K] + Sigma[K] * Drive * Rng.NextNormal();
        }

        SteadyVector(P.SteadySpeedMps, P.SteadyDirectionDeg, Out->Steady);
        for (int K = 0; K < 3; ++K)
        {
            Out->Total[K] = Out->Steady[K] + (P.bGustsEnabled ? Gust[K] : 0.0);
        }
        ClampMagnitude(Out->Total, P.MaxTotalMps);
        for (int K = 0; K < 3; ++K)
        {
            Out->Gust[K] = Out->Total[K] - Out->Steady[K];
        }
        return true;
    }

private:
    SplitMix64 Rng;
    double Gust[3];
};
}  // namespace DroneWind
