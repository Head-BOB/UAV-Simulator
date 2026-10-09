// Tier: V
// Tests for DroneWind.h (demo wind generator). Plain C++17, no Unreal, no framework.
// From the repo root:
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/wind_test.cpp -o wind_test
//   ./wind_test
// Exit code 0 = all pass. Lives outside the plugin Source/ tree so UnrealBuildTool never compiles it.
// Statistical tests use fixed seeds, so their outcome is fixed; the tolerances come from theory
// (derivations in the comments), not from looking at the output.

#include "DroneWind.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

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

static bool SameBits(const double a[3], const double b[3])
{
    return std::memcmp(a, b, 3 * sizeof(double)) == 0;
}

using namespace DroneWind;

static void TestSplitMixReference()
{
    // Reference outputs of SplitMix64 for seed 0 (the published test vector) and seed 12345,
    // reproduced by an independent Python implementation of the same algorithm.
    SplitMix64 R0(0);
    Check(R0.Next() == 0xe220a8397b1dcdafULL, "splitmix seed 0 output 1");
    Check(R0.Next() == 0x6e789e6aa1b965f4ULL, "splitmix seed 0 output 2");
    Check(R0.Next() == 0x06c45d188009454fULL, "splitmix seed 0 output 3");
    SplitMix64 R1(12345);
    Check(R1.Next() == 0x22118258a9d111a0ULL, "splitmix seed 12345 output 1");
    // Uniform from 0x22118258a9d111a0: (x >> 11) * 2^-53 = 0.1330796686614273, so NextUnitOpen = 1 - that.
    SplitMix64 R2(12345);
    Check(Near(R2.NextUnitOpen(), 1.0 - 0.1330796686614273, 1e-15), "unit open value");
}

static void TestDefaults()
{
    const Params P;
    Check(P.SteadySpeedMps == 0.0 && P.SteadyDirectionDeg == 0.0, "default steady wind is 0");
    Check(P.GustSigmaMps == 1.0 && P.GustCorrelationTimeS == 2.0 && P.GustVerticalScale == 0.3, "demo gust defaults");
    Check(P.MaxTotalMps == 15.0 && P.bGustsEnabled, "demo clamp and gust default");
    Check(Generator::kDefaultSeed == 12345ULL, "default seed");
    Check(ParamsValid(P), "defaults are valid");
}

static void TestSteadyVector()
{
    // Direction is degrees from north toward which the wind blows: N = v cos, E = v sin.
    double V[3];
    SteadyVector(10.0, 0.0, V);
    Check(Near(V[0], 10.0, 1e-12) && Near(V[1], 0.0, 1e-12) && V[2] == 0.0, "dir 0 is north");
    SteadyVector(10.0, 90.0, V);
    Check(Near(V[0], 0.0, 1e-12) && Near(V[1], 10.0, 1e-12), "dir 90 is east");
    SteadyVector(10.0, 180.0, V);
    Check(Near(V[0], -10.0, 1e-12) && Near(V[1], 0.0, 1e-12), "dir 180 is south");
    SteadyVector(10.0, 270.0, V);
    Check(Near(V[0], 0.0, 1e-12) && Near(V[1], -10.0, 1e-12), "dir 270 is west");
    SteadyVector(2.0, 30.0, V);  // cos 30 = sqrt(3)/2, sin 30 = 1/2
    Check(Near(V[0], 1.7320508075688772, 1e-12) && Near(V[1], 1.0, 1e-12), "dir 30");
}

static void TestKeyHelpers()
{
    Check(ClampSteadySpeed(-1.0, 15.0) == 0.0, "speed floor");
    Check(ClampSteadySpeed(16.0, 15.0) == 15.0, "speed cap");
    Check(ClampSteadySpeed(7.0, 15.0) == 7.0, "speed inside");
    Check(ClampSteadySpeed(std::nan(""), 15.0) == 0.0, "speed NaN to 0");
    Check(WrapDirectionDeg(-15.0) == 345.0, "wrap -15");
    Check(WrapDirectionDeg(375.0) == 15.0, "wrap 375");
    Check(WrapDirectionDeg(360.0) == 0.0, "wrap 360");
    Check(WrapDirectionDeg(720.0) == 0.0, "wrap 720");
    Check(WrapDirectionDeg(0.0) == 0.0, "wrap 0");
    Check(WrapDirectionDeg(HUGE_VAL) == 0.0, "wrap Inf");
}

static void TestClampMagnitude()
{
    // 3-4-5: (30, 40, 0) has norm 50; clamping to 15 scales by 15/50 = 0.3 -> (9, 12, 0).
    double V[3] = {30.0, 40.0, 0.0};
    ClampMagnitude(V, 15.0);
    Check(Near(V[0], 9.0, 1e-12) && Near(V[1], 12.0, 1e-12) && V[2] == 0.0, "clamp 3-4-5");
    // Inside: unchanged, bit for bit.
    double W[3] = {3.0, 4.0, 0.0};
    const double Before[3] = {3.0, 4.0, 0.0};
    ClampMagnitude(W, 15.0);
    Check(SameBits(W, Before), "inside is untouched");
}

static void TestParamsValidation()
{
    Params P;
    Check(ParamsValid(P), "valid baseline");
    P = Params(); P.GustSigmaMps = std::nan(""); Check(!ParamsValid(P), "NaN sigma");
    P = Params(); P.GustSigmaMps = -0.1; Check(!ParamsValid(P), "negative sigma");
    P = Params(); P.GustCorrelationTimeS = 0.0; Check(!ParamsValid(P), "tau 0");
    P = Params(); P.GustVerticalScale = -1.0; Check(!ParamsValid(P), "negative scale");
    P = Params(); P.MaxTotalMps = 31.0; Check(!ParamsValid(P), "max above Rust limit");
    P = Params(); P.MaxTotalMps = 0.0; Check(!ParamsValid(P), "max 0");
    P = Params(); P.SteadySpeedMps = HUGE_VAL; Check(!ParamsValid(P), "Inf speed");
    P = Params(); P.SteadyDirectionDeg = std::nan(""); Check(!ParamsValid(P), "NaN direction");
    P = Params(); P.MaxTotalMps = 30.0; Check(ParamsValid(P), "max exactly 30 is allowed");

    // Invalid input: false, and neither the output nor the random sequence moves.
    Generator A(7), B(7);
    Sample Out;
    std::memset(&Out, 0, sizeof(Out));
    Out.Total[0] = 777.0;
    Params Bad;
    Bad.GustSigmaMps = std::nan("");
    Check(!A.Step(Bad, 0.002, &Out) && Out.Total[0] == 777.0, "bad params leave output");
    Check(!A.Step(Params(), 0.0, &Out), "dt 0 rejected");
    Check(!A.Step(Params(), std::nan(""), &Out), "dt NaN rejected");
    Check(!A.Step(Params(), -0.002, &Out), "negative dt rejected");
    Check(!A.Step(Params(), 0.002, nullptr), "null output rejected");
    Sample SA, SB;
    Check(A.Step(Params(), 0.002, &SA) && B.Step(Params(), 0.002, &SB) && SameBits(SA.Total, SB.Total),
          "rejected calls did not advance the state");
}

static void TestDeterminism()
{
    const Params P;
    Generator A(12345), B(12345), C(12346);
    bool Same = true, Differ = false;
    for (int I = 0; I < 5000; ++I)
    {
        Sample SA, SB, SC;
        A.Step(P, 0.002, &SA);
        B.Step(P, 0.002, &SB);
        C.Step(P, 0.002, &SC);
        Same = Same && SameBits(SA.Total, SB.Total) && SameBits(SA.Gust, SB.Gust);
        Differ = Differ || !SameBits(SA.Total, SC.Total);
    }
    Check(Same, "same seed gives bit-identical sequences");
    Check(Differ, "different seeds give different sequences");

    // The very first sample already differs between seeds (not just a later divergence).
    Generator D(12345), E(12346);
    Sample S1, S2;
    D.Step(P, 0.002, &S1);
    E.Step(P, 0.002, &S2);
    Check(!SameBits(S1.Total, S2.Total), "first sample differs between seeds");
}

static void TestZeroSigmaIsExactlySteady()
{
    Params P;
    P.GustSigmaMps = 0.0;
    P.SteadySpeedMps = 10.0;
    P.SteadyDirectionDeg = 37.0;
    double Steady[3];
    SteadyVector(10.0, 37.0, Steady);
    Generator G(12345);
    bool Exact = true;
    for (int I = 0; I < 2000; ++I)
    {
        Sample S;
        G.Step(P, 0.002, &S);
        Exact = Exact && S.Total[0] == Steady[0] && S.Total[1] == Steady[1] && S.Total[2] == Steady[2] &&
                S.Gust[0] == 0.0 && S.Gust[1] == 0.0 && S.Gust[2] == 0.0;
    }
    Check(Exact, "zero sigma gives exactly the steady wind");
}

static void TestGustToggleKeepsSequence()
{
    // Gusts off outputs the steady wind exactly; switching back on resumes the same sequence a
    // generator that never switched off would have produced.
    Params On, Off;
    Off.bGustsEnabled = false;
    On.SteadySpeedMps = Off.SteadySpeedMps = 5.0;
    Generator A(99), B(99);
    bool Ok = true;
    for (int I = 0; I < 600; ++I)
    {
        Sample SA, SB;
        A.Step(On, 0.002, &SA);
        const bool Phase = (I / 100) % 2 == 1;  // off during steps 100-199, 300-399, 500-599
        B.Step(Phase ? Off : On, 0.002, &SB);
        if (Phase)
        {
            Ok = Ok && SameBits(SB.Total, SB.Steady) && SB.Gust[0] == 0.0 && SB.Gust[1] == 0.0 && SB.Gust[2] == 0.0;
        }
        else
        {
            Ok = Ok && SameBits(SA.Total, SB.Total);
        }
    }
    Check(Ok, "gust toggle keeps the random sequence");
}

static void TestClampRespected()
{
    // Steady 15 m/s with sigma 5 m/s would exceed 15 about half the time without the clamp.
    Params P;
    P.SteadySpeedMps = 15.0;
    P.GustSigmaMps = 5.0;
    Generator G(12345);
    bool Inside = true;
    int AtLimit = 0;
    for (int I = 0; I < 100000; ++I)
    {
        Sample S;
        G.Step(P, 0.002, &S);
        const double Mag = std::sqrt(S.Total[0] * S.Total[0] + S.Total[1] * S.Total[1] + S.Total[2] * S.Total[2]);
        Inside = Inside && Mag <= 15.0 * (1.0 + 1e-12);
        if (Mag >= 15.0 * (1.0 - 1e-12)) ++AtLimit;
    }
    Check(Inside, "total magnitude never exceeds the clamp");
    Check(AtLimit > 1000, "the clamp was actually active");

    // A steady wind alone above the clamp is limited too (gusts off).
    Params Q;
    Q.SteadySpeedMps = 20.0;
    Q.bGustsEnabled = false;
    Generator H;
    Sample S;
    H.Step(Q, 0.002, &S);
    Check(Near(S.Total[0], 15.0, 1e-12), "steady above the clamp is limited");
}

static void TestStatistics()
{
    // OU theory: stationary mean 0, stationary std s (s = sigma for N/E, 0.3 sigma for D),
    // lag-1 autocorrelation exp(-dt/tau).
    // Mean test: dt = 0.01 s, tau = 2 s, N = 1e6 steps (T = 1e4 s). The std of the time mean is
    // s * sqrt(2 tau / T) = s * 0.02, so a bound of 5x that is 0.1 s (a 5-sigma bound).
    // Std test: the relative std of the sample std is about sqrt(2 * 2 tau / T) / 2 = 0.014;
    // bound = 5x = 0.07 s.
    Params P;
    P.GustSigmaMps = 1.0;
    Generator G(12345);
    const int N = 1000000;
    double Sum[3] = {0, 0, 0}, SumSq[3] = {0, 0, 0};
    for (int I = 0; I < N; ++I)
    {
        Sample S;
        G.Step(P, 0.01, &S);
        for (int K = 0; K < 3; ++K)
        {
            Sum[K] += S.Gust[K];
            SumSq[K] += S.Gust[K] * S.Gust[K];
        }
    }
    const double Scale[3] = {1.0, 1.0, 0.3};
    for (int K = 0; K < 3; ++K)
    {
        const double Mean = Sum[K] / N;
        const double Std = std::sqrt(SumSq[K] / N - Mean * Mean);
        Check(Near(Mean, 0.0, 0.1 * Scale[K]), "long-run mean is near zero");
        Check(Near(Std, Scale[K], 0.07 * Scale[K]), "long-run std matches sigma (vertical scaled by 0.3)");
    }

    // Autocorrelation: dt = 0.1, tau = 2 -> a = exp(-0.05) = 0.951229424500714 (hand value).
    // The std of the lag-1 estimate is about sqrt((1 - a^2) / N) = 0.0003 for N = 1e6; bound 0.002.
    Generator H(2024);
    double Prev = 0.0, Num = 0.0, Den = 0.0;
    for (int I = 0; I < N; ++I)
    {
        Sample S;
        H.Step(P, 0.1, &S);
        Num += Prev * S.Gust[0];
        Den += S.Gust[0] * S.Gust[0];
        Prev = S.Gust[0];
    }
    Check(Near(Num / Den, 0.951229424500714, 0.002), "lag-1 autocorrelation is exp(-dt/tau)");
}

int main()
{
    TestSplitMixReference();
    TestDefaults();
    TestSteadyVector();
    TestKeyHelpers();
    TestClampMagnitude();
    TestParamsValidation();
    TestDeterminism();
    TestZeroSigmaIsExactlySteady();
    TestGustToggleKeepsSequence();
    TestClampRespected();
    TestStatistics();
    if (g_failures == 0)
    {
        std::printf("wind_test: all passed\n");
        return 0;
    }
    std::printf("wind_test: %d failure(s)\n", g_failures);
    return 1;
}
