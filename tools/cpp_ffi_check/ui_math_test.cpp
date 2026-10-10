// Tier: V
// Tests for DroneUiMath.h. Plain C++17, no Unreal, no framework.
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/ui_math_test.cpp -o ui_math_test
// Expected values are hand calculations (shown) cross-checked with an independent python3 f64 run.

#include "DroneUiMath.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

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

using namespace DroneUi;

static void TestContrast()
{
    // White on black is the maximum, 21 (WCAG): (1 + 0.05) / (0 + 0.05).
    Check(Near(ContrastRatio(0xFFFFFF, 0x000000), 21.0, 1e-9), "white on black is 21");
    Check(Near(ContrastRatio(0x000000, 0xFFFFFF), 21.0, 1e-9), "contrast is symmetric");
    Check(Near(ContrastRatio(0x123456, 0x123456), 1.0, 1e-12), "same colour is 1");
    // sRGB transfer function: 0 -> 0, 255 -> 1, 128 -> 0.2158605 (IEC 61966-2-1).
    Check(SrgbToLinear(0) == 0.0 && Near(SrgbToLinear(255), 1.0, 1e-12) && Near(SrgbToLinear(128), 0.21586050011389926, 1e-12), "sRGB to linear");
    // Text on the darker panel colour (python3): off-white / 0x121722 = 15.25.
    Check(Near(ContrastRatio(kText, kPanelBottom), 15.25, 0.01), "text on panel bottom 15.25");
    // The requirement: every text colour on both panel colours is at least 4.5:1.
    const std::uint32_t Texts[] = {kText, kTextMuted, kAccent, kWarning, kCritical};
    const std::uint32_t Backs[] = {kPanelTop, kPanelBottom};
    bool All = true;
    for (const std::uint32_t T : Texts)
        for (const std::uint32_t B : Backs) All = All && ContrastRatio(T, B) >= kMinTextContrast;
    Check(All, "every text colour on both panel colours is >= 4.5:1");
    // The weakest pair (red on the lighter panel colour) is 5.53 (python3): the margin to 4.5 is real, not rounding.
    Check(Near(ContrastRatio(kCritical, kPanelBottom), 5.528, 0.01), "red on panel bottom 5.53");
}

static void TestScale()
{
    Check(UiScale(1080.0, 1.0) == 1.0 && UiScale(2160.0, 1.0) == 2.0 && UiScale(1080.0, 1.5) == 1.5, "ui scale 1080p, 4K, DPI 1.5");
    Check(UiScale(std::nan(""), 1.0) == 1.0 && UiScale(0.0, 1.0) == 1.0 && UiScale(1080.0, -1.0) == 1.0, "bad scale input gives 1");
    // Type scale: every size at least 14 px at 1080p, strictly increasing.
    bool Ok = kTypeScalePx[0] >= 14;
    for (int I = 1; I < 5; ++I) Ok = Ok && kTypeScalePx[I] > kTypeScalePx[I - 1];
    Check(Ok, "type scale starts at 14 px and increases");
    // 8 px grid: 11 -> 8, 12 -> 16 (halves up), 3.9 -> 0, 20 -> 24.
    Check(SnapToGrid(11.0) == 8.0 && SnapToGrid(12.0) == 16.0 && SnapToGrid(3.9) == 0.0 && SnapToGrid(20.0) == 24.0, "snap to the 8 px grid");
    Check(kCornerRadiusPx == 6.0 && kHairlinePx == 1.0 && kPanelOpacity == 0.85 && kFadeSeconds == 0.15, "brief constants");
}

static void TestSmoother()
{
    // d = 1, v = 0, w = 2, dt = 0.5: d' = (1 + (0 + 2) 0.5) e^-1 = 2 e^-1 = 0.7357588823428847; v' = (0 - 2 * 2 * 0.5) e^-1 = -2 e^-1.
    Smoother S;
    S.Snap(1.0);
    S.Step(0.0, 2.0, 0.5);
    Check(Near(S.X, 0.7357588823428847, 1e-12) && Near(S.V, -0.7357588823428847, 1e-12), "one exact step (hand)");
    // Semigroup: two half steps equal one full step (it is the exact solution), to rounding.
    Smoother A, B;
    A.Snap(3.0);
    B.Snap(3.0);
    A.Step(-1.0, 5.0, 0.2);
    B.Step(-1.0, 5.0, 0.1);
    B.Step(-1.0, 5.0, 0.1);
    Check(Near(A.X, B.X, 1e-12) && Near(A.V, B.V, 1e-12), "two half steps equal one step");
    // From rest it never overshoots the target and ends on it. w = 6, dt = 1/60, 3 s: |d| = (1 + 18) e^-18 = 2.9e-7.
    Smoother C;
    C.Snap(0.0);
    bool Monotone = true;
    double Prev = 0.0;
    for (int I = 0; I < 180; ++I)
    {
        C.Step(10.0, 6.0, 1.0 / 60.0);
        if (C.X < Prev - 1e-15 || C.X > 10.0 + 1e-12) Monotone = false;
        Prev = C.X;
    }
    Check(Monotone, "no overshoot, monotone");
    Check(Near(C.X, 10.0, 1e-5), "settles on the target after 3 s");
    // A huge step lands on the target (no blow-up); zero step changes nothing.
    Smoother D;
    D.Snap(5.0);
    D.Step(0.0, 6.0, 1.0e4);
    Check(Near(D.X, 0.0, 1e-12) && std::isfinite(D.V), "huge dt is stable");
    Smoother E;
    E.Snap(5.0);
    E.Step(0.0, 6.0, 0.0);
    Check(E.X == 5.0 && E.V == 0.0, "zero dt changes nothing");
    // Invalid input is ignored; the first Step snaps.
    Smoother F;
    F.Snap(2.0);
    F.Step(std::nan(""), 6.0, 0.1);
    F.Step(1.0, -1.0, 0.1);
    F.Step(1.0, 6.0, -0.1);
    Check(F.X == 2.0 && F.V == 0.0, "invalid input ignored");
    Smoother G;
    G.Step(7.0, 6.0, 0.1);
    Check(G.X == 7.0 && G.bSet, "first step snaps to the target");
}

static void TestRing()
{
    Ring<4> R;
    for (int I = 1; I <= 6; ++I) R.Push(I);
    Check(R.Size() == 4 && R.At(0) == 3.0 && R.At(1) == 4.0 && R.At(2) == 5.0 && R.At(3) == 6.0, "ring keeps the newest 4, oldest first");
    double Lo = 0.0, Hi = 0.0;
    Check(R.MinMax(&Lo, &Hi) && Lo == 3.0 && Hi == 6.0, "min and max");
    R.Push(std::nan(""));
    R.Push(HUGE_VAL);
    Check(R.Size() == 4 && R.At(3) == 6.0, "non-finite values are not stored");
    Check(R.At(4) == 0.0, "out of range index returns 0");
    R.Clear();
    Check(R.Size() == 0 && !R.MinMax(&Lo, &Hi) && R.At(0) == 0.0, "clear empties it");
    Ring<3> One;
    One.Push(5.0);
    Check(One.Size() == 1 && One.At(0) == 5.0, "partially filled ring");
}

static void TestTicks()
{
    Tick T[32];
    // Centre 12.3, half span 5, step 1, major every 5, 10 px per unit: values 8..17 (7.3 -> ceil 8, 17.3 -> floor 17): 10 ticks.
    int N = ComputeTicks(12.3, 5.0, 1.0, 5, 10.0, T, 32);
    Check(N == 10 && T[0].Value == 8.0 && T[9].Value == 17.0, "ticks 8..17");
    // Value 8 is below the reading: offset (12.3 - 8) * 10 = 43 px down (positive). Value 17 above: (12.3 - 17) * 10 = -47.
    Check(Near(T[0].OffsetPx, 43.0, 1e-9) && Near(T[9].OffsetPx, -47.0, 1e-9), "tick offsets (up is negative)");
    Check(!T[0].bMajor && T[2].bMajor && T[2].Value == 10.0 && T[7].bMajor && T[7].Value == 15.0, "major ticks at 10 and 15");
    // Negative values: centre -2, half 3: -5..1; majors at -5 and 0 (C++ -5 % 5 == 0).
    N = ComputeTicks(-2.0, 3.0, 1.0, 5, 1.0, T, 32);
    Check(N == 7 && T[0].Value == -5.0 && T[0].bMajor && T[5].Value == 0.0 && T[5].bMajor && !T[3].bMajor, "ticks across zero");
    // MaxOut limits the count, lowest value first.
    N = ComputeTicks(12.3, 5.0, 1.0, 5, 10.0, T, 3);
    Check(N == 3 && T[0].Value == 8.0 && T[2].Value == 10.0, "max out");
    Check(ComputeTicks(1.0, 0.0, 1.0, 5, 1.0, T, 8) == 0 && ComputeTicks(1.0, 5.0, 0.0, 5, 1.0, T, 8) == 0 && ComputeTicks(1.0, 5.0, 1.0, 0, 1.0, T, 8) == 0 &&
          ComputeTicks(std::nan(""), 5.0, 1.0, 5, 1.0, T, 8) == 0 && ComputeTicks(1.0, 5.0, 1.0, 5, 1.0, nullptr, 8) == 0 && ComputeTicks(1.0, 5.0, 1.0, 5, 1.0, T, 0) == 0,
          "invalid input gives no ticks");
}

static void TestHorizon()
{
    Point L{}, R{};
    // Level, pitch +0.1 rad at 100 px/rad: the horizon sits 10 px BELOW the centre (nose up), endpoints (-50, 10) and (50, 10).
    HorizonEnds(0.0, 0.1, 100.0, 50.0, &L, &R);
    Check(Near(L.X, -50.0, 1e-12) && Near(L.Y, 10.0, 1e-12) && Near(R.X, 50.0, 1e-12) && Near(R.Y, 10.0, 1e-12), "pitch up moves the horizon down");
    // Roll +90 deg (right wing down), no pitch, half width 100: the right end goes UP: right (0, -100), left (0, +100).
    HorizonEnds(kPi / 2.0, 0.0, 100.0, 100.0, &L, &R);
    Check(Near(L.X, 0.0, 1e-9) && Near(L.Y, 100.0, 1e-9) && Near(R.X, 0.0, 1e-9) && Near(R.Y, -100.0, 1e-9), "right roll lifts the right end");
    // Roll +90 with pitch 0.1 (shift 10): local (+-100, 10) -> left (10, 100), right (10, -100).
    HorizonEnds(kPi / 2.0, 0.1, 100.0, 100.0, &L, &R);
    Check(Near(L.X, 10.0, 1e-9) && Near(L.Y, 100.0, 1e-9) && Near(R.X, 10.0, 1e-9) && Near(R.Y, -100.0, 1e-9), "roll and pitch together");
    // Small right roll 0.1 rad, W 100: right end Y = -100 sin 0.1 = -9.9833.
    HorizonEnds(0.1, 0.0, 100.0, 100.0, &L, &R);
    Check(Near(R.Y, -100.0 * std::sin(0.1), 1e-9) && R.Y < 0.0 && L.Y > 0.0 && Near(R.X, 100.0 * std::cos(0.1), 1e-9), "small right roll");
    HorizonEnds(0.0, 0.0, 1.0, 1.0, nullptr, &R);  // must not crash
    Check(true, "null output tolerated");
}

static void TestHeading()
{
    Check(WrapDeg360(370.0) == 10.0 && WrapDeg360(-10.0) == 350.0 && WrapDeg360(360.0) == 0.0 && WrapDeg360(std::nan("")) == 0.0, "wrap 0..360");
    // Wind blowing toward north (0), nose east (90): the arrow points LEFT, -90 deg.
    Check(Near(WindArrowDeg(0.0, 90.0), -90.0, 1e-12), "north wind, nose east: -90");
    Check(Near(WindArrowDeg(90.0, 0.0), 90.0, 1e-12), "east wind, nose north: +90");
    Check(Near(WindArrowDeg(350.0, 10.0), -20.0, 1e-12), "wraps across north: -20");
    Check(Near(WindArrowDeg(190.0, 10.0), 180.0, 1e-12), "straight behind: +180 (range is (-180, 180])");
    Check(Near(WindArrowDeg(10.0, 10.0), 0.0, 1e-12), "wind toward the nose: 0");
}

static void TestStatus()
{
    Check(StatusFromMargin(1.5) == Status::Ok && StatusFromMargin(1.4999) == Status::Warning && StatusFromMargin(1.0) == Status::Warning, "margin thresholds 1.5 and 1.0");
    Check(StatusFromMargin(0.9999) == Status::Critical && StatusFromMargin(std::nan("")) == Status::Critical && StatusFromMargin(99.0) == Status::Ok, "critical below 1, NaN critical");
    // Utilisation is the inverse: 2/3 warns (margin 1.5), 1.0 is critical, 0.5 is fine.
    Check(StatusFromUtilisation(0.5) == Status::Ok && StatusFromUtilisation(2.0 / 3.0) == Status::Warning && StatusFromUtilisation(0.99) == Status::Warning, "utilisation warning band");
    Check(StatusFromUtilisation(1.0) == Status::Critical && StatusFromUtilisation(std::nan("")) == Status::Critical, "utilisation critical");
    Check(StatusRgb(Status::Ok) == kAccent && StatusRgb(Status::Warning) == kWarning && StatusRgb(Status::Critical) == kCritical, "colours follow status");
}

static void TestFormat()
{
    char B[32];
    FormatFixed(B, sizeof(B), 3.14159, 2, 7);
    Check(std::strcmp(B, "   3.14") == 0, "fixed width 7, 2 decimals");
    FormatFixed(B, sizeof(B), -12.5, 1, 8);
    Check(std::strcmp(B, "   -12.5") == 0, "negative right-aligned");
    FormatFixed(B, sizeof(B), std::nan(""), 2, 6);
    Check(std::strcmp(B, "    --") == 0, "NaN prints --");
    FormatFixed(B, sizeof(B), 1.0, 99, 3);
    Check(std::strcmp(B, "1.000000") == 0, "decimals capped at 6");
    Check(FormatFixed(nullptr, 4, 1.0, 1, 1) == 0 && FormatFixed(B, 0, 1.0, 1, 1) == 0, "null or empty buffer");
    char Small[4];
    FormatFixed(Small, sizeof(Small), 123456.0, 1, 1);
    Check(std::strlen(Small) == 3, "output is truncated to the buffer, still terminated");
    FormatClock(B, sizeof(B), 125.9);
    Check(std::strcmp(B, "02:05") == 0, "clock 125.9 s is 02:05");
    FormatClock(B, sizeof(B), 0.0);
    Check(std::strcmp(B, "00:00") == 0, "clock 0");
    FormatClock(B, sizeof(B), -1.0);
    Check(std::strcmp(B, "--:--") == 0, "negative time");
    FormatClock(B, sizeof(B), 1.0e9);
    Check(std::strcmp(B, "99:59") == 0, "clock capped");
    FormatClock(B, sizeof(B), std::nan(""));
    Check(std::strcmp(B, "--:--") == 0, "NaN time");
}

static void TestFadeAndGate()
{
    // Full fade in 0.15 s: 0 -> 1 with dt 0.075 gives 0.5, then 1.
    double A = FadeStep(0.0, 1.0, 0.075);
    Check(Near(A, 0.5, 1e-12), "half fade after 75 ms");
    A = FadeStep(A, 1.0, 0.075);
    Check(Near(A, 1.0, 1e-12), "full fade after 150 ms");
    Check(FadeStep(1.0, 1.0, 5.0) == 1.0 && FadeStep(0.0, 0.0, 5.0) == 0.0, "stays at the target");
    Check(Near(FadeStep(1.0, 0.0, 0.15), 0.0, 1e-12) && Near(FadeStep(1.0, 0.0, 0.03), 0.8, 1e-12), "fade out");
    Check(FadeStep(0.3, 1.0, std::nan("")) == 0.3 && FadeStep(0.3, 1.0, -1.0) == 0.3 && FadeStep(0.3, 1.0, 0.0) == 0.3, "invalid or zero dt unchanged");
    Check(FadeStep(0.9, 1.0, 10.0) == 1.0 && FadeStep(0.4, 0.5, 10.0) == 0.5 && FadeStep(0.6, 0.5, 10.0) == 0.5, "never past the target");
    Check(FadeStep(1.5, 1.0, 0.0) == 1.0 && FadeStep(-0.5, 0.0, 0.0) == 0.0, "result is always in [0, 1]");

    // 30 Hz gate: dt 1/60 -> every other call; 600 calls (10 s) -> 300 updates.
    DataGate G;
    int Updates = 0;
    for (int I = 0; I < 600; ++I) Updates += G.Due(1.0 / 60.0) ? 1 : 0;
    Check(Updates == 300, "300 updates in 10 s at 60 fps");
    // 144 fps -> about 30 updates per second: 1440 calls (10 s) -> 300 +- 1.
    DataGate H;
    int U144 = 0;
    for (int I = 0; I < 1440; ++I) U144 += H.Due(1.0 / 144.0) ? 1 : 0;
    Check(U144 >= 299 && U144 <= 301, "about 300 updates in 10 s at 144 fps");
    // 10 fps: every call is due.
    DataGate S;
    int U10 = 0;
    for (int I = 0; I < 10; ++I) U10 += S.Due(0.1) ? 1 : 0;
    Check(U10 == 10, "slow frames update every call");
    // Exactly one period is due: 1/30 s on the first call.
    DataGate Exact;
    Check(Exact.Due(1.0 / 30.0), "exactly one period is due");
    // A stall gives one update, then normal pacing (no burst).
    DataGate T;
    Check(T.Due(1.0) && !T.Due(0.0), "after a stall: one update, no burst");
    Check(!T.Due(std::nan("")) && !T.Due(-1.0), "invalid dt never fires");
}

int main()
{
    TestContrast();
    TestScale();
    TestSmoother();
    TestRing();
    TestTicks();
    TestHorizon();
    TestHeading();
    TestStatus();
    TestFormat();
    TestFadeAndGate();
    if (g_failures == 0)
    {
        std::printf("ui_math_test: all passed\n");
        return 0;
    }
    std::printf("ui_math_test: %d failure(s)\n", g_failures);
    return 1;
}
