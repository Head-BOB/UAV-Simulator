// Tier: V
// Tests for DroneVisualMath.h. Plain C++17, no Unreal, no framework.
// From the repo root:
//   g++ -std=c++17 -Wall -Wextra -Werror -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public tools/cpp_ffi_check/visual_math_test.cpp -o visual_math_test
//   ./visual_math_test
// Exit code 0 = all pass. Lives outside the plugin Source/ tree so UnrealBuildTool never compiles it.

#include "DroneVisualMath.h"

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

using namespace DroneVisual;

static void TestSpin()
{
    // 6000 RPM = 100 rev/s. In 0.01 s that is exactly 1 revolution = 360 deg, which wraps to 0.
    Check(Near(AdvanceSpinDeg(0.0, 6000.0, 1.0, 0.01), 0.0, 1e-9), "6000 rpm 0.01 s is one revolution");
    // 3000 RPM = 50 rev/s; 0.01 s is half a revolution = 180 deg.
    Check(Near(AdvanceSpinDeg(0.0, 3000.0, 1.0, 0.01), 180.0, 1e-9), "3000 rpm half revolution");
    // Direction comes only from the sign: -1 gives -180, wrapped to +180.
    Check(Near(AdvanceSpinDeg(0.0, 3000.0, -1.0, 0.01), 180.0, 1e-9), "reverse half revolution wraps to 180");
    // 1000 RPM = 6000 deg/s; 1/60 s = 100 deg forward, and 260 deg when reversed (360 - 100).
    Check(Near(AdvanceSpinDeg(0.0, 1000.0, 1.0, 1.0 / 60.0), 100.0, 1e-9), "1000 rpm 1/60 s is 100 deg");
    Check(Near(AdvanceSpinDeg(0.0, 1000.0, -1.0, 1.0 / 60.0), 260.0, 1e-9), "reverse 100 deg is 260");
    // Starting angle is kept: 350 + 100 = 450 -> 90.
    Check(Near(AdvanceSpinDeg(350.0, 1000.0, 1.0, 1.0 / 60.0), 90.0, 1e-9), "wrap past 360");
    // Zero RPM: no motion (the motor is off).
    Check(AdvanceSpinDeg(123.0, 0.0, 1.0, 0.01) == 123.0, "zero rpm no motion");
    // Non-finite input leaves the angle unchanged.
    Check(AdvanceSpinDeg(10.0, std::nan(""), 1.0, 0.01) == 10.0, "NaN rpm ignored");
    Check(AdvanceSpinDeg(10.0, 1000.0, 1.0, HUGE_VAL) == 10.0, "Inf dt ignored");
    Check(std::isnan(AdvanceSpinDeg(std::nan(""), 1000.0, 1.0, 0.01)), "NaN prev is returned unchanged");
    // Result is always in [0, 360).
    const double A = AdvanceSpinDeg(359.999, 10000.0, 1.0, 0.016);
    Check(A >= 0.0 && A < 360.0, "result in range");
    // The axis constant is a unit vector (a typo here would silently shear the spin).
    const double N = std::sqrt(kSpinAxisBoneLocal[0] * kSpinAxisBoneLocal[0] + kSpinAxisBoneLocal[1] * kSpinAxisBoneLocal[1] +
                               kSpinAxisBoneLocal[2] * kSpinAxisBoneLocal[2]);
    Check(Near(N, 1.0, 1e-12), "spin axis is a unit vector");
    Check(kSpinDirectionScale == 1.0 || kSpinDirectionScale == -1.0, "direction scale is +-1");
}

static void TestScale()
{
    double S = -1.0;
    // 0.4 / 0.1708 = 2.34192037...
    Check(ComputeUniformScale(0.4, 0.1708, &S) && Near(S, 2.3419203747072599, 1e-12), "scale 0.4 / 0.1708");
    double Untouched = 5.0;
    Check(!ComputeUniformScale(0.4, 0.0, &Untouched) && Untouched == 5.0, "zero model span rejected");
    Check(!ComputeUniformScale(-0.4, 0.17, &Untouched), "negative config span rejected");
    Check(!ComputeUniformScale(std::nan(""), 0.17, &Untouched), "NaN rejected");
    Check(!ComputeUniformScale(0.4, 0.17, nullptr), "null out rejected");

    // Shipped config motor_pos_m: FR(+0.2,+0.2) FL(+0.2,-0.2) RR(-0.2,+0.2) RL(-0.2,-0.2).
    const double Pos[4][3] = {{0.2, 0.2, 0.0}, {0.2, -0.2, 0.0}, {-0.2, 0.2, 0.0}, {-0.2, -0.2, 0.0}};
    double LR = 0.0, FR = 0.0;
    ConfigSpansM(Pos, &LR, &FR);
    Check(Near(LR, 0.4, 1e-12) && Near(FR, 0.4, 1e-12), "config spans are 0.4 m");
    // Non-square layout: left-right 0.30, front-rear 0.50 (mean of the two pairs).
    const double Rect[4][3] = {{0.25, 0.15, 0.0}, {0.25, -0.15, 0.0}, {-0.25, 0.15, 0.0}, {-0.25, -0.15, 0.0}};
    ConfigSpansM(Rect, &LR, &FR);
    Check(Near(LR, 0.3, 1e-12) && Near(FR, 0.5, 1e-12), "rectangular config spans");

    // Leftover: model front-rear 0.1453 * 2.34192037 = 0.34028...; config 0.4 -> mismatch -0.05971...
    // 0.1453 * 2.3419203747072599 = 0.340281...: -0.059719 (hand multiplication).
    Check(Near(FrontRearMismatchM(2.3419203747072599, 0.1453, 0.4), -0.05971, 1e-4), "front-rear leftover");
    Check(Near(FrontRearMismatchM(2.0, 0.2, 0.4), 0.0, 1e-12), "no leftover when proportions match");
}

static void TestOpacity()
{
    Check(MotionDiscOpacity(0.0, 10000.0) == 0.0, "off is transparent");
    Check(Near(MotionDiscOpacity(5000.0, 10000.0), 0.225, 1e-12), "half rpm gives 0.45 * 0.5");
    Check(Near(MotionDiscOpacity(10000.0, 10000.0), 0.45, 1e-12), "full rpm gives the max");
    Check(Near(MotionDiscOpacity(20000.0, 10000.0), 0.45, 1e-12), "over max is clamped");
    Check(MotionDiscOpacity(-5.0, 10000.0) == 0.0, "negative rpm is 0");
    Check(MotionDiscOpacity(5000.0, 0.0) == 0.0, "zero max rpm is 0");
    Check(MotionDiscOpacity(std::nan(""), 10000.0) == 0.0, "NaN is 0");
    Check(MotionDiscOpacity(2500.0, 10000.0) < MotionDiscOpacity(7500.0, 10000.0), "rises with rpm");
}

int main()
{
    TestSpin();
    TestScale();
    TestOpacity();
    if (g_failures == 0)
    {
        std::printf("visual_math_test: all passed\n");
        return 0;
    }
    std::printf("visual_math_test: %d failure(s)\n", g_failures);
    return 1;
}
